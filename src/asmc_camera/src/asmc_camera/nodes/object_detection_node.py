"""Front-camera YOLO producer; models warm before subscriptions start."""
import copy
import threading
import time
from collections import deque
from dataclasses import dataclass

import numpy as np
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import CompressedImage

from asmc_msgs.msg import CameraObjectBox, CameraObjectBoxArray
from asmc_camera.camera.frame_buffer import EncodedFrame, LatestFrameBuffer
from asmc_camera.detection.classes import load_class_groups
from asmc_camera.detection.processor import ObjectFrameProcessor
from asmc_camera.detection.hood_mask import HoodMask
from asmc_camera.detection.ego_hood_filter import EgoHoodFilter
from asmc_camera.detection.ultralytics_backend import UltralyticsDetector


EXPECTED_MD5 = {CameraObjectBox: "34412c90553e06734f1acade77c8e1e3",
                CameraObjectBoxArray: "310c671afb3693e7909db6453d115b79"}


def check_message_build():
    for message, expected in EXPECTED_MD5.items():
        if message._md5sum != expected:
            raise RuntimeError("stale asmc_msgs build: %s MD5=%s expected=%s; rebuild and source outer workspace"
                               % (message.__name__, message._md5sum, expected))


@dataclass(frozen=True)
class ObjectFrame(EncodedFrame):
    header: object = None


from asmc_camera.runtime.ros_control import RosInferenceControl
from asmc_camera.runtime.state import ACTIVE


class ObjectDetectionNode:
    def __init__(self, backend=None):
        check_message_build()
        if list(rospy.get_param("~views", ["front"])) != ["front"]:
            raise ValueError("P2a supports views=[front] only")
        groups = load_class_groups(rospy.get_param("~class_groups"))
        hood_mask = HoodMask.load(bool(rospy.get_param("~hood_mask", False)),
                                  rospy.get_param("~hood_mask_config", ""))
        ego_hood_filter = EgoHoodFilter.load(bool(rospy.get_param("~ego_hood_filter", False)),
                                            rospy.get_param("~ego_hood_filter_config", ""))
        if hood_mask.enabled and ego_hood_filter.enabled:
            raise ValueError("hood input mask and ego hood post-filter are mutually exclusive")
        if backend is None:
            backend = UltralyticsDetector(
                weights=rospy.get_param("~weights"),
                device=rospy.get_param("~device", "cuda:0"),
                require_cuda=bool(rospy.get_param("~require_cuda", True)),
                conf_thres=float(rospy.get_param("~conf_thres", 0.7)),
                allowed_names=rospy.get_param("~allowed_names", ["bus", "truck", "person", "car"]),
                expected_sha256=rospy.get_param("~expected_sha256", ""),
                predict_overrides=rospy.get_param("~predict_overrides", {}),
                log=rospy.loginfo,
            )
        self.processor = ObjectFrameProcessor(backend, groups, hood_mask=hood_mask, ego_hood_filter=ego_hood_filter)
        self.backend_info = dict(backend.info)
        self.buffer = LatestFrameBuffer(["front"], batch_wait_ms=0)
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.output_frames = self.decode_errors = self.inference_errors = self.mask_errors = self.filter_errors = self.header_errors = 0
        self.latest_input_stamp_ns = self.latest_result_stamp_ns = 0
        self.last_input_at = 0.0
        self.started_at = time.monotonic()
        self.stage_samples = {}
        self.e2e_samples, self.queue_samples = deque(maxlen=500), deque(maxlen=500)
        self.duplicates = []
        self.publisher = rospy.Publisher("/perception/camera/objects", CameraObjectBoxArray, queue_size=5)
        self.raw_publisher = (rospy.Publisher("/perception/camera/objects/debug/raw", CameraObjectBoxArray, queue_size=5)
                              if bool(rospy.get_param("~debug_raw", False)) else None)
        self.filter_counts = dict(raw=0, kept=0, excluded=0)
        self.excluded_boxes = 0
        self.diagnostics = rospy.Publisher("/perception/camera/objects/diagnostics", DiagnosticArray, queue_size=5)
        self.last_header = None
        self.control = RosInferenceControl("yolo", ["front"], 20., on_transition=self._invalidate)
        self.subscriber = rospy.Subscriber(rospy.get_param("~image_topic", "/image_jpeg/compressed"),
                                           CompressedImage, self._callback, queue_size=1, buff_size=2**24)
        rospy.on_shutdown(self.shutdown)
        self.worker = threading.Thread(target=self._worker, daemon=True)
        self.monitor = threading.Thread(target=self._monitor, daemon=True)
        self.worker.start()
        self.monitor.start()
        rospy.loginfo("camera object detector ready: model=%s device=%s", backend.info.get("weights_sha256"),
                      backend.info.get("device"))

    def _invalidate(self, mode):
        if mode != ACTIVE and self.last_header is not None:
            message = CameraObjectBoxArray(header=copy.deepcopy(self.last_header), valid=False)
            self.publisher.publish(message)
            if self.raw_publisher is not None:
                self.raw_publisher.publish(message)

    def _callback(self, message):
        with self.lock:
            self.last_input_at = time.monotonic()
            self.latest_input_stamp_ns = message.header.stamp.to_nsec()
            if message.header.stamp.is_zero() or message.header.frame_id != "camera_front":
                self.header_errors += 1
                rospy.logwarn_throttle(5, "camera objects requires nonzero source stamp and camera_front frame_id")
                return
        self.control.state.observe("front", message.header.stamp.to_nsec())
        self.buffer.push(ObjectFrame(view="front", stamp=message.header.stamp,
                                     frame_id=message.header.frame_id, data=bytes(message.data),
                                     received_monotonic=time.monotonic(),
                                     received_ros_ns=rospy.Time.now().to_nsec(), header=copy.deepcopy(message.header)))

    def _worker(self):
        while not self.stop.is_set() and not rospy.is_shutdown():
            for frame in self.buffer.pop_batch(timeout=0.2):
                stamps = {"front": frame.stamp.to_nsec()}
                token = self.control.state.begin(stamps)
                if token is None:
                    continue
                if self.raw_publisher is not None:
                    message, raw, timings, error = self.processor.process_with_raw(frame.header, frame.data)
                else:
                    message, timings, error = self.processor.process(frame.header, frame.data)
                    raw = None
                if self.stop.is_set():
                    return
                with self.control.state.output_guard(token) as allowed:
                    if not allowed:
                        continue
                    self.control.state.complete(token, stamps, message.valid, error[0] if error else "")
                    if self.control.state.mode == ACTIVE:
                        self.publisher.publish(message)
                        if self.raw_publisher is not None:
                            self.raw_publisher.publish(raw)
                        self.last_header = copy.deepcopy(frame.header)
                with self.lock:
                    self.filter_counts = dict(self.processor.last_counts)
                    self.excluded_boxes += self.filter_counts["excluded"]
                    self.output_frames += 1
                    self.latest_result_stamp_ns = frame.stamp.to_nsec()
                    self.queue_samples.append((time.monotonic() - frame.received_monotonic) * 1000)
                    elapsed_ns = rospy.Time.now().to_nsec() - frame.stamp.to_nsec()
                    if elapsed_ns >= 0:
                        self.e2e_samples.append(elapsed_ns / 1e6)
                    for key, value in timings.items():
                        self.stage_samples.setdefault(key, deque(maxlen=500)).append(value)
                    if error:
                        if error[0] == "decode":
                            self.decode_errors += 1
                        elif error[0] == "filter":
                            self.filter_errors += 1
                        elif error[0] == "mask":
                            self.mask_errors += 1
                        else:
                            self.inference_errors += 1
                if error:
                    rospy.logwarn_throttle(2, "camera objects %s failure: %s", *error)

    def _monitor(self):
        previous_count, previous_at, graph_at = 0, self.started_at, 0
        while not self.stop.wait(1.0) and not rospy.is_shutdown():
            now = time.monotonic()
            if now - graph_at >= 5:
                graph_at = now
                try:
                    _, _, state = rospy.get_master().getSystemState()
                    nodes = {node for group in state for _, names in group for node in names}
                    duplicates = sorted(nodes & {"/yolo_detection_node", "/camera_bbox_detector"})
                    for topic, names in state[0]:
                        if topic in ("/perception/camera/objects", "/YOLO/BBox") and len(names) > 1:
                            duplicates.append("multiple publishers: " + topic)
                    self.duplicates = duplicates
                    if duplicates:
                        rospy.logwarn_throttle(5, "duplicate camera inference: %s", ", ".join(duplicates))
                except Exception as exc:
                    rospy.logwarn_throttle(5, "camera object graph check failed: %s", exc)
            with self.lock:
                values = dict(self.backend_info)
                values.update({"hood_mask_"+key: value for key,value in self.processor.hood_mask.info.items()})
                values.update({"ego_hood_filter_"+key: value for key,value in self.processor.ego_hood_filter.info.items()})
                values.update(debug_raw_enabled=self.raw_publisher is not None,
                              ego_hood_filter_last_counts=self.filter_counts,
                              ego_hood_filter_excluded_total=self.excluded_boxes, filter_errors=self.filter_errors)
                fps = (self.output_frames - previous_count) / max(now - previous_at, 1e-6)
                previous_count, previous_at = self.output_frames, now
                values.update(processing_fps=round(fps, 3), output_frames=self.output_frames,
                              received_frames=self.buffer.received["front"], replaced_frames=self.buffer.replaced["front"],
                              decode_errors=self.decode_errors, inference_errors=self.inference_errors,
                              mask_errors=self.mask_errors,
                              header_errors=self.header_errors, duplicate_nodes=self.duplicates,
                              latest_input_stamp_ns=self.latest_input_stamp_ns,
                              latest_result_stamp_ns=self.latest_result_stamp_ns,
                              input_silence_sec=round(now - (self.last_input_at or self.started_at), 3))
                for prefix, samples in [("end_to_end", self.e2e_samples), ("queue_to_output", self.queue_samples)]:
                    values[prefix + "_p50_ms"] = float(np.percentile(samples, 50)) if samples else None
                    values[prefix + "_p95_ms"] = float(np.percentile(samples, 95)) if samples else None
                values["average_stage_ms"] = {k: round(float(np.mean(v)), 3) for k, v in self.stage_samples.items()}
            status = DiagnosticStatus(name="camera_objects/inference", hardware_id=str(values.get("device", "unknown")))
            status.level = DiagnosticStatus.WARN if (self.duplicates or values["input_silence_sec"] > 1
                                                     or self.decode_errors or self.inference_errors or self.mask_errors or self.filter_errors or self.header_errors) else DiagnosticStatus.OK
            status.message = "input/processing warning" if status.level else "running"
            status.values = [KeyValue(key=str(k), value=str(v)) for k, v in values.items()]
            self.diagnostics.publish(DiagnosticArray(header=copy.deepcopy(self._diagnostic_header()), status=[status]))

    @staticmethod
    def _diagnostic_header():
        from std_msgs.msg import Header
        return Header(stamp=rospy.Time.now())

    def shutdown(self):
        self.control.close()
        self.stop.set()
        self.subscriber.unregister()
        self.buffer.close()
        for thread in (self.worker, self.monitor):
            if thread is not threading.current_thread():
                thread.join(timeout=2)


def main():
    rospy.init_node("camera_object_detector")
    try:
        ObjectDetectionNode()
    except Exception as exc:
        rospy.logfatal("camera object detector startup failed: %s", exc)
        raise SystemExit(1)
    rospy.spin()
