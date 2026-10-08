"""Front-camera YOLO producer; models warm before subscriptions start."""
import copy
import threading
import time
from collections import deque
from dataclasses import dataclass
from functools import partial

import numpy as np
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import CompressedImage

from asmc_msgs.msg import CameraObjectBox, CameraObjectBoxArray
from asmc_camera.camera.frame_buffer import EncodedFrame, LatestFrameBuffer
from asmc_camera.detection.classes import load_class_groups
from asmc_camera.detection.processor import ObjectFrameProcessor
from asmc_camera.detection.hood_mask import HoodMask, load_view_masks
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
        self.views = list(rospy.get_param("~views", ["front"]))
        if not self.views or len(set(self.views)) != len(self.views) or set(self.views)-{'front','left','right'}:
            raise ValueError("views must contain unique front/left/right names")
        production_batch = bool(rospy.get_param("~batched_runtime", False))
        experimental = bool(rospy.get_param('~seg_experiment', False))
        if self.views != ['front'] and not (experimental or production_batch):
            raise ValueError('multiple object views require seg_experiment=true')
        self.debug_masks = bool(rospy.get_param('~debug_masks', False))
        groups = load_class_groups(rospy.get_param("~class_groups"))
        hood_mask = HoodMask.load(bool(rospy.get_param("~hood_mask", False)),
                                  rospy.get_param("~hood_mask_config", ""))
        view_masks = load_view_masks(self.views, hood_mask,
                                    bool(rospy.get_param('~side_ego_mask', False)),
                                    rospy.get_param('~side_ego_mask_configs', {}))
        ego_hood_filter = EgoHoodFilter.load(bool(rospy.get_param("~ego_hood_filter", False)),
                                            rospy.get_param("~ego_hood_filter_config", ""))
        if hood_mask.enabled and ego_hood_filter.enabled:
            raise ValueError("hood input mask and ego hood post-filter are mutually exclusive")
        if backend is None:
            batched = production_batch or bool(rospy.get_param('~batched_experiment', False))
            if batched and not (experimental or production_batch):
                raise ValueError('batched runtime requires seg_experiment=true')
            if batched:
                if production_batch:
                    from asmc_camera.detection.yolo_batch_backend import BatchedYoloDetector
                    factory = BatchedYoloDetector
                else:
                    from asmc_camera.detection.batched_backend import BatchedSegmentationDetector
                    factory = BatchedSegmentationDetector
            elif experimental:
                from asmc_camera.detection.segmentation import SegmentationDetector
                factory = SegmentationDetector
            else:
                factory = UltralyticsDetector
            extra = dict(task=rospy.get_param('~task', 'detect'),
                         front_width=int(rospy.get_param('~front_width', 1280)),
                         batch_mode=rospy.get_param('~batch_mode', '2plus1'),
                         runtime_backend=rospy.get_param('~runtime_backend', 'tensorrt'),
                         source_checkpoint_sha256=rospy.get_param('~source_checkpoint_sha256',''),
                         expected_prompt_sha256=rospy.get_param('~expected_prompt_sha256','')) if batched else {}
            backend = factory(
                weights=rospy.get_param("~weights"),
                device=rospy.get_param("~device", "cuda:0"),
                require_cuda=bool(rospy.get_param("~require_cuda", True)),
                conf_thres=float(rospy.get_param("~conf_thres", 0.7)),
                allowed_names=rospy.get_param("~allowed_names", ["bus", "truck", "person", "car"]),
                expected_sha256=rospy.get_param("~expected_sha256", ""),
                predict_overrides=rospy.get_param("~predict_overrides", {}),
                log=rospy.loginfo,
                model_kind=rospy.get_param('~model_kind', 'yolo'),
                prompt_embeddings=rospy.get_param('~prompt_embeddings', ''),
                **extra
            )
        self.batch_backend = backend if getattr(backend, 'batched', False) is True else None
        self.processors = {v: ObjectFrameProcessor(backend, groups,
            hood_mask=view_masks[v],
            ego_hood_filter=ego_hood_filter if v == 'front' else EgoHoodFilter(),
            include_masks=self.debug_masks,
            label_mode=rospy.get_param("~label_mode", "group")) for v in self.views}
        self.processor = self.processors.get('front', self.processors[self.views[0]])
        self.backend_info = dict(backend.info)
        batch_wait = float(rospy.get_param('~batch_wait_ms', 10 if self.batch_backend else 0))
        if not 0 <= batch_wait <= 10:
            raise ValueError('object batch wait must be in [0,10] ms')
        self.buffer = LatestFrameBuffer(self.views, batch_wait_ms=batch_wait)
        self.view_counts = {v: 0 for v in self.views}
        self.rotation = 0
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
        self.last_headers = {}
        self.mask_publishers = {v: rospy.Publisher(
            '/perception/camera/objects/debug/'+('instances' if v == 'front' else v+'/instances')+'/compressed',
            CompressedImage, queue_size=1) for v in self.views} if self.debug_masks else {}
        self.control = RosInferenceControl("yolo", self.views, 20., on_transition=self._invalidate)
        self.control.state.record('startup', model_load_ms=self.backend_info.get('model_load_ms'),
                                  dummy_warmup_ms=self.backend_info.get('warmup_ms'))
        topics = dict(front=rospy.get_param('~image_topic','/image_jpeg/compressed'),
                      left='/cam_left/image_jpeg/compressed', right='/cam_right/image_jpeg/compressed')
        topics.update(rospy.get_param('~image_topics', {}))
        self.subscribers = [rospy.Subscriber(topics[v], CompressedImage, partial(self._callback, view=v),
                            queue_size=1, buff_size=2**24) for v in self.views]
        self.subscriber = self.subscribers[0]
        rospy.on_shutdown(self.shutdown)
        self.worker = threading.Thread(target=self._worker, daemon=True)
        self.monitor = threading.Thread(target=self._monitor, daemon=True)
        self.worker.start()
        self.monitor.start()
        rospy.loginfo("camera object detector ready: model=%s device=%s", backend.info.get("weights_sha256"),
                      backend.info.get("device"))

    def _invalidate(self, mode):
        if mode != ACTIVE:
            for header in self.last_headers.values():
                message = CameraObjectBoxArray(header=copy.deepcopy(header), valid=False)
                self.publisher.publish(message)
                if self.raw_publisher is not None:
                    self.raw_publisher.publish(message)

    def _callback(self, message, view='front'):
        with self.lock:
            self.last_input_at = time.monotonic()
            self.latest_input_stamp_ns = message.header.stamp.to_nsec()
            if message.header.stamp.is_zero() or message.header.frame_id != 'camera_'+view:
                self.header_errors += 1
                rospy.logwarn_throttle(5, "camera objects requires nonzero source stamp and matching camera view frame_id")
                return
        self.control.state.observe(view, message.header.stamp.to_nsec())
        self.buffer.push(ObjectFrame(view=view, stamp=message.header.stamp,
                                     frame_id=message.header.frame_id, data=bytes(message.data),
                                     received_monotonic=time.monotonic(),
                                     received_ros_ns=rospy.Time.now().to_nsec(), header=copy.deepcopy(message.header)))

    def _worker(self):
        while not self.stop.is_set() and not rospy.is_shutdown():
            frames = self.buffer.pop_batch(timeout=0.2)
            order = self.views[self.rotation:]+self.views[:self.rotation]
            frames.sort(key=lambda f: order.index(f.view))
            self.rotation = (self.rotation+1) % len(self.views)
            if self.batch_backend is not None:
                from asmc_camera.detection.batch_processor import process_batch
                admitted, tokens = [], {}
                for frame in frames:
                    token = self.control.state.begin({frame.view:frame.stamp.to_nsec()}, per_view=True)
                    if token is not None:
                        admitted.append(frame)
                        tokens[frame.view] = token
                for frame, message, raw, timings, error in process_batch(
                        admitted, self.processors, self.batch_backend, self.raw_publisher is not None):
                    if self.stop.is_set():
                        return
                    self._consume(frame, tokens[frame.view], message, raw, timings, error)
                continue
            for frame in frames:
                processor = self.processors[frame.view]
                stamps = {frame.view: frame.stamp.to_nsec()}
                token = self.control.state.begin(stamps, per_view=len(self.views)>1)
                if token is None:
                    continue
                if self.raw_publisher is not None:
                    message, raw, timings, error = processor.process_with_raw(frame.header, frame.data)
                else:
                    message, timings, error = processor.process(frame.header, frame.data)
                    raw = None
                if self.stop.is_set():
                    return
                self._consume(frame, token, message, raw, timings, error)

    def _consume(self, frame, token, message, raw, timings, error):
        processor = self.processors[frame.view]
        stamps = {frame.view:frame.stamp.to_nsec()}
        with self.control.state.output_guard(token) as allowed:
            if not allowed:
                return
            self.control.state.complete(token, stamps, message.valid, error[0] if error else '')
            if self.control.state.mode == ACTIVE:
                self.publisher.publish(message)
                if self.raw_publisher is not None:
                    self.raw_publisher.publish(raw if raw is not None else message)
                self.last_header = copy.deepcopy(frame.header)
                self.last_headers[frame.view] = copy.deepcopy(frame.header)
                if self.debug_masks and message.valid:
                    self._publish_masks(frame, processor, message)
                self.control.state.record('published', stamps=stamps, timings=timings, valid=bool(message.valid))
        with self.lock:
            self.filter_counts = dict(processor.last_counts)
            self.excluded_boxes += self.filter_counts['excluded']
            self.output_frames += 1
            self.view_counts[frame.view] += 1
            self.latest_result_stamp_ns = frame.stamp.to_nsec()
            self.queue_samples.append((time.monotonic()-frame.received_monotonic)*1000)
            elapsed_ns = rospy.Time.now().to_nsec()-frame.stamp.to_nsec()
            if elapsed_ns >= 0:
                self.e2e_samples.append(elapsed_ns/1e6)
            for key,value in timings.items():
                self.stage_samples.setdefault(key,deque(maxlen=500)).append(value)
            if error:
                if error[0] == 'decode': self.decode_errors += 1
                elif error[0] == 'filter': self.filter_errors += 1
                elif error[0] == 'mask': self.mask_errors += 1
                else: self.inference_errors += 1
        if error:
            rospy.logwarn_throttle(2, 'camera objects %s failure: %s', *error)

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
                values['ego_mask_by_view'] = {v: p.hood_mask.info for v, p in self.processors.items()}
                values.update({"ego_hood_filter_"+key: value for key,value in self.processor.ego_hood_filter.info.items()})
                values.update(debug_raw_enabled=self.raw_publisher is not None,
                              ego_hood_filter_last_counts=self.filter_counts,
                              ego_hood_filter_excluded_total=self.excluded_boxes, filter_errors=self.filter_errors)
                fps = (self.output_frames - previous_count) / max(now - previous_at, 1e-6)
                previous_count, previous_at = self.output_frames, now
                values.update(label_mode=self.processor.label_mode,
                              target_hz_by_view=dict(self.control.state.target_hz_by_view),
                              processing_fps=round(fps, 3), output_frames=self.output_frames,
                              received_frames=sum(self.buffer.received.values()), replaced_frames=sum(self.buffer.replaced.values()),
                              views=self.views, output_by_view=dict(self.view_counts),
                              received_by_view=dict(self.buffer.received), replaced_by_view=dict(self.buffer.replaced),
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

    def _publish_masks(self, frame, processor, message):
        publisher = self.mask_publishers[frame.view]
        if not publisher.get_num_connections():
            return
        import cv2
        from asmc_camera.detection.visualization import render_object_overlay
        image = processor.last_image.copy()
        for index, mask in enumerate(processor.last_masks):
            if mask is not None:
                color = np.array([60+(index*47)%190, 180, 60+(index*83)%190])
                image[mask] = (image[mask]*.55+color*.45).astype(np.uint8)
        image = render_object_overlay(image, message, processor.hood_mask,
                                      mask_label='hood mask' if frame.view == 'front' else 'ego mask '+frame.view)
        ok, data = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY,80])
        if ok:
            publisher.publish(CompressedImage(header=copy.deepcopy(frame.header),format='jpeg',data=data.tobytes()))

    def shutdown(self):
        self.stop.set()
        for subscriber in self.subscribers:
            subscriber.unregister()
        self.buffer.close()
        for thread in (self.worker, self.monitor):
            if thread is not threading.current_thread():
                thread.join(timeout=2)
        self.control.close()


def main():
    rospy.init_node("camera_object_detector")
    try:
        ObjectDetectionNode()
    except Exception as exc:
        rospy.logfatal("camera object detector startup failed: %s", exc)
        raise SystemExit(1)
    rospy.spin()
