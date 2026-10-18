#!/usr/bin/env python3
"""Timestamp-synchronized ROS1 visualizer for camera semantic probabilities."""

from __future__ import annotations

import threading
import time

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage, Image

from asmc_camera.camera.visualization import render_mosaic, render_panel, binary_lane_palette
from asmc_camera.runtime.demand_inputs import DemandSynchronizer, DemandInputs
from asmc_camera.runtime.ros_control import LaneEpochGate
from asmc_camera.runtime.render_worker import RenderWorker


SUPPORTED_VIEWS = ("front", "left", "right")


class SemanticVisualizerNode:
    def __init__(self) -> None:
        # This renderer owns its process. Small LUT/resize tasks otherwise wake
        # all 20 CPU threads, reducing wall time but spending ~4x CPU time.
        opencv_threads = int(rospy.get_param('~opencv_threads', 1))
        if opencv_threads < 1:
            raise ValueError('opencv_threads must be positive')
        cv2.setNumThreads(opencv_threads)
        self.views = tuple(str(value) for value in rospy.get_param("~views", ["front"]))
        unknown = set(self.views) - set(SUPPORTED_VIEWS)
        if not self.views or unknown or len(set(self.views)) != len(self.views):
            raise ValueError(
                "views must be unique values from {}; got {}".format(
                    SUPPORTED_VIEWS, self.views
                )
            )
        camera_topics = dict(rospy.get_param("~camera_topics"))
        missing = [view for view in self.views if view not in camera_topics]
        if missing:
            raise ValueError("camera_topics missing views: {}".format(missing))
        self.secondary_mode = str(
            rospy.get_param("~secondary_mode", "lane")
        ).strip()
        if self.secondary_mode not in ("lane", "road_marking"):
            raise ValueError("secondary_mode must be lane or road_marking")
        self.lane_palette = binary_lane_palette(rospy.get_param('~lane_color_bgr', [0, 230, 255]))
        secondary_topic_suffix = (
            "lane/probability"
            if self.secondary_mode == "lane"
            else "road_marking/class_id"
        )

        self.publish_hz = float(rospy.get_param("~publish_hz", 10.0))
        self.jpeg_quality = int(rospy.get_param("~jpeg_quality", 80))
        self.panel_width = int(rospy.get_param("~panel_width", 640))
        self.mosaic_view_width = int(rospy.get_param("~mosaic_view_width", 640))
        sync_queue_size = int(rospy.get_param("~sync_queue_size", 8))
        if self.publish_hz <= 0.0:
            raise ValueError("publish_hz must be positive")
        if not 1 <= self.jpeg_quality <= 100:
            raise ValueError("jpeg_quality must be between 1 and 100")
        self.publish_per_view_panels = bool(
            rospy.get_param("~publish_per_view_panels", False)
        )

        self.panel_publishers = {}
        if self.publish_per_view_panels:
            self.panel_publishers = {
                view: rospy.Publisher(
                    "/perception/camera/{}/debug/panel/compressed".format(view),
                    CompressedImage,
                    queue_size=1,
                )
                for view in self.views
            }
        self.mosaic_publisher = rospy.Publisher(
            "/perception/camera/debug/mosaic/compressed",
            CompressedImage,
            queue_size=1,
        )
        self._lock = threading.RLock()
        self.last_display_stamp = None
        self.last_display_key = None
        self.samples = {}  # One exact-stamp triple per view; newer inputs replace it.
        self.latest_panels = {}
        self.latest_headers = {}
        self.panel_keys = {}
        self.callback_counts = {view: 0 for view in self.views}
        self.render_counts = {view: 0 for view in self.views}
        self.mosaic_count = 0
        self.decode_errors = self.shape_errors = 0
        self._last_log_wall = time.monotonic()
        self._last_mosaic_count = 0
        self.gate = LaneEpochGate(on_standby=self._clear)
        self.synchronizers = []
        buffer_bytes = int(rospy.get_param("~subscriber_buffer_bytes", 16 << 20))
        for view in self.views:
            topics = [str(camera_topics[view]),
                      "/perception/camera/{}/drivable/probability".format(view),
                      "/perception/camera/{}/{}".format(view, secondary_topic_suffix)]
            group = DemandSynchronizer(
                topics, Image, lambda v=view: self._requested(v), None,
                context=self.gate.context, on_reset=lambda v=view: self._reset_view(v),
                queue_size=sync_queue_size, subscriber_queue_size=1,
                subscriber_buffer_bytes=buffer_bytes,
                message_types=[CompressedImage, Image, Image])
            # Stamp the captured subscription epoch, not a newer status that
            # could arrive between synchronizer delivery and this callback.
            group.callback = lambda *messages, v=view, g=group: self._callback(
                *messages, v, context=g.current_context)
            self.synchronizers.append(group)
        self.demand_inputs = DemandInputs(self.synchronizers)
        self.render_worker = RenderWorker(lambda: self._publish(None), self.publish_hz,
            lambda error: rospy.logerr_throttle(2., 'semantic view worker failed: %s', error))
        self.timer = rospy.Timer(rospy.Duration(.1), self._heartbeat)
        rospy.on_shutdown(self.close)

        rospy.loginfo(
            "camera semantic visualizer views=%s secondary_mode=%s "
            "publish_hz=%.2f panel_width=%d "
            "mosaic_view_width=%d jpeg_quality=%d per_view_panels=%s",
            self.views,
            self.secondary_mode,
            self.publish_hz,
            self.panel_width,
            self.mosaic_view_width,
            self.jpeg_quality,
            self.publish_per_view_panels,
        )

    def _requested(self, view):
        publisher = self.panel_publishers.get(view)
        return (self.mosaic_publisher.get_num_connections() > 0 or
                bool(publisher and publisher.get_num_connections() > 0))

    def _clear(self):
        with self._lock:
            self.samples.clear()

    def _reset_view(self, view):
        with self._lock:
            self.samples.pop(view, None)

    def _callback(self, rgb, drivable, secondary, view, context=None):
        # No JPEG or large arrays in a transport/synchronizer callback.
        context = self.gate.context() if context is None else context
        with self._lock:
            self.callback_counts[view] += 1
            self.samples[view] = (rgb, drivable, secondary, context, time.monotonic())
        worker = getattr(self, 'render_worker', None)
        if worker is not None:
            worker.offer()

    def _current(self, sample, context):
        return bool(sample and sample[3] == context and
                    self.gate.allows(sample[0].header.stamp) and
                    time.monotonic()-sample[4] <= .35 and
                    -.05 <= (rospy.Time.now()-sample[0].header.stamp).to_sec() <= .35)

    def _render(self, sample, view):
        rgb, drivable, secondary = sample[:3]
        image = cv2.imdecode(np.frombuffer(rgb.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            self.decode_errors += 1
            return None
        try:
            probability = self._probability_array(drivable)
            other = (self._probability_array(secondary) if self.secondary_mode == 'lane'
                     else self._class_id_array(secondary))
            if probability.shape != image.shape[:2] or other.shape != image.shape[:2]:
                raise ValueError('RGB/semantic shape mismatch')
            return render_panel(image, probability, other, view=view,
                stamp_text='{:.3f}'.format(rgb.header.stamp.to_sec()),
                condition_label=str(rospy.get_param_cached('~condition_label', '')),
                drivable_threshold=float(rospy.get_param_cached('~drivable_threshold', .5)),
                lane_threshold=float(rospy.get_param_cached('~lane_threshold', .5)),
                secondary_mode=self.secondary_mode, panel_width=self.panel_width,
                lane_palette=self.lane_palette)
        except (TypeError, ValueError) as error:
            self.shape_errors += 1
            rospy.logwarn_throttle(2., 'visualizer rejected synchronized frame: %s', error)
            return None

    def _publish(self, _event):
        if not any(self._requested(view) for view in self.views):
            self._clear()
            self.latest_panels.clear()
            self.latest_headers.clear()
            self.panel_keys.clear()
            self.last_display_stamp = self.last_display_key = None
            return False
        with self._lock:
            samples = dict(self.samples)
        context = self.gate.context()
        panels, headers, fresh = {}, {}, {}
        for view in self.views:
            if not self._requested(view):
                continue
            sample = samples.get(view)
            if self._current(sample, context):
                key = (sample[0].header.stamp.to_nsec(), context)
                if self.panel_keys.get(view) != key:
                    panel = self._render(sample, view)
                    if panel is None:
                        continue
                    self.latest_panels[view] = panel
                    self.latest_headers[view] = sample[0].header
                    self.panel_keys[view] = key
                    self.render_counts[view] += 1
                panels[view] = self.latest_panels[view]
                headers[view] = sample[0].header
                fresh[view] = sample
            else:
                # Never keep a paused camera's semantic result on screen.
                height = (360 if view == 'front' else 480) * 3
                panels[view] = np.zeros((height, self.panel_width, 3), np.uint8)
                cv2.putText(panels[view], view.upper()+' WAITING / STALE', (12, 35),
                            cv2.FONT_HERSHEY_SIMPLEX, .7, (180, 180, 180), 1)
        display_key = (context, tuple((view, headers[view].stamp.to_nsec() if view in headers else None)
                                      for view in self.views))
        if display_key == self.last_display_key:
            return False
        encoded_panels = {}
        for view, publisher in self.panel_publishers.items():
            if publisher.get_num_connections() and view in panels:
                encoded_panels[view] = self._compressed_message(panels[view], headers.get(view), 'camera_'+view)
        mosaic = None
        if self.mosaic_publisher.get_num_connections() and panels:
            # Missing/invalid views are visible rather than blocking other views.
            for view in self.views:
                if view not in panels:
                    panels[view] = np.zeros((1080, self.panel_width, 3), np.uint8)
            header = max(headers.values(), key=lambda h: h.stamp.to_nsec()) if headers else None
            mosaic = self._compressed_message(render_mosaic(panels, self.views, self.mosaic_view_width),
                                              header, 'camera_debug_mosaic')
        with self.gate.lock:
            if context != self.gate.context():
                return False
            if any(not self._current(sample, context) for sample in fresh.values()):
                return False
            for view, message in encoded_panels.items():
                if message is not None and self._requested(view):
                    self.panel_publishers[view].publish(message)
            if mosaic is not None and self.mosaic_publisher.get_num_connections():
                self.mosaic_publisher.publish(mosaic)
                self.mosaic_count += 1
            self.last_display_stamp = max((h.stamp.to_nsec() for h in headers.values()), default=None)
            self.last_display_key = display_key
        self._maybe_log(time.monotonic())
        return bool(encoded_panels or mosaic is not None)

    def _heartbeat(self, _event):
        # Only demand/expiry/context transitions need polling. A current source
        # is offered by its exact-stamp callback, without waiting for this timer.
        with self._lock:
            samples = dict(self.samples)
        context = self.gate.context()
        if not any(self._requested(v) for v in self.views):
            if samples or self.latest_panels:
                self.render_worker.offer()
            return
        key = (context, tuple((v, samples[v][0].header.stamp.to_nsec()
               if self._current(samples.get(v), context) else None) for v in self.views))
        if key != self.last_display_key:
            self.render_worker.offer()

    def close(self):
        self.timer.shutdown()
        self.demand_inputs.close()
        self.render_worker.close()

    @staticmethod
    def _probability_array(message: Image) -> np.ndarray:
        if message.encoding == "mono8":
            row = np.frombuffer(message.data, dtype=np.uint8).reshape(
                message.height, message.step
            )
            return np.ascontiguousarray(row[:, : message.width])
        if message.encoding == "32FC1":
            if message.step % 4:
                raise ValueError("32FC1 step is not divisible by four")
            row = np.frombuffer(message.data, dtype=np.float32).reshape(
                message.height, message.step // 4
            )
            return np.ascontiguousarray(row[:, : message.width])
        raise ValueError("unsupported probability encoding: {}".format(message.encoding))

    @staticmethod
    def _class_id_array(message: Image) -> np.ndarray:
        if message.encoding not in ("mono8", "8UC1"):
            raise ValueError(
                "unsupported class ID encoding: {}".format(message.encoding)
            )
        row = np.frombuffer(message.data, dtype=np.uint8).reshape(
            message.height, message.step
        )
        value = np.ascontiguousarray(row[:, : message.width])
        if value.size and int(value.max()) > 3:
            raise ValueError("road-marking class ID must be in [0,3]")
        return value

    def _compressed_message(self, image: np.ndarray, header, frame_id=None):
        ok, encoded = cv2.imencode(
            ".jpg", image, (cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality)
        )
        if not ok:
            self.decode_errors += 1
            rospy.logwarn_throttle(2.0, "visualizer failed to encode debug JPEG")
            return None
        message = CompressedImage()
        message.header.stamp = header.stamp if header is not None else rospy.Time()
        message.header.frame_id = frame_id or (header.frame_id if header is not None else "")
        message.format = "jpeg"
        message.data = encoded.tobytes()
        return message

    def _maybe_log(self, now: float) -> None:
        elapsed = now - self._last_log_wall
        if elapsed < 2.0:
            return
        rospy.loginfo('visualizer mosaic_hz=%.2f callbacks=%s rendered=%s decode_errors=%d shape_errors=%d',
                      (self.mosaic_count-self._last_mosaic_count)/elapsed,
                      self.callback_counts, self.render_counts, self.decode_errors, self.shape_errors)
        self._last_log_wall = now
        self._last_mosaic_count = self.mosaic_count



def main() -> None:
    rospy.init_node("camera_semantic_visualizer")
    SemanticVisualizerNode()
    rospy.spin()


if __name__ == "__main__":
    main()
