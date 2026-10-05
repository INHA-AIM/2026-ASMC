#!/usr/bin/env python3
"""Timestamp-synchronized ROS1 visualizer for camera semantic probabilities."""

from __future__ import annotations

import threading
import time

import cv2
import message_filters
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage, Image

from asmc_camera.camera.visualization import render_mosaic, render_panel


SUPPORTED_VIEWS = ("front", "left", "right")


class SemanticVisualizerNode:
    def __init__(self) -> None:
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
        secondary_topic_suffix = (
            "lane/probability"
            if self.secondary_mode == "lane"
            else "road_marking/class_id"
        )

        self.publish_hz = float(rospy.get_param("~publish_hz", 10.0))
        self.jpeg_quality = int(rospy.get_param("~jpeg_quality", 80))
        self.panel_width = int(rospy.get_param("~panel_width", 640))
        self.mosaic_view_width = int(rospy.get_param("~mosaic_view_width", 640))
        sync_queue_size = int(rospy.get_param("~sync_queue_size", 30))
        if self.publish_hz <= 0.0:
            raise ValueError("publish_hz must be positive")
        if not 1 <= self.jpeg_quality <= 100:
            raise ValueError("jpeg_quality must be between 1 and 100")
        self.minimum_period = 1.0 / self.publish_hz
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
        self.latest_panels = {}
        self.latest_headers = {}
        self.last_panel_wall = {view: 0.0 for view in self.views}
        self.last_mosaic_wall = 0.0
        self.callback_counts = {view: 0 for view in self.views}
        self.render_counts = {view: 0 for view in self.views}
        self.decode_errors = 0
        self.shape_errors = 0
        self._lock = threading.Lock()
        self._last_log_wall = time.monotonic()
        self._last_render_total = 0

        self.subscribers = []
        self.synchronizers = []
        buffer_bytes = int(rospy.get_param("~subscriber_buffer_bytes", 16 << 20))
        for view in self.views:
            rgb = message_filters.Subscriber(
                str(camera_topics[view]),
                CompressedImage,
                queue_size=1,
                buff_size=buffer_bytes,
            )
            drivable = message_filters.Subscriber(
                "/perception/camera/{}/drivable/probability".format(view),
                Image,
                queue_size=1,
                buff_size=buffer_bytes,
            )
            secondary = message_filters.Subscriber(
                "/perception/camera/{}/{}".format(view, secondary_topic_suffix),
                Image,
                queue_size=1,
                buff_size=buffer_bytes,
            )
            synchronizer = message_filters.TimeSynchronizer(
                (rgb, drivable, secondary), sync_queue_size
            )
            synchronizer.registerCallback(self._callback, view)
            self.subscribers.extend((rgb, drivable, secondary))
            self.synchronizers.append(synchronizer)

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

    def _callback(
        self,
        rgb_message: CompressedImage,
        drivable_message: Image,
        secondary_message: Image,
        view: str,
    ) -> None:
        self.callback_counts[view] += 1
        panel_publisher = self.panel_publishers.get(view)
        publish_panel = (
            panel_publisher is not None
            and panel_publisher.get_num_connections() > 0
        )
        publish_mosaic = self.mosaic_publisher.get_num_connections() > 0
        if not publish_panel and not publish_mosaic:
            self._maybe_log(time.monotonic())
            return
        now = time.monotonic()
        if now - self.last_panel_wall[view] < self.minimum_period:
            self._maybe_log(now)
            return

        image = cv2.imdecode(
            np.frombuffer(rgb_message.data, dtype=np.uint8), cv2.IMREAD_COLOR
        )
        if image is None:
            self.decode_errors += 1
            rospy.logwarn_throttle(2.0, "visualizer failed to decode camera JPEG")
            return
        try:
            drivable = self._probability_array(drivable_message)
            secondary = (
                self._probability_array(secondary_message)
                if self.secondary_mode == "lane"
                else self._class_id_array(secondary_message)
            )
            if drivable.shape != image.shape[:2] or secondary.shape != image.shape[:2]:
                raise ValueError(
                    "RGB {} drivable {} secondary {}".format(
                        image.shape[:2], drivable.shape, secondary.shape
                    )
                )
            panel = render_panel(
                image,
                drivable,
                secondary,
                view=view,
                stamp_text="{:.3f}".format(rgb_message.header.stamp.to_sec()),
                condition_label=str(rospy.get_param_cached("~condition_label", "")),
                drivable_threshold=float(
                    rospy.get_param_cached("~drivable_threshold", 0.5)
                ),
                lane_threshold=float(
                    rospy.get_param_cached("~lane_threshold", 0.5)
                ),
                secondary_mode=self.secondary_mode,
                panel_width=self.panel_width,
            )
        except (TypeError, ValueError) as exc:
            self.shape_errors += 1
            rospy.logwarn_throttle(2.0, "visualizer rejected synchronized frame: %s", exc)
            return

        if publish_panel:
            encoded = self._compressed_message(panel, rgb_message.header)
            if encoded is None:
                return
            panel_publisher.publish(encoded)
        self.last_panel_wall[view] = now
        self.render_counts[view] += 1

        if not publish_mosaic:
            self._maybe_log(now)
            return
        with self._lock:
            self.latest_panels[view] = panel
            self.latest_headers[view] = rgb_message.header
            if (
                len(self.latest_panels) == len(self.views)
                and now - self.last_mosaic_wall >= self.minimum_period
            ):
                mosaic = render_mosaic(
                    self.latest_panels, self.views, self.mosaic_view_width
                )
                newest_view = max(
                    self.views,
                    key=lambda name: self.latest_headers[name].stamp.to_sec(),
                )
                mosaic_message = self._compressed_message(
                    mosaic, self.latest_headers[newest_view], "camera_debug_mosaic"
                )
                if mosaic_message is not None:
                    self.mosaic_publisher.publish(mosaic_message)
                    self.last_mosaic_wall = now
        self._maybe_log(now)

    @staticmethod
    def _probability_array(message: Image) -> np.ndarray:
        if message.encoding == "mono8":
            row = np.frombuffer(message.data, dtype=np.uint8).reshape(
                message.height, message.step
            )
            return np.ascontiguousarray(row[:, : message.width], dtype=np.float32) / 255.0
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
        message.header.stamp = header.stamp
        message.header.frame_id = frame_id or header.frame_id
        message.format = "jpeg"
        message.data = encoded.tobytes()
        return message

    def _maybe_log(self, now: float) -> None:
        elapsed = now - self._last_log_wall
        if elapsed < 2.0:
            return
        total = sum(self.render_counts.values())
        rospy.loginfo(
            "visualizer fps=%.2f callbacks=%s rendered=%s decode_errors=%d shape_errors=%d",
            (total - self._last_render_total) / elapsed,
            self.callback_counts,
            self.render_counts,
            self.decode_errors,
            self.shape_errors,
        )
        self._last_log_wall = now
        self._last_render_total = total


def main() -> None:
    rospy.init_node("camera_semantic_visualizer")
    SemanticVisualizerNode()
    rospy.spin()


if __name__ == "__main__":
    main()
