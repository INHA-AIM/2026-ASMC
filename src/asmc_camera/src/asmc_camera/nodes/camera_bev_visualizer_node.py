#!/usr/bin/env python3
"""Publish a compact RGB/semantic BEV comparison and optional legacy debug views."""

from __future__ import annotations

import threading
import time
from collections import deque
from functools import partial

import cv2
import message_filters
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage, Image

from asmc_camera.bev.calibration import calibrations_from_dict
from asmc_camera.bev.fusion import build_quality_maps
from asmc_camera.bev.grid import BevGridSpec
from asmc_camera.bev.projector import CameraBevProjector
from asmc_camera.bev.visualization import (
    build_bev_mosaic,
    crop_bev_layers,
    draw_ground_grid_overlay,
    fuse_projected_rgb,
    render_bev_comparison,
    render_fused_bev,
    render_fused_quality,
)


def _enabled_views(value):
    if isinstance(value, str):
        values = [item.strip() for item in value.split(",")]
    else:
        values = [str(item).strip() for item in value]
    return [item for item in values if item]


class CameraBevVisualizerNode:
    def __init__(self) -> None:
        self.grid = BevGridSpec.from_mapping(
            {
                name: rospy.get_param("~" + name)
                for name in (
                    "frame_id",
                    "x_min_m",
                    "x_max_m",
                    "y_min_m",
                    "y_max_m",
                    "resolution_m",
                )
            }
        )
        self.display_grid = BevGridSpec(
            x_min_m=float(rospy.get_param("~display_x_min_m", self.grid.x_min_m)),
            x_max_m=float(rospy.get_param("~display_x_max_m", self.grid.x_max_m)),
            y_min_m=float(rospy.get_param("~display_y_min_m", self.grid.y_min_m)),
            y_max_m=float(rospy.get_param("~display_y_max_m", self.grid.y_max_m)),
            resolution_m=self.grid.resolution_m,
            frame_id=self.grid.frame_id,
        )
        camera_values = rospy.get_param("~cameras")
        all_cameras = calibrations_from_dict(
            {
                "ground_plane": rospy.get_param("~ground_plane"),
                "cameras": camera_values,
            }
        )
        self.views = _enabled_views(
            rospy.get_param("~enabled_views", ["front", "left", "right"])
        )
        self.cameras = {view: all_cameras[view] for view in self.views}
        self.view_order = [view for view in ("left", "front", "right") if view in self.views]

        max_ground_range = float(rospy.get_param("~max_ground_range_m", 22.0))
        min_camera_depth = float(rospy.get_param("~min_camera_depth_m", 0.1))
        self.projectors = {
            view: CameraBevProjector(
                self.cameras[view],
                self.grid,
                max_ground_range_m=max_ground_range,
                min_camera_depth_m=min_camera_depth,
            )
            for view in self.views
        }
        self.rgb_quality = build_quality_maps(
            self.projectors,
            self.grid,
            edge_taper_fraction=float(rospy.get_param("~edge_taper_fraction", 0.12)),
            forward_fade_start_m=float(
                rospy.get_param("~forward_fade_start_m", 15.0)
            ),
            lateral_fade_start_m=float(
                rospy.get_param("~lateral_fade_start_m", 6.0)
            ),
            rear_fade_start_m=float(rospy.get_param("~rear_fade_start_m", -4.0)),
            boundary_weight=float(rospy.get_param("~boundary_weight", 0.20)),
        )

        self.jpeg_quality = int(rospy.get_param("~jpeg_quality", 85))
        self.publish_legacy_outputs = bool(
            rospy.get_param("~publish_legacy_outputs", False)
        )
        self.publish_calibration_grid = bool(
            rospy.get_param("~publish_calibration_grid", False)
        )
        self.publish_per_view_mosaic = bool(
            rospy.get_param("~publish_per_view_mosaic", False)
        ) and self.publish_legacy_outputs
        self.fused_size = (
            int(rospy.get_param("~fused_width", 672)),
            int(rospy.get_param("~fused_height", 1050)),
        )
        if min(self.fused_size) <= 0:
            raise ValueError("fused visualization dimensions must be positive")
        self.display_label = str(rospy.get_param("~display_label", "FUSED"))
        self.comparison_drivable_threshold = float(
            rospy.get_param("~comparison_drivable_threshold", 0.5)
        )
        if not 0.0 <= self.comparison_drivable_threshold <= 1.0:
            raise ValueError("comparison_drivable_threshold must lie in [0,1]")
        self.raw_comparison_max_skew_ns = int(
            round(float(rospy.get_param("~raw_comparison_max_skew_ms", 100.0)) * 1e6)
        )
        if self.raw_comparison_max_skew_ns < 0:
            raise ValueError("raw_comparison_max_skew_ms cannot be negative")
        self.raw_history_size = int(rospy.get_param("~raw_comparison_history_size", 40))
        if self.raw_history_size <= 0:
            raise ValueError("raw_comparison_history_size must be positive")
        self.tile_size = (
            int(rospy.get_param("~tile_width", 300)),
            int(rospy.get_param("~tile_height", 600)),
        )
        self.longitudinal_step_m = float(
            rospy.get_param("~camera_grid_longitudinal_step_m", 5.0)
        )
        self.lateral_step_m = float(
            rospy.get_param("~camera_grid_lateral_step_m", 2.0)
        )
        camera_grid_hz = float(rospy.get_param("~camera_grid_max_hz", 5.0))
        self.camera_grid_period = 1.0 / max(0.1, camera_grid_hz)
        self.last_grid_output = {view: 0.0 for view in self.views}

        self.layers = {}
        self.latest_stamp = rospy.Time()
        self.dirty = False
        self.fused_layers = None
        self.fused_stamp = rospy.Time()
        self.fused_dirty = False
        self.raw_frames = {
            view: deque(maxlen=self.raw_history_size) for view in self.views
        }
        self.lock = threading.Lock()
        self.subscribers = []
        self.synchronizers = []
        self.grid_publishers = {}
        for view in self.views:
            namespace = "/perception/bev/debug/{}".format(view)
            if self.publish_per_view_mosaic:
                drivable = message_filters.Subscriber(
                    namespace + "/drivable_probability", Image, queue_size=2
                )
                marking = message_filters.Subscriber(
                    namespace + "/road_marking/class_id", Image, queue_size=2
                )
                coverage = message_filters.Subscriber(
                    namespace + "/coverage", Image, queue_size=2
                )
                sync = message_filters.TimeSynchronizer(
                    [drivable, marking, coverage], queue_size=4
                )
                sync.registerCallback(partial(self._bev_callback, view))
                self.subscribers.extend([drivable, marking, coverage])
                self.synchronizers.append(sync)

            if self.publish_calibration_grid:
                self.grid_publishers[view] = rospy.Publisher(
                    namespace + "/camera_grid_overlay/compressed",
                    CompressedImage,
                    queue_size=1,
                )
            self.subscribers.append(
                rospy.Subscriber(
                    self.cameras[view].image_topic,
                    CompressedImage,
                    partial(self._camera_callback, view),
                    queue_size=1,
                    buff_size=16 * 1024 * 1024,
                )
            )

        fused_namespace = "/perception/bev/camera"
        fused_topics = [
            fused_namespace + "/drivable_probability",
            fused_namespace + "/road_marking/class_id",
            fused_namespace + "/coverage",
        ]
        if self.publish_legacy_outputs:
            fused_topics.extend(
                [
                    fused_namespace + "/road_marking/confidence",
                    fused_namespace + "/source_count",
                ]
            )
        fused_subscribers = [
            message_filters.Subscriber(topic, Image, queue_size=2)
            for topic in fused_topics
        ]
        fused_sync = message_filters.TimeSynchronizer(
            fused_subscribers, queue_size=4
        )
        fused_sync.registerCallback(
            self._legacy_fused_callback
            if self.publish_legacy_outputs
            else self._compact_fused_callback
        )
        self.subscribers.extend(fused_subscribers)
        self.synchronizers.append(fused_sync)
        self.comparison_publisher = rospy.Publisher(
            "/perception/bev/debug/comparison/compressed",
            CompressedImage,
            queue_size=1,
        )
        self.fused_publisher = None
        self.quality_publisher = None
        if self.publish_legacy_outputs:
            self.fused_publisher = rospy.Publisher(
                "/perception/bev/debug/fused/compressed",
                CompressedImage,
                queue_size=1,
            )
            self.quality_publisher = rospy.Publisher(
                "/perception/bev/debug/fused_quality/compressed",
                CompressedImage,
                queue_size=1,
            )
        fused_hz = float(rospy.get_param("~fused_max_hz", 5.0))
        self.fused_timer = rospy.Timer(
            rospy.Duration(1.0 / max(0.1, fused_hz)), self._publish_outputs
        )

        self.mosaic_publisher = None
        self.mosaic_timer = None
        if self.publish_per_view_mosaic:
            self.mosaic_publisher = rospy.Publisher(
                "/perception/bev/debug/mosaic/compressed",
                CompressedImage,
                queue_size=1,
            )
            mosaic_hz = float(rospy.get_param("~mosaic_max_hz", 5.0))
            self.mosaic_timer = rospy.Timer(
                rospy.Duration(1.0 / max(0.1, mosaic_hz)), self._publish_mosaic
            )
        rospy.loginfo(
            "camera BEV visualizer ready views=%s comparison=2x%sx%s "
            "legacy=%s calibration_grid=%s per_view_mosaic=%s raw_history=%d",
            self.views,
            self.fused_size[0],
            self.fused_size[1],
            self.publish_legacy_outputs,
            self.publish_calibration_grid,
            self.publish_per_view_mosaic,
            self.raw_history_size,
        )

    def _compact_fused_callback(
        self,
        drivable: Image,
        marking: Image,
        coverage: Image,
    ) -> None:
        self._store_fused_layers(drivable, marking, coverage)

    def _legacy_fused_callback(
        self,
        drivable: Image,
        marking: Image,
        coverage: Image,
        confidence: Image,
        source_count: Image,
    ) -> None:
        self._store_fused_layers(
            drivable,
            marking,
            coverage,
            confidence=confidence,
            source_count=source_count,
        )

    def _store_fused_layers(
        self,
        drivable: Image,
        marking: Image,
        coverage: Image,
        confidence: Image = None,
        source_count: Image = None,
    ) -> None:
        try:
            layers = {
                "drivable": self._mono8_array(drivable, self.grid.shape),
                "road_marking": self._mono8_array(marking, self.grid.shape),
                "coverage": self._mono8_array(coverage, self.grid.shape),
            }
            if confidence is not None:
                layers["confidence"] = self._mono8_array(
                    confidence, self.grid.shape
                )
            if source_count is not None:
                layers["source_count"] = self._mono8_array(
                    source_count, self.grid.shape
                )
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "fused BEV visualization input invalid: %s", exc)
            return
        with self.lock:
            self.fused_layers = layers
            self.fused_stamp = drivable.header.stamp
            self.fused_dirty = True

    def _bev_callback(
        self, view: str, drivable: Image, marking: Image, coverage: Image
    ) -> None:
        try:
            layers = {
                "drivable": self._mono8_array(drivable, self.grid.shape),
                "road_marking": self._mono8_array(marking, self.grid.shape),
                "coverage": self._mono8_array(coverage, self.grid.shape),
            }
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "%s BEV visualization input invalid: %s", view, exc)
            return
        with self.lock:
            self.layers[view] = layers
            self.latest_stamp = drivable.header.stamp
            self.dirty = True

    def _camera_callback(self, view: str, message: CompressedImage) -> None:
        comparison_requested = self.comparison_publisher.get_num_connections() > 0
        grid_publisher = self.grid_publishers.get(view)
        grid_requested = (
            grid_publisher is not None
            and grid_publisher.get_num_connections() > 0
        )
        if not comparison_requested and not grid_requested:
            return
        if comparison_requested:
            with self.lock:
                self.raw_frames[view].append(
                    (
                        bytes(message.data),
                        message.header.stamp,
                        message.header.frame_id,
                    )
                )
        if not grid_requested:
            return
        now = time.monotonic()
        if now - self.last_grid_output[view] < self.camera_grid_period:
            return
        image = cv2.imdecode(
            np.frombuffer(bytes(message.data), dtype=np.uint8), cv2.IMREAD_COLOR
        )
        if image is None:
            rospy.logwarn_throttle(2.0, "%s calibration image decode failed", view)
            return
        try:
            overlay = draw_ground_grid_overlay(
                image,
                self.cameras[view],
                x_min_m=self.display_grid.x_min_m,
                x_max_m=self.display_grid.x_max_m,
                y_min_m=self.display_grid.y_min_m,
                y_max_m=self.display_grid.y_max_m,
                longitudinal_step_m=self.longitudinal_step_m,
                lateral_step_m=self.lateral_step_m,
            )
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "%s calibration overlay failed: %s", view, exc)
            return
        grid_publisher.publish(
            self._compressed(overlay, message.header.stamp, message.header.frame_id)
        )
        self.last_grid_output[view] = now

    def _publish_mosaic(self, _event) -> None:
        if (
            self.mosaic_publisher is None
            or self.mosaic_publisher.get_num_connections() == 0
        ):
            return
        with self.lock:
            if not self.dirty:
                return
            layers = dict(self.layers)
            stamp = self.latest_stamp
            self.dirty = False
        try:
            mosaic = build_bev_mosaic(
                layers, self.grid, self.view_order, self.tile_size
            )
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "BEV mosaic failed: %s", exc)
            return
        self.mosaic_publisher.publish(
            self._compressed(mosaic, stamp, self.grid.frame_id)
        )

    def _publish_outputs(self, _event) -> None:
        publish_comparison = self.comparison_publisher.get_num_connections() > 0
        publish_fused = (
            self.fused_publisher is not None
            and self.fused_publisher.get_num_connections() > 0
        )
        publish_quality = (
            self.quality_publisher is not None
            and self.quality_publisher.get_num_connections() > 0
        )
        if not publish_comparison and not publish_fused and not publish_quality:
            return
        with self.lock:
            if not self.fused_dirty or self.fused_layers is None:
                return
            layers = dict(self.fused_layers)
            stamp = self.fused_stamp
            raw_frames = {
                view: tuple(history) for view, history in self.raw_frames.items()
            }
            self.fused_dirty = False
        try:
            layers = crop_bev_layers(layers, self.grid, self.display_grid)
            if publish_comparison:
                rgb_bev = self._build_rgb_bev(raw_frames, stamp)
                rgb_bev = crop_bev_layers(
                    {"rgb": rgb_bev}, self.grid, self.display_grid
                )["rgb"]
                comparison = render_bev_comparison(
                    rgb_bev,
                    layers["drivable"],
                    layers["road_marking"],
                    layers["coverage"],
                    self.display_grid,
                    panel_size=self.fused_size,
                    drivable_threshold=self.comparison_drivable_threshold,
                )
                self.comparison_publisher.publish(
                    self._compressed(comparison, stamp, self.grid.frame_id)
                )
            if publish_fused:
                image = render_fused_bev(
                    layers["drivable"],
                    layers["road_marking"],
                    layers["confidence"],
                    layers["coverage"],
                    layers["source_count"],
                    self.display_grid,
                    label_prefix=self.display_label,
                )
                image = cv2.resize(
                    image,
                    self.fused_size,
                    interpolation=cv2.INTER_NEAREST,
                )
                self.fused_publisher.publish(
                    self._compressed(image, stamp, self.grid.frame_id)
                )
            if publish_quality:
                quality = render_fused_quality(
                    layers["road_marking"],
                    layers["confidence"],
                    layers["coverage"],
                    layers["source_count"],
                    self.display_grid,
                    label_prefix=self.display_label.replace("FUSED", "QUALITY", 1),
                )
                quality = cv2.resize(
                    quality,
                    self.fused_size,
                    interpolation=cv2.INTER_NEAREST,
                )
                self.quality_publisher.publish(
                    self._compressed(quality, stamp, self.grid.frame_id)
                )
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "BEV comparison render failed: %s", exc)
            return

    def _build_rgb_bev(self, raw_frames, fused_stamp):
        missing = [view for view in self.views if not raw_frames.get(view)]
        if missing:
            raise ValueError("raw camera frames are missing: {}".format(missing))
        fused_stamp_ns = int(fused_stamp.to_nsec())
        if fused_stamp_ns <= 0:
            raise ValueError("fused BEV stamp is invalid")
        stamps_ns = [fused_stamp_ns]
        projected = {}
        size = (self.grid.width_px, self.grid.height_px)
        for view in self.views:
            valid_history = [
                entry for entry in raw_frames[view] if int(entry[1].to_nsec()) > 0
            ]
            if not valid_history:
                raise ValueError("{} raw camera history has no valid stamp".format(view))
            payload, stamp, _frame_id = min(
                valid_history,
                key=lambda entry: abs(int(entry[1].to_nsec()) - fused_stamp_ns),
            )
            stamp_ns = int(stamp.to_nsec())
            stamps_ns.append(stamp_ns)
            image = cv2.imdecode(
                np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_COLOR
            )
            camera = self.cameras[view]
            expected = (camera.height, camera.width, 3)
            if image is None or image.shape != expected:
                raise ValueError(
                    "{} raw camera image must be {}".format(view, expected)
                )
            projected[view] = cv2.warpPerspective(
                image,
                self.projectors[view].homography.bev_from_image,
                size,
                flags=cv2.INTER_LINEAR,
                borderMode=cv2.BORDER_CONSTANT,
                borderValue=(0, 0, 0),
            )
        if max(stamps_ns) - min(stamps_ns) > self.raw_comparison_max_skew_ns:
            raise ValueError(
                "raw/fused timestamp skew {:.1f}ms exceeds {:.1f}ms".format(
                    (max(stamps_ns) - min(stamps_ns)) / 1e6,
                    self.raw_comparison_max_skew_ns / 1e6,
                )
            )
        return fuse_projected_rgb(projected, self.rgb_quality)

    @staticmethod
    def _mono8_array(message: Image, expected_shape) -> np.ndarray:
        if message.encoding not in ("mono8", "8UC1"):
            raise ValueError("expected mono8, got {}".format(message.encoding))
        if (int(message.height), int(message.width)) != tuple(expected_shape):
            raise ValueError(
                "expected shape {}, got {}".format(
                    tuple(expected_shape), (int(message.height), int(message.width))
                )
            )
        data = np.frombuffer(bytes(message.data), dtype=np.uint8)
        required = int(message.height) * int(message.step)
        if message.step < message.width or data.size < required:
            raise ValueError("invalid image step/data length")
        return np.ascontiguousarray(
            data[:required].reshape(int(message.height), int(message.step))[
                :, : int(message.width)
            ]
        )

    def _compressed(
        self, image: np.ndarray, stamp, frame_id: str
    ) -> CompressedImage:
        ok, payload = cv2.imencode(
            ".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
        )
        if not ok:
            raise RuntimeError("JPEG encoding failed")
        message = CompressedImage()
        message.header.stamp = stamp
        message.header.frame_id = frame_id
        message.format = "jpeg"
        message.data = payload.tobytes()
        return message


def main() -> None:
    rospy.init_node("camera_bev_visualizer")
    CameraBevVisualizerNode()
    rospy.spin()


if __name__ == "__main__":
    main()
