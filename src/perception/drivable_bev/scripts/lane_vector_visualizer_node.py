#!/usr/bin/env python3
"""Lazy rqt debug rendering for metric lane evidence and fitted splines."""

from __future__ import annotations

import threading

import cv2
import message_filters
import numpy as np
import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import CompressedImage, Image, PointCloud2
from visualization_msgs.msg import Marker, MarkerArray

from drivable_bev.calibration import calibrations_from_dict
from drivable_bev.grid import BevGridSpec
from drivable_bev.visualization import (
    crop_bev_layers,
    draw_lane_reprojection,
    draw_metric_evidence,
    draw_metric_grid,
    draw_metric_lanes,
    render_fused_bev,
)


class LaneVectorVisualizerNode:
    def __init__(self):
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
        camera_values = dict(rospy.get_param("~cameras"))
        all_cameras = calibrations_from_dict(
            {
                "ground_plane": rospy.get_param("~ground_plane"),
                "cameras": camera_values,
            }
        )
        self.cameras = {
            name: camera
            for name, camera in all_cameras.items()
            if name in ("front", "left", "right")
        }
        self.view_order = ["left", "front", "right"]
        self.output_size = (
            int(rospy.get_param("~lane_debug_width", 672)),
            int(rospy.get_param("~lane_debug_height", 1050)),
        )
        self.display_label = str(rospy.get_param("~display_label", "FUSED"))
        self.jpeg_quality = int(rospy.get_param("~jpeg_quality", 85))
        self.lock = threading.Lock()
        self.layers = None
        self.evidence_xy = np.empty((0, 2), dtype=np.float32)
        self.evidence_class = np.empty(0, dtype=np.uint8)
        self.lanes = []
        self.boundaries = []
        self.images = {}
        self.latest_stamp = rospy.Time()

        base = "/perception/lane/debug"
        self.publishers = {
            "evidence": rospy.Publisher(
                base + "/evidence_bev/compressed", CompressedImage, queue_size=1
            ),
            "fitted": rospy.Publisher(
                base + "/fitted_bev/compressed", CompressedImage, queue_size=1
            ),
            "boundaries": rospy.Publisher(
                base + "/boundaries_bev/compressed", CompressedImage, queue_size=1
            ),
            "reprojection": rospy.Publisher(
                base + "/camera_reprojection_mosaic/compressed",
                CompressedImage,
                queue_size=1,
            ),
        }

        fused_base = "/perception/bev/camera"
        fused_topics = [
            fused_base + "/drivable_probability",
            fused_base + "/road_marking/class_id",
            fused_base + "/road_marking/confidence",
            fused_base + "/coverage",
            fused_base + "/source_count",
        ]
        fused_subscribers = [
            message_filters.Subscriber(topic, Image, queue_size=2)
            for topic in fused_topics
        ]
        fused_sync = message_filters.TimeSynchronizer(fused_subscribers, queue_size=4)
        fused_sync.registerCallback(self._fused_callback)
        self.subscribers = list(fused_subscribers)
        self.synchronizers = [fused_sync]
        # Evidence and spline subscriptions are connected only while an image
        # debug topic has a viewer. This keeps all publishers discoverable in
        # rqt without forcing PointCloud/Marker serialization in production.
        self.debug_subscribers = []
        for view in self.view_order:
            self.subscribers.append(
                rospy.Subscriber(
                    self.cameras[view].image_topic,
                    CompressedImage,
                    self._camera_callback,
                    callback_args=view,
                    queue_size=1,
                    buff_size=16 * 1024 * 1024,
                )
            )
        publish_hz = max(0.1, float(rospy.get_param("~lane_debug_max_hz", 5.0)))
        self.timer = rospy.Timer(rospy.Duration(1.0 / publish_hz), self._publish)
        rospy.loginfo(
            "lane vector visualizer ready display=x[%.1f,%.1f] y[%.1f,%.1f]",
            self.display_grid.x_min_m,
            self.display_grid.x_max_m,
            self.display_grid.y_min_m,
            self.display_grid.y_max_m,
        )

    def _fused_callback(self, drivable, marking, confidence, coverage, source_count):
        try:
            layers = {
                "drivable": self._mono8(drivable, self.grid.shape),
                "road_marking": self._mono8(marking, self.grid.shape),
                "confidence": self._mono8(confidence, self.grid.shape),
                "coverage": self._mono8(coverage, self.grid.shape),
                "source_count": self._mono8(source_count, self.grid.shape),
            }
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "lane visualizer rejected BEV: %s", exc)
            return
        with self.lock:
            self.layers = layers
            self.latest_stamp = drivable.header.stamp

    def _evidence_callback(self, message):
        try:
            rows = list(
                point_cloud2.read_points(
                    message, field_names=("x", "y", "class_id"), skip_nans=True
                )
            )
            xy = np.asarray([(row[0], row[1]) for row in rows], dtype=np.float32)
            if not len(rows):
                xy = np.empty((0, 2), dtype=np.float32)
            classes = np.asarray([row[2] for row in rows], dtype=np.uint8)
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "lane evidence visualization failed: %s", exc)
            return
        with self.lock:
            self.evidence_xy = xy
            self.evidence_class = classes
            self.latest_stamp = message.header.stamp

    def _marker_callback(self, message):
        lanes = []
        stamp = None
        for marker in message.markers:
            if marker.action == Marker.DELETEALL or marker.type != Marker.LINE_STRIP:
                continue
            class_id = (
                1
                if marker.ns.startswith("white")
                else 2
                if marker.ns.startswith("yellow")
                else 0
            )
            if not class_id or len(marker.points) < 2:
                continue
            lanes.append(
                (
                    class_id,
                    np.asarray([(point.x, point.y) for point in marker.points]),
                )
            )
            stamp = marker.header.stamp
        with self.lock:
            self.lanes = lanes
            if stamp is not None:
                self.latest_stamp = stamp

    def _boundary_marker_callback(self, message):
        lanes = []
        stamp = None
        for marker in message.markers:
            if marker.action == Marker.DELETEALL or marker.type != Marker.LINE_STRIP:
                continue
            class_id = (
                1
                if marker.ns.startswith("white")
                else 2
                if marker.ns.startswith("yellow")
                else 0
            )
            if not class_id or len(marker.points) < 2:
                continue
            lanes.append(
                (
                    class_id,
                    np.asarray([(point.x, point.y) for point in marker.points]),
                )
            )
            stamp = marker.header.stamp
        with self.lock:
            self.boundaries = lanes
            if stamp is not None:
                self.latest_stamp = stamp

    def _camera_callback(self, message, view):
        if self.publishers["reprojection"].get_num_connections() == 0:
            return
        image = cv2.imdecode(
            np.frombuffer(bytes(message.data), dtype=np.uint8), cv2.IMREAD_COLOR
        )
        if image is None:
            return
        with self.lock:
            self.images[view] = image

    def _publish(self, _event):
        requested = {
            name: publisher.get_num_connections() > 0
            for name, publisher in self.publishers.items()
        }
        self._update_debug_subscribers(any(requested.values()))
        if not any(requested.values()):
            return
        with self.lock:
            layers = None if self.layers is None else dict(self.layers)
            evidence_xy = self.evidence_xy.copy()
            evidence_class = self.evidence_class.copy()
            lanes = [(class_id, points.copy()) for class_id, points in self.lanes]
            boundaries = [
                (class_id, points.copy()) for class_id, points in self.boundaries
            ]
            images = {view: image.copy() for view, image in self.images.items()}
            stamp = self.latest_stamp
        try:
            if requested["evidence"]:
                evidence = self._base(layers)
                draw_metric_evidence(
                    evidence, evidence_xy, evidence_class, self.display_grid, radius=1
                )
                self.publishers["evidence"].publish(self._compressed(evidence, stamp))
            if requested["fitted"]:
                fitted = self._base(layers)
                draw_metric_lanes(fitted, lanes, self.display_grid, thickness=2)
                self.publishers["fitted"].publish(self._compressed(fitted, stamp))
            if requested["boundaries"]:
                boundary_image = self._base(layers)
                draw_metric_lanes(
                    boundary_image, boundaries, self.display_grid, thickness=3
                )
                self.publishers["boundaries"].publish(
                    self._compressed(boundary_image, stamp)
                )
            if requested["reprojection"] and all(
                view in images for view in self.view_order
            ):
                columns = []
                for view in self.view_order:
                    overlay = draw_lane_reprojection(
                        images[view], lanes, self.cameras[view], thickness=2
                    )
                    width = 480
                    height = max(1, int(round(overlay.shape[0] * width / overlay.shape[1])))
                    columns.append(cv2.resize(overlay, (width, height)))
                target_height = max(image.shape[0] for image in columns)
                padded = [
                    cv2.copyMakeBorder(
                        image,
                        0,
                        target_height - image.shape[0],
                        0,
                        0,
                        cv2.BORDER_CONSTANT,
                        value=(0, 0, 0),
                    )
                    for image in columns
                ]
                mosaic = np.concatenate(padded, axis=1)
                self.publishers["reprojection"].publish(
                    self._compressed(mosaic, stamp, resize=False)
                )
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "lane debug render failed: %s", exc)

    def _update_debug_subscribers(self, needed):
        if needed and not self.debug_subscribers:
            self.debug_subscribers = [
                rospy.Subscriber(
                    "/perception/lane/debug/evidence_points",
                    PointCloud2,
                    self._evidence_callback,
                    queue_size=1,
                ),
                rospy.Subscriber(
                    "/perception/lane/debug/markers",
                    MarkerArray,
                    self._marker_callback,
                    queue_size=1,
                ),
                rospy.Subscriber(
                    "/perception/lane/boundaries",
                    MarkerArray,
                    self._boundary_marker_callback,
                    queue_size=1,
                ),
            ]
        elif not needed and self.debug_subscribers:
            for subscriber in self.debug_subscribers:
                subscriber.unregister()
            self.debug_subscribers = []

    def _base(self, layers):
        if layers is None:
            canvas = np.zeros((*self.display_grid.shape, 3), dtype=np.uint8)
            draw_metric_grid(canvas, self.display_grid, 5.0, 2.0)
        else:
            value = crop_bev_layers(layers, self.grid, self.display_grid)
            canvas = render_fused_bev(
                value["drivable"],
                value["road_marking"],
                value["confidence"],
                value["coverage"],
                value["source_count"],
                self.display_grid,
                label_prefix=self.display_label,
            )
        return canvas

    def _compressed(self, image, stamp, resize=True):
        if resize and image.shape[1] != self.output_size[0]:
            image = cv2.resize(image, self.output_size, interpolation=cv2.INTER_NEAREST)
        ok, encoded = cv2.imencode(
            ".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
        )
        if not ok:
            raise RuntimeError("lane debug JPEG encoding failed")
        message = CompressedImage()
        message.header.stamp = stamp
        message.header.frame_id = self.grid.frame_id
        message.format = "jpeg"
        message.data = encoded.tobytes()
        return message

    @staticmethod
    def _mono8(message, shape):
        if message.encoding not in ("mono8", "8UC1"):
            raise ValueError("expected mono8")
        if (int(message.height), int(message.width)) != tuple(shape):
            raise ValueError("BEV image shape differs from grid")
        data = np.frombuffer(bytes(message.data), dtype=np.uint8)
        required = int(message.height) * int(message.step)
        if int(message.step) < int(message.width) or data.size < required:
            raise ValueError("BEV image is incomplete")
        return np.ascontiguousarray(
            data[:required].reshape(int(message.height), int(message.step))[
                :, : int(message.width)
            ]
        )


def main():
    rospy.init_node("camera_lane_vector_visualizer")
    LaneVectorVisualizerNode()
    rospy.spin()


if __name__ == "__main__":
    main()
