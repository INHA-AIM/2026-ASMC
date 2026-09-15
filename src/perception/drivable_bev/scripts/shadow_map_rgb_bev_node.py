#!/usr/bin/env python3
"""Fuse three raw cameras in metric BEV and overlay shadow-zone map pseudo-GT."""

from __future__ import annotations

from collections import deque
import hashlib
import json
from pathlib import Path
import threading
import time

import cv2
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import message_filters
from morai_msgs.msg import EgoVehicleStatus
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage

from drivable_bev.calibration import calibrations_from_dict
from drivable_bev.fusion import build_quality_maps, timestamp_span_ns
from drivable_bev.grid import BevGridSpec
from drivable_bev.projector import CameraBevProjector
from drivable_bev.rgb_bev import (
    StampedEgoPose,
    fuse_rgb_views,
    nearest_ego_pose,
    overlay_shadow_map_boundaries,
    warp_rgb_to_bev,
)
from drivable_bev.shadow_map_gt import map_xy_to_base


VIEWS = ("front", "left", "right")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_boundaries(path: Path):
    with path.open("r", encoding="utf-8") as stream:
        value = json.load(stream)
    if value.get("schema_version") != "asmc-shadow-lane-pseudo-gt-1.0.0":
        raise ValueError("unsupported shadow pseudo-GT schema")
    expected = {
        "white_negative_normal",
        "yellow_center",
        "white_positive_normal",
    }
    values = value.get("boundaries") or {}
    if set(values) != expected:
        raise ValueError("shadow pseudo-GT must contain exactly three boundaries")
    boundaries = {}
    for name, points in values.items():
        array = np.asarray(points, dtype=np.float64)
        if array.ndim != 2 or array.shape[1] < 2 or len(array) < 2:
            raise ValueError("{} is not a valid map polyline".format(name))
        if not np.isfinite(array).all():
            raise ValueError("{} contains non-finite map coordinates".format(name))
        boundaries[name] = np.ascontiguousarray(array[:, :2])
    return boundaries, value


class ShadowMapRgbBevNode:
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
        all_cameras = calibrations_from_dict(
            {
                "ground_plane": rospy.get_param("~ground_plane"),
                "cameras": rospy.get_param("~cameras"),
            }
        )
        self.cameras = {name: all_cameras[name] for name in VIEWS}
        max_range = float(rospy.get_param("~max_ground_range_m", 30.0))
        min_depth = float(rospy.get_param("~min_camera_depth_m", 0.1))
        self.projectors = {
            name: CameraBevProjector(camera, self.grid, max_range, min_depth)
            for name, camera in self.cameras.items()
        }
        self.quality = build_quality_maps(
            self.projectors,
            self.grid,
            forward_fade_start_m=min(20.0, self.grid.x_max_m - 1.0),
            lateral_fade_start_m=min(6.0, self.grid.y_max_m - 0.5),
        )

        pseudo_gt_path = Path(rospy.get_param("~pseudo_gt_path")).resolve()
        if not pseudo_gt_path.is_file():
            raise ValueError("pseudo-GT does not exist: {}".format(pseudo_gt_path))
        self.boundaries_map, pseudo_document = _load_boundaries(pseudo_gt_path)
        self.pseudo_gt_path = pseudo_gt_path
        self.pseudo_gt_hash = _sha256(pseudo_gt_path)
        self.pseudo_gt_status = str(pseudo_document.get("status", "unknown"))
        self.deployment_allowed = bool(pseudo_document.get("deployment_allowed", False))
        pose_alignment = dict(pseudo_document.get("pose_alignment") or {})
        self.ego_heading_offset_deg = float(
            pose_alignment.get("ego_heading_offset_deg", 0.0)
        )
        if (
            not np.isfinite(self.ego_heading_offset_deg)
            or abs(self.ego_heading_offset_deg) > 5.0
        ):
            raise ValueError("pseudo-GT ego heading offset is invalid")

        self.max_skew_ns = int(
            round(float(rospy.get_param("~camera_sync_slop_ms", 35.0)) * 1e6)
        )
        self.max_ego_age_ns = int(
            round(float(rospy.get_param("~ego_pose_max_age_ms", 40.0)) * 1e6)
        )
        max_hz = float(rospy.get_param("~max_publish_hz", 10.0))
        if max_hz <= 0.0:
            raise ValueError("max_publish_hz must be positive")
        self.min_period_ns = int(round(1e9 / max_hz))
        self.jpeg_quality = int(rospy.get_param("~jpeg_quality", 90))
        self.overlay_alpha = float(rospy.get_param("~overlay_alpha", 0.85))
        if not 1 <= self.jpeg_quality <= 100:
            raise ValueError("jpeg_quality must lie in [1,100]")

        self.raw_publisher = rospy.Publisher(
            str(rospy.get_param("~raw_topic")), CompressedImage, queue_size=1
        )
        self.overlay_publisher = rospy.Publisher(
            str(rospy.get_param("~overlay_topic")), CompressedImage, queue_size=1
        )
        self.diagnostics_publisher = rospy.Publisher(
            str(rospy.get_param("~diagnostics_topic")), DiagnosticArray, queue_size=1
        )

        self.pose_lock = threading.Lock()
        self.poses = deque(maxlen=int(rospy.get_param("~ego_buffer_size", 250)))
        self.last_output_stamp_ns = 0
        self.counters = {
            "input_triplets": 0,
            "outputs_raw": 0,
            "outputs_overlay": 0,
            "throttle_drops": 0,
            "camera_skew_drops": 0,
            "ego_age_drops": 0,
            "decode_errors": 0,
            "projection_errors": 0,
        }
        self.last_metrics = {
            "camera_skew_ms": float("nan"),
            "ego_delta_ms": float("nan"),
            "processing_ms": float("nan"),
            "coverage_fraction": 0.0,
        }

        self.ego_subscriber = rospy.Subscriber(
            str(rospy.get_param("~ego_topic", "/Ego_topic")),
            EgoVehicleStatus,
            self._on_ego,
            queue_size=100,
        )
        self.camera_subscribers = [
            message_filters.Subscriber(
                self.cameras[view].image_topic,
                CompressedImage,
                queue_size=3,
                buff_size=16 * 1024 * 1024,
            )
            for view in VIEWS
        ]
        self.camera_sync = message_filters.ApproximateTimeSynchronizer(
            self.camera_subscribers,
            queue_size=int(rospy.get_param("~sync_queue_size", 12)),
            slop=float(self.max_skew_ns) / 1e9,
            allow_headerless=False,
        )
        self.camera_sync.registerCallback(self._on_cameras)
        interval = float(rospy.get_param("~diagnostics_interval_sec", 2.0))
        self.diagnostics_timer = rospy.Timer(
            rospy.Duration(max(0.2, interval)), self._publish_diagnostics
        )
        rospy.logwarn(
            "shadow map RGB BEV is review-only: pseudo_gt=%s deployment_allowed=%s",
            self.pseudo_gt_status,
            self.deployment_allowed,
        )
        rospy.loginfo(
            "shadow map RGB BEV ready grid=x[%.1f,%.1f] y[%.1f,%.1f] %.2fm "
            "topics=(%s, %s)",
            self.grid.x_min_m,
            self.grid.x_max_m,
            self.grid.y_min_m,
            self.grid.y_max_m,
            self.grid.resolution_m,
            self.raw_publisher.resolved_name,
            self.overlay_publisher.resolved_name,
        )

    @staticmethod
    def _message_stamp_ns(message) -> int:
        stamp_ns = int(message.header.stamp.to_nsec())
        return stamp_ns if stamp_ns > 0 else int(rospy.Time.now().to_nsec())

    def _on_ego(self, message: EgoVehicleStatus) -> None:
        try:
            pose = StampedEgoPose(
                stamp_ns=self._message_stamp_ns(message),
                x_m=float(message.position.x),
                y_m=float(message.position.y),
                heading_deg=float(message.heading),
            )
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "invalid Ego pose for map overlay: %s", exc)
            return
        with self.pose_lock:
            self.poses.append(pose)

    def _decode(self, message, view):
        image = cv2.imdecode(
            np.frombuffer(bytes(message.data), dtype=np.uint8), cv2.IMREAD_COLOR
        )
        expected = (self.cameras[view].height, self.cameras[view].width)
        if image is None or image.shape[:2] != expected:
            raise ValueError(
                "{} decode shape {} does not match {}".format(
                    view, None if image is None else image.shape[:2], expected
                )
            )
        return image

    def _on_cameras(self, front, left, right) -> None:
        self.counters["input_triplets"] += 1
        if (
            self.raw_publisher.get_num_connections() == 0
            and self.overlay_publisher.get_num_connections() == 0
        ):
            return
        messages = {"front": front, "left": left, "right": right}
        stamps_ns = [self._message_stamp_ns(messages[name]) for name in VIEWS]
        output_stamp_ns = max(stamps_ns)
        skew_ns = timestamp_span_ns(stamps_ns)
        self.last_metrics["camera_skew_ms"] = skew_ns / 1e6
        if skew_ns > self.max_skew_ns:
            self.counters["camera_skew_drops"] += 1
            return
        if (
            self.last_output_stamp_ns > 0
            and 0 <= output_stamp_ns - self.last_output_stamp_ns < self.min_period_ns
        ):
            self.counters["throttle_drops"] += 1
            return

        started = time.perf_counter()
        try:
            decoded = {view: self._decode(messages[view], view) for view in VIEWS}
        except Exception as exc:
            self.counters["decode_errors"] += 1
            rospy.logwarn_throttle(2.0, "raw camera BEV decode failed: %s", exc)
            return
        try:
            warped = {
                view: warp_rgb_to_bev(
                    decoded[view],
                    self.projectors[view].homography.bev_from_image,
                    self.grid.shape,
                )
                for view in VIEWS
            }
            fused, source_count = fuse_rgb_views(warped, self.quality)
        except Exception as exc:
            self.counters["projection_errors"] += 1
            rospy.logwarn_throttle(2.0, "raw camera BEV projection failed: %s", exc)
            return

        output_stamp = rospy.Time(
            secs=output_stamp_ns // 1_000_000_000,
            nsecs=output_stamp_ns % 1_000_000_000,
        )
        if self.raw_publisher.get_num_connections() > 0:
            self.raw_publisher.publish(self._compressed(fused, output_stamp))
            self.counters["outputs_raw"] += 1

        if self.overlay_publisher.get_num_connections() > 0:
            target_stamp_ns = sorted(stamps_ns)[1]
            with self.pose_lock:
                poses = tuple(self.poses)
            pose, delta_ns = nearest_ego_pose(poses, target_stamp_ns, self.max_ego_age_ns)
            self.last_metrics["ego_delta_ms"] = (
                float("nan") if delta_ns is None else delta_ns / 1e6
            )
            if pose is None:
                self.counters["ego_age_drops"] += 1
                rospy.logwarn_throttle(
                    2.0,
                    "map overlay waiting for time-aligned Ego pose (nearest delta=%.1fms)",
                    float("nan") if delta_ns is None else delta_ns / 1e6,
                )
            else:
                boundaries_base = {
                    name: map_xy_to_base(
                        points,
                        (pose.x_m, pose.y_m),
                        pose.heading_deg + self.ego_heading_offset_deg,
                    )
                    for name, points in self.boundaries_map.items()
                }
                overlay = overlay_shadow_map_boundaries(
                    fused, boundaries_base, self.grid, self.overlay_alpha
                )
                self.overlay_publisher.publish(self._compressed(overlay, output_stamp))
                self.counters["outputs_overlay"] += 1

        self.last_output_stamp_ns = output_stamp_ns
        self.last_metrics["processing_ms"] = (time.perf_counter() - started) * 1000.0
        self.last_metrics["coverage_fraction"] = float(np.mean(source_count > 0))

    def _compressed(self, image, stamp):
        ok, payload = cv2.imencode(
            ".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
        )
        if not ok:
            raise RuntimeError("JPEG encoding failed")
        message = CompressedImage()
        message.header.stamp = stamp
        message.header.frame_id = self.grid.frame_id
        message.format = "jpeg"
        message.data = payload.tobytes()
        return message

    def _publish_diagnostics(self, _event) -> None:
        status = DiagnosticStatus()
        status.name = "drivable_bev/shadow_map_rgb_review"
        status.hardware_id = "three_camera_map_pseudo_gt"
        if self.counters["outputs_overlay"] == 0:
            status.level = DiagnosticStatus.WARN
            status.message = "waiting for rqt subscriber or aligned input"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "review-only RGB BEV and map overlay active"
        values = {
            **self.counters,
            **{
                name: "{:.3f}".format(value)
                for name, value in self.last_metrics.items()
            },
            "grid": "x[{:.1f},{:.1f}] y[{:.1f},{:.1f}] {:.2f}m/cell".format(
                self.grid.x_min_m,
                self.grid.x_max_m,
                self.grid.y_min_m,
                self.grid.y_max_m,
                self.grid.resolution_m,
            ),
            "pseudo_gt_path": str(self.pseudo_gt_path),
            "pseudo_gt_sha256": self.pseudo_gt_hash,
            "pseudo_gt_status": self.pseudo_gt_status,
            "deployment_allowed": str(self.deployment_allowed).lower(),
            "ego_heading_offset_deg": "{:.3f}".format(
                self.ego_heading_offset_deg
            ),
        }
        status.values = [
            KeyValue(key=str(key), value=str(value)) for key, value in values.items()
        ]
        message = DiagnosticArray()
        message.header.stamp = rospy.Time.now()
        message.status = [status]
        self.diagnostics_publisher.publish(message)


def main() -> None:
    rospy.init_node("shadow_map_rgb_bev")
    ShadowMapRgbBevNode()
    rospy.spin()


if __name__ == "__main__":
    main()
