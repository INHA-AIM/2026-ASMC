#!/usr/bin/env python3
"""Publish source-separated HD-map development layers around Ego."""

from __future__ import annotations

from collections import deque
import json
import math
from pathlib import Path
import threading
import time

from geometry_msgs.msg import Point
from morai_msgs.msg import EgoVehicleStatus, GPSMessage
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from asmc_hd_map.live_pose import projected_gps_to_base_link, quaternion_yaw_rad
from asmc_hd_map.map_overlay import (
    SegmentLayer,
    dashed_polyline_segments,
    lane_shape_group,
    feature_paint_segments,
)
from asmc_hd_map.review_bundle import load_review_layers


def _point(x: float, y: float, z: float = 0.04) -> Point:
    value = Point()
    value.x, value.y, value.z = float(x), float(y), float(z)
    return value


def _delete_all(stamp) -> Marker:
    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.header.stamp = stamp
    marker.action = Marker.DELETEALL
    return marker


def _line_marker(namespace: str, marker_id: int, segments: np.ndarray, stamp, width: float, color) -> Marker:
    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.header.stamp = stamp
    marker.ns = namespace
    marker.id = marker_id
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = width
    marker.color = ColorRGBA(*color)
    for segment in segments:
        marker.points.append(_point(segment[0, 0], segment[0, 1]))
        marker.points.append(_point(segment[1, 0], segment[1, 1]))
    return marker


def _as_layer(polylines) -> SegmentLayer:
    return SegmentLayer.from_features([
        {"points": np.asarray(points, dtype=np.float64).tolist()}
        for points in polylines
    ])


def _canonical_coordinates(features):
    return [(feature.get("geometry") or {}).get("coordinates", ()) for feature in features]


def _canonical_lane_layer(features, shape: str) -> SegmentLayer:
    selected = [feature for feature in features if lane_shape_group(feature) == shape]
    if shape == "solid":
        return _as_layer(_canonical_coordinates(selected))
    segments = []
    for feature in selected:
        value = feature_paint_segments(feature)
        if len(value):
            segments.append(value)
    return SegmentLayer(
        np.concatenate(segments, axis=0)
        if segments else np.zeros((0, 2, 2), dtype=np.float64)
    )


def _won_tae_h5_vectors(path: Path):
    """Load only Won-tae's stored vector observations, never a rebuilt map."""

    try:
        import h5py
    except ImportError as error:
        raise RuntimeError("three-source comparison requires h5py") from error
    lanes = []
    stoplines = []
    with h5py.File(str(path), "r") as handle:
        if "observed_lane_points" in handle and "observed_lane_offsets" in handle:
            points = np.asarray(handle["observed_lane_points"], dtype=np.float64)
            offsets = np.asarray(handle["observed_lane_offsets"], dtype=np.int64)
            colors = json.loads(handle.attrs.get("observed_lane_colors", "[]"))
            for index, (start, end) in enumerate(zip(offsets[:-1], offsets[1:])):
                value = points[int(start):int(end)]
                if len(value) < 2:
                    continue
                lanes.append({
                    "points": value,
                    "color": str(colors[index]).lower() if index < len(colors) else "unknown",
                })
        required = {"stopline_points", "stopline_dirs", "stopline_lengths"}
        if required.issubset(handle.keys()):
            centers = np.asarray(handle["stopline_points"], dtype=np.float64)
            directions = np.asarray(handle["stopline_dirs"], dtype=np.float64)
            lengths = np.asarray(handle["stopline_lengths"], dtype=np.float64)
            for center, direction, length in zip(centers, directions, lengths):
                norm = float(np.linalg.norm(direction))
                if norm <= 1e-9 or not np.isfinite([*center, *direction, length]).all():
                    continue
                half = 0.5 * float(length) * direction / norm
                stoplines.append({"points": np.stack((center - half, center + half))})
    return lanes, stoplines


def _npz_drivable_grid(path: Path):
    with np.load(str(path), allow_pickle=False) as archive:
        occupancy = np.asarray(archive["occupancy"], dtype=np.int8)
        resolution = float(np.asarray(archive["resolution_m"]).reshape(-1)[0])
        origin = np.asarray(archive["origin_xy"], dtype=np.float64).reshape(-1)[:2]
    return _validated_drivable_grid(occupancy, resolution, origin, "latest compact map")


def _won_tae_h5_drivable_grid(path: Path):
    try:
        import h5py
    except ImportError as error:
        raise RuntimeError("HD-map source comparison requires h5py") from error
    with h5py.File(str(path), "r") as handle:
        road = np.asarray(handle["road"], dtype=np.uint8)
        occupancy = np.where(road > 0, 0, 100).astype(np.int8)
        resolution = 1.0 / float(handle.attrs["pixels_per_meter"])
        origin = np.asarray(
            handle.attrs["world_offset_in_meters"], dtype=np.float64,
        ).reshape(-1)[:2]
    return _validated_drivable_grid(occupancy, resolution, origin, "legacy H5 bake")


def _validated_drivable_grid(occupancy, resolution: float, origin, label: str):
    values = np.asarray(occupancy, dtype=np.int8)
    origin_xy = np.asarray(origin, dtype=np.float64)
    if values.ndim != 2 or not values.size:
        raise ValueError("%s occupancy must be a non-empty 2-D array" % label)
    if resolution <= 0.0 or origin_xy.shape != (2,) or not np.isfinite(origin_xy).all():
        raise ValueError("%s raster metadata is invalid" % label)
    if not set(np.unique(values)).issubset({-1, 0, 100}):
        raise ValueError("%s occupancy must use -1/0/100" % label)
    return {
        "occupancy": np.ascontiguousarray(values),
        "resolution_m": float(resolution),
        "origin_xy": origin_xy,
        "label": label,
    }


def _local_drivable_grid(source, pose, radius_m: float, resolution_m: float) -> np.ndarray:
    """Sample a map raster into an Ego-centered base_link OccupancyGrid."""

    size = int(math.ceil(2.0 * float(radius_m) / float(resolution_m)))
    extent = size * float(resolution_m)
    coordinates = -0.5 * extent + (np.arange(size, dtype=np.float64) + 0.5) * resolution_m
    local_x, local_y = np.meshgrid(coordinates, coordinates)
    heading = math.radians(float(pose[2]))
    cosine, sine = math.cos(heading), math.sin(heading)
    map_x = float(pose[0]) + cosine * local_x - sine * local_y
    map_y = float(pose[1]) + sine * local_x + cosine * local_y
    source_resolution = float(source["resolution_m"])
    source_origin = source["origin_xy"]
    pixel_x = np.rint((map_x - source_origin[0]) / source_resolution).astype(np.int64)
    pixel_y = np.rint((map_y - source_origin[1]) / source_resolution).astype(np.int64)
    occupancy = source["occupancy"]
    inside = (
        (pixel_x >= 0) & (pixel_x < occupancy.shape[1])
        & (pixel_y >= 0) & (pixel_y < occupancy.shape[0])
    )
    local = np.full((size, size), -1, dtype=np.int8)
    local[inside] = occupancy[pixel_y[inside], pixel_x[inside]]
    return local


class CandidateMapOverlayNode:
    def __init__(self) -> None:
        config_path = Path(rospy.get_param("~config_path")).resolve()
        configured_root = str(rospy.get_param("~repo_root", "")).strip()
        if configured_root:
            repo_root = Path(configured_root).resolve()
        else:
            repo_root = next(
                (parent for parent in config_path.parents if (parent / "src").is_dir()),
                None,
            )
            if repo_root is None:
                raise ValueError("repository root was not found; set ~repo_root")
        review = load_review_layers(config_path, repo_root)
        self.comparison_mode = bool(rospy.get_param("~comparison_mode", False))
        self.compare_drivable_sources = bool(
            rospy.get_param("~compare_drivable_sources", False)
        )
        self.curation_only = bool(rospy.get_param("~curation_only", False))
        self.topic_root = "/" + str(
            rospy.get_param("~topic_root", "/asmc_hd_map")
        ).strip("/")
        if self.curation_only and not self.comparison_mode:
            raise ValueError("curation_only requires comparison_mode")
        self.radius_m = float(rospy.get_param("~local_radius_m", 80.0))
        self.max_hz = float(rospy.get_param("~max_publish_hz", 5.0))
        if self.radius_m <= 0.0 or self.max_hz <= 0.0:
            raise ValueError("local_radius_m and max_publish_hz must be positive")
        self.minimum_period = 1.0 / self.max_hz
        self.last_publish = 0.0
        self.pose_source = str(rospy.get_param("~pose_source", "gps_imu")).strip().lower()
        if self.pose_source not in {"gps_imu", "ego_status"}:
            raise ValueError("pose_source must be gps_imu or ego_status")
        self._first_pose_logged = False
        self.pose_distance_warning_m = float(
            rospy.get_param("~pose_distance_warning_m", 20.0)
        )
        if self.pose_distance_warning_m <= 0.0:
            raise ValueError("pose_distance_warning_m must be positive")

        self.layers = {
            "current_links": _as_layer(_canonical_coordinates(review["current_links"])),
            "nominal": _as_layer(_canonical_coordinates(review["nominal_boundaries"])),
            "stoplines": _as_layer(_canonical_coordinates(review["stoplines"])),
            "route": _as_layer([review["route"]]),
        }
        self.calibrated = {}
        for color in ("white", "yellow", "unknown"):
            self.calibrated[color] = _as_layer(_canonical_coordinates([
                feature for feature in review["observed_boundaries"]
                if str(feature.get("attributes", {}).get("lane_color", "unknown")).lower() == color
            ]))
        self.adopted = {}
        for color in ("white", "yellow", "unknown"):
            color_features = [
                feature for feature in review["adopted_legacy_boundaries"]
                if str(
                    feature.get("attributes", {}).get(
                        "color", feature.get("attributes", {}).get("lane_color", "unknown")
                    )
                ).lower() == color
            ]
            for shape in ("solid", "broken"):
                self.adopted[(color, shape)] = _canonical_lane_layer(color_features, shape)
        self.changed_legacy = _as_layer(_canonical_coordinates(review["changed_legacy_candidates"]))
        self.legacy = {}
        for tier in ("A", "B"):
            for color in ("white", "yellow", "blue", "unknown"):
                self.legacy[(tier, color)] = _as_layer([
                    feature["coordinates"] for feature in review["legacy_references"]
                    if feature["tier"] == tier and feature["color"] == color
                ])
        self.observations = {
            status: _as_layer([
                segment["points"] for segment in review["observation_segments"]
                if segment["review_status"] == status
            ])
            for status in ("automatic", "review_required", "rejected", "unmatched")
        }
        if self.comparison_mode:
            preplan_features = review["adopted_legacy_boundaries"]
            self.compare_preplan = {}
            for color in ("white", "yellow", "blue", "unknown"):
                color_features = [
                    feature for feature in preplan_features
                    if str((feature.get("attributes") or {}).get(
                        "color",
                        (feature.get("attributes") or {}).get("lane_color", "unknown"),
                    )).lower() == color
                ]
                for shape in ("solid", "broken"):
                    self.compare_preplan[(color, shape)] = _canonical_lane_layer(
                        color_features, shape,
                    )
            legacy_lane_features = [
                feature for feature in review["legacy_references"]
                if feature.get("lane_type") != "530"
            ]
            self.compare_legacy = {}
            for color in ("white", "yellow", "blue", "unknown"):
                color_features = [
                    feature for feature in legacy_lane_features
                    if feature["color"] == color
                ]
                self.compare_legacy[(color, "solid")] = _as_layer([
                    feature["coordinates"] for feature in color_features
                    if "broken" not in feature["shape"].lower()
                ])
                self.compare_legacy[(color, "broken")] = SegmentLayer(
                    dashed_polyline_segments(
                        [
                            feature["coordinates"] for feature in color_features
                            if "broken" in feature["shape"].lower()
                        ],
                        3.0, 3.0,
                    )
                )
            self.compare_legacy_stoplines = _as_layer([
                feature["coordinates"] for feature in review["legacy_references"]
                if feature.get("lane_type") == "530"
            ])
            h5_path = review["source_paths"].get("won_tae_h5")
            if h5_path is None:
                raise ValueError("comparison config requires sources.won_tae_h5")
            if self.compare_drivable_sources:
                compact_grid_path = review["source_paths"].get("compact_drivable_grid")
                if compact_grid_path is None:
                    raise ValueError(
                        "drivable source comparison requires sources.compact_drivable_grid"
                    )
                self.compare_latest_drivable = _npz_drivable_grid(compact_grid_path)
                self.compare_legacy_drivable = _won_tae_h5_drivable_grid(h5_path)
                self.drivable_resolution_m = float(
                    rospy.get_param("~drivable_resolution_m", 0.20)
                )
                self.drivable_max_hz = float(
                    rospy.get_param("~drivable_max_publish_hz", 1.0)
                )
                if self.drivable_resolution_m <= 0.0 or self.drivable_max_hz <= 0.0:
                    raise ValueError(
                        "drivable_resolution_m and drivable_max_publish_hz must be positive"
                    )
                self.drivable_minimum_period = 1.0 / self.drivable_max_hz
                self.drivable_last_publish = 0.0
            won_tae_lanes, won_tae_stoplines = _won_tae_h5_vectors(h5_path)
            self.compare_won_tae = {
                (color, "solid"): _as_layer([
                    feature["points"] for feature in won_tae_lanes
                    if feature["color"] == color
                ])
                for color in ("white", "yellow", "blue", "unknown")
            }
            self.compare_won_tae_stoplines = _as_layer([
                feature["points"] for feature in won_tae_stoplines
            ])
            self.compare_preplan_stoplines = self.layers["stoplines"]
            curated_path = review["source_paths"].get("curated_candidate_map")
            if curated_path is not None:
                curated_document = json.loads(curated_path.read_text(encoding="utf-8"))
                curated_features = [
                    feature for feature in curated_document.get("features", [])
                    if feature.get("feature_type") in {"lane_marking", "lane_boundary"}
                ]
                self.curated_feature_count = len(curated_features)
                self.curated = {}
                for color in ("white", "yellow", "blue", "unknown"):
                    color_features = [
                        feature for feature in curated_features
                        if str((feature.get("attributes") or {}).get("color", "unknown")).lower()
                        == color
                    ]
                    for shape in ("solid", "broken"):
                        self.curated[(color, shape)] = _canonical_lane_layer(
                            color_features, shape,
                        )
                self.curated_stoplines = _as_layer(_canonical_coordinates([
                    feature for feature in curated_document.get("features", [])
                    if feature.get("feature_type") == "stopline"
                ]))
            elif self.curation_only:
                raise ValueError("curation_only requires sources.curated_candidate_map")
            self.comparison_primary = str(
                rospy.get_param("~comparison_primary", "preplan")
            ).strip().lower()
            if self.comparison_primary not in {"preplan", "curated"}:
                raise ValueError("comparison_primary must be preplan or curated")
            if self.comparison_primary == "curated" and not hasattr(self, "curated"):
                raise ValueError(
                    "comparison_primary=curated requires sources.curated_candidate_map"
                )
        self.high_speed_labels = []
        for feature in review["current_links"]:
            if "high_speed_parallel" not in feature.get("attributes", {}).get("corridor_roles", []):
                continue
            coordinates = np.asarray(feature["geometry"]["coordinates"], dtype=np.float64)
            self.high_speed_labels.append({
                "link_id": str(feature["topology"]["current_link_ids"][0]),
                "ego_lane": feature.get("attributes", {}).get("ego_lane"),
                "xy": coordinates[len(coordinates) // 2, :2],
            })

        topic_names = {
            "current_links": self.topic_root + "/review/current_link_centers",
            "nominal": self.topic_root + "/review/nominal_boundaries",
            "calibrated": self.topic_root + "/review/observed_calibrated_boundaries",
            # Topic is kept for compatibility; it now contains the reviewed
            # hybrid candidate (stable legacy + current reconstructed gaps).
            "adopted": self.topic_root + "/review/adopted_legacy_boundaries",
            "changed_legacy": self.topic_root + "/review/changed_region_legacy_candidates",
            "legacy": self.topic_root + "/review/legacy_ab_reference_lanes",
            "observations": self.topic_root + "/review/source_observations",
            "stoplines": self.topic_root + "/review/stopline_candidates",
            "route": self.topic_root + "/review/competition_route",
            "labels": self.topic_root + "/review/high_speed_link_labels",
            "ego": self.topic_root + "/review/ego_context",
        }
        if self.curation_only:
            topic_names = {
                "curated": self.topic_root + "/curation/selected_candidate",
                "ego": self.topic_root + "/curation/ego_context",
            }
        elif self.comparison_mode:
            if self.comparison_primary == "curated":
                topic_names = {
                    "compare_primary": self.topic_root + "/hd_map_sources/lanes/final_curated",
                    "compare_won_tae": self.topic_root + "/hd_map_sources/lanes/won_tae_h5",
                    "compare_legacy": self.topic_root + "/hd_map_sources/lanes/legacy_origin_aligned",
                    "ego": self.topic_root + "/hd_map_sources/ego_context",
                }
                if self.compare_drivable_sources:
                    topic_names.update({
                        "compare_latest_drivable": (
                            self.topic_root + "/hd_map_sources/drivable/latest"
                        ),
                        "compare_legacy_drivable": (
                            self.topic_root + "/hd_map_sources/drivable/legacy_h5_bake"
                        ),
                    })
            else:
                topic_names = {
                    "compare_primary": self.topic_root + "/compare/preplan_hybrid",
                    "compare_won_tae": self.topic_root + "/compare/won_tae_bake",
                    "compare_legacy": self.topic_root + "/compare/legacy_origin_aligned",
                    "ego": self.topic_root + "/compare/ego_context",
                }
        self.publishers = {}
        for key, topic in topic_names.items():
            message_type = (
                OccupancyGrid if key in {
                    "compare_latest_drivable", "compare_legacy_drivable"
                } else MarkerArray
            )
            self.publishers[key] = rospy.Publisher(
                topic, message_type, queue_size=1, latch=False,
            )
        self.subscribers = []
        if self.pose_source == "gps_imu":
            try:
                from pyproj import Transformer
            except ImportError as error:
                raise RuntimeError("gps_imu pose source requires pyproj") from error
            frame = review["observed_document"].get("frame") or {}
            origin = frame.get("local_origin_in_global") or []
            if len(origin) != 3 or not np.isfinite(np.asarray(origin, dtype=np.float64)).all():
                raise ValueError("candidate map requires a finite three-element global origin")
            self.map_origin_xy = np.asarray(origin[:2], dtype=np.float64)
            self.gps_lever_arm_xy = np.asarray([
                float(rospy.get_param("~gps_lever_arm_x_m", 0.35)),
                float(rospy.get_param("~gps_lever_arm_y_m", 0.0)),
            ])
            self.maximum_imu_gap_sec = float(
                rospy.get_param("~maximum_gps_imu_gap_ms", 100.0)
            ) / 1000.0
            if self.maximum_imu_gap_sec <= 0.0:
                raise ValueError("maximum_gps_imu_gap_ms must be positive")
            utm_epsg = int(rospy.get_param("~utm_epsg", 32652))
            self.utm_transformer = Transformer.from_crs(
                "EPSG:4326", f"EPSG:{utm_epsg}", always_xy=True,
            )
            self.imu_samples = deque(maxlen=200)
            self.pose_lock = threading.Lock()
            self._offset_warning_emitted = False
            self.subscribers.extend([
                rospy.Subscriber(
                    str(rospy.get_param("~imu_topic", "/imu")),
                    Imu, self._on_imu, queue_size=10,
                ),
                rospy.Subscriber(
                    str(rospy.get_param("~gps_topic", "/gps")),
                    GPSMessage, self._on_gps, queue_size=2,
                ),
            ])
        else:
            self.subscribers.append(rospy.Subscriber(
                str(rospy.get_param("~ego_topic", "/Ego_topic")),
                EgoVehicleStatus, self._on_ego, queue_size=1,
            ))
        if self.curation_only:
            rospy.logwarn(
                "route curation overlay is PARTIAL/REVIEW ONLY: only explicitly implemented "
                "segments are published; missing geometry is intentional"
            )
        elif self.comparison_mode:
            rospy.logwarn(
                "three-source HD-map comparison is REVIEW ONLY: %s, immutable Won-tae H5 "
                "vectors, and origin-only corridor legacy are never merged",
                "final curated markings"
                if self.comparison_primary == "curated" else "pre-plan hybrid",
            )
        else:
            rospy.logwarn(
                "candidate HD-map overlay is REVIEW ONLY: operational=false; selected physical "
                "markings require final driving review; observed/raw and rejected legacy layers "
                "are validation references"
            )
        if self.curation_only:
            rospy.loginfo(
                "route curation overlay ready pose_source=%s radius=%.1fm rate<=%.1fHz "
                "curated_markings=%d",
                self.pose_source, self.radius_m, self.max_hz,
                self.curated_feature_count,
            )
        elif self.comparison_mode and self.comparison_primary == "curated":
            rospy.loginfo(
                "HD-map source comparison ready pose_source=%s radius=%.1fm rate<=%.1fHz "
                "final_markings=%d final_stoplines=%d legacy_refs=%d drivable=%s",
                self.pose_source, self.radius_m, self.max_hz,
                self.curated_feature_count,
                len(self.curated_stoplines),
                len(review["legacy_references"]),
                "latest+legacy_h5" if self.compare_drivable_sources else "disabled",
            )
        else:
            rospy.loginfo(
                "candidate map overlay ready pose_source=%s radius=%.1fm rate<=%.1fHz "
                "links=%d selected_markings=%d legacy_refs=%d",
                self.pose_source, self.radius_m, self.max_hz,
                len(review["current_links"]), len(review["adopted_legacy_boundaries"]),
                len(review["legacy_references"]),
            )

    @staticmethod
    def _subscribed(publisher) -> bool:
        return publisher.get_num_connections() > 0

    def _publish_single(self, key, layer, pose, stamp, namespace, width, color) -> None:
        publisher = self.publishers[key]
        if not self._subscribed(publisher):
            return
        local = layer.around_ego(pose[:2], pose[2], self.radius_m)
        publisher.publish(MarkerArray(markers=[
            _delete_all(stamp), _line_marker(namespace, 0, local, stamp, width, color),
        ]))

    def _publish_calibrated(self, pose, stamp) -> None:
        publisher = self.publishers["calibrated"]
        if not self._subscribed(publisher):
            return
        colors = {
            "white": (0.96, 0.96, 0.96, 1.0),
            "yellow": (1.0, 0.82, 0.04, 1.0),
            "unknown": (0.2, 0.9, 0.3, 0.9),
        }
        markers = [_delete_all(stamp)]
        for marker_id, color in enumerate(("white", "yellow", "unknown")):
            local = self.calibrated[color].around_ego(pose[:2], pose[2], self.radius_m)
            markers.append(_line_marker(f"observed_calibrated_{color}", marker_id, local, stamp, 0.14, colors[color]))
        publisher.publish(MarkerArray(markers=markers))

    def _publish_adopted(self, pose, stamp) -> None:
        publisher = self.publishers["adopted"]
        if not self._subscribed(publisher):
            return
        colors = {
            "white": (0.96, 0.96, 0.96, 1.0),
            "yellow": (1.0, 0.82, 0.04, 1.0),
            "unknown": (0.2, 0.9, 0.3, 0.9),
        }
        markers = [_delete_all(stamp)]
        marker_id = 0
        for color in ("white", "yellow", "unknown"):
            for shape in ("solid", "broken"):
                local = self.adopted[(color, shape)].around_ego(
                    pose[:2], pose[2], self.radius_m,
                )
                markers.append(_line_marker(
                    f"reviewed_hybrid_{color}_{shape}", marker_id,
                    local, stamp, 0.14, colors[color],
                ))
                marker_id += 1
        publisher.publish(MarkerArray(markers=markers))

    def _publish_changed_legacy(self, pose, stamp) -> None:
        publisher = self.publishers["changed_legacy"]
        if not self._subscribed(publisher):
            return
        local = self.changed_legacy.around_ego(pose[:2], pose[2], self.radius_m)
        publisher.publish(MarkerArray(markers=[
            _delete_all(stamp),
            _line_marker("changed_region_legacy_review", 0, local, stamp, 0.07, (0.95, 0.2, 0.9, 0.75)),
        ]))

    def _publish_legacy(self, pose, stamp) -> None:
        publisher = self.publishers["legacy"]
        if not self._subscribed(publisher):
            return
        base_colors = {
            "white": (0.95, 0.3, 0.9), "yellow": (0.75, 0.15, 1.0),
            "blue": (0.2, 0.5, 1.0), "unknown": (0.65, 0.4, 0.7),
        }
        markers = [_delete_all(stamp)]
        marker_id = 0
        for tier in ("A", "B"):
            for color in ("white", "yellow", "blue", "unknown"):
                local = self.legacy[(tier, color)].around_ego(pose[:2], pose[2], self.radius_m)
                rgb = base_colors[color]
                alpha = 0.75 if tier == "A" else 0.4
                markers.append(_line_marker(f"legacy_{tier}_{color}", marker_id, local, stamp, 0.07, (*rgb, alpha)))
                marker_id += 1
        publisher.publish(MarkerArray(markers=markers))

    def _publish_observations(self, pose, stamp) -> None:
        publisher = self.publishers["observations"]
        if not self._subscribed(publisher):
            return
        colors = {
            "automatic": (0.1, 0.7, 1.0, 0.7),
            "review_required": (1.0, 0.5, 0.05, 0.9),
            "rejected": (1.0, 0.1, 0.1, 0.9),
            "unmatched": (0.9, 0.1, 0.7, 0.9),
        }
        markers = [_delete_all(stamp)]
        for marker_id, status in enumerate(colors):
            local = self.observations[status].around_ego(pose[:2], pose[2], self.radius_m)
            markers.append(_line_marker(f"source_{status}", marker_id, local, stamp, 0.06, colors[status]))
        publisher.publish(MarkerArray(markers=markers))

    @staticmethod
    def _map_to_local(xy, pose):
        heading = math.radians(pose[2])
        delta = np.asarray(xy) - np.asarray(pose[:2])
        return np.asarray([
            delta[0] * math.cos(heading) + delta[1] * math.sin(heading),
            -delta[0] * math.sin(heading) + delta[1] * math.cos(heading),
        ])

    def _publish_labels(self, pose, stamp) -> None:
        publisher = self.publishers["labels"]
        if not self._subscribed(publisher):
            return
        markers = [_delete_all(stamp)]
        marker_id = 0
        for record in self.high_speed_labels:
            local = self._map_to_local(record["xy"], pose)
            if np.linalg.norm(local) > self.radius_m:
                continue
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = stamp
            marker.ns = "high_speed_link_label"
            marker.id = marker_id
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.pose.position = _point(local[0], local[1], 0.35)
            marker.pose.orientation.w = 1.0
            marker.scale.z = 0.55
            marker.color = ColorRGBA(0.1, 0.9, 0.95, 0.9)
            marker.text = f"{record['link_id']} lane={record['ego_lane']}"
            markers.append(marker)
            marker_id += 1
        publisher.publish(MarkerArray(markers=markers))

    def _publish_ego(self, stamp) -> None:
        publisher = self.publishers["ego"]
        if not self._subscribed(publisher):
            return
        vehicle = Marker()
        vehicle.header.frame_id = "base_link"
        vehicle.header.stamp = stamp
        vehicle.ns, vehicle.id = "ego_vehicle", 0
        vehicle.type, vehicle.action = Marker.CUBE, Marker.ADD
        vehicle.pose.position.x, vehicle.pose.position.z = 1.5275, 0.04
        vehicle.pose.orientation.w = 1.0
        vehicle.scale.x, vehicle.scale.y, vehicle.scale.z = 4.635, 1.892, 0.08
        vehicle.color = ColorRGBA(0.1, 0.65, 1.0, 0.55)

        origin = Marker()
        origin.header.frame_id = "base_link"
        origin.header.stamp = stamp
        origin.ns, origin.id = "base_link_origin", 0
        origin.type, origin.action = Marker.SPHERE, Marker.ADD
        origin.pose.position.z = 0.12
        origin.pose.orientation.w = 1.0
        origin.scale.x = origin.scale.y = origin.scale.z = 0.24
        origin.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)

        forward = Marker()
        forward.header.frame_id = "base_link"
        forward.header.stamp = stamp
        forward.ns, forward.id = "base_link_forward_x", 0
        forward.type, forward.action = Marker.ARROW, Marker.ADD
        forward.points = [_point(0.0, 0.0, 0.14), _point(4.0, 0.0, 0.14)]
        forward.scale.x, forward.scale.y, forward.scale.z = 0.08, 0.18, 0.28
        forward.color = ColorRGBA(1.0, 0.15, 0.1, 0.95)

        left = Marker()
        left.header.frame_id = "base_link"
        left.header.stamp = stamp
        left.ns, left.id = "base_link_left_y", 0
        left.type, left.action = Marker.ARROW, Marker.ADD
        left.points = [_point(0.0, 0.0, 0.16), _point(0.0, 2.0, 0.16)]
        left.scale.x, left.scale.y, left.scale.z = 0.06, 0.15, 0.24
        left.color = ColorRGBA(0.15, 1.0, 0.25, 0.95)

        publisher.publish(MarkerArray(markers=[
            _delete_all(stamp), vehicle, origin, forward, left,
        ]))

    def _publish_comparison_source(
        self, key, layers, stoplines, pose, stamp, namespace, unknown_color,
    ) -> None:
        publisher = self.publishers[key]
        if not self._subscribed(publisher):
            return
        markers = [_delete_all(stamp)]
        colors = {
            "white": (0.96, 0.96, 0.96, 0.98),
            "yellow": (1.0, 0.82, 0.04, 0.98),
            "blue": (0.2, 0.55, 1.0, 0.95),
            "unknown": unknown_color,
        }
        marker_id = 0
        for color in ("white", "yellow", "blue", "unknown"):
            for shape in ("solid", "broken"):
                layer = layers.get((color, shape))
                if layer is None:
                    continue
                local = layer.around_ego(pose[:2], pose[2], self.radius_m)
                if not len(local):
                    continue  # RViz rejects empty LINE_LIST; DELETEALL already cleared it.
                markers.append(_line_marker(
                    f"{namespace}_{color}_{shape}", marker_id, local, stamp, 0.12,
                    colors[color],
                ))
                marker_id += 1
        stops = stoplines.around_ego(pose[:2], pose[2], self.radius_m)
        if len(stops):
            markers.append(_line_marker(
                f"{namespace}_stopline", marker_id, stops, stamp, 0.20,
                (1.0, 0.1, 0.1, 0.95),
            ))
        publisher.publish(MarkerArray(markers=markers))

    def _publish_drivable_comparison(self, pose, stamp) -> None:
        if "compare_latest_drivable" not in self.publishers:
            return
        now = time.monotonic()
        if now - self.drivable_last_publish < self.drivable_minimum_period:
            return
        for key, source in (
            ("compare_latest_drivable", self.compare_latest_drivable),
            ("compare_legacy_drivable", self.compare_legacy_drivable),
        ):
            local = _local_drivable_grid(
                source, pose, self.radius_m, self.drivable_resolution_m,
            )
            message = OccupancyGrid()
            message.header.frame_id = "base_link"
            message.header.stamp = stamp
            message.info.resolution = self.drivable_resolution_m
            message.info.width = int(local.shape[1])
            message.info.height = int(local.shape[0])
            extent = message.info.width * message.info.resolution
            message.info.origin.position.x = -0.5 * extent
            message.info.origin.position.y = -0.5 * extent
            message.info.origin.orientation.w = 1.0
            message.data = local.reshape(-1).tolist()
            self.publishers[key].publish(message)
        self.drivable_last_publish = now

    def _publish_three_source_comparison(self, pose, stamp) -> None:
        primary_layers = (
            self.curated if self.comparison_primary == "curated" else self.compare_preplan
        )
        primary_stoplines = (
            self.curated_stoplines
            if self.comparison_primary == "curated" else self.compare_preplan_stoplines
        )
        primary_namespace = (
            "final_curated" if self.comparison_primary == "curated" else "preplan_hybrid"
        )
        self._publish_comparison_source(
            "compare_primary", primary_layers, primary_stoplines,
            pose, stamp, primary_namespace, (0.1, 0.9, 0.95, 0.95),
        )
        self._publish_comparison_source(
            "compare_won_tae",
            self.compare_won_tae,
            self.compare_won_tae_stoplines,
            pose, stamp, "won_tae_bake", (1.0, 0.55, 0.05, 0.95),
        )
        self._publish_comparison_source(
            "compare_legacy", self.compare_legacy, self.compare_legacy_stoplines,
            pose, stamp, "legacy_origin_aligned", (0.95, 0.15, 0.9, 0.9),
        )
        self._publish_drivable_comparison(pose, stamp)
        self._publish_ego(stamp)

    def _publish_curated_candidate(self, pose, stamp) -> None:
        self._publish_comparison_source(
            "curated", self.curated, self.curated_stoplines,
            pose, stamp, "route_curation", (0.1, 0.9, 0.95, 0.95),
        )
        self._publish_ego(stamp)

    def _publish_pose(self, pose, stamp) -> None:
        now = time.monotonic()
        if now - self.last_publish < self.minimum_period:
            return
        if not all(math.isfinite(value) for value in pose):
            rospy.logwarn_throttle(2.0, "candidate map overlay ignored non-finite map pose")
            return
        if not self._first_pose_logged:
            nearest_link_m = self.layers["current_links"].nearest_distance_m(pose[:2])
            rospy.loginfo(
                "candidate map first pose source=%s x=%.3f y=%.3f heading=%.3fdeg "
                "nearest_current_link=%.3fm",
                self.pose_source, pose[0], pose[1], pose[2], nearest_link_m,
            )
            if nearest_link_m > self.pose_distance_warning_m:
                rospy.logwarn(
                    "candidate map pose is %.3fm from the nearest built corridor link; "
                    "the vehicle may be outside the competition corridor, otherwise check "
                    "simulator map, GPS offset, UTM zone, and candidate-map origin",
                    nearest_link_m,
                )
            self._first_pose_logged = True
        if self.curation_only:
            self._publish_curated_candidate(pose, stamp)
            self.last_publish = now
            return
        if self.comparison_mode:
            self._publish_three_source_comparison(pose, stamp)
            self.last_publish = now
            return
        self._publish_single("current_links", self.layers["current_links"], pose, stamp, "current_link_center", 0.05, (0.0, 0.75, 0.9, 0.75))
        self._publish_single("nominal", self.layers["nominal"], pose, stamp, "nominal_boundary", 0.05, (0.55, 0.55, 0.55, 0.65))
        self._publish_adopted(pose, stamp)
        self._publish_calibrated(pose, stamp)
        self._publish_changed_legacy(pose, stamp)
        self._publish_legacy(pose, stamp)
        self._publish_observations(pose, stamp)
        self._publish_single("stoplines", self.layers["stoplines"], pose, stamp, "stopline_candidate", 0.20, (1.0, 0.1, 0.1, 0.95))
        self._publish_single("route", self.layers["route"], pose, stamp, "competition_route", 0.10, (0.15, 1.0, 0.2, 0.85))
        self._publish_labels(pose, stamp)
        self._publish_ego(stamp)
        self.last_publish = now

    def _on_ego(self, message) -> None:
        pose = (float(message.position.x), float(message.position.y), float(message.heading))
        if abs(pose[0]) <= 1e-6 and abs(pose[1]) <= 1e-6:
            rospy.logerr_throttle(
                2.0,
                "candidate map rejected /Ego_topic.position=(0,0): this bridge does not "
                "provide map XY in EgoVehicleStatus; use pose_source:=gps_imu",
            )
            return
        nearest_link_m = self.layers["current_links"].nearest_distance_m(pose[:2])
        if nearest_link_m > self.pose_distance_warning_m:
            rospy.logerr_throttle(
                2.0,
                "candidate map rejected EgoVehicleStatus pose %.3fm from the competition "
                "corridor; position is not usable map XY, use pose_source:=gps_imu",
                nearest_link_m,
            )
            return
        stamp = message.header.stamp if message.header.stamp.to_nsec() else rospy.Time.now()
        self._publish_pose(pose, stamp)

    def _on_imu(self, message) -> None:
        try:
            orientation = message.orientation
            yaw = quaternion_yaw_rad(
                orientation.x, orientation.y, orientation.z, orientation.w,
            )
        except ValueError as error:
            rospy.logwarn_throttle(2.0, "candidate map ignored invalid IMU: %s", error)
            return
        stamp = message.header.stamp if message.header.stamp.to_nsec() else rospy.Time.now()
        with self.pose_lock:
            self.imu_samples.append((stamp.to_sec(), yaw))

    def _on_gps(self, message) -> None:
        if int(message.status) <= 0:
            rospy.logwarn_throttle(2.0, "candidate map ignored invalid GPS status=%s", message.status)
            return
        stamp = message.header.stamp if message.header.stamp.to_nsec() else rospy.Time.now()
        with self.pose_lock:
            samples = tuple(self.imu_samples)
        if not samples:
            rospy.logwarn_throttle(2.0, "candidate map is waiting for IMU orientation")
            return
        stamp_sec = stamp.to_sec()
        imu_stamp, yaw = min(samples, key=lambda value: abs(value[0] - stamp_sec))
        gap_sec = abs(imu_stamp - stamp_sec)
        if gap_sec > self.maximum_imu_gap_sec:
            rospy.logwarn_throttle(
                2.0, "candidate map GPS/IMU gap %.1fms exceeds %.1fms",
                gap_sec * 1000.0, self.maximum_imu_gap_sec * 1000.0,
            )
            return
        raw_offset = np.asarray(
            [float(message.eastOffset), float(message.northOffset)], dtype=np.float64,
        )
        offset_delta = float(np.linalg.norm(raw_offset - self.map_origin_xy))
        if offset_delta > 0.10 and not self._offset_warning_emitted:
            rospy.logwarn(
                "GPS message offset differs from candidate-map origin by %.3fm; "
                "using candidate-map origin %s",
                offset_delta, self.map_origin_xy.tolist(),
            )
            self._offset_warning_emitted = True
        try:
            easting, northing = self.utm_transformer.transform(
                float(message.longitude), float(message.latitude),
            )
            base_xy = projected_gps_to_base_link(
                (easting, northing), self.map_origin_xy, yaw, self.gps_lever_arm_xy,
            )
        except (ValueError, OverflowError) as error:
            rospy.logwarn_throttle(2.0, "candidate map GPS conversion failed: %s", error)
            return
        self._publish_pose(
            (float(base_xy[0]), float(base_xy[1]), math.degrees(yaw)), stamp,
        )


def main() -> None:
    rospy.init_node("candidate_map_overlay")
    CandidateMapOverlayNode()
    rospy.spin()


if __name__ == "__main__":
    main()
