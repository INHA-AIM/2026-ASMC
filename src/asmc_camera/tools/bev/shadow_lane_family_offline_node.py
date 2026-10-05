#!/usr/bin/env python3
"""Offline A/B audit of independent splines and a shared lane family.

The node consumes the existing debug metric-evidence cloud during raw-bag
replay.  It publishes no planning or perception result and writes immutable
JSON/overlay artifacts when ROS shuts down.
"""

from __future__ import annotations

from collections import Counter, deque
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import threading
import time
import traceback

import cv2
import numpy as np
import rospy
from morai_msgs.msg import EgoVehicleStatus
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2

from asmc_camera.bev.boundary_candidates import (
    TemporalBoundaryTracker,
    select_shadow_boundaries_with_audit,
    select_shadow_lane_tracks_with_audit,
)
from asmc_camera.bev.lane_evidence import reduce_evidence_to_nodes
from asmc_camera.bev.lane_tracking import associate_lane_nodes_with_audit
from asmc_camera.bev.model_grid_projection import MetricEvidence
from asmc_camera.bev.invalid_support import (
    classify_invalid_support,
    invalid_support_config_from_dict,
)
from asmc_camera.offline.bev.invalid_support_review import InvalidSupportReviewSampler
from asmc_camera.offline.bev.invalid_map_audit import (
    InvalidMapReviewSampler,
    invalid_category_masks,
    invalid_distance_band,
)
from asmc_camera.bev.planning_usability import (
    DEFAULT_PLANNING_ZONES,
    measure_planning_usability,
    planning_zones_from_config,
)
from asmc_camera.offline.bev.pose_time_alignment import align_ego_pose, pose_delta
from asmc_camera.offline.bev.rgb_bev import StampedEgoPose
from asmc_camera.offline.bev.shadow_map_gt import (
    map_xy_to_base,
    nearest_polyline_distance,
    nearest_polyline_signed_normal_distance,
)
from asmc_camera.bev.shared_lane_family import (
    LEFT_ROLE,
    RIGHT_ROLE,
    ROLE_ORDER,
    SAMPLE_INVALID,
    SAMPLE_OBSERVED,
    SAMPLE_STRUCTURE_INFERRED,
    YELLOW_ROLE,
    fit_shared_lane_family_with_audit,
)
from asmc_camera.bev.spline_fitting import fit_lane_tracks_with_audit


SCHEMA_VERSION = "asmc-shadow-lane-family-ab-1.5.0"
METHODS = ("independent", "shared")
DISTANCE_BINS = ((0.0, 10.0), (10.0, 15.0), (15.0, 20.0))


def _utc_now():
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def _sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _percentile(values, percentile):
    finite = np.asarray(
        [value for value in values if value is not None and np.isfinite(value)],
        dtype=np.float64,
    )
    return float(np.percentile(finite, percentile)) if len(finite) else None


def _longest_false_streak(values):
    current = longest = 0
    for value in values:
        current = 0 if bool(value) else current + 1
        longest = max(longest, current)
    return int(longest)


def _curve_y_at_x(points, x_m):
    value = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    order = np.argsort(value[:, 0], kind="mergesort")
    value = value[order]
    unique_x, indices = np.unique(value[:, 0], return_index=True)
    target = float(np.clip(float(x_m), unique_x[0], unique_x[-1]))
    return float(np.interp(target, unique_x, value[indices, 1]))


class ShadowLaneFamilyOfflineNode:
    def __init__(self):
        self.output_dir = Path(rospy.get_param("~output_dir")).expanduser().resolve()
        self.run_id = str(rospy.get_param("~run_id")).strip()
        if not self.run_id:
            raise ValueError("offline family audit requires run_id")
        self.output_dir.mkdir(parents=True, exist_ok=False)
        self.frames_path = self.output_dir / "frames.jsonl"
        self.pseudo_gt_path = Path(
            rospy.get_param("~pseudo_gt_path")
        ).expanduser().resolve()
        with self.pseudo_gt_path.open("r", encoding="utf-8") as stream:
            self.pseudo_gt = json.load(stream)
        if self.pseudo_gt.get("status") != "map_pseudo_gt":
            raise ValueError("offline family audit requires map_pseudo_gt")
        self.heading_offset_deg = float(
            (self.pseudo_gt.get("pose_alignment") or {}).get(
                "ego_heading_offset_deg", 0.0
            )
        )
        self.map_boundaries = {
            name: np.asarray(points, dtype=np.float64)[:, :2]
            for name, points in self.pseudo_gt["boundaries"].items()
        }
        self.shadow_polygon = np.ascontiguousarray(
            np.asarray(
                self.pseudo_gt["shadow_area"]["polygon_xy_m"], dtype=np.float32
            )[:, :2]
        )
        self.evidence_topic = str(
            rospy.get_param(
                "~evidence_topic", "/perception/lane/debug/evidence_points"
            )
        )
        self.ego_topic = str(rospy.get_param("~ego_topic", "/Ego_topic"))
        self.ego_max_age_ns = int(
            round(float(rospy.get_param("~ego_pose_max_age_ms", 50.0)) * 1e6)
        )
        self.ego_alignment_mode = str(
            rospy.get_param("~ego_pose_alignment_mode", "nearest")
        ).strip().lower()
        self.ego_max_bracket_ns = int(
            round(float(rospy.get_param("~ego_pose_max_bracket_ms", 100.0)) * 1e6)
        )
        if self.ego_alignment_mode not in ("nearest", "interpolate"):
            raise ValueError("ego_pose_alignment_mode must be nearest or interpolate")
        self.ego_buffer = deque(maxlen=int(rospy.get_param("~ego_buffer_size", 300)))
        self.lock = threading.Lock()
        self.started_at = _utc_now()
        self.started_monotonic = time.monotonic()
        self.rows = []
        self.errors = Counter()
        self.overlay_stride = int(rospy.get_param("~overlay_stride", 30))
        self.overlay_limit = int(rospy.get_param("~overlay_limit", 24))
        self.overlay_count = 0
        self.overlay_dir = self.output_dir / "overlays"
        self.high_residual_overlay_count = 0
        self.high_residual_overlay_threshold_m = float(
            rospy.get_param("~high_residual_overlay_threshold_m", 0.50)
        )
        self.high_residual_overlay_limit = int(
            rospy.get_param("~high_residual_overlay_limit", 100)
        )
        self.invalid_review_sampler = InvalidSupportReviewSampler(
            limit_per_category=int(
                rospy.get_param("~invalid_support_overlay_limit_per_category", 0)
            ),
            min_frame_gap=int(
                rospy.get_param("~invalid_support_overlay_min_frame_gap", 15)
            ),
        )
        self.invalid_review_manifest = []
        self.invalid_map_review_sampler = InvalidMapReviewSampler(
            limit_per_category_band=int(
                rospy.get_param("~invalid_map_overlay_limit_per_category_band", 0)
            ),
            min_frame_gap=int(
                rospy.get_param("~invalid_map_overlay_min_frame_gap", 15)
            ),
        )
        self.invalid_map_review_manifest = []
        self.pseudo_gt_disagreement_m = float(
            rospy.get_param("~pseudo_gt_disagreement_m", 0.50)
        )
        if (
            self.high_residual_overlay_threshold_m <= 0.0
            or self.high_residual_overlay_limit < 0
            or self.pseudo_gt_disagreement_m <= 0.0
        ):
            raise ValueError("offline review thresholds are invalid")
        self.x_bounds = (
            float(rospy.get_param("~evidence_x_min_m")),
            float(rospy.get_param("~evidence_x_max_m")),
        )
        self.y_bounds = (
            float(rospy.get_param("~evidence_y_min_m")),
            float(rospy.get_param("~evidence_y_max_m")),
        )
        self.node_parameters = {
            "x_bin_m": float(rospy.get_param("~x_bin_m")),
            "lateral_cluster_gap_m": float(
                rospy.get_param("~lateral_cluster_gap_m")
            ),
            "x_origin_m": self.x_bounds[0],
        }
        self.track_parameters = {
            "max_track_gap_m": float(rospy.get_param("~max_track_gap_m")),
            "max_lateral_residual_m": float(
                rospy.get_param("~max_lateral_residual_m")
            ),
            "max_heading_change_deg": float(
                rospy.get_param("~max_heading_change_deg")
            ),
            "min_track_nodes": int(rospy.get_param("~min_track_nodes")),
            "min_observed_span_m": float(
                rospy.get_param("~min_observed_span_m")
            ),
        }
        self.spline_parameters = {
            "smoothing": float(rospy.get_param("~spline_smoothing")),
            "sample_spacing_m": float(
                rospy.get_param("~spline_sample_spacing_m")
            ),
            "robust_iterations": int(
                rospy.get_param("~spline_robust_iterations")
            ),
            "outlier_scale": float(rospy.get_param("~spline_outlier_scale")),
            "control_spacing_m": float(
                rospy.get_param("~spline_control_spacing_m")
            ),
        }
        self.boundary_parameters = dict(rospy.get_param("~shadow_boundary"))
        self.boundary_parameters.pop("enabled", None)
        temporal = {
            "max_lateral_residual_m": float(
                self.boundary_parameters.pop(
                    "temporal_max_lateral_residual_m", 0.65
                )
            ),
            "max_missed_frames": int(
                self.boundary_parameters.pop("temporal_max_missed_frames", 8)
            ),
            "reset_gap_sec": float(
                self.boundary_parameters.pop("temporal_reset_gap_sec", 0.75)
            ),
            "velocity_smoothing": float(
                self.boundary_parameters.pop("temporal_velocity_smoothing", 0.50)
            ),
            "max_lateral_speed_mps": float(
                self.boundary_parameters.pop("temporal_max_lateral_speed_mps", 4.0)
            ),
        }
        self.independent_tracker = TemporalBoundaryTracker(**temporal)
        self.independent_role_ids = {}
        self.independent_id_switches = Counter()
        self.shared_parameters = dict(rospy.get_param("~shared_lane_family"))
        self.shared_parameters.pop("evaluation_only", None)
        self.shared_parameters.pop("runtime_debug_enabled", None)
        self.shared_parameters.pop("deployment_allowed", None)
        self.shared_parameters.pop("representation", None)
        self.invalid_support_config = invalid_support_config_from_dict(
            rospy.get_param("~invalid_support_classification", {})
        )
        self.planning_zones = planning_zones_from_config(
            rospy.get_param(
                "~planning_usability_zones",
                [
                    {
                        "name": zone.name,
                        "x_min_m": zone.x_min_m,
                        "x_max_m": zone.x_max_m,
                    }
                    for zone in DEFAULT_PLANNING_ZONES
                ],
            )
        )
        self.config_snapshot = {
            "node": self.node_parameters,
            "track": self.track_parameters,
            "independent_spline": self.spline_parameters,
            "boundary": self.boundary_parameters,
            "shared_lane_family": self.shared_parameters,
            "planning_usability": {
                "interval_contract": "both_endpoint_support_non_invalid",
                "zones": [
                    {
                        "name": value.name,
                        "x_min_m": value.x_min_m,
                        "x_max_m": value.x_max_m,
                    }
                    for value in self.planning_zones
                ],
            },
            "invalid_support_classification": (
                self.invalid_support_config.to_dict()
            ),
            "invalid_support_overlay_review": {
                "limit_per_category": self.invalid_review_sampler.limit_per_category,
                "min_frame_gap": self.invalid_review_sampler.min_frame_gap,
                "measurement_only": True,
                "invalid_geometry_planning_authority": False,
            },
            "ego_pose_alignment": {
                "mode": self.ego_alignment_mode,
                "max_nearest_age_ms": self.ego_max_age_ns / 1e6,
                "max_bracket_span_ms": self.ego_max_bracket_ns / 1e6,
                "offline_map_evaluation_only": True,
                "runtime_ego_motion_compensation": False,
            },
            "invalid_map_distance_review": {
                "limit_per_category_band": self.invalid_map_review_sampler.limit,
                "min_frame_gap": self.invalid_map_review_sampler.min_frame_gap,
                "error_bands_m": [0.35, 0.60],
                "bands_are_acceptance_gates": False,
            },
        }
        rospy.Subscriber(
            self.ego_topic, EgoVehicleStatus, self._ego_callback, queue_size=200
        )
        rospy.Subscriber(
            self.evidence_topic, PointCloud2, self._evidence_callback, queue_size=2
        )
        rospy.on_shutdown(self._finalize)
        rospy.logwarn(
            "shadow lane-family A/B is offline-only: run=%s output=%s",
            self.run_id,
            self.output_dir,
        )

    def _ego_callback(self, message):
        stamp_ns = int(message.header.stamp.to_nsec())
        if stamp_ns <= 0:
            stamp_ns = int(rospy.Time.now().to_nsec())
        value = StampedEgoPose(
            stamp_ns=stamp_ns,
            x_m=float(message.position.x),
            y_m=float(message.position.y),
            heading_deg=float(message.heading),
        )
        if np.isfinite((value.x_m, value.y_m, value.heading_deg)).all():
            with self.lock:
                self.ego_buffer.append(value)

    def _align_ego(self, stamp_ns, mode=None):
        with self.lock:
            values = list(self.ego_buffer)
        return align_ego_pose(
            values,
            stamp_ns,
            self.ego_max_age_ns,
            self.ego_max_bracket_ns,
            self.ego_alignment_mode if mode is None else mode,
        )

    @staticmethod
    def _decode_evidence(message):
        rows = list(
            point_cloud2.read_points(
                message,
                field_names=(
                    "x",
                    "y",
                    "confidence",
                    "geometric_quality",
                    "class_id",
                    "source_mask",
                ),
                skip_nans=True,
            )
        )
        if not rows:
            return MetricEvidence.empty()
        value = np.asarray(rows, dtype=np.float64)
        return MetricEvidence(
            xy=np.ascontiguousarray(value[:, :2], dtype=np.float32),
            class_id=np.ascontiguousarray(value[:, 4], dtype=np.uint8),
            confidence=np.ascontiguousarray(value[:, 2], dtype=np.float32),
            geometric_quality=np.ascontiguousarray(value[:, 3], dtype=np.float32),
            source_mask=np.ascontiguousarray(value[:, 5], dtype=np.uint8),
        )

    def _map_targets(self, ego):
        if ego is None:
            return None
        x_m, y_m, heading_deg = ego.x_m, ego.y_m, ego.heading_deg
        if cv2.pointPolygonTest(
            self.shadow_polygon, (float(x_m), float(y_m)), False
        ) < 0.0:
            return None
        transformed = {
            name: map_xy_to_base(
                points,
                (x_m, y_m),
                heading_deg + self.heading_offset_deg,
            )
            for name, points in self.map_boundaries.items()
        }
        clipped = {}
        for name, points in transformed.items():
            keep = (
                (points[:, 0] >= self.x_bounds[0] - 1.0)
                & (points[:, 0] <= self.x_bounds[1] + 1.0)
                & (points[:, 1] >= self.y_bounds[0] - 2.0)
                & (points[:, 1] <= self.y_bounds[1] + 2.0)
            )
            if np.count_nonzero(keep) >= 2:
                clipped[name] = np.ascontiguousarray(points[keep])
        yellow = clipped.get("yellow_center")
        whites = [
            clipped[name]
            for name in ("white_positive_normal", "white_negative_normal")
            if name in clipped
        ]
        if yellow is None or len(whites) != 2:
            return None
        reference_x = float(self.boundary_parameters.get("reference_x_m", 5.0))
        whites.sort(key=lambda curve: -_curve_y_at_x(curve, reference_x))
        return {
            LEFT_ROLE: whites[0],
            YELLOW_ROLE: yellow,
            RIGHT_ROLE: whites[1],
        }

    def _boundary_metrics(self, points, target, inferred_points=None):
        prediction = np.asarray(points, dtype=np.float64).reshape(-1, 2)
        reference = np.asarray(target, dtype=np.float64).reshape(-1, 2)
        inferred = (
            np.empty((0, 2), dtype=np.float64)
            if inferred_points is None
            else np.asarray(inferred_points, dtype=np.float64).reshape(-1, 2)
        )
        if len(prediction) < 2 or len(reference) < 2:
            return {
                "available": False,
                "point_count": int(len(prediction)),
                "overlap_x_m": None,
                "overlap_span_m": 0.0,
                "map_distance_p50_m": None,
                "map_distance_p95_m": None,
                "reference_y_error_m": None,
                "distance_bins": {},
                "inferred_pseudo_gt_disagreement": {
                    "interpretation": "review_signal_only",
                    "threshold_m": self.pseudo_gt_disagreement_m,
                    "point_count": 0,
                    "above_threshold_count": 0,
                    "above_threshold_rate": None,
                    "distance_p95_m": None,
                },
            }
        overlap_min = max(float(np.min(prediction[:, 0])), float(np.min(reference[:, 0])))
        overlap_max = min(float(np.max(prediction[:, 0])), float(np.max(reference[:, 0])))
        selected = (
            (prediction[:, 0] >= overlap_min)
            & (prediction[:, 0] <= overlap_max)
        )
        prediction = prediction[selected]
        if overlap_max - overlap_min < 2.0 or len(prediction) < 2:
            return {
                "available": False,
                "point_count": int(len(prediction)),
                "overlap_x_m": [overlap_min, overlap_max],
                "overlap_span_m": max(0.0, overlap_max - overlap_min),
                "map_distance_p50_m": None,
                "map_distance_p95_m": None,
                "reference_y_error_m": None,
                "distance_bins": {},
                "inferred_pseudo_gt_disagreement": {
                    "interpretation": "review_signal_only",
                    "threshold_m": self.pseudo_gt_disagreement_m,
                    "point_count": 0,
                    "above_threshold_count": 0,
                    "above_threshold_rate": None,
                    "distance_p95_m": None,
                },
            }
        distance = nearest_polyline_distance(prediction, reference)
        reference_x = float(self.boundary_parameters.get("reference_x_m", 5.0))
        lateral_error = None
        if overlap_min <= reference_x <= overlap_max:
            lateral_error = _curve_y_at_x(prediction, reference_x) - _curve_y_at_x(
                reference, reference_x
            )
        result = {
            "available": True,
            "point_count": int(len(prediction)),
            "overlap_x_m": [overlap_min, overlap_max],
            "overlap_span_m": overlap_max - overlap_min,
            "map_distance_p50_m": _percentile(distance, 50),
            "map_distance_p95_m": _percentile(distance, 95),
            "reference_y_error_m": lateral_error,
            "distance_bins": {},
            "inferred_pseudo_gt_disagreement": {
                "interpretation": "review_signal_only",
                "threshold_m": self.pseudo_gt_disagreement_m,
                "point_count": 0,
                "above_threshold_count": 0,
                "above_threshold_rate": None,
                "distance_p95_m": None,
            },
        }
        for lower, upper in DISTANCE_BINS:
            selected = (prediction[:, 0] >= lower) & (prediction[:, 0] < upper)
            label = "{:g}_{:g}m".format(lower, upper)
            result["distance_bins"][label] = {
                "point_count": int(np.count_nonzero(selected)),
                "p50_m": _percentile(distance[selected], 50),
                "p95_m": _percentile(distance[selected], 95),
            }
        inferred = inferred[
            (inferred[:, 0] >= overlap_min) & (inferred[:, 0] <= overlap_max)
        ]
        if len(inferred):
            inferred_distance = nearest_polyline_distance(inferred, reference)
            disagreed = inferred_distance > self.pseudo_gt_disagreement_m
            result["inferred_pseudo_gt_disagreement"].update(
                {
                    "point_count": int(len(inferred)),
                    "above_threshold_count": int(np.count_nonzero(disagreed)),
                    "above_threshold_rate": float(np.mean(disagreed)),
                    "distance_p95_m": _percentile(inferred_distance, 95),
                }
            )
        return result

    def _method_map_metrics(self, boundaries, targets):
        if targets is None:
            return {"available": False, "roles": {}}
        result = {}
        for role, value in boundaries.items():
            if role not in targets:
                continue
            if hasattr(value, "sample_state"):
                valid = value.sample_state != SAMPLE_INVALID
                inferred = value.sample_state == SAMPLE_STRUCTURE_INFERRED
                points = value.sample_points[valid]
                inferred_points = value.sample_points[inferred]
            else:
                points = value.lane.sample_points
                inferred_points = None
            result[role] = self._boundary_metrics(
                points, targets[role], inferred_points=inferred_points
            )
        return {"available": bool(result), "roles": result}

    @staticmethod
    def _invalid_boundary_map_metrics(points, target):
        prediction = np.asarray(points, dtype=np.float64).reshape(-1, 2)
        reference = np.asarray(target, dtype=np.float64).reshape(-1, 2)
        if not len(prediction) or len(reference) < 2:
            return {
                "available": False,
                "point_count": int(len(prediction)),
                "signed_normal_p50_m": None,
                "signed_normal_mean_m": None,
                "absolute_p50_m": None,
                "absolute_p95_m": None,
                "distance_bins": {},
            }
        signed = nearest_polyline_signed_normal_distance(prediction, reference)
        absolute = np.abs(signed)
        result = {
            "available": True,
            "point_count": int(len(prediction)),
            "signed_normal_p50_m": _percentile(signed, 50),
            "signed_normal_mean_m": float(np.mean(signed)),
            "absolute_p50_m": _percentile(absolute, 50),
            "absolute_p95_m": _percentile(absolute, 95),
            "distance_bins": {},
        }
        for lower, upper in DISTANCE_BINS:
            selected = (prediction[:, 0] >= lower) & (prediction[:, 0] < upper)
            label = "{:g}_{:g}m".format(lower, upper)
            result["distance_bins"][label] = {
                "point_count": int(np.count_nonzero(selected)),
                "absolute_p50_m": _percentile(absolute[selected], 50),
                "absolute_p95_m": _percentile(absolute[selected], 95),
                "signed_normal_p50_m": _percentile(signed[selected], 50),
            }
        return result

    def _invalid_map_metrics(
        self, boundaries, targets, planning_usability, classification
    ):
        """Compare retained INVALID samples with the same-role map boundary."""
        if targets is None or not boundaries:
            return {
                "available": False,
                "categories": {},
                "interpretation": "review_signal_only",
            }
        by_category = {}
        for role in ROLE_ORDER:
            value = boundaries.get(role)
            if value is None or role not in targets:
                continue
            masks = invalid_category_masks(
                value.sample_points[:, 0],
                value.sample_state,
                planning_usability,
                classification,
            )
            for category, selected in masks.items():
                if not np.any(selected):
                    continue
                metric = self._invalid_boundary_map_metrics(
                    value.sample_points[selected], targets[role]
                )
                by_category.setdefault(category, {"roles": {}})["roles"][role] = metric
        for category, value in by_category.items():
            p95 = [
                metric["absolute_p95_m"]
                for metric in value["roles"].values()
                if metric.get("absolute_p95_m") is not None
            ]
            value["frame_absolute_p95_m"] = max(p95) if p95 else None
            value["review_band"] = invalid_distance_band(
                value["frame_absolute_p95_m"]
            )
        return {
            "available": bool(by_category),
            "categories": by_category,
            "interpretation": "role_matched_map_pseudo_gt_review_signal_only",
            "error_bands_m": [0.35, 0.60],
            "bands_are_acceptance_gates": False,
            "invalid_geometry_planning_authority": False,
        }

    def _evidence_callback(self, message):
        started = time.perf_counter()
        stamp_ns = int(message.header.stamp.to_nsec())
        row = {
            "frame_index": int(len(self.rows)),
            "stamp_ns": stamp_ns,
            "methods": {},
        }
        try:
            evidence = self._decode_evidence(message)
            nodes = reduce_evidence_to_nodes(evidence, **self.node_parameters)
            tracks, association_audit = associate_lane_nodes_with_audit(
                nodes, **self.track_parameters
            )
            selected_tracks, prefit_audit = select_shadow_lane_tracks_with_audit(
                tracks, self.boundary_parameters
            )
            lanes, spline_audit = fit_lane_tracks_with_audit(
                selected_tracks, **self.spline_parameters
            )
            observations, boundary_audit = select_shadow_boundaries_with_audit(
                lanes, self.boundary_parameters
            )
            tracked = self.independent_tracker.update(observations, stamp_ns)
            current_ids = {
                value.observation.role: int(value.stable_id)
                for value in tracked
                if value.observation.role in ROLE_ORDER
            }
            for role, stable_id in current_ids.items():
                if role in self.independent_role_ids and self.independent_role_ids[role] != stable_id:
                    self.independent_id_switches[role] += 1
                self.independent_role_ids[role] = stable_id
            independent = {value.role: value for value in observations if value.role in ROLE_ORDER}
            independent_residual = [
                float(value.lane.residual_p95_m) for value in independent.values()
            ]
            row["methods"]["independent"] = {
                "available": bool(independent),
                "yellow_observed": YELLOW_ROLE in independent,
                "structure_complete": all(role in independent for role in ROLE_ORDER),
                "fully_observed": all(role in independent for role in ROLE_ORDER),
                "observed_roles": [role for role in ROLE_ORDER if role in independent],
                "inferred_roles": [],
                "residual_p95_m": max(independent_residual) if independent_residual else None,
                "role_ids": current_ids,
                "audit": {
                    "spline": spline_audit,
                    "boundary": boundary_audit,
                },
            }
            family, family_audit = fit_shared_lane_family_with_audit(
                selected_tracks, self.shared_parameters
            )
            shared = {} if family is None else {
                value.role: value for value in family.boundaries
            }
            min_family_span_m = float(
                self.shared_parameters.get("min_family_span_m", 3.0)
            )
            role_valid_lengths = {} if family is None else {
                value.role: value.observed_length_m + value.inferred_length_m
                for value in family.boundaries
            }
            fully_observed_mask = (
                np.zeros(0, dtype=bool)
                if family is None
                else np.logical_and.reduce(
                    [
                        value.sample_state == SAMPLE_OBSERVED
                        for value in family.boundaries
                    ]
                )
            )
            inferred_mask = (
                np.zeros(0, dtype=bool)
                if family is None
                else np.logical_or.reduce(
                    [
                        value.sample_state == SAMPLE_STRUCTURE_INFERRED
                        for value in family.boundaries
                    ]
                )
            )
            valid_mask = (
                np.zeros(0, dtype=bool) if family is None else family.sample_valid
            )
            planning_usability = measure_planning_usability(
                family,
                zones=self.planning_zones,
                anchor_tolerance_m=self.invalid_support_config.gap_tolerance_m,
            )
            invalid_support_classification = classify_invalid_support(
                planning_usability, self.invalid_support_config
            )
            row["methods"]["shared"] = {
                "available": family is not None,
                "yellow_observed": bool(
                    family is not None
                    and np.any(
                        family.boundary(YELLOW_ROLE).sample_state
                        == SAMPLE_OBSERVED
                    )
                ),
                "yellow_structurally_available": bool(
                    family is not None
                    and role_valid_lengths.get(YELLOW_ROLE, 0.0)
                    >= min_family_span_m
                ),
                "structure_complete": bool(
                    family is not None
                    and all(
                        role_valid_lengths.get(role, 0.0) >= min_family_span_m
                        for role in ROLE_ORDER
                    )
                ),
                "fully_observed": bool(
                    family is not None
                    and np.any(valid_mask)
                    and np.all(fully_observed_mask[valid_mask])
                ),
                "observed_roles": [] if family is None else [
                    value.role
                    for value in family.boundaries
                    if np.any(value.sample_state == SAMPLE_OBSERVED)
                ],
                "inferred_roles": [] if family is None else [
                    value.role
                    for value in family.boundaries
                    if np.any(value.sample_state == SAMPLE_STRUCTURE_INFERRED)
                ],
                "residual_p95_m": None if family is None else family.data_residual_p95_m,
                "inlier_residual_p95_m": (
                    None if family is None else family.inlier_residual_p95_m
                ),
                "outlier_rate": None if family is None else family.outlier_rate,
                "weighted_rms_m": None if family is None else family.weighted_rms_m,
                "valid_family_length_m": None if family is None else float(
                    sum(
                        np.linalg.norm(np.diff(family.centre_points, axis=0), axis=1)
                        * (family.sample_valid[:-1] & family.sample_valid[1:])
                    )
                ),
                "locally_fully_observed_fraction": (
                    None
                    if family is None or not np.any(valid_mask)
                    else float(np.mean(fully_observed_mask[valid_mask]))
                ),
                "structure_inferred_fraction": (
                    None
                    if family is None or not np.any(valid_mask)
                    else float(np.mean(inferred_mask[valid_mask]))
                ),
                "boundary_support": None if family is None else {
                    value.role: {
                        "observed_length_m": value.observed_length_m,
                        "inferred_length_m": value.inferred_length_m,
                        "invalid_length_m": value.invalid_length_m,
                        "max_inferred_gap_m": value.max_inferred_gap_m,
                        "bridged_length_ratio": value.bridged_length_ratio,
                        "sample_counts": {
                            "observed": int(
                                np.count_nonzero(
                                    value.sample_state == SAMPLE_OBSERVED
                                )
                            ),
                            "structure_inferred": int(
                                np.count_nonzero(
                                    value.sample_state
                                    == SAMPLE_STRUCTURE_INFERRED
                                )
                            ),
                            "invalid": int(
                                np.count_nonzero(
                                    value.sample_state == SAMPLE_INVALID
                                )
                            ),
                        },
                    }
                    for value in family.boundaries
                },
                "widths_m": None if family is None else {
                    "left": family.left_width_m,
                    "right": family.right_width_m,
                },
                "audit": family_audit,
                "planning_usability": planning_usability,
                "invalid_support_classification": invalid_support_classification,
            }
            ego_alignment = self._align_ego(stamp_ns)
            nearest_alignment = self._align_ego(stamp_ns, mode="nearest")
            targets = self._map_targets(ego_alignment.pose)
            nearest_targets = self._map_targets(nearest_alignment.pose)
            row["ego_pose_alignment"] = ego_alignment.to_dict()
            row["ego_pose_alignment"]["delta_from_nearest"] = pose_delta(
                ego_alignment.pose, nearest_alignment.pose
            )
            row["ego_pose_age_ms"] = ego_alignment.nearest_age_ms
            row["within_map_odd"] = targets is not None
            row["methods"]["independent"]["map"] = self._method_map_metrics(
                independent, targets
            )
            row["methods"]["shared"]["map"] = self._method_map_metrics(
                shared, targets
            )
            invalid_map = self._invalid_map_metrics(
                shared,
                targets,
                planning_usability,
                invalid_support_classification,
            )
            nearest_invalid_map = self._invalid_map_metrics(
                shared,
                nearest_targets,
                planning_usability,
                invalid_support_classification,
            )
            row["invalid_map_distance"] = {
                "aligned": invalid_map,
                "nearest_reference": nearest_invalid_map,
                "comparison_purpose": "pose_time_alignment_sensitivity",
                "runtime_support_state_changed": False,
            }
            row["pipeline"] = {
                "evidence": int(len(evidence.xy)),
                "nodes": int(len(nodes)),
                "tracks": int(len(tracks)),
                "selected_tracks": int(len(selected_tracks)),
                "association": association_audit,
                "prefit": prefit_audit,
            }
            if (
                self.overlay_stride > 0
                and len(self.rows) % self.overlay_stride == 0
                and self.overlay_count < self.overlay_limit
            ):
                self._write_overlay(
                    len(self.rows),
                    independent,
                    shared,
                    targets,
                    selected_tracks,
                    category="periodic",
                    residual_p95_m=(
                        None if family is None else family.data_residual_p95_m
                    ),
                )
                self.overlay_count += 1
            if (
                family is not None
                and family.data_residual_p95_m
                >= self.high_residual_overlay_threshold_m
                and self.high_residual_overlay_count
                < self.high_residual_overlay_limit
            ):
                self._write_overlay(
                    len(self.rows),
                    independent,
                    shared,
                    targets,
                    selected_tracks,
                    category="high_residual",
                    residual_p95_m=family.data_residual_p95_m,
                )
                self.high_residual_overlay_count += 1
            for review_category in self.invalid_review_sampler.select(
                len(self.rows), invalid_support_classification
            ):
                relative_path = self._write_overlay(
                    len(self.rows),
                    independent,
                    shared,
                    targets,
                    selected_tracks,
                    category="invalid_support/{}".format(review_category),
                    residual_p95_m=(
                        None if family is None else family.data_residual_p95_m
                    ),
                    invalid_support_classification=invalid_support_classification,
                    selected_review_category=review_category,
                    show_invalid=True,
                )
                self.invalid_review_manifest.append(
                    {
                        "run_id": self.run_id,
                        "frame_index": int(len(self.rows)),
                        "stamp_ns": stamp_ns,
                        "selected_category": review_category,
                        "active_categories": [
                            name
                            for name, enabled in invalid_support_classification[
                                "flags"
                            ].items()
                            if enabled
                        ],
                        "primary_reason": invalid_support_classification[
                            "primary_reason"
                        ],
                        "measurements": invalid_support_classification[
                            "measurements"
                        ],
                        "overlay": relative_path,
                        "map_overlay_available": targets is not None,
                        "invalid_geometry_planning_authority": False,
                    }
                )
            category_bands = {
                category: value["review_band"]
                for category, value in invalid_map.get("categories", {}).items()
                if value.get("review_band") != "unavailable"
            }
            for review_category, review_band in self.invalid_map_review_sampler.select(
                len(self.rows), category_bands
            ):
                category_metric = invalid_map["categories"][review_category]
                relative_path = self._write_overlay(
                    len(self.rows),
                    independent,
                    shared,
                    targets,
                    selected_tracks,
                    category="invalid_map_distance/{}/{}".format(
                        review_category, review_band
                    ),
                    residual_p95_m=(
                        None if family is None else family.data_residual_p95_m
                    ),
                    invalid_support_classification=invalid_support_classification,
                    selected_review_category=review_category,
                    show_invalid=True,
                    invalid_map_review={
                        "band": review_band,
                        "frame_absolute_p95_m": category_metric.get(
                            "frame_absolute_p95_m"
                        ),
                    },
                )
                self.invalid_map_review_manifest.append(
                    {
                        "run_id": self.run_id,
                        "frame_index": int(len(self.rows)),
                        "stamp_ns": stamp_ns,
                        "selected_category": review_category,
                        "review_band": review_band,
                        "frame_absolute_p95_m": category_metric.get(
                            "frame_absolute_p95_m"
                        ),
                        "roles": category_metric.get("roles", {}),
                        "overlay": relative_path,
                        "ego_pose_alignment": row["ego_pose_alignment"],
                        "inside_pseudo_gt_odd": True,
                        "invalid_geometry_planning_authority": False,
                    }
                )
        except Exception as exc:
            name = type(exc).__name__
            self.errors[name] += 1
            row["error"] = "{}: {}".format(name, exc)
            row["traceback"] = traceback.format_exc()
            rospy.logwarn_throttle(
                2.0,
                "shared family offline frame failed: %s\n%s",
                exc,
                row["traceback"],
            )
        row["processing_ms"] = (time.perf_counter() - started) * 1000.0
        self.rows.append(row)
        if len(self.rows) % 100 == 0:
            rospy.loginfo(
                "shared family offline frames=%d errors=%d",
                len(self.rows),
                sum(self.errors.values()),
            )

    def _metric_pixel(self, points, shape):
        value = np.asarray(points, dtype=np.float64).reshape(-1, 2)
        height, width = shape[:2]
        row = (self.x_bounds[1] - value[:, 0]) / (
            self.x_bounds[1] - self.x_bounds[0]
        ) * (height - 1)
        column = (self.y_bounds[1] - value[:, 1]) / (
            self.y_bounds[1] - self.y_bounds[0]
        ) * (width - 1)
        return np.rint(np.column_stack((column, row))).astype(np.int32)

    def _draw_curve(
        self,
        image,
        points,
        color,
        thickness=2,
        sample_state=None,
        invalid_color=None,
    ):
        pixel = self._metric_pixel(points, image.shape)
        if sample_state is None:
            cv2.polylines(image, [pixel], False, color, thickness, cv2.LINE_AA)
            return
        state = np.asarray(sample_state, dtype=np.uint8)
        for index in range(len(pixel) - 1):
            endpoints = state[index : index + 2]
            if np.any(endpoints == SAMPLE_INVALID):
                if invalid_color is None:
                    continue
                segment_color = invalid_color
            else:
                segment_color = (
                    color
                    if np.all(endpoints == SAMPLE_OBSERVED)
                    else (0, 128, 255)
                )
            cv2.line(
                image,
                tuple(pixel[index]),
                tuple(pixel[index + 1]),
                segment_color,
                thickness,
                cv2.LINE_AA,
            )

    def _draw_support_points(self, image, selected_tracks, shared):
        if not shared:
            return
        role_by_class = {
            1: [LEFT_ROLE, RIGHT_ROLE],
            2: [YELLOW_ROLE],
        }
        for track in selected_tracks:
            candidates = []
            for role in role_by_class.get(int(track.class_id), []):
                value = shared.get(role)
                if value is None:
                    continue
                valid = value.sample_state != SAMPLE_INVALID
                if np.any(valid):
                    candidates.append(value.sample_points[valid])
            if not candidates:
                continue
            reference = np.concatenate(candidates)
            points = np.asarray(track.xy, dtype=np.float64).reshape(-1, 2)
            distance = np.asarray(
                [
                    float(np.min(np.linalg.norm(reference - point, axis=1)))
                    for point in points
                ]
            )
            pixels = self._metric_pixel(points, image.shape)
            for pixel, residual in zip(pixels, distance):
                color = (
                    (0, 0, 255)
                    if residual > float(
                        self.shared_parameters.get("support_residual_max_m", 0.35)
                    )
                    else (150, 150, 150)
                )
                cv2.circle(image, tuple(pixel), 2, color, -1, cv2.LINE_AA)

    def _write_overlay(
        self,
        index,
        independent,
        shared,
        targets,
        selected_tracks,
        category,
        residual_p95_m,
        invalid_support_classification=None,
        selected_review_category=None,
        show_invalid=False,
        invalid_map_review=None,
    ):
        output_dir = self.overlay_dir / category
        output_dir.mkdir(parents=True, exist_ok=True)
        image = np.zeros((750, 480, 3), dtype=np.uint8)
        for x_m in np.arange(math.ceil(self.x_bounds[0] / 5.0) * 5.0, self.x_bounds[1] + 0.1, 5.0):
            p = self._metric_pixel(
                [[x_m, self.y_bounds[0]], [x_m, self.y_bounds[1]]], image.shape
            )
            cv2.line(image, tuple(p[0]), tuple(p[1]), (45, 45, 45), 1)
        for y_m in np.arange(math.ceil(self.y_bounds[0] / 2.0) * 2.0, self.y_bounds[1] + 0.1, 2.0):
            p = self._metric_pixel(
                [[self.x_bounds[0], y_m], [self.x_bounds[1], y_m]], image.shape
            )
            cv2.line(image, tuple(p[0]), tuple(p[1]), (45, 45, 45), 1)
        if targets:
            for curve in targets.values():
                self._draw_curve(image, curve, (90, 90, 90), 1)
        for value in independent.values():
            self._draw_curve(image, value.lane.sample_points, (255, 180, 0), 2)
        self._draw_support_points(image, selected_tracks, shared)
        for value in shared.values():
            color = (0, 255, 255) if value.class_id == 2 else (255, 0, 255)
            self._draw_curve(
                image,
                value.sample_points,
                color,
                2,
                value.sample_state,
                invalid_color=(0, 0, 255) if show_invalid else None,
            )
        cv2.putText(image, "map gray (review only) | independent cyan", (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.43, (230, 230, 230), 1, cv2.LINE_AA)
        invalid_label = "invalid red (math only; unsupported)" if show_invalid else "invalid hidden"
        cv2.putText(image, "shared obs yellow/magenta | inferred orange | {}".format(invalid_label), (8, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (230, 230, 230), 1, cv2.LINE_AA)
        cv2.putText(image, "support gray | residual outlier red", (8, 62), cv2.FONT_HERSHEY_SIMPLEX, 0.43, (180, 180, 180), 1, cv2.LINE_AA)
        if residual_p95_m is not None:
            cv2.putText(
                image,
                "family residual p95={:.3f}m".format(residual_p95_m),
                (8, 82),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.43,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
        if invalid_support_classification is not None:
            measurements = invalid_support_classification.get("measurements", {})
            category_text = "review={} | active={}".format(
                selected_review_category,
                ",".join(
                    name
                    for name, enabled in invalid_support_classification.get(
                        "flags", {}
                    ).items()
                    if enabled
                )
                or "none",
            )
            cv2.putText(
                image,
                category_text,
                (8, 102),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.34,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
            metric_text = "lead={:.2f}m trail={:.2f}m near={:.0f}% mid={:.0f}%".format(
                float(measurements.get("vehicle_near_leading_tail_m", 0.0)),
                float(measurements.get("preview_trailing_tail_m", 0.0)),
                100.0 * float(measurements.get("near_coverage_ratio", 0.0)),
                100.0 * float(measurements.get("mid_coverage_ratio", 0.0)),
            )
            cv2.putText(
                image,
                metric_text,
                (8, 120),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.38,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
        if invalid_map_review is not None:
            p95 = invalid_map_review.get("frame_absolute_p95_m")
            label = "map normal-distance={} p95={}".format(
                invalid_map_review.get("band", "unknown"),
                "n/a" if p95 is None else "{:.3f}m".format(float(p95)),
            )
            cv2.putText(
                image,
                label,
                (8, 140),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.38,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
        path = output_dir / "frame_{:06d}.jpg".format(index)
        if not cv2.imwrite(str(path), image):
            raise OSError("failed to write overlay: {}".format(path))
        return str(path.relative_to(self.output_dir))

    @staticmethod
    def _summarize_method(rows, method):
        values = [row["methods"][method] for row in rows if "error" not in row]
        denominator = max(len(values), 1)
        odd_values = [
            row["methods"][method]
            for row in rows
            if "error" not in row and row.get("within_map_odd", False)
        ]
        odd_denominator = max(len(odd_values), 1)
        residual = [value["residual_p95_m"] for value in values if value.get("residual_p95_m") is not None]
        inlier_residual = [value["inlier_residual_p95_m"] for value in values if value.get("inlier_residual_p95_m") is not None]
        outlier_rate = [value["outlier_rate"] for value in values if value.get("outlier_rate") is not None]
        inferred_fraction = [value["structure_inferred_fraction"] for value in values if value.get("structure_inferred_fraction") is not None]
        fully_observed_fraction = [value["locally_fully_observed_fraction"] for value in values if value.get("locally_fully_observed_fraction") is not None]
        valid_family_length = [value["valid_family_length_m"] for value in values if value.get("valid_family_length_m") is not None]
        widths_left = [value["widths_m"]["left"] for value in values if value.get("widths_m")]
        widths_right = [value["widths_m"]["right"] for value in values if value.get("widths_m")]
        map_distance = []
        map_by_role = {role: [] for role in ROLE_ORDER}
        map_by_bin = {"{:g}_{:g}m".format(*limits): [] for limits in DISTANCE_BINS}
        map_reference_jitter = {role: [] for role in ROLE_ORDER}
        previous_reference_error = {role: None for role in ROLE_ORDER}
        map_role_evaluation_counts = Counter()
        pseudo_gt_disagreement_rates = []
        for value in values:
            frame_roles = (value.get("map") or {}).get("roles", {})
            for role in ROLE_ORDER:
                metric = frame_roles.get(role)
                if not metric or not metric.get("available", False):
                    previous_reference_error[role] = None
                    continue
                map_role_evaluation_counts[role] += 1
                if metric.get("map_distance_p95_m") is not None:
                    map_distance.append(metric["map_distance_p95_m"])
                    map_by_role[role].append(metric["map_distance_p95_m"])
                for label, item in metric.get("distance_bins", {}).items():
                    if item.get("p95_m") is not None:
                        map_by_bin[label].append(item["p95_m"])
                lateral_error = metric.get("reference_y_error_m")
                disagreement = metric.get("inferred_pseudo_gt_disagreement") or {}
                if disagreement.get("above_threshold_rate") is not None:
                    pseudo_gt_disagreement_rates.append(
                        disagreement["above_threshold_rate"]
                    )
                if lateral_error is None:
                    previous_reference_error[role] = None
                else:
                    if previous_reference_error[role] is not None:
                        map_reference_jitter[role].append(
                            abs(float(lateral_error) - previous_reference_error[role])
                        )
                    previous_reference_error[role] = float(lateral_error)
        result = {
            "frames": int(len(values)),
            "availability_rate": float(sum(value["available"] for value in values)) / denominator,
            "yellow_observed_rate": float(sum(value["yellow_observed"] for value in values)) / denominator,
            "structure_complete_rate": float(sum(value["structure_complete"] for value in values)) / denominator,
            "fully_observed_rate": float(sum(value["fully_observed"] for value in values)) / denominator,
            "map_supported_odd": {
                "frames": int(len(odd_values)),
                "availability_rate": float(
                    sum(value["available"] for value in odd_values)
                ) / odd_denominator,
                "yellow_observed_rate": float(
                    sum(value["yellow_observed"] for value in odd_values)
                ) / odd_denominator,
                "structure_complete_rate": float(
                    sum(value["structure_complete"] for value in odd_values)
                ) / odd_denominator,
                "fully_observed_rate": float(
                    sum(value["fully_observed"] for value in odd_values)
                ) / odd_denominator,
            },
            "longest_unavailable_frames": _longest_false_streak(value["available"] for value in values),
            "longest_incomplete_frames": _longest_false_streak(value["structure_complete"] for value in values),
            "residual_p95_frame_distribution_m": {
                "p50": _percentile(residual, 50),
                "p95": _percentile(residual, 95),
            },
            "map_p95_frame_distribution_m": {
                "p50": _percentile(map_distance, 50),
                "p95": _percentile(map_distance, 95),
            },
            "map_p95_by_role_m": {
                role: _percentile(metric, 95) for role, metric in map_by_role.items()
            },
            "map_p95_by_distance_bin_m": {
                label: _percentile(metric, 95) for label, metric in map_by_bin.items()
            },
            "map_evaluation_frames_by_role": dict(map_role_evaluation_counts),
            "map_reference_y_jitter_p95_m": {
                role: _percentile(metric, 95)
                for role, metric in map_reference_jitter.items()
            },
        }
        if method == "shared":
            result["yellow_structurally_available_rate"] = float(
                sum(value.get("yellow_structurally_available", False) for value in values)
            ) / denominator
            result["inference_frame_rate"] = float(
                sum(bool(value.get("inferred_roles")) for value in values)
            ) / denominator
            result["widths_m"] = {
                "left_p50": _percentile(widths_left, 50),
                "left_p95": _percentile(widths_left, 95),
                "right_p50": _percentile(widths_right, 50),
                "right_p95": _percentile(widths_right, 95),
            }
            result["support_audit"] = {
                "inlier_residual_p95_m": _percentile(inlier_residual, 95),
                "outlier_rate_p50": _percentile(outlier_rate, 50),
                "outlier_rate_p95": _percentile(outlier_rate, 95),
                "structure_inferred_fraction_p50": _percentile(
                    inferred_fraction, 50
                ),
                "structure_inferred_fraction_p95": _percentile(
                    inferred_fraction, 95
                ),
                "locally_fully_observed_fraction_p50": _percentile(
                    fully_observed_fraction, 50
                ),
                "valid_family_length_p50_m": _percentile(
                    valid_family_length, 50
                ),
                "pseudo_gt_disagreement_rate_p95": _percentile(
                    pseudo_gt_disagreement_rates, 95
                ),
                "pseudo_gt_disagreement_is_review_signal_only": True,
            }
        return result

    @staticmethod
    def _summarize_pose_alignment(rows):
        values = [row.get("ego_pose_alignment") or {} for row in rows]
        modes = Counter(value.get("mode", "missing") for value in values)
        nearest_age = [value.get("nearest_age_ms") for value in values]
        bracket = [value.get("bracket_span_ms") for value in values]
        translation = [
            (value.get("delta_from_nearest") or {}).get("translation_m")
            for value in values
        ]
        heading = [
            (value.get("delta_from_nearest") or {}).get("heading_deg")
            for value in values
        ]
        return {
            "mode_counts": dict(sorted(modes.items())),
            "nearest_age_ms": {
                "p50": _percentile(nearest_age, 50),
                "p95": _percentile(nearest_age, 95),
            },
            "bracket_span_ms": {
                "p50": _percentile(bracket, 50),
                "p95": _percentile(bracket, 95),
            },
            "delta_from_nearest": {
                "translation_m_p50": _percentile(translation, 50),
                "translation_m_p95": _percentile(translation, 95),
                "heading_deg_p50": _percentile(heading, 50),
                "heading_deg_p95": _percentile(heading, 95),
            },
            "offline_map_evaluation_only": True,
            "runtime_ego_motion_compensation": False,
        }

    @staticmethod
    def _summarize_invalid_map(rows):
        def summarize_alignment(name):
            values = {}
            bands = Counter()
            for row in rows:
                root = ((row.get("invalid_map_distance") or {}).get(name) or {})
                for category, category_value in root.get("categories", {}).items():
                    bands[(category, category_value.get("review_band", "unavailable"))] += 1
                    for role, metric in category_value.get("roles", {}).items():
                        key = (category, role)
                        value = values.setdefault(
                            key,
                            {"absolute_p50": [], "absolute_p95": [], "signed_p50": []},
                        )
                        value["absolute_p50"].append(metric.get("absolute_p50_m"))
                        value["absolute_p95"].append(metric.get("absolute_p95_m"))
                        value["signed_p50"].append(metric.get("signed_normal_p50_m"))
            return {
                "by_category_role": {
                    "{}/{}".format(category, role): {
                        "frames": len(value["absolute_p95"]),
                        "frame_absolute_p50_m_p50": _percentile(value["absolute_p50"], 50),
                        "frame_absolute_p95_m_p50": _percentile(value["absolute_p95"], 50),
                        "frame_absolute_p95_m_p95": _percentile(value["absolute_p95"], 95),
                        "signed_normal_p50_m_p50": _percentile(value["signed_p50"], 50),
                    }
                    for (category, role), value in sorted(values.items())
                },
                "review_band_frame_counts": {
                    "{}/{}".format(category, band): int(count)
                    for (category, band), count in sorted(bands.items())
                },
            }

        aligned = summarize_alignment("aligned")
        nearest = summarize_alignment("nearest_reference")
        deltas = []
        for row in rows:
            root = row.get("invalid_map_distance") or {}
            aligned_categories = (root.get("aligned") or {}).get("categories", {})
            nearest_categories = (root.get("nearest_reference") or {}).get("categories", {})
            for category in set(aligned_categories) & set(nearest_categories):
                aligned_p95 = aligned_categories[category].get("frame_absolute_p95_m")
                nearest_p95 = nearest_categories[category].get("frame_absolute_p95_m")
                if aligned_p95 is not None and nearest_p95 is not None:
                    deltas.append(float(aligned_p95) - float(nearest_p95))
        return {
            "aligned": aligned,
            "nearest_reference": nearest,
            "aligned_minus_nearest_frame_p95_m": {
                "p50": _percentile(deltas, 50),
                "p95": _percentile(deltas, 95),
                "absolute_p95": _percentile(np.abs(deltas), 95),
            },
            "interpretation": "descriptive_role_matched_map_pseudo_gt_review_only",
            "error_bands_m": [0.35, 0.60],
            "bands_are_acceptance_gates": False,
            "invalid_geometry_planning_authority": False,
        }

    def _finalize(self):
        if getattr(self, "finalized", False):
            return
        self.finalized = True
        try:
            with self.frames_path.open("w", encoding="utf-8") as stream:
                for row in self.rows:
                    stream.write(json.dumps(row, sort_keys=True) + "\n")
            valid = [row for row in self.rows if "error" not in row]
            processing = [row["processing_ms"] for row in valid]
            metrics = {
                "schema_version": SCHEMA_VERSION,
                "run_id": self.run_id,
                "frames": int(len(self.rows)),
                "valid_frames": int(len(valid)),
                "errors": dict(sorted(self.errors.items())),
                "processing_ms": {
                    "p50": _percentile(processing, 50),
                    "p95": _percentile(processing, 95),
                },
                "independent_role_id_switches": dict(
                    sorted(self.independent_id_switches.items())
                ),
                "methods": {
                    method: self._summarize_method(valid, method)
                    for method in METHODS
                },
                "invalid_support_overlay_review": {
                    "selected_overlays": int(len(self.invalid_review_manifest)),
                    "by_category": self.invalid_review_sampler.summary(),
                    "measurement_only": True,
                    "invalid_geometry_planning_authority": False,
                },
                "ego_pose_alignment": self._summarize_pose_alignment(valid),
                "invalid_map_distance": self._summarize_invalid_map(valid),
            }
            review_manifest_path = self.output_dir / "invalid_support_review.json"
            review_manifest_path.write_text(
                json.dumps(
                    {
                        "schema_version": "asmc-invalid-support-overlay-review-1.0.0",
                        "run_id": self.run_id,
                        "measurement_only": True,
                        "invalid_geometry_planning_authority": False,
                        "selection": {
                            "limit_per_category": (
                                self.invalid_review_sampler.limit_per_category
                            ),
                            "min_frame_gap": self.invalid_review_sampler.min_frame_gap,
                            "counts": self.invalid_review_sampler.summary(),
                        },
                        "overlays": self.invalid_review_manifest,
                    },
                    indent=2,
                    sort_keys=True,
                )
                + "\n",
                encoding="utf-8",
            )
            invalid_map_manifest_path = self.output_dir / "invalid_map_distance_review.json"
            invalid_map_manifest_path.write_text(
                json.dumps(
                    {
                        "schema_version": "asmc-invalid-map-distance-review-1.0.0",
                        "run_id": self.run_id,
                        "measurement_only": True,
                        "pseudo_gt_is_absolute_truth": False,
                        "bands_are_acceptance_gates": False,
                        "error_bands_m": [0.35, 0.60],
                        "invalid_geometry_planning_authority": False,
                        "selection": {
                            "limit_per_category_band": self.invalid_map_review_sampler.limit,
                            "min_frame_gap": self.invalid_map_review_sampler.min_frame_gap,
                            "counts": self.invalid_map_review_sampler.summary(),
                        },
                        "overlays": self.invalid_map_review_manifest,
                    },
                    indent=2,
                    sort_keys=True,
                )
                + "\n",
                encoding="utf-8",
            )
            metadata = {
                "schema_version": SCHEMA_VERSION,
                "status": "complete",
                "offline_only": True,
                "deployment_allowed": False,
                "started_at_utc": self.started_at,
                "finished_at_utc": _utc_now(),
                "elapsed_wall_s": time.monotonic() - self.started_monotonic,
                "run_id": self.run_id,
                "input_topics": [self.evidence_topic, self.ego_topic],
                "pseudo_gt": {
                    "path": str(self.pseudo_gt_path),
                    "sha256": _sha256(self.pseudo_gt_path),
                    "status": self.pseudo_gt.get("status"),
                    "deployment_allowed": self.pseudo_gt.get("deployment_allowed"),
                    "ego_heading_offset_deg": self.heading_offset_deg,
                    "disagreement_threshold_m": self.pseudo_gt_disagreement_m,
                    "interpretation": "review_signal_only_not_false_bridge_truth",
                },
                "configuration": self.config_snapshot,
                "research_basis": {
                    "LaneCPP": "continuous spline, tangent/normal physical priors",
                    "Differentiable_LSQ": "confidence-weighted least-squares fitting",
                    "LaneAF_StructLane": "lane-instance association and structural relations only",
                    "external_runtime_dependency": False,
                },
                "artifacts": {
                    "frames": self.frames_path.name,
                    "metrics": "metrics.json",
                    "overlays": "overlays/",
                    "invalid_support_review": review_manifest_path.name,
                    "invalid_map_distance_review": invalid_map_manifest_path.name,
                },
            }
            (self.output_dir / "metrics.json").write_text(
                json.dumps(metrics, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
            (self.output_dir / "metadata.json").write_text(
                json.dumps(metadata, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
            rospy.loginfo(
                "shadow lane-family A/B finalized frames=%d output=%s",
                len(self.rows),
                self.output_dir,
            )
        except Exception as exc:
            rospy.logerr("failed to finalize shared family audit: %s", exc)


def main():
    rospy.init_node("shadow_lane_family_offline")
    ShadowLaneFamilyOfflineNode()
    rospy.spin()


if __name__ == "__main__":
    main()
