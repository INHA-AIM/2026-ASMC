#!/usr/bin/env python3
"""Fuse three model-grid road-marking outputs into debug metric splines."""

from __future__ import annotations

import hashlib
import json
import threading
import time
from collections import deque
from dataclasses import dataclass
from functools import partial

import message_filters
import numpy as np
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from asmc_msgs.msg import (
    LaneBoundaryFamily,
    LaneBoundaryFamilyMember,
    LaneBoundaryObservation,
    LaneBoundaryObservationArray,
)
from geometry_msgs.msg import Point, Point32
from sensor_msgs import point_cloud2
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

from asmc_camera.bev.boundary_candidates import (
    TemporalBoundaryTracker,
    select_shadow_lane_tracks_with_audit,
    select_shadow_boundaries_with_audit,
)
from asmc_camera.bev.boundary_output import (
    BoundaryOutputFrame,
    COMPLETENESS_COMPLETE,
    COMPLETENESS_EMPTY,
    build_shadow_boundary_output,
)
from asmc_camera.bev.boundary_family_output import build_boundary_family_output
from asmc_camera.bev.calibration import calibrations_from_dict
from asmc_camera.bev.invalid_support import (
    classify_invalid_support,
    invalid_support_config_from_dict,
)
from asmc_camera.bev.lane_evidence import reduce_evidence_to_nodes
from asmc_camera.bev.lane_tracking import associate_lane_nodes_with_audit
from asmc_camera.bev.model_grid_projection import (
    MetricEvidence,
    ModelGridGroundProjector,
)
from asmc_camera.bev.pipeline_audit import PipelineAuditAccumulator
from asmc_camera.bev.planning_usability import (
    DEFAULT_PLANNING_ZONES,
    measure_planning_usability,
    planning_zones_from_config,
)
from asmc_camera.bev.shared_lane_family import (
    LEFT_ROLE,
    RIGHT_ROLE,
    YELLOW_ROLE,
    contiguous_sample_state_segments,
    fit_shared_lane_family_with_audit,
)
from asmc_camera.bev.spline_fitting import fit_lane_tracks_with_audit


from asmc_camera.runtime.ros_control import LaneEpochGate
from asmc_camera.runtime.latest_work import LatestWork
from asmc_camera.runtime.demand_inputs import DemandInputs, DemandSynchronizer


@dataclass(frozen=True)
class _ViewFrame:
    view: str
    stamp_ns: int
    header: object
    class_id: np.ndarray
    confidence: np.ndarray
    decode_copy_ms: float
    generation: object = None


RUNTIME_PROFILE_SCHEMA = "asmc-lane-runtime-profile-1.2.0"
RUNTIME_PROFILE_STAGES = (
    "input_decode_copy",
    "projection_audit",
    "projection_extract",
    "evidence_concat",
    "node_reduction",
    "association",
    "shadow_prefit",
    "shared_family_fit",
    "invalid_support_classification",
    "independent_spline_fit",
    "evidence_message_publish",
    "candidate_marker_publish",
    "shared_marker_publish",
    "shared_diagnostics_publish",
    "boundary_family_message_publish",
    "boundary_selection",
    "temporal_tracking",
    "boundary_output_build",
    "legacy_boundary_marker_publish",
    "typed_boundary_message_publish",
    "profiled_body",
    "pipeline_audit_update",
    "pipeline_audit_message_publish",
    "callback_total",
)


def _measure(timings, key, function):
    started = time.perf_counter()
    try:
        return function()
    finally:
        timings[key] = timings.get(key, 0.0) + (
            time.perf_counter() - started
        ) * 1000.0


def _names(value):
    values = (
        [item.strip() for item in value.split(",")]
        if isinstance(value, str)
        else [str(item).strip() for item in value]
    )
    values = [item for item in values if item]
    if not values or len(values) != len(set(values)):
        raise ValueError("lane vector views must be non-empty and unique")
    return values


class LaneVectorizerNode:
    def __init__(self) -> None:
        self.last_runtime_header = None
        self.runtime_generation = None
        self.epoch_gate = LaneEpochGate(on_standby=self._runtime_invalidate)
        self.views = _names(rospy.get_param("~views", ["front", "left", "right"]))
        camera_values = dict(rospy.get_param("~cameras"))
        cameras = calibrations_from_dict(
            {
                "ground_plane": rospy.get_param("~ground_plane"),
                "cameras": camera_values,
            }
        )
        self.ground_plane = next(iter(cameras.values())).ground_plane
        self.calibration_sha256 = str(rospy.get_param("~source_sha256", ""))
        missing = sorted(set(self.views) - set(cameras))
        if missing:
            raise ValueError("lane vector calibration missing views: {}".format(missing))
        model_hw = tuple(int(value) for value in rospy.get_param("~input_hw", rospy.get_param("~model_input_hw")))
        model_hw_by_view = dict(rospy.get_param("~input_hw_by_view", {}))
        if set(model_hw_by_view) - set(cameras):
            raise ValueError("model_input_hw_by_view contains an unknown camera")
        model_hw_by_view = {view: tuple(int(v) for v in model_hw_by_view.get(view, model_hw))
                            for view in self.views}
        source_masks = {
            str(name): int(value)
            for name, value in dict(rospy.get_param("~source_masks")).items()
        }
        if set(self.views) - set(source_masks):
            raise ValueError("source_masks must cover every enabled view")
        projector_kwargs = {
            "x_bounds_m": (
                float(rospy.get_param("~evidence_x_min_m")),
                float(rospy.get_param("~evidence_x_max_m")),
            ),
            "y_bounds_m": (
                float(rospy.get_param("~evidence_y_min_m")),
                float(rospy.get_param("~evidence_y_max_m")),
            ),
            "max_ground_range_m": float(rospy.get_param("~max_ground_range_m")),
            "min_camera_depth_m": float(rospy.get_param("~min_camera_depth_m")),
            "edge_taper_fraction": float(rospy.get_param("~edge_taper_fraction")),
            "forward_fade_start_m": float(
                rospy.get_param("~forward_fade_start_m")
            ),
        }
        self.projectors = {
            view: ModelGridGroundProjector(
                cameras[view], model_hw=model_hw_by_view[view],
                source_mask=source_masks[view], **projector_kwargs
            )
            for view in self.views
        }
        threshold_values = dict(rospy.get_param("~confidence_by_class"))
        self.confidence_by_class = {
            1: float(threshold_values["white"]),
            2: float(threshold_values["yellow"]),
        }
        self.node_parameters = {
            "x_bin_m": float(rospy.get_param("~x_bin_m")),
            "lateral_cluster_gap_m": float(
                rospy.get_param("~lateral_cluster_gap_m")
            ),
            "x_origin_m": float(rospy.get_param("~evidence_x_min_m")),
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
        self.shared_family_parameters = dict(
            rospy.get_param("~shared_lane_family", {})
        )
        self.shared_family_debug_enabled = bool(
            self.shared_family_parameters.pop("runtime_debug_enabled", False)
        )
        self.shared_family_deployment_allowed = bool(
            self.shared_family_parameters.pop("deployment_allowed", False)
        )
        self.shared_family_representation = str(
            self.shared_family_parameters.pop(
                "representation", "shared_parametric_bspline_normal_offsets_v2"
            )
        )
        self.boundary_family_candidate_enabled = bool(
            rospy.get_param("~enable_boundary_family_candidate", False)
        )
        self.independent_boundary_pipeline_enabled = bool(
            rospy.get_param("~enable_independent_boundary_pipeline", False)
        )
        self.publish_pipeline_audit = bool(
            rospy.get_param("~publish_pipeline_audit", False)
        )
        self.boundary_family_algorithm_id = str(
            rospy.get_param(
                "~boundary_family_algorithm_id",
                "twinlite_v6_shadow_boundary_family_candidate_v3",
            )
        ).strip()
        if not self.boundary_family_algorithm_id:
            raise ValueError("boundary family algorithm_id is required")
        self.shared_family_runtime_enabled = bool(
            self.shared_family_debug_enabled
            or self.boundary_family_candidate_enabled
        )
        # Kept for compatibility with the first offline-only configuration.
        self.shared_family_parameters.pop("evaluation_only", None)
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
        if self.shared_family_deployment_allowed:
            raise ValueError(
                "shared lane-family is a debug result and cannot be deployment_allowed"
            )
        self.shadow_boundary_parameters = dict(
            rospy.get_param("~shadow_boundary", {})
        )
        self.shadow_boundary_enabled = bool(
            self.shadow_boundary_parameters.pop("enabled", True)
        )
        self.temporal_boundary_parameters = {
            "max_lateral_residual_m": float(
                self.shadow_boundary_parameters.pop(
                    "temporal_max_lateral_residual_m", 0.65
                )
            ),
            "max_missed_frames": int(
                self.shadow_boundary_parameters.pop(
                    "temporal_max_missed_frames", 3
                )
            ),
            "reset_gap_sec": float(
                self.shadow_boundary_parameters.pop("temporal_reset_gap_sec", 0.75)
            ),
            "velocity_smoothing": float(
                self.shadow_boundary_parameters.pop(
                    "temporal_velocity_smoothing", 0.50
                )
            ),
            "max_lateral_speed_mps": float(
                self.shadow_boundary_parameters.pop(
                    "temporal_max_lateral_speed_mps", 4.0
                )
            ),
        }
        self.boundary_tracker = TemporalBoundaryTracker(
            **self.temporal_boundary_parameters
        )
        self.operating_domain = str(
            rospy.get_param("~operating_domain", "shadow_two_way_v1")
        ).strip()
        self.algorithm_id = str(
            rospy.get_param(
                "~algorithm_id", "twinlite_v6_shadow_boundary_contract_v1"
            )
        ).strip()
        self.expected_boundary_count = int(
            rospy.get_param("~expected_boundary_count", 3)
        )
        if not self.operating_domain or not self.algorithm_id:
            raise ValueError("boundary operating_domain and algorithm_id are required")
        if not 0 < self.expected_boundary_count <= 255:
            raise ValueError("expected_boundary_count must lie in [1,255]")
        boundary_contract_config = {
            "algorithm_id": self.algorithm_id,
            "operating_domain": self.operating_domain,
            "expected_boundary_count": self.expected_boundary_count,
            "model_input_hw": list(model_hw),
            "model_input_hw_by_view": model_hw_by_view,
            "source_masks": source_masks,
            "confidence_by_class": self.confidence_by_class,
            "node": self.node_parameters,
            "track": self.track_parameters,
            "spline": self.spline_parameters,
            "shadow_boundary": self.shadow_boundary_parameters,
            "temporal_boundary": self.temporal_boundary_parameters,
            "boundary_reference_policy": "nearest_shared_longitudinal_support_v1",
            "calibration_sha256": self.calibration_sha256,
            "ground_plane": {
                "frame_id": self.ground_plane.frame_id,
                "coefficients": self.ground_plane.coefficients,
                "source": self.ground_plane.source,
            },
        }
        self.boundary_config_sha256 = hashlib.sha256(
            json.dumps(
                boundary_contract_config,
                sort_keys=True,
                separators=(",", ":"),
            ).encode("utf-8")
        ).hexdigest()
        boundary_family_contract_config = {
            "algorithm_id": self.boundary_family_algorithm_id,
            "operating_domain": self.operating_domain,
            "expected_boundary_count": self.expected_boundary_count,
            "representation": self.shared_family_representation,
            "model_input_hw": list(model_hw),
            "model_input_hw_by_view": model_hw_by_view,
            "source_masks": source_masks,
            "confidence_by_class": self.confidence_by_class,
            "node": self.node_parameters,
            "track": self.track_parameters,
            "shadow_prefit": self.shadow_boundary_parameters,
            "shared_lane_family": self.shared_family_parameters,
            "calibration_sha256": self.calibration_sha256,
            "ground_plane": {
                "frame_id": self.ground_plane.frame_id,
                "coefficients": self.ground_plane.coefficients,
                "source": self.ground_plane.source,
            },
            "support_policy": (
                "bounded_distributed_same_frame_multi_role_completion_v3"
            ),
        }
        self.boundary_family_config_sha256 = hashlib.sha256(
            json.dumps(
                boundary_family_contract_config,
                sort_keys=True,
                separators=(",", ":"),
            ).encode("utf-8")
        ).hexdigest()
        self.frame_id = str(rospy.get_param("~frame_id", "base_link"))
        vehicle_footprint = dict(rospy.get_param("~vehicle_footprint", {}))
        self.vehicle_length_m = float(vehicle_footprint.get("length_m", 4.635))
        self.vehicle_width_m = float(vehicle_footprint.get("width_m", 1.892))
        self.vehicle_rear_overhang_m = float(
            vehicle_footprint.get("rear_overhang_m", 0.790)
        )
        self.vehicle_wheelbase_m = float(
            vehicle_footprint.get("wheelbase_m", 3.000)
        )
        self.vehicle_marker_height_m = float(
            vehicle_footprint.get("marker_height_m", 0.12)
        )
        if (
            self.vehicle_length_m <= 0.0
            or self.vehicle_width_m <= 0.0
            or not 0.0 <= self.vehicle_rear_overhang_m < self.vehicle_length_m
            or self.vehicle_wheelbase_m <= 0.0
            or self.vehicle_wheelbase_m
            > self.vehicle_length_m - self.vehicle_rear_overhang_m
            or self.vehicle_marker_height_m <= 0.0
        ):
            raise ValueError("vehicle_footprint parameters are invalid")
        self.max_skew_ns = int(
            round(float(rospy.get_param("~max_timestamp_skew_ms", 35.0)) * 1e6)
        )
        self.queue_size = int(rospy.get_param("~sync_queue_size", 12))
        if self.max_skew_ns < 0 or self.queue_size <= 0:
            raise ValueError("lane vector synchronization settings are invalid")

        self.buffers = {view: deque() for view in self.views}
        self.buffer_generation = None
        self.buffer_lock = threading.Lock()
        self.processing_lock = threading.Lock()
        self.fit_work = LatestWork()
        self.fit_stop = threading.Event()
        self.inputs = {view: 0 for view in self.views}
        self.pair_errors = {view: 0 for view in self.views}
        self.queue_drops = {view: 0 for view in self.views}
        self.sync_rejected = 0
        self.outputs = 0
        self.errors = 0
        self.last_skew_ms = 0.0
        self.last_counts = {}
        self.processing_ms = deque(maxlen=512)
        self.boundary_publish_ms = deque(maxlen=512)
        self.end_to_end_ms = deque(maxlen=512)
        self.output_times = deque(maxlen=256)
        self.pipeline_audit = (
            PipelineAuditAccumulator() if self.publish_pipeline_audit else None
        )
        self.active_candidate_marker_keys = set()
        self.active_boundary_marker_keys = set()
        self.active_shared_family_marker_keys = set()
        self.marker_delete_counts = {
            "candidate": 0,
            "boundary": 0,
            "shared_family": 0,
        }
        self.shared_family_fit_ms = deque(maxlen=512)
        self.runtime_stage_ms = {
            stage: deque(maxlen=512) for stage in RUNTIME_PROFILE_STAGES
        }
        self.shared_family_errors = 0
        self.boundary_family_outputs = 0
        self.boundary_family_invalid_outputs = 0
        self.last_shared_family_audit = {
            "accepted": False,
            "reason": "disabled",
        }

        self.evidence_publisher = rospy.Publisher(
            "/perception/lane/debug/evidence_points", PointCloud2, queue_size=1
        )
        self.marker_publisher = rospy.Publisher(
            "/perception/lane/debug/markers", MarkerArray, queue_size=1
        )
        self.legacy_boundary_publisher = rospy.Publisher(
            "/perception/lane/boundaries", MarkerArray, queue_size=1
        )
        self.boundary_observation_publisher = None
        if self.independent_boundary_pipeline_enabled:
            self.boundary_observation_publisher = rospy.Publisher(
                "/perception/lane/boundary_observations",
                LaneBoundaryObservationArray,
                queue_size=1,
            )
        self.boundary_family_publisher = rospy.Publisher(
            "/perception/lane/boundary_family",
            LaneBoundaryFamily,
            queue_size=1,
        )
        self.diagnostics_publisher = rospy.Publisher(
            "/perception/lane/diagnostics", DiagnosticArray, queue_size=1
        )
        self.pipeline_audit_publisher = None
        if self.publish_pipeline_audit:
            self.pipeline_audit_publisher = rospy.Publisher(
                "/perception/lane/pipeline_audit", DiagnosticArray, queue_size=10
            )
        self.shared_family_marker_publisher = rospy.Publisher(
            "/perception/lane/debug/shared_family/markers",
            MarkerArray,
            queue_size=1,
        )
        self.shared_family_diagnostics_publisher = rospy.Publisher(
            "/perception/lane/debug/shared_family/diagnostics",
            DiagnosticArray,
            queue_size=10,
        )

        self.fit_worker = threading.Thread(target=self._fit_worker, name='lane-fit', daemon=True)
        self.fit_worker.start()
        rospy.on_shutdown(self._shutdown)
        self.subscribers = []
        self.synchronizers = []
        for view in self.views:
            base = "/perception/camera/{}/road_marking/model_grid".format(view)
            group = DemandSynchronizer(
                [base + "/class_id", base + "/confidence"], Image,
                lambda: not self.epoch_gate.enabled or self.epoch_gate.mode != 0,
                partial(self._view_callback, view),
                context=self.epoch_gate.context,
                on_reset=self._reset_grid_inputs)
            self.subscribers.append(group)
        self.demand_inputs = DemandInputs(self.subscribers)
        interval = max(
            0.2, float(rospy.get_param("~diagnostics_interval_sec", 2.0))
        )
        self.diagnostics_timer = rospy.Timer(
            rospy.Duration(interval), self._publish_diagnostics
        )
        rospy.loginfo(
            "lane vector pilot ready views=%s model_hw=%s thresholds=%s "
            "skew<=%.1fms shared_family_debug=%s boundary_family_candidate=%s "
            "pipeline_audit=%s",
            self.views,
            model_hw,
            self.confidence_by_class,
            self.max_skew_ns / 1e6,
            self.shared_family_debug_enabled,
            self.boundary_family_candidate_enabled,
            self.publish_pipeline_audit,
        )

    def _reset_grid_inputs(self):
        self.fit_work.clear()
        with self.buffer_lock:
            for queue in self.buffers.values():
                queue.clear()

    def _runtime_invalidate(self):
        if hasattr(self, 'fit_work'):
            self.fit_work.clear()
        if hasattr(self, 'buffer_lock'):
            with self.buffer_lock:
                for queue in self.buffers.values():
                    queue.clear()
        if self.last_runtime_header is not None and hasattr(self, "boundary_family_publisher"):
            message = LaneBoundaryFamily()
            message.header = self.last_runtime_header
            message.valid = False
            message.status_reason = "inference_standby"
            self.boundary_family_publisher.publish(message)

    def _view_callback(self, view, class_message, confidence_message):
        # Do not wait for the fit's epoch lock in a transport callback. Tag
        # receipt now; the worker checks generation and cutoff under that lock.
        generation = self.epoch_gate.context()
        # The fit owns the gate lock for its whole calculation. Transport must
        # remain latest-only and never wait for that lock; _process rechecks.
        if self.epoch_gate.enabled and (self.epoch_gate.mode == 0 or
                class_message.header.stamp.to_nsec() <= self.epoch_gate.cutoff):
            return
        self.inputs[view] += 1
        stamps = (
            int(class_message.header.stamp.to_nsec()),
            int(confidence_message.header.stamp.to_nsec()),
        )
        if stamps[0] <= 0 or stamps[0] != stamps[1]:
            self.pair_errors[view] += 1
            return
        try:
            decode_started = time.perf_counter()
            projector = self.projectors[view]
            class_id = self._mono8(class_message, projector.model_hw)
            confidence = self._mono8(confidence_message, projector.model_hw)
            decode_copy_ms = (time.perf_counter() - decode_started) * 1000.0
        except Exception as exc:
            self.pair_errors[view] += 1
            rospy.logwarn_throttle(2.0, "%s lane grid rejected: %s", view, exc)
            return
        frame = _ViewFrame(
            view,
            stamps[0],
            class_message.header,
            class_id,
            confidence,
            decode_copy_ms,
            generation,
        )
        ready = None
        with self.buffer_lock:
            if generation != self.epoch_gate.context():
                return
            if generation != self.buffer_generation:
                for old_queue in self.buffers.values():
                    old_queue.clear()
                self.buffer_generation = generation
            queue = self.buffers[view]
            if len(queue) >= self.queue_size:
                queue.popleft()
                self.queue_drops[view] += 1
            queue.append(frame)
            ready = self._take_synchronized_locked()
        if ready is not None:
            self.fit_work.offer(ready)

    def _fit_worker(self):
        while not self.fit_stop.is_set():
            frames = self.fit_work.pop()
            if frames is None or self.fit_stop.is_set():
                return
            try:
                self._process(frames)
            except Exception as exc:
                if self.fit_stop.is_set() or rospy.is_shutdown():
                    return
                self.errors += 1
                rospy.logerr_throttle(2., 'lane-fit worker failed: %s', exc)

    def _shutdown(self):
        self.fit_stop.set()
        self.fit_work.close()
        if self.fit_worker is not threading.current_thread():
            self.fit_worker.join(timeout=2.)

    def _take_synchronized_locked(self):
        while all(self.buffers[view] for view in self.views):
            frames = [self.buffers[view][0] for view in self.views]
            stamps = [frame.stamp_ns for frame in frames]
            span = max(stamps) - min(stamps)
            if span <= self.max_skew_ns:
                self.last_skew_ms = span / 1e6
                return {
                    view: self.buffers[view].popleft() for view in self.views
                }
            oldest = self.views[int(np.argmin(stamps))]
            self.buffers[oldest].popleft()
            self.sync_rejected += 1
        return None

    def _process(self, frames):
        with self.processing_lock, self.epoch_gate.lock:
            if self.fit_stop.is_set():
                return
            if self.epoch_gate.enabled and any(f.generation != self.epoch_gate.context() for f in frames.values()):
                return
            if not all(self.epoch_gate.allows(f.header.stamp) for f in frames.values()):
                return
            if self.epoch_gate.enabled and self.runtime_generation != self.epoch_gate.context():
                self.boundary_tracker.reset()
                self.runtime_generation = self.epoch_gate.context()
            self.last_runtime_header = max(frames.values(), key=lambda f: f.stamp_ns).header
            started = time.perf_counter()
            family_candidate_published = False
            timings = {
                "input_decode_copy": float(
                    sum(frame.decode_copy_ms for frame in frames.values())
                )
            }
            frame_audit = {
                "projection": {},
                "association": {},
                "prefit": {},
                "spline": {},
                "boundary_selection": {},
                "temporal": {},
                "shared_family": {},
                "counts": {},
                "marker_policy": {
                    "candidate": "explicit_delete_missing_ids",
                    "boundary": "explicit_delete_missing_ids",
                    "canonical_boundary_message": "independent_of_marker_rendering",
                },
            }
            try:
                view_evidence = []
                for view in self.views:
                    projector = self.projectors[view]
                    if self.publish_pipeline_audit:
                        frame_audit["projection"][view] = _measure(
                            timings,
                            "projection_audit",
                            lambda projector=projector, view=view: projector.selection_audit(
                                frames[view].class_id,
                                frames[view].confidence,
                                self.confidence_by_class,
                            ),
                        )
                    view_evidence.append(
                        _measure(
                            timings,
                            "projection_extract",
                            lambda projector=projector, view=view: projector.extract(
                                frames[view].class_id,
                                frames[view].confidence,
                                self.confidence_by_class,
                            ),
                        )
                    )
                evidence = _measure(
                    timings,
                    "evidence_concat",
                    lambda: MetricEvidence.concatenate(view_evidence),
                )
                nodes = _measure(
                    timings,
                    "node_reduction",
                    lambda: reduce_evidence_to_nodes(
                        evidence, **self.node_parameters
                    ),
                )
                tracks, frame_audit["association"] = _measure(
                    timings,
                    "association",
                    lambda: associate_lane_nodes_with_audit(
                        nodes, **self.track_parameters
                    ),
                )
                if self.shadow_boundary_enabled:
                    fitted_tracks, frame_audit["prefit"] = _measure(
                        timings,
                        "shadow_prefit",
                        lambda: select_shadow_lane_tracks_with_audit(
                            tracks, self.shadow_boundary_parameters
                        ),
                    )
                else:
                    fitted_tracks = tracks
                    frame_audit["prefit"] = {
                        "input_tracks": int(len(tracks)),
                        "eligible_tracks": int(len(tracks)),
                        "selected_tracks": int(len(tracks)),
                        "rejected": {},
                    }
                shared_family = None
                if self.shared_family_runtime_enabled:
                    try:
                        shared_family, shared_audit = _measure(
                            timings,
                            "shared_family_fit",
                            lambda: fit_shared_lane_family_with_audit(
                                fitted_tracks, self.shared_family_parameters
                            ),
                        )
                    except Exception as exc:
                        self.shared_family_errors += 1
                        shared_audit = {
                            "accepted": False,
                            "reason": "runtime_error",
                            "error": "{}: {}".format(type(exc).__name__, exc),
                        }
                        rospy.logwarn_throttle(
                            2.0, "shared lane-family fit failed: %s", exc
                        )
                    shared_elapsed_ms = float(
                        timings.get("shared_family_fit", 0.0)
                    )
                    self.shared_family_fit_ms.append(shared_elapsed_ms)
                    shared_audit = dict(shared_audit)
                    shared_audit["fit_ms"] = shared_elapsed_ms
                    shared_audit["deployment_allowed"] = False
                    shared_audit["runtime_debug_enabled"] = bool(
                        self.shared_family_debug_enabled
                    )
                    shared_audit["candidate_output_enabled"] = bool(
                        self.boundary_family_candidate_enabled
                    )
                    planning_usability = measure_planning_usability(
                        shared_family,
                        zones=self.planning_zones,
                        anchor_tolerance_m=(
                            self.invalid_support_config.gap_tolerance_m
                        ),
                    )
                    shared_audit["invalid_support_classification"] = _measure(
                        timings,
                        "invalid_support_classification",
                        lambda: classify_invalid_support(
                            planning_usability, self.invalid_support_config
                        ),
                    )
                    frame_audit["shared_family"] = shared_audit
                    self.last_shared_family_audit = shared_audit
                newest_header = max(
                    frames.values(), key=lambda value: value.stamp_ns
                ).header
                header = Header(
                    stamp=newest_header.stamp,
                    frame_id=self.frame_id,
                )
                if self.boundary_family_candidate_enabled:
                    if self.fit_stop.is_set():
                        return
                    family_output = build_boundary_family_output(
                        shared_family,
                        status_reason=self.last_shared_family_audit.get(
                            "reason", "unknown"
                        ),
                        expected_boundary_count=self.expected_boundary_count,
                    )
                    _measure(
                        timings,
                        "boundary_family_message_publish",
                        lambda: self.boundary_family_publisher.publish(
                            self._boundary_family_message(family_output, header)
                        ),
                    )
                    family_candidate_published = True
                    self.boundary_family_outputs += 1
                    if not family_output.valid:
                        self.boundary_family_invalid_outputs += 1
                # Debug/audit subscribers must never activate another fitter.
                independent_pipeline_needed = self.independent_boundary_pipeline_enabled
                lanes = ()
                frame_audit["spline"] = {
                    "enabled": False,
                    "reason": "canonical_shared_family_only",
                }
                if independent_pipeline_needed:
                    lanes, frame_audit["spline"] = _measure(
                        timings,
                        "independent_spline_fit",
                        lambda: fit_lane_tracks_with_audit(
                            fitted_tracks, **self.spline_parameters
                        ),
                    )
                if self.evidence_publisher.get_num_connections() > 0:
                    _measure(
                        timings,
                        "evidence_message_publish",
                        lambda: self.evidence_publisher.publish(
                            self._point_cloud(evidence, header)
                        ),
                    )
                if self.marker_publisher.get_num_connections() > 0:
                    _measure(
                        timings,
                        "candidate_marker_publish",
                        lambda: self.marker_publisher.publish(
                            self._markers(lanes, header)
                        ),
                    )
                if self.shared_family_runtime_enabled:
                    if (
                        self.shared_family_debug_enabled
                        and self.shared_family_marker_publisher.get_num_connections() > 0
                    ):
                        _measure(
                            timings,
                            "shared_marker_publish",
                            lambda: self.shared_family_marker_publisher.publish(
                                self._shared_family_markers(shared_family, header)
                            ),
                        )
                    _measure(
                        timings,
                        "shared_diagnostics_publish",
                        lambda: self.shared_family_diagnostics_publisher.publish(
                            self._shared_family_diagnostics_message(
                                shared_family,
                                self.last_shared_family_audit,
                                header,
                            )
                        ),
                    )
                tracked_boundaries = ()
                boundary_output = BoundaryOutputFrame(
                    COMPLETENESS_EMPTY, self.expected_boundary_count, ()
                )
                if self.shadow_boundary_enabled and independent_pipeline_needed:
                    observations, frame_audit["boundary_selection"] = _measure(
                        timings,
                        "boundary_selection",
                        lambda: select_shadow_boundaries_with_audit(
                            lanes, self.shadow_boundary_parameters
                        ),
                    )
                    tracked_boundaries = _measure(
                        timings,
                        "temporal_tracking",
                        lambda: self.boundary_tracker.update(
                            observations,
                            max(frame.stamp_ns for frame in frames.values()),
                        ),
                    )
                    frame_audit["temporal"] = dict(self.boundary_tracker.last_audit)
                    boundary_output = _measure(
                        timings,
                        "boundary_output_build",
                        lambda: build_shadow_boundary_output(
                            tracked_boundaries,
                            preferred_reference_x_m=float(
                                self.shadow_boundary_parameters.get(
                                    "reference_x_m", 5.0
                                )
                            ),
                            expected_boundary_count=self.expected_boundary_count,
                        ),
                    )
                    if self.legacy_boundary_publisher.get_num_connections() > 0:
                        _measure(
                            timings,
                            "legacy_boundary_marker_publish",
                            lambda: self.legacy_boundary_publisher.publish(
                                self._boundary_markers(tracked_boundaries, header)
                            ),
                        )
                    if self.independent_boundary_pipeline_enabled:
                        publish_started = time.perf_counter()
                        _measure(
                            timings,
                            "typed_boundary_message_publish",
                            lambda: self.boundary_observation_publisher.publish(
                                self._boundary_observation_array(
                                    boundary_output, header, valid=True
                                )
                            ),
                        )
                        self.boundary_publish_ms.append(
                            (time.perf_counter() - publish_started) * 1000.0
                        )
            except Exception as exc:
                self.errors += 1
                timings["profiled_body"] = (
                    time.perf_counter() - started
                ) * 1000.0
                newest_header = max(
                    frames.values(), key=lambda value: value.stamp_ns
                ).header
                if self.publish_pipeline_audit:
                    frame_audit["error"] = "{}: {}".format(
                        type(exc).__name__, exc
                    )
                    frame_audit["counts"] = dict(
                        frame_audit.get("counts") or {}
                    )
                    self._publish_pipeline_audit(
                        frame_audit, timings, newest_header
                    )
                timings["callback_total"] = (
                    time.perf_counter() - started
                ) * 1000.0
                self._record_runtime_timings(timings)
                rospy.logerr_throttle(2.0, "lane vectorization failed: %s", exc)
                if (
                    self.boundary_family_candidate_enabled
                    and not family_candidate_published
                ):
                    try:
                        newest_header = max(
                            frames.values(), key=lambda value: value.stamp_ns
                        ).header
                        error_header = Header(
                            stamp=newest_header.stamp,
                            frame_id=self.frame_id,
                        )
                        error_output = build_boundary_family_output(
                            None,
                            status_reason="pipeline_error: {}".format(
                                type(exc).__name__
                            ),
                            expected_boundary_count=self.expected_boundary_count,
                        )
                        self.boundary_family_publisher.publish(
                            self._boundary_family_message(
                                error_output, error_header
                            )
                        )
                        self.boundary_family_outputs += 1
                        self.boundary_family_invalid_outputs += 1
                    except Exception as publish_exc:
                        rospy.logerr_throttle(
                            2.0,
                            "failed to publish invalid boundary-family candidate: %s",
                            publish_exc,
                        )
                if (
                    self.shadow_boundary_enabled
                    and self.independent_boundary_pipeline_enabled
                ):
                    newest_header = max(
                        frames.values(), key=lambda value: value.stamp_ns
                    ).header
                    error_header = Header(
                        stamp=newest_header.stamp,
                        frame_id=self.frame_id,
                    )
                    self.boundary_observation_publisher.publish(
                        self._boundary_observation_array(
                            BoundaryOutputFrame(
                                COMPLETENESS_EMPTY,
                                self.expected_boundary_count,
                                (),
                            ),
                            error_header,
                            valid=False,
                        )
                    )
                return
            elapsed_ms = (time.perf_counter() - started) * 1000.0
            timings["profiled_body"] = elapsed_ms
            self.processing_ms.append(elapsed_ms)
            self.output_times.append(time.monotonic())
            self.outputs += 1
            self.last_counts = {
                "evidence": int(len(evidence.xy)),
                "nodes": int(len(nodes)),
                "tracks": int(len(tracks)),
                "prefit_tracks": int(len(fitted_tracks)),
                "lanes": int(len(lanes)),
                "lane_candidates": int(len(lanes)),
                "white_lanes": int(sum(lane.class_id == 1 for lane in lanes)),
                "yellow_lanes": int(sum(lane.class_id == 2 for lane in lanes)),
                "shadow_boundaries": int(len(tracked_boundaries)),
                "shadow_yellow_boundaries": int(
                    sum(item.observation.lane.class_id == 2 for item in tracked_boundaries)
                ),
                "shadow_white_boundaries": int(
                    sum(item.observation.lane.class_id == 1 for item in tracked_boundaries)
                ),
                "shadow_corridor_complete": bool(
                    {item.observation.role for item in tracked_boundaries}
                    >= {
                        "yellow_centre_boundary",
                        "left_outer_boundary",
                        "right_outer_boundary",
                    }
                ),
                "boundary_completeness": int(boundary_output.completeness),
                "shared_family_available": bool(shared_family is not None),
                "shared_family_acceptance_mode": str(
                    self.last_shared_family_audit.get(
                        "acceptance_mode", "rejected"
                    )
                ),
                "shared_family_distributed": bool(
                    self.last_shared_family_audit.get("acceptance_mode")
                    == "distributed_multi_role"
                ),
                "shared_family_observed_roles": int(
                    len(self.last_shared_family_audit.get("observed_roles", []))
                    if self.shared_family_runtime_enabled
                    else 0
                ),
                "boundary_family_candidate_published": bool(
                    family_candidate_published
                ),
            }
            if self.publish_pipeline_audit:
                frame_audit["counts"] = {
                    "model_marking_pixels": int(
                        sum(
                            value["model_marking_pixels"]
                            for value in frame_audit["projection"].values()
                        )
                    ),
                    "evidence": int(len(evidence.xy)),
                    "nodes": int(len(nodes)),
                    "tracks": int(len(tracks)),
                    "prefit_tracks": int(len(fitted_tracks)),
                    "lanes": int(len(lanes)),
                    "observations": int(
                        frame_audit["boundary_selection"].get(
                            "selected_boundaries", 0
                        )
                    ),
                    "boundaries": int(len(tracked_boundaries)),
                    "yellow_boundaries": int(
                        sum(
                            item.observation.lane.class_id == 2
                            for item in tracked_boundaries
                        )
                    ),
                    "complete": bool(
                        boundary_output.completeness == COMPLETENESS_COMPLETE
                        and len(boundary_output.boundaries)
                        == self.expected_boundary_count
                    ),
                    "boundary_completeness": int(boundary_output.completeness),
                    "shared_family_available": bool(shared_family is not None),
                    "shared_family_distributed": bool(
                        self.last_shared_family_audit.get("acceptance_mode")
                        == "distributed_multi_role"
                    ),
                }
                newest_header = max(
                    frames.values(), key=lambda value: value.stamp_ns
                ).header
                self._publish_pipeline_audit(frame_audit, timings, newest_header)
            timings["callback_total"] = (
                time.perf_counter() - started
            ) * 1000.0
            self._record_runtime_timings(timings)
            completed_ns = int(rospy.Time.now().to_nsec())
            source_ns = max(frame.stamp_ns for frame in frames.values())
            age_ms = (completed_ns - source_ns) / 1e6
            if 0.0 <= age_ms <= 60000.0:
                self.end_to_end_ms.append(age_ms)

    def _record_runtime_timings(self, timings):
        for stage in RUNTIME_PROFILE_STAGES:
            value = float(timings.get(stage, 0.0))
            if np.isfinite(value) and value >= 0.0:
                self.runtime_stage_ms[stage].append(value)

    def _publish_pipeline_audit(self, frame_audit, timings, source_header):
        if not self.publish_pipeline_audit:
            return
        frame_audit["runtime_profile_schema"] = RUNTIME_PROFILE_SCHEMA
        frame_audit["timing_ms"] = dict(timings)
        audit_snapshot = _measure(
            timings,
            "pipeline_audit_update",
            lambda: self.pipeline_audit.update(frame_audit),
        )
        _measure(
            timings,
            "pipeline_audit_message_publish",
            lambda: self.pipeline_audit_publisher.publish(
                self._pipeline_audit_message(audit_snapshot, source_header)
            ),
        )

    def _point_cloud(self, evidence, source_header):
        header = Header(stamp=source_header.stamp, frame_id=self.frame_id)
        fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
            PointField("confidence", 12, PointField.FLOAT32, 1),
            PointField("geometric_quality", 16, PointField.FLOAT32, 1),
            PointField("class_id", 20, PointField.UINT8, 1),
            PointField("source_mask", 21, PointField.UINT8, 1),
        ]
        # ROS Noetic's point_cloud2 helper calls len(points); keep a concrete
        # list here instead of a generator. The evidence count is already
        # bounded by the configured model grid and debug ROI.
        points = [
            (
                float(evidence.xy[index, 0]),
                float(evidence.xy[index, 1]),
                0.0,
                float(evidence.confidence[index]),
                float(evidence.geometric_quality[index]),
                int(evidence.class_id[index]),
                int(evidence.source_mask[index]),
            )
            for index in range(len(evidence.xy))
        ]
        return point_cloud2.create_cloud(header, fields, points)

    def _markers(self, lanes, source_header):
        markers = []
        current_keys = set()
        for lane in lanes:
            marker = Marker()
            marker.header.stamp = source_header.stamp
            marker.header.frame_id = self.frame_id
            # Color-only Semantic evidence can also contain parking-box and
            # safety-zone markings.  Keep the debug namespace honest until a
            # lane-boundary discrimination gate has passed.
            marker.ns = (
                "white_candidate" if lane.class_id == 1 else "yellow_candidate"
            )
            marker.id = int(lane.lane_id)
            current_keys.add((marker.ns, marker.id))
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.08
            marker.color.a = float(np.clip(0.35 + 0.65 * lane.confidence, 0.0, 1.0))
            if lane.class_id == 1:
                marker.color.r = marker.color.g = marker.color.b = 1.0
            else:
                marker.color.r = marker.color.g = 1.0
                marker.color.b = 0.0
            marker.points = [
                Point(
                    x=float(x),
                    y=float(y),
                    z=float(self.ground_plane.xyz([[x, y]])[0, 2] + 0.03),
                )
                for x, y in lane.sample_points
            ]
            marker.text = json.dumps(
                {
                    "status": "lane_candidate",
                    "semantic_scope": "color_marking_not_boundary_instance",
                    "confidence": round(lane.confidence, 4),
                    "source_mask": lane.source_mask,
                    "valid_x": [
                        round(lane.valid_x_min_m, 3),
                        round(lane.valid_x_max_m, 3),
                    ],
                    "observed_length_m": round(lane.observed_length_m, 3),
                    "bridged_ratio": round(lane.bridged_length_ratio, 4),
                    "residual_p95_m": round(lane.residual_p95_m, 4),
                },
                sort_keys=True,
            )
            markers.append(marker)
        deleted = self.active_candidate_marker_keys - current_keys
        markers.extend(self._delete_markers(deleted, source_header))
        self.marker_delete_counts["candidate"] += len(deleted)
        self.active_candidate_marker_keys = current_keys
        return MarkerArray(markers=markers)

    def _boundary_markers(self, boundaries, source_header):
        markers = []
        current_keys = set()
        for tracked in boundaries:
            observation = tracked.observation
            lane = observation.lane
            marker = Marker()
            marker.header.stamp = source_header.stamp
            marker.header.frame_id = self.frame_id
            marker.ns = (
                "white_boundary" if lane.class_id == 1 else "yellow_boundary"
            )
            marker.id = int(tracked.stable_id)
            current_keys.add((marker.ns, marker.id))
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.11
            marker.color.a = float(np.clip(0.45 + 0.55 * observation.score, 0.0, 1.0))
            if lane.class_id == 1:
                marker.color.r = marker.color.g = marker.color.b = 1.0
            else:
                marker.color.r = marker.color.g = 1.0
                marker.color.b = 0.0
            marker.points = [
                Point(
                    x=float(x),
                    y=float(y),
                    z=float(self.ground_plane.xyz([[x, y]])[0, 2] + 0.05),
                )
                for x, y in lane.sample_points
            ]
            marker.text = json.dumps(
                {
                    "status": "provisional_shadow_boundary",
                    "semantic_scope": "boundary_geometry_without_home_or_opposing_label",
                    "stable_id": tracked.stable_id,
                    "lateral_order_left_to_right": observation.lateral_order_left_to_right,
                    "role": observation.role,
                    "class_id": lane.class_id,
                    "color": "white" if lane.class_id == 1 else "yellow",
                    "score": round(observation.score, 4),
                    "confidence": round(lane.confidence, 4),
                    "reference_y_m": round(observation.reference_y_m, 3),
                    "heading_deg": round(observation.heading_deg, 3),
                    "valid_x": [
                        round(lane.valid_x_min_m, 3),
                        round(lane.valid_x_max_m, 3),
                    ],
                    "source_mask": lane.source_mask,
                    "observed_length_m": round(lane.observed_length_m, 3),
                    "bridged_ratio": round(lane.bridged_length_ratio, 4),
                    "track_age_frames": tracked.age_frames,
                    "reacquired_after_missed_frames": tracked.reacquired_after_missed_frames,
                },
                sort_keys=True,
            )
            markers.append(marker)
        deleted = self.active_boundary_marker_keys - current_keys
        markers.extend(self._delete_markers(deleted, source_header))
        self.marker_delete_counts["boundary"] += len(deleted)
        self.active_boundary_marker_keys = current_keys
        return MarkerArray(markers=markers)

    def _shared_family_markers(self, family, source_header):
        """Render state-tagged shared-family segments without changing public output."""
        markers = self._vehicle_footprint_markers(source_header)
        current_keys = set()
        role_order = {
            LEFT_ROLE: 0,
            YELLOW_ROLE: 1,
            RIGHT_ROLE: 2,
        }
        if family is not None:
            for boundary in family.boundaries:
                state_indices = {"observed": 0, "structure_inferred": 0, "invalid": 0}
                for state, points in contiguous_sample_state_segments(
                    boundary.sample_points, boundary.sample_state
                ):
                    marker = Marker()
                    marker.header.stamp = source_header.stamp
                    marker.header.frame_id = self.frame_id
                    marker.ns = "shared_family/{}/{}".format(boundary.role, state)
                    marker.id = (
                        100 * role_order[boundary.role]
                        + 10 * {"observed": 0, "structure_inferred": 1, "invalid": 2}[state]
                        + state_indices[state]
                    )
                    state_indices[state] += 1
                    current_keys.add((marker.ns, marker.id))
                    marker.type = Marker.LINE_STRIP
                    marker.action = Marker.ADD
                    marker.pose.orientation.w = 1.0
                    if state == "observed":
                        marker.scale.x = 0.13
                        marker.color.a = 0.98
                        if boundary.class_id == 2:
                            marker.color.r = marker.color.g = 1.0
                            marker.color.b = 0.0
                        else:
                            marker.color.r = marker.color.g = marker.color.b = 1.0
                    elif state == "structure_inferred":
                        marker.scale.x = 0.10
                        marker.color.r = 1.0
                        marker.color.g = 0.55
                        marker.color.b = 0.0
                        marker.color.a = 0.92
                    else:
                        marker.scale.x = 0.06
                        marker.color.r = 1.0
                        marker.color.g = 0.0
                        marker.color.b = 0.20
                        marker.color.a = 0.55
                    marker.points = [
                        Point(
                            x=float(x),
                            y=float(y),
                            z=float(
                                self.ground_plane.xyz([[x, y]])[0, 2] + 0.08
                            ),
                        )
                        for x, y in points
                    ]
                    marker.text = json.dumps(
                        {
                            "status": state,
                            "role": boundary.role,
                            "class_id": boundary.class_id,
                            "confidence": round(boundary.confidence, 4),
                            "observed_length_m": round(
                                boundary.observed_length_m, 3
                            ),
                            "inferred_length_m": round(
                                boundary.inferred_length_m, 3
                            ),
                            "invalid_length_m": round(
                                boundary.invalid_length_m, 3
                            ),
                            "max_inferred_gap_m": round(
                                boundary.max_inferred_gap_m, 3
                            ),
                            "deployment_allowed": False,
                        },
                        sort_keys=True,
                    )
                    markers.append(marker)
        deleted = self.active_shared_family_marker_keys - current_keys
        markers.extend(self._delete_markers(deleted, source_header))
        self.marker_delete_counts["shared_family"] += len(deleted)
        self.active_shared_family_marker_keys = current_keys
        return MarkerArray(markers=markers)

    def _vehicle_footprint_markers(self, source_header):
        """Build a metric ego reference without affecting perception output."""
        rear_x = -self.vehicle_rear_overhang_m
        front_x = self.vehicle_length_m - self.vehicle_rear_overhang_m
        half_width = 0.5 * self.vehicle_width_m
        body_center_x = 0.5 * (rear_x + front_x)

        def ground_point(x, y, z_offset):
            ground_z = float(self.ground_plane.xyz([[x, y]])[0, 2])
            return Point(x=float(x), y=float(y), z=ground_z + z_offset)

        body = Marker()
        body.header.stamp = source_header.stamp
        body.header.frame_id = self.frame_id
        body.ns = "ego_vehicle_body"
        body.id = 0
        body.type = Marker.CUBE
        body.action = Marker.ADD
        body.pose.position.x = body_center_x
        body.pose.position.y = 0.0
        body.pose.position.z = float(
            self.ground_plane.xyz([[body_center_x, 0.0]])[0, 2]
            + 0.5 * self.vehicle_marker_height_m
        )
        body.pose.orientation.w = 1.0
        body.scale.x = self.vehicle_length_m
        body.scale.y = self.vehicle_width_m
        body.scale.z = self.vehicle_marker_height_m
        body.color.r = 0.15
        body.color.g = 0.55
        body.color.b = 0.95
        body.color.a = 0.28

        outline = Marker()
        outline.header = body.header
        outline.ns = "ego_vehicle_outline"
        outline.id = 0
        outline.type = Marker.LINE_STRIP
        outline.action = Marker.ADD
        outline.pose.orientation.w = 1.0
        outline.scale.x = 0.08
        outline.color.r = 0.25
        outline.color.g = 0.85
        outline.color.b = 1.0
        outline.color.a = 0.95
        outline.points = [
            ground_point(rear_x, half_width, 0.02),
            ground_point(front_x, half_width, 0.02),
            ground_point(front_x, -half_width, 0.02),
            ground_point(rear_x, -half_width, 0.02),
            ground_point(rear_x, half_width, 0.02),
        ]

        rear_axle = Marker()
        rear_axle.header = body.header
        rear_axle.ns = "ego_vehicle_rear_axle"
        rear_axle.id = 0
        rear_axle.type = Marker.LINE_STRIP
        rear_axle.action = Marker.ADD
        rear_axle.pose.orientation.w = 1.0
        rear_axle.scale.x = 0.10
        rear_axle.color.r = 1.0
        rear_axle.color.g = 0.35
        rear_axle.color.b = 0.10
        rear_axle.color.a = 1.0
        rear_axle.points = [
            ground_point(0.0, half_width, 0.04),
            ground_point(0.0, -half_width, 0.04),
        ]

        front_axle = Marker()
        front_axle.header = body.header
        front_axle.ns = "ego_vehicle_front_axle"
        front_axle.id = 0
        front_axle.type = Marker.LINE_STRIP
        front_axle.action = Marker.ADD
        front_axle.pose.orientation.w = 1.0
        front_axle.scale.x = 0.10
        front_axle.color.r = 0.25
        front_axle.color.g = 1.0
        front_axle.color.b = 0.35
        front_axle.color.a = 1.0
        front_axle.points = [
            ground_point(self.vehicle_wheelbase_m, half_width, 0.04),
            ground_point(self.vehicle_wheelbase_m, -half_width, 0.04),
        ]

        origin = Marker()
        origin.header = body.header
        origin.ns = "ego_vehicle_base_link"
        origin.id = 0
        origin.type = Marker.SPHERE
        origin.action = Marker.ADD
        origin.pose.position = ground_point(0.0, 0.0, 0.10)
        origin.pose.orientation.w = 1.0
        origin.scale.x = origin.scale.y = origin.scale.z = 0.22
        origin.color.r = 1.0
        origin.color.g = 0.18
        origin.color.b = 0.10
        origin.color.a = 1.0

        return [body, outline, rear_axle, front_axle, origin]

    def _shared_family_diagnostics_message(self, family, audit, source_header):
        status = DiagnosticStatus()
        status.name = "drivable_bev/shared_lane_family_debug"
        status.hardware_id = "camera_metric_lane"
        accepted = bool(family is not None and audit.get("accepted", False))
        runtime_error = audit.get("reason") == "runtime_error"
        status.level = (
            DiagnosticStatus.ERROR
            if runtime_error
            else DiagnosticStatus.OK
            if accepted
            else DiagnosticStatus.WARN
        )
        status.message = str(audit.get("reason", "unknown"))
        values = {
            "runtime_debug_enabled": self.shared_family_debug_enabled,
            "candidate_output_enabled": self.boundary_family_candidate_enabled,
            "candidate_topic": "/perception/lane/boundary_family",
            "candidate_algorithm_id": self.boundary_family_algorithm_id,
            "candidate_config_sha256": self.boundary_family_config_sha256,
            "deployment_allowed": False,
            "representation": self.shared_family_representation,
            "accepted": accepted,
            "reason": audit.get("reason", "unknown"),
            "acceptance_mode": audit.get("acceptance_mode", "rejected"),
            "fit_ms": audit.get("fit_ms"),
            "observed_roles": audit.get("observed_roles", []),
            "inferred_roles": audit.get("inferred_roles", []),
            "widths_m": audit.get("widths_m", {}),
            "data_residual_p95_m": audit.get("data_residual_p95_m"),
            "inlier_residual_p95_m": audit.get("inlier_residual_p95_m"),
            "outlier_rate": audit.get("outlier_rate"),
            "valid_x_m": audit.get("valid_x_m"),
            "boundary_support": audit.get("boundary_support", {}),
            "same_frame_completion": audit.get("same_frame_completion", {}),
            "distributed_support": audit.get("distributed_support", {}),
            "invalid_support_classification": audit.get(
                "invalid_support_classification", {}
            ),
            "invalid_default_planning_use_allowed": False,
            "error": audit.get("error"),
        }
        status.values = [self._value(key, value) for key, value in values.items()]
        message = DiagnosticArray()
        message.header.stamp = source_header.stamp
        message.status = [status]
        return message

    def _boundary_family_message(self, output, source_header):
        message = LaneBoundaryFamily()
        message.header.stamp = source_header.stamp
        message.header.frame_id = self.frame_id
        message.valid = bool(output.valid)
        message.deployment_allowed = False
        message.expected_boundary_count = int(output.expected_boundary_count)
        message.operating_domain = self.operating_domain
        message.algorithm_id = self.boundary_family_algorithm_id
        message.representation = self.shared_family_representation
        message.config_sha256 = self.boundary_family_config_sha256
        message.status_reason = str(output.status_reason)
        message.valid_x_min_m = float(output.valid_x_min_m)
        message.valid_x_max_m = float(output.valid_x_max_m)
        message.left_width_m = float(output.left_width_m)
        message.right_width_m = float(output.right_width_m)
        message.data_residual_p50_m = float(output.data_residual_p50_m)
        message.data_residual_p95_m = float(output.data_residual_p95_m)
        message.inlier_residual_p50_m = float(output.inlier_residual_p50_m)
        message.inlier_residual_p95_m = float(output.inlier_residual_p95_m)
        message.outlier_rate = float(output.outlier_rate)
        message.weighted_rms_m = float(output.weighted_rms_m)
        for value in output.boundaries:
            boundary = LaneBoundaryFamilyMember()
            boundary.role = int(value.role)
            boundary.color = int(value.color)
            boundary.source_mask = int(value.source_mask)
            boundary.overall_state = int(value.overall_state)
            boundary.confidence = float(value.confidence)
            boundary.residual_valid = bool(value.residual_valid)
            boundary.residual_p50_m = float(value.residual_p50_m)
            boundary.residual_p95_m = float(value.residual_p95_m)
            boundary.observed_length_m = float(value.observed_length_m)
            boundary.inferred_length_m = float(value.inferred_length_m)
            boundary.invalid_length_m = float(value.invalid_length_m)
            boundary.max_inferred_gap_m = float(value.max_inferred_gap_m)
            boundary.bridged_length_ratio = float(value.bridged_length_ratio)
            xyz = self.ground_plane.xyz(value.polyline_xy)
            boundary.polyline = [
                Point32(x=float(x), y=float(y), z=float(z)) for x, y, z in xyz
            ]
            boundary.support_state = [
                int(state) for state in value.support_state
            ]
            message.boundaries.append(boundary)
        return message

    def _delete_markers(self, keys, source_header):
        markers = []
        for namespace, marker_id in sorted(keys):
            marker = Marker()
            marker.header.stamp = source_header.stamp
            marker.header.frame_id = self.frame_id
            marker.ns = str(namespace)
            marker.id = int(marker_id)
            marker.action = Marker.DELETE
            markers.append(marker)
        return markers

    def _boundary_observation_array(self, output, source_header, valid):
        message = LaneBoundaryObservationArray()
        message.header.stamp = source_header.stamp
        message.header.frame_id = self.frame_id
        message.valid = bool(valid)
        message.completeness = int(output.completeness)
        message.expected_boundary_count = int(output.expected_boundary_count)
        message.operating_domain = self.operating_domain
        message.algorithm_id = self.algorithm_id
        message.config_sha256 = self.boundary_config_sha256
        for value in output.boundaries:
            boundary = LaneBoundaryObservation()
            boundary.tracking_id = int(value.tracking_id)
            boundary.lateral_order = int(value.lateral_order)
            boundary.order_valid = bool(value.order_valid)
            boundary.color = int(value.color)
            boundary.source_mask = int(value.source_mask)
            boundary.semantic_score = float(value.semantic_score)
            boundary.geometric_quality = float(value.geometric_quality)
            boundary.reference_x_m = float(value.reference_x_m)
            boundary.reference_y_m = float(value.reference_y_m)
            boundary.valid_x_min_m = float(value.valid_x_min_m)
            boundary.valid_x_max_m = float(value.valid_x_max_m)
            boundary.observed_length_m = float(value.observed_length_m)
            boundary.bridged_length_ratio = float(value.bridged_length_ratio)
            boundary.age_frames = int(value.age_frames)
            boundary.reacquired_after_missed_frames = int(
                value.reacquired_after_missed_frames
            )
            boundary.polyline = [
                Point32(
                    x=float(x),
                    y=float(y),
                    z=float(self.ground_plane.xyz([[x, y]])[0, 2]),
                )
                for x, y in value.polyline_xy
            ]
            message.boundaries.append(boundary)
        return message

    @staticmethod
    def _mono8(message, expected_shape):
        if message.encoding not in ("mono8", "8UC1"):
            raise ValueError("expected mono8 image")
        if (int(message.height), int(message.width)) != tuple(expected_shape):
            raise ValueError("model-grid shape differs from configured input")
        data = np.frombuffer(bytes(message.data), dtype=np.uint8)
        required = int(message.height) * int(message.step)
        if int(message.step) < int(message.width) or data.size < required:
            raise ValueError("model-grid image data is incomplete")
        return np.ascontiguousarray(
            data[:required].reshape(int(message.height), int(message.step))[
                :, : int(message.width)
            ]
        )

    @staticmethod
    def _value(key, value):
        if isinstance(value, (dict, list, tuple)):
            value = json.dumps(value, sort_keys=True)
        elif value is None:
            value = "unavailable"
        return KeyValue(key=str(key), value=str(value))

    def _pipeline_audit_message(self, snapshot, source_header):
        frame = snapshot.get("last_frame") or {}
        outcome = str(frame.get("primary_outcome", "unknown"))
        counts = frame.get("counts") or {}
        status = DiagnosticStatus()
        status.name = "drivable_bev/lane_pipeline_audit"
        status.hardware_id = "camera_metric_lane"
        if frame.get("error"):
            status.level = DiagnosticStatus.ERROR
        elif bool(counts.get("complete", False)):
            status.level = DiagnosticStatus.OK
        else:
            status.level = DiagnosticStatus.WARN
        status.message = outcome
        status.values = [
            self._value("primary_outcome", outcome),
            self._value("frame", frame),
            self._value("cumulative_outcomes", snapshot.get("outcomes", {})),
            self._value("cumulative_rejections", snapshot.get("rejections", {})),
            self._value(
                "current_streak_frames", snapshot.get("current_streak_frames", {})
            ),
            self._value(
                "longest_streak_frames", snapshot.get("longest_streak_frames", {})
            ),
            self._value("frames", snapshot.get("frames", 0)),
        ]
        message = DiagnosticArray()
        message.header.stamp = source_header.stamp
        message.status = [status]
        return message

    def _publish_diagnostics(self, _event):
        processing = np.asarray(self.processing_ms, dtype=np.float64)
        shared_processing = np.asarray(
            self.shared_family_fit_ms, dtype=np.float64
        )
        boundary_publish = np.asarray(self.boundary_publish_ms, dtype=np.float64)
        latency = np.asarray(self.end_to_end_ms, dtype=np.float64)
        fps = 0.0
        if len(self.output_times) >= 2:
            fps = (len(self.output_times) - 1) / max(
                self.output_times[-1] - self.output_times[0], 1e-9
            )
        runtime_stage_summary = {}
        for stage, samples in self.runtime_stage_ms.items():
            values = np.asarray(samples, dtype=np.float64)
            runtime_stage_summary[stage] = {
                "count": int(len(values)),
                "p50": float(np.percentile(values, 50)) if len(values) else None,
                "p95": float(np.percentile(values, 95)) if len(values) else None,
                "p99": float(np.percentile(values, 99)) if len(values) else None,
                "max": float(np.max(values)) if len(values) else None,
            }
        status = DiagnosticStatus()
        status.name = "drivable_bev/lane_vectorizer"
        status.hardware_id = "camera_metric_lane"
        status.level = DiagnosticStatus.ERROR if self.errors else DiagnosticStatus.OK
        status.message = "errors" if self.errors else "running"
        values = {
            "views": self.views,
            "inputs_by_view": self.inputs,
            "pair_errors_by_view": self.pair_errors,
            "queue_drops_by_view": self.queue_drops,
            "pending_fit_replacements": self.fit_work.replaced,
            "sync_rejected": self.sync_rejected,
            "outputs": self.outputs,
            "errors": self.errors,
            "processing_fps": round(fps, 3),
            "processing_p50_ms": float(np.percentile(processing, 50)) if len(processing) else None,
            "processing_p95_ms": float(np.percentile(processing, 95)) if len(processing) else None,
            "boundary_publish_p50_ms": float(np.percentile(boundary_publish, 50)) if len(boundary_publish) else None,
            "boundary_publish_p95_ms": float(np.percentile(boundary_publish, 95)) if len(boundary_publish) else None,
            "end_to_end_p50_ms": float(np.percentile(latency, 50)) if len(latency) else None,
            "end_to_end_p95_ms": float(np.percentile(latency, 95)) if len(latency) else None,
            "last_timestamp_skew_ms": round(self.last_skew_ms, 4),
            "last_counts": self.last_counts,
            "confidence_by_class": self.confidence_by_class,
            "shadow_boundary_enabled": self.shadow_boundary_enabled,
            "shadow_boundary_parameters": self.shadow_boundary_parameters,
            "temporal_boundary_parameters": self.temporal_boundary_parameters,
            "shared_family_debug_enabled": self.shared_family_debug_enabled,
            "shared_family_runtime_enabled": self.shared_family_runtime_enabled,
            "shared_family_deployment_allowed": False,
            "shared_family_representation": self.shared_family_representation,
            "shared_family_errors": self.shared_family_errors,
            "boundary_family_candidate_enabled": self.boundary_family_candidate_enabled,
            "boundary_family_topic": "/perception/lane/boundary_family",
            "independent_boundary_pipeline_enabled": (
                self.independent_boundary_pipeline_enabled
            ),
            "boundary_family_outputs": self.boundary_family_outputs,
            "boundary_family_invalid_outputs": self.boundary_family_invalid_outputs,
            "boundary_family_algorithm_id": self.boundary_family_algorithm_id,
            "boundary_family_config_sha256": self.boundary_family_config_sha256,
            "shared_family_fit_p50_ms": float(np.percentile(shared_processing, 50))
            if len(shared_processing)
            else None,
            "shared_family_fit_p95_ms": float(np.percentile(shared_processing, 95))
            if len(shared_processing)
            else None,
            "runtime_profile_schema": RUNTIME_PROFILE_SCHEMA,
            "runtime_stage_ms": runtime_stage_summary,
            "runtime_debug_subscribers": {
                "evidence_points": self.evidence_publisher.get_num_connections(),
                "candidate_markers": self.marker_publisher.get_num_connections(),
                "legacy_boundary_markers": self.legacy_boundary_publisher.get_num_connections(),
                "shared_family_markers": self.shared_family_marker_publisher.get_num_connections(),
                "boundary_family": self.boundary_family_publisher.get_num_connections(),
            },
            "last_shared_family_audit": self.last_shared_family_audit,
            "ground_plane_frame": self.ground_plane.frame_id,
            "ground_plane_coefficients": self.ground_plane.coefficients,
            "ground_plane_source": self.ground_plane.source,
            "calibration_sha256": self.calibration_sha256,
            "boundary_operating_domain": self.operating_domain,
            "boundary_algorithm_id": self.algorithm_id,
            "boundary_config_sha256": self.boundary_config_sha256,
            "boundary_expected_count": self.expected_boundary_count,
            "pipeline_audit_enabled": self.publish_pipeline_audit,
            "pipeline_audit": (
                self.pipeline_audit.snapshot()
                if self.pipeline_audit is not None
                else {"enabled": False}
            ),
            "marker_policy": "explicit_delete_missing_ids",
            "marker_delete_counts": self.marker_delete_counts,
        }
        status.values = [self._value(key, value) for key, value in values.items()]
        message = DiagnosticArray()
        message.header.stamp = rospy.Time.now()
        message.status = [status]
        self.diagnostics_publisher.publish(message)


def main():
    rospy.init_node("camera_lane_vectorizer")
    LaneVectorizerNode()
    rospy.spin()


if __name__ == "__main__":
    main()
