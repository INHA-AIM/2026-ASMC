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
from asmc_msgs.msg import LaneBoundaryObservation, LaneBoundaryObservationArray
from geometry_msgs.msg import Point, Point32
from sensor_msgs import point_cloud2
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

from drivable_bev.boundary_candidates import (
    TemporalBoundaryTracker,
    select_shadow_lane_tracks,
    select_shadow_boundaries,
)
from drivable_bev.boundary_output import (
    BoundaryOutputFrame,
    COMPLETENESS_EMPTY,
    build_shadow_boundary_output,
)
from drivable_bev.calibration import calibrations_from_dict
from drivable_bev.lane_evidence import reduce_evidence_to_nodes
from drivable_bev.lane_tracking import associate_lane_nodes
from drivable_bev.model_grid_projection import (
    MetricEvidence,
    ModelGridGroundProjector,
)
from drivable_bev.spline_fitting import fit_lane_tracks


@dataclass(frozen=True)
class _ViewFrame:
    view: str
    stamp_ns: int
    header: object
    class_id: np.ndarray
    confidence: np.ndarray


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
        model_hw = tuple(int(value) for value in rospy.get_param("~model_input_hw"))
        source_masks = {
            str(name): int(value)
            for name, value in dict(rospy.get_param("~source_masks")).items()
        }
        if set(self.views) - set(source_masks):
            raise ValueError("source_masks must cover every enabled view")
        projector_kwargs = {
            "model_hw": model_hw,
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
                cameras[view], source_mask=source_masks[view], **projector_kwargs
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
        self.shadow_boundary_parameters = dict(
            rospy.get_param("~shadow_boundary", {})
        )
        self.shadow_boundary_enabled = bool(
            self.shadow_boundary_parameters.pop("enabled", True)
        )
        self.boundary_tracker = TemporalBoundaryTracker(
            max_lateral_residual_m=float(
                self.shadow_boundary_parameters.pop(
                    "temporal_max_lateral_residual_m", 0.65
                )
            ),
            max_missed_frames=int(
                self.shadow_boundary_parameters.pop(
                    "temporal_max_missed_frames", 3
                )
            ),
            reset_gap_sec=float(
                self.shadow_boundary_parameters.pop("temporal_reset_gap_sec", 0.75)
            ),
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
            "source_masks": source_masks,
            "confidence_by_class": self.confidence_by_class,
            "node": self.node_parameters,
            "track": self.track_parameters,
            "spline": self.spline_parameters,
            "shadow_boundary": self.shadow_boundary_parameters,
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
        self.frame_id = str(rospy.get_param("~frame_id", "base_link"))
        self.max_skew_ns = int(
            round(float(rospy.get_param("~max_timestamp_skew_ms", 35.0)) * 1e6)
        )
        self.queue_size = int(rospy.get_param("~sync_queue_size", 12))
        if self.max_skew_ns < 0 or self.queue_size <= 0:
            raise ValueError("lane vector synchronization settings are invalid")

        self.buffers = {view: deque() for view in self.views}
        self.buffer_lock = threading.Lock()
        self.processing_lock = threading.Lock()
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

        self.evidence_publisher = rospy.Publisher(
            "/perception/lane/debug/evidence_points", PointCloud2, queue_size=1
        )
        self.marker_publisher = rospy.Publisher(
            "/perception/lane/debug/markers", MarkerArray, queue_size=1
        )
        self.legacy_boundary_publisher = rospy.Publisher(
            "/perception/lane/boundaries", MarkerArray, queue_size=1
        )
        self.boundary_observation_publisher = rospy.Publisher(
            "/perception/lane/boundary_observations",
            LaneBoundaryObservationArray,
            queue_size=1,
        )
        self.diagnostics_publisher = rospy.Publisher(
            "/perception/lane/diagnostics", DiagnosticArray, queue_size=1
        )

        self.subscribers = []
        self.synchronizers = []
        for view in self.views:
            base = "/perception/camera/{}/road_marking/model_grid".format(view)
            class_id = message_filters.Subscriber(
                base + "/class_id", Image, queue_size=2
            )
            confidence = message_filters.Subscriber(
                base + "/confidence", Image, queue_size=2
            )
            synchronizer = message_filters.TimeSynchronizer(
                [class_id, confidence], queue_size=4
            )
            synchronizer.registerCallback(partial(self._view_callback, view))
            self.subscribers.extend([class_id, confidence])
            self.synchronizers.append(synchronizer)
        interval = max(
            0.2, float(rospy.get_param("~diagnostics_interval_sec", 2.0))
        )
        self.diagnostics_timer = rospy.Timer(
            rospy.Duration(interval), self._publish_diagnostics
        )
        rospy.loginfo(
            "lane vector pilot ready views=%s model_hw=%s thresholds=%s skew<=%.1fms",
            self.views,
            model_hw,
            self.confidence_by_class,
            self.max_skew_ns / 1e6,
        )

    def _view_callback(self, view, class_message, confidence_message):
        self.inputs[view] += 1
        stamps = (
            int(class_message.header.stamp.to_nsec()),
            int(confidence_message.header.stamp.to_nsec()),
        )
        if stamps[0] <= 0 or stamps[0] != stamps[1]:
            self.pair_errors[view] += 1
            return
        try:
            projector = self.projectors[view]
            class_id = self._mono8(class_message, projector.model_hw)
            confidence = self._mono8(confidence_message, projector.model_hw)
        except Exception as exc:
            self.pair_errors[view] += 1
            rospy.logwarn_throttle(2.0, "%s lane grid rejected: %s", view, exc)
            return
        frame = _ViewFrame(
            view, stamps[0], class_message.header, class_id, confidence
        )
        ready = None
        with self.buffer_lock:
            queue = self.buffers[view]
            if len(queue) >= self.queue_size:
                queue.popleft()
                self.queue_drops[view] += 1
            queue.append(frame)
            ready = self._take_synchronized_locked()
        if ready is not None:
            self._process(ready)

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
        with self.processing_lock:
            started = time.perf_counter()
            try:
                evidence = MetricEvidence.concatenate(
                    [
                        self.projectors[view].extract(
                            frames[view].class_id,
                            frames[view].confidence,
                            self.confidence_by_class,
                        )
                        for view in self.views
                    ]
                )
                nodes = reduce_evidence_to_nodes(evidence, **self.node_parameters)
                tracks = associate_lane_nodes(nodes, **self.track_parameters)
                fitted_tracks = (
                    select_shadow_lane_tracks(tracks, self.shadow_boundary_parameters)
                    if self.shadow_boundary_enabled
                    else tracks
                )
                lanes = fit_lane_tracks(fitted_tracks, **self.spline_parameters)
                newest_header = max(
                    frames.values(), key=lambda value: value.stamp_ns
                ).header
                header = Header(
                    stamp=newest_header.stamp,
                    frame_id=self.frame_id,
                )
                if self.evidence_publisher.get_num_connections() > 0:
                    self.evidence_publisher.publish(self._point_cloud(evidence, header))
                if self.marker_publisher.get_num_connections() > 0:
                    self.marker_publisher.publish(self._markers(lanes, header))
                tracked_boundaries = ()
                boundary_output = BoundaryOutputFrame(
                    COMPLETENESS_EMPTY, self.expected_boundary_count, ()
                )
                if self.shadow_boundary_enabled:
                    observations = select_shadow_boundaries(
                        lanes, self.shadow_boundary_parameters
                    )
                    tracked_boundaries = self.boundary_tracker.update(
                        observations, max(frame.stamp_ns for frame in frames.values())
                    )
                    boundary_output = build_shadow_boundary_output(
                        tracked_boundaries,
                        preferred_reference_x_m=float(
                            self.shadow_boundary_parameters.get("reference_x_m", 5.0)
                        ),
                        expected_boundary_count=self.expected_boundary_count,
                    )
                    if self.legacy_boundary_publisher.get_num_connections() > 0:
                        self.legacy_boundary_publisher.publish(
                            self._boundary_markers(tracked_boundaries, header)
                        )
                    publish_started = time.perf_counter()
                    self.boundary_observation_publisher.publish(
                        self._boundary_observation_array(
                            boundary_output, header, valid=True
                        )
                    )
                    self.boundary_publish_ms.append(
                        (time.perf_counter() - publish_started) * 1000.0
                    )
            except Exception as exc:
                self.errors += 1
                rospy.logerr_throttle(2.0, "lane vectorization failed: %s", exc)
                if self.shadow_boundary_enabled:
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
            }
            completed_ns = int(rospy.Time.now().to_nsec())
            source_ns = max(frame.stamp_ns for frame in frames.values())
            age_ms = (completed_ns - source_ns) / 1e6
            if 0.0 <= age_ms <= 60000.0:
                self.end_to_end_ms.append(age_ms)

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
        delete = Marker()
        delete.header.stamp = source_header.stamp
        delete.header.frame_id = self.frame_id
        delete.action = Marker.DELETEALL
        markers = [delete]
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
        return MarkerArray(markers=markers)

    def _boundary_markers(self, boundaries, source_header):
        delete = Marker()
        delete.header.stamp = source_header.stamp
        delete.header.frame_id = self.frame_id
        delete.action = Marker.DELETEALL
        markers = [delete]
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
        return MarkerArray(markers=markers)

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

    def _publish_diagnostics(self, _event):
        processing = np.asarray(self.processing_ms, dtype=np.float64)
        boundary_publish = np.asarray(self.boundary_publish_ms, dtype=np.float64)
        latency = np.asarray(self.end_to_end_ms, dtype=np.float64)
        fps = 0.0
        if len(self.output_times) >= 2:
            fps = (len(self.output_times) - 1) / max(
                self.output_times[-1] - self.output_times[0], 1e-9
            )
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
            "ground_plane_frame": self.ground_plane.frame_id,
            "ground_plane_coefficients": self.ground_plane.coefficients,
            "ground_plane_source": self.ground_plane.source,
            "calibration_sha256": self.calibration_sha256,
            "boundary_operating_domain": self.operating_domain,
            "boundary_algorithm_id": self.algorithm_id,
            "boundary_config_sha256": self.boundary_config_sha256,
            "boundary_expected_count": self.expected_boundary_count,
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
