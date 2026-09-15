"""Camera semantic projection into a metric ego-frame BEV."""

from .calibration import (
    CameraCalibration,
    GroundPlane,
    calibration_from_dict,
    calibrations_from_dict,
    ground_plane_from_dict,
    load_calibration_snapshot,
    validate_sensor_set_snapshot,
)
from .grid import BevGridSpec
from .fusion import (
    CameraBevFusion,
    FusedSemantic,
    build_quality_maps,
    timestamp_span_ns,
    timestamps_within_slop,
)
from .projector import CameraBevProjector, ProjectedSemantic
from .model_grid_projection import MetricEvidence, ModelGridGroundProjector
from .lane_evidence import LaneNode, reduce_evidence_to_nodes
from .lane_tracking import LaneTrack, associate_lane_nodes
from .spline_fitting import FittedLane, fit_lane_track, fit_lane_tracks
from .boundary_candidates import (
    BoundaryObservation,
    TemporalBoundaryTracker,
    TrackedBoundary,
    select_shadow_lane_tracks,
    select_shadow_boundaries,
)
from .boundary_output import (
    BoundaryOutputFrame,
    BoundaryOutputValue,
    build_shadow_boundary_output,
)
from .boundary_family_output import (
    BoundaryFamilyMemberOutput,
    BoundaryFamilyOutputFrame,
    build_boundary_family_output,
)
from .invalid_support import (
    DEFAULT_INVALID_SUPPORT_CONFIG,
    InvalidSupportClassificationConfig,
    classify_invalid_support,
    invalid_support_config_from_dict,
)
from .invalid_support_review import (
    HEALTHY_REVIEW_CATEGORY,
    INVALID_REVIEW_CATEGORIES,
    InvalidSupportReviewSampler,
    invalid_support_review_categories,
)
from .invalid_map_audit import (
    ERROR_BANDS_M,
    InvalidMapReviewSampler,
    invalid_category_masks,
    invalid_distance_band,
)
from .pose_time_alignment import EgoPoseAlignment, align_ego_pose, pose_delta
from .bag_batch import (
    SelectedBagRun,
    diagnostic_int,
    diagnostic_values,
    load_bag_selection,
    result_quality_gates,
    topic_message_count,
)
from .planning_usability import (
    DEFAULT_PLANNING_ZONES,
    LongitudinalZone,
    measure_planning_usability,
    planning_zones_from_config,
)
from .shadow_result_audit import (
    robustness_delta,
    summarize_boundary_frames,
    summarize_boundary_run_groups,
    typed_boundary_frame,
)

__all__ = [
    "BevGridSpec",
    "CameraCalibration",
    "GroundPlane",
    "CameraBevProjector",
    "CameraBevFusion",
    "FusedSemantic",
    "ProjectedSemantic",
    "MetricEvidence",
    "ModelGridGroundProjector",
    "LaneNode",
    "LaneTrack",
    "FittedLane",
    "BoundaryObservation",
    "TrackedBoundary",
    "TemporalBoundaryTracker",
    "SelectedBagRun",
    "reduce_evidence_to_nodes",
    "associate_lane_nodes",
    "fit_lane_track",
    "fit_lane_tracks",
    "select_shadow_boundaries",
    "BoundaryOutputFrame",
    "BoundaryOutputValue",
    "build_shadow_boundary_output",
    "BoundaryFamilyMemberOutput",
    "BoundaryFamilyOutputFrame",
    "build_boundary_family_output",
    "DEFAULT_INVALID_SUPPORT_CONFIG",
    "InvalidSupportClassificationConfig",
    "classify_invalid_support",
    "invalid_support_config_from_dict",
    "HEALTHY_REVIEW_CATEGORY",
    "INVALID_REVIEW_CATEGORIES",
    "InvalidSupportReviewSampler",
    "invalid_support_review_categories",
    "ERROR_BANDS_M",
    "InvalidMapReviewSampler",
    "invalid_category_masks",
    "invalid_distance_band",
    "EgoPoseAlignment",
    "align_ego_pose",
    "pose_delta",
    "DEFAULT_PLANNING_ZONES",
    "LongitudinalZone",
    "measure_planning_usability",
    "planning_zones_from_config",
    "select_shadow_lane_tracks",
    "load_bag_selection",
    "diagnostic_int",
    "diagnostic_values",
    "result_quality_gates",
    "topic_message_count",
    "robustness_delta",
    "summarize_boundary_frames",
    "summarize_boundary_run_groups",
    "typed_boundary_frame",
    "calibration_from_dict",
    "calibrations_from_dict",
    "ground_plane_from_dict",
    "build_quality_maps",
    "load_calibration_snapshot",
    "validate_sensor_set_snapshot",
    "timestamp_span_ns",
    "timestamps_within_slop",
]
