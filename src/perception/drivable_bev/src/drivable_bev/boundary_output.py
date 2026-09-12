"""Typed boundary-output values independent of ROS message generation."""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Sequence, Tuple

import numpy as np

from .boundary_candidates import TrackedBoundary, lane_y_at_x


COLOR_UNKNOWN = 0
COLOR_WHITE = 1
COLOR_YELLOW = 2

COMPLETENESS_EMPTY = 0
COMPLETENESS_PARTIAL = 1
COMPLETENESS_COMPLETE = 2

_COMPLETE_SHADOW_ROLES = {
    "left_outer_boundary",
    "yellow_centre_boundary",
    "right_outer_boundary",
}


@dataclass(frozen=True)
class BoundaryOutputValue:
    tracking_id: int
    lateral_order: int
    order_valid: bool
    color: int
    source_mask: int
    semantic_score: float
    geometric_quality: float
    reference_x_m: float
    reference_y_m: float
    valid_x_min_m: float
    valid_x_max_m: float
    observed_length_m: float
    bridged_length_ratio: float
    age_frames: int
    reacquired_after_missed_frames: int
    polyline_xy: np.ndarray


@dataclass(frozen=True)
class BoundaryOutputFrame:
    completeness: int
    expected_boundary_count: int
    boundaries: Tuple[BoundaryOutputValue, ...]


def _color_from_class(class_id: int) -> int:
    if int(class_id) == 1:
        return COLOR_WHITE
    if int(class_id) == 2:
        return COLOR_YELLOW
    return COLOR_UNKNOWN


def _ordered_polyline(points) -> np.ndarray:
    value = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    if len(value) < 2:
        raise ValueError("boundary polyline needs at least two points")
    if not np.isfinite(value).all():
        raise ValueError("boundary polyline contains non-finite values")
    value = value[np.argsort(value[:, 0], kind="mergesort")]
    if np.any(np.diff(value[:, 0]) < -1e-9):
        raise ValueError("boundary polyline must be ordered by increasing x")
    return np.ascontiguousarray(value, dtype=np.float32)


def build_shadow_boundary_output(
    tracked_boundaries: Sequence[TrackedBoundary],
    preferred_reference_x_m: float,
    expected_boundary_count: int = 3,
) -> BoundaryOutputFrame:
    """Convert shadow-ODD tracks to a general boundary observation payload.

    Internal selector roles are used only to determine current ODD completeness.
    They are intentionally not copied into the public output value.
    """
    if not np.isfinite(preferred_reference_x_m):
        raise ValueError("preferred boundary reference x must be finite")
    if int(expected_boundary_count) <= 0 or int(expected_boundary_count) > 255:
        raise ValueError("expected boundary count must lie in [1,255]")

    ordered = sorted(
        tracked_boundaries,
        key=lambda item: item.observation.lateral_order_left_to_right,
    )
    tracking_ids = [int(item.stable_id) for item in ordered]
    lateral_orders = [
        int(item.observation.lateral_order_left_to_right) for item in ordered
    ]
    if len(tracking_ids) != len(set(tracking_ids)):
        raise ValueError("boundary tracking IDs must be unique per frame")
    if lateral_orders != list(range(len(lateral_orders))):
        raise ValueError("boundary lateral order must be contiguous and zero-based")

    values = []
    all_cross_preferred_reference = True
    for item in ordered:
        observation = item.observation
        lane = observation.lane
        lower = float(lane.valid_x_min_m)
        upper = float(lane.valid_x_max_m)
        if not np.isfinite([lower, upper]).all() or upper < lower:
            raise ValueError("boundary valid x range is invalid")
        crosses_reference = lower <= float(preferred_reference_x_m) <= upper
        all_cross_preferred_reference &= crosses_reference
        reference_x = float(np.clip(preferred_reference_x_m, lower, upper))
        reference_y = float(lane_y_at_x(lane, reference_x))
        if not np.isfinite(reference_y):
            raise ValueError("boundary reference point is invalid")
        values.append(
            BoundaryOutputValue(
                tracking_id=int(item.stable_id),
                lateral_order=int(observation.lateral_order_left_to_right),
                order_valid=False,
                color=_color_from_class(lane.class_id),
                source_mask=int(lane.source_mask),
                semantic_score=float(np.clip(lane.confidence, 0.0, 1.0)),
                geometric_quality=float(np.clip(observation.score, 0.0, 1.0)),
                reference_x_m=reference_x,
                reference_y_m=reference_y,
                valid_x_min_m=lower,
                valid_x_max_m=upper,
                observed_length_m=float(lane.observed_length_m),
                bridged_length_ratio=float(
                    np.clip(lane.bridged_length_ratio, 0.0, 1.0)
                ),
                age_frames=int(item.age_frames),
                reacquired_after_missed_frames=int(
                    item.reacquired_after_missed_frames
                ),
                polyline_xy=_ordered_polyline(lane.sample_points),
            )
        )

    values = tuple(
        replace(value, order_valid=bool(all_cross_preferred_reference))
        for value in values
    )
    roles = {item.observation.role for item in ordered}
    if not values:
        completeness = COMPLETENESS_EMPTY
    elif _COMPLETE_SHADOW_ROLES.issubset(roles):
        completeness = COMPLETENESS_COMPLETE
    else:
        completeness = COMPLETENESS_PARTIAL
    return BoundaryOutputFrame(
        completeness=completeness,
        expected_boundary_count=int(expected_boundary_count),
        boundaries=values,
    )
