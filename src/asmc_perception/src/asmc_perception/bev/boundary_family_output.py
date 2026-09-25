"""Transport-safe values for the provisional shared boundary-family output."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

from .shared_lane_family import (
    LEFT_ROLE,
    OBSERVED,
    RIGHT_ROLE,
    ROLE_ORDER,
    SAMPLE_INVALID,
    SAMPLE_OBSERVED,
    SAMPLE_STRUCTURE_INFERRED,
    STRUCTURE_INFERRED,
    YELLOW_ROLE,
)


ROLE_UNKNOWN = 0
ROLE_LEFT_OUTER_BOUNDARY = 1
ROLE_ROAD_CENTRE_BOUNDARY = 2
ROLE_RIGHT_OUTER_BOUNDARY = 3

COLOR_UNKNOWN = 0
COLOR_WHITE = 1
COLOR_YELLOW = 2

SUPPORT_INVALID = SAMPLE_INVALID
SUPPORT_OBSERVED = SAMPLE_OBSERVED
SUPPORT_STRUCTURE_INFERRED = SAMPLE_STRUCTURE_INFERRED

_ROLE_IDS = {
    LEFT_ROLE: ROLE_LEFT_OUTER_BOUNDARY,
    YELLOW_ROLE: ROLE_ROAD_CENTRE_BOUNDARY,
    RIGHT_ROLE: ROLE_RIGHT_OUTER_BOUNDARY,
}
_OVERALL_STATE_IDS = {
    OBSERVED: SUPPORT_OBSERVED,
    STRUCTURE_INFERRED: SUPPORT_STRUCTURE_INFERRED,
}
_ALLOWED_SAMPLE_STATES = {
    SUPPORT_INVALID,
    SUPPORT_OBSERVED,
    SUPPORT_STRUCTURE_INFERRED,
}


@dataclass(frozen=True)
class BoundaryFamilyMemberOutput:
    role: int
    color: int
    source_mask: int
    overall_state: int
    confidence: float
    residual_valid: bool
    residual_p50_m: float
    residual_p95_m: float
    observed_length_m: float
    inferred_length_m: float
    invalid_length_m: float
    max_inferred_gap_m: float
    bridged_length_ratio: float
    polyline_xy: np.ndarray
    support_state: np.ndarray


@dataclass(frozen=True)
class BoundaryFamilyOutputFrame:
    valid: bool
    status_reason: str
    expected_boundary_count: int
    valid_x_min_m: float
    valid_x_max_m: float
    left_width_m: float
    right_width_m: float
    data_residual_p50_m: float
    data_residual_p95_m: float
    inlier_residual_p50_m: float
    inlier_residual_p95_m: float
    outlier_rate: float
    weighted_rms_m: float
    boundaries: Tuple[BoundaryFamilyMemberOutput, ...]


def _finite_float(value, name: str) -> float:
    result = float(value)
    if not np.isfinite(result):
        raise ValueError("{} must be finite".format(name))
    return result


def _member(boundary) -> BoundaryFamilyMemberOutput:
    if boundary.role not in _ROLE_IDS:
        raise ValueError("unknown shared boundary role: {!r}".format(boundary.role))
    if int(boundary.class_id) not in (1, 2):
        raise ValueError("shared boundary class must be white or yellow")
    if boundary.state not in _OVERALL_STATE_IDS:
        raise ValueError("unknown shared boundary state: {!r}".format(boundary.state))

    points = np.asarray(boundary.sample_points, dtype=np.float64).reshape(-1, 2)
    states = np.asarray(boundary.sample_state, dtype=np.uint8).reshape(-1)
    if len(points) < 2 or len(points) != len(states):
        raise ValueError("shared boundary points and support states must align")
    if not np.isfinite(points).all():
        raise ValueError("shared boundary contains non-finite points")
    unknown_states = sorted(set(int(value) for value in states) - _ALLOWED_SAMPLE_STATES)
    if unknown_states:
        raise ValueError(
            "unknown shared boundary support states: {}".format(unknown_states)
        )
    if not np.any(states != SUPPORT_INVALID):
        raise ValueError("accepted shared boundary must contain usable support")

    residual_valid = (
        boundary.residual_p50_m is not None
        and boundary.residual_p95_m is not None
    )
    residual_p50 = (
        _finite_float(boundary.residual_p50_m, "residual_p50_m")
        if residual_valid
        else 0.0
    )
    residual_p95 = (
        _finite_float(boundary.residual_p95_m, "residual_p95_m")
        if residual_valid
        else 0.0
    )
    return BoundaryFamilyMemberOutput(
        role=_ROLE_IDS[boundary.role],
        color=COLOR_YELLOW if int(boundary.class_id) == 2 else COLOR_WHITE,
        source_mask=int(boundary.source_mask),
        overall_state=_OVERALL_STATE_IDS[boundary.state],
        confidence=float(np.clip(_finite_float(boundary.confidence, "confidence"), 0.0, 1.0)),
        residual_valid=bool(residual_valid),
        residual_p50_m=residual_p50,
        residual_p95_m=residual_p95,
        observed_length_m=_finite_float(
            boundary.observed_length_m, "observed_length_m"
        ),
        inferred_length_m=_finite_float(
            boundary.inferred_length_m, "inferred_length_m"
        ),
        invalid_length_m=_finite_float(
            boundary.invalid_length_m, "invalid_length_m"
        ),
        max_inferred_gap_m=_finite_float(
            boundary.max_inferred_gap_m, "max_inferred_gap_m"
        ),
        bridged_length_ratio=float(
            np.clip(
                _finite_float(boundary.bridged_length_ratio, "bridged_length_ratio"),
                0.0,
                1.0,
            )
        ),
        polyline_xy=np.ascontiguousarray(points, dtype=np.float32),
        support_state=np.ascontiguousarray(states, dtype=np.uint8),
    )


def build_boundary_family_output(
    family,
    status_reason: str,
    expected_boundary_count: int = 3,
) -> BoundaryFamilyOutputFrame:
    """Validate and freeze a shared fit for ROS serialization.

    ``structure_inferred`` samples remain in the output as explicitly degraded
    usable geometry. ``invalid`` samples retain the fitted coordinates for
    audit and an explicitly opted-in low-authority preview/fallback experiment,
    but they are not claimed perception evidence. Default consumers must split
    at them; they are never promoted or silently connected here.
    """
    expected = int(expected_boundary_count)
    if not 0 < expected <= 255:
        raise ValueError("expected boundary count must lie in [1,255]")
    reason = str(status_reason or "unknown")
    if family is None:
        return BoundaryFamilyOutputFrame(
            valid=False,
            status_reason=reason,
            expected_boundary_count=expected,
            valid_x_min_m=0.0,
            valid_x_max_m=0.0,
            left_width_m=0.0,
            right_width_m=0.0,
            data_residual_p50_m=0.0,
            data_residual_p95_m=0.0,
            inlier_residual_p50_m=0.0,
            inlier_residual_p95_m=0.0,
            outlier_rate=0.0,
            weighted_rms_m=0.0,
            boundaries=(),
        )

    by_role = {boundary.role: boundary for boundary in family.boundaries}
    if len(by_role) != len(family.boundaries) or set(by_role) != set(ROLE_ORDER):
        raise ValueError("accepted shared family must contain each role exactly once")
    members = tuple(_member(by_role[role]) for role in ROLE_ORDER)
    if len(members) != expected:
        raise ValueError("shared family boundary count differs from contract")

    lower = _finite_float(family.valid_x_min_m, "valid_x_min_m")
    upper = _finite_float(family.valid_x_max_m, "valid_x_max_m")
    if upper < lower:
        raise ValueError("shared family valid x range is inverted")
    outlier_rate = _finite_float(family.outlier_rate, "outlier_rate")
    if not 0.0 <= outlier_rate <= 1.0:
        raise ValueError("shared family outlier rate must lie in [0,1]")
    return BoundaryFamilyOutputFrame(
        valid=True,
        status_reason=reason,
        expected_boundary_count=expected,
        valid_x_min_m=lower,
        valid_x_max_m=upper,
        left_width_m=_finite_float(family.left_width_m, "left_width_m"),
        right_width_m=_finite_float(family.right_width_m, "right_width_m"),
        data_residual_p50_m=_finite_float(
            family.data_residual_p50_m, "data_residual_p50_m"
        ),
        data_residual_p95_m=_finite_float(
            family.data_residual_p95_m, "data_residual_p95_m"
        ),
        inlier_residual_p50_m=_finite_float(
            family.inlier_residual_p50_m, "inlier_residual_p50_m"
        ),
        inlier_residual_p95_m=_finite_float(
            family.inlier_residual_p95_m, "inlier_residual_p95_m"
        ),
        outlier_rate=outlier_rate,
        weighted_rms_m=_finite_float(family.weighted_rms_m, "weighted_rms_m"),
        boundaries=members,
    )
