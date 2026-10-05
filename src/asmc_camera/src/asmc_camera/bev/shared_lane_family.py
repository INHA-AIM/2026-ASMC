"""Shared lane-family fitting for the K-City GPS-shadow ODD.

This module leaves the established independent boundary-observation contract
unchanged. It provides the evaluated shared representation to offline analysis,
debug visualization, and an explicitly enabled typed candidate interface. A
single parametric centre curve explains the yellow road-centre marking and the
two white outer boundaries through normal-direction offsets.

The implementation is original, dependency-light NumPy code.  It borrows the
general ideas of confidence-weighted least squares and continuous physical
priors from the papers documented in
``src/asmc_camera/docs/bev-development-history.md``; no source is copied
from the research repositories under ``external``.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Mapping, Optional, Sequence, Tuple

import numpy as np

from .spline_fitting import bspline_basis


LEFT_ROLE = "left_outer_boundary"
YELLOW_ROLE = "yellow_centre_boundary"
RIGHT_ROLE = "right_outer_boundary"
ROLE_ORDER = (LEFT_ROLE, YELLOW_ROLE, RIGHT_ROLE)

OBSERVED = "observed"
STRUCTURE_INFERRED = "structure_inferred"

SAMPLE_INVALID = 0
SAMPLE_OBSERVED = 1
SAMPLE_STRUCTURE_INFERRED = 2

SAMPLE_STATE_NAMES = {
    SAMPLE_INVALID: "invalid",
    SAMPLE_OBSERVED: OBSERVED,
    SAMPLE_STRUCTURE_INFERRED: STRUCTURE_INFERRED,
}


@dataclass(frozen=True)
class SharedFamilyBoundary:
    role: str
    class_id: int
    state: str
    sample_points: np.ndarray
    sample_observed: np.ndarray
    sample_state: np.ndarray
    confidence: float
    source_mask: int
    observed_length_m: float
    inferred_length_m: float
    invalid_length_m: float
    max_inferred_gap_m: float
    bridged_length_ratio: float
    residual_p50_m: Optional[float]
    residual_p95_m: Optional[float]


@dataclass(frozen=True)
class SharedLaneFamily:
    control_points: np.ndarray
    knots: np.ndarray
    degree: int
    centre_points: np.ndarray
    sample_valid: np.ndarray
    local_support_count: np.ndarray
    left_width_m: float
    right_width_m: float
    boundaries: Tuple[SharedFamilyBoundary, ...]
    valid_x_min_m: float
    valid_x_max_m: float
    data_residual_p50_m: float
    data_residual_p95_m: float
    inlier_residual_p50_m: float
    inlier_residual_p95_m: float
    outlier_rate: float
    weighted_rms_m: float
    iterations: int

    def boundary(self, role: str) -> SharedFamilyBoundary:
        matches = [value for value in self.boundaries if value.role == role]
        if len(matches) != 1:
            raise KeyError("shared family has no unique role {!r}".format(role))
        return matches[0]


def contiguous_sample_state_segments(points, sample_state, min_points=2):
    """Split one sampled boundary without drawing across state transitions.

    A state belongs to a sampled point, not to the interval between two points.
    Keeping only runs whose endpoints share the same state avoids visually
    promoting an invalid or inferred transition into an observed line.
    """
    values = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    states = np.asarray(sample_state, dtype=np.uint8).reshape(-1)
    if len(values) != len(states):
        raise ValueError("shared boundary point and state counts differ")
    if int(min_points) < 2:
        raise ValueError("state segments require at least two points")
    unknown = sorted(set(int(value) for value in states) - set(SAMPLE_STATE_NAMES))
    if unknown:
        raise ValueError("unknown shared boundary sample states: {}".format(unknown))

    segments = []
    start = 0
    while start < len(states):
        end = start + 1
        while end < len(states) and states[end] == states[start]:
            end += 1
        if end - start >= int(min_points):
            segments.append(
                (
                    SAMPLE_STATE_NAMES[int(states[start])],
                    np.ascontiguousarray(values[start:end]),
                )
            )
        start = end
    return tuple(segments)


DEFAULT_SHARED_FAMILY_CONFIG = {
    "reference_x_m": 5.0,
    "nominal_left_width_m": 3.48,
    "nominal_right_width_m": 3.52,
    "lane_width_min_m": 2.40,
    "lane_width_max_m": 4.60,
    "width_prior_strength": 0.35,
    "spline_smoothing": 0.08,
    "control_spacing_m": 2.5,
    "sample_spacing_m": 0.25,
    "robust_iterations": 5,
    "robust_scale_m": 0.15,
    "outlier_scale": 2.5,
    "min_observed_roles": 2,
    "min_family_span_m": 3.0,
    "observed_support_radius_m": 0.45,
    "support_residual_max_m": 0.35,
    "min_local_support_roles": 2,
    "max_structure_bridge_m": 0.75,
    "min_observed_segment_m": 0.50,
    "inferred_confidence_scale": 0.60,
    # Explicit, bounded fail-soft policy for the shadow-road candidate.  This
    # never creates a family from one observed role: it is applied only after
    # the normal multi-role acceptance gate has passed.  Samples promoted by
    # this policy remain STRUCTURE_INFERRED, never OBSERVED.
    "same_frame_completion_enabled": False,
    "same_frame_completion_x_min_m": 0.0,
    "same_frame_completion_x_max_m": 15.0,
    # Conservative fallback for two or more globally assigned roles whose
    # supports are staggered along the road and therefore fail the strict
    # local-overlap gate.  It is disabled in the reusable library default and
    # explicitly enabled by the shadow-road runtime configuration.
    "distributed_support_enabled": False,
    "distributed_support_x_min_m": 0.0,
    "distributed_support_x_max_m": 15.0,
    "distributed_min_role_support_m": 2.0,
    "distributed_min_union_support_m": 6.0,
    "distributed_max_internal_gap_m": 3.0,
    "distributed_max_heading_delta_deg": 15.0,
    "distributed_max_inlier_residual_p95_m": 0.30,
    "distributed_max_outlier_rate": 0.35,
    "distributed_max_curvature_inv_m": 0.12,
}


def _config(values: Optional[Mapping]) -> dict:
    result = dict(DEFAULT_SHARED_FAMILY_CONFIG)
    result.update(dict(values or {}))
    positive = (
        "nominal_left_width_m",
        "nominal_right_width_m",
        "lane_width_min_m",
        "lane_width_max_m",
        "control_spacing_m",
        "sample_spacing_m",
        "robust_scale_m",
        "outlier_scale",
        "min_family_span_m",
        "observed_support_radius_m",
        "support_residual_max_m",
        "max_structure_bridge_m",
        "min_observed_segment_m",
        "distributed_min_role_support_m",
        "distributed_min_union_support_m",
        "distributed_max_internal_gap_m",
        "distributed_max_heading_delta_deg",
        "distributed_max_inlier_residual_p95_m",
        "distributed_max_curvature_inv_m",
    )
    if any(float(result[key]) <= 0.0 for key in positive):
        raise ValueError("shared lane-family distances must be positive")
    if float(result["lane_width_max_m"]) <= float(result["lane_width_min_m"]):
        raise ValueError("shared lane-family width bounds are invalid")
    if float(result["spline_smoothing"]) < 0.0:
        raise ValueError("shared lane-family smoothing must be non-negative")
    if float(result["width_prior_strength"]) < 0.0:
        raise ValueError("width prior strength must be non-negative")
    if int(result["robust_iterations"]) < 1:
        raise ValueError("robust iterations must be positive")
    if int(result["min_observed_roles"]) not in (2, 3):
        raise ValueError("min_observed_roles must be two or three")
    if int(result["min_local_support_roles"]) not in (2, 3):
        raise ValueError("min_local_support_roles must be two or three")
    if not 0.0 < float(result["inferred_confidence_scale"]) <= 1.0:
        raise ValueError("inferred confidence scale must lie in (0,1]")
    completion_bounds = (
        float(result["same_frame_completion_x_min_m"]),
        float(result["same_frame_completion_x_max_m"]),
    )
    if not np.isfinite(completion_bounds).all():
        raise ValueError("same-frame completion bounds must be finite")
    if completion_bounds[1] <= completion_bounds[0]:
        raise ValueError("same-frame completion range must have positive length")
    distributed_bounds = (
        float(result["distributed_support_x_min_m"]),
        float(result["distributed_support_x_max_m"]),
    )
    if not np.isfinite(distributed_bounds).all():
        raise ValueError("distributed-support bounds must be finite")
    if distributed_bounds[1] <= distributed_bounds[0]:
        raise ValueError("distributed-support range must have positive length")
    if not 0.0 <= float(result["distributed_max_outlier_rate"]) <= 1.0:
        raise ValueError("distributed-support outlier rate must lie in [0,1]")
    if float(result["distributed_max_heading_delta_deg"]) > 90.0:
        raise ValueError("distributed-support heading delta must not exceed 90deg")
    return result


def _open_uniform_knots(control_count: int, degree: int) -> np.ndarray:
    interior_count = control_count - degree - 1
    interior = (
        np.linspace(0.0, 1.0, interior_count + 2, dtype=np.float64)[1:-1]
        if interior_count > 0
        else np.empty(0, dtype=np.float64)
    )
    return np.concatenate(
        [np.zeros(degree + 1), interior, np.ones(degree + 1)]
    )


def _track_weights(track) -> np.ndarray:
    result = np.asarray(
        [
            max(
                1e-6,
                float(node.confidence)
                * float(node.geometric_quality)
                * math.sqrt(max(1, int(node.support))),
            )
            for node in track.nodes
        ],
        dtype=np.float64,
    )
    # Give every observed role comparable leverage.  Otherwise a thick line
    # seen by two cameras can dominate the family purely through point count.
    return result / max(float(np.mean(result)), 1e-9)


def _track_score(track) -> float:
    points = np.asarray(track.xy, dtype=np.float64).reshape(-1, 2)
    if len(points) < 2:
        return 0.0
    span = max(0.0, float(np.ptp(points[:, 0])))
    weights = _track_weights(track)
    confidence = float(
        np.average([node.confidence for node in track.nodes], weights=weights)
    )
    return confidence * min(1.0, span / 12.0)


def _track_y_at_x(track, x_m: float) -> float:
    points = np.asarray(track.xy, dtype=np.float64).reshape(-1, 2)
    order = np.argsort(points[:, 0], kind="mergesort")
    points = points[order]
    unique_x, indices = np.unique(points[:, 0], return_index=True)
    target = float(np.clip(float(x_m), unique_x[0], unique_x[-1]))
    return float(np.interp(target, unique_x, points[indices, 1]))


def _track_separation(first, second, reference_x_m: float) -> float:
    first_xy = np.asarray(first.xy, dtype=np.float64).reshape(-1, 2)
    second_xy = np.asarray(second.xy, dtype=np.float64).reshape(-1, 2)
    lower = max(float(first_xy[:, 0].min()), float(second_xy[:, 0].min()))
    upper = min(float(first_xy[:, 0].max()), float(second_xy[:, 0].max()))
    if upper - lower < 1.0:
        return _track_y_at_x(second, reference_x_m) - _track_y_at_x(
            first, reference_x_m
        )
    samples = np.linspace(lower, upper, max(3, int(math.ceil(upper - lower)) + 1))
    return float(
        np.median(
            [
                _track_y_at_x(second, value) - _track_y_at_x(first, value)
                for value in samples
            ]
        )
    )


def _assign_observed_roles(tracks: Sequence, values: Mapping) -> tuple[dict, dict]:
    """Assign only physically supported roles; never invent a one-line family."""
    valid = [
        track
        for track in tracks
        if int(track.class_id) in (1, 2)
        and len(track.nodes) >= 2
        and float(np.ptp(np.asarray(track.xy)[:, 0]))
        >= float(values["min_family_span_m"])
    ]
    yellow = sorted(
        (track for track in valid if int(track.class_id) == 2),
        key=lambda track: (-_track_score(track), abs(_track_y_at_x(track, values["reference_x_m"]))),
    )
    white = [track for track in valid if int(track.class_id) == 1]
    roles = {}
    audit = {
        "input_tracks": int(len(tracks)),
        "eligible_tracks": int(len(valid)),
        "discarded_tracks": int(len(tracks) - len(valid)),
        "yellow_candidates": int(len(yellow)),
        "white_candidates": int(len(white)),
        "assignment": {},
    }
    width_min = float(values["lane_width_min_m"])
    width_max = float(values["lane_width_max_m"])
    reference_x = float(values["reference_x_m"])
    if yellow:
        centre = yellow[0]
        roles[YELLOW_ROLE] = centre
        for role, sign, nominal in (
            (LEFT_ROLE, 1.0, float(values["nominal_left_width_m"])),
            (RIGHT_ROLE, -1.0, float(values["nominal_right_width_m"])),
        ):
            candidates = []
            for track in white:
                separation = _track_separation(centre, track, reference_x)
                if separation * sign <= 0.0:
                    continue
                width = abs(separation)
                if not width_min <= width <= width_max:
                    continue
                score = _track_score(track) * math.exp(
                    -abs(width - nominal) / max(nominal, 1e-6)
                )
                candidates.append((score, track, width))
            if candidates:
                _, chosen, measured = max(candidates, key=lambda item: item[0])
                roles[role] = chosen
                audit["assignment"][role] = {
                    "measured_separation_m": float(measured),
                    "candidate_count": int(len(candidates)),
                }
    else:
        # With no yellow, two outer whites can still determine the road-centre
        # family.  A single white is intentionally insufficient until temporal
        # propagation is evaluated in a later stage.
        pairs = []
        expected_total = float(values["nominal_left_width_m"]) + float(
            values["nominal_right_width_m"]
        )
        for first_index, first in enumerate(white):
            for second in white[first_index + 1 :]:
                separation = abs(_track_separation(first, second, reference_x))
                if not 2.0 * width_min <= separation <= 2.0 * width_max:
                    continue
                score = (_track_score(first) + _track_score(second)) * math.exp(
                    -abs(separation - expected_total) / max(expected_total, 1e-6)
                )
                pairs.append((score, first, second, separation))
        if pairs:
            _, first, second, measured = max(pairs, key=lambda item: item[0])
            if _track_y_at_x(first, reference_x) >= _track_y_at_x(second, reference_x):
                roles[LEFT_ROLE], roles[RIGHT_ROLE] = first, second
            else:
                roles[LEFT_ROLE], roles[RIGHT_ROLE] = second, first
            audit["assignment"]["white_pair"] = {
                "measured_separation_m": float(measured),
                "candidate_count": int(len(pairs)),
            }
    audit["observed_roles"] = [role for role in ROLE_ORDER if role in roles]
    audit["observed_role_count"] = int(len(roles))
    return roles, audit


def _basis_derivative(parameters, knots, degree, control_count) -> np.ndarray:
    values = np.asarray(parameters, dtype=np.float64).reshape(-1)
    epsilon = 1e-4
    lower = np.clip(values - epsilon, 0.0, 1.0)
    upper = np.clip(values + epsilon, 0.0, 1.0)
    denominator = np.maximum(upper - lower, 1e-9)[:, None]
    return (
        bspline_basis(upper, knots, degree, control_count)
        - bspline_basis(lower, knots, degree, control_count)
    ) / denominator


def _fit_controls(
    targets: np.ndarray,
    parameters: np.ndarray,
    weights: np.ndarray,
    control_count: int,
    degree: int,
    smoothing: float,
) -> tuple[np.ndarray, np.ndarray]:
    knots = _open_uniform_knots(control_count, degree)
    basis = bspline_basis(parameters, knots, degree, control_count)
    root = np.sqrt(np.maximum(weights, 1e-9))[:, None]
    design = basis * root
    target = targets * root
    if control_count >= 3 and smoothing > 0.0:
        regularizer = np.diff(np.eye(control_count), n=2, axis=0)
        design = np.vstack((design, math.sqrt(smoothing) * regularizer))
        target = np.vstack((target, np.zeros((len(regularizer), 2))))
    controls = np.linalg.lstsq(design, target, rcond=None)[0]
    return knots, controls


def _curve_geometry(parameters, knots, degree, controls):
    basis = bspline_basis(parameters, knots, degree, len(controls))
    points = basis @ controls
    tangent = _basis_derivative(parameters, knots, degree, len(controls)) @ controls
    norm = np.linalg.norm(tangent, axis=1)
    if np.any(norm <= 1e-9):
        raise ValueError("shared family centre spline has a degenerate tangent")
    tangent /= norm[:, None]
    # Select the continuous normal pointing toward base_link positive y.
    normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))
    flip = normal[:, 1] < 0.0
    normal[flip] *= -1.0
    return points, tangent, normal


def _nearest_curve_indices(points, curve):
    values = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    reference = np.asarray(curve, dtype=np.float64).reshape(-1, 2)
    result = np.empty(len(values), dtype=np.int64)
    for start in range(0, len(values), 512):
        stop = min(len(values), start + 512)
        squared = np.sum(
            (values[start:stop, None, :] - reference[None, :, :]) ** 2,
            axis=2,
        )
        result[start:stop] = np.argmin(squared, axis=1)
    return result


def _nearest_point_distances(points, reference):
    values = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    target = np.asarray(reference, dtype=np.float64).reshape(-1, 2)
    if not len(target):
        return np.full(len(values), np.inf, dtype=np.float64)
    result = np.empty(len(values), dtype=np.float64)
    for start in range(0, len(values), 512):
        stop = min(len(values), start + 512)
        squared = np.sum(
            (values[start:stop, None, :] - target[None, :, :]) ** 2,
            axis=2,
        )
        result[start:stop] = np.sqrt(np.min(squared, axis=1))
    return result


def _run_lengths(mask, sample_station):
    values = np.asarray(mask, dtype=bool).reshape(-1)
    station = np.asarray(sample_station, dtype=np.float64).reshape(-1)
    if len(values) != len(station):
        raise ValueError("sample mask and station must have equal length")
    result = []
    start = 0
    while start < len(values):
        stop = start + 1
        while stop < len(values) and values[stop] == values[start]:
            stop += 1
        lower = station[start]
        upper = station[stop - 1]
        result.append((start, stop, bool(values[start]), max(0.0, float(upper - lower))))
        start = stop
    return result


def _fill_short_internal_gaps(mask, sample_station, maximum_gap_m):
    result = np.asarray(mask, dtype=bool).copy()
    for start, stop, value, length in _run_lengths(result, sample_station):
        if value or start == 0 or stop == len(result):
            continue
        if result[start - 1] and result[stop] and length <= float(maximum_gap_m):
            result[start:stop] = True
    return result


def _remove_short_support_islands(mask, sample_station, minimum_length_m):
    result = np.asarray(mask, dtype=bool).copy()
    for start, stop, value, length in _run_lengths(result, sample_station):
        if value and length < float(minimum_length_m):
            result[start:stop] = False
    return result


def _longest_state_length(state, expected, sample_station):
    mask = np.asarray(state) == int(expected)
    lengths = [length for _, _, value, length in _run_lengths(mask, sample_station) if value]
    return max(lengths, default=0.0)


def _mask_segment_length(mask, segment_length) -> float:
    values = np.asarray(mask, dtype=bool).reshape(-1)
    segments = np.asarray(segment_length, dtype=np.float64).reshape(-1)
    if len(values) != len(segments) + 1:
        raise ValueError("sample mask and segment lengths do not align")
    return float(np.sum(segments[values[:-1] & values[1:]]))


def _longest_internal_false_run(mask, sample_station) -> float:
    runs = _run_lengths(mask, sample_station)
    return max(
        (
            length
            for start, stop, value, length in runs
            if not value and start > 0 and stop < len(mask)
        ),
        default=0.0,
    )


def _track_axis(track) -> np.ndarray:
    points = np.asarray(track.xy, dtype=np.float64).reshape(-1, 2)
    weights = _track_weights(track)
    centre = np.average(points, axis=0, weights=weights)
    centered = points - centre
    covariance = (centered * weights[:, None]).T @ centered
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    axis = eigenvectors[:, int(np.argmax(eigenvalues))]
    norm = float(np.linalg.norm(axis))
    if norm <= 1e-9:
        raise ValueError("lane role has a degenerate principal axis")
    return axis / norm


def _maximum_role_heading_delta_deg(roles: Mapping) -> float:
    axes = [_track_axis(track) for track in roles.values()]
    maximum = 0.0
    for first_index, first in enumerate(axes):
        for second in axes[first_index + 1 :]:
            # Lane direction is unoriented; antiparallel axes describe the
            # same physical heading.
            cosine = float(np.clip(abs(np.dot(first, second)), 0.0, 1.0))
            maximum = max(maximum, math.degrees(math.acos(cosine)))
    return float(maximum)


def _curve_curvature_p95(points) -> float:
    values = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    segment = np.diff(values, axis=0)
    length = np.linalg.norm(segment, axis=1)
    valid = length > 1e-6
    if int(np.sum(valid)) < 2:
        return float("inf")
    unit = segment[valid] / length[valid, None]
    cosine = np.sum(unit[:-1] * unit[1:], axis=1)
    sine = unit[:-1, 0] * unit[1:, 1] - unit[:-1, 1] * unit[1:, 0]
    turn = np.abs(np.arctan2(sine, np.clip(cosine, -1.0, 1.0)))
    scale = 0.5 * (length[valid][:-1] + length[valid][1:])
    curvature = turn / np.maximum(scale, 1e-6)
    return float(np.percentile(curvature, 95)) if len(curvature) else 0.0


def _distributed_support_gate(
    roles,
    role_support,
    centre_samples,
    sample_station,
    centre_segment,
    residual,
    values,
):
    """Evaluate a same-frame, multi-role but non-overlapping fallback."""
    lower = float(values["distributed_support_x_min_m"])
    upper = float(values["distributed_support_x_max_m"])
    window = (centre_samples[:, 0] >= lower) & (centre_samples[:, 0] <= upper)
    support_in_window = {
        role: np.asarray(role_support[role], dtype=bool) & window
        for role in ROLE_ORDER
    }
    role_lengths = {
        role: _mask_segment_length(support_in_window[role], centre_segment)
        for role in ROLE_ORDER
    }
    qualified_roles = [
        role
        for role in roles
        if role_lengths[role] >= float(values["distributed_min_role_support_m"])
    ]
    union_support = np.any(
        np.vstack([support_in_window[role] for role in ROLE_ORDER]), axis=0
    )
    union_length = _mask_segment_length(union_support, centre_segment)
    internal_gap = _longest_internal_false_run(union_support, sample_station)
    inlier = residual <= float(values["support_residual_max_m"])
    inlier_p95 = (
        float(np.percentile(residual[inlier], 95)) if np.any(inlier) else float("inf")
    )
    outlier_rate = float(np.mean(~inlier))
    heading_delta = _maximum_role_heading_delta_deg(
        {role: roles[role] for role in qualified_roles}
    ) if len(qualified_roles) >= 2 else float("inf")
    # Judge shape only inside the fallback's bounded consumer window. A noisy
    # mathematical tail outside this window must not veto otherwise coherent
    # same-frame evidence, and that tail is never promoted by this policy.
    curvature_p95 = _curve_curvature_p95(centre_samples[window])
    checks = {
        "qualified_role_count": len(qualified_roles)
        >= int(values["min_observed_roles"]),
        "union_support_length": union_length
        >= float(values["distributed_min_union_support_m"]),
        "internal_gap": internal_gap
        <= float(values["distributed_max_internal_gap_m"]),
        "heading_consistency": heading_delta
        <= float(values["distributed_max_heading_delta_deg"]),
        "inlier_residual": inlier_p95
        <= float(values["distributed_max_inlier_residual_p95_m"]),
        "outlier_rate": outlier_rate
        <= float(values["distributed_max_outlier_rate"]),
        "curvature": curvature_p95
        <= float(values["distributed_max_curvature_inv_m"]),
    }
    enabled = bool(values["distributed_support_enabled"])
    accepted = bool(enabled and all(checks.values()))
    audit = {
        "enabled": enabled,
        "attempted": True,
        "accepted": accepted,
        "x_min_m": lower,
        "x_max_m": upper,
        "qualified_roles": qualified_roles,
        "role_support_length_m": role_lengths,
        "union_support_length_m": union_length,
        "max_internal_unsupported_gap_m": internal_gap,
        "max_role_heading_delta_deg": heading_delta,
        "inlier_residual_p95_m": inlier_p95,
        "outlier_rate": outlier_rate,
        "curve_curvature_p95_inv_m": curvature_p95,
        "checks": checks,
        "rejection_reasons": [name for name, passed in checks.items() if not passed],
        "temporal_prediction_used": False,
        "map_geometry_used": False,
    }
    return accepted, union_support, audit


def _weighted_average(values, weights) -> float:
    value = np.asarray(values, dtype=np.float64)
    weight = np.asarray(weights, dtype=np.float64)
    return float(np.sum(value * weight) / max(float(np.sum(weight)), 1e-9))


def _resample_curve(points, parameters, spacing_m):
    values = np.asarray(points, dtype=np.float64)
    distance = np.linalg.norm(np.diff(values, axis=0), axis=1)
    cumulative = np.concatenate(([0.0], np.cumsum(distance)))
    if cumulative[-1] <= 1e-9:
        raise ValueError("shared family centre has no measurable length")
    target = np.arange(0.0, cumulative[-1], float(spacing_m))
    if not len(target) or target[-1] < cumulative[-1]:
        target = np.append(target, cumulative[-1])
    sample = np.column_stack(
        [np.interp(target, cumulative, values[:, axis]) for axis in range(2)]
    )
    sample_parameter = np.interp(target, cumulative, parameters)
    return sample, sample_parameter, target


def fit_shared_lane_family_with_audit(
    tracks: Sequence,
    config: Optional[Mapping] = None,
) -> tuple[Optional[SharedLaneFamily], dict]:
    """Fit one centre spline and normal-offset boundaries from lane tracks.

    The first offline gate requires two independently observed roles.  Missing
    members of a geometrically valid family are marked ``structure_inferred``;
    no temporal prediction is performed here.
    """
    values = _config(config)
    roles, audit = _assign_observed_roles(tracks, values)
    if len(roles) < int(values["min_observed_roles"]):
        audit.update({"accepted": False, "reason": "too_few_observed_roles"})
        return None, audit

    nominal = {
        LEFT_ROLE: float(values["nominal_left_width_m"]),
        YELLOW_ROLE: 0.0,
        RIGHT_ROLE: -float(values["nominal_right_width_m"]),
    }
    role_points = {
        role: np.asarray(track.xy, dtype=np.float64).reshape(-1, 2)
        for role, track in roles.items()
    }
    role_weights = {role: _track_weights(track) for role, track in roles.items()}
    points = np.concatenate([role_points[role] for role in roles])
    base_weights = np.concatenate([role_weights[role] for role in roles])
    point_roles = np.concatenate(
        [np.full(len(role_points[role]), role, dtype=object) for role in roles]
    )

    # Build a neutral first guess.  The shadow road is longitudinal in
    # base_link, so positive-y is a safe initial normal; subsequent iterations
    # derive the actual normal from the common parametric spline.
    neutral_normal = np.asarray([0.0, 1.0], dtype=np.float64)
    targets = points - np.asarray(
        [nominal[str(role)] * neutral_normal for role in point_roles]
    )
    centered = targets - np.average(targets, axis=0, weights=base_weights)
    covariance = (centered * base_weights[:, None]).T @ centered
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    axis = eigenvectors[:, int(np.argmax(eigenvalues))]
    if axis[0] < 0.0:
        axis = -axis
    origin = np.average(targets, axis=0, weights=base_weights)
    station = (targets - origin) @ axis
    station_min = float(np.min(station))
    station_span = float(np.ptp(station))
    if station_span < float(values["min_family_span_m"]):
        audit.update({"accepted": False, "reason": "short_family_span"})
        return None, audit
    parameters = np.clip((station - station_min) / station_span, 0.0, 1.0)
    degree = min(3, len(points) - 1)
    control_count = int(
        np.clip(
            math.ceil(station_span / float(values["control_spacing_m"])) + degree,
            4,
            min(len(points), 16),
        )
    )
    robust = np.ones(len(points), dtype=np.float64)
    knots, controls = _fit_controls(
        targets,
        parameters,
        base_weights,
        control_count,
        degree,
        float(values["spline_smoothing"]),
    )
    left_width = float(values["nominal_left_width_m"])
    right_width = float(values["nominal_right_width_m"])
    dense_parameters = np.linspace(0.0, 1.0, max(200, control_count * 32))
    residual = np.empty(len(points), dtype=np.float64)
    assigned_parameters = parameters.copy()
    assigned_normals = np.tile(neutral_normal, (len(points), 1))

    for _ in range(int(values["robust_iterations"])):
        dense_curve, _, dense_normals = _curve_geometry(
            dense_parameters, knots, degree, controls
        )
        nearest = _nearest_curve_indices(points, dense_curve)
        assigned_parameters = dense_parameters[nearest]
        assigned_centres = dense_curve[nearest]
        assigned_normals = dense_normals[nearest]
        signed = np.sum((points - assigned_centres) * assigned_normals, axis=1)

        prior_strength = float(values["width_prior_strength"])
        width_min = float(values["lane_width_min_m"])
        width_max = float(values["lane_width_max_m"])
        for role, prior, sign in (
            (LEFT_ROLE, float(values["nominal_left_width_m"]), 1.0),
            (RIGHT_ROLE, float(values["nominal_right_width_m"]), -1.0),
        ):
            mask = point_roles == role
            if not np.any(mask):
                continue
            measured = sign * signed[mask]
            weight = base_weights[mask] * robust[mask]
            estimate = (
                _weighted_average(measured, weight) + prior_strength * prior
            ) / (1.0 + prior_strength)
            if role == LEFT_ROLE:
                left_width = float(np.clip(estimate, width_min, width_max))
            else:
                right_width = float(np.clip(estimate, width_min, width_max))

        offsets = np.asarray(
            [
                left_width
                if role == LEFT_ROLE
                else -right_width
                if role == RIGHT_ROLE
                else 0.0
                for role in point_roles
            ],
            dtype=np.float64,
        )
        targets = points - offsets[:, None] * assigned_normals
        knots, controls = _fit_controls(
            targets,
            assigned_parameters,
            base_weights * robust,
            control_count,
            degree,
            float(values["spline_smoothing"]),
        )
        centres, _, normals = _curve_geometry(
            assigned_parameters, knots, degree, controls
        )
        signed = np.sum((points - centres) * normals, axis=1)
        residual = np.abs(signed - offsets)
        median = float(np.median(residual))
        mad = float(np.median(np.abs(residual - median)))
        scale = max(
            float(values["robust_scale_m"]),
            float(values["outlier_scale"]) * 1.4826 * mad,
        )
        robust = 1.0 / (1.0 + (residual / max(scale, 1e-9)) ** 2)

    dense_curve, _, _ = _curve_geometry(dense_parameters, knots, degree, controls)
    centre_samples, sample_parameters, sample_station = _resample_curve(
        dense_curve, dense_parameters, float(values["sample_spacing_m"])
    )
    centre_samples, _, sample_normals = _curve_geometry(
        sample_parameters, knots, degree, controls
    )
    if np.any(np.diff(centre_samples[:, 0]) < -0.10):
        audit.update({"accepted": False, "reason": "non_monotonic_forward_curve"})
        return None, audit

    offsets = {
        LEFT_ROLE: left_width,
        YELLOW_ROLE: 0.0,
        RIGHT_ROLE: -right_width,
    }
    observed_confidence = {
        role: float(
            np.average(
                [node.confidence for node in roles[role].nodes],
                weights=role_weights[role],
            )
        )
        for role in roles
    }
    fallback_confidence = min(observed_confidence.values())

    # A role is observed only where its actual metric nodes lie close to that
    # role's fitted curve.  Station-only support promoted laterally displaced
    # outliers to observations, especially during lateral-sweep frames.
    role_sample_points = {
        role: centre_samples + offsets[role] * sample_normals for role in ROLE_ORDER
    }
    support_distance = min(
        float(values["observed_support_radius_m"]),
        float(values["support_residual_max_m"]),
    )
    role_support = {}
    for role in ROLE_ORDER:
        if role not in roles:
            role_support[role] = np.zeros(len(centre_samples), dtype=bool)
            continue
        supported = (
            _nearest_point_distances(role_sample_points[role], role_points[role])
            <= support_distance
        )
        role_support[role] = _remove_short_support_islands(
            supported,
            sample_station,
            float(values["min_observed_segment_m"]),
        )

    local_support_count = np.sum(
        np.vstack([role_support[role] for role in ROLE_ORDER]), axis=0
    ).astype(np.uint8)
    raw_sample_valid = local_support_count >= int(values["min_local_support_roles"])
    raw_sample_valid = _fill_short_internal_gaps(
        raw_sample_valid,
        sample_station,
        float(values["max_structure_bridge_m"]),
    )
    raw_sample_valid = _remove_short_support_islands(
        raw_sample_valid,
        sample_station,
        float(values["min_family_span_m"]),
    )
    centre_segment = np.linalg.norm(np.diff(centre_samples, axis=0), axis=1)
    valid_centre_segment = raw_sample_valid[:-1] & raw_sample_valid[1:]
    strict_valid_family_length = float(np.sum(centre_segment[valid_centre_segment]))
    acceptance_mode = "strict_local_multi_role"
    distributed_audit = {
        "enabled": bool(values["distributed_support_enabled"]),
        "attempted": False,
        "accepted": False,
    }
    if strict_valid_family_length < float(values["min_family_span_m"]):
        distributed_accepted, distributed_support, distributed_audit = (
            _distributed_support_gate(
                roles,
                role_support,
                centre_samples,
                sample_station,
                centre_segment,
                residual,
                values,
            )
        )
        if not distributed_accepted:
            audit.update(
                {
                    "accepted": False,
                    "reason": "insufficient_local_multi_role_support",
                    "valid_family_length_m": strict_valid_family_length,
                    "strict_valid_family_length_m": strict_valid_family_length,
                    "distributed_support": distributed_audit,
                }
            )
            return None, audit
        raw_sample_valid = distributed_support
        acceptance_mode = "distributed_multi_role"
    valid_family_length = _mask_segment_length(raw_sample_valid, centre_segment)

    # Time-bounded shadow ODD promotion.  The shared curve is already accepted
    # from independent multi-role evidence above; this only keeps its fitted
    # geometry continuous inside the configured consumer window.  Direct
    # observations continue to be derived from raw_sample_valid so the policy
    # cannot relabel unsupported pixels as measured evidence.
    completion_mask = np.zeros(len(centre_samples), dtype=bool)
    if bool(values["same_frame_completion_enabled"]):
        completion_mask = (
            (~raw_sample_valid)
            & (centre_samples[:, 0] >= float(values["same_frame_completion_x_min_m"]))
            & (centre_samples[:, 0] <= float(values["same_frame_completion_x_max_m"]))
        )
    sample_valid = raw_sample_valid | completion_mask
    promoted_segments = (
        sample_valid[:-1]
        & sample_valid[1:]
        & ~(raw_sample_valid[:-1] & raw_sample_valid[1:])
    )
    promoted_length_m = float(np.sum(centre_segment[promoted_segments]))

    boundaries = []
    for role in ROLE_ORDER:
        sample_points = role_sample_points[role]
        if role in roles:
            role_mask = point_roles == role
            role_residual = residual[role_mask]
            residual_p50 = float(np.percentile(role_residual, 50))
            residual_p95 = float(np.percentile(role_residual, 95))
            source_mask = int(
                np.bitwise_or.reduce(
                    np.asarray(
                        [node.source_mask for node in roles[role].nodes],
                        dtype=np.uint8,
                    )
                )
            )
            confidence = observed_confidence[role]
        else:
            residual_p50 = residual_p95 = None
            source_mask = 0
            confidence = fallback_confidence

        # Strict frames retain the established local multi-role observation
        # semantics byte-for-byte. In the distributed fallback raw_sample_valid
        # is the bounded union support, so each role's own close evidence is
        # observed while the other family members remain inferred.
        sample_observed = role_support[role] & raw_sample_valid
        sample_state = np.full(len(sample_points), SAMPLE_INVALID, dtype=np.uint8)
        sample_state[sample_valid] = SAMPLE_STRUCTURE_INFERRED
        sample_state[sample_observed] = SAMPLE_OBSERVED
        state = OBSERVED if np.any(sample_observed) else STRUCTURE_INFERRED
        if state != OBSERVED:
            confidence *= float(values["inferred_confidence_scale"])
        segment = np.linalg.norm(np.diff(sample_points, axis=0), axis=1)
        valid_segments = sample_valid[:-1] & sample_valid[1:]
        observed_segments = (
            sample_state[:-1] == SAMPLE_OBSERVED
        ) & (sample_state[1:] == SAMPLE_OBSERVED)
        inferred_segments = valid_segments & ~observed_segments
        invalid_segments = ~valid_segments
        observed_length = float(np.sum(segment[observed_segments]))
        inferred_length = float(np.sum(segment[inferred_segments]))
        invalid_length = float(np.sum(segment[invalid_segments]))
        valid_length = observed_length + inferred_length
        bridged_ratio = inferred_length / max(valid_length, 1e-9)
        boundaries.append(
            SharedFamilyBoundary(
                role=role,
                class_id=2 if role == YELLOW_ROLE else 1,
                state=state,
                sample_points=np.ascontiguousarray(sample_points, dtype=np.float64),
                sample_observed=np.ascontiguousarray(sample_observed),
                sample_state=np.ascontiguousarray(sample_state),
                confidence=float(np.clip(confidence, 0.0, 1.0)),
                source_mask=source_mask,
                observed_length_m=observed_length,
                inferred_length_m=inferred_length,
                invalid_length_m=invalid_length,
                max_inferred_gap_m=_longest_state_length(
                    sample_state, SAMPLE_STRUCTURE_INFERRED, sample_station
                ),
                bridged_length_ratio=float(np.clip(bridged_ratio, 0.0, 1.0)),
                residual_p50_m=residual_p50,
                residual_p95_m=residual_p95,
            )
        )

    weighted_rms = math.sqrt(
        float(np.sum(base_weights * residual * residual))
        / max(float(np.sum(base_weights)), 1e-9)
    )
    inlier = residual <= float(values["support_residual_max_m"])
    if not np.any(inlier):
        audit.update({"accepted": False, "reason": "no_residual_inliers"})
        return None, audit
    valid_samples = centre_samples[sample_valid]
    family = SharedLaneFamily(
        control_points=np.ascontiguousarray(controls, dtype=np.float64),
        knots=np.ascontiguousarray(knots, dtype=np.float64),
        degree=int(degree),
        centre_points=np.ascontiguousarray(centre_samples, dtype=np.float64),
        sample_valid=np.ascontiguousarray(sample_valid),
        local_support_count=np.ascontiguousarray(local_support_count),
        left_width_m=float(left_width),
        right_width_m=float(right_width),
        boundaries=tuple(boundaries),
        valid_x_min_m=float(np.min(valid_samples[:, 0])),
        valid_x_max_m=float(np.max(valid_samples[:, 0])),
        data_residual_p50_m=float(np.percentile(residual, 50)),
        data_residual_p95_m=float(np.percentile(residual, 95)),
        inlier_residual_p50_m=float(np.percentile(residual[inlier], 50)),
        inlier_residual_p95_m=float(np.percentile(residual[inlier], 95)),
        outlier_rate=float(np.mean(~inlier)),
        weighted_rms_m=float(weighted_rms),
        iterations=int(values["robust_iterations"]),
    )
    audit.update(
        {
            "accepted": True,
            "reason": "accepted",
            "acceptance_mode": acceptance_mode,
            "representation": "shared_parametric_bspline_normal_offsets_v2",
            "observed_roles": [role for role in ROLE_ORDER if role in roles],
            "inferred_roles": [role for role in ROLE_ORDER if role not in roles],
            "widths_m": {
                "left": float(left_width),
                "right": float(right_width),
            },
            "data_residual_p50_m": family.data_residual_p50_m,
            "data_residual_p95_m": family.data_residual_p95_m,
            "inlier_residual_p50_m": family.inlier_residual_p50_m,
            "inlier_residual_p95_m": family.inlier_residual_p95_m,
            "outlier_rate": family.outlier_rate,
            "valid_family_length_m": valid_family_length,
            "strict_valid_family_length_m": strict_valid_family_length,
            "distributed_support": distributed_audit,
            "same_frame_completion": {
                "enabled": bool(values["same_frame_completion_enabled"]),
                "x_min_m": float(values["same_frame_completion_x_min_m"]),
                "x_max_m": float(values["same_frame_completion_x_max_m"]),
                "promoted_sample_count": int(np.sum(completion_mask)),
                "promoted_length_m": promoted_length_m,
                "state": STRUCTURE_INFERRED,
                "temporal_prediction_used": False,
                "map_geometry_used": False,
            },
            "local_support_count": {
                "p50": float(np.percentile(local_support_count, 50)),
                "p95": float(np.percentile(local_support_count, 95)),
            },
            "boundary_support": {
                value.role: {
                    "state": value.state,
                    "observed_length_m": value.observed_length_m,
                    "inferred_length_m": value.inferred_length_m,
                    "invalid_length_m": value.invalid_length_m,
                    "max_inferred_gap_m": value.max_inferred_gap_m,
                    "bridged_length_ratio": value.bridged_length_ratio,
                }
                for value in boundaries
            },
            "weighted_rms_m": family.weighted_rms_m,
            "valid_x_m": [family.valid_x_min_m, family.valid_x_max_m],
        }
    )
    return family, audit


def fit_shared_lane_family(
    tracks: Sequence, config: Optional[Mapping] = None
) -> SharedLaneFamily:
    family, audit = fit_shared_lane_family_with_audit(tracks, config)
    if family is None:
        raise ValueError("shared lane-family rejected: {}".format(audit["reason"]))
    return family
