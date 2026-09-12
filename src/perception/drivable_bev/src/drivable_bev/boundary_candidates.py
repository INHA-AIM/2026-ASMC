"""Shadow-section lane-boundary selection and short-term ID tracking.

The road-marking network predicts colour, not lane-boundary instances.  This
module therefore keeps generic fitted splines separate from the deliberately
narrow two-way shadow-section contract: one yellow centre boundary and, when
visible, one white outer boundary on either side.  It never assigns home or
opposing-lane semantics.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Mapping, Optional, Sequence, Tuple

import numpy as np

from .spline_fitting import FittedLane


@dataclass(frozen=True)
class BoundaryObservation:
    lane: FittedLane
    role: str
    score: float
    reference_y_m: float
    heading_deg: float
    lateral_order_left_to_right: int = 0


@dataclass(frozen=True)
class TrackedBoundary:
    stable_id: int
    observation: BoundaryObservation
    age_frames: int
    reacquired_after_missed_frames: int


@dataclass
class _TrackState:
    stable_id: int
    observation: BoundaryObservation
    age_frames: int = 1
    missed_frames: int = 0


@dataclass(frozen=True)
class _TrackCandidate:
    track: object
    score: float
    reference_y_m: float
    heading_deg: float


DEFAULT_SHADOW_BOUNDARY_CONFIG = {
    "reference_x_m": 5.0,
    "min_confidence": 0.50,
    "min_longitudinal_span_m": 3.0,
    "min_observed_length_m": 3.0,
    "max_bridged_ratio": 0.45,
    "max_residual_p95_m": 0.30,
    "max_abs_heading_deg": 50.0,
    "max_abs_lateral_m": 7.0,
    "lane_width_min_m": 2.4,
    "lane_width_max_m": 4.6,
    "expected_lane_width_m": 3.55,
    "max_white_without_yellow": 2,
}


def _config(values: Optional[Mapping]) -> dict:
    result = dict(DEFAULT_SHADOW_BOUNDARY_CONFIG)
    result.update(dict(values or {}))
    positive = (
        "min_longitudinal_span_m",
        "min_observed_length_m",
        "max_residual_p95_m",
        "max_abs_lateral_m",
        "lane_width_min_m",
        "lane_width_max_m",
        "expected_lane_width_m",
    )
    if any(float(result[key]) <= 0.0 for key in positive):
        raise ValueError("shadow boundary distances must be positive")
    if not 0.0 <= float(result["min_confidence"]) <= 1.0:
        raise ValueError("shadow boundary confidence must lie in [0,1]")
    if not 0.0 <= float(result["max_bridged_ratio"]) <= 1.0:
        raise ValueError("max_bridged_ratio must lie in [0,1]")
    if not 0.0 < float(result["max_abs_heading_deg"]) < 90.0:
        raise ValueError("max_abs_heading_deg must lie in (0,90)")
    if float(result["lane_width_max_m"]) <= float(result["lane_width_min_m"]):
        raise ValueError("lane width bounds are invalid")
    if int(result["max_white_without_yellow"]) < 1:
        raise ValueError("max_white_without_yellow must be positive")
    return result


def _ordered_points(lane: FittedLane) -> np.ndarray:
    points = np.asarray(lane.sample_points, dtype=np.float64).reshape(-1, 2)
    if len(points) < 2:
        return points
    return points[np.argsort(points[:, 0], kind="mergesort")]


def lane_y_at_x(lane: FittedLane, x_m: float) -> float:
    points = _ordered_points(lane)
    if not len(points):
        return float("nan")
    unique_x, indices = np.unique(points[:, 0], return_index=True)
    y = points[indices, 1]
    target = float(np.clip(float(x_m), unique_x[0], unique_x[-1]))
    return float(np.interp(target, unique_x, y))


def lane_heading_deg(lane: FittedLane) -> float:
    points = _ordered_points(lane)
    if len(points) < 2:
        return 90.0
    centered = points - np.mean(points, axis=0)
    covariance = centered.T @ centered
    values, vectors = np.linalg.eigh(covariance)
    direction = vectors[:, int(np.argmax(values))]
    if direction[0] < 0.0:
        direction = -direction
    return float(math.degrees(math.atan2(direction[1], direction[0])))


def _lane_score(lane: FittedLane, heading_deg: float) -> float:
    span = max(0.0, float(lane.valid_x_max_m - lane.valid_x_min_m))
    length_factor = min(1.0, float(lane.observed_length_m) / 15.0)
    span_factor = min(1.0, span / 12.0)
    bridge_factor = max(0.0, 1.0 - float(lane.bridged_length_ratio))
    residual_factor = math.exp(-max(0.0, float(lane.residual_p95_m)) / 0.20)
    heading_factor = max(0.0, math.cos(math.radians(abs(heading_deg))))
    return float(
        lane.confidence
        * (0.35 + 0.65 * length_factor)
        * (0.35 + 0.65 * span_factor)
        * bridge_factor
        * residual_factor
        * heading_factor
    )


def _track_y_at_x(track, x_m: float) -> float:
    points = np.asarray(track.xy, dtype=np.float64).reshape(-1, 2)
    order = np.argsort(points[:, 0], kind="mergesort")
    points = points[order]
    unique_x, indices = np.unique(points[:, 0], return_index=True)
    target = float(np.clip(float(x_m), unique_x[0], unique_x[-1]))
    return float(np.interp(target, unique_x, points[indices, 1]))


def _track_candidate(track, values: Mapping) -> Optional[_TrackCandidate]:
    points = np.asarray(track.xy, dtype=np.float64).reshape(-1, 2)
    if len(points) < 2:
        return None
    span = float(np.max(points[:, 0]) - np.min(points[:, 0]))
    if span < float(values["min_longitudinal_span_m"]):
        return None
    centered = points - np.mean(points, axis=0)
    covariance = centered.T @ centered
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    direction = eigenvectors[:, int(np.argmax(eigenvalues))]
    if direction[0] < 0.0:
        direction = -direction
    heading = float(math.degrees(math.atan2(direction[1], direction[0])))
    if abs(heading) > float(values["max_abs_heading_deg"]):
        return None
    reference_y = _track_y_at_x(track, float(values["reference_x_m"]))
    if abs(reference_y) > float(values["max_abs_lateral_m"]):
        return None
    node_weights = np.asarray(
        [max(1e-6, node.geometric_quality * math.sqrt(node.support)) for node in track.nodes],
        dtype=np.float64,
    )
    confidence = float(
        np.average([node.confidence for node in track.nodes], weights=node_weights)
    )
    if confidence < float(values["min_confidence"]):
        return None
    score = confidence * min(1.0, span / 15.0) * max(
        0.0, math.cos(math.radians(abs(heading)))
    )
    return _TrackCandidate(track, float(score), reference_y, heading)


def _track_separation(first: _TrackCandidate, second: _TrackCandidate) -> float:
    first_points = np.asarray(first.track.xy, dtype=np.float64)
    second_points = np.asarray(second.track.xy, dtype=np.float64)
    lower = max(float(first_points[:, 0].min()), float(second_points[:, 0].min()))
    upper = min(float(first_points[:, 0].max()), float(second_points[:, 0].max()))
    if upper - lower < 1.0:
        return second.reference_y_m - first.reference_y_m
    samples = np.linspace(lower, upper, max(3, int(math.ceil(upper - lower)) + 1))
    return float(
        np.median(
            [
                _track_y_at_x(second.track, value)
                - _track_y_at_x(first.track, value)
                for value in samples
            ]
        )
    )


def select_shadow_lane_tracks(tracks: Sequence, config: Optional[Mapping] = None) -> tuple:
    """Preselect at most three shadow-boundary tracks before cubic fitting."""
    values = _config(config)
    candidates = [
        value for value in (_track_candidate(track, values) for track in tracks)
        if value is not None
    ]
    yellow = sorted(
        (item for item in candidates if item.track.class_id == 2),
        key=lambda item: (-item.score, abs(item.reference_y_m)),
    )
    whites = [item for item in candidates if item.track.class_id == 1]
    selected = []
    if yellow:
        centre = yellow[0]
        selected.append(centre)
        width_min = float(values["lane_width_min_m"])
        width_max = float(values["lane_width_max_m"])
        expected = float(values["expected_lane_width_m"])
        for sign in (1.0, -1.0):
            matches = []
            for item in whites:
                signed = _track_separation(centre, item)
                if signed * sign <= 0.0 or not width_min <= abs(signed) <= width_max:
                    continue
                width_score = math.exp(-abs(abs(signed) - expected) / expected)
                matches.append((item.score * width_score, item))
            if matches:
                selected.append(sorted(matches, key=lambda value: -value[0])[0][1])
    else:
        for item in sorted(whites, key=lambda value: -value.score):
            if any(abs(item.reference_y_m - other.reference_y_m) < 1.5 for other in selected):
                continue
            selected.append(item)
            if len(selected) >= int(values["max_white_without_yellow"]):
                break
    return tuple(
        item.track
        for item in sorted(selected, key=lambda value: -value.reference_y_m)
    )


def _eligible_observations(lanes: Sequence[FittedLane], values: Mapping) -> list:
    result = []
    reference_x = float(values["reference_x_m"])
    for lane in lanes:
        if int(lane.class_id) not in (1, 2):
            continue
        span = float(lane.valid_x_max_m - lane.valid_x_min_m)
        heading = lane_heading_deg(lane)
        reference_y = lane_y_at_x(lane, reference_x)
        if not np.isfinite(reference_y):
            continue
        if float(lane.confidence) < float(values["min_confidence"]):
            continue
        if span < float(values["min_longitudinal_span_m"]):
            continue
        if float(lane.observed_length_m) < float(values["min_observed_length_m"]):
            continue
        if float(lane.bridged_length_ratio) > float(values["max_bridged_ratio"]):
            continue
        if float(lane.residual_p95_m) > float(values["max_residual_p95_m"]):
            continue
        if abs(heading) > float(values["max_abs_heading_deg"]):
            continue
        if abs(reference_y) > float(values["max_abs_lateral_m"]):
            continue
        result.append(
            BoundaryObservation(
                lane=lane,
                role="unresolved",
                score=_lane_score(lane, heading),
                reference_y_m=reference_y,
                heading_deg=heading,
            )
        )
    return result


def _separation(first: BoundaryObservation, second: BoundaryObservation) -> float:
    lower = max(first.lane.valid_x_min_m, second.lane.valid_x_min_m)
    upper = min(first.lane.valid_x_max_m, second.lane.valid_x_max_m)
    if upper - lower < 1.0:
        return second.reference_y_m - first.reference_y_m
    samples = np.linspace(lower, upper, max(3, int(math.ceil(upper - lower)) + 1))
    differences = [
        lane_y_at_x(second.lane, x_value) - lane_y_at_x(first.lane, x_value)
        for x_value in samples
    ]
    return float(np.median(differences))


def select_shadow_boundaries(
    lanes: Sequence[FittedLane], config: Optional[Mapping] = None
) -> Tuple[BoundaryObservation, ...]:
    """Select colour-aware boundary candidates for the two-way shadow ODD.

    The output is ordered from physical left to right (positive to negative
    ``base_link`` y). Missing observations are not synthesized.
    """
    values = _config(config)
    eligible = _eligible_observations(lanes, values)
    yellow = sorted(
        (item for item in eligible if item.lane.class_id == 2),
        key=lambda item: (-item.score, abs(item.reference_y_m), item.lane.lane_id),
    )
    white = [item for item in eligible if item.lane.class_id == 1]
    selected = []
    if yellow:
        centre = yellow[0]
        selected.append(
            BoundaryObservation(
                centre.lane,
                "yellow_centre_boundary",
                centre.score,
                centre.reference_y_m,
                centre.heading_deg,
            )
        )
        width_min = float(values["lane_width_min_m"])
        width_max = float(values["lane_width_max_m"])
        expected = float(values["expected_lane_width_m"])
        by_side = {"left_outer_boundary": [], "right_outer_boundary": []}
        for item in white:
            signed = _separation(centre, item)
            distance = abs(signed)
            if not width_min <= distance <= width_max:
                continue
            role = "left_outer_boundary" if signed > 0.0 else "right_outer_boundary"
            width_score = math.exp(-abs(distance - expected) / max(expected, 1e-6))
            by_side[role].append((item.score * width_score, item))
        for role in ("left_outer_boundary", "right_outer_boundary"):
            if not by_side[role]:
                continue
            score, item = sorted(
                by_side[role], key=lambda value: (-value[0], value[1].lane.lane_id)
            )[0]
            selected.append(
                BoundaryObservation(
                    item.lane,
                    role,
                    float(score),
                    item.reference_y_m,
                    item.heading_deg,
                )
            )
    else:
        # Keep a conservative pair for continuity when the yellow line is
        # briefly occluded. Their semantic role remains unresolved.
        chosen = []
        for item in sorted(white, key=lambda value: (-value.score, value.lane.lane_id)):
            if any(abs(item.reference_y_m - other.reference_y_m) < 1.5 for other in chosen):
                continue
            chosen.append(item)
            if len(chosen) >= int(values["max_white_without_yellow"]):
                break
        selected.extend(chosen)

    ordered = sorted(
        selected,
        key=lambda item: (-item.reference_y_m, item.lane.class_id, item.lane.lane_id),
    )
    return tuple(
        BoundaryObservation(
            lane=item.lane,
            role=item.role,
            score=item.score,
            reference_y_m=item.reference_y_m,
            heading_deg=item.heading_deg,
            lateral_order_left_to_right=index,
        )
        for index, item in enumerate(ordered)
    )


def boundary_lateral_residual(
    first: BoundaryObservation, second: BoundaryObservation
) -> float:
    if first.lane.class_id != second.lane.class_id:
        return float("inf")
    lower = max(first.lane.valid_x_min_m, second.lane.valid_x_min_m)
    upper = min(first.lane.valid_x_max_m, second.lane.valid_x_max_m)
    if upper - lower < 1.0:
        return abs(first.reference_y_m - second.reference_y_m)
    samples = np.linspace(lower, upper, max(4, int(math.ceil(upper - lower)) + 1))
    residual = [
        abs(lane_y_at_x(first.lane, x_value) - lane_y_at_x(second.lane, x_value))
        for x_value in samples
    ]
    return float(np.median(residual))


class TemporalBoundaryTracker:
    """Assign stable local IDs without hallucinating unobserved boundaries."""

    def __init__(
        self,
        max_lateral_residual_m: float = 0.65,
        max_missed_frames: int = 3,
        reset_gap_sec: float = 0.75,
    ) -> None:
        if max_lateral_residual_m <= 0.0 or max_missed_frames < 0 or reset_gap_sec <= 0.0:
            raise ValueError("temporal boundary tracker settings are invalid")
        self.max_lateral_residual_m = float(max_lateral_residual_m)
        self.max_missed_frames = int(max_missed_frames)
        self.reset_gap_ns = int(round(float(reset_gap_sec) * 1e9))
        self._tracks = {}
        self._next_id = 1
        self._last_stamp_ns = None

    def reset(self) -> None:
        self._tracks = {}
        self._last_stamp_ns = None

    def update(
        self, observations: Sequence[BoundaryObservation], stamp_ns: int
    ) -> Tuple[TrackedBoundary, ...]:
        stamp_ns = int(stamp_ns)
        if stamp_ns <= 0:
            raise ValueError("boundary timestamp must be positive")
        if self._last_stamp_ns is not None and (
            stamp_ns <= self._last_stamp_ns
            or stamp_ns - self._last_stamp_ns > self.reset_gap_ns
        ):
            self.reset()
        self._last_stamp_ns = stamp_ns

        candidates = []
        for stable_id, state in self._tracks.items():
            for index, observation in enumerate(observations):
                residual = boundary_lateral_residual(state.observation, observation)
                if residual <= self.max_lateral_residual_m:
                    role_penalty = 0.0 if state.observation.role == observation.role else 0.05
                    candidates.append((residual + role_penalty, stable_id, index))
        matched_tracks = set()
        matched_observations = set()
        output = []
        for _, stable_id, index in sorted(candidates):
            if stable_id in matched_tracks or index in matched_observations:
                continue
            state = self._tracks[stable_id]
            missed = state.missed_frames
            state.observation = observations[index]
            state.age_frames += 1
            state.missed_frames = 0
            matched_tracks.add(stable_id)
            matched_observations.add(index)
            output.append(
                TrackedBoundary(stable_id, observations[index], state.age_frames, missed)
            )

        for stable_id, state in list(self._tracks.items()):
            if stable_id in matched_tracks:
                continue
            state.missed_frames += 1
            if state.missed_frames > self.max_missed_frames:
                del self._tracks[stable_id]

        for index, observation in enumerate(observations):
            if index in matched_observations:
                continue
            stable_id = self._next_id
            self._next_id += 1
            self._tracks[stable_id] = _TrackState(stable_id, observation)
            output.append(TrackedBoundary(stable_id, observation, 1, 0))

        return tuple(
            sorted(
                output,
                key=lambda item: item.observation.lateral_order_left_to_right,
            )
        )
