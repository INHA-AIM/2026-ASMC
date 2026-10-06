"""Planning-consumer continuity metrics for a shared lane family.

The shared-family producer assigns support to sampled boundary points, while
the read-only planning adapter consumes intervals whose two endpoints are both
valid.  This module mirrors that interval rule without depending on ROS, so an
offline replay can measure the geometry that planning can actually consume.

It deliberately does not repair gaps.  In particular, internal occlusions and
unobserved tails are reported separately so a later bridge experiment cannot
silently turn extrapolation into a usable lane strip.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Iterable, Sequence, Tuple

import numpy as np

from .shared_lane_family import (
    LEFT_ROLE,
    RIGHT_ROLE,
    SAMPLE_INVALID,
    SAMPLE_OBSERVED,
    SAMPLE_STRUCTURE_INFERRED,
    SharedLaneFamily,
    YELLOW_ROLE,
)


@dataclass(frozen=True)
class LongitudinalZone:
    """One longitudinal audit interval in ``base_link`` metres."""

    name: str
    x_min_m: float
    x_max_m: float

    def __post_init__(self):
        if not np.isfinite((self.x_min_m, self.x_max_m)).all():
            raise ValueError("planning-usability zone bounds must be finite")
        if self.x_max_m <= self.x_min_m:
            raise ValueError("planning-usability zone must have positive length")


# ``planning_consumer`` matches the current read-only adapter.  The remaining
# zones explain where a failure occurs instead of hiding it in one aggregate.
DEFAULT_PLANNING_ZONES = (
    LongitudinalZone("planning_consumer", 0.0, 15.0),
    LongitudinalZone("vehicle_envelope", -0.790, 3.845),
    LongitudinalZone("near_ahead", 3.845, 10.0),
    LongitudinalZone("mid_ahead", 10.0, 15.0),
    LongitudinalZone("far_debug", 15.0, 20.0),
)


def planning_zones_from_config(values=None):
    """Load named audit zones from a ROS/YAML-compatible sequence."""
    if values is None:
        return DEFAULT_PLANNING_ZONES
    if not isinstance(values, (list, tuple)) or not values:
        raise ValueError("planning-usability zones must be a non-empty sequence")
    zones = []
    for value in values:
        if not isinstance(value, dict):
            raise ValueError("each planning-usability zone must be a mapping")
        unknown = sorted(set(value) - {"name", "x_min_m", "x_max_m"})
        missing = sorted({"name", "x_min_m", "x_max_m"} - set(value))
        if unknown or missing:
            raise ValueError(
                "invalid planning-usability zone keys: missing={} unknown={}".format(
                    missing, unknown
                )
            )
        zones.append(
            LongitudinalZone(
                str(value["name"]),
                float(value["x_min_m"]),
                float(value["x_max_m"]),
            )
        )
    result = tuple(zones)
    if len({zone.name for zone in result}) != len(result):
        raise ValueError("planning-usability zone names must be unique")
    return result


def _merge_intervals(intervals: Iterable[Tuple[float, float]], tolerance=1e-9):
    values = sorted(
        (float(start), float(end))
        for start, end in intervals
        if np.isfinite((start, end)).all() and end > start
    )
    if not values:
        return ()
    merged = [list(values[0])]
    for start, end in values[1:]:
        if start <= merged[-1][1] + float(tolerance):
            merged[-1][1] = max(merged[-1][1], end)
        else:
            merged.append([start, end])
    return tuple((float(start), float(end)) for start, end in merged)


def _clip_intervals(intervals, zone):
    return _merge_intervals(
        (
            max(float(start), zone.x_min_m),
            min(float(end), zone.x_max_m),
        )
        for start, end in intervals
    )


def _interval_length(intervals):
    return float(sum(end - start for start, end in intervals))


def _subtract_intervals(base, removed):
    """Return the portion of merged ``base`` not covered by ``removed``."""
    result = []
    removed = _merge_intervals(removed)
    for start, end in _merge_intervals(base):
        cursor = start
        for cut_start, cut_end in removed:
            if cut_end <= cursor:
                continue
            if cut_start >= end:
                break
            if cut_start > cursor:
                result.append((cursor, min(cut_start, end)))
            cursor = max(cursor, cut_end)
            if cursor >= end:
                break
        if cursor < end:
            result.append((cursor, end))
    return _merge_intervals(result)


def _gap_metrics(usable, zone, anchor_tolerance_m):
    usable = _clip_intervals(usable, zone)
    gaps = []
    cursor = zone.x_min_m
    for start, end in usable:
        if start > cursor:
            gaps.append((cursor, start))
        cursor = max(cursor, end)
    if cursor < zone.x_max_m:
        gaps.append((cursor, zone.x_max_m))

    internal = [
        (start, end)
        for start, end in gaps
        if start > zone.x_min_m + 1e-9 and end < zone.x_max_m - 1e-9
    ]
    start_anchored = bool(
        usable and usable[0][0] <= zone.x_min_m + float(anchor_tolerance_m)
    )
    first_usable_x_m = float(usable[0][0]) if usable else None
    last_usable_x_m = float(usable[-1][1]) if usable else None
    leading_gap_m = (
        float(max(0.0, usable[0][0] - zone.x_min_m))
        if usable
        else float(zone.x_max_m - zone.x_min_m)
    )
    trailing_gap_m = (
        float(max(0.0, zone.x_max_m - usable[-1][1]))
        if usable
        else float(zone.x_max_m - zone.x_min_m)
    )
    continuous_horizon = 0.0
    if start_anchored:
        continuous_horizon = max(0.0, usable[0][1] - zone.x_min_m)
    return {
        "usable_interval_count": int(len(usable)),
        "first_usable_x_m": first_usable_x_m,
        "last_usable_x_m": last_usable_x_m,
        "leading_unusable_gap_m": leading_gap_m,
        "trailing_unusable_gap_m": trailing_gap_m,
        "start_anchored": start_anchored,
        "continuous_from_start_m": float(continuous_horizon),
        "longest_unusable_gap_m": float(
            max((end - start for start, end in gaps), default=0.0)
        ),
        "longest_internal_unusable_gap_m": float(
            max((end - start for start, end in internal), default=0.0)
        ),
        "internal_unusable_gap_count": int(len(internal)),
    }


def _summarize_strip(usable, observed, zone, anchor_tolerance_m):
    usable = _clip_intervals(usable, zone)
    observed = _clip_intervals(observed, zone)
    degraded = _subtract_intervals(usable, observed)
    length = zone.x_max_m - zone.x_min_m
    result = {
        "available_length_m": _interval_length(usable),
        "observed_length_m": _interval_length(observed),
        "degraded_length_m": _interval_length(degraded),
    }
    result.update(
        {
            "coverage_ratio": result["available_length_m"] / length,
            "observed_coverage_ratio": result["observed_length_m"] / length,
            "degraded_coverage_ratio": result["degraded_length_m"] / length,
            "fully_continuous": bool(
                result["available_length_m"] >= length - 1e-6
            ),
        }
    )
    result.update(_gap_metrics(usable, zone, anchor_tolerance_m))
    return result


def _support_intervals(family: SharedLaneFamily):
    points = np.asarray(family.centre_points, dtype=np.float64).reshape(-1, 2)
    if len(points) < 2:
        return {name: {"usable": (), "observed": ()} for name in (
            "left_strip", "right_strip", "both_strips"
        )}
    states = {
        role: np.asarray(family.boundary(role).sample_state, dtype=np.uint8).reshape(-1)
        for role in (LEFT_ROLE, YELLOW_ROLE, RIGHT_ROLE)
    }
    if any(len(value) != len(points) for value in states.values()):
        raise ValueError("shared-family points and support-state counts differ")
    valid_states = {
        SAMPLE_INVALID,
        SAMPLE_OBSERVED,
        SAMPLE_STRUCTURE_INFERRED,
    }
    unknown = sorted(
        set(int(item) for value in states.values() for item in value) - valid_states
    )
    if unknown:
        raise ValueError("unknown shared-family support states: {}".format(unknown))

    pairs = {
        "left_strip": (LEFT_ROLE, YELLOW_ROLE),
        "right_strip": (YELLOW_ROLE, RIGHT_ROLE),
        "both_strips": (LEFT_ROLE, YELLOW_ROLE, RIGHT_ROLE),
    }
    result = {
        name: {"usable": [], "observed": []} for name in pairs
    }
    for index in range(len(points) - 1):
        start_x = float(min(points[index, 0], points[index + 1, 0]))
        end_x = float(max(points[index, 0], points[index + 1, 0]))
        if end_x <= start_x:
            continue
        for name, roles in pairs.items():
            endpoints = [
                int(states[role][sample_index])
                for role in roles
                for sample_index in (index, index + 1)
            ]
            if all(value != SAMPLE_INVALID for value in endpoints):
                result[name]["usable"].append((start_x, end_x))
                if all(value == SAMPLE_OBSERVED for value in endpoints):
                    result[name]["observed"].append((start_x, end_x))
    return {
        name: {
            key: _merge_intervals(intervals)
            for key, intervals in value.items()
        }
        for name, value in result.items()
    }


def measure_planning_usability(
    family: SharedLaneFamily | None,
    zones: Sequence[LongitudinalZone] = DEFAULT_PLANNING_ZONES,
    anchor_tolerance_m: float = 0.26,
) -> Dict:
    """Measure strip continuity using the planning adapter's interval rule."""
    if not np.isfinite(anchor_tolerance_m) or anchor_tolerance_m < 0.0:
        raise ValueError("anchor tolerance must be finite and non-negative")
    checked_zones = tuple(zones)
    if not checked_zones:
        raise ValueError("at least one planning-usability zone is required")
    if len({zone.name for zone in checked_zones}) != len(checked_zones):
        raise ValueError("planning-usability zone names must be unique")

    intervals = (
        _support_intervals(family)
        if family is not None
        else {
            name: {"usable": (), "observed": ()}
            for name in ("left_strip", "right_strip", "both_strips")
        }
    )
    return {
        "available": family is not None,
        "interval_contract": "both_endpoint_support_non_invalid",
        "anchor_tolerance_m": float(anchor_tolerance_m),
        "zones": {
            zone.name: {
                "x_min_m": float(zone.x_min_m),
                "x_max_m": float(zone.x_max_m),
                **{
                    name: _summarize_strip(
                        intervals[name]["usable"],
                        intervals[name]["observed"],
                        zone,
                        anchor_tolerance_m,
                    )
                    for name in (
                        "left_strip",
                        "right_strip",
                        "both_strips",
                    )
                },
            }
            for zone in checked_zones
        },
    }
