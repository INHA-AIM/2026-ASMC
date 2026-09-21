"""Geometry-first correspondence independent of MGeo identifier names."""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
import math

import numpy as np

from .geometry import cumulative_distance, quantile_or_none, resample_polyline
from .spatial import GridIndex


@dataclass(frozen=True)
class MatchPolicy:
    search_distance_m: float = 8.0
    search_heading_deg: float = 30.0
    strict_distance_m: float = 1.0
    strict_heading_deg: float = 15.0
    tier_a_coverage: float = 0.80
    tier_a_p95_m: float = 0.75
    relaxed_distance_m: float = 1.5
    relaxed_heading_deg: float = 15.0
    tier_b_coverage: float = 0.60


def match_samples(
    points: np.ndarray,
    tangents: np.ndarray,
    target: GridIndex,
    policy: MatchPolicy,
) -> dict:
    distances: list[float] = []
    headings: list[float] = []
    owners: list[str | None] = []
    for point, tangent in zip(points, tangents):
        distance, heading, index = target.query(
            point,
            tangent,
            max_distance_m=policy.search_distance_m,
            max_heading_deg=policy.search_heading_deg,
        )
        distances.append(distance)
        headings.append(heading)
        owners.append(None if index is None or target.owners is None else str(target.owners[index]))

    distance_array = np.asarray(distances, dtype=np.float64)
    heading_array = np.asarray(headings, dtype=np.float64)
    strict = (
        (distance_array <= policy.strict_distance_m)
        & (heading_array <= policy.strict_heading_deg)
    )
    relaxed = (
        (distance_array <= policy.relaxed_distance_m)
        & (heading_array <= policy.relaxed_heading_deg)
    )
    strict_coverage = float(np.mean(strict)) if len(strict) else 0.0
    relaxed_coverage = float(np.mean(relaxed)) if len(relaxed) else 0.0
    strict_p95 = quantile_or_none(distance_array[strict], 0.95)
    relaxed_p95 = quantile_or_none(distance_array[relaxed], 0.95)

    if (
        strict_coverage >= policy.tier_a_coverage
        and strict_p95 is not None
        and strict_p95 <= policy.tier_a_p95_m
    ):
        tier = "A"
    elif relaxed_coverage >= policy.tier_b_coverage:
        tier = "B"
    else:
        tier = "D"

    owner_counter = Counter(owner for owner, keep in zip(owners, relaxed) if owner and keep)
    minimum_support = max(3, int(math.ceil(max(1, int(np.sum(relaxed))) * 0.10)))
    targets = [
        {"id": owner, "support": count}
        for owner, count in owner_counter.most_common()
        if count >= minimum_support
    ]
    return {
        "tier": tier,
        "sample_count": len(points),
        "strict_coverage": strict_coverage,
        "relaxed_coverage": relaxed_coverage,
        "strict_distance_p50_m": quantile_or_none(distance_array[strict], 0.50),
        "strict_distance_p95_m": strict_p95,
        "relaxed_distance_p95_m": relaxed_p95,
        "targets": targets,
        "distances": distance_array,
        "headings": heading_array,
        "strict_mask": strict,
    }


def match_feature(
    feature: dict,
    translation_xyz: np.ndarray,
    sample_step_m: float,
    target: GridIndex,
    policy: MatchPolicy,
) -> dict:
    points, tangents = resample_polyline(feature.get("points", ()), sample_step_m)
    if len(points):
        points = points + translation_xyz
    result = match_samples(points, tangents, target, policy)
    result["source_id"] = str(feature.get("idx", ""))
    return result


def coalesce_boolean_runs(
    values: np.ndarray,
    station_m: np.ndarray,
    close_false_gap_m: float,
    remove_true_island_m: float,
) -> np.ndarray:
    """Close small false gaps and remove tiny true islands deterministically."""

    result = np.asarray(values, dtype=bool).copy()
    if len(result) != len(station_m):
        raise ValueError("values and station_m length mismatch")
    for target_value, maximum_length in (
        (False, close_false_gap_m),
        (True, remove_true_island_m),
    ):
        source = result.copy()
        start = 0
        while start < len(source):
            end = start + 1
            while end < len(source) and source[end] == source[start]:
                end += 1
            run_length = float(station_m[end - 1] - station_m[start])
            # Do not invent support before the first observation or after the
            # last one.  Only an island enclosed by the opposite state can be
            # closed/removed.
            enclosed = start > 0 and end < len(source)
            if enclosed and source[start] == target_value and run_length <= maximum_length:
                result[start:end] = not target_value
            start = end
    return result


def boolean_intervals(values: np.ndarray, points: np.ndarray) -> list[dict]:
    station = cumulative_distance(points)
    intervals: list[dict] = []
    start = 0
    while start < len(values):
        end = start + 1
        while end < len(values) and values[end] == values[start]:
            end += 1
        intervals.append(
            {
                "status": "reuse" if bool(values[start]) else "rebuild",
                "start_index": start,
                "end_index": end - 1,
                "start_s_m": float(station[start]),
                "end_s_m": float(station[end - 1]),
                "length_m": float(station[end - 1] - station[start]),
                "start_xy": [float(v) for v in points[start, :2]],
                "end_xy": [float(v) for v in points[end - 1, :2]],
            }
        )
        start = end
    return intervals
