"""Geometry and input-policy helpers for observed lane calibration.

The functions in this module deliberately know nothing about the old raster
baker.  They operate on ordered observations and canonical metric polylines.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

import numpy as np

from .geometry import cumulative_distance, points_xyz, resample_polyline


def truthy_csv(value: object) -> bool:
    return str(value or "").strip().lower() in {"1", "1.0", "true", "yes"}


def classify_observation_row(row: dict) -> str:
    """Return the mutually-exclusive provenance class for a CSV row."""

    if truthy_csv(row.get("generated")):
        return "generated"
    if truthy_csv(row.get("interpolated")):
        return "interpolated"
    return "direct"


def undirected_heading_error_deg(first: np.ndarray, second: np.ndarray) -> float:
    first = np.asarray(first, dtype=np.float64)[:2]
    second = np.asarray(second, dtype=np.float64)[:2]
    first_norm = float(np.linalg.norm(first))
    second_norm = float(np.linalg.norm(second))
    if first_norm < 1e-12 or second_norm < 1e-12:
        return 180.0
    cosine = abs(float(np.dot(first, second) / (first_norm * second_norm)))
    return math.degrees(math.acos(float(np.clip(cosine, -1.0, 1.0))))


def split_ordered_observations(
    rows: list[dict], *, max_time_gap_sec: float, max_point_gap_m: float,
) -> list[list[dict]]:
    """Split an already grouped observation stream at temporal/spatial gaps."""

    result: list[list[dict]] = []
    current: list[dict] = []
    previous_time: float | None = None
    previous_xy: np.ndarray | None = None
    for row in rows:
        timestamp = float(row["timestamp"])
        xy = np.asarray([float(row["lane_map_x"]), float(row["lane_map_y"])])
        split = False
        if previous_time is not None:
            delta_time = timestamp - previous_time
            split = delta_time <= 0.0 or delta_time > max_time_gap_sec
        if previous_xy is not None:
            split = split or float(np.linalg.norm(xy - previous_xy)) > max_point_gap_m
        if split and current:
            result.append(current)
            current = []
        current.append(row)
        previous_time = timestamp
        previous_xy = xy
    if current:
        result.append(current)
    return result


def resample_observation_rows(rows: list[dict], step_m: float) -> tuple[np.ndarray, np.ndarray]:
    points = np.asarray([
        [float(row["lane_map_x"]), float(row["lane_map_y"]), 0.0] for row in rows
    ], dtype=np.float64)
    return resample_polyline(points, step_m)


@dataclass(frozen=True)
class Projection:
    owner_id: str
    station_m: float
    distance_m: float
    signed_offset_m: float
    heading_error_deg: float
    point_xyz: tuple[float, float, float]


class PolylineProjectionIndex:
    """Cell-indexed exact point-to-segment projection for named polylines."""

    def __init__(self, polylines: dict[str, Iterable[Iterable[float]]], cell_size_m: float = 2.0):
        if cell_size_m <= 0:
            raise ValueError("cell_size_m must be positive")
        self.cell_size_m = float(cell_size_m)
        self.starts: list[np.ndarray] = []
        self.vectors: list[np.ndarray] = []
        self.lengths: list[float] = []
        self.stations: list[float] = []
        self.owners: list[str] = []
        self.z_starts: list[float] = []
        self.z_ends: list[float] = []
        self.cells: dict[tuple[int, int], list[int]] = {}
        for owner_id in sorted(polylines):
            points = points_xyz(polylines[owner_id])
            stations = cumulative_distance(points)
            for index, (start, end) in enumerate(zip(points[:-1], points[1:])):
                vector = end[:2] - start[:2]
                length = float(np.linalg.norm(vector))
                if length < 1e-9:
                    continue
                segment_id = len(self.starts)
                self.starts.append(start[:2].copy())
                self.vectors.append(vector)
                self.lengths.append(length)
                self.stations.append(float(stations[index]))
                self.owners.append(owner_id)
                self.z_starts.append(float(start[2]))
                self.z_ends.append(float(end[2]))
                low = np.floor(np.minimum(start[:2], end[:2]) / self.cell_size_m).astype(int)
                high = np.floor(np.maximum(start[:2], end[:2]) / self.cell_size_m).astype(int)
                for cell_x in range(int(low[0]), int(high[0]) + 1):
                    for cell_y in range(int(low[1]), int(high[1]) + 1):
                        self.cells.setdefault((cell_x, cell_y), []).append(segment_id)

    def query(
        self, point: Iterable[float], tangent: Iterable[float], *,
        max_distance_m: float, max_heading_deg: float,
    ) -> list[Projection]:
        point_xy = np.asarray(point, dtype=np.float64)[:2]
        tangent_xy = np.asarray(tangent, dtype=np.float64)[:2]
        cell = np.floor(point_xy / self.cell_size_m).astype(int)
        radius = int(math.ceil(max_distance_m / self.cell_size_m)) + 1
        segment_ids: set[int] = set()
        for cell_x in range(int(cell[0]) - radius, int(cell[0]) + radius + 1):
            for cell_y in range(int(cell[1]) - radius, int(cell[1]) + radius + 1):
                segment_ids.update(self.cells.get((cell_x, cell_y), ()))
        best: dict[str, Projection] = {}
        for segment_id in sorted(segment_ids):
            start = self.starts[segment_id]
            vector = self.vectors[segment_id]
            length = self.lengths[segment_id]
            parameter = float(np.clip(np.dot(point_xy - start, vector) / (length * length), 0.0, 1.0))
            projected = start + parameter * vector
            delta = point_xy - projected
            distance = float(np.linalg.norm(delta))
            if distance > max_distance_m:
                continue
            heading_error = undirected_heading_error_deg(tangent_xy, vector)
            if heading_error > max_heading_deg:
                continue
            unit_tangent = vector / length
            normal = np.asarray([-unit_tangent[1], unit_tangent[0]])
            signed_offset = float(np.dot(delta, normal))
            owner_id = self.owners[segment_id]
            z = self.z_starts[segment_id] + parameter * (
                self.z_ends[segment_id] - self.z_starts[segment_id]
            )
            projection = Projection(
                owner_id=owner_id,
                station_m=self.stations[segment_id] + parameter * length,
                distance_m=distance,
                signed_offset_m=signed_offset,
                heading_error_deg=heading_error,
                point_xyz=(float(projected[0]), float(projected[1]), float(z)),
            )
            previous = best.get(owner_id)
            if previous is None or (distance, heading_error) < (
                previous.distance_m, previous.heading_error_deg,
            ):
                best[owner_id] = projection
        return sorted(best.values(), key=lambda item: (item.distance_m, item.heading_error_deg, item.owner_id))


def associate_projection_sequence(
    candidates: list[list[Projection]], connected_pairs: set[frozenset[str]],
    *, search_distance_m: float, max_heading_deg: float,
    connected_transition_cost: float,
) -> list[Projection | None]:
    """Viterbi association that forbids jumps between unrelated boundaries."""

    previous_costs: dict[str | None, float] = {None: 0.0}
    backtrace: list[dict[str | None, str | None]] = []
    states_by_step: list[dict[str | None, Projection | None]] = []
    for options in candidates:
        states = {item.owner_id: item for item in options}
        states[None] = None
        costs: dict[str | None, float] = {}
        parents: dict[str | None, str | None] = {}
        for state, projection in states.items():
            emission = 1.15 if projection is None else (
                projection.distance_m / max(search_distance_m, 1e-9)
                + 0.25 * projection.heading_error_deg / max(max_heading_deg, 1e-9)
            )
            best_cost = math.inf
            best_parent: str | None = None
            for previous, previous_cost in previous_costs.items():
                if previous is None or state is None:
                    transition = 0.35 if previous != state else 0.0
                elif previous == state:
                    transition = 0.0
                elif frozenset((previous, state)) in connected_pairs:
                    transition = connected_transition_cost
                else:
                    continue
                candidate_cost = previous_cost + transition + emission
                if candidate_cost < best_cost:
                    best_cost = candidate_cost
                    best_parent = previous
            costs[state] = best_cost
            parents[state] = best_parent
        previous_costs = costs
        backtrace.append(parents)
        states_by_step.append(states)
    if not previous_costs:
        return [None] * len(candidates)
    state = min(previous_costs, key=lambda item: (previous_costs[item], str(item)))
    result: list[Projection | None] = [None] * len(candidates)
    for index in range(len(candidates) - 1, -1, -1):
        result[index] = states_by_step[index].get(state)
        state = backtrace[index].get(state)
    return result


def robust_station_profile(
    stations: Iterable[float], offsets: Iterable[float], *,
    bin_size_m: float, mad_multiplier: float,
) -> tuple[list[dict], np.ndarray]:
    stations_array = np.asarray(list(stations), dtype=np.float64)
    offsets_array = np.asarray(list(offsets), dtype=np.float64)
    if len(stations_array) != len(offsets_array):
        raise ValueError("stations and offsets must have equal length")
    inlier = np.zeros(len(stations_array), dtype=bool)
    profile: list[dict] = []
    if len(stations_array) == 0:
        return profile, inlier
    bins = np.floor(stations_array / bin_size_m).astype(int)
    for bin_id in sorted(set(bins.tolist())):
        indices = np.flatnonzero(bins == bin_id)
        values = offsets_array[indices]
        median = float(np.median(values))
        mad = float(np.median(np.abs(values - median)))
        tolerance = max(0.02, mad_multiplier * 1.4826 * mad)
        accepted = indices[np.abs(values - median) <= tolerance]
        inlier[accepted] = True
        if len(accepted) == 0:
            continue
        profile.append({
            "station_m": float(np.median(stations_array[accepted])),
            "offset_m": float(np.median(offsets_array[accepted])),
            "support": int(len(accepted)),
            "mad_m": mad,
            "bin_index": int(bin_id),
        })
    return profile, inlier


def interpolate_profile(profile: list[dict], station_m: float, max_gap_m: float) -> float | None:
    if not profile:
        return None
    stations = np.asarray([row["station_m"] for row in profile], dtype=np.float64)
    offsets = np.asarray([row["offset_m"] for row in profile], dtype=np.float64)
    if station_m < stations[0] - 1e-9 or station_m > stations[-1] + 1e-9:
        return None
    index = int(np.searchsorted(stations, station_m))
    if index < len(stations) and abs(float(stations[index]) - station_m) <= 1e-9:
        return float(offsets[index])
    if index == 0:
        return float(offsets[0])
    if index >= len(stations):
        return float(offsets[-1])
    if stations[index] - stations[index - 1] > max_gap_m + 1e-9:
        return None
    ratio = (station_m - stations[index - 1]) / max(stations[index] - stations[index - 1], 1e-12)
    return float(offsets[index - 1] + ratio * (offsets[index] - offsets[index - 1]))


def profile_runs(profile: list[dict], max_gap_m: float) -> list[list[dict]]:
    if not profile:
        return []
    ordered = sorted(profile, key=lambda row: row["station_m"])
    runs: list[list[dict]] = [[ordered[0]]]
    for row in ordered[1:]:
        if row["station_m"] - runs[-1][-1]["station_m"] > max_gap_m + 1e-9:
            runs.append([])
        runs[-1].append(row)
    return runs


def extract_polyline_interval(raw: Iterable[Iterable[float]], start_m: float, end_m: float, step_m: float = 0.25) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    points, tangents = resample_polyline(raw, step_m)
    stations = cumulative_distance(points)
    start_m = float(np.clip(start_m, 0.0, stations[-1]))
    end_m = float(np.clip(end_m, start_m, stations[-1]))
    selected = (stations >= start_m - 1e-9) & (stations <= end_m + 1e-9)
    result_points = points[selected]
    result_tangents = tangents[selected]
    result_stations = stations[selected]
    for value in (start_m, end_m):
        if not np.any(np.isclose(result_stations, value, atol=1e-8)):
            index = int(np.searchsorted(stations, value))
            index = min(max(index, 1), len(stations) - 1)
            ratio = (value - stations[index - 1]) / max(stations[index] - stations[index - 1], 1e-12)
            point = points[index - 1] + ratio * (points[index] - points[index - 1])
            tangent = tangents[index - 1]
            result_points = np.vstack((result_points, point))
            result_tangents = np.vstack((result_tangents, tangent))
            result_stations = np.append(result_stations, value)
    order = np.argsort(result_stations)
    return result_points[order], result_tangents[order], result_stations[order]


def offset_interval(points: np.ndarray, tangents: np.ndarray, offsets: Iterable[float]) -> np.ndarray:
    offsets_array = np.asarray(list(offsets), dtype=np.float64)
    normal = np.column_stack((-tangents[:, 1], tangents[:, 0]))
    result = points.copy()
    result[:, :2] += offsets_array[:, None] * normal
    return result
