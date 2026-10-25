"""Build a partial, human-reviewed route curation candidate.

This stage deliberately emits only reviewed segments.  Planned decisions in the
configuration are documentation and cannot silently produce geometry.
"""

from __future__ import annotations

from collections import Counter
import json
from pathlib import Path

import cv2
import numpy as np
import yaml

from .current_builder import _write_csv, _write_json
from .geometry import cumulative_distance, points_xyz, resample_polyline
from .io_mgeo import load_json, resolve_source, sha256_file
from .map_overlay import dashed_polyline_segments
from .review_bundle import _load_legacy_references, load_review_layers
from .schema import SCHEMA_VERSION, make_candidate_feature, validate_canonical_document
from .spatial import GridIndex


CONFIG_SCHEMA = "asmc.hd_map.route_curation/v1"


def _true_runs(mask: np.ndarray) -> list[tuple[int, int]]:
    result: list[tuple[int, int]] = []
    start = None
    for index, value in enumerate(np.asarray(mask, dtype=bool)):
        if value and start is None:
            start = index
        if start is not None and (not value or index == len(mask) - 1):
            end = index if not value else index + 1
            result.append((start, end))
            start = None
    return result


def _route_owner(station: float, ranges: list[tuple[float, float, str]]) -> str | None:
    for index, (start, end, link_id) in enumerate(ranges):
        if start <= station < end or (index == len(ranges) - 1 and station <= end):
            return link_id
    return None


def _pattern(shape: str) -> str:
    return "broken" if "broken" in str(shape).lower() else "solid"


def _endpoint_route_station(
    points: np.ndarray, route_index: GridIndex, route_stations: np.ndarray,
) -> tuple[float, float]:
    """Return route stations of the first and last polyline point."""

    result = []
    for point in (points[0], points[-1]):
        _, _, match = route_index.query(point, None, max_distance_m=20.0, max_heading_deg=180.0)
        if match is None:
            raise ValueError("marking endpoint cannot be projected to the competition route")
        result.append(float(route_stations[match]))
    return result[0], result[1]


def _clip_polyline_by_arc(
    points: np.ndarray, start_m: float, end_m: float,
) -> np.ndarray:
    """Return an arc-length clip with exact interpolated endpoints."""

    source = np.asarray(points, dtype=np.float64)
    stations = cumulative_distance(source)
    total = float(stations[-1])
    start = float(start_m)
    end = float(end_m)
    if not 0.0 <= start < end <= total:
        raise ValueError(
            f"polyline arc clip [{start:.3f}, {end:.3f}] is outside [0, {total:.3f}]"
        )

    def interpolate(station: float) -> np.ndarray:
        return np.asarray([
            np.interp(station, stations, source[:, dimension])
            for dimension in range(source.shape[1])
        ], dtype=np.float64)

    interior = source[(stations > start) & (stations < end)]
    return np.vstack((interpolate(start), interior, interpolate(end)))


def _smooth_polyline_handoff(
    from_points: np.ndarray,
    to_points: np.ndarray,
    from_trim_m: float,
    to_trim_m: float,
    output_step_m: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, dict]:
    """Replace a lateral endpoint step with a tangent-continuous cubic handoff."""

    source_from = np.asarray(from_points, dtype=np.float64)
    source_to = np.asarray(to_points, dtype=np.float64)
    from_length = float(cumulative_distance(source_from)[-1])
    to_length = float(cumulative_distance(source_to)[-1])
    if not 0.0 < from_trim_m < from_length:
        raise ValueError("smooth handoff from-trim is outside its source")
    if not 0.0 < to_trim_m < to_length:
        raise ValueError("smooth handoff to-trim is outside its source")
    clipped_from = _clip_polyline_by_arc(
        source_from, 0.0, from_length - float(from_trim_m),
    )
    clipped_to = _clip_polyline_by_arc(
        source_to, float(to_trim_m), to_length,
    )
    start = clipped_from[-1].copy()
    end = clipped_to[0].copy()
    start_tangent = clipped_from[-1, :2] - clipped_from[-2, :2]
    end_tangent = clipped_to[1, :2] - clipped_to[0, :2]
    start_tangent /= max(float(np.linalg.norm(start_tangent)), 1e-9)
    end_tangent /= max(float(np.linalg.norm(end_tangent)), 1e-9)
    transition_span = float(np.linalg.norm(end[:2] - start[:2]))
    handle_m = max(transition_span / 3.0, output_step_m)
    control_start = start[:2] + handle_m * start_tangent
    control_end = end[:2] - handle_m * end_tangent
    sample_count = max(3, int(np.ceil(transition_span / output_step_m)) + 1)
    ratios = np.linspace(0.0, 1.0, sample_count)
    connector = np.empty((sample_count, source_from.shape[1]), dtype=np.float64)
    for index, ratio in enumerate(ratios):
        inverse = 1.0 - ratio
        connector[index, :2] = (
            inverse ** 3 * start[:2]
            + 3.0 * inverse ** 2 * ratio * control_start
            + 3.0 * inverse * ratio ** 2 * control_end
            + ratio ** 3 * end[:2]
        )
        if connector.shape[1] > 2:
            connector[index, 2:] = (
                inverse * start[2:] + ratio * end[2:]
            )
    first_direction = connector[1, :2] - connector[0, :2]
    last_direction = connector[-1, :2] - connector[-2, :2]

    def angle_degrees(left: np.ndarray, right: np.ndarray) -> float:
        denominator = max(
            float(np.linalg.norm(left) * np.linalg.norm(right)), 1e-9,
        )
        cosine = float(np.clip(np.dot(left, right) / denominator, -1.0, 1.0))
        return float(np.degrees(np.arccos(cosine)))

    return clipped_from, clipped_to, connector, {
        "method": "tangent_continuous_cubic_handoff",
        "from_trim_m": float(from_trim_m),
        "to_trim_m": float(to_trim_m),
        "transition_span_m": transition_span,
        "start_tangent_error_deg": angle_degrees(start_tangent, first_direction),
        "end_tangent_error_deg": angle_degrees(last_direction, end_tangent),
        "output_point_count": int(len(connector)),
    }


def _extend_route_end(
    points: np.ndarray,
    anchor_route_s_m: float,
    route_samples: np.ndarray,
    route_tangents: np.ndarray,
    route_stations: np.ndarray,
    route_index: GridIndex,
    maximum_extension_m: float,
) -> tuple[np.ndarray, float]:
    """Extend a boundary to an anchor route cross-section without changing its lateral offset."""

    output = np.asarray(points, dtype=np.float64).copy()
    first_s, last_s = _endpoint_route_station(output, route_index, route_stations)
    endpoint_index = 0 if first_s > last_s else len(output) - 1
    endpoint_s = max(first_s, last_s)
    if anchor_route_s_m <= endpoint_s + 1e-6:
        raise ValueError("endpoint alignment must extend, not shorten, the selected marking")
    route_index_value = int(np.argmin(np.abs(route_stations - anchor_route_s_m)))
    base = route_samples[route_index_value]
    anchor_tangent = route_tangents[route_index_value, :2]
    tangent_norm = float(np.linalg.norm(anchor_tangent))
    if tangent_norm <= 1e-9:
        raise ValueError("route tangent is degenerate at endpoint alignment anchor")
    anchor_tangent = anchor_tangent / tangent_norm
    anchor_normal = np.asarray([-anchor_tangent[1], anchor_tangent[0]], dtype=np.float64)

    endpoint = output[endpoint_index]
    _, _, endpoint_match = route_index.query(
        endpoint, None, max_distance_m=20.0, max_heading_deg=180.0,
    )
    if endpoint_match is None:
        raise ValueError("target marking endpoint cannot be projected to the competition route")
    endpoint_base = route_samples[endpoint_match]
    endpoint_tangent = route_tangents[endpoint_match, :2]
    endpoint_tangent = endpoint_tangent / max(float(np.linalg.norm(endpoint_tangent)), 1e-9)
    endpoint_normal = np.asarray([-endpoint_tangent[1], endpoint_tangent[0]], dtype=np.float64)
    lateral = float(np.dot(endpoint[:2] - endpoint_base[:2], endpoint_normal))
    target = np.zeros(output.shape[1], dtype=np.float64)
    target[:2] = base[:2] + lateral * anchor_normal
    if output.shape[1] > 2:
        target[2:] = endpoint[2:]
    extension = float(np.linalg.norm(target[:2] - endpoint[:2]))
    if extension > maximum_extension_m:
        raise ValueError(
            f"endpoint alignment extension {extension:.3f}m exceeds {maximum_extension_m:.3f}m"
        )
    if endpoint_index == 0:
        output = np.vstack((target, output))
    else:
        output = np.vstack((output, target))
    return output, extension


def _extend_route_start(
    points: np.ndarray,
    anchor_route_s_m: float,
    route_samples: np.ndarray,
    route_tangents: np.ndarray,
    route_stations: np.ndarray,
    route_index: GridIndex,
    maximum_extension_m: float,
) -> tuple[np.ndarray, float]:
    """Extend a boundary back to a route cross-section at fixed lateral offset."""

    output = np.asarray(points, dtype=np.float64).copy()
    first_s, last_s = _endpoint_route_station(output, route_index, route_stations)
    endpoint_index = 0 if first_s < last_s else len(output) - 1
    endpoint_s = min(first_s, last_s)
    if anchor_route_s_m >= endpoint_s - 1e-6:
        if abs(anchor_route_s_m - endpoint_s) <= 1e-6:
            return output, 0.0
        raise ValueError("start alignment must extend backward, not shorten, the marking")
    route_index_value = int(np.argmin(np.abs(route_stations - anchor_route_s_m)))
    base = route_samples[route_index_value]
    anchor_tangent = route_tangents[route_index_value, :2]
    anchor_tangent /= max(float(np.linalg.norm(anchor_tangent)), 1e-9)
    anchor_normal = np.asarray([-anchor_tangent[1], anchor_tangent[0]])

    endpoint = output[endpoint_index]
    _, _, endpoint_match = route_index.query(
        endpoint, None, max_distance_m=20.0, max_heading_deg=180.0,
    )
    if endpoint_match is None:
        raise ValueError("marking start cannot be projected to the competition route")
    endpoint_tangent = route_tangents[endpoint_match, :2]
    endpoint_tangent /= max(float(np.linalg.norm(endpoint_tangent)), 1e-9)
    endpoint_normal = np.asarray([-endpoint_tangent[1], endpoint_tangent[0]])
    lateral = float(np.dot(
        endpoint[:2] - route_samples[endpoint_match, :2], endpoint_normal,
    ))
    target = np.zeros(output.shape[1], dtype=np.float64)
    target[:2] = base[:2] + lateral * anchor_normal
    if output.shape[1] > 2:
        target[2:] = endpoint[2:]
    extension = float(np.linalg.norm(target[:2] - endpoint[:2]))
    if extension > maximum_extension_m:
        raise ValueError(
            f"start alignment extension {extension:.3f}m exceeds "
            f"{maximum_extension_m:.3f}m"
        )
    if endpoint_index == 0:
        output = np.vstack((target, output))
    else:
        output = np.vstack((output, target))
    return output, extension


def _route_normal_constant_offset(
    points: np.ndarray,
    route_samples: np.ndarray,
    route_tangents: np.ndarray,
    route_stations: np.ndarray,
    route_index: GridIndex,
    maximum_correction_m: float,
    route_s_min_m: float | None = None,
    route_s_max_m: float | None = None,
    target_route_normal_offset_m: float | None = None,
) -> tuple[np.ndarray, dict]:
    """Straighten a short observed marking at its robust route-normal offset.

    This is deliberately an explicit, gated correction rather than a generic
    smoother.  The observed station extent is retained, while the small
    lateral wiggle is replaced by one median offset from the current route.
    """

    raw = np.asarray(points, dtype=np.float64)
    projected = []
    for point in raw:
        _, _, match = route_index.query(
            point, None, max_distance_m=20.0, max_heading_deg=180.0,
        )
        if match is None:
            raise ValueError("route-normal smoothing point cannot be projected")
        tangent = route_tangents[match, :2]
        norm = float(np.linalg.norm(tangent))
        if norm <= 1e-9:
            raise ValueError("route-normal smoothing encountered a degenerate tangent")
        tangent = tangent / norm
        normal = np.asarray([-tangent[1], tangent[0]], dtype=np.float64)
        lateral = float(np.dot(point[:2] - route_samples[match, :2], normal))
        station = float(route_stations[match])
        if route_s_min_m is not None and station < route_s_min_m:
            continue
        if route_s_max_m is not None and station >= route_s_max_m:
            continue
        projected.append((station, int(match), lateral, point))

    if len(projected) < 2:
        raise ValueError("route-normal smoothing has insufficient points in its route interval")

    observed_median_lateral = float(np.median([row[2] for row in projected]))
    target_lateral = (
        observed_median_lateral
        if target_route_normal_offset_m is None else float(target_route_normal_offset_m)
    )
    # Geometry direction does not encode traffic direction.  Sorting and
    # de-duplicating by route station prevents a noisy observation from
    # briefly folding back on itself in the canonical map.
    ordered = sorted(projected, key=lambda row: row[0])
    unique = []
    seen_matches = set()
    for row in ordered:
        if row[1] in seen_matches:
            continue
        seen_matches.add(row[1])
        unique.append(row)
    if len(unique) < 2:
        raise ValueError("route-normal smoothing requires two distinct route stations")

    output = []
    corrections = []
    for _, match, _, source_point in unique:
        tangent = route_tangents[match, :2]
        tangent = tangent / max(float(np.linalg.norm(tangent)), 1e-9)
        normal = np.asarray([-tangent[1], tangent[0]], dtype=np.float64)
        target = np.asarray(source_point, dtype=np.float64).copy()
        target[:2] = route_samples[match, :2] + target_lateral * normal
        corrections.append(float(np.linalg.norm(target[:2] - source_point[:2])))
        output.append(target)
    maximum_correction = max(corrections)
    if maximum_correction > maximum_correction_m:
        raise ValueError(
            "route-normal smoothing correction "
            f"{maximum_correction:.3f}m exceeds {maximum_correction_m:.3f}m"
        )
    return np.asarray(output), {
        "method": "constant_median_route_normal_offset",
        "observed_median_route_normal_offset_m": observed_median_lateral,
        "route_normal_offset_m": target_lateral,
        "correction_p95_m": float(np.quantile(corrections, 0.95)),
        "maximum_correction_m": maximum_correction,
        "source_point_count": len(projected),
        "output_point_count": len(output),
    }


def _arc_length_moving_average(
    points: np.ndarray,
    sample_step_m: float,
    smoothing_length_m: float,
    maximum_correction_m: float,
) -> tuple[np.ndarray, dict]:
    """Remove small polyline jitter without replacing the observed curve."""

    sampled, _ = resample_polyline(np.asarray(points, dtype=np.float64), sample_step_m)
    stations = cumulative_distance(sampled)
    half_window = max(0.5 * smoothing_length_m, sample_step_m)
    output = sampled.copy()
    for index, station in enumerate(stations):
        distance = np.abs(stations - station)
        selected = distance <= half_window
        weights = np.maximum(0.0, 1.0 - distance[selected] / (half_window + 1e-9))
        output[index] = np.average(sampled[selected], axis=0, weights=weights)
    # The observation endpoints are authoritative anchors.
    output[0], output[-1] = sampled[0], sampled[-1]
    corrections = np.linalg.norm(output[:, :2] - sampled[:, :2], axis=1)
    maximum_correction = float(np.max(corrections))
    if maximum_correction > maximum_correction_m:
        raise ValueError(
            "arc-length polyline smoothing correction "
            f"{maximum_correction:.3f}m exceeds {maximum_correction_m:.3f}m"
        )
    return output, {
        "method": "arc_length_triangular_moving_average",
        "smoothing_length_m": float(smoothing_length_m),
        "source_point_count": len(points),
        "resampled_point_count": len(sampled),
        "correction_p95_m": float(np.quantile(corrections, 0.95)),
        "maximum_correction_m": maximum_correction,
        "endpoint_observation_anchors_preserved": True,
    }


def _route_normal_robust_profile(
    source_vectors: list[np.ndarray],
    route_samples: np.ndarray,
    route_tangents: np.ndarray,
    route_stations: np.ndarray,
    route_owners: list[str],
    route_index: GridIndex,
    allowed_owners: set[str],
    route_s_min_m: float,
    route_s_max_m: float,
    sample_step_m: float,
    station_bin_m: float,
    smoothing_length_m: float,
    maximum_source_distance_m: float,
    maximum_heading_difference_deg: float,
    maximum_profile_gap_m: float,
    maximum_smoothing_correction_m: float,
) -> tuple[np.ndarray, dict]:
    """Fit a smooth route-normal offset profile to direct H5 observations.

    Repeated LiDAR traces are reduced to robust station-bin medians before
    smoothing.  The first and last accepted bin remain fixed, so smoothing
    cannot move the reviewed observation anchors.  Gaps larger than the
    explicit profile gate are never interpolated.
    """

    observations: list[tuple[float, float, float]] = []
    for vector in source_vectors:
        sampled, tangents = resample_polyline(vector, sample_step_m)
        for point, tangent in zip(sampled, tangents):
            _, _, match = route_index.query(
                point, tangent,
                max_distance_m=maximum_source_distance_m,
                max_heading_deg=maximum_heading_difference_deg,
            )
            if match is None:
                continue
            station = float(route_stations[match])
            owner = str(route_owners[match])
            if not (route_s_min_m <= station < route_s_max_m) or owner not in allowed_owners:
                continue
            route_tangent = route_tangents[match, :2].copy()
            route_tangent /= max(float(np.linalg.norm(route_tangent)), 1e-9)
            normal = np.asarray([-route_tangent[1], route_tangent[0]])
            offset = float(np.dot(point[:2] - route_samples[match, :2], normal))
            z = float(point[2]) if len(point) > 2 else float(route_samples[match, 2])
            observations.append((station, offset, z))
    if len(observations) < 2:
        raise ValueError("route-normal profile has insufficient direct observations")

    bins: dict[int, list[tuple[float, float, float]]] = {}
    for row in observations:
        bin_index = int(np.floor((row[0] - route_s_min_m) / station_bin_m))
        bins.setdefault(bin_index, []).append(row)
    profile_rows = []
    for values in bins.values():
        value = np.asarray(values, dtype=np.float64)
        profile_rows.append((
            float(np.median(value[:, 0])),
            float(np.median(value[:, 1])),
            float(np.median(value[:, 2])),
            len(value),
            float(np.quantile(value[:, 1], 0.95) - np.quantile(value[:, 1], 0.05)),
        ))
    profile_rows.sort(key=lambda row: row[0])

    profile_runs: list[list[tuple[float, float, float, int, float]]] = []
    for row in profile_rows:
        if not profile_runs or row[0] - profile_runs[-1][-1][0] > maximum_profile_gap_m:
            profile_runs.append([row])
        else:
            profile_runs[-1].append(row)
    profile_runs = [run for run in profile_runs if len(run) >= 2]
    if not profile_runs:
        raise ValueError("route-normal profile has no continuous two-bin run")
    selected_run = max(profile_runs, key=lambda run: run[-1][0] - run[0][0])

    profile_stations = np.asarray([row[0] for row in selected_run], dtype=np.float64)
    raw_offsets = np.asarray([row[1] for row in selected_run], dtype=np.float64)
    profile_z = np.asarray([row[2] for row in selected_run], dtype=np.float64)
    smoothed_offsets = raw_offsets.copy()
    half_window = max(0.5 * smoothing_length_m, station_bin_m)
    for index, station in enumerate(profile_stations):
        distance = np.abs(profile_stations - station)
        selected = distance <= half_window
        weights = np.maximum(0.0, 1.0 - distance[selected] / (half_window + 1e-9))
        smoothed_offsets[index] = float(
            np.average(raw_offsets[selected], weights=weights)
        )
    # Endpoint observations are immutable anchors.
    smoothed_offsets[0], smoothed_offsets[-1] = raw_offsets[0], raw_offsets[-1]
    corrections = np.abs(smoothed_offsets - raw_offsets)
    maximum_correction = float(np.max(corrections))
    if maximum_correction > maximum_smoothing_correction_m:
        raise ValueError(
            "route-normal profile smoothing correction "
            f"{maximum_correction:.3f}m exceeds {maximum_smoothing_correction_m:.3f}m"
        )

    selected_route_indices = [
        index for index, (station, owner) in enumerate(zip(route_stations, route_owners))
        if profile_stations[0] <= station <= profile_stations[-1]
        and owner in allowed_owners
    ]
    if len(selected_route_indices) < 2:
        raise ValueError("route-normal profile has insufficient output route samples")
    output = []
    for index in selected_route_indices:
        station = float(route_stations[index])
        offset = float(np.interp(station, profile_stations, smoothed_offsets))
        tangent = route_tangents[index, :2].copy()
        tangent /= max(float(np.linalg.norm(tangent)), 1e-9)
        normal = np.asarray([-tangent[1], tangent[0]])
        target = route_samples[index].copy()
        target[:2] += offset * normal
        target[2] = float(np.interp(station, profile_stations, profile_z))
        output.append(target)

    return np.asarray(output), {
        "method": "robust_station_bin_route_normal_profile",
        "source_point_count": len(observations),
        "profile_bin_count": len(selected_run),
        "discarded_profile_run_count": len(profile_runs) - 1,
        "route_s_min_m": float(profile_stations[0]),
        "route_s_max_m": float(profile_stations[-1]),
        "route_normal_offset_p05_m": float(np.quantile(smoothed_offsets, 0.05)),
        "route_normal_offset_median_m": float(np.median(smoothed_offsets)),
        "route_normal_offset_p95_m": float(np.quantile(smoothed_offsets, 0.95)),
        "smoothing_correction_p95_m": float(np.quantile(corrections, 0.95)),
        "maximum_smoothing_correction_m": maximum_correction,
        "maximum_profile_bin_spread_m": max(row[4] for row in selected_run),
        "maximum_profile_gap_m": float(np.max(np.diff(profile_stations))),
        "start_anchor_offset_m": float(raw_offsets[0]),
        "end_anchor_offset_m": float(raw_offsets[-1]),
        "output_point_count": len(output),
    }


def _route_normal_profile_from_points(
    points: np.ndarray,
    route_samples: np.ndarray,
    route_tangents: np.ndarray,
    route_stations: np.ndarray,
    route_index: GridIndex,
    sample_step_m: float,
    maximum_route_distance_m: float,
    maximum_heading_difference_deg: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Project a reviewed boundary to a monotonic route-normal profile."""

    sampled, tangents = resample_polyline(
        np.asarray(points, dtype=np.float64), sample_step_m,
    )
    observations: dict[int, list[tuple[float, float]]] = {}
    for point, tangent in zip(sampled, tangents):
        _, _, match = route_index.query(
            point, tangent,
            max_distance_m=maximum_route_distance_m,
            max_heading_deg=maximum_heading_difference_deg,
        )
        if match is None:
            continue
        route_tangent = route_tangents[match, :2].copy()
        route_tangent /= max(float(np.linalg.norm(route_tangent)), 1e-9)
        normal = np.asarray([-route_tangent[1], route_tangent[0]])
        offset = float(np.dot(point[:2] - route_samples[match, :2], normal))
        z = float(point[2]) if len(point) > 2 else float(route_samples[match, 2])
        observations.setdefault(int(match), []).append((offset, z))
    if len(observations) < 2:
        raise ValueError("reviewed boundary has insufficient route-normal samples")
    matches = np.asarray(sorted(observations), dtype=np.int64)
    stations = route_stations[matches]
    offsets = np.asarray([
        np.median([row[0] for row in observations[int(match)]])
        for match in matches
    ], dtype=np.float64)
    z_values = np.asarray([
        np.median([row[1] for row in observations[int(match)]])
        for match in matches
    ], dtype=np.float64)
    return stations, offsets, z_values


def _parallel_route_normal_profile(
    reference_points: np.ndarray,
    anchor_points: np.ndarray,
    route_samples: np.ndarray,
    route_tangents: np.ndarray,
    route_stations: np.ndarray,
    route_owners: list[str],
    route_index: GridIndex,
    allowed_owners: set[str],
    route_s_min_m: float,
    route_s_max_m: float,
    sample_step_m: float,
    maximum_route_distance_m: float,
    maximum_heading_difference_deg: float,
    maximum_anchor_extrapolation_m: float,
    lateral_separation_min_m: float,
    lateral_separation_max_m: float,
) -> tuple[np.ndarray, dict]:
    """Continue an anchor parallel to a reviewed opposite boundary.

    The reference boundary keeps its route-normal variation and curvature. A
    single separation measured at the reviewed start cross-section is added,
    so a noisy tail in the anchor source cannot bend the continuation.
    """

    reference_s, reference_offset, reference_z = _route_normal_profile_from_points(
        reference_points, route_samples, route_tangents, route_stations,
        route_index, sample_step_m, maximum_route_distance_m,
        maximum_heading_difference_deg,
    )
    anchor_s, anchor_offset, _ = _route_normal_profile_from_points(
        anchor_points, route_samples, route_tangents, route_stations,
        route_index, sample_step_m, maximum_route_distance_m,
        maximum_heading_difference_deg,
    )
    if route_s_min_m >= route_s_max_m:
        raise ValueError("parallel route-normal profile has an empty interval")
    reference_gap = max(
        float(reference_s[0] - route_s_min_m),
        float(route_s_max_m - reference_s[-1]),
        0.0,
    )
    if reference_gap > sample_step_m + 1e-6:
        raise ValueError(
            "parallel route-normal reference does not cover the output interval"
        )
    anchor_extrapolation = max(
        float(anchor_s[0] - route_s_min_m),
        float(route_s_min_m - anchor_s[-1]),
        0.0,
    )
    if anchor_extrapolation > maximum_anchor_extrapolation_m:
        raise ValueError(
            "parallel route-normal anchor extrapolation "
            f"{anchor_extrapolation:.3f}m exceeds "
            f"{maximum_anchor_extrapolation_m:.3f}m"
        )
    reference_start_offset = float(np.interp(
        route_s_min_m, reference_s, reference_offset,
    ))
    anchor_start_offset = float(np.interp(
        route_s_min_m, anchor_s, anchor_offset,
    ))
    lateral_separation = anchor_start_offset - reference_start_offset
    if not lateral_separation_min_m <= lateral_separation <= lateral_separation_max_m:
        raise ValueError(
            "parallel route-normal separation "
            f"{lateral_separation:.3f}m is outside "
            f"[{lateral_separation_min_m:.3f}, {lateral_separation_max_m:.3f}]m"
        )

    interior = [
        float(station)
        for station, owner in zip(route_stations, route_owners)
        if route_s_min_m < station < route_s_max_m and owner in allowed_owners
    ]
    output_stations = np.asarray(
        [route_s_min_m, *interior, route_s_max_m], dtype=np.float64,
    )
    if len(output_stations) < 2:
        raise ValueError("parallel route-normal profile has insufficient output samples")
    output = []
    for station in output_stations:
        base = np.asarray([
            np.interp(station, route_stations, route_samples[:, dimension])
            for dimension in range(route_samples.shape[1])
        ], dtype=np.float64)
        tangent = np.asarray([
            np.interp(station, route_stations, route_tangents[:, dimension])
            for dimension in range(2)
        ], dtype=np.float64)
        tangent /= max(float(np.linalg.norm(tangent)), 1e-9)
        normal = np.asarray([-tangent[1], tangent[0]])
        target = base.copy()
        target[:2] += (
            float(np.interp(station, reference_s, reference_offset))
            + lateral_separation
        ) * normal
        if len(target) > 2:
            target[2] = float(np.interp(station, reference_s, reference_z))
        output.append(target)
    return np.asarray(output), {
        "method": "anchored_parallel_route_normal_profile",
        "route_s_min_m": float(route_s_min_m),
        "route_s_max_m": float(route_s_max_m),
        "reference_route_s_min_m": float(reference_s[0]),
        "reference_route_s_max_m": float(reference_s[-1]),
        "anchor_route_s_min_m": float(anchor_s[0]),
        "anchor_route_s_max_m": float(anchor_s[-1]),
        "reference_start_offset_m": reference_start_offset,
        "anchor_start_offset_m": anchor_start_offset,
        "lateral_separation_m": lateral_separation,
        "anchor_extrapolation_m": anchor_extrapolation,
        "output_point_count": len(output),
    }


def _load_won_tae_h5(path: Path) -> list[dict]:
    """Load the immutable line vectors stored by Won-tae's original bake."""

    try:
        import h5py
    except ImportError as error:
        raise RuntimeError("Won-tae H5 route curation requires python3-h5py") from error
    vectors = []
    with h5py.File(str(path), "r") as handle:
        required = {"observed_lane_points", "observed_lane_offsets"}
        if not required.issubset(handle.keys()):
            raise ValueError("Won-tae H5 is missing observed lane vectors")
        points = np.asarray(handle["observed_lane_points"], dtype=np.float64)
        offsets = np.asarray(handle["observed_lane_offsets"], dtype=np.int64)
        colors = json.loads(handle.attrs.get("observed_lane_colors", "[]"))
        sides = json.loads(handle.attrs.get("observed_lane_sides", "[]"))
        sources = json.loads(handle.attrs.get("observed_lane_sources", "[]"))
        for index, (start, end) in enumerate(zip(offsets[:-1], offsets[1:])):
            vectors.append({
                "index": index,
                "points": points[int(start):int(end)],
                "color": str(colors[index]).lower() if index < len(colors) else "unknown",
                # This is collection-vehicle side only.  It is deliberately not
                # copied to canonical left/right boundary ownership.
                "source_side": str(sides[index]) if index < len(sides) else "UNKNOWN",
                "source_file": Path(str(sources[index])).name if index < len(sources) else "unknown",
            })
    return vectors


def _load_won_tae_h5_stoplines(path: Path) -> list[dict]:
    """Load Won-tae's stored stopline vectors with their reviewed node IDs."""

    try:
        import h5py
    except ImportError as error:
        raise RuntimeError("Won-tae H5 route curation requires python3-h5py") from error
    with h5py.File(str(path), "r") as handle:
        required = {"stopline_points", "stopline_dirs", "stopline_lengths"}
        if not required.issubset(handle.keys()):
            raise ValueError("Won-tae H5 is missing stopline vectors")
        centers = np.asarray(handle["stopline_points"], dtype=np.float64)
        directions = np.asarray(handle["stopline_dirs"], dtype=np.float64)
        lengths = np.asarray(handle["stopline_lengths"], dtype=np.float64)
        source_ids = json.loads(handle.attrs.get("stopline_ids", "[]"))
        traffic_lights = json.loads(handle.attrs.get("stopline_traffic_lights", "[]"))
    result = []
    for index, (center, direction, length) in enumerate(zip(centers, directions, lengths)):
        norm = float(np.linalg.norm(direction))
        if norm <= 1e-9:
            raise ValueError(f"Won-tae H5 stopline {index} has a degenerate direction")
        unit = direction / norm
        half = 0.5 * float(length) * unit
        result.append({
            "index": index,
            "source_id": str(source_ids[index]) if index < len(source_ids) else str(index),
            "traffic_light_id": (
                str(traffic_lights[index]) if index < len(traffic_lights) else ""
            ),
            "center": center,
            "points": np.stack((center - half, center + half)),
            "length_m": float(length),
        })
    return result


def _join_h5_vectors(vectors: list[np.ndarray], maximum_gap_m: float | None) -> tuple[np.ndarray, list[float]]:
    """Orient and concatenate selected H5 vectors, auditing any inferred gap."""

    if not vectors:
        raise ValueError("at least one H5 vector is required")
    output = np.asarray(vectors[0], dtype=np.float64).copy()
    gaps: list[float] = []
    for raw in vectors[1:]:
        candidate = np.asarray(raw, dtype=np.float64)
        choices = []
        for existing in (output, output[::-1]):
            for oriented in (candidate, candidate[::-1]):
                choices.append((
                    float(np.linalg.norm(existing[-1, :2] - oriented[0, :2])),
                    existing, oriented,
                ))
        gap, output, oriented = min(choices, key=lambda row: row[0])
        if maximum_gap_m is None:
            raise ValueError("multiple H5 vectors require an explicit maximum gap")
        if gap > maximum_gap_m:
            raise ValueError(f"H5 vector gap {gap:.3f}m exceeds {maximum_gap_m:.3f}m")
        gaps.append(gap)
        output = np.vstack((output, oriented))
    return output, gaps


def _render_overlay(
    path: Path, route: np.ndarray, route_stations: np.ndarray,
    interval: dict, features: list[dict], visual: dict,
) -> None:
    start_s = float(interval["route_s_min_m"])
    end_s = float(interval["route_s_max_m"])
    route_part = route[(route_stations >= start_s) & (route_stations <= end_s)]
    points = [route_part]
    points.extend(points_xyz(feature["geometry"]["coordinates"]) for feature in features)
    finite = [value for value in points if len(value)]
    low = np.min(np.vstack(finite)[:, :2], axis=0)
    high = np.max(np.vstack(finite)[:, :2], axis=0)
    margin = float(visual["corridor_margin_m"])
    low, high = low - margin, high + margin
    width, height = int(visual["width_px"]), int(visual["height_px"])
    padding = int(visual["padding_px"])
    span = np.maximum(high - low, 1.0)
    scale = min((width - 2 * padding) / span[0], (height - 2 * padding) / span[1])
    image = np.full((height, width, 3), 18, dtype=np.uint8)

    def pixel(value: np.ndarray) -> np.ndarray:
        result = np.empty((len(value), 2), dtype=np.int32)
        result[:, 0] = np.round(padding + (value[:, 0] - low[0]) * scale).astype(np.int32)
        result[:, 1] = np.round(height - padding - (value[:, 1] - low[1]) * scale).astype(np.int32)
        return result

    cv2.polylines(image, [pixel(route_part)], False, (40, 190, 50), 2, cv2.LINE_AA)
    palette = {"white": (245, 245, 245), "yellow": (0, 220, 255), "blue": (240, 120, 30)}
    for feature in features:
        value = points_xyz(feature["geometry"]["coordinates"])
        color = (
            (20, 40, 255) if feature["feature_type"] == "stopline"
            else palette.get(feature["attributes"]["color"], (220, 80, 220))
        )
        if feature["attributes"]["pattern"] == "broken":
            segments = dashed_polyline_segments(
                [value],
                float(feature["attributes"].get("dash_length_m", 3.0)),
                float(feature["attributes"].get("dash_gap_m", 3.0)),
                float(feature["attributes"].get("dash_phase_m", 0.0)),
            )
            for segment in segments:
                pixels = pixel(segment)
                cv2.line(image, tuple(pixels[0]), tuple(pixels[1]), color, 3, cv2.LINE_AA)
        else:
            cv2.polylines(image, [pixel(value)], False, color, 3, cv2.LINE_AA)
    primary = str((interval.get("source_policy") or {}).get("primary", "unknown"))
    legend = [
        str(interval["id"]).upper(),
        f"green=official route  white/yellow=selected {primary} paint",
        "explicit endpoint/pattern/gap corrections are recorded in feature validation",
        f"PARTIAL CANDIDATE / stoplines={sum(row['feature_type'] == 'stopline' for row in features)} / operational=false",
    ]
    for index, text in enumerate(legend):
        cv2.putText(
            image, text, (16, 24 + 21 * index), cv2.FONT_HERSHEY_SIMPLEX,
            0.48, (235, 235, 235), 1, cv2.LINE_AA,
        )
    path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(path), image):
        raise IOError(f"failed to write overlay: {path}")


def build_route_curation(config_path: Path, repo_root: Path, output_dir: Path) -> dict:
    with config_path.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream) or {}
    if config.get("schema_version") != CONFIG_SCHEMA:
        raise ValueError("unsupported route curation config schema")
    policy = config.get("promotion_policy") or {}
    if policy.get("operational") is not False or policy.get("partial_candidate") is not True:
        raise ValueError("route curation stage must remain a non-operational partial candidate")

    comparison_path = resolve_source(repo_root, config["sources"]["comparison_config"])
    review = load_review_layers(comparison_path, repo_root)
    implemented = [
        segment for segment in config["segments"]
        if segment.get("status") == "implemented_candidate"
    ]
    unsupported = {
        (segment.get("source_policy") or {}).get("primary") for segment in implemented
    } - {
        "legacy_origin_aligned", "won_tae_h5", "current_mgeo_route_guide", "preplan",
    }
    if unsupported:
        raise ValueError(f"implemented segment uses unsupported source: {sorted(unsupported)}")
    if not implemented:
        raise ValueError("route curation has no implemented segment")

    route = points_xyz(review["route"])
    step = float(config["geometry"]["sample_step_m"])
    route_samples, route_tangents = resample_polyline(route, step)
    route_stations = cumulative_distance(route_samples)
    route_ranges = sorted([
        (
            float(feature["attributes"]["route_s_min_m"]),
            float(feature["attributes"]["route_s_max_m"]),
            str(feature["topology"]["current_link_ids"][0]),
        )
        for feature in review["current_links"]
        if "official_route" in feature.get("attributes", {}).get("corridor_roles", [])
    ])
    route_owners = [_route_owner(float(station), route_ranges) or "" for station in route_stations]
    route_index = GridIndex(route_samples, route_tangents, route_owners, cell_size_m=3.0)
    overrides = {
        (str(row["segment_id"]), str(row["legacy_source_id"]), str(row["current_link_id"])): row
        for row in config.get("marking_overrides", [])
    }
    curation_legacy_references = list(review["legacy_references"])
    explicit_legacy_ids = {
        str(marking["source_id"])
        for segment in implemented for marking in segment.get("legacy_markings", [])
    }
    available_legacy_ids = {
        str(reference["id"]) for reference in curation_legacy_references
    }
    missing_explicit_legacy_ids = explicit_legacy_ids - available_legacy_ids
    if missing_explicit_legacy_ids:
        all_legacy_references = {
            str(reference["id"]): reference
            for reference in _load_legacy_references(review["config"], repo_root)
        }
        unresolved = missing_explicit_legacy_ids - set(all_legacy_references)
        if unresolved:
            raise ValueError(f"explicit legacy sources are missing: {sorted(unresolved)}")
        curation_legacy_references.extend(
            all_legacy_references[source_id]
            for source_id in sorted(missing_explicit_legacy_ids)
        )
    legacy_references = {
        str(reference["id"]): reference for reference in curation_legacy_references
    }
    legacy_endpoint_stations = {}
    alignment_source_ids = {
        str(row["align_route_end_to_legacy_source_id"])
        for row in config.get("marking_overrides", [])
        if "align_route_end_to_legacy_source_id" in row
    }
    for source_id in sorted(alignment_source_ids):
        if source_id not in legacy_references:
            raise ValueError(f"endpoint alignment source is missing: {source_id}")
        reference = legacy_references[source_id]
        value = np.asarray(reference["coordinates"], dtype=np.float64)
        legacy_endpoint_stations[source_id] = max(
            _endpoint_route_station(value, route_index, route_stations)
        )
    minimum_length = float(config["geometry"]["minimum_segment_length_m"])
    output_stride = max(1, int(round(float(config["geometry"]["output_step_m"]) / step)))
    legacy_sha = str(config.get("comparison", {}).get("legacy_lane_set_sha256", ""))
    if not legacy_sha:
        legacy_sha = next(
            (feature.get("provenance", {}).get("source_sha256", "") for feature in review["adopted_legacy_boundaries"]),
            "",
        )
    # Raw comparison references do not carry canonical provenance, so use the
    # pinned source hash from the comparison config when available.
    with comparison_path.open("r", encoding="utf-8") as stream:
        comparison_config = yaml.safe_load(stream) or {}
    legacy_sha = str((comparison_config.get("comparison") or {}).get("legacy_lane_set_sha256", legacy_sha))
    if not legacy_sha:
        raise ValueError("legacy lane source hash is missing")

    features: list[dict] = []
    inventory: list[dict] = []
    stopline_inventory: list[dict] = []
    for segment in implemented:
        primary_source = (segment.get("source_policy") or {}).get("primary")
        explicit_legacy = {
            str(row["source_id"]): row for row in segment.get("legacy_markings", [])
        }
        if primary_source != "legacy_origin_aligned" and not explicit_legacy:
            continue
        segment_links = {str(value) for value in segment["current_link_ids"]}
        start_s, end_s = float(segment["route_s_min_m"]), float(segment["route_s_max_m"])
        emitted_explicit_legacy = Counter()
        excluded_legacy_ids = {
            str(value) for value in segment.get("excluded_legacy_source_ids", [])
        }
        for reference in curation_legacy_references:
            reference_id = str(reference["id"])
            if reference_id in excluded_legacy_ids:
                continue
            selection = explicit_legacy.get(reference_id)
            if primary_source != "legacy_origin_aligned" and selection is None:
                continue
            if not segment_links.intersection(reference["target_current_link_ids"]):
                continue
            if selection is not None:
                expected_color = str(selection.get("color", reference["color"]))
                if str(reference["color"]) != expected_color:
                    raise ValueError(
                        f"legacy marking {reference_id} color changed: {reference['color']}"
                    )
            sampled, tangents = resample_polyline(reference["coordinates"], step)
            selected = np.zeros(len(sampled), dtype=bool)
            owners: list[str | None] = [None] * len(sampled)
            stations = np.full(len(sampled), np.nan, dtype=np.float64)
            marking_start_s = max(
                start_s,
                float(selection.get("route_s_min_m", start_s)) if selection else start_s,
            )
            marking_end_s = min(
                end_s,
                float(selection.get("route_s_max_m", end_s)) if selection else end_s,
            )
            if marking_start_s >= marking_end_s:
                raise ValueError(f"legacy marking {reference_id} has an empty route interval")
            for index, (point, tangent) in enumerate(zip(sampled, tangents)):
                _, _, match = route_index.query(
                    point, tangent,
                    max_distance_m=float(config["geometry"]["maximum_route_distance_m"]),
                    max_heading_deg=float(config["geometry"]["maximum_heading_difference_deg"]),
                )
                if match is None:
                    continue
                station = float(route_stations[match])
                owner = str(route_index.owners[match])
                inside = (
                    marking_start_s <= station < marking_end_s
                    if segment.get("end_exclusive")
                    else marking_start_s <= station <= marking_end_s
                )
                if inside and owner in segment_links:
                    selected[index], owners[index], stations[index] = True, owner, station
            for run_index, (run_start, run_end) in enumerate(_true_runs(selected)):
                raw_points = sampled[run_start:run_end]
                if len(raw_points) < 2 or cumulative_distance(raw_points)[-1] < minimum_length:
                    continue
                output_points = raw_points[::output_stride]
                if not np.array_equal(output_points[-1], raw_points[-1]):
                    output_points = np.vstack((output_points, raw_points[-1]))
                run_owners = sorted({value for value in owners[run_start:run_end] if value})
                if selection is not None:
                    run_owners = sorted(
                        set(run_owners)
                        | {
                            str(value) for value in selection.get(
                                "additional_target_current_link_ids", [],
                            )
                        }
                    )
                    if selection.get("target_current_link_ids"):
                        run_owners = sorted({
                            str(value) for value in selection["target_current_link_ids"]
                        })
                source_pattern = _pattern(reference["shape"])
                pattern = str(selection.get("pattern")) if selection else source_pattern
                applied = []
                if selection is not None and pattern != source_pattern:
                    if "attribute_override_id" not in selection:
                        raise ValueError(
                            f"legacy marking {reference_id} changes pattern without an override id"
                        )
                    applied.append(str(selection["attribute_override_id"]))
                for owner in run_owners:
                    override = overrides.get((str(segment["id"]), str(reference["id"]), owner))
                    if override:
                        if "set_pattern" in override:
                            pattern = str(override["set_pattern"])
                        applied.append(str(override["id"]))
                extension_m = 0.0
                alignment_source_id = None
                alignment_route_s_m = None
                for owner in run_owners:
                    override = overrides.get((str(segment["id"]), str(reference["id"]), owner))
                    if not override or "align_route_end_to_legacy_source_id" not in override:
                        continue
                    alignment_source_id = str(override["align_route_end_to_legacy_source_id"])
                    if alignment_source_id not in legacy_endpoint_stations:
                        raise ValueError(f"endpoint alignment source is missing: {alignment_source_id}")
                    alignment_route_s_m = legacy_endpoint_stations[alignment_source_id]
                    output_points, extension_m = _extend_route_end(
                        output_points,
                        alignment_route_s_m,
                        route_samples, route_tangents, route_stations, route_index,
                        float(override["maximum_extension_m"]),
                    )
                    break
                if selection is not None and "align_route_end_to_route_s_m" in selection:
                    if alignment_route_s_m is not None:
                        raise ValueError(
                            f"legacy marking {reference_id} has two endpoint alignments"
                        )
                    alignment_route_s_m = float(selection["align_route_end_to_route_s_m"])
                    output_points, extension_m = _extend_route_end(
                        output_points,
                        alignment_route_s_m,
                        route_samples, route_tangents, route_stations, route_index,
                        float(selection["maximum_extension_m"]),
                    )
                    applied.append(str(selection["endpoint_alignment_id"]))
                feature = make_candidate_feature(
                    feature_type="lane_marking",
                    canonical_id=(
                        f"{config['map_id']}:lane_marking:{segment['id']}:"
                        f"{reference['id']}:{run_index:02d}"
                    ),
                    points=output_points,
                    attributes={
                        "color": str(reference["color"]),
                        "pattern": pattern,
                        "line_group": "single",
                        "functional_role": "lane_boundary_paint",
                        "dash_length_m": float(reference["dash_length_m"]),
                        "dash_gap_m": float(reference["dash_gap_m"]),
                        "dash_phase_m": float(
                            selection.get("dash_phase_m", 0.0)
                            if selection is not None else 0.0
                        ),
                        "line_width_m": float(reference["line_width_m"]),
                        "curation_segment_id": str(segment["id"]),
                        "source_display": (
                            "legacy_origin_aligned_supplement"
                            if selection is not None else "legacy_origin_aligned"
                        ),
                        "selection_role": (
                            str(selection.get("functional_role", "lane_boundary_paint"))
                            if selection is not None else "primary"
                        ),
                    },
                    source_id=str(reference["id"]),
                    source_sha256=legacy_sha,
                    target_link_ids=run_owners,
                    validation={
                        "human_reviewed": True,
                        "evidence_ids": list(segment.get("evidence", [])),
                        "route_s_min_m": float(np.nanmin(stations[run_start:run_end])),
                        "route_s_max_m": max(
                            float(np.nanmax(stations[run_start:run_end])),
                            float(alignment_route_s_m) if alignment_route_s_m is not None else -np.inf,
                        ),
                        "legacy_audit_tier": str(reference["tier"]),
                        "attribute_override_ids": applied,
                        "endpoint_alignment_source_id": alignment_source_id,
                        "endpoint_alignment_route_s_m": alignment_route_s_m,
                        "endpoint_extension_m": extension_m,
                        "explicit_legacy_selection_id": (
                            str(selection["id"]) if selection is not None else None
                        ),
                    },
                )
                feature["status"] = "reviewed"
                features.append(feature)
                inventory.append({
                    "segment_id": segment["id"],
                    "source_id": reference["id"],
                    "run_index": run_index,
                    "color": reference["color"],
                    "source_pattern": source_pattern,
                    "output_pattern": pattern,
                    "current_link_ids": ";".join(run_owners),
                    "route_s_min_m": feature["validation"]["route_s_min_m"],
                    "route_s_max_m": feature["validation"]["route_s_max_m"],
                    "attribute_override_ids": ";".join(applied),
                    "source_display": (
                        "legacy_origin_aligned_supplement"
                        if selection is not None else "legacy_origin_aligned"
                    ),
                    "source_vector_indices": "",
                    "gap_bridge_lengths_m": "",
                })
                if selection is not None:
                    emitted_explicit_legacy[reference_id] += 1
        for reference_id in explicit_legacy:
            if emitted_explicit_legacy[reference_id] != 1:
                raise ValueError(
                    f"explicit legacy marking {reference_id} emitted "
                    f"{emitted_explicit_legacy[reference_id]} runs; expected exactly one"
                )

    h5_segments = [
        segment for segment in implemented
        if (
            (segment.get("source_policy") or {}).get("primary") == "won_tae_h5"
            or segment.get("h5_markings")
        )
    ]
    h5_sha = ""
    h5_vectors: list[dict] = []
    if h5_segments:
        h5_path = review["source_paths"].get("won_tae_h5")
        if h5_path is None:
            raise ValueError("comparison config does not define sources.won_tae_h5")
        h5_sha = sha256_file(h5_path)
        expected_h5_sha = str((comparison_config.get("comparison") or {}).get("won_tae_h5_sha256", ""))
        if h5_sha != expected_h5_sha:
            raise ValueError("Won-tae H5 hash differs from the pinned comparison source")
        h5_vectors = _load_won_tae_h5(h5_path)
        lateral_reference_indices = {
            int(marking["route_normal_smoothing"]["align_lateral_to_h5_vector_index"])
            for segment in h5_segments for marking in segment.get("h5_markings", [])
            if (marking.get("route_normal_smoothing") or {}).get(
                "align_lateral_to_h5_vector_index"
            ) is not None
        }
        h5_lateral_reference_offsets = {}
        for index in sorted(lateral_reference_indices):
            _, audit = _route_normal_constant_offset(
                h5_vectors[index]["points"],
                route_samples, route_tangents, route_stations, route_index,
                maximum_correction_m=20.0,
            )
            h5_lateral_reference_offsets[index] = float(audit["route_normal_offset_m"])
        alignment_vector_indices = {
            int(marking["align_route_end_to_h5_vector_index"])
            for segment in h5_segments for marking in segment.get("h5_markings", [])
            if "align_route_end_to_h5_vector_index" in marking
        }
        h5_endpoint_stations = {
            index: max(
                _endpoint_route_station(h5_vectors[index]["points"], route_index, route_stations)
            )
            for index in sorted(alignment_vector_indices)
        }
        for segment in h5_segments:
            segment_links = {str(value) for value in segment["current_link_ids"]}
            start_s, end_s = float(segment["route_s_min_m"]), float(segment["route_s_max_m"])
            for marking in segment.get("h5_markings", []):
                marking_start_s = max(start_s, float(marking.get("route_s_min_m", start_s)))
                marking_end_s = min(end_s, float(marking.get("route_s_max_m", end_s)))
                if marking_start_s >= marking_end_s:
                    raise ValueError(f"H5 marking {marking['id']} has an empty route interval")
                marking_minimum_length = float(
                    marking.get("minimum_segment_length_m", minimum_length)
                )
                vector_indices = [int(value) for value in marking["vector_indices"]]
                selected_vectors = [h5_vectors[index] for index in vector_indices]
                expected_file = str(marking["expected_source_file"])
                output_color = str(marking["color"]).lower()
                expected_source_color = str(
                    marking.get("expected_source_color", output_color)
                ).lower()
                for vector in selected_vectors:
                    if (
                        vector["source_file"] != expected_file
                        or vector["color"] != expected_source_color
                    ):
                        raise ValueError(
                            f"H5 vector {vector['index']} metadata changed: "
                            f"{vector['source_file']} {vector['color']}"
                        )
                joined, gap_lengths = _join_h5_vectors(
                    [vector["points"] for vector in selected_vectors],
                    float(marking["maximum_gap_m"]) if len(selected_vectors) > 1 else None,
                )
                smoothing_audit = None
                polyline_smoothing_audit = None
                if marking.get("polyline_smoothing"):
                    if marking.get("route_normal_smoothing"):
                        raise ValueError(
                            f"H5 marking {marking['id']} cannot use two smoothing methods"
                        )
                    smoothing = marking["polyline_smoothing"]
                    if smoothing.get("method") != "arc_length_moving_average":
                        raise ValueError(
                            f"unsupported H5 polyline smoothing method for {marking['id']}"
                        )
                    joined, polyline_smoothing_audit = _arc_length_moving_average(
                        joined, step,
                        float(smoothing["smoothing_length_m"]),
                        float(smoothing["maximum_correction_m"]),
                    )
                if marking.get("route_normal_smoothing"):
                    smoothing = marking["route_normal_smoothing"]
                    if smoothing.get("method") != "constant_median_offset":
                        raise ValueError(
                            f"unsupported H5 smoothing method for {marking['id']}"
                        )
                    lateral_reference_index = smoothing.get(
                        "align_lateral_to_h5_vector_index"
                    )
                    target_lateral = None
                    if lateral_reference_index is not None:
                        lateral_reference_index = int(lateral_reference_index)
                        target_lateral = h5_lateral_reference_offsets[lateral_reference_index]
                    joined, smoothing_audit = _route_normal_constant_offset(
                        joined,
                        route_samples, route_tangents, route_stations, route_index,
                        float(smoothing["maximum_correction_m"]),
                        marking_start_s, marking_end_s,
                        target_lateral,
                    )
                    if lateral_reference_index is not None:
                        smoothing_audit["aligned_to_h5_vector_index"] = lateral_reference_index
                sampled, tangents = resample_polyline(joined, step)
                selected = np.zeros(len(sampled), dtype=bool)
                owners: list[str | None] = [None] * len(sampled)
                stations = np.full(len(sampled), np.nan, dtype=np.float64)
                marking_maximum_route_distance = float(marking.get(
                    "maximum_route_distance_m",
                    config["geometry"]["maximum_route_distance_m"],
                ))
                marking_maximum_heading = float(marking.get(
                    "maximum_heading_difference_deg",
                    config["geometry"]["maximum_heading_difference_deg"],
                ))
                for index, (point, tangent) in enumerate(zip(sampled, tangents)):
                    _, _, match = route_index.query(
                        point, tangent,
                        max_distance_m=marking_maximum_route_distance,
                        max_heading_deg=marking_maximum_heading,
                    )
                    if match is None:
                        continue
                    station = float(route_stations[match])
                    owner = str(route_index.owners[match])
                    inside = (
                        marking_start_s <= station < marking_end_s
                        if segment.get("end_exclusive")
                        else marking_start_s <= station <= marking_end_s
                    )
                    if inside and owner in segment_links:
                        selected[index], owners[index], stations[index] = True, owner, station
                # A reviewed multi-vector bridge can create one resampled point
                # whose local tangent is undefined at the join.  Fill only such
                # bounded holes, and re-project them without a heading gate.
                if gap_lengths:
                    sampled_stations = cumulative_distance(sampled)
                    for hole_start, hole_end in _true_runs(~selected):
                        if hole_start == 0 or hole_end >= len(selected):
                            continue
                        hole_length = float(
                            sampled_stations[hole_end] - sampled_stations[hole_start - 1]
                        )
                        if hole_length > float(marking["maximum_gap_m"]):
                            continue
                        for index in range(hole_start, hole_end):
                            _, _, match = route_index.query(
                                sampled[index], None,
                                max_distance_m=marking_maximum_route_distance,
                                max_heading_deg=180.0,
                            )
                            if match is None:
                                continue
                            station = float(route_stations[match])
                            owner = str(route_index.owners[match])
                            inside = (
                                marking_start_s <= station < marking_end_s
                                if segment.get("end_exclusive")
                                else marking_start_s <= station <= marking_end_s
                            )
                            if inside and owner in segment_links:
                                selected[index], owners[index], stations[index] = True, owner, station
                emitted_run_count = 0
                candidate_runs = []
                for run_index, (run_start, run_end) in enumerate(_true_runs(selected)):
                    raw_points = sampled[run_start:run_end]
                    run_length = (
                        float(cumulative_distance(raw_points)[-1]) if len(raw_points) >= 2 else 0.0
                    )
                    if run_length < marking_minimum_length:
                        continue
                    candidate_runs.append((run_length, run_index, run_start, run_end))
                all_candidate_runs = list(candidate_runs)
                candidate_run_count = len(all_candidate_runs)
                if marking.get("run_selection") == "longest" and candidate_runs:
                    candidate_runs = [max(candidate_runs, key=lambda row: row[0])]
                route_start_anchor = None
                route_start_anchor_s = None
                if marking.get("prepend_route_min_source_anchor"):
                    run_endpoint_indices = [
                        endpoint_index
                        for _, _, run_start, run_end in all_candidate_runs
                        for endpoint_index in (run_start, run_end - 1)
                    ]
                    if not run_endpoint_indices:
                        raise ValueError(
                            f"H5 marking {marking['id']} has no source point for its start anchor"
                        )
                    route_start_anchor_index = int(
                        run_endpoint_indices[
                            int(np.nanargmin(stations[run_endpoint_indices]))
                        ]
                    )
                    route_start_anchor = sampled[route_start_anchor_index].copy()
                    route_start_anchor_s = float(stations[route_start_anchor_index])
                for _, run_index, run_start, run_end in candidate_runs:
                    raw_points = sampled[run_start:run_end]
                    output_points = raw_points[::output_stride]
                    if not np.array_equal(output_points[-1], raw_points[-1]):
                        output_points = np.vstack((output_points, raw_points[-1]))
                    run_owners = sorted(
                        {value for value in owners[run_start:run_end] if value}
                        | {
                            str(value)
                            for value in marking.get("additional_target_current_link_ids", [])
                        }
                    )
                    if marking.get("target_current_link_ids"):
                        run_owners = sorted({
                            str(value) for value in marking["target_current_link_ids"]
                        })
                    alignment_vector_index = marking.get("align_route_end_to_h5_vector_index")
                    alignment_route_s_m = None
                    endpoint_extension_m = 0.0
                    start_anchor_extension_m = 0.0
                    correction_ids = []
                    if route_start_anchor is not None:
                        run_start_station = float(np.nanmin(stations[run_start:run_end]))
                        if route_start_anchor_s >= run_start_station:
                            raise ValueError(
                                f"H5 marking {marking['id']} start anchor does not precede "
                                "the selected run"
                            )
                        prepend = stations[run_start] <= stations[run_end - 1]
                        endpoint = output_points[0] if prepend else output_points[-1]
                        start_anchor_extension_m = float(
                            np.linalg.norm(route_start_anchor[:2] - endpoint[:2])
                        )
                        maximum_start_extension_m = float(
                            marking["maximum_start_extension_m"]
                        )
                        if start_anchor_extension_m > maximum_start_extension_m:
                            raise ValueError(
                                f"H5 marking {marking['id']} start-anchor extension "
                                f"{start_anchor_extension_m:.3f} m exceeds "
                                f"{maximum_start_extension_m:.3f} m"
                            )
                        if prepend:
                            output_points = np.vstack((route_start_anchor, output_points))
                        else:
                            output_points = np.vstack((output_points, route_start_anchor))
                        correction_ids.append(str(marking["start_anchor_alignment_id"]))
                    if alignment_vector_index is not None:
                        alignment_vector_index = int(alignment_vector_index)
                        if alignment_vector_index not in h5_endpoint_stations:
                            raise ValueError(
                                f"H5 endpoint alignment vector is missing: {alignment_vector_index}"
                            )
                        alignment_route_s_m = h5_endpoint_stations[alignment_vector_index]
                        output_points, endpoint_extension_m = _extend_route_end(
                            output_points, alignment_route_s_m,
                            route_samples, route_tangents, route_stations, route_index,
                            float(marking["maximum_extension_m"]),
                        )
                        correction_ids.append(str(marking["endpoint_alignment_id"]))
                    if gap_lengths:
                        correction_ids.append(str(marking["gap_bridge_id"]))
                    if smoothing_audit:
                        correction_ids.append(str(smoothing["correction_id"]))
                    if polyline_smoothing_audit:
                        correction_ids.append(str(marking["polyline_smoothing"]["correction_id"]))
                    if marking.get("attribute_override_id"):
                        correction_ids.append(str(marking["attribute_override_id"]))
                    source_id = "won_tae_h5_vectors_" + "_".join(str(value) for value in vector_indices)
                    feature = make_candidate_feature(
                        feature_type="lane_marking",
                        canonical_id=(
                            f"{config['map_id']}:lane_marking:{segment['id']}:"
                            f"{marking['id']}:{run_index:02d}"
                        ),
                        points=output_points,
                        attributes={
                            "color": output_color,
                            "pattern": str(marking["pattern"]),
                            "line_group": "single",
                            "functional_role": str(marking.get(
                                "functional_role", "lane_boundary_paint"
                            )),
                            "dash_length_m": float(marking.get("dash_length_m", 0.0)),
                            "dash_gap_m": float(marking.get("dash_gap_m", 0.0)),
                            "dash_phase_m": float(marking.get("dash_phase_m", 0.0)),
                            "line_width_m": float(marking.get("line_width_m", 0.15)),
                            "curation_segment_id": str(segment["id"]),
                            "source_display": "won_tae_h5",
                            "geometry_quality": (
                                "direct_observed_route_normal_smoothed"
                                if smoothing_audit else
                                "direct_observed_arc_length_smoothed"
                                if polyline_smoothing_audit else
                                "direct_observed_with_reviewed_gap_bridge"
                                if gap_lengths else "direct_observed"
                            ),
                        },
                        source_id=source_id,
                        source_sha256=h5_sha,
                        target_link_ids=run_owners,
                        validation={
                            "human_reviewed": True,
                            "evidence_ids": list(segment.get("evidence", [])),
                            "route_s_min_m": min(
                                float(np.nanmin(stations[run_start:run_end])),
                                route_start_anchor_s
                                if route_start_anchor_s is not None else np.inf,
                            ),
                            "route_s_max_m": max(
                                float(np.nanmax(stations[run_start:run_end])),
                                float(alignment_route_s_m)
                                if alignment_route_s_m is not None else -np.inf,
                            ),
                            "h5_vector_indices": vector_indices,
                            "h5_source_file": expected_file,
                            "h5_source_color": expected_source_color,
                            "output_color": output_color,
                            "h5_source_sides_audit_only": [
                                vector["source_side"] for vector in selected_vectors
                            ],
                            "h5_run_selection": str(marking.get("run_selection", "all")),
                            "h5_candidate_run_count": candidate_run_count,
                            "h5_discarded_run_count": candidate_run_count - len(candidate_runs),
                            "gap_bridge_lengths_m": gap_lengths,
                            "correction_ids": correction_ids,
                            "endpoint_alignment_vector_index": alignment_vector_index,
                            "endpoint_extension_m": endpoint_extension_m,
                            "route_start_anchor_s_m": route_start_anchor_s,
                            "start_anchor_extension_m": start_anchor_extension_m,
                            "route_normal_smoothing": smoothing_audit,
                            "polyline_smoothing": polyline_smoothing_audit,
                            "canonical_side_from_h5_source_side_forbidden": True,
                        },
                    )
                    feature["provenance"] = {
                        "source_kind": "current_lidar_semantic",
                        "source_id": source_id,
                        "source_sha256": h5_sha,
                        "transform": (
                            "reviewed_constant_median_route_normal_offset"
                            if smoothing_audit
                            else "reviewed_arc_length_moving_average"
                            if polyline_smoothing_audit
                            else "won_tae_h5_baked_vector_no_additional_offset"
                        ),
                    }
                    feature["status"] = "reviewed"
                    features.append(feature)
                    inventory.append({
                        "segment_id": segment["id"],
                        "source_id": source_id,
                        "run_index": run_index,
                        "color": output_color,
                        "source_pattern": "unknown",
                        "output_pattern": str(marking["pattern"]),
                        "current_link_ids": ";".join(run_owners),
                        "route_s_min_m": feature["validation"]["route_s_min_m"],
                        "route_s_max_m": feature["validation"]["route_s_max_m"],
                        "attribute_override_ids": ";".join(correction_ids),
                        "source_display": "won_tae_h5",
                        "source_vector_indices": ";".join(str(value) for value in vector_indices),
                        "gap_bridge_lengths_m": ";".join(f"{value:.6f}" for value in gap_lengths),
                    })
                    emitted_run_count += 1
                if emitted_run_count != 1:
                    run_summary = [
                        {
                            "length_m": float(run_length),
                            "route_s_min_m": float(np.nanmin(stations[run_start:run_end])),
                            "route_s_max_m": float(np.nanmax(stations[run_start:run_end])),
                            "start_xy": sampled[run_start, :2].tolist(),
                            "end_xy": sampled[run_end - 1, :2].tolist(),
                        }
                        for run_length, _, run_start, run_end in candidate_runs
                    ]
                    raise ValueError(
                        f"H5 marking {marking['id']} emitted {emitted_run_count} runs; "
                        f"expected exactly one: {run_summary}"
                    )

    profile_requests = [
        (segment, marking)
        for segment in implemented
        for marking in segment.get("h5_route_profile_markings", [])
    ]
    for segment, marking in profile_requests:
        if not h5_vectors or not h5_sha:
            raise ValueError("H5 route profile requires the pinned Won-tae source")
        vector_indices = [int(value) for value in marking["vector_indices"]]
        selected_vectors = [h5_vectors[index] for index in vector_indices]
        expected_file = str(marking["expected_source_file"])
        expected_source_color = str(marking["expected_source_color"]).lower()
        for vector in selected_vectors:
            if (
                vector["source_file"] != expected_file
                or vector["color"] != expected_source_color
            ):
                raise ValueError(
                    f"H5 route-profile vector {vector['index']} metadata changed: "
                    f"{vector['source_file']} {vector['color']}"
                )
        start_s = max(
            float(segment["route_s_min_m"]),
            float(marking.get("route_s_min_m", segment["route_s_min_m"])),
        )
        end_s = min(
            float(segment["route_s_max_m"]),
            float(marking.get("route_s_max_m", segment["route_s_max_m"])),
        )
        allowed_owners = {
            str(value) for value in marking.get(
                "route_owner_current_link_ids", segment["current_link_ids"]
            )
        }
        try:
            output_points, profile_audit = _route_normal_robust_profile(
                [vector["points"] for vector in selected_vectors],
                route_samples, route_tangents, route_stations, route_owners, route_index,
                allowed_owners, start_s, end_s, step,
                float(marking["station_bin_m"]),
                float(marking["smoothing_length_m"]),
                float(marking["maximum_source_distance_m"]),
                float(marking["maximum_heading_difference_deg"]),
                float(marking["maximum_profile_gap_m"]),
                float(marking["maximum_smoothing_correction_m"]),
            )
        except ValueError as error:
            raise ValueError(f"H5 route profile {marking['id']}: {error}") from error
        raw_output_points = output_points
        output_points = raw_output_points[::output_stride]
        if not np.array_equal(output_points[-1], raw_output_points[-1]):
            output_points = np.vstack((output_points, raw_output_points[-1]))
        startpoint_extension_m = 0.0
        startpoint_alignment_route_s_m = None
        startpoint_alignment_id = None
        if "align_route_start_to_route_s_m" in marking:
            startpoint_alignment_route_s_m = float(
                marking["align_route_start_to_route_s_m"]
            )
            startpoint_alignment_id = str(marking["startpoint_alignment_id"])
            output_points, startpoint_extension_m = _extend_route_start(
                output_points,
                startpoint_alignment_route_s_m,
                route_samples, route_tangents, route_stations, route_index,
                float(marking["maximum_start_extension_m"]),
            )
        endpoint_extension_m = 0.0
        endpoint_alignment_route_s_m = None
        endpoint_alignment_id = None
        if "align_route_end_to_route_s_m" in marking:
            endpoint_alignment_route_s_m = float(
                marking["align_route_end_to_route_s_m"]
            )
            endpoint_alignment_id = str(marking["endpoint_alignment_id"])
            output_points, endpoint_extension_m = _extend_route_end(
                output_points,
                endpoint_alignment_route_s_m,
                route_samples, route_tangents, route_stations, route_index,
                float(marking["maximum_extension_m"]),
            )
        source_id = "won_tae_h5_profile_" + "_".join(
            str(value) for value in vector_indices
        )
        correction_id = str(marking["profile_correction_id"])
        feature = make_candidate_feature(
            feature_type="lane_marking",
            canonical_id=(
                f"{config['map_id']}:lane_marking:{segment['id']}:"
                f"{marking['id']}"
            ),
            points=output_points,
            attributes={
                "color": str(marking["color"]),
                "pattern": str(marking["pattern"]),
                "line_group": "single",
                "functional_role": str(marking["functional_role"]),
                "dash_length_m": float(marking.get("dash_length_m", 0.0)),
                "dash_gap_m": float(marking.get("dash_gap_m", 0.0)),
                "dash_phase_m": float(marking.get("dash_phase_m", 0.0)),
                "line_width_m": float(marking.get("line_width_m", 0.15)),
                "curation_segment_id": str(segment["id"]),
                "source_display": "won_tae_h5_robust_profile",
                "geometry_quality": "direct_observed_robust_route_normal_profile",
            },
            source_id=source_id,
            source_sha256=h5_sha,
            target_link_ids=[str(value) for value in marking["target_current_link_ids"]],
            validation={
                "human_reviewed": True,
                "evidence_ids": list(segment.get("evidence", [])),
                "route_s_min_m": min(
                    profile_audit["route_s_min_m"],
                    startpoint_alignment_route_s_m
                    if startpoint_alignment_route_s_m is not None else np.inf,
                ),
                "route_s_max_m": max(
                    profile_audit["route_s_max_m"],
                    endpoint_alignment_route_s_m
                    if endpoint_alignment_route_s_m is not None else -np.inf,
                ),
                "h5_vector_indices": vector_indices,
                "h5_source_file": expected_file,
                "h5_source_color": expected_source_color,
                "output_color": str(marking["color"]),
                "profile_correction_id": correction_id,
                "route_normal_profile": profile_audit,
                "endpoint_observation_anchors_preserved": True,
                "startpoint_alignment_id": startpoint_alignment_id,
                "startpoint_alignment_route_s_m": startpoint_alignment_route_s_m,
                "startpoint_extension_m": startpoint_extension_m,
                "endpoint_alignment_id": endpoint_alignment_id,
                "endpoint_alignment_route_s_m": endpoint_alignment_route_s_m,
                "endpoint_extension_m": endpoint_extension_m,
                "canonical_side_from_h5_source_side_forbidden": True,
            },
        )
        feature["provenance"] = {
            "source_kind": "current_lidar_semantic",
            "source_id": source_id,
            "source_sha256": h5_sha,
            "transform": "robust_station_bin_route_normal_profile",
        }
        feature["status"] = "reviewed"
        features.append(feature)
        inventory.append({
            "segment_id": segment["id"],
            "source_id": source_id,
            "run_index": 0,
            "color": str(marking["color"]),
            "source_pattern": "unknown",
            "output_pattern": str(marking["pattern"]),
            "current_link_ids": ";".join(
                str(value) for value in marking["target_current_link_ids"]
            ),
            "route_s_min_m": feature["validation"]["route_s_min_m"],
            "route_s_max_m": feature["validation"]["route_s_max_m"],
            "attribute_override_ids": correction_id,
            "source_display": "won_tae_h5_robust_profile",
            "source_vector_indices": ";".join(str(value) for value in vector_indices),
            "gap_bridge_lengths_m": "",
        })

    parallel_requests = [
        (segment, marking)
        for segment in implemented
        for marking in segment.get("parallel_route_profile_markings", [])
    ]
    for segment, marking in parallel_requests:
        def select_profile(source_id: str) -> dict:
            matches = [
                feature for feature in features
                if feature["feature_type"] == "lane_marking"
                and feature["attributes"].get("curation_segment_id")
                == str(segment["id"])
                and feature["provenance"].get("source_id") == source_id
            ]
            if len(matches) != 1:
                raise ValueError(
                    f"parallel profile source {source_id} matched {len(matches)} features"
                )
            return matches[0]

        reference_source_id = str(marking["reference_source_id"])
        anchor_source_id = str(
            marking.get("anchor_source_id", marking.get("anchor_legacy_source_id"))
        )
        reference = select_profile(reference_source_id)
        replaced_feature = (
            select_profile(str(marking["replace_source_id"]))
            if "replace_source_id" in marking else None
        )
        if "anchor_legacy_source_id" in marking:
            if anchor_source_id not in legacy_references:
                raise ValueError(
                    f"parallel profile legacy anchor is missing: {anchor_source_id}"
                )
            legacy_anchor = legacy_references[anchor_source_id]
            anchor = {
                "id": f"legacy_raw:{anchor_source_id}",
                "attributes": {
                    "color": str(legacy_anchor["color"]),
                    "pattern": _pattern(str(legacy_anchor["shape"])),
                },
                "geometry": {
                    "coordinates": np.asarray(
                        legacy_anchor["coordinates"], dtype=np.float64,
                    ).tolist(),
                },
                "provenance": {
                    "source_id": anchor_source_id,
                    "source_sha256": legacy_sha,
                },
            }
        else:
            anchor = select_profile(anchor_source_id)
        route_end_reference = None
        if "route_end_legacy_source_id" in marking:
            route_end_source_id = str(marking["route_end_legacy_source_id"])
            if route_end_source_id not in legacy_references:
                raise ValueError(
                    f"parallel route-end legacy source is missing: {route_end_source_id}"
                )
            legacy_endpoint_s = max(_endpoint_route_station(
                np.asarray(
                    legacy_references[route_end_source_id]["coordinates"],
                    dtype=np.float64,
                ),
                route_index, route_stations,
            ))
            if abs(
                legacy_endpoint_s - float(marking["route_s_max_m"])
            ) > float(marking["route_end_tolerance_m"]):
                raise ValueError(
                    f"parallel route-end reference {route_end_source_id} changed"
                )
            route_end_reference = {
                "legacy_source_id": route_end_source_id,
                "legacy_route_end_s_m": legacy_endpoint_s,
                "selection_rule": "legacy_centerline_physical_end",
            }
        expected_reference_pattern = str(
            marking.get("reference_pattern", marking["pattern"])
        )
        if (
            reference["attributes"].get("color")
            != str(marking["reference_color"])
            or reference["attributes"].get("pattern") != expected_reference_pattern
            or anchor["attributes"].get("pattern") != str(marking["pattern"])
        ):
            raise ValueError(
                f"parallel profile {marking['id']} source color/pattern changed"
            )
        output_points, parallel_audit = _parallel_route_normal_profile(
            points_xyz(reference["geometry"]["coordinates"]),
            points_xyz(anchor["geometry"]["coordinates"]),
            route_samples, route_tangents, route_stations, route_owners,
            route_index,
            {str(value) for value in marking["route_owner_current_link_ids"]},
            float(marking["route_s_min_m"]),
            float(marking["route_s_max_m"]),
            step,
            float(marking["maximum_route_distance_m"]),
            float(marking["maximum_heading_difference_deg"]),
            float(marking["maximum_anchor_extrapolation_m"]),
            float(marking["lateral_separation_min_m"]),
            float(marking["lateral_separation_max_m"]),
        )
        if marking.get("align_start_to_anchor_endpoint", False):
            anchor_points = points_xyz(anchor["geometry"]["coordinates"])
            candidates = (anchor_points[0], anchor_points[-1])
            anchor_endpoint = min(
                candidates,
                key=lambda point: float(np.linalg.norm(
                    point[:2] - output_points[0, :2],
                )),
            )
            start_snap_m = float(np.linalg.norm(
                anchor_endpoint[:2] - output_points[0, :2],
            ))
            if start_snap_m > float(marking["maximum_start_snap_m"]):
                raise ValueError(
                    f"parallel profile {marking['id']} start snap "
                    f"{start_snap_m:.3f}m exceeds its gate"
                )
            output_points[0] = anchor_endpoint
            parallel_audit["start_anchor_snap_m"] = start_snap_m
            parallel_audit["start_anchor_endpoint_aligned"] = True
        source_id = f"parallel:{reference_source_id}:anchored:{anchor_source_id}"
        selection_id = str(marking["selection_id"])
        feature = make_candidate_feature(
            feature_type="lane_marking",
            canonical_id=(
                f"{config['map_id']}:lane_marking:{segment['id']}:"
                f"{marking['id']}"
            ),
            points=output_points,
            attributes={
                "color": str(marking["color"]),
                "pattern": str(marking["pattern"]),
                "line_group": "single",
                "functional_role": str(marking["functional_role"]),
                "dash_length_m": float(marking.get("dash_length_m", 0.0)),
                "dash_gap_m": float(marking.get("dash_gap_m", 0.0)),
                "dash_phase_m": float(marking.get("dash_phase_m", 0.0)),
                "line_width_m": float(marking.get("line_width_m", 0.15)),
                "curation_segment_id": str(segment["id"]),
                "source_display": str(marking.get(
                    "source_display", "won_tae_h5_derived_parallel",
                )),
                "geometry_quality": str(marking.get(
                    "geometry_quality",
                    "reviewed_parallel_to_direct_observed_boundary",
                )),
            },
            source_id=source_id,
            source_sha256=str(reference["provenance"]["source_sha256"]),
            target_link_ids=[
                str(value) for value in marking["target_current_link_ids"]
            ],
            validation={
                "human_reviewed": True,
                "evidence_ids": list(segment.get("evidence", [])),
                "selection_id": selection_id,
                "reference_feature_id": str(reference["id"]),
                "anchor_feature_id": str(anchor["id"]),
                "legacy_anchor_source_id": (
                    anchor_source_id
                    if "anchor_legacy_source_id" in marking else None
                ),
                "replaced_feature_id": (
                    str(replaced_feature["id"])
                    if replaced_feature is not None else None
                ),
                "parallel_route_normal_profile": parallel_audit,
                "route_end_reference": route_end_reference,
                "noisy_anchor_tail_forbidden": True,
            },
        )
        feature["provenance"] = {
            "source_kind": str(reference["provenance"]["source_kind"]),
            "source_id": source_id,
            "source_sha256": str(reference["provenance"]["source_sha256"]),
            "transform": str(marking.get(
                "provenance_transform",
                "anchored_parallel_offset_from_won_tae_h5_boundary",
            )),
        }
        feature["status"] = "reviewed"
        if replaced_feature is not None:
            features.remove(replaced_feature)
            replaced_source_id = str(marking["replace_source_id"])
            inventory[:] = [
                row for row in inventory
                if not (
                    row.get("segment_id") == str(segment["id"])
                    and row.get("source_id") == replaced_source_id
                )
            ]
        features.append(feature)
        inventory.append({
            "segment_id": segment["id"],
            "source_id": source_id,
            "run_index": 0,
            "color": str(marking["color"]),
            "source_pattern": str(reference["attributes"]["pattern"]),
            "output_pattern": str(marking["pattern"]),
            "current_link_ids": ";".join(
                str(value) for value in marking["target_current_link_ids"]
            ),
            "route_s_min_m": parallel_audit["route_s_min_m"],
            "route_s_max_m": parallel_audit["route_s_max_m"],
            "attribute_override_ids": selection_id,
            "source_display": str(marking.get(
                "source_display", "won_tae_h5_derived_parallel",
            )),
            "source_vector_indices": "",
            "gap_bridge_lengths_m": "",
        })

    midpoint_requests = [
        (segment, marking)
        for segment in implemented
        for marking in segment.get("h5_midpoint_markings", [])
    ]
    for segment, marking in midpoint_requests:
        if not h5_vectors or not h5_sha:
            raise ValueError("H5 midpoint marking requires the pinned Won-tae source")
        left_index = int(marking["left_h5_vector_index"])
        right_index = int(marking["right_h5_vector_index"])
        left_vector, right_vector = h5_vectors[left_index], h5_vectors[right_index]
        for vector, expected_file, expected_color in (
            (left_vector, str(marking["left_expected_source_file"]), str(marking["left_expected_color"])),
            (right_vector, str(marking["right_expected_source_file"]), str(marking["right_expected_color"])),
        ):
            if vector["source_file"] != expected_file or vector["color"] != expected_color:
                raise ValueError(
                    f"H5 midpoint source {vector['index']} metadata changed: "
                    f"{vector['source_file']} {vector['color']}"
                )
        left_points, left_tangents = resample_polyline(left_vector["points"], step)
        right_points, right_tangents = resample_polyline(right_vector["points"], step)
        left_indexer = GridIndex(left_points, left_tangents, cell_size_m=3.0)
        right_indexer = GridIndex(right_points, right_tangents, cell_size_m=3.0)
        start_s = float(marking.get("route_s_min_m", segment["route_s_min_m"]))
        end_s = float(marking.get("route_s_max_m", segment["route_s_max_m"]))
        route_owner = str(marking["route_owner_current_link_id"])
        selected = np.zeros(len(route_samples), dtype=bool)
        midpoint_points = np.zeros_like(route_samples)
        left_offsets = np.full(len(route_samples), np.nan, dtype=np.float64)
        right_offsets = np.full(len(route_samples), np.nan, dtype=np.float64)
        maximum_distance = float(marking["maximum_source_distance_m"])
        maximum_heading = float(marking["maximum_heading_difference_deg"])
        for index, (point, tangent, station, owner) in enumerate(zip(
            route_samples, route_tangents, route_stations, route_owners,
        )):
            if not (start_s <= station < end_s) or owner != route_owner:
                continue
            _, _, left_match = left_indexer.query(
                point, tangent, maximum_distance, maximum_heading,
            )
            _, _, right_match = right_indexer.query(
                point, tangent, maximum_distance, maximum_heading,
            )
            if left_match is None or right_match is None:
                continue
            normal = np.asarray([-tangent[1], tangent[0]])
            left_offset = float(np.dot(left_points[left_match, :2] - point[:2], normal))
            right_offset = float(np.dot(right_points[right_match, :2] - point[:2], normal))
            if left_offset <= right_offset:
                continue
            midpoint_points[index] = point
            midpoint_points[index, :2] += 0.5 * (left_offset + right_offset) * normal
            midpoint_points[index, 2] = 0.5 * (
                left_points[left_match, 2] + right_points[right_match, 2]
            )
            left_offsets[index], right_offsets[index], selected[index] = (
                left_offset, right_offset, True,
            )
        emitted_run_count = 0
        for run_index, (run_start, run_end) in enumerate(_true_runs(selected)):
            raw_points = midpoint_points[run_start:run_end]
            if len(raw_points) < 2 or cumulative_distance(raw_points)[-1] < minimum_length:
                continue
            output_points = raw_points[::output_stride]
            if not np.array_equal(output_points[-1], raw_points[-1]):
                output_points = np.vstack((output_points, raw_points[-1]))
            widths = left_offsets[run_start:run_end] - right_offsets[run_start:run_end]
            midpoint_offsets = 0.5 * (
                left_offsets[run_start:run_end] + right_offsets[run_start:run_end]
            )
            source_id = f"won_tae_h5_midpoint_{left_index}_{right_index}"
            feature = make_candidate_feature(
                feature_type="lane_marking",
                canonical_id=(
                    f"{config['map_id']}:lane_marking:{segment['id']}:"
                    f"{marking['id']}:{run_index:02d}"
                ),
                points=output_points,
                attributes={
                    "color": str(marking["color"]),
                    "pattern": str(marking["pattern"]),
                    "line_group": "single",
                    "functional_role": str(marking["functional_role"]),
                    "dash_length_m": float(marking["dash_length_m"]),
                    "dash_gap_m": float(marking["dash_gap_m"]),
                    "dash_phase_m": float(marking.get("dash_phase_m", 0.0)),
                    "line_width_m": float(marking.get("line_width_m", 0.15)),
                    "curation_segment_id": str(segment["id"]),
                    "source_display": "won_tae_h5_derived_midpoint",
                    "geometry_quality": "direct_observed_boundary_midpoint",
                },
                source_id=source_id,
                source_sha256=h5_sha,
                target_link_ids=[str(value) for value in marking["target_current_link_ids"]],
                validation={
                    "human_reviewed": True,
                    "evidence_ids": list(segment.get("evidence", [])),
                    "route_s_min_m": float(route_stations[run_start]),
                    "route_s_max_m": float(route_stations[run_end - 1]),
                    "left_h5_vector_index": left_index,
                    "right_h5_vector_index": right_index,
                    "observed_two_lane_width_median": float(np.median(widths)),
                    "observed_two_lane_width_p05": float(np.quantile(widths, 0.05)),
                    "observed_two_lane_width_p95": float(np.quantile(widths, 0.95)),
                    "midpoint_route_offset_median": float(np.median(midpoint_offsets)),
                    "selection_id": str(marking["selection_id"]),
                    "legacy_single_lane_divider_forbidden": True,
                },
            )
            feature["provenance"] = {
                "source_kind": "current_lidar_semantic",
                "source_id": source_id,
                "source_sha256": h5_sha,
                "transform": "route_normal_midpoint_of_won_tae_h5_boundaries",
            }
            feature["status"] = "reviewed"
            features.append(feature)
            inventory.append({
                "segment_id": segment["id"],
                "source_id": source_id,
                "run_index": run_index,
                "color": str(marking["color"]),
                "source_pattern": "derived_midpoint",
                "output_pattern": str(marking["pattern"]),
                "current_link_ids": ";".join(
                    str(value) for value in marking["target_current_link_ids"]
                ),
                "route_s_min_m": feature["validation"]["route_s_min_m"],
                "route_s_max_m": feature["validation"]["route_s_max_m"],
                "attribute_override_ids": str(marking["selection_id"]),
                "source_display": "won_tae_h5_derived_midpoint",
                "source_vector_indices": f"{left_index};{right_index}",
                "gap_bridge_lengths_m": "",
            })
            emitted_run_count += 1
        if emitted_run_count != 1:
            raise ValueError(
                f"H5 midpoint marking {marking['id']} emitted {emitted_run_count} runs; expected one"
            )

    guide_segments = [
        segment for segment in implemented if segment.get("preplan_guides")
    ]
    if guide_segments:
        preplan_by_id = {
            str(feature["id"]): feature for feature in review["adopted_legacy_boundaries"]
        }
        for segment in guide_segments:
            segment_links = {str(value) for value in segment["current_link_ids"]}
            for guide in segment["preplan_guides"]:
                source_id = str(guide["preplan_source_id"])
                if source_id not in preplan_by_id:
                    raise ValueError(f"pre-plan guide source is missing: {source_id}")
                source = preplan_by_id[source_id]
                source_pattern = str(
                    source.get("attributes", {}).get("lane_shape", "unknown")
                )
                output_pattern = str(guide["pattern"])
                pattern_override_id = (
                    str(guide["pattern_override_id"])
                    if "pattern_override_id" in guide else None
                )
                if source_pattern != output_pattern and pattern_override_id is None:
                    raise ValueError(
                        f"pre-plan guide {guide['id']} changes pattern without an override id"
                    )
                selection_id = str(
                    guide.get("selection_id", pattern_override_id or guide["id"])
                )
                sampled, tangents = resample_polyline(
                    source["geometry"]["coordinates"], step,
                )
                selected = np.zeros(len(sampled), dtype=bool)
                owners: list[str | None] = [None] * len(sampled)
                stations = np.full(len(sampled), np.nan, dtype=np.float64)
                guide_start = float(guide["route_s_min_m"])
                guide_end = float(guide["route_s_max_m"])
                for index, (point, tangent) in enumerate(zip(sampled, tangents)):
                    _, _, match = route_index.query(
                        point, tangent,
                        max_distance_m=float(config["geometry"]["maximum_route_distance_m"]),
                        max_heading_deg=float(config["geometry"]["maximum_heading_difference_deg"]),
                    )
                    if match is None:
                        continue
                    station = float(route_stations[match])
                    owner = str(route_index.owners[match])
                    if guide_start <= station < guide_end and owner in segment_links:
                        selected[index], owners[index], stations[index] = True, owner, station
                emitted_run_count = 0
                for run_index, (run_start, run_end) in enumerate(_true_runs(selected)):
                    raw_points = sampled[run_start:run_end]
                    if len(raw_points) < 2 or cumulative_distance(raw_points)[-1] < minimum_length:
                        continue
                    output_points = raw_points[::output_stride]
                    if not np.array_equal(output_points[-1], raw_points[-1]):
                        output_points = np.vstack((output_points, raw_points[-1]))
                    run_owners = sorted(
                        {value for value in owners[run_start:run_end] if value}
                        | {
                            str(value)
                            for value in guide.get("additional_target_current_link_ids", [])
                        }
                    )
                    feature = make_candidate_feature(
                        feature_type="lane_marking",
                        canonical_id=(
                            f"{config['map_id']}:lane_marking:{segment['id']}:"
                            f"{guide['id']}:{run_index:02d}"
                        ),
                        points=output_points,
                        attributes={
                            "color": str(guide["color"]),
                            "pattern": output_pattern,
                            "line_group": "single",
                            "functional_role": "intersection_guide_paint",
                            "dash_length_m": float(guide["dash_length_m"]),
                            "dash_gap_m": float(guide["dash_gap_m"]),
                            "dash_phase_m": float(guide.get("dash_phase_m", 0.0)),
                            "line_width_m": float(guide.get("line_width_m", 0.15)),
                            "curation_segment_id": str(segment["id"]),
                            "source_display": "cleaned_preplan_guide",
                            "geometry_quality": "current_topology_guide_reviewed",
                        },
                        source_id=source_id,
                        source_sha256=str(source["provenance"]["source_sha256"]),
                        target_link_ids=run_owners,
                        validation={
                            "human_reviewed": True,
                            "evidence_ids": list(segment.get("evidence", [])),
                            "route_s_min_m": float(np.nanmin(stations[run_start:run_end])),
                            "route_s_max_m": float(np.nanmax(stations[run_start:run_end])),
                            "source_pattern": source_pattern,
                            "selection_id": selection_id,
                            "pattern_override_id": pattern_override_id,
                            "continuous_solid_boundary_forbidden": True,
                        },
                    )
                    feature["provenance"] = {
                        "source_kind": str(source["provenance"]["source_kind"]),
                        "source_id": source_id,
                        "source_sha256": str(source["provenance"]["source_sha256"]),
                        "transform": (
                            "reviewed_route_clip"
                            if source_pattern == output_pattern
                            else "reviewed_route_clip_and_pattern_override"
                        ),
                    }
                    feature["status"] = "reviewed"
                    features.append(feature)
                    inventory.append({
                        "segment_id": segment["id"],
                        "source_id": source_id,
                        "run_index": run_index,
                        "color": str(guide["color"]),
                        "source_pattern": source_pattern,
                        "output_pattern": output_pattern,
                        "current_link_ids": ";".join(run_owners),
                        "route_s_min_m": feature["validation"]["route_s_min_m"],
                        "route_s_max_m": feature["validation"]["route_s_max_m"],
                        "attribute_override_ids": pattern_override_id or selection_id,
                        "source_display": "cleaned_preplan_guide",
                        "source_vector_indices": "",
                        "gap_bridge_lengths_m": "",
                    })
                    emitted_run_count += 1
                if emitted_run_count != 1:
                    raise ValueError(
                        f"pre-plan guide {guide['id']} emitted {emitted_run_count} runs; expected one"
                    )

    review_marking_segments = [
        segment for segment in implemented if segment.get("review_markings")
    ]
    if review_marking_segments:
        source_collections = {
            "preplan": list(review["adopted_legacy_boundaries"]),
            "observed": list(review["observed_document"]["features"]),
            "legacy_raw": [
                {
                    "id": f"legacy_raw:{reference['id']}",
                    "geometry": {
                        "coordinates": np.asarray(
                            reference["coordinates"], dtype=np.float64,
                        ).tolist(),
                    },
                    "attributes": {
                        "color": str(reference["color"]),
                        "lane_shape": str(reference["shape"]),
                        "dash_length_m": float(reference["dash_length_m"]),
                        "dash_gap_m": float(reference["dash_gap_m"]),
                        "line_width_m": float(reference["line_width_m"]),
                    },
                    "provenance": {
                        "source_kind": "legacy_katri",
                        "source_id": str(reference["id"]),
                        "source_sha256": legacy_sha,
                    },
                    "topology": {
                        "current_link_ids": [
                            str(value)
                            for value in reference["target_current_link_ids"]
                        ],
                    },
                }
                for reference in curation_legacy_references
            ],
        }
        for segment in review_marking_segments:
            segment_links = {str(value) for value in segment["current_link_ids"]}
            for marking in segment["review_markings"]:
                collection_name = str(marking["source_collection"])
                if collection_name not in source_collections:
                    raise ValueError(
                        f"review marking source collection is invalid: {collection_name}"
                    )
                collection = source_collections[collection_name]
                if "source_feature_id" in marking:
                    matches = [
                        row for row in collection
                        if str(row["id"]) == str(marking["source_feature_id"])
                    ]
                else:
                    matches = [
                        row for row in collection
                        if str(row.get("provenance", {}).get("source_id"))
                        == str(marking["source_provenance_id"])
                    ]
                if len(matches) != 1:
                    raise ValueError(
                        f"review marking {marking['id']} matched {len(matches)} "
                        f"{collection_name} features"
                    )
                source = matches[0]
                source_attributes = source.get("attributes", {})
                source_mgeo = source_attributes.get("mgeo", {})
                raw_source_pattern = source_attributes.get(
                    "lane_shape", source_mgeo.get("lane_shape", "unknown"),
                )
                source_pattern = (
                    "unknown" if str(raw_source_pattern).lower() == "unknown"
                    else _pattern(str(raw_source_pattern))
                )
                raw_source_color = source_attributes.get(
                    "color", source_attributes.get(
                        "lane_color", source_mgeo.get("lane_color", "unknown"),
                    ),
                )
                if isinstance(raw_source_color, list):
                    raw_source_color = raw_source_color[0] if raw_source_color else "unknown"
                source_color = str(raw_source_color).lower()
                output_color = str(marking["color"]).lower()
                output_pattern = str(marking["pattern"]).lower()
                pattern_override_id = (
                    str(marking["pattern_override_id"])
                    if "pattern_override_id" in marking else None
                )
                color_override_id = (
                    str(marking["color_override_id"])
                    if "color_override_id" in marking else None
                )
                if source_pattern != output_pattern and pattern_override_id is None:
                    raise ValueError(
                        f"review marking {marking['id']} changes pattern without an override"
                    )
                if source_color not in {"unknown", output_color} and color_override_id is None:
                    raise ValueError(
                        f"review marking {marking['id']} changes color without an override"
                    )
                sampled, tangents = resample_polyline(
                    source["geometry"]["coordinates"], step,
                )
                selected = np.zeros(len(sampled), dtype=bool)
                owners: list[str | None] = [None] * len(sampled)
                stations = np.full(len(sampled), np.nan, dtype=np.float64)
                marking_start = float(marking["route_s_min_m"])
                marking_end = float(marking["route_s_max_m"])
                use_full_source_geometry = bool(
                    marking.get("use_full_source_geometry", False)
                )
                for index, (point, tangent) in enumerate(zip(sampled, tangents)):
                    _, _, match = route_index.query(
                        point, tangent,
                        max_distance_m=float(marking.get(
                            "maximum_route_distance_m",
                            config["geometry"]["maximum_route_distance_m"],
                        )),
                        max_heading_deg=float(marking.get(
                            "maximum_heading_difference_deg",
                            config["geometry"]["maximum_heading_difference_deg"],
                        )),
                    )
                    if match is None:
                        continue
                    station = float(route_stations[match])
                    owner = str(route_index.owners[match])
                    if use_full_source_geometry or (
                        marking_start <= station < marking_end and owner in segment_links
                    ):
                        selected[index], owners[index], stations[index] = True, owner, station
                emitted_run_count = 0
                for run_index, (run_start, run_end) in enumerate(_true_runs(selected)):
                    raw_points = sampled[run_start:run_end]
                    if (
                        len(raw_points) < 2
                        or cumulative_distance(raw_points)[-1] < float(marking.get(
                            "minimum_segment_length_m", minimum_length,
                        ))
                    ):
                        continue
                    run_route_min = float(np.nanmin(stations[run_start:run_end]))
                    run_route_max = float(np.nanmax(stations[run_start:run_end]))
                    if use_full_source_geometry:
                        maximum_extension = float(
                            marking["maximum_full_source_route_extension_m"]
                        )
                        if (
                            run_route_min < marking_start - maximum_extension
                            or run_route_max > marking_end + maximum_extension
                        ):
                            raise ValueError(
                                f"review marking {marking['id']} full-source geometry "
                                f"projects to [{run_route_min:.3f}, {run_route_max:.3f}], "
                                f"outside its reviewed extension gate"
                            )
                    output_points = raw_points[::output_stride]
                    if not np.array_equal(output_points[-1], raw_points[-1]):
                        output_points = np.vstack((output_points, raw_points[-1]))
                    start_alignment_s = None
                    start_extension_m = 0.0
                    if "align_route_start_to_route_s_m" in marking:
                        start_alignment_s = float(
                            marking["align_route_start_to_route_s_m"]
                        )
                        try:
                            output_points, start_extension_m = _extend_route_start(
                                output_points, start_alignment_s,
                                route_samples, route_tangents, route_stations, route_index,
                                float(marking["maximum_start_extension_m"]),
                            )
                        except ValueError as error:
                            raise ValueError(
                                f"review marking {marking['id']} start alignment: {error}"
                            ) from error
                    end_alignment_s = None
                    end_extension_m = 0.0
                    if "align_route_end_to_route_s_m" in marking:
                        end_alignment_s = float(marking["align_route_end_to_route_s_m"])
                        try:
                            output_points, end_extension_m = _extend_route_end(
                                output_points, end_alignment_s,
                                route_samples, route_tangents, route_stations, route_index,
                                float(marking["maximum_end_extension_m"]),
                            )
                        except ValueError as error:
                            raise ValueError(
                                f"review marking {marking['id']} end alignment: {error}"
                            ) from error
                    target_link_ids = [
                        str(value) for value in marking.get(
                            "target_current_link_ids",
                            source.get("topology", {}).get("current_link_ids", []),
                        )
                    ]
                    if not target_link_ids:
                        target_link_ids = sorted({
                            value for value in owners[run_start:run_end] if value
                        })
                    selection_id = str(marking["selection_id"])
                    override_ids = [
                        value for value in (pattern_override_id, color_override_id)
                        if value is not None
                    ]
                    feature = make_candidate_feature(
                        feature_type="lane_marking",
                        canonical_id=(
                            f"{config['map_id']}:lane_marking:{segment['id']}:"
                            f"{marking['id']}:{run_index:02d}"
                        ),
                        points=output_points,
                        attributes={
                            "color": output_color,
                            "pattern": output_pattern,
                            "line_group": "single",
                            "functional_role": str(marking["functional_role"]),
                            "dash_length_m": float(marking.get("dash_length_m", 0.0)),
                            "dash_gap_m": float(marking.get("dash_gap_m", 0.0)),
                            "dash_phase_m": float(marking.get("dash_phase_m", 0.0)),
                            "line_width_m": float(marking.get("line_width_m", 0.15)),
                            "curation_segment_id": str(segment["id"]),
                            "source_display": str(marking["source_display"]),
                            "geometry_quality": str(marking["geometry_quality"]),
                        },
                        source_id=str(source["provenance"]["source_id"]),
                        source_sha256=str(source["provenance"]["source_sha256"]),
                        target_link_ids=target_link_ids,
                        validation={
                            "human_reviewed": True,
                            "evidence_ids": list(segment.get("evidence", [])),
                            "route_s_min_m": min(
                                run_route_min,
                                start_alignment_s if start_alignment_s is not None else np.inf,
                            ),
                            "route_s_max_m": max(
                                run_route_max,
                                end_alignment_s if end_alignment_s is not None else -np.inf,
                            ),
                            "source_feature_id": str(source["id"]),
                            "source_collection": collection_name,
                            "source_color": source_color,
                            "source_pattern": source_pattern,
                            "selection_id": selection_id,
                            "attribute_override_ids": override_ids,
                            "start_alignment_route_s_m": start_alignment_s,
                            "start_extension_m": start_extension_m,
                            "end_alignment_route_s_m": end_alignment_s,
                            "end_extension_m": end_extension_m,
                            "use_full_source_geometry": use_full_source_geometry,
                            "maximum_full_source_route_extension_m": (
                                float(marking["maximum_full_source_route_extension_m"])
                                if use_full_source_geometry else None
                            ),
                        },
                    )
                    transformations = [
                        "reviewed_full_source_geometry"
                        if use_full_source_geometry else "reviewed_route_clip"
                    ]
                    if override_ids:
                        transformations.append("attribute_override")
                    if start_alignment_s is not None:
                        transformations.append("start_cross_section_alignment")
                    if end_alignment_s is not None:
                        transformations.append("end_cross_section_alignment")
                    feature["provenance"] = {
                        "source_kind": str(source["provenance"]["source_kind"]),
                        "source_id": str(source["provenance"]["source_id"]),
                        "source_sha256": str(source["provenance"]["source_sha256"]),
                        "transform": "+".join(transformations),
                    }
                    feature["status"] = "reviewed"
                    features.append(feature)
                    inventory.append({
                        "segment_id": segment["id"],
                        "source_id": source["provenance"]["source_id"],
                        "run_index": run_index,
                        "color": output_color,
                        "source_pattern": source_pattern,
                        "output_pattern": output_pattern,
                        "current_link_ids": ";".join(target_link_ids),
                        "route_s_min_m": feature["validation"]["route_s_min_m"],
                        "route_s_max_m": feature["validation"]["route_s_max_m"],
                        "attribute_override_ids": ";".join(override_ids or [selection_id]),
                        "source_display": str(marking["source_display"]),
                        "source_vector_indices": "",
                        "gap_bridge_lengths_m": "",
                    })
                    emitted_run_count += 1
                expected_run_count = int(marking.get("expected_run_count", 1))
                if emitted_run_count != expected_run_count:
                    raise ValueError(
                        f"review marking {marking['id']} emitted {emitted_run_count} "
                        f"runs; expected {expected_run_count}"
                    )

    current_link_segments = [
        segment for segment in implemented if segment.get("current_link_markings")
    ]
    current_link_sha = ""
    current_link_path = None
    if current_link_segments:
        current_link_source = review["nominal_document"]["sources"]["current_link_set"]
        current_link_path = resolve_source(
            repo_root, current_link_source["configured_path"],
        )
        current_link_sha = sha256_file(current_link_path)
        if current_link_sha != str(current_link_source["sha256"]):
            raise ValueError("current link set changed since the nominal baseline")
        current_links_by_id = {
            str(row["idx"]): row for row in load_json(current_link_path)
        }
        for segment in current_link_segments:
            for marking in segment["current_link_markings"]:
                source_link_id = str(marking["source_current_link_id"])
                if source_link_id not in current_links_by_id:
                    raise ValueError(f"current link marking source is missing: {source_link_id}")
                source_link = current_links_by_id[source_link_id]
                sampled, tangents = resample_polyline(source_link["points"], step)
                source_stations = cumulative_distance(sampled)
                source_station_min = float(marking.get("source_station_min_m", 0.0))
                source_station_max = float(
                    marking.get("source_station_max_m", source_stations[-1])
                )
                station_mask = (
                    (source_stations >= source_station_min)
                    & (source_stations <= source_station_max + 1e-9)
                )
                if np.count_nonzero(station_mask) < 2:
                    raise ValueError(
                        f"current link marking {marking['id']} has an empty station interval"
                    )
                sampled = sampled[station_mask]
                tangents = tangents[station_mask]
                source_stations = source_stations[station_mask]
                side = str(marking["side"]).lower()
                if side not in {"left", "right"}:
                    raise ValueError(f"current link marking side is invalid: {side}")
                side_sign = 1.0 if side == "left" else -1.0
                anchor_vector_index = marking.get("anchor_h5_vector_index")
                anchor_audit = None
                if anchor_vector_index is not None:
                    anchor_vector_index = int(anchor_vector_index)
                    if not h5_vectors or anchor_vector_index >= len(h5_vectors):
                        raise ValueError(
                            f"current link marking H5 anchor is missing: {anchor_vector_index}"
                        )
                    anchor_points = np.asarray(
                        h5_vectors[anchor_vector_index]["points"], dtype=np.float64,
                    )
                    anchor_point = anchor_points[np.argmin(
                        np.linalg.norm(anchor_points[:, :2] - sampled[0, :2], axis=1)
                    )]
                    start_normal = np.asarray([-tangents[0, 1], tangents[0, 0]])
                    delta = anchor_point[:2] - sampled[0, :2]
                    signed_offset = float(np.dot(delta, start_normal))
                    longitudinal_gap = float(np.dot(delta, tangents[0, :2]))
                    if signed_offset * side_sign <= 0.0:
                        raise ValueError(
                            f"current link marking H5 anchor is on the wrong side: {marking['id']}"
                        )
                    if abs(longitudinal_gap) > float(marking["anchor_max_longitudinal_gap_m"]):
                        raise ValueError(
                            f"current link marking H5 anchor is longitudinally misaligned: {marking['id']}"
                        )
                    anchor_audit = {
                        "h5_vector_index": anchor_vector_index,
                        "derived_signed_normal_offset_m": signed_offset,
                        "longitudinal_gap_m": longitudinal_gap,
                        "anchor_point": anchor_point.tolist(),
                    }
                    signed_offsets = np.full(len(sampled), signed_offset, dtype=np.float64)
                elif marking.get("offset_profile"):
                    profile = sorted(
                        (
                            float(row["source_station_m"]),
                            side_sign * float(row["offset_m"]),
                        )
                        for row in marking["offset_profile"]
                    )
                    profile_stations = np.asarray(
                        [row[0] for row in profile], dtype=np.float64,
                    )
                    profile_offsets = np.asarray(
                        [row[1] for row in profile], dtype=np.float64,
                    )
                    if (
                        len(profile) < 2
                        or np.any(np.diff(profile_stations) <= 0.0)
                        or profile_stations[0] > source_stations[0] + 1e-6
                        or profile_stations[-1] < source_stations[-1] - 1e-6
                    ):
                        raise ValueError(
                            f"current link marking offset profile does not cover "
                            f"its station interval: {marking['id']}"
                        )
                    signed_offsets = np.interp(
                        source_stations, profile_stations, profile_offsets,
                    )
                    signed_offset = float(np.median(signed_offsets))
                elif "offset_start_m" in marking or "offset_end_m" in marking:
                    if "offset_start_m" not in marking or "offset_end_m" not in marking:
                        raise ValueError(
                            f"current link marking requires both offset endpoints: {marking['id']}"
                        )
                    start_offset = side_sign * float(marking["offset_start_m"])
                    end_offset = side_sign * float(marking["offset_end_m"])
                    span = max(float(source_stations[-1] - source_stations[0]), 1e-9)
                    ratio = (source_stations - source_stations[0]) / span
                    signed_offsets = start_offset + ratio * (end_offset - start_offset)
                    signed_offset = float(np.median(signed_offsets))
                else:
                    signed_offset = float(marking["offset_m"]) * side_sign
                    signed_offsets = np.full(len(sampled), signed_offset, dtype=np.float64)
                normal = np.column_stack((-tangents[:, 1], tangents[:, 0]))
                geometry = sampled.copy()
                geometry[:, :2] += signed_offsets.reshape(-1, 1) * normal
                output_points = geometry[::output_stride]
                if not np.array_equal(output_points[-1], geometry[-1]):
                    output_points = np.vstack((output_points, geometry[-1]))
                target_link_ids = [
                    str(value) for value in marking.get(
                        "target_current_link_ids", [source_link_id],
                    )
                ]
                feature = make_candidate_feature(
                    feature_type="lane_marking",
                    canonical_id=(
                        f"{config['map_id']}:lane_marking:{segment['id']}:"
                        f"{marking['id']}"
                    ),
                    points=output_points,
                    attributes={
                        "color": str(marking["color"]),
                        "pattern": str(marking["pattern"]),
                        "line_group": "single",
                        "functional_role": str(marking["functional_role"]),
                        "dash_length_m": float(marking.get("dash_length_m", 0.0)),
                        "dash_gap_m": float(marking.get("dash_gap_m", 0.0)),
                        "dash_phase_m": float(marking.get("dash_phase_m", 0.0)),
                        "line_width_m": float(marking.get("line_width_m", 0.15)),
                        "curation_segment_id": str(segment["id"]),
                        "source_display": "current_link_offset",
                        "geometry_quality": "current_topology_reviewed_offset",
                    },
                    source_id=source_link_id,
                    source_sha256=current_link_sha,
                    target_link_ids=target_link_ids,
                    validation={
                        "human_reviewed": True,
                        "evidence_ids": list(segment.get("evidence", [])),
                        "selection_id": str(marking["selection_id"]),
                        "source_current_link_id": source_link_id,
                        "side": side,
                        "signed_normal_offset_m": signed_offset,
                        "signed_normal_offset_start_m": float(signed_offsets[0]),
                        "signed_normal_offset_end_m": float(signed_offsets[-1]),
                        "source_station_min_m": float(source_stations[0]),
                        "source_station_max_m": float(source_stations[-1]),
                        "offset_profile": [
                            {
                                "source_station_m": float(row["source_station_m"]),
                                "offset_m": float(row["offset_m"]),
                            }
                            for row in marking.get("offset_profile", [])
                        ],
                        "observation_reference_ids": [
                            str(value) for value in marking.get(
                                "observation_reference_ids", [],
                            )
                        ],
                        "observation_reference_sha256": (
                            h5_sha if marking.get("observation_reference_ids") else None
                        ),
                        "parallel_reference_ids": [
                            str(value) for value in marking.get(
                                "parallel_reference_ids", [],
                            )
                        ],
                        "parallel_separation_m": (
                            float(marking["parallel_separation_m"])
                            if "parallel_separation_m" in marking else None
                        ),
                        "h5_anchor": anchor_audit,
                        "source_width_start_m": float(source_link.get("width_start", 0.0)),
                        "source_width_end_m": float(source_link.get("width_end", 0.0)),
                    },
                )
                feature["provenance"] = {
                    "source_kind": "current_mgeo",
                    "source_id": source_link_id,
                    "source_sha256": current_link_sha,
                    "transform": (
                        "current_link_normal_offset_anchored_to_won_tae_h5"
                        if anchor_audit else
                        "current_link_normal_offset_parallel_to_reviewed_boundary"
                        if marking.get("parallel_reference_ids") else
                        "current_link_piecewise_normal_offset_from_won_tae_h5_profile"
                        if marking.get("observation_reference_ids") else
                        "current_link_station_varying_normal_offset_with_driving_review"
                        if not np.allclose(signed_offsets, signed_offsets[0]) else
                        "current_link_constant_normal_offset_with_driving_review"
                    ),
                }
                feature["status"] = "reviewed"
                features.append(feature)
                inventory.append({
                    "segment_id": segment["id"],
                    "source_id": source_link_id,
                    "run_index": 0,
                    "color": str(marking["color"]),
                    "source_pattern": "current_link_offset",
                    "output_pattern": str(marking["pattern"]),
                    "current_link_ids": ";".join(target_link_ids),
                    "route_s_min_m": "",
                    "route_s_max_m": "",
                    "attribute_override_ids": str(marking["selection_id"]),
                    "source_display": "current_link_offset",
                    "source_vector_indices": "",
                    "gap_bridge_lengths_m": "",
                })

    continuity_requests = [
        (segment, bridge)
        for segment in implemented
        for bridge in segment.get("same_pattern_continuity_bridges", [])
    ]
    for segment, bridge in continuity_requests:
        def select_feature(selector: dict) -> dict:
            matches = [
                feature for feature in features
                if feature["feature_type"] == "lane_marking"
                and feature["attributes"].get("curation_segment_id")
                == str(selector["segment_id"])
                and feature["provenance"].get("source_id")
                == str(selector["source_id"])
                and (
                    "selection_id" not in selector
                    or feature["validation"].get("selection_id")
                    == str(selector["selection_id"])
                )
            ]
            if len(matches) != 1:
                raise ValueError(
                    f"continuity selector {selector} matched {len(matches)} features"
                )
            return matches[0]

        from_feature = select_feature(bridge["from"])
        to_feature = select_feature(bridge["to"])
        expected_color = str(bridge["color"])
        expected_pattern = str(bridge["pattern"])
        for feature in (from_feature, to_feature):
            if (
                feature["attributes"].get("color") != expected_color
                or feature["attributes"].get("pattern") != expected_pattern
            ):
                raise ValueError(
                    f"continuity bridge {bridge['id']} cannot cross a color/pattern transition"
                )
        from_point = np.asarray(from_feature["geometry"]["coordinates"][-1], dtype=np.float64)
        to_point = np.asarray(to_feature["geometry"]["coordinates"][0], dtype=np.float64)
        gap_m = float(np.linalg.norm(to_point[:2] - from_point[:2]))
        if gap_m <= 1e-6 or gap_m > float(bridge["maximum_gap_m"]):
            raise ValueError(
                f"continuity bridge {bridge['id']} gap {gap_m:.3f}m is outside its gate"
            )
        smoothing_audit = None
        connector_points = np.vstack((from_point, to_point))
        if bridge.get("transition_style") == "tangent_continuous_cubic":
            clipped_from, clipped_to, connector_points, smoothing_audit = (
                _smooth_polyline_handoff(
                    points_xyz(from_feature["geometry"]["coordinates"]),
                    points_xyz(to_feature["geometry"]["coordinates"]),
                    float(bridge["from_trim_m"]),
                    float(bridge["to_trim_m"]),
                    float(config["geometry"]["output_step_m"]),
                )
            )
            from_feature["geometry"]["coordinates"] = clipped_from.tolist()
            to_feature["geometry"]["coordinates"] = clipped_to.tolist()
            from_feature["validation"]["handoff_end_trim_m"] = float(
                bridge["from_trim_m"]
            )
            to_feature["validation"]["handoff_start_trim_m"] = float(
                bridge["to_trim_m"]
            )
        source_id = (
            f"continuity:{from_feature['provenance']['source_id']}:"
            f"{to_feature['provenance']['source_id']}"
        )
        feature = make_candidate_feature(
            feature_type="lane_marking",
            canonical_id=(
                f"{config['map_id']}:lane_marking:{segment['id']}:"
                f"{bridge['id']}"
            ),
            points=connector_points,
            attributes={
                "color": expected_color,
                "pattern": expected_pattern,
                "line_group": "single",
                "functional_role": str(bridge["functional_role"]),
                "dash_length_m": float(bridge.get("dash_length_m", 0.0)),
                "dash_gap_m": float(bridge.get("dash_gap_m", 0.0)),
                "dash_phase_m": float(bridge.get("dash_phase_m", 0.0)),
                "line_width_m": float(bridge.get("line_width_m", 0.15)),
                "curation_segment_id": str(segment["id"]),
                "source_display": "reviewed_same_pattern_continuity",
                "geometry_quality": (
                    "reviewed_tangent_continuous_handoff"
                    if smoothing_audit else
                    "bounded_same_pattern_endpoint_bridge"
                ),
            },
            source_id=source_id,
            source_sha256=str(from_feature["provenance"]["source_sha256"]),
            target_link_ids=[str(value) for value in bridge["target_current_link_ids"]],
            validation={
                "human_reviewed": True,
                "evidence_ids": list(segment.get("evidence", [])),
                "selection_id": str(bridge["selection_id"]),
                "gap_bridge_length_m": gap_m,
                "from_feature_id": str(from_feature["id"]),
                "to_feature_id": str(to_feature["id"]),
                "same_color_and_pattern_required": True,
                "pattern_transition_bridge_forbidden": True,
                "handoff_smoothing": smoothing_audit,
            },
        )
        feature["provenance"] = {
            "source_kind": str(from_feature["provenance"]["source_kind"]),
            "source_id": source_id,
            "source_sha256": str(from_feature["provenance"]["source_sha256"]),
            "transform": (
                "reviewed_tangent_continuous_same_pattern_handoff"
                if smoothing_audit else
                "reviewed_same_pattern_endpoint_continuity_bridge"
            ),
        }
        feature["status"] = "reviewed"
        features.append(feature)
        inventory.append({
            "segment_id": segment["id"],
            "source_id": source_id,
            "run_index": 0,
            "color": expected_color,
            "source_pattern": expected_pattern,
            "output_pattern": expected_pattern,
            "current_link_ids": ";".join(
                str(value) for value in bridge["target_current_link_ids"]
            ),
            "route_s_min_m": "",
            "route_s_max_m": "",
            "attribute_override_ids": str(bridge["selection_id"]),
            "source_display": "reviewed_same_pattern_continuity",
            "source_vector_indices": "",
            "gap_bridge_lengths_m": f"{gap_m:.6f}",
        })

    transition_requests = [
        (segment, connector)
        for segment in implemented
        for connector in segment.get("attribute_transition_connectors", [])
    ]
    for segment, connector in transition_requests:
        def select_transition_feature(selector: dict) -> dict:
            matches = [
                feature for feature in features
                if feature["feature_type"] == "lane_marking"
                and feature["attributes"].get("curation_segment_id")
                == str(selector["segment_id"])
                and feature["provenance"].get("source_id")
                == str(selector["source_id"])
                and (
                    "selection_id" not in selector
                    or feature["validation"].get("selection_id")
                    == str(selector["selection_id"])
                )
            ]
            if len(matches) != 1:
                raise ValueError(
                    f"attribute transition selector {selector} matched {len(matches)} features"
                )
            return matches[0]

        from_feature = select_transition_feature(connector["from"])
        to_feature = select_transition_feature(connector["to"])
        expected_from = (
            str(connector["from_color"]), str(connector["from_pattern"])
        )
        expected_to = (
            str(connector["to_color"]), str(connector["to_pattern"])
        )
        actual_from = (
            str(from_feature["attributes"].get("color")),
            str(from_feature["attributes"].get("pattern")),
        )
        actual_to = (
            str(to_feature["attributes"].get("color")),
            str(to_feature["attributes"].get("pattern")),
        )
        if actual_from != expected_from or actual_to != expected_to:
            raise ValueError(
                f"attribute transition {connector['id']} source attributes changed"
            )
        from_point = np.asarray(
            from_feature["geometry"]["coordinates"][-1], dtype=np.float64,
        )
        to_point = np.asarray(
            to_feature["geometry"]["coordinates"][0], dtype=np.float64,
        )
        gap_m = float(np.linalg.norm(to_point[:2] - from_point[:2]))
        if gap_m <= 1e-6 or gap_m > float(connector["maximum_gap_m"]):
            raise ValueError(
                f"attribute transition {connector['id']} gap {gap_m:.3f}m is outside its gate"
            )
        output_color = str(connector["output_color"])
        output_pattern = str(connector["output_pattern"])
        feature = make_candidate_feature(
            feature_type="lane_marking",
            canonical_id=(
                f"{config['map_id']}:lane_marking:{segment['id']}:"
                f"{connector['id']}"
            ),
            points=np.vstack((from_point, to_point)),
            attributes={
                "color": output_color,
                "pattern": output_pattern,
                "line_group": "single",
                "functional_role": str(connector["functional_role"]),
                "dash_length_m": float(connector.get("dash_length_m", 0.0)),
                "dash_gap_m": float(connector.get("dash_gap_m", 0.0)),
                "dash_phase_m": 0.0,
                "line_width_m": float(connector.get("line_width_m", 0.15)),
                "curation_segment_id": str(segment["id"]),
                "source_display": "reviewed_attribute_transition",
                "geometry_quality": "bounded_node_attribute_transition",
            },
            source_id=(
                f"transition:{from_feature['provenance']['source_id']}:"
                f"{to_feature['provenance']['source_id']}"
            ),
            source_sha256=str(to_feature["provenance"]["source_sha256"]),
            target_link_ids=[
                str(value) for value in connector["target_current_link_ids"]
            ],
            validation={
                "human_reviewed": True,
                "evidence_ids": list(segment.get("evidence", [])),
                "selection_id": str(connector["selection_id"]),
                "transition_length_m": gap_m,
                "from_feature_id": str(from_feature["id"]),
                "to_feature_id": str(to_feature["id"]),
                "from_color_pattern": list(expected_from),
                "to_color_pattern": list(expected_to),
                "output_color_pattern": [output_color, output_pattern],
                "explicit_attribute_transition": True,
            },
        )
        feature["provenance"] = {
            "source_kind": str(to_feature["provenance"]["source_kind"]),
            "source_id": (
                f"transition:{from_feature['provenance']['source_id']}:"
                f"{to_feature['provenance']['source_id']}"
            ),
            "source_sha256": str(to_feature["provenance"]["source_sha256"]),
            "transform": "reviewed_node_attribute_transition_connector",
        }
        feature["status"] = "reviewed"
        features.append(feature)
        inventory.append({
            "segment_id": segment["id"],
            "source_id": feature["provenance"]["source_id"],
            "run_index": 0,
            "color": output_color,
            "source_pattern": f"{expected_from[1]}->{expected_to[1]}",
            "output_pattern": output_pattern,
            "current_link_ids": ";".join(
                str(value) for value in connector["target_current_link_ids"]
            ),
            "route_s_min_m": "",
            "route_s_max_m": "",
            "attribute_override_ids": str(connector["selection_id"]),
            "source_display": "reviewed_attribute_transition",
            "source_vector_indices": "",
            "gap_bridge_lengths_m": f"{gap_m:.6f}",
        })

    requested_stoplines = [
        (segment, row)
        for segment in implemented for row in segment.get("h5_stoplines", [])
    ]
    if requested_stoplines:
        if not h5_sha:
            raise ValueError("H5 stopline selection requires a pinned Won-tae H5 source")
        h5_stoplines = {
            row["source_id"]: row for row in _load_won_tae_h5_stoplines(h5_path)
        }
        for segment, selection in requested_stoplines:
            source_id = str(selection["source_id"])
            if source_id not in h5_stoplines:
                raise ValueError(f"H5 stopline is missing: {source_id}")
            source = h5_stoplines[source_id]
            _, _, match = route_index.query(
                source["center"], None,
                max_distance_m=float(selection["maximum_route_distance_m"]),
                max_heading_deg=180.0,
            )
            if match is None:
                raise ValueError(f"H5 stopline {source_id} is not near the competition route")
            route_s = float(route_stations[match])
            expected_route_s = float(selection["expected_route_s_m"])
            if abs(route_s - expected_route_s) > float(selection["route_s_tolerance_m"]):
                raise ValueError(f"H5 stopline {source_id} route station changed")
            target_link_id = str(selection["target_current_link_id"])
            target_link_ids = [target_link_id] + [
                str(value)
                for value in selection.get("additional_target_current_link_ids", [])
            ]
            feature = make_candidate_feature(
                feature_type="stopline",
                canonical_id=f"{config['map_id']}:stopline:{segment['id']}:{source_id}",
                points=source["points"],
                attributes={
                    "color": "white",
                    "pattern": "solid",
                    "line_group": "single",
                    "functional_role": "stopline",
                    "line_width_m": float(selection.get("line_width_m", 0.40)),
                    "curation_segment_id": str(segment["id"]),
                    "source_display": "won_tae_h5",
                    "traffic_light_id": str(source["traffic_light_id"]),
                },
                source_id=source_id,
                source_sha256=h5_sha,
                target_link_ids=target_link_ids,
                validation={
                    "human_reviewed": True,
                    "evidence_ids": list(segment.get("evidence", [])),
                    "route_s_m": route_s,
                    "h5_stopline_index": int(source["index"]),
                    "h5_length_m": float(source["length_m"]),
                    "selection_reason": str(selection["reason"]),
                },
            )
            feature["provenance"] = {
                "source_kind": "current_lidar_semantic",
                "source_id": source_id,
                "source_sha256": h5_sha,
                "transform": "won_tae_h5_stopline_vector_no_geometry_change",
            }
            feature["status"] = "reviewed"
            features.append(feature)
            stopline_inventory.append({
                "segment_id": segment["id"],
                "source_id": source_id,
                "current_link_id": ";".join(target_link_ids),
                "route_s_m": route_s,
                "length_m": float(source["length_m"]),
                "traffic_light_id": str(source["traffic_light_id"]),
                "reason": str(selection["reason"]),
            })

    from .adjacent_review import adjacent_markings
    adjacent = adjacent_markings(config, review, features, repo_root)
    features.extend(adjacent)
    for feature in adjacent:
        a=feature['attributes']
        inventory.append(dict(segment_id=a['curation_segment_id'],
            source_id=feature['provenance']['source_id'],run_index=0,color=a['color'],
            source_pattern=a['pattern'],output_pattern=a['pattern'],
            current_link_ids=';'.join(feature['topology']['current_link_ids']),
            route_s_min_m='',route_s_max_m='',attribute_override_ids='',
            source_display=a['source_display'],source_vector_indices='',gap_bridge_lengths_m=''))
    features.sort(key=lambda row: row["id"])
    document = {
        "schema_version": SCHEMA_VERSION,
        "map_id": config["map_id"],
        "frame": review["observed_document"]["frame"],
        "sources": {
            "curation_config": {
                "path": str(config_path.relative_to(repo_root)),
                "sha256": sha256_file(config_path),
            },
            "comparison_config": {
                "path": str(comparison_path.relative_to(repo_root)),
                "sha256": sha256_file(comparison_path),
            },
            "legacy_lane_boundary_set": {"sha256": legacy_sha},
            "won_tae_h5": {"sha256": h5_sha} if h5_sha else {},
            "preplan_hybrid_candidate": {
                "path": str(review["source_paths"]["legacy_adopted_map"].relative_to(repo_root)),
                "sha256": review["source_hashes"]["legacy_adopted_map"],
            } if guide_segments or review_marking_segments else {},
            "current_link_set": {
                "path": str(current_link_path.relative_to(repo_root)),
                "sha256": current_link_sha,
            } if current_link_path is not None else {},
        },
        "build": {
            "build_id": config["build_id"],
            "operational": False,
            "partial_candidate": True,
            "implemented_segment_ids": [str(row["id"]) for row in implemented],
            "stoplines_included": bool(stopline_inventory),
            "unimplemented_segments_emit_no_geometry": True,
        },
        "features": features,
    }
    validate_canonical_document(document)
    output_dir.mkdir(parents=True, exist_ok=True)
    _write_json(output_dir / "route_curated_candidate_map.json", document)
    _write_csv(output_dir / "selected_markings.csv", inventory, [
        "segment_id", "source_id", "run_index", "color", "source_pattern",
        "output_pattern", "current_link_ids", "route_s_min_m", "route_s_max_m",
        "attribute_override_ids", "source_display", "source_vector_indices",
        "gap_bridge_lengths_m",
    ])
    _write_csv(output_dir / "selected_stoplines.csv", stopline_inventory, [
        "segment_id", "source_id", "current_link_id", "route_s_m", "length_m",
        "traffic_light_id", "reason",
    ])
    for segment in implemented:
        selected_features = [
            feature for feature in features
            if feature["attributes"]["curation_segment_id"] == segment["id"]
        ]
        _render_overlay(
            output_dir / "overlays" / f"{segment['id']}.png",
            route_samples, route_stations, segment, selected_features,
            config["visualization"],
        )
    summary = {
        "build_id": config["build_id"],
        "map_id": config["map_id"],
        "operational": False,
        "partial_candidate": True,
        "implemented_segments": [str(row["id"]) for row in implemented],
        "planned_segment_count": sum(row.get("status") == "planned" for row in config["segments"]),
        "selected_marking_count": sum(row["feature_type"] == "lane_marking" for row in features),
        "selected_stopline_count": len(stopline_inventory),
        "selected_by_color": dict(sorted(Counter(
            row["attributes"]["color"] for row in features
            if row["feature_type"] == "lane_marking"
        ).items())),
        "selected_by_pattern": dict(sorted(Counter(
            row["attributes"]["pattern"] for row in features
            if row["feature_type"] == "lane_marking"
        ).items())),
        "attribute_override_count": sum(
            bool(row["validation"].get("attribute_override_ids"))
            or bool(row["validation"].get("correction_ids"))
            or bool(row["validation"].get("pattern_override_id"))
            for row in features if row["feature_type"] == "lane_marking"
        ),
        "stoplines_included": bool(stopline_inventory),
        "decision": (
            "only explicitly implemented review segments are emitted; "
            "every planned decision remains documentation-only"
        ),
    }
    _write_json(output_dir / "summary.json", summary)
    return summary
