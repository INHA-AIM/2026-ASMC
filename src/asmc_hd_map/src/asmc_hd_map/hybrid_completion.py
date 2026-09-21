"""Complete current-only gaps around a reviewed legacy HD-map candidate.

The legacy KATRI geometry remains the preferred painted-lane source in stable
regions.  This stage replaces only explicitly reviewed 2025 changes with
smooth current-link geometry.  Won-tae's observations are intentionally used
as width/normal-offset evidence, never copied as the canonical polyline.
"""

from __future__ import annotations

from collections import Counter
import copy
import csv
from pathlib import Path

import cv2
import numpy as np
import yaml

from .current_builder import _write_csv, _write_json
from .geometry import cumulative_distance, points_xyz, resample_polyline
from .io_mgeo import load_json, resolve_source, sha256_file, source_record
from .map_overlay import dashed_polyline_segments, lane_shape_group
from .schema import SCHEMA_VERSION, validate_canonical_document
from .spatial import GridIndex


CONFIG_SCHEMA = "asmc.hd_map.hybrid_completion_config/v1"


def _read_csv(path: Path) -> list[dict]:
    with path.open("r", encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))


def _feature_points(feature: dict) -> np.ndarray:
    return points_xyz((feature.get("geometry") or {}).get("coordinates", ()))


def _plain(points: np.ndarray) -> list[list[float]]:
    return [[float(value) for value in row] for row in np.asarray(points)]


def _true_runs(mask: np.ndarray) -> list[tuple[int, int]]:
    runs: list[tuple[int, int]] = []
    start = None
    for index, value in enumerate(np.asarray(mask, dtype=bool)):
        if value and start is None:
            start = index
        if start is not None and (not value or index == len(mask) - 1):
            end = index if not value else index + 1
            runs.append((start, end))
            start = None
    return runs


def _route_samples(path: Path, step_m: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    values = []
    with path.open("r", encoding="utf-8") as stream:
        for line in stream:
            raw = line.strip().replace(",", " ").split()
            if len(raw) >= 2:
                values.append([float(raw[0]), float(raw[1]), float(raw[2]) if len(raw) > 2 else 0.0])
    points, tangents = resample_polyline(values, step_m)
    return points, tangents, cumulative_distance(points)


def _profile_medians(rows: list[dict]) -> dict[str, float]:
    values: dict[str, list[float]] = {}
    for row in rows:
        if str(row.get("applied", "")).strip().lower() not in {"true", "1"}:
            continue
        parent_id = str(row["parent_boundary_id"])
        values.setdefault(parent_id, []).append(float(row["offset_m"]))
    return {
        parent_id: float(np.median(offsets))
        for parent_id, offsets in values.items() if offsets
    }


def _boundary_lookup(nominal: dict) -> tuple[dict[tuple[str, str], dict], dict[frozenset[str], dict]]:
    by_link_side: dict[tuple[str, str], dict] = {}
    by_pair: dict[frozenset[str], dict] = {}
    for feature in nominal["features"]:
        if feature["feature_type"] != "lane_boundary":
            continue
        sides = (feature.get("attributes") or {}).get("link_sides") or {}
        for link_id, side in sides.items():
            by_link_side[(str(link_id), str(side))] = feature
        links = [str(value) for value in feature.get("topology", {}).get("current_link_ids", [])]
        if len(links) == 2:
            by_pair[frozenset(links)] = feature
    return by_link_side, by_pair


def _nominal_half_width(feature: dict) -> float:
    attributes = feature.get("attributes") or {}
    widths = list(attributes.get("nominal_width_start_m") or []) + list(
        attributes.get("nominal_width_end_m") or []
    )
    finite = [float(value) for value in widths if np.isfinite(float(value))]
    return 0.5 * float(np.median(finite)) if finite else 1.75


def _link_side_widths(
    links: list[dict], by_link_side: dict[tuple[str, str], dict],
    profile_medians: dict[str, float], config: dict,
) -> tuple[dict[tuple[str, str], float], list[dict]]:
    minimum = float(config["minimum_half_width_m"])
    maximum = float(config["maximum_half_width_m"])
    mirror = bool(config["mirror_single_side_observation"])
    result: dict[tuple[str, str], float] = {}
    audit: list[dict] = []
    for link in links:
        link_id = str(link["topology"]["current_link_ids"][0])
        raw: dict[str, tuple[float, bool, str]] = {}
        for side in ("left", "right"):
            feature = by_link_side.get((link_id, side))
            if feature is None:
                attributes = link.get("attributes") or {}
                nominal_values = [
                    float(attributes.get("nominal_width_start_m", 3.5)),
                    float(attributes.get("nominal_width_end_m", 3.5)),
                ]
                nominal_half = 0.5 * float(np.median(nominal_values))
                parent_id = ""
            else:
                parent_id = str(feature["id"])
                nominal_half = _nominal_half_width(feature)
            correction = profile_medians.get(parent_id)
            if correction is None:
                width = nominal_half
            else:
                # A boundary polyline follows link direction, so its positive
                # normal is left.  Positive correction expands the left side
                # but moves the right boundary inward.
                width = nominal_half + correction if side == "left" else nominal_half - correction
            raw[side] = (float(np.clip(width, minimum, maximum)), correction is not None, parent_id)
        if mirror and raw["left"][1] != raw["right"][1]:
            observed_side = "left" if raw["left"][1] else "right"
            missing_side = "right" if observed_side == "left" else "left"
            raw[missing_side] = (raw[observed_side][0], False, raw[missing_side][2])
            mirrored_from = observed_side
        else:
            mirrored_from = ""
        for side in ("left", "right"):
            result[(link_id, side)] = raw[side][0]
            audit.append({
                "current_link_id": link_id,
                "side": side,
                "half_width_m": raw[side][0],
                "accepted_observation_on_side": raw[side][1],
                "mirrored_from_side": mirrored_from if not raw[side][1] else "",
                "parent_boundary_id": raw[side][2],
            })
    return result, audit


def _moving_average(values: np.ndarray, samples: int) -> np.ndarray:
    if samples <= 1 or len(values) < 3:
        return values.copy()
    samples = min(samples | 1, len(values) if len(values) % 2 else len(values) - 1)
    if samples <= 1:
        return values.copy()
    pad = samples // 2
    padded = np.pad(values, (pad, pad), mode="edge")
    return np.convolve(padded, np.ones(samples) / samples, mode="valid")


def _route_owner_lookup(current_links: list[dict], station: float) -> str:
    matches = []
    for feature in current_links:
        attributes = feature.get("attributes") or {}
        if "official_route" not in attributes.get("corridor_roles", []):
            continue
        start = float(attributes["route_s_min_m"])
        end = float(attributes["route_s_max_m"])
        if start - 0.6 <= station <= end + 0.6:
            matches.append((abs(0.5 * (start + end) - station), str(feature["topology"]["current_link_ids"][0])))
    if not matches:
        raise ValueError(f"no official-route link owns station {station:.3f}m")
    return min(matches)[1]


def _make_reconstructed_feature(
    *, map_id: str, feature_id: str, points: np.ndarray, link_ids: list[str],
    lane_color: str, geometry_status: str, source_hash: str, attributes: dict,
    transform: str = "current_route_normal_offset_with_observed_width_prior",
    source_kind: str = "current_mgeo", source_id: str | None = None,
) -> dict:
    feature_attributes = dict(attributes)
    lane_shape = str(feature_attributes.pop("lane_shape", "solid"))
    return {
        "id": f"{map_id}:lane_boundary:current_reconstructed:{feature_id}",
        "feature_type": "lane_boundary",
        "frame_id": "map",
        "geometry": {"type": "LineString", "coordinates": _plain(points)},
        "attributes": {
            "geometry_status": geometry_status,
            "lane_color": lane_color,
            "lane_shape": lane_shape,
            **feature_attributes,
        },
        "topology": {
            "current_link_ids": sorted(set(link_ids)),
            "binding_status": "current_boundary_owner",
        },
        "provenance": {
            "source_kind": source_kind,
            "source_id": source_id or "+".join(sorted(set(link_ids))),
            "source_sha256": source_hash,
            "transform": transform,
        },
        "validation": {
            "operational": False,
            "driving_review_required": True,
            "observed_geometry_copied": False,
        },
        "status": "candidate",
    }


def _is_direct_observation(row: dict) -> bool:
    for key in ("generated", "interpolated"):
        value = str(row.get(key, "0")).strip().lower()
        if value not in {"", "0", "0.0", "false"}:
            return False
    return True


def _observed_outer_midpoint_profile(
    rows: list[dict], interval: dict, route_index: GridIndex,
    route_points: np.ndarray, route_tangents: np.ndarray,
    route_station: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, dict]:
    """Return a smooth route-station/lateral profile from two road edges.

    The observations are never copied as canonical polylines.  Both outer
    edges are independently projected to the official route, binned in route
    station, and reduced with a median.  Their midpoint is the shared boundary
    of the two through lanes observed before the roundabout.
    """

    start = float(interval["start_s_m"])
    end = float(interval["end_s_m"])
    bin_m = float(interval["observation_station_bin_m"])
    if bin_m <= 0.0:
        raise ValueError("observation_station_bin_m must be positive")
    specifications = {
        "left": (
            str(interval["observation_left_source_file"]),
            str(interval["observation_left_color"]).upper(),
        ),
        "right": (
            str(interval["observation_right_source_file"]),
            str(interval["observation_right_color"]).upper(),
        ),
    }
    binned: dict[str, dict[int, list[float]]] = {"left": {}, "right": {}}
    matched_rows = {"left": 0, "right": 0}
    for row in rows:
        if not _is_direct_observation(row):
            continue
        for name, (source_file, lane_color) in specifications.items():
            if str(row.get("source_file", "")) != source_file:
                continue
            if str(row.get("lane_color", "")).upper() != lane_color:
                continue
            try:
                point = np.asarray([
                    float(row["lane_map_x"]), float(row["lane_map_y"]), 0.0,
                ], dtype=np.float64)
            except (KeyError, TypeError, ValueError):
                continue
            distance, _, match = route_index.query(point, max_distance_m=10.0)
            if match is None or distance > 10.0:
                continue
            station = float(route_station[match])
            if not start - bin_m <= station <= end + bin_m:
                continue
            normal = np.asarray([
                -route_tangents[match, 1], route_tangents[match, 0],
            ])
            lateral = float((point[:2] - route_points[match, :2]) @ normal)
            bin_index = int(np.floor((station - start) / bin_m))
            binned[name].setdefault(bin_index, []).append(lateral)
            matched_rows[name] += 1

    common = sorted(set(binned["left"]).intersection(binned["right"]))
    required = int(interval["minimum_common_observation_bins"])
    if len(common) < required:
        raise ValueError(
            f"{interval['name']} has only {len(common)} common observation bins; "
            f"expected at least {required}"
        )
    profile_station = np.asarray([
        start + (index + 0.5) * bin_m for index in common
    ], dtype=np.float64)
    left = np.asarray([
        np.median(binned["left"][index]) for index in common
    ], dtype=np.float64)
    right = np.asarray([
        np.median(binned["right"][index]) for index in common
    ], dtype=np.float64)
    raw_midpoint = 0.5 * (left + right)
    inward_correction = float(interval.get("observation_inward_correction_m", 0.0))
    if not np.isfinite(inward_correction) or inward_correction < 0.0:
        raise ValueError("observation_inward_correction_m must be finite and non-negative")
    midpoint = raw_midpoint + inward_correction
    if not (np.all(left > midpoint) and np.all(midpoint > right)):
        raise ValueError("observed outer-edge midpoint has invalid lateral ordering")
    if float(np.median(midpoint)) >= 0.0:
        raise ValueError("observed midpoint must lie on the route-right side")
    smooth_samples = max(1, int(round(
        float(interval.get("observation_smoothing_length_m", 5.0)) / bin_m
    )))
    midpoint = _moving_average(midpoint, smooth_samples)
    audit = {
        "observation_left_source_file": specifications["left"][0],
        "observation_right_source_file": specifications["right"][0],
        "observation_left_row_count": matched_rows["left"],
        "observation_right_row_count": matched_rows["right"],
        "observation_common_bin_count": len(common),
        "observed_left_lateral_median_m": float(np.median(left)),
        "observed_right_lateral_median_m": float(np.median(right)),
        "raw_derived_shared_lateral_median_m": float(np.median(raw_midpoint)),
        "driving_review_inward_correction_m": inward_correction,
        "derived_shared_lateral_median_m": float(np.median(midpoint)),
        "derived_two_lane_width_median_m": float(np.median(left - right)),
    }
    return profile_station, midpoint, audit


def _build_route_boundaries(
    route_points: np.ndarray, route_tangents: np.ndarray, route_station: np.ndarray,
    current_links: list[dict], widths: dict[tuple[str, str], float],
    config: dict, map_id: str, source_hash: str,
    route_index: GridIndex, observation_rows: list[dict], observation_hash: str,
) -> tuple[list[dict], list[dict]]:
    output: list[dict] = []
    audit: list[dict] = []
    sample_step = float(config["sample_step_m"])
    output_stride = max(1, int(round(float(config["output_step_m"]) / sample_step)))
    smooth_samples = max(1, int(round(float(config["width_smoothing_length_m"]) / sample_step)))
    for interval in config["intervals"]:
        mask = (
            (route_station >= float(interval["start_s_m"]))
            & (route_station <= float(interval["end_s_m"]))
        )
        indices = np.flatnonzero(mask)
        if len(indices) < 2:
            raise ValueError(f"route reconstruction interval is empty: {interval['name']}")
        points = route_points[indices]
        tangents = route_tangents[indices]
        stations = route_station[indices]
        owners = [_route_owner_lookup(current_links, float(value)) for value in stations]
        normals = np.column_stack((-tangents[:, 1], tangents[:, 0]))
        requested_sides = [str(value) for value in interval.get("sides", ("left", "right"))]
        if not requested_sides or any(value not in {"left", "right"} for value in requested_sides):
            raise ValueError(f"invalid route reconstruction sides: {requested_sides}")
        observation_audit = {}
        observation_profile = None
        if interval.get("width_source_mode") == "observed_outer_midpoint":
            profile_station, profile_lateral, observation_audit = _observed_outer_midpoint_profile(
                observation_rows, interval, route_index, route_points,
                route_tangents, route_station,
            )
            valid = (stations >= profile_station[0]) & (stations <= profile_station[-1])
            points = points[valid]
            tangents = tangents[valid]
            stations = stations[valid]
            normals = normals[valid]
            owners = [owner for owner, keep in zip(owners, valid) if keep]
            if len(points) < 2:
                raise ValueError(f"observation profile is empty: {interval['name']}")
            observation_profile = np.interp(stations, profile_station, profile_lateral)
        for side in requested_sides:
            sign = 1.0 if side == "left" else -1.0
            if observation_profile is not None:
                if side != "right":
                    raise ValueError("observed outer midpoint currently defines only a right boundary")
                # Keep driving-review insets within the same physically
                # plausible width contract as current-link-derived profiles.
                half_width = np.clip(
                    np.abs(observation_profile),
                    float(config["minimum_half_width_m"]),
                    float(config["maximum_half_width_m"]),
                )
                lateral = -half_width
                geometry = points.copy()
                geometry[:, :2] += lateral[:, None] * normals
                width_source = "won_tae_direct_outer_edges_midpoint"
                feature_source_hash = observation_hash
                transform = "current_route_normal_offset_from_observed_outer_edge_midpoint"
                feature_source_kind = "current_lidar_semantic"
                feature_source_id = "+".join((
                    str(interval["observation_left_source_file"]),
                    str(interval["observation_right_source_file"]),
                ))
            else:
                half_width = np.asarray([widths[(owner, side)] for owner in owners], dtype=np.float64)
                half_width = _moving_average(half_width, smooth_samples)
                geometry = points.copy()
                geometry[:, :2] += sign * half_width[:, None] * normals
                width_source = "current_link_nominal_plus_accepted_observation_median"
                feature_source_hash = source_hash
                transform = "current_route_normal_offset_with_observed_width_prior"
                feature_source_kind = "current_mgeo"
                feature_source_id = None
            geometry = geometry[::output_stride]
            if not np.array_equal(geometry[-1], points[-1]):
                geometry = np.vstack((geometry, points[-1] + np.r_[
                    sign * half_width[-1] * normals[-1], 0.0,
                ]))
            feature = _make_reconstructed_feature(
                map_id=map_id,
                feature_id=f"route:{interval['name']}:{side}",
                points=geometry,
                link_ids=owners,
                lane_color=str(interval[f"{side}_color"]),
                geometry_status="current_reconstructed_route_gap",
                source_hash=feature_source_hash,
                attributes={
                    "boundary_role": "route_edge",
                    "side": side,
                    "lane_shape": str(interval.get("lane_shape", "solid")),
                    "dash_length_m": float(interval.get("dash_length_m", 0.0)),
                    "dash_gap_m": float(interval.get("dash_gap_m", 0.0)),
                    "route_interval_name": str(interval["name"]),
                    "route_s_min_m": float(stations[0]),
                    "route_s_max_m": float(stations[-1]),
                    "width_source": width_source,
                    "reconstruction_reason": str(interval["reason"]),
                    **observation_audit,
                },
                transform=transform,
                source_kind=feature_source_kind,
                source_id=feature_source_id,
            )
            output.append(feature)
            audit.append({
                "reconstruction_id": feature["id"],
                "kind": "route_gap",
                "name": interval["name"],
                "side": side,
                "lane_color": interval[f"{side}_color"],
                "route_s_min_m": float(stations[0]),
                "route_s_max_m": float(stations[-1]),
                "length_m": float(cumulative_distance(geometry)[-1]),
                "half_width_min_m": float(np.min(half_width)),
                "half_width_median_m": float(np.median(half_width)),
                "half_width_max_m": float(np.max(half_width)),
                "current_link_ids": ";".join(sorted(set(owners))),
            })
    return output, audit


def _build_high_speed_boundaries(
    nominal: dict, by_pair: dict[frozenset[str], dict], profile_medians: dict[str, float],
    config: dict, map_id: str, source_hash: str,
) -> tuple[list[dict], list[dict]]:
    output: list[dict] = []
    audit: list[dict] = []
    for first, second in config["link_pairs"]:
        pair = frozenset((str(first), str(second)))
        if pair not in by_pair:
            raise ValueError(f"missing current shared boundary for {sorted(pair)}")
        parent = by_pair[pair]
        points, tangents = resample_polyline(_feature_points(parent), 0.5)
        correction = float(profile_medians.get(str(parent["id"]), 0.0))
        normals = np.column_stack((-tangents[:, 1], tangents[:, 0]))
        points[:, :2] += correction * normals
        feature = _make_reconstructed_feature(
            map_id=map_id,
            feature_id=f"high_speed_shared:{first}:{second}",
            points=points,
            link_ids=[str(first), str(second)],
            lane_color=str(config["lane_color"]),
            geometry_status="current_reconstructed_high_speed_lane1_shared",
            source_hash=source_hash,
            attributes={
                "boundary_role": "shared",
                "link_sides": copy.deepcopy(parent.get("attributes", {}).get("link_sides", {})),
                "parent_boundary_id": str(parent["id"]),
                "accepted_observation_median_offset_m": correction,
                "width_source": "current_shared_boundary_plus_accepted_observation_median",
            },
        )
        output.append(feature)
        audit.append({
            "reconstruction_id": feature["id"],
            "kind": "high_speed_lane1_shared",
            "name": f"{first}+{second}",
            "side": "shared",
            "lane_color": config["lane_color"],
            "route_s_min_m": "",
            "route_s_max_m": "",
            "length_m": float(cumulative_distance(points)[-1]),
            "half_width_min_m": "",
            "half_width_median_m": "",
            "half_width_max_m": "",
            "current_link_ids": ";".join(sorted(pair)),
        })
    return output, audit


def _replacement_mask(
    points: np.ndarray, tangents: np.ndarray,
    route_index: GridIndex, route_station: np.ndarray, route_intervals: list[dict],
    default_route_distance_m: float, rebuilt_index: GridIndex | None,
    rebuilt_distance_m: float,
) -> np.ndarray:
    replace = np.zeros(len(points), dtype=bool)
    maximum_route_distance = max(
        [default_route_distance_m]
        + [float(row.get("legacy_suppression_distance_m", default_route_distance_m)) for row in route_intervals]
    )
    for index, (point, tangent) in enumerate(zip(points, tangents)):
        distance, _, match = route_index.query(
            point, tangent, max_distance_m=maximum_route_distance, max_heading_deg=45.0,
        )
        if match is not None:
            station = float(route_station[match])
            route_point = route_index.points[match]
            route_tangent = route_index.tangents[match]
            normal = np.asarray([-route_tangent[1], route_tangent[0]])
            physical_side = "left" if float((point[:2] - route_point[:2]) @ normal) >= 0.0 else "right"
            for row in route_intervals:
                if not float(row["start_s_m"]) <= station <= float(row["end_s_m"]):
                    continue
                limit = float(row.get("legacy_suppression_distance_m", default_route_distance_m))
                sides = {str(value) for value in row.get("sides", ("left", "right"))}
                if distance <= limit and physical_side in sides:
                    replace[index] = True
                    break
            if replace[index]:
                continue
        if rebuilt_index is not None:
            distance, _, match = rebuilt_index.query(
                point, tangent, max_distance_m=rebuilt_distance_m, max_heading_deg=35.0,
            )
            if match is not None and distance <= rebuilt_distance_m:
                replace[index] = True
    return replace


def _clip_legacy_features(
    legacy_features: list[dict], route_index: GridIndex, route_station: np.ndarray,
    route_config: dict, high_speed_features: list[dict], high_speed_config: dict,
) -> tuple[list[dict], dict]:
    rebuilt_points: list[np.ndarray] = []
    rebuilt_tangents: list[np.ndarray] = []
    for feature in high_speed_features:
        points, tangents = resample_polyline(_feature_points(feature), 0.5)
        rebuilt_points.extend(points)
        rebuilt_tangents.extend(tangents)
    rebuilt_index = (
        GridIndex(np.asarray(rebuilt_points), np.asarray(rebuilt_tangents), cell_size_m=2.0)
        if rebuilt_points else None
    )
    output: list[dict] = []
    removed_points = 0
    split_features = 0
    removed_features = 0
    for feature in legacy_features:
        points, tangents = resample_polyline(_feature_points(feature), 0.5)
        replace = _replacement_mask(
            points, tangents, route_index, route_station, route_config["intervals"],
            float(route_config["legacy_suppression_distance_m"]), rebuilt_index,
            float(high_speed_config["legacy_replacement_distance_m"]),
        )
        removed_points += int(np.count_nonzero(replace))
        runs = []
        for start, end in _true_runs(~replace):
            run = points[start:end]
            if len(run) >= 2 and cumulative_distance(run)[-1] >= 2.0:
                runs.append(run)
        if not runs:
            removed_features += 1
            continue
        if len(runs) != 1 or np.any(replace):
            split_features += 1
        for index, run in enumerate(runs):
            copied = copy.deepcopy(feature)
            copied["id"] = f"{feature['id']}:hybrid_clip:{index:02d}"
            copied["geometry"]["coordinates"] = _plain(run)
            copied.setdefault("validation", {})["hybrid_replacement_clipped"] = bool(np.any(replace))
            output.append(copied)
    return output, {
        "legacy_input_feature_count": len(legacy_features),
        "legacy_output_feature_count": len(output),
        "legacy_fully_removed_feature_count": removed_features,
        "legacy_split_or_clipped_feature_count": split_features,
        "legacy_sample_points_removed": removed_points,
    }


def _filter_stoplines(stoplines: list[dict], rejected: dict[str, str]) -> tuple[list[dict], list[dict]]:
    kept, audit = [], []
    for feature in stoplines:
        route_nodes = [str(value) for value in feature.get("attributes", {}).get("route_node_ids", [])]
        matched = {node: rejected[node] for node in route_nodes if node in rejected}
        if matched:
            audit.append({
                "stopline_id": feature["id"],
                "route_s_m": feature.get("attributes", {}).get("route_s_m"),
                "route_node_ids": ";".join(route_nodes),
                "decision": "rejected_not_painted",
                "reason": ";".join(f"{key}:{value}" for key, value in sorted(matched.items())),
            })
        else:
            kept.append(feature)
            audit.append({
                "stopline_id": feature["id"],
                "route_s_m": feature.get("attributes", {}).get("route_s_m"),
                "route_node_ids": ";".join(route_nodes),
                "decision": "kept_candidate",
                "reason": "not_rejected_by_driving_review",
            })
    return kept, audit


def _draw_polyline(
    image: np.ndarray, points: np.ndarray, transform, color, thickness: int,
    *, dashed: bool = False, dash_length_m: float = 3.0, dash_gap_m: float = 3.0,
) -> None:
    if len(points) < 2:
        return
    if not dashed:
        cv2.polylines(image, [transform(points)], False, color, thickness, cv2.LINE_AA)
        return
    for segment in dashed_polyline_segments([points], dash_length_m, dash_gap_m):
        cv2.line(
            image, tuple(transform(segment)[0]), tuple(transform(segment)[1]),
            color, thickness, cv2.LINE_AA,
        )


def _render_overlay(
    path: Path, title: str, bounds: tuple[np.ndarray, np.ndarray],
    current_links: list[dict], legacy: list[dict], rebuilt: list[dict],
    stoplines: list[dict], route: np.ndarray, visual: dict,
) -> dict:
    low, high = bounds
    width, height = int(visual["width_px"]), int(visual["height_px"])
    if "high_speed" in title:
        height = max(height, 1600)
    padding = int(visual["padding_px"])
    span = np.maximum(high - low, 1.0)
    scale = min((width - 2 * padding) / span[0], (height - 2 * padding) / span[1])
    center = 0.5 * (low + high)
    draw_span = np.asarray([(width - 2 * padding) / scale, (height - 2 * padding) / scale])
    draw_low, draw_high = center - draw_span / 2.0, center + draw_span / 2.0
    image = np.full((height, width, 3), 20, dtype=np.uint8)

    def transform(points: np.ndarray) -> np.ndarray:
        pixels = np.empty((len(points), 2), dtype=np.int32)
        pixels[:, 0] = np.round(padding + (points[:, 0] - draw_low[0]) * scale).astype(np.int32)
        pixels[:, 1] = np.round(padding + (draw_high[1] - points[:, 1]) * scale).astype(np.int32)
        return pixels

    def intersects(points: np.ndarray) -> bool:
        return bool(len(points) and np.all(np.max(points[:, :2], axis=0) >= low) and np.all(np.min(points[:, :2], axis=0) <= high))

    counts = Counter()
    for feature in current_links:
        points = _feature_points(feature)
        if intersects(points):
            _draw_polyline(image, points, transform, (180, 105, 25), 1)
            counts["current_link"] += 1
    for feature in legacy:
        points = _feature_points(feature)
        if intersects(points):
            color = (245, 245, 245) if feature.get("attributes", {}).get("lane_color") == "white" else (20, 220, 245)
            attributes = feature.get("attributes", {})
            mgeo = attributes.get("mgeo") or {}
            _draw_polyline(
                image, points, transform, color, 2,
                dashed=lane_shape_group(feature) == "broken",
                dash_length_m=float(attributes.get("dash_length_m", mgeo.get("dash_interval_L1", 3.0))),
                dash_gap_m=float(attributes.get("dash_gap_m", mgeo.get("dash_interval_L2", 3.0))),
            )
            counts["legacy_kept"] += 1
    for feature in rebuilt:
        points = _feature_points(feature)
        if intersects(points):
            color = (255, 255, 255) if feature.get("attributes", {}).get("lane_color") == "white" else (0, 235, 255)
            attributes = feature.get("attributes", {})
            dashed = lane_shape_group(feature) == "broken"
            dash_length = float(attributes.get("dash_length_m", 3.0))
            dash_gap = float(attributes.get("dash_gap_m", 3.0))
            _draw_polyline(
                image, points, transform, (210, 60, 210), 5,
                dashed=dashed, dash_length_m=dash_length, dash_gap_m=dash_gap,
            )
            _draw_polyline(
                image, points, transform, color, 3,
                dashed=dashed, dash_length_m=dash_length, dash_gap_m=dash_gap,
            )
            counts["current_reconstructed"] += 1
    for feature in stoplines:
        points = _feature_points(feature)
        if intersects(points):
            _draw_polyline(image, points, transform, (20, 30, 255), 4)
            counts["reviewed_stopline"] += 1
    if intersects(route):
        _draw_polyline(image, route, transform, (45, 210, 45), 2)
    legend = [
        title,
        "cyan=current link center  green=organizer route",
        "white/yellow=kept legacy  magenta outline=current reconstructed",
        "red=painted-stopline candidate after driving-review rejection",
        "CANDIDATE: operational=false; reconstructed segments need final driving review",
    ]
    for index, line in enumerate(legend):
        cv2.putText(image, line, (14, 22 + 19 * index), cv2.FONT_HERSHEY_SIMPLEX, 0.43, (238, 238, 238), 1, cv2.LINE_AA)
    path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(path), image):
        raise IOError(f"failed to write overlay: {path}")
    return dict(sorted(counts.items()))


def build_hybrid_completion(config_path: Path, repo_root: Path, output_dir: Path) -> dict:
    with config_path.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream) or {}
    if config.get("schema_version") != CONFIG_SCHEMA:
        raise ValueError("unsupported hybrid completion config schema")
    if config["promotion_policy"].get("operational") is not False:
        raise ValueError("hybrid completion must remain non-operational")

    paths = {key: resolve_source(repo_root, value) for key, value in config["sources"].items()}
    parent = load_json(paths["legacy_adopted_map"])
    nominal = load_json(paths["nominal_map"])
    profiles = _read_csv(paths["boundary_offset_profiles"])
    pre_roundabout_observations = _read_csv(paths["pre_roundabout_observations"])
    route_config = config["route_reconstruction"]
    route_points, route_tangents, route_station = _route_samples(
        paths["competition_route"], float(route_config["sample_step_m"]),
    )
    route_index = GridIndex(route_points, route_tangents, cell_size_m=3.0)
    current_links = [feature for feature in parent["features"] if feature["feature_type"] == "link_centerline"]
    stoplines = [feature for feature in parent["features"] if feature["feature_type"] == "stopline"]
    legacy = [feature for feature in parent["features"] if feature["feature_type"] == "lane_boundary"]
    by_link_side, by_pair = _boundary_lookup(nominal)
    medians = _profile_medians(profiles)
    widths, width_audit = _link_side_widths(
        current_links, by_link_side, medians, route_config,
    )
    nominal_hash = sha256_file(paths["nominal_map"])
    pre_roundabout_observation_hash = sha256_file(paths["pre_roundabout_observations"])
    route_rebuilt, route_audit = _build_route_boundaries(
        route_points, route_tangents, route_station, current_links, widths,
        route_config, config["map_id"], nominal_hash, route_index,
        pre_roundabout_observations, pre_roundabout_observation_hash,
    )
    high_speed_rebuilt, high_speed_audit = _build_high_speed_boundaries(
        nominal, by_pair, medians, config["high_speed_missing_shared_boundaries"],
        config["map_id"], nominal_hash,
    )
    clipped_legacy, clipping_audit = _clip_legacy_features(
        legacy, route_index, route_station, route_config, high_speed_rebuilt,
        config["high_speed_missing_shared_boundaries"],
    )
    kept_stoplines, stopline_audit = _filter_stoplines(
        stoplines, config["stopline_review"]["rejected_route_node_ids"],
    )

    map_id = str(config["map_id"])
    static_features = []
    for feature in current_links + kept_stoplines:
        copied = copy.deepcopy(feature)
        copied["id"] = f"{map_id}:{str(feature['id']).split(':', 1)[-1]}"
        static_features.append(copied)
    for feature in clipped_legacy:
        feature["id"] = f"{map_id}:{str(feature['id']).split(':', 1)[-1]}"
    rebuilt = route_rebuilt + high_speed_rebuilt
    document = {
        "schema_version": SCHEMA_VERSION,
        "map_id": map_id,
        "frame": parent["frame"],
        "sources": {
            "parent_legacy_adopted_map": source_record(repo_root, config["sources"]["legacy_adopted_map"], paths["legacy_adopted_map"]),
            "current_nominal_map": source_record(repo_root, config["sources"]["nominal_map"], paths["nominal_map"]),
            "observed_boundary_offset_profiles": source_record(repo_root, config["sources"]["boundary_offset_profiles"], paths["boundary_offset_profiles"]),
            "pre_roundabout_observations": source_record(repo_root, config["sources"]["pre_roundabout_observations"], paths["pre_roundabout_observations"]),
            "competition_route": source_record(repo_root, config["sources"]["competition_route"], paths["competition_route"]),
        },
        "build": {
            "build_id": config["build_id"],
            "stage": "hybrid_legacy_current_completion_candidate",
            "operational": False,
            "current_link_node_topology_authority": True,
            "legacy_stable_region_geometry_authority": True,
            "observed_geometry_is_width_evidence_only": True,
            "legacy_fragments_near_reconstructed_geometry_removed": True,
            "current_stopline_candidates_manually_filtered": True,
        },
        "features": static_features + clipped_legacy + rebuilt,
        "review_layers": copy.deepcopy(parent.get("review_layers") or {}),
    }
    validate_canonical_document(document)
    _write_json(output_dir / "hybrid_candidate_map.json", document)
    _write_csv(output_dir / "reconstruction_audit.csv", route_audit + high_speed_audit, [
        "reconstruction_id", "kind", "name", "side", "lane_color",
        "route_s_min_m", "route_s_max_m", "length_m", "half_width_min_m",
        "half_width_median_m", "half_width_max_m", "current_link_ids",
    ])
    _write_csv(output_dir / "link_width_prior_audit.csv", width_audit, [
        "current_link_id", "side", "half_width_m", "accepted_observation_on_side",
        "mirrored_from_side", "parent_boundary_id",
    ])
    _write_csv(output_dir / "stopline_review.csv", stopline_audit, [
        "stopline_id", "route_s_m", "route_node_ids", "decision", "reason",
    ])

    margin = float(config["visualization"]["corridor_margin_m"])
    overlay_ranges = {
        "obstacle_bypass_2025": (190.0, 455.0),
        "pre_roundabout_observed_right_2025": (465.0, 565.0),
        "post_dynamic_right_turn_broken_2025": (535.0, 615.0),
        "roundabout_exit_to_high_speed_2025": (585.0, 680.0),
        "high_speed_entry_straight_2025": (900.0, 1000.0),
        "high_speed_lane1_shared_completion": (930.0, 1782.0),
        "high_speed_01_north": (930.0, 1119.0),
        "high_speed_02_middle_north": (1118.0, 1287.0),
        "high_speed_03_middle_south": (1286.0, 1519.0),
        "high_speed_04_south": (1518.0, 1742.0),
        "high_speed_safety_zone_observed_right_2025": (1575.0, 1760.0),
        "high_speed_05_exit": (1741.0, 1782.0),
        "tollgate_to_shadow_2025": (1760.0, 1900.0),
        "shadow_stopline_review": (1850.0, 2140.0),
        "full_corridor": (0.0, float(route_station[-1])),
    }
    overlays = {}
    for name, (start, end) in overlay_ranges.items():
        selected = route_points[(route_station >= start) & (route_station <= end)]
        if not len(selected):
            continue
        low = np.min(selected[:, :2], axis=0) - margin
        high = np.max(selected[:, :2], axis=0) + margin
        overlays[name] = {
            "route_s_m": [start, end],
            "bounds_xy": [low.tolist(), high.tolist()],
            "layer_counts": _render_overlay(
                output_dir / "overlays" / f"{name}.png", name, (low, high),
                current_links, clipped_legacy, rebuilt, kept_stoplines, route_points,
                config["visualization"],
            ),
        }

    summary = {
        "build_id": config["build_id"],
        "map_id": map_id,
        "status": "candidate_hybrid_completed_non_operational",
        "operational": False,
        "counts": {
            "current_link_center_count": len(current_links),
            "legacy_lane_boundary_count_after_clipping": len(clipped_legacy),
            "route_reconstructed_boundary_count": len(route_rebuilt),
            "high_speed_missing_shared_boundary_count": len(high_speed_rebuilt),
            "stopline_input_count": len(stoplines),
            "stopline_kept_count": len(kept_stoplines),
            "stopline_rejected_count": len(stoplines) - len(kept_stoplines),
        },
        "clipping": clipping_audit,
        "rejected_stoplines": [row for row in stopline_audit if row["decision"] != "kept_candidate"],
        "route_reconstruction_intervals": route_config["intervals"],
        "high_speed_reconstructed_link_pairs": config["high_speed_missing_shared_boundaries"]["link_pairs"],
        "overlays": overlays,
        "decision": (
            "stable old-map paint geometry is retained; reviewed 2025 route gaps and the "
            "missing high-speed lane-1 shared boundary are rebuilt from current topology "
            "using observations only as robust width evidence, including the reviewed "
            "pre-roundabout two-lane outer-edge midpoint"
        ),
    }
    _write_json(output_dir / "summary.json", summary)
    return summary
