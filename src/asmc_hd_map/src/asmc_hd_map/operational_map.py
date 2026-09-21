"""Build the reviewable K-City 2025 vector map and static surface layers.

The builder deliberately separates four contracts:

* current MGeo owns topology and link IDs;
* Won-tae's direct 2025 observations anchor lateral geometry and width;
* legacy KATRI markings contribute shape/paint semantics only after a local
  observation gate;
* rasters are disposable products derived from the vector document.

The output remains ``operational=false`` until route-by-route driving review.
"""

from __future__ import annotations

from collections import Counter, defaultdict
import csv
from io import BytesIO
import json
import math
from pathlib import Path
import zipfile
from typing import Iterable

import cv2
import numpy as np
import yaml

from .current_builder import _plain, _write_csv, _write_json
from .geometry import cumulative_distance, points_xyz, quantile_or_none, resample_polyline
from .io_mgeo import load_json, resolve_source, sha256_file, source_record
from .observed import PolylineProjectionIndex, interpolate_profile
from .schema import SCHEMA_VERSION, validate_canonical_document


CONFIG_SCHEMA = "asmc.hd_map.operational_map_config/v1"


def _read_csv(path: Path) -> list[dict]:
    with path.open("r", encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))


def _feature_points(feature: dict) -> np.ndarray:
    geometry = feature.get("geometry") or {}
    coordinates = geometry.get("coordinates") or []
    if geometry.get("type") == "Polygon" and coordinates:
        coordinates = coordinates[0]
    return points_xyz(coordinates)


def _retag_id(raw: str, map_id: str, feature_type: str | None = None) -> str:
    parts = str(raw).split(":")
    if feature_type and feature_type in parts:
        suffix = ":".join(parts[parts.index(feature_type):])
    else:
        suffix = str(raw).split(":", 1)[-1]
    return f"{map_id}:{suffix}"


def _verify_file(repo_root: Path, record: dict) -> tuple[Path, dict]:
    path = resolve_source(repo_root, str(record["path"]))
    if not path.is_file():
        raise FileNotFoundError(path)
    actual = sha256_file(path)
    expected = str(record["sha256"])
    if actual != expected:
        raise ValueError(f"source hash changed for {record['path']}: {actual} != {expected}")
    result = source_record(repo_root, str(record["path"]), path)
    if record.get("role"):
        result["role"] = str(record["role"])
    return path, result


def _verified_sources(config: dict, repo_root: Path) -> tuple[dict[str, Path], dict]:
    paths: dict[str, Path] = {}
    records: dict[str, dict] = {}
    for name, record in sorted((config.get("sources") or {}).items()):
        path, result = _verify_file(repo_root, record)
        paths[name] = path
        records[name] = result
    snapshot_records = []
    for record in config["source_snapshot"]["files"]:
        _, result = _verify_file(repo_root, record)
        result["path"] = str(record["path"])
        snapshot_records.append(result)
    snapshot = {
        "git_commit": str(config["source_snapshot"]["git_commit"]),
        "file_count": len(snapshot_records),
        "files": snapshot_records,
    }
    return paths, {"pipeline_sources": records, "won_tae_snapshot": snapshot}


def _profile_ranges(stations: np.ndarray, supported: np.ndarray, max_gap_m: float) -> list[list[float]]:
    ranges: list[list[float]] = []
    start: float | None = None
    previous: float | None = None
    for station, valid in zip(stations, supported):
        value = float(station)
        if valid and start is None:
            start = value
        if start is not None and (
            not valid or (previous is not None and value - previous > max_gap_m + 1e-9)
        ):
            ranges.append([start, float(previous if previous is not None else start)])
            start = value if valid else None
        previous = value
    if start is not None and previous is not None:
        ranges.append([start, float(previous)])
    return ranges


def _tangents(points: np.ndarray) -> np.ndarray:
    if len(points) < 2:
        return np.zeros((len(points), 2), dtype=np.float64)
    delta = np.zeros((len(points), 2), dtype=np.float64)
    delta[0] = points[1, :2] - points[0, :2]
    delta[-1] = points[-1, :2] - points[-2, :2]
    if len(points) > 2:
        delta[1:-1] = points[2:, :2] - points[:-2, :2]
    norm = np.linalg.norm(delta, axis=1)
    valid = norm > 1e-9
    delta[valid] /= norm[valid, None]
    return delta


def _apply_variable_normal_offset(points: np.ndarray, offsets: np.ndarray) -> np.ndarray:
    tangents = _tangents(points)
    normals = np.column_stack((-tangents[:, 1], tangents[:, 0]))
    result = points.copy()
    result[:, :2] += offsets[:, None] * normals
    return result


def _load_offset_profiles(path: Path) -> dict[str, list[dict]]:
    grouped: dict[str, list[dict]] = defaultdict(list)
    for row in _read_csv(path):
        if str(row.get("applied", "")).lower() != "true":
            continue
        if str(row.get("conflict", "")).lower() == "true":
            continue
        grouped[str(row["parent_boundary_id"])].append({
            "station_m": float(row["station_m"]),
            "offset_m": float(row["offset_m"]),
            "support": int(row["support"]),
            "lane_color": str(row["lane_color"]).strip().lower(),
            "source_files": str(row["source_files"]),
            "source_kinds": str(row["source_kinds"]),
        })
    for rows in grouped.values():
        rows.sort(key=lambda row: row["station_m"])
    return grouped


def build_observation_anchored_boundaries(
    nominal_document: dict, profile_rows: dict[str, list[dict]], *, map_id: str,
    sample_step_m: float, max_gap_m: float, observed_source_hash: str,
) -> tuple[list[dict], list[dict]]:
    """Create continuous logical boundaries with observation-only local offsets."""

    features: list[dict] = []
    audit: list[dict] = []
    for parent in sorted(
        (feature for feature in nominal_document["features"] if feature["feature_type"] == "lane_boundary"),
        key=lambda feature: feature["id"],
    ):
        parent_id = str(parent["id"])
        sampled, _ = resample_polyline(_feature_points(parent), sample_step_m)
        stations = cumulative_distance(sampled)
        profile = profile_rows.get(parent_id, [])
        offsets = np.zeros(len(sampled), dtype=np.float64)
        supported = np.zeros(len(sampled), dtype=bool)
        compact = [{"station_m": row["station_m"], "offset_m": row["offset_m"]} for row in profile]
        for index, station in enumerate(stations):
            value = interpolate_profile(compact, float(station), max_gap_m)
            if value is not None:
                offsets[index] = value
                supported[index] = True
        adjusted = _apply_variable_normal_offset(sampled, offsets)
        ranges = _profile_ranges(stations, supported, max_gap_m)
        colors = Counter()
        source_files: set[str] = set()
        source_kinds: set[str] = set()
        for row in profile:
            if row["lane_color"] not in {"", "unknown"}:
                colors[row["lane_color"]] += int(row["support"])
            source_files.update(filter(None, row["source_files"].split(";")))
            source_kinds.update(filter(None, row["source_kinds"].split(";")))
        color = colors.most_common(1)[0][0] if colors else "unknown"
        canonical_id = _retag_id(parent_id, map_id, "lane_boundary")
        attributes = dict(parent.get("attributes") or {})
        attributes.update({
            "continuous_reference_geometry": True,
            "geometry_status": "observed_calibrated" if ranges else "link_nominal_fallback",
            "lane_color": color,
            "pattern": "unknown",
            "line_group": "unknown",
            "crossing_rule": "unknown",
            "marking_component_ids": [],
            "observed_station_ranges_m": ranges,
            "observed_fraction": float(np.mean(supported)) if len(supported) else 0.0,
            "parent_boundary_id": parent_id,
            "source_files": sorted(source_files),
        })
        feature = {
            "id": canonical_id,
            "feature_type": "lane_boundary",
            "frame_id": "map",
            "geometry": {"type": "LineString", "coordinates": adjusted.tolist()},
            "attributes": attributes,
            "topology": dict(parent.get("topology") or {}),
            "provenance": {
                "source_kind": "current_lidar_semantic" if ranges else "link_width_proxy",
                "source_id": "+".join(sorted(source_files)) if source_files else parent_id,
                "source_sha256": observed_source_hash if ranges else parent["provenance"]["source_sha256"],
                "transform": "station_robust_signed_normal_offset" if ranges else "unchanged_nominal_fallback",
            },
            "validation": {
                "operational": False,
                "direct_observation_anchor": bool(ranges),
                "source_kinds": sorted(source_kinds),
            },
            "status": "candidate",
        }
        features.append(feature)
        audit.append({
            "boundary_id": canonical_id,
            "parent_boundary_id": parent_id,
            "current_link_ids": ";".join(feature["topology"].get("current_link_ids", [])),
            "observed_fraction": attributes["observed_fraction"],
            "observed_range_count": len(ranges),
            "maximum_abs_offset_m": max((abs(row["offset_m"]) for row in profile), default=0.0),
            "lane_color": color,
            "source_files": ";".join(sorted(source_files)),
        })
    return features, audit


def _lane_semantics(feature: dict) -> tuple[str, str, str]:
    attributes = feature.get("attributes") or {}
    mgeo = attributes.get("mgeo") or {}
    raw_color = attributes.get("lane_color", mgeo.get("lane_color", "unknown"))
    colors = raw_color if isinstance(raw_color, list) else [raw_color]
    color_set = {str(value).strip().lower() for value in colors}
    color = next((value for value in ("yellow", "white", "blue") if value in color_set), "unknown")
    raw_shape = attributes.get("lane_shape", mgeo.get("lane_shape", "unknown"))
    shapes = raw_shape if isinstance(raw_shape, list) else [raw_shape]
    lowered = [str(value).strip().lower() for value in shapes]
    pattern = "broken" if any("broken" in value for value in lowered) else (
        "solid" if any("solid" in value for value in lowered) else "unknown"
    )
    line_group = "double" if len([value for value in lowered if value not in {"", "unknown"}]) > 1 else "single"
    return color, pattern, line_group


def _majority_match(
    points: np.ndarray, tangents: np.ndarray, index: PolylineProjectionIndex,
    owner_links: dict[str, set[str]], target_links: set[str], *,
    max_distance_m: float, max_heading_deg: float,
) -> tuple[str | None, list]:
    votes: Counter[str] = Counter()
    accepted = []
    for point, tangent in zip(points, tangents):
        candidates = index.query(
            point, tangent, max_distance_m=max_distance_m, max_heading_deg=max_heading_deg,
        )
        candidates = [item for item in candidates if not target_links or owner_links[item.owner_id] & target_links]
        if candidates:
            votes[candidates[0].owner_id] += 1
            accepted.append(candidates[0])
        else:
            accepted.append(None)
    if not votes:
        return None, accepted
    owner = min(votes, key=lambda value: (-votes[value], value))
    return owner, [item if item is not None and item.owner_id == owner else None for item in accepted]


def _align_template_to_anchor(
    feature: dict, anchor_index: PolylineProjectionIndex, anchor_links: dict[str, set[str]],
    config: dict, width_difference_m: float | None,
) -> tuple[np.ndarray, dict]:
    sampled, tangents = resample_polyline(_feature_points(feature), float(config["geometry"]["sample_step_m"]))
    target_links = set(feature.get("topology", {}).get("current_link_ids", []))
    owner, assignments = _majority_match(
        sampled, tangents, anchor_index, anchor_links, target_links,
        max_distance_m=float(config["legacy_gate"]["maximum_projection_distance_m"]),
        max_heading_deg=float(config["legacy_gate"]["maximum_projection_heading_deg"]),
    )
    matched = [(index, item) for index, item in enumerate(assignments) if item is not None]
    coverage = len(matched) / max(len(sampled), 1)
    pre_distances = np.asarray([item.distance_m for _, item in matched], dtype=np.float64)
    offsets = np.zeros(len(sampled), dtype=np.float64)
    support = np.zeros(len(sampled), dtype=bool)
    for index, item in matched:
        tangent = tangents[index]
        normal = np.asarray([-tangent[1], tangent[0]], dtype=np.float64)
        target = np.asarray(item.point_xyz[:2], dtype=np.float64)
        offsets[index] = float((target - sampled[index, :2]) @ normal)
        support[index] = True
    if np.any(support):
        # A 1 m robust median suppresses the raw observation wiggle while only
        # moving the legacy curve in its local normal direction.
        stations = cumulative_distance(sampled)
        bins = np.floor(stations / 1.0).astype(int)
        profile = []
        for bin_id in sorted(set(bins[support].tolist())):
            values = offsets[(bins == bin_id) & support]
            profile.append({
                "station_m": float(np.median(stations[(bins == bin_id) & support])),
                "offset_m": float(np.median(values)),
            })
        smoothed = np.zeros(len(sampled), dtype=np.float64)
        for index, station in enumerate(stations):
            value = interpolate_profile(profile, float(station), 2.0)
            if value is not None:
                smoothed[index] = value
        adjusted = _apply_variable_normal_offset(sampled, smoothed)
    else:
        adjusted = sampled
    adjusted_tangents = _tangents(adjusted)
    _, post_assignments = _majority_match(
        adjusted, adjusted_tangents, anchor_index, anchor_links, target_links,
        max_distance_m=float(config["legacy_gate"]["maximum_projection_distance_m"]),
        max_heading_deg=float(config["legacy_gate"]["maximum_projection_heading_deg"]),
    )
    post_matches = [item for item in post_assignments if item is not None]
    residual_p95 = quantile_or_none(
        np.asarray([item.distance_m for item in post_matches], dtype=np.float64), 0.95,
    )
    heading_p95 = quantile_or_none(
        np.asarray([item.heading_error_deg for item in post_matches], dtype=np.float64), 0.95,
    )
    gate = config["legacy_gate"]
    checks = {
        "coverage": coverage >= float(gate["minimum_observed_coverage"]),
        "lateral_residual": residual_p95 is not None and residual_p95 <= float(gate["maximum_lateral_residual_p95_m"]),
        "heading": heading_p95 is not None and heading_p95 <= float(gate["maximum_heading_difference_p95_deg"]),
        "topology_binding": bool(target_links),
    }
    # `width_difference_m` compares the observed lane width with the current
    # link's nominal/fallback width. It is useful for the lane-area audit but
    # is not an error of this individual, normal-aligned paint template.
    width_consistency = (
        width_difference_m is None
        or width_difference_m <= float(gate["maximum_width_difference_m"])
    )
    adopted = all(checks.values())
    metrics = {
        "anchor_boundary_id": owner,
        "sample_count": len(sampled),
        "matched_count": len(matched),
        "coverage": coverage,
        "pre_alignment_lateral_p95_m": quantile_or_none(pre_distances, 0.95),
        "lateral_residual_p95_m": residual_p95,
        "heading_difference_p95_deg": heading_p95,
        "width_difference_m": width_difference_m,
        "width_consistency_passed": width_consistency,
        "decision": "aligned_template" if adopted else "review_required",
        "failed_checks": ";".join(sorted(key for key, value in checks.items() if not value)),
    }
    return adjusted if adopted else sampled, metrics


def _width_profile_by_link(path: Path) -> dict[str, list[dict]]:
    grouped: dict[str, list[dict]] = defaultdict(list)
    for row in _read_csv(path):
        grouped[str(row["current_link_id"])].append({
            "station_m": float(row["link_station_m"]),
            "width_m": float(row["observed_width_m"]),
            "status": str(row["width_status"]),
        })
    for rows in grouped.values():
        rows.sort(key=lambda value: value["station_m"])
    return grouped


def _median_widths(width_profiles: dict[str, list[dict]]) -> dict[str, float]:
    return {
        link_id: float(np.median([row["width_m"] for row in rows]))
        for link_id, rows in width_profiles.items() if rows
    }


def _longest_supported_point_run(points: np.ndarray, assignments: list) -> np.ndarray:
    runs: list[tuple[int, int]] = []
    start = None
    for index, assignment in enumerate(assignments + [None]):
        if assignment is not None and start is None:
            start = index
        elif assignment is None and start is not None:
            runs.append((start, index))
            start = None
    if not runs:
        raise ValueError("reviewed marking has no samples on its reviewed logical boundary")
    begin, end = max(
        runs,
        key=lambda value: float(cumulative_distance(points[value[0]:value[1]])[-1]),
    )
    return points[begin:end]


def _extend_reviewed_marking_to_boundary_endpoint(
    points: np.ndarray, boundary: dict, endpoint: str, maximum_extension_m: float,
) -> tuple[np.ndarray, float]:
    """Close a short, explicitly reviewed marking endpoint to its logical boundary."""

    target_points = points_xyz(boundary["geometry"]["coordinates"])
    if len(points) < 2 or len(target_points) < 2:
        raise ValueError("reviewed marking extension requires two-point polylines")
    if endpoint not in {"start", "end"}:
        raise ValueError(f"unsupported logical boundary endpoint: {endpoint}")
    target = target_points[0 if endpoint == "start" else -1]
    first_gap = float(np.linalg.norm(points[0, :2] - target[:2]))
    last_gap = float(np.linalg.norm(points[-1, :2] - target[:2]))
    gap = min(first_gap, last_gap)
    if gap > float(maximum_extension_m) + 1e-9:
        raise ValueError(
            f"reviewed marking endpoint gap {gap:.3f}m exceeds {maximum_extension_m:.3f}m"
        )
    if gap <= 1e-9:
        return points, 0.0
    if first_gap < last_gap:
        return np.vstack((target, points)), gap
    return np.vstack((points, target)), gap


def build_lane_markings(
    hybrid_document: dict, logical_boundaries: list[dict], observed_document: dict,
    width_profiles: dict[str, list[dict]], config: dict, map_id: str,
) -> tuple[list[dict], list[dict], list[dict]]:
    anchors = [
        feature for feature in observed_document["features"]
        if feature["feature_type"] == "lane_boundary"
        and feature.get("attributes", {}).get("geometry_status") == "observed_calibrated"
    ]
    anchor_polylines = {feature["id"]: feature["geometry"]["coordinates"] for feature in anchors}
    anchor_index = PolylineProjectionIndex(anchor_polylines, cell_size_m=2.0)
    anchor_links = {
        feature["id"]: set(feature.get("topology", {}).get("current_link_ids", []))
        for feature in anchors
    }
    logical_index = PolylineProjectionIndex(
        {feature["id"]: feature["geometry"]["coordinates"] for feature in logical_boundaries},
        cell_size_m=2.0,
    )
    logical_links = {
        feature["id"]: set(feature.get("topology", {}).get("current_link_ids", []))
        for feature in logical_boundaries
    }
    logical_by_id = {feature["id"]: feature for feature in logical_boundaries}
    observed_width = _median_widths(width_profiles)
    markings: list[dict] = []
    review_required: list[dict] = []
    audit: list[dict] = []
    reviewed_markings = {
        str(key): dict(value)
        for key, value in config.get("driving_reviewed_markings", {}).items()
    }
    reviewed_intervals = {
        str(value["name"]): dict(value)
        for value in config.get("driving_reviewed_intervals", [])
    }
    for sequence, source in enumerate(sorted(
        (feature for feature in hybrid_document["features"] if feature["feature_type"] == "lane_boundary"),
        key=lambda feature: feature["id"],
    )):
        target_links = set(source.get("topology", {}).get("current_link_ids", []))
        link_width_values = [observed_width[link] for link in target_links if link in observed_width]
        width_difference = None
        if link_width_values:
            nominal_values = []
            for link_id in target_links:
                for boundary in logical_boundaries:
                    if link_id in boundary.get("topology", {}).get("current_link_ids", []):
                        nominal_values.extend(boundary.get("attributes", {}).get("nominal_width_start_m", []))
            nominal = float(np.median(nominal_values)) if nominal_values else 3.5
            width_difference = abs(float(np.median(link_width_values)) - nominal)

        source_kind = source.get("provenance", {}).get("source_kind")
        if source_kind == "legacy_katri":
            points, metrics = _align_template_to_anchor(
                source, anchor_index, anchor_links, config, width_difference,
            )
        else:
            points, tangents = resample_polyline(
                _feature_points(source), float(config["geometry"]["sample_step_m"]),
            )
            owner, assignments = _majority_match(
                points, tangents, anchor_index, anchor_links, target_links,
                max_distance_m=2.0, max_heading_deg=30.0,
            )
            matched = [item for item in assignments if item is not None]
            metrics = {
                "anchor_boundary_id": owner,
                "sample_count": len(points),
                "matched_count": len(matched),
                "coverage": len(matched) / max(len(points), 1),
                "pre_alignment_lateral_p95_m": None,
                "lateral_residual_p95_m": quantile_or_none(
                    np.asarray([item.distance_m for item in matched]), 0.95,
                ),
                "heading_difference_p95_deg": quantile_or_none(
                    np.asarray([item.heading_error_deg for item in matched]), 0.95,
                ),
                "width_difference_m": width_difference,
                "decision": "current_2025_geometry",
                "failed_checks": "",
            }
        point_tangents = _tangents(points)
        logical_owner, logical_assignments = _majority_match(
            points, point_tangents, logical_index, logical_links, target_links,
            max_distance_m=2.0, max_heading_deg=30.0,
        )
        logical_matches = [item for item in logical_assignments if item is not None]
        binding_p95 = quantile_or_none(
            np.asarray([item.distance_m for item in logical_matches]), 0.95,
        )
        if binding_p95 is None or binding_p95 > 0.75:
            logical_owner = None
        color, pattern, line_group = _lane_semantics(source)
        legacy_source_id = str(source.get("provenance", {}).get("source_id") or "")
        driving_review = reviewed_markings.get(legacy_source_id)
        if driving_review is not None:
            interval = reviewed_intervals.get(str(driving_review["interval"]))
            if interval is None:
                raise ValueError(f"unknown driving-review interval: {driving_review['interval']}")
            reviewed_lane_id = str(interval["lane_id"])
            reviewed_side = str(driving_review["side"])
            if logical_owner is None:
                raise ValueError(
                    f"reviewed marking has no logical boundary owner: {legacy_source_id}"
                )
            logical_sides = logical_by_id[logical_owner].get("attributes", {}).get("link_sides", {})
            if str(logical_sides.get(reviewed_lane_id)) != reviewed_side:
                raise ValueError(
                    f"reviewed marking {legacy_source_id} does not bind to "
                    f"{reviewed_lane_id}:{reviewed_side}"
                )
            original_review_count = len(points)
            if bool(driving_review.get("clip_to_reviewed_lane", False)):
                points = _longest_supported_point_run(points, logical_assignments)
            color = str(driving_review.get("color", color))
            pattern = str(driving_review.get("pattern", pattern))
            metrics["automatic_decision"] = metrics["decision"]
            metrics["decision"] = "driving_review_approved"
            metrics["driving_review_interval"] = str(driving_review["interval"])
            metrics["reviewed_lane_id"] = reviewed_lane_id
            metrics["pre_review_sample_count"] = original_review_count
            metrics["reviewed_sample_count"] = len(points)
        else:
            metrics["automatic_decision"] = metrics["decision"]
            metrics["driving_review_interval"] = ""
            metrics["reviewed_lane_id"] = ""
            metrics["pre_review_sample_count"] = len(points)
            metrics["reviewed_sample_count"] = len(points)
        start_extension_m = 0.0
        end_extension_m = 0.0
        if driving_review is not None:
            maximum_extension = float(driving_review.get("maximum_extension_m", 2.0))
            if bool(driving_review.get("extend_to_boundary_start", False)):
                points, start_extension_m = _extend_reviewed_marking_to_boundary_endpoint(
                    points, logical_by_id[logical_owner], "start", maximum_extension,
                )
            if bool(driving_review.get("extend_to_boundary_end", False)):
                points, end_extension_m = _extend_reviewed_marking_to_boundary_endpoint(
                    points, logical_by_id[logical_owner], "end", maximum_extension,
                )
        metrics["reviewed_start_extension_m"] = start_extension_m
        metrics["reviewed_end_extension_m"] = end_extension_m
        geometry_status = str(source.get("attributes", {}).get("geometry_status", "unknown"))
        if driving_review is not None:
            geometry_status = "live_driving_reviewed_legacy"
        marking_id = f"{map_id}:lane_marking:{sequence:04d}"
        marking_topology = dict(source.get("topology") or {})
        if driving_review is not None:
            marking_topology["current_link_ids"] = [metrics["reviewed_lane_id"]]
            marking_topology["binding_status"] = "current_boundary_owner"
        feature = {
            "id": marking_id,
            "feature_type": "lane_marking",
            "frame_id": "map",
            "geometry": {"type": "LineString", "coordinates": points.tolist()},
            "attributes": {
                "color": color,
                "pattern": pattern,
                "line_group": line_group,
                "functional_role": "center_or_edge" if color == "yellow" else "divider_or_edge",
                "geometry_status": geometry_status,
                "logical_boundary_id": logical_owner,
                "observed_station_range": None,
                "legacy_source_id": legacy_source_id if source_kind == "legacy_katri" else None,
                "driving_review_interval": metrics["driving_review_interval"] or None,
            },
            "topology": marking_topology,
            "provenance": dict(source.get("provenance") or {}),
            "validation": {
                "operational": False,
                "geometry_anchor_decision": metrics["decision"],
                "logical_boundary_binding_p95_m": binding_p95,
                "legacy_gate": metrics if source_kind == "legacy_katri" else None,
                "driving_review_approved": driving_review is not None,
            },
            "status": "candidate",
        }
        include_in_candidate = (
            source_kind != "legacy_katri"
            or metrics["decision"] in {"aligned_template", "driving_review_approved"}
        )
        if include_in_candidate:
            markings.append(feature)
        else:
            feature["attributes"]["geometry_status"] = "legacy_template_review_required"
            feature["validation"]["excluded_from_candidate_geometry"] = True
            review_required.append(feature)
        if include_in_candidate and logical_owner is not None:
            logical_by_id[logical_owner]["attributes"]["marking_component_ids"].append(marking_id)
        audit.append({
            "marking_id": marking_id,
            "source_feature_id": source["id"],
            "source_kind": source_kind,
            "current_link_ids": ";".join(sorted(target_links)),
            "color": color,
            "pattern": pattern,
            "line_group": line_group,
            "logical_boundary_id": logical_owner,
            "logical_binding_p95_m": binding_p95,
            **metrics,
        })

    marking_by_id = {feature["id"]: feature for feature in markings}
    for boundary in logical_boundaries:
        components = [
            marking_by_id[value] for value in boundary["attributes"]["marking_component_ids"]
            if value in marking_by_id
        ]
        colors = Counter(item["attributes"]["color"] for item in components if item["attributes"]["color"] != "unknown")
        patterns = Counter(item["attributes"]["pattern"] for item in components if item["attributes"]["pattern"] != "unknown")
        groups = Counter(item["attributes"]["line_group"] for item in components if item["attributes"]["line_group"] != "unknown")
        color = colors.most_common(1)[0][0] if colors else boundary["attributes"].get("lane_color", "unknown")
        pattern = patterns.most_common(1)[0][0] if patterns else "unknown"
        line_group = groups.most_common(1)[0][0] if groups else "unknown"
        if color == "yellow":
            crossing = "prohibited"
        elif color == "white" and pattern == "broken":
            crossing = "permitted"
        elif color == "white" and pattern == "solid":
            crossing = "restricted"
        else:
            crossing = "unknown"
        boundary["attributes"].update({
            "lane_color": color,
            "pattern": pattern,
            "line_group": line_group,
            "crossing_rule": crossing,
        })
    return markings, audit, review_required


def _projection_for_owner(
    index: PolylineProjectionIndex, point: np.ndarray, tangent: np.ndarray, owner_id: str,
    max_distance_m: float = 8.0,
) -> np.ndarray | None:
    for projection in index.query(
        point, tangent, max_distance_m=max_distance_m, max_heading_deg=60.0,
    ):
        if projection.owner_id == owner_id:
            return np.asarray(projection.point_xyz, dtype=np.float64)
    return None


def _rolling_median(values: np.ndarray, radius: int) -> np.ndarray:
    result = values.copy()
    for index in range(len(values)):
        start, end = max(0, index - radius), min(len(values), index + radius + 1)
        finite = values[start:end][np.isfinite(values[start:end])]
        if len(finite):
            result[index] = float(np.median(finite))
    return result


def build_lane_areas(
    link_features: list[dict], boundaries: list[dict], width_profiles: dict[str, list[dict]],
    config: dict, map_id: str, composite_hash: str,
) -> tuple[list[dict], list[dict], dict[str, int]]:
    side_by_link: dict[str, dict[str, str]] = defaultdict(dict)
    for boundary in boundaries:
        for link_id, side in (boundary.get("attributes", {}).get("link_sides") or {}).items():
            side_by_link[str(link_id)][str(side)] = boundary["id"]
    boundary_index = PolylineProjectionIndex(
        {feature["id"]: feature["geometry"]["coordinates"] for feature in boundaries},
        cell_size_m=2.0,
    )
    boundary_by_id = {feature["id"]: feature for feature in boundaries}
    areas: list[dict] = []
    profile_rows: list[dict] = []
    quality_codes: dict[str, int] = {}
    fallback = float(config["geometry"]["fallback_width_m"])
    minimum = float(config["geometry"]["sanity_width_min_m"])
    maximum = float(config["geometry"]["sanity_width_max_m"])
    smooth_radius = max(1, int(round(
        0.5 * float(config["geometry"]["width_smoothing_length_m"])
        / float(config["geometry"]["sample_step_m"])
    )))
    for link in sorted(link_features, key=lambda feature: feature["topology"]["current_link_ids"][0]):
        link_id = str(link["topology"]["current_link_ids"][0])
        center, tangents = resample_polyline(
            _feature_points(link), float(config["geometry"]["sample_step_m"]),
        )
        if len(center) < 2:
            continue
        stations = cumulative_distance(center)
        sides = side_by_link.get(link_id, {})
        left_id, right_id = sides.get("left"), sides.get("right")
        observed_profile = width_profiles.get(link_id, [])
        compact_width = [
            {"station_m": row["station_m"], "offset_m": row["width_m"]}
            for row in observed_profile
        ]
        nominal = float(link.get("attributes", {}).get("nominal_width_start_m") or fallback)
        left_points: list[np.ndarray] = []
        right_points: list[np.ndarray] = []
        raw_widths: list[float] = []
        sample_status: list[str] = []
        for point, tangent, station in zip(center, tangents, stations):
            normal = np.asarray([-tangent[1], tangent[0]], dtype=np.float64)
            width = interpolate_profile(compact_width, float(station), 2.0)
            width = float(width if width is not None else nominal)
            left = _projection_for_owner(boundary_index, point, tangent, left_id) if left_id else None
            right = _projection_for_owner(boundary_index, point, tangent, right_id) if right_id else None
            if left is not None and float((left[:2] - point[:2]) @ normal) <= 0.0:
                left = None
            if right is not None and float((right[:2] - point[:2]) @ normal) >= 0.0:
                right = None
            if left is not None and right is not None:
                measured = float(np.linalg.norm(left[:2] - right[:2]))
                if minimum * 0.75 <= measured <= maximum * 1.25:
                    width = measured
                    status = "both_boundaries"
                else:
                    left = right = None
            if left is None and right is None:
                left = point.copy()
                right = point.copy()
                left[:2] += 0.5 * width * normal
                right[:2] -= 0.5 * width * normal
                status = "observed_width_centerline" if interpolate_profile(compact_width, float(station), 2.0) is not None else "link_nominal_fallback"
            elif left is not None and right is None:
                right = left.copy()
                right[:2] -= width * normal
                status = "left_boundary_plus_width"
            elif right is not None and left is None:
                left = right.copy()
                left[:2] += width * normal
                status = "right_boundary_plus_width"
            left_points.append(left)
            right_points.append(right)
            raw_widths.append(float(np.linalg.norm(left[:2] - right[:2])))
            sample_status.append(status)
        left_array = np.asarray(left_points)
        right_array = np.asarray(right_points)
        raw_width = np.asarray(raw_widths)
        is_transition = bool(link.get("topology", {}).get("is_merge_target") or link.get("topology", {}).get("is_split_source"))
        smoothed_width = raw_width if is_transition else _rolling_median(raw_width, smooth_radius)
        corrected_center = 0.5 * (left_array + right_array)
        polygon = np.vstack((left_array, right_array[::-1], left_array[:1]))
        observed_samples = sum(status != "link_nominal_fallback" for status in sample_status)
        both_samples = sum(status == "both_boundaries" for status in sample_status)
        if both_samples > 0 or observed_profile:
            quality_code = 3 if both_samples >= max(1, len(sample_status) // 2) else 2
            quality = "direct_observed" if quality_code == 3 else "partially_observed"
        else:
            quality_code, quality = 1, "link_nominal_fallback"
        sane = bool(np.all((smoothed_width >= minimum) & (smoothed_width <= maximum)))
        roles = list(link.get("attributes", {}).get("corridor_roles", []))
        lane_id = link_id
        area = {
            "id": f"{map_id}:lane_area:{lane_id}",
            "feature_type": "lane_area",
            "frame_id": "map",
            "geometry": {"type": "Polygon", "coordinates": [polygon.tolist()]},
            "attributes": {
                "lane_id": lane_id,
                "centerline": corrected_center.tolist(),
                "width_profile": [
                    {"station_m": float(station), "width_m": float(width)}
                    for station, width in zip(stations, smoothed_width)
                ],
                "left_boundary_id": left_id,
                "right_boundary_id": right_id,
                "geometry_quality": quality,
                "source_quality_code": quality_code,
                "route_maneuver_roles": roles,
                "route_s_min_m": link.get("attributes", {}).get("route_s_min_m"),
                "route_s_max_m": link.get("attributes", {}).get("route_s_max_m"),
                "width_sanity_passed": sane,
            },
            "topology": dict(link.get("topology") or {}),
            "provenance": {
                "source_kind": "current_lidar_semantic" if quality_code > 1 else "link_width_proxy",
                "source_id": link_id,
                "source_sha256": composite_hash,
                "transform": "boundary_midpoint_and_station_width_profile",
            },
            "validation": {
                "operational": False,
                "observed_sample_fraction": observed_samples / max(len(sample_status), 1),
                "width_sanity_passed": sane,
                "requires_review": not sane or quality_code == 1,
            },
            "status": "candidate",
        }
        areas.append(area)
        quality_codes[lane_id] = quality_code
        for station, raw, smooth, status in zip(stations, raw_width, smoothed_width, sample_status):
            profile_rows.append({
                "lane_id": lane_id,
                "station_m": float(station),
                "raw_width_m": float(raw),
                "smoothed_width_m": float(smooth),
                "sample_status": status,
                "source_quality": quality,
                "sanity_passed": minimum <= smooth <= maximum,
            })
    return areas, profile_rows, quality_codes


def _node_point(node: dict) -> np.ndarray:
    raw = np.asarray(node.get("point") or [], dtype=np.float64).reshape(-1)
    if len(raw) < 2:
        raise ValueError(f"node has no XY point: {node.get('idx')}")
    return np.asarray([raw[0], raw[1], raw[2] if len(raw) > 2 else 0.0], dtype=np.float64)


def _link_endpoint_heading(link: dict, node_id: str) -> tuple[np.ndarray, float] | None:
    points = points_xyz(link.get("points") or [])
    if len(points) < 2:
        return None
    if str(link.get("to_node_idx")) == node_id:
        vector = points[-1, :2] - points[-2, :2]
    elif str(link.get("from_node_idx")) == node_id:
        vector = points[1, :2] - points[0, :2]
    else:
        return None
    norm = float(np.linalg.norm(vector))
    if norm < 1e-9:
        return None
    width_values = []
    for key in ("width_start", "width_end"):
        try:
            width_values.append(float(link[key]))
        except (KeyError, TypeError, ValueError):
            pass
    return vector / norm, float(np.median(width_values)) if width_values else 3.5


def build_reviewed_stoplines(
    nodes: list[dict], links: list[dict], config: dict, map_id: str, node_sha: str,
    route_points: np.ndarray,
) -> tuple[list[dict], list[dict]]:
    node_by_id = {str(node.get("idx")): node for node in nodes}
    incident: dict[str, list[dict]] = defaultdict(list)
    for link in links:
        incident[str(link.get("from_node_idx"))].append(link)
        incident[str(link.get("to_node_idx"))].append(link)
    route_samples, route_tangents = resample_polyline(route_points, 0.5)
    route_stations = cumulative_distance(route_samples)
    route_index = PolylineProjectionIndex({"route": route_samples.tolist()}, cell_size_m=3.0)
    features: list[dict] = []
    audit: list[dict] = []
    h5_allowlist = [str(value) for value in config["stoplines"]["h5_allowlist"]]
    driving_review = {
        str(key): str(value)
        for key, value in config["stoplines"].get("driving_review_allowlist", {}).items()
    }
    allowlist = list(dict.fromkeys(h5_allowlist + sorted(driving_review)))
    rejected = {str(key): str(value) for key, value in config["stoplines"].get("rejected", {}).items()}
    for node_id in allowlist:
        if node_id in rejected:
            audit.append({"node_id": node_id, "decision": "rejected", "reason": rejected[node_id]})
            continue
        node = node_by_id.get(node_id)
        if node is None:
            raise ValueError(f"H5 allowlisted stopline node is missing from current node set: {node_id}")
        candidates = []
        for link in sorted(incident.get(node_id, []), key=lambda value: str(value.get("idx"))):
            result = _link_endpoint_heading(link, node_id)
            if result is not None:
                candidates.append((link, result[0], result[1]))
        if not candidates:
            raise ValueError(f"cannot resolve stopline heading: {node_id}")
        incoming = [item for item in candidates if str(item[0].get("to_node_idx")) == node_id]
        link, heading, width = (incoming or candidates)[0]
        point = _node_point(node)
        normal = np.asarray([-heading[1], heading[0]], dtype=np.float64)
        endpoints = np.stack((point - 0.5 * width * np.r_[normal, 0.0], point + 0.5 * width * np.r_[normal, 0.0]))
        route_options = route_index.query(point, heading, max_distance_m=30.0, max_heading_deg=90.0)
        route_s = None
        if route_options:
            route_s = float(route_options[0].station_m)
        selection_source = (
            "live_driving_review" if node_id in driving_review else "won_tae_h5_allowlist"
        )
        selection_reason = driving_review.get(node_id, "won_tae_h5_allowlist")
        feature = {
            "id": f"{map_id}:stopline:{node_id}",
            "feature_type": "stopline",
            "frame_id": "map",
            "geometry": {"type": "LineString", "coordinates": endpoints.tolist()},
            "attributes": {
                "node_ids": [node_id],
                "route_node_ids": [node_id],
                "route_s_m": route_s,
                "length_m": width,
                "heading_xy": heading.tolist(),
                "bar_direction_xy": normal.tolist(),
                "traffic_light_ids": [str(node.get("traffic_light_id"))] if node.get("traffic_light_id") else [],
                "signal_controlled": bool(node.get("traffic_light_id")),
                "geometry_status": f"{selection_source}_painted_stopline",
            },
            "topology": {"current_link_ids": [str(link.get("idx"))]},
            "provenance": {
                "source_kind": "current_mgeo",
                "source_id": node_id,
                "source_sha256": node_sha,
                "transform": f"current_node_heading_filtered_by_{selection_source}",
            },
            "validation": {
                "operational": False,
                "painted_stopline_ground_truth": True,
                "selection_source": selection_source,
                "selection_reason": selection_reason,
            },
            "status": "candidate",
        }
        features.append(feature)
        audit.append({
            "node_id": node_id,
            "decision": "kept",
            "reason": selection_reason,
            "current_link_id": str(link.get("idx")),
            "route_s_m": route_s,
            "length_m": width,
        })
    for node_id, reason in sorted(rejected.items()):
        if node_id not in allowlist:
            audit.append({"node_id": node_id, "decision": "rejected", "reason": reason})
    return features, audit


def _direct_observation_polyline(path: Path, side: str, color: str) -> np.ndarray:
    values = []
    for row in _read_csv(path):
        generated = str(row.get("generated", "")).strip().lower() in {"1", "true", "yes"}
        interpolated = str(row.get("interpolated", "")).strip().lower() in {"1", "true", "yes"}
        if generated or interpolated:
            continue
        if str(row.get("side", "")).strip().upper() != side.upper():
            continue
        if str(row.get("lane_color", "")).strip().upper() != color.upper():
            continue
        values.append([float(row["lane_map_x"]), float(row["lane_map_y"]), 0.0])
    points = points_xyz(values)
    if len(points) < 2:
        raise ValueError(f"forbidden boundary source has too few direct points: {path}")
    return points


def build_forbidden_surfaces(config: dict, repo_root: Path, map_id: str) -> list[dict]:
    features = []
    for index, item in enumerate(config["special_zones"].get("forbidden_between_observations", [])):
        first_spec, second_spec = item["first"], item["second"]
        first_path = resolve_source(repo_root, first_spec["path"])
        second_path = resolve_source(repo_root, second_spec["path"])
        first = _direct_observation_polyline(first_path, first_spec["side"], first_spec["color"])
        second = _direct_observation_polyline(second_path, second_spec["side"], second_spec["color"])
        first_sampled, _ = resample_polyline(first, 0.5)
        second_sampled, _ = resample_polyline(second, 0.5)
        # Keep only the mutually supported span.  This avoids extending the
        # safety-island polygon along the longer observation file.
        first_distance = np.min(np.linalg.norm(first_sampled[:, None, :2] - second_sampled[None, :, :2], axis=2), axis=1)
        second_distance = np.min(np.linalg.norm(second_sampled[:, None, :2] - first_sampled[None, :, :2], axis=2), axis=1)
        first_overlap = first_sampled[first_distance <= 6.0]
        second_overlap = second_sampled[second_distance <= 6.0]
        if len(first_overlap) < 2 or len(second_overlap) < 2:
            raise ValueError(f"forbidden surface boundaries do not overlap: {item['name']}")
        same = np.linalg.norm(first_overlap[0, :2] - second_overlap[0, :2]) + np.linalg.norm(first_overlap[-1, :2] - second_overlap[-1, :2])
        reverse = np.linalg.norm(first_overlap[0, :2] - second_overlap[-1, :2]) + np.linalg.norm(first_overlap[-1, :2] - second_overlap[0, :2])
        return_side = second_overlap[::-1] if same <= reverse else second_overlap
        polygon = np.vstack((first_overlap, return_side, first_overlap[:1]))
        combined_hash = sha256_file(first_path) + "+" + sha256_file(second_path)
        features.append({
            "id": f"{map_id}:forbidden_surface:{index:02d}:{item['name']}",
            "feature_type": "forbidden_surface",
            "frame_id": "map",
            "geometry": {"type": "Polygon", "coordinates": [polygon.tolist()]},
            "attributes": {
                "name": str(item["name"]),
                "functional_role": "safety_island",
                "drivable": False,
                "source_files": [str(first_spec["path"]), str(second_spec["path"])],
            },
            "topology": {"current_link_ids": []},
            "provenance": {
                "source_kind": "current_gps_direct",
                "source_id": str(item["name"]),
                "source_sha256": combined_hash,
                "transform": "direct_observation_mutual_span_polygon",
            },
            "validation": {"operational": False, "manual_bridge_applied": False},
            "status": "candidate",
        })
    return features


def _raster_bounds(features: Iterable[dict], margin_m: float) -> tuple[np.ndarray, np.ndarray]:
    clouds = [_feature_points(feature)[:, :2] for feature in features if len(_feature_points(feature))]
    if not clouds:
        raise ValueError("cannot rasterize an empty feature collection")
    combined = np.vstack(clouds)
    return np.min(combined, axis=0) - margin_m, np.max(combined, axis=0) + margin_m


def _pixel_polygon(points: np.ndarray, low: np.ndarray, high: np.ndarray, resolution: float) -> np.ndarray:
    pixels = np.empty((len(points), 2), dtype=np.int32)
    pixels[:, 0] = np.round((points[:, 0] - low[0]) / resolution).astype(np.int32)
    pixels[:, 1] = np.round((high[1] - points[:, 1]) / resolution).astype(np.int32)
    return pixels


def _deterministic_npz(path: Path, arrays: dict[str, np.ndarray]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(path, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=6) as archive:
        for name in sorted(arrays):
            buffer = BytesIO()
            np.lib.format.write_array(buffer, np.asarray(arrays[name]), allow_pickle=False)
            info = zipfile.ZipInfo(f"{name}.npy", date_time=(1980, 1, 1, 0, 0, 0))
            info.compress_type = zipfile.ZIP_DEFLATED
            info.external_attr = 0o644 << 16
            archive.writestr(info, buffer.getvalue(), compress_type=zipfile.ZIP_DEFLATED, compresslevel=6)


def bake_static_layers(
    lane_areas: list[dict], boundaries: list[dict], stoplines: list[dict],
    forbidden: list[dict], config: dict, output_dir: Path,
) -> tuple[dict, list[dict]]:
    resolution = float(config["raster"]["resolution_m"])
    low, high = _raster_bounds(lane_areas + forbidden, float(config["raster"]["margin_m"]))
    width = int(math.ceil((high[0] - low[0]) / resolution)) + 1
    height = int(math.ceil((high[1] - low[1]) / resolution)) + 1
    shape = (height, width)
    road = np.zeros(shape, dtype=np.uint8)
    route = np.zeros(shape, dtype=np.uint8)
    maneuver = np.zeros(shape, dtype=np.uint8)
    forbidden_mask = np.zeros(shape, dtype=np.uint8)
    unknown = np.zeros(shape, dtype=np.uint8)
    lane_ids = np.zeros(shape, dtype=np.uint16)
    rules = np.zeros(shape, dtype=np.uint8)
    quality = np.zeros(shape, dtype=np.uint8)
    lookup = []
    lane_number = {lane_id: index + 1 for index, lane_id in enumerate(sorted(
        area["attributes"]["lane_id"] for area in lane_areas
    ))}
    prefix_values = tuple(str(value) for value in config["special_zones"]["maneuver_role_prefixes"])
    exclude_roles = set(str(value) for value in config["special_zones"].get("exclude_roles", []))

    # Non-route context is drawn first, so the official route owns overlap
    # pixels in lane_id_raster.
    ordered = sorted(
        lane_areas,
        key=lambda feature: (
            "official_route" in feature["attributes"]["route_maneuver_roles"],
            feature["attributes"]["lane_id"],
        ),
    )
    for area in ordered:
        lane_id = str(area["attributes"]["lane_id"])
        roles = set(area["attributes"]["route_maneuver_roles"])
        polygon = _pixel_polygon(_feature_points(area), low, high, resolution)
        local = np.zeros(shape, dtype=np.uint8)
        cv2.fillPoly(local, [polygon], 1)
        road[local > 0] = 1
        if "official_route" in roles:
            route[local > 0] = 1
        special = any(any(role.startswith(prefix) for prefix in prefix_values) for role in roles)
        if special and not (roles & exclude_roles):
            maneuver[local > 0] = 1
        lane_ids[local > 0] = lane_number[lane_id]
        quality_code = int(area["attributes"]["source_quality_code"])
        quality[local > 0] = np.maximum(quality[local > 0], quality_code)
        if quality_code <= 1:
            unknown[local > 0] = 1
        lookup.append({
            "raster_value": lane_number[lane_id],
            "lane_id": lane_id,
            "roles": ";".join(sorted(roles)),
            "source_quality": area["attributes"]["geometry_quality"],
        })

    for surface in forbidden:
        polygon = _pixel_polygon(_feature_points(surface), low, high, resolution)
        cv2.fillPoly(forbidden_mask, [polygon], 1)
    for layer in (road, route, maneuver, unknown):
        layer[forbidden_mask > 0] = 0
    lane_ids[forbidden_mask > 0] = 0
    quality[forbidden_mask > 0] = 0

    rule_code = {"permitted": 1, "restricted": 2, "prohibited": 3, "unknown": 4}
    thickness = max(1, int(round(float(config["raster"]["boundary_thickness_m"]) / resolution)))
    for boundary in boundaries:
        points = _pixel_polygon(_feature_points(boundary), low, high, resolution)
        code = rule_code.get(str(boundary["attributes"].get("crossing_rule", "unknown")), 4)
        cv2.polylines(rules, [points], False, int(code), thickness, cv2.LINE_8)
    for stopline in stoplines:
        points = _pixel_polygon(_feature_points(stopline), low, high, resolution)
        cv2.polylines(rules, [points], False, 5, thickness, cv2.LINE_8)

    arrays = {
        "road_surface_mask": road,
        "route_corridor_mask": route,
        "maneuver_surface_mask": maneuver,
        "forbidden_surface_mask": forbidden_mask,
        "unknown_mask": unknown,
        "lane_id_raster": lane_ids,
        "boundary_rule_raster": rules,
        "source_quality_raster": quality,
    }
    _deterministic_npz(output_dir / "static_layers.npz", arrays)
    layer_dir = output_dir / "layers"
    layer_dir.mkdir(parents=True, exist_ok=True)
    for name, value in arrays.items():
        if value.dtype == np.uint16:
            maximum = max(int(value.max()), 1)
            preview = np.round(value.astype(np.float64) * 255.0 / maximum).astype(np.uint8)
        else:
            maximum = max(int(value.max()), 1)
            preview = np.round(value.astype(np.float64) * 255.0 / maximum).astype(np.uint8)
        if not cv2.imwrite(str(layer_dir / f"{name}.png"), preview):
            raise IOError(f"failed to write raster preview: {name}")
    metadata = {
        "frame_id": "map",
        "resolution_m": resolution,
        "origin_xy_m": low.tolist(),
        "bounds_xy_m": [low.tolist(), high.tolist()],
        "width_px": width,
        "height_px": height,
        "axis_convention": "col=+x,row=-y",
        "boundary_rule_codes": {
            "0": "none", "1": "permitted", "2": "restricted",
            "3": "prohibited", "4": "unknown", "5": "stopline",
        },
        "source_quality_codes": {
            "0": "outside_or_forbidden", "1": "link_nominal_fallback",
            "2": "partially_observed", "3": "direct_observed",
        },
        "maneuver_surface_semantics": "route_and_special-zone physical lane polygons; permission remains boundary-rule/planner policy",
    }
    _write_json(output_dir / "static_layers_metadata.json", metadata)
    return metadata, lookup


def _draw_dashed(image: np.ndarray, pixels: np.ndarray, color: tuple[int, int, int], thickness: int) -> None:
    for index in range(0, len(pixels) - 1, 2):
        cv2.line(image, tuple(pixels[index]), tuple(pixels[index + 1]), color, thickness, cv2.LINE_AA)


def _render_overlay(
    path: Path, title: str, lane_areas: list[dict], boundaries: list[dict],
    markings: list[dict], stoplines: list[dict], forbidden: list[dict], visual: dict,
) -> None:
    selected = lane_areas or []
    relevant_links = {
        value for area in selected for value in area.get("topology", {}).get("current_link_ids", [])
    }
    visible_boundaries = [
        feature for feature in boundaries
        if set(feature.get("topology", {}).get("current_link_ids", [])) & relevant_links
    ]
    visible_markings = [
        feature for feature in markings
        if set(feature.get("topology", {}).get("current_link_ids", [])) & relevant_links
    ]
    visible_stops = [
        feature for feature in stoplines
        if set(feature.get("topology", {}).get("current_link_ids", [])) & relevant_links
    ]
    low, high = _raster_bounds(selected + visible_boundaries + visible_markings, 8.0)
    width, height, padding = int(visual["width_px"]), int(visual["height_px"]), int(visual["padding_px"])
    span = np.maximum(high - low, 1.0)
    scale = min((width - 2 * padding) / span[0], (height - 2 * padding) / span[1])
    center = 0.5 * (low + high)
    draw_span = np.asarray([(width - 2 * padding) / scale, (height - 2 * padding) / scale])
    draw_low, draw_high = center - 0.5 * draw_span, center + 0.5 * draw_span
    image = np.full((height, width, 3), 24, dtype=np.uint8)

    def pixels(feature: dict) -> np.ndarray:
        points = _feature_points(feature)
        result = np.empty((len(points), 2), dtype=np.int32)
        result[:, 0] = np.round(padding + (points[:, 0] - draw_low[0]) * scale).astype(np.int32)
        result[:, 1] = np.round(padding + (draw_high[1] - points[:, 1]) * scale).astype(np.int32)
        return result

    for area in selected:
        overlay = image.copy()
        roles = area["attributes"]["route_maneuver_roles"]
        color = (52, 62, 52) if "official_route" in roles else (42, 48, 58)
        cv2.fillPoly(overlay, [pixels(area)], color)
        image = cv2.addWeighted(overlay, 0.55, image, 0.45, 0.0)
    for surface in forbidden:
        cv2.fillPoly(image, [pixels(surface)], (45, 45, 125))
    for boundary in visible_boundaries:
        color = (210, 100, 40) if boundary["attributes"]["observed_fraction"] > 0 else (85, 85, 85)
        cv2.polylines(image, [pixels(boundary)], False, color, 1, cv2.LINE_AA)
    for marking in visible_markings:
        color = (245, 245, 245) if marking["attributes"]["color"] == "white" else (
            (20, 220, 245) if marking["attributes"]["color"] == "yellow" else (180, 80, 180)
        )
        if marking["attributes"]["pattern"] == "broken":
            _draw_dashed(image, pixels(marking), color, 2)
        else:
            cv2.polylines(image, [pixels(marking)], False, color, 2, cv2.LINE_AA)
    for stopline in visible_stops:
        cv2.polylines(image, [pixels(stopline)], False, (40, 40, 245), 3, cv2.LINE_AA)
    lines = [
        title,
        "orange=direct-observation anchored logical boundary  gray=nominal fallback",
        "white/yellow=physical marking template  red=reviewed stopline",
        "dark red=safety island forbidden  CANDIDATE / operational=false",
    ]
    for index, line in enumerate(lines):
        cv2.putText(image, line, (12, 21 + 19 * index), cv2.FONT_HERSHEY_SIMPLEX, 0.43, (238, 238, 238), 1, cv2.LINE_AA)
    path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(path), image):
        raise IOError(path)


def _load_route(path: Path) -> np.ndarray:
    values = []
    with path.open("r", encoding="utf-8") as stream:
        for line in stream:
            row = line.strip().replace(",", " ").split()
            if len(row) >= 2:
                values.append([float(row[0]), float(row[1]), float(row[2]) if len(row) > 2 else 0.0])
    result = points_xyz(values)
    if len(result) < 2:
        raise ValueError(f"organizer route is empty: {path}")
    return result


def _anchor_registry(
    matches_path: Path, nominal_document: dict, snapshot: dict,
) -> list[dict]:
    parent_by_id = {
        feature["id"]: feature for feature in nominal_document["features"]
        if feature["feature_type"] == "lane_boundary"
    }
    source_hashes = {
        Path(record["path"]).name: record["sha256"]
        for record in snapshot["files"] if record.get("role") == "direct_observation"
    }
    rows = []
    for raw in _read_csv(matches_path):
        parent = parent_by_id.get(str(raw.get("parent_boundary_id")))
        link_sides = (parent or {}).get("attributes", {}).get("link_sides", {})
        rows.append({
            "segment_id": raw.get("segment_id"),
            "run_index": raw.get("run_index"),
            "source_file": raw.get("source_file"),
            "source_sha256": source_hashes.get(str(raw.get("source_file"))),
            "source_kind": raw.get("source_kind"),
            "collection_side_audit_only": raw.get("collection_side"),
            "lane_color": raw.get("lane_color"),
            "parent_boundary_id": raw.get("parent_boundary_id"),
            "canonical_link_sides": json.dumps(link_sides, sort_keys=True),
            "station_start_m": raw.get("station_start_m"),
            "station_end_m": raw.get("station_end_m"),
            "observed_length_m": raw.get("observed_length_m"),
            "coverage": raw.get("coverage"),
            "nominal_distance_p95_m": raw.get("nominal_distance_p95_m"),
            "fit_residual_p95_m": raw.get("fit_residual_p95_m"),
            "anchor_status": raw.get("match_status"),
            "geometry_anchor_eligible": str(raw.get("match_status")) == "automatic",
        })
    return rows


def _clone_centerlines(document: dict, map_id: str) -> list[dict]:
    result = []
    for source in sorted(
        (feature for feature in document["features"] if feature["feature_type"] == "link_centerline"),
        key=lambda feature: feature["id"],
    ):
        feature = json.loads(json.dumps(source))
        feature["id"] = _retag_id(source["id"], map_id, "link_centerline")
        feature["status"] = "candidate"
        feature.setdefault("validation", {})["operational"] = False
        result.append(feature)
    return result


def _hash_join(*values: str) -> str:
    import hashlib
    digest = hashlib.sha256()
    for value in values:
        digest.update(str(value).encode("utf-8"))
        digest.update(b"\0")
    return digest.hexdigest()


def build_operational_map(config_path: Path, repo_root: Path, output_dir: Path) -> dict:
    with config_path.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream) or {}
    if config.get("schema_version") != CONFIG_SCHEMA:
        raise ValueError("unsupported operational map config schema")
    if config.get("promotion_policy", {}).get("operational") is not False:
        raise ValueError("the operational-map builder cannot self-promote")
    paths, source_contract = _verified_sources(config, repo_root)
    nominal = load_json(paths["nominal_map"])
    observed = load_json(paths["observed_map"])
    hybrid = load_json(paths["hybrid_map"])
    validate_canonical_document(nominal)
    validate_canonical_document(observed)
    validate_canonical_document(hybrid)
    map_id = str(config["map_id"])
    profile_rows = _load_offset_profiles(paths["boundary_offset_profiles"])
    width_profiles = _width_profile_by_link(paths["link_width_profiles"])
    boundaries, boundary_audit = build_observation_anchored_boundaries(
        nominal, profile_rows, map_id=map_id,
        sample_step_m=float(config["geometry"]["sample_step_m"]),
        max_gap_m=float(config["geometry"]["direct_gap_limit_m"]),
        observed_source_hash=source_contract["pipeline_sources"]["observed_map"]["sha256"],
    )
    markings, marking_audit, legacy_review_required = build_lane_markings(
        hybrid, boundaries, observed, width_profiles, config, map_id,
    )
    centerlines = _clone_centerlines(nominal, map_id)
    composite_hash = _hash_join(
        source_contract["pipeline_sources"]["current_link_set"]["sha256"],
        source_contract["pipeline_sources"]["observed_map"]["sha256"],
    )
    lane_areas, final_width_rows, quality_codes = build_lane_areas(
        centerlines, boundaries, width_profiles, config, map_id, composite_hash,
    )
    current_nodes = load_json(paths["current_node_set"])
    current_links = load_json(paths["current_link_set"])
    route = _load_route(paths["organizer_route"])
    stoplines, stopline_audit = build_reviewed_stoplines(
        current_nodes, current_links, config, map_id,
        source_contract["pipeline_sources"]["current_node_set"]["sha256"], route,
    )
    forbidden = build_forbidden_surfaces(config, repo_root, map_id)

    document = {
        "schema_version": SCHEMA_VERSION,
        "map_id": map_id,
        "frame": dict(nominal["frame"]),
        "sources": source_contract,
        "build": {
            "build_id": str(config["build_id"]),
            "stage": "won_tae_observation_anchored_vector_and_static_surface_candidate",
            "operational": False,
            "topology_authority": "current_mgeo",
            "geometry_authority": "won_tae_direct_2025_observation",
            "legacy_role": "locally_gated_marking_template",
            "raster_is_derived": True,
        },
        "features": centerlines + markings + boundaries + lane_areas + stoplines + forbidden,
        "review_layers": {
            "legacy_template_review_required": legacy_review_required,
        },
    }
    validate_canonical_document(document)
    output_dir.mkdir(parents=True, exist_ok=True)
    _write_json(output_dir / "source_snapshot.json", source_contract)
    _write_json(output_dir / "operational_candidate_map.json", document)

    anchor_rows = _anchor_registry(paths["segment_matches"], nominal, source_contract["won_tae_snapshot"])
    _write_csv(
        output_dir / "observation_anchor_registry.csv", anchor_rows,
        [
            "segment_id", "run_index", "source_file", "source_sha256", "source_kind",
            "collection_side_audit_only", "lane_color", "parent_boundary_id",
            "canonical_link_sides", "station_start_m", "station_end_m", "observed_length_m",
            "coverage", "nominal_distance_p95_m", "fit_residual_p95_m", "anchor_status",
            "geometry_anchor_eligible",
        ],
    )
    boundary_fields = [
        "boundary_id", "parent_boundary_id", "current_link_ids", "observed_fraction",
        "observed_range_count", "maximum_abs_offset_m", "lane_color", "source_files",
    ]
    _write_csv(output_dir / "logical_boundary_audit.csv", boundary_audit, boundary_fields)
    marking_fields = [
        "marking_id", "source_feature_id", "source_kind", "current_link_ids", "color",
        "pattern", "line_group", "logical_boundary_id", "logical_binding_p95_m",
        "anchor_boundary_id", "sample_count", "matched_count", "coverage",
        "pre_alignment_lateral_p95_m", "lateral_residual_p95_m",
        "heading_difference_p95_deg", "width_difference_m",
        "width_consistency_passed", "automatic_decision", "decision",
        "driving_review_interval", "reviewed_lane_id", "pre_review_sample_count",
        "reviewed_sample_count", "reviewed_start_extension_m",
        "reviewed_end_extension_m", "failed_checks",
    ]
    _write_csv(output_dir / "hybrid_observation_audit.csv", marking_audit, marking_fields)
    _write_csv(
        output_dir / "legacy_template_audit.csv",
        [row for row in marking_audit if row["source_kind"] == "legacy_katri"], marking_fields,
    )
    _write_csv(
        output_dir / "lane_width_profiles.csv", final_width_rows,
        [
            "lane_id", "station_m", "raw_width_m", "smoothed_width_m", "sample_status",
            "source_quality", "sanity_passed",
        ],
    )
    _write_csv(
        output_dir / "stopline_audit.csv", stopline_audit,
        ["node_id", "decision", "reason", "current_link_id", "route_s_m", "length_m"],
    )
    raster_metadata, lane_lookup = bake_static_layers(
        lane_areas, boundaries, stoplines, forbidden, config, output_dir,
    )
    _write_csv(
        output_dir / "lane_id_lookup.csv", lane_lookup,
        ["raster_value", "lane_id", "roles", "source_quality"],
    )

    overlays = output_dir / "overlays"
    _render_overlay(
        overlays / "full_corridor.png", "K-City 2025 operational candidate / full corridor",
        lane_areas, boundaries, markings, stoplines, forbidden, config["visualization"],
    )
    zone_filters = {
        "high_speed.png": lambda roles: "high_speed_same_direction" in roles,
        "shadow.png": lambda roles: any(role.startswith("shadow_") for role in roles),
        "static_obstacle.png": lambda roles: any(role.startswith("static_obstacle_") for role in roles),
    }
    for filename, predicate in zone_filters.items():
        selected = [
            area for area in lane_areas
            if predicate(set(area["attributes"]["route_maneuver_roles"]))
        ]
        if selected:
            _render_overlay(
                overlays / filename, f"K-City 2025 operational candidate / {filename[:-4]}",
                selected, boundaries, markings, stoplines, forbidden, config["visualization"],
            )
    for interval in config.get("driving_reviewed_intervals", []):
        lane_id = str(interval["lane_id"])
        selected = [
            area for area in lane_areas if str(area["attributes"]["lane_id"]) == lane_id
        ]
        if len(selected) != 1:
            raise ValueError(f"reviewed interval lane area is not unique: {lane_id}")
        _render_overlay(
            overlays / f"reviewed_{interval['name']}.png",
            f"K-City 2025 reviewed interval / {interval['name']}",
            selected, boundaries, markings, stoplines, forbidden, config["visualization"],
        )

    roles = Counter(
        role for area in lane_areas for role in area["attributes"]["route_maneuver_roles"]
    )
    legacy_rows = [row for row in marking_audit if row["source_kind"] == "legacy_katri"]
    nonfallback_widths = np.asarray([
        float(row["smoothed_width_m"]) for row in final_width_rows
        if row["source_quality"] != "link_nominal_fallback"
    ], dtype=np.float64)
    fallback_lane_ids = sorted(
        lane_id for lane_id, code in quality_codes.items() if code == 1
    )
    summary = {
        "build_id": str(config["build_id"]),
        "map_id": map_id,
        "status": "candidate_requires_route_by_route_review",
        "operational": False,
        "source_snapshot": {
            "git_commit": source_contract["won_tae_snapshot"]["git_commit"],
            "verified_file_count": source_contract["won_tae_snapshot"]["file_count"],
        },
        "counts": {
            "current_link_centerline": len(centerlines),
            "lane_marking": len(markings),
            "legacy_review_only": len(legacy_review_required),
            "logical_lane_boundary": len(boundaries),
            "lane_area": len(lane_areas),
            "stopline_kept": len(stoplines),
            "stopline_rejected": sum(row["decision"] == "rejected" for row in stopline_audit),
            "forbidden_surface": len(forbidden),
            "observation_anchor_runs": sum(row["geometry_anchor_eligible"] for row in anchor_rows),
            "legacy_template_count": len(legacy_rows),
            "legacy_template_aligned_count": sum(row["decision"] == "aligned_template" for row in legacy_rows),
            "legacy_template_driving_reviewed_count": sum(
                row["decision"] == "driving_review_approved" for row in legacy_rows
            ),
        },
        "corridor_roles": dict(sorted(roles.items())),
        "quality": {
            "lane_area_quality_counts": dict(sorted(Counter(quality_codes.values()).items())),
            "width_sanity_failure_samples": sum(not bool(row["sanity_passed"]) for row in final_width_rows),
            "nominal_fallback_lane_count": sum(value == 1 for value in quality_codes.values()),
            "nominal_fallback_lane_ids": fallback_lane_ids,
            "nonfallback_width_p05_m": quantile_or_none(nonfallback_widths, 0.05),
            "nonfallback_width_p50_m": quantile_or_none(nonfallback_widths, 0.50),
            "nonfallback_width_p95_m": quantile_or_none(nonfallback_widths, 0.95),
        },
        "raster": raster_metadata,
        "contracts": {
            "topology": "current Node/Link",
            "lateral_geometry_and_width": "Won-tae direct 2025 observations",
            "paint_representation": "observation-gated legacy template or current-2025 reconstruction",
            "stoplines": "Won-tae H5 allowlist plus driving review",
            "runtime_connection": False,
        },
        "driving_reviewed_intervals": list(config.get("driving_reviewed_intervals", [])),
        "next_stage": "drive_review_each_route_interval_then_promote_selected_features",
    }
    _write_json(output_dir / "summary.json", summary)
    readme = "# K-City 2025 operational candidate\n\n"
    readme += "This report is generated from immutable Won-tae observation inputs. "
    readme += "It remains `operational=false`; no runtime package consumes it.\n\n"
    readme += "- `operational_candidate_map.json`: vector source of truth candidate\n"
    readme += "- `static_layers.npz`: derived raster bundle\n"
    readme += "- `observation_anchor_registry.csv`: direct/generated distinction and canonical side binding\n"
    readme += "- `legacy_template_audit.csv`: local normal-alignment gate for legacy paint templates\n"
    readme += "- `overlays/`: full/high-speed/shadow/static-obstacle review images\n"
    readme += f"\nSelected markings: {len(markings)}; legacy automatic/driving-reviewed/review-only: "
    readme += f"{sum(row['decision'] == 'aligned_template' for row in legacy_rows)}/"
    readme += f"{sum(row['decision'] == 'driving_review_approved' for row in legacy_rows)}/"
    readme += f"{len(legacy_review_required)}.\n"
    readme += f"Lane areas direct/partial/nominal: {sum(value == 3 for value in quality_codes.values())}/"
    readme += f"{sum(value == 2 for value in quality_codes.values())}/{sum(value == 1 for value in quality_codes.values())}.\n"
    (output_dir / "README.md").write_text(readme, encoding="utf-8")
    return summary
