"""Build observed-calibrated lane-boundary candidates from stage-1 geometry."""

from __future__ import annotations

from collections import Counter, defaultdict
import csv
import hashlib
import json
import math
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import yaml

from .current_builder import _plain, _write_csv, _write_json
from .geometry import cumulative_distance, points_xyz, quantile_or_none, resample_polyline
from .io_mgeo import resolve_source, sha256_file, source_record
from .observed import (
    PolylineProjectionIndex,
    associate_projection_sequence,
    classify_observation_row,
    extract_polyline_interval,
    interpolate_profile,
    offset_interval,
    profile_runs,
    resample_observation_rows,
    robust_station_profile,
    split_ordered_observations,
)
from .schema import SCHEMA_VERSION, validate_canonical_document


CONFIG_SCHEMA = "asmc.hd_map.observed_calibration_config/v1"
OBSERVED_SOURCE_KINDS = {"current_lidar_semantic", "current_gps_direct"}


def _read_csv(path: Path) -> list[dict]:
    with path.open("r", encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))


def _color_for(row: dict, source: dict) -> str:
    side = str(row.get("side", "")).strip().upper()
    overrides = {str(key).upper(): str(value).upper() for key, value in (
        source.get("side_colors") or {}
    ).items()}
    return overrides.get(side, str(row.get("lane_color", "UNKNOWN")).strip().upper())


def _source_group(row: dict, source: dict, configured_name: str) -> str:
    values = [configured_name]
    group_column = source.get("group_column")
    if group_column:
        values.append(f"{group_column}={row.get(group_column, '')}")
    if row.get("source_file"):
        values.append(f"source_file={row['source_file']}")
    return "|".join(values)


def _load_observations(config: dict, repo_root: Path) -> tuple[list[dict], list[dict], dict[str, dict]]:
    audit_rows: list[dict] = []
    direct_rows: list[dict] = []
    records: dict[str, dict] = {}
    for source_index, source in enumerate(config["sources"]):
        configured_path = str(source["path"])
        path = resolve_source(repo_root, configured_path)
        if not path.is_file():
            raise FileNotFoundError(path)
        measurement = str(source["measurement"])
        if measurement not in OBSERVED_SOURCE_KINDS:
            raise ValueError(f"invalid observed measurement kind: {measurement}")
        rows = _read_csv(path)
        counts = Counter(classify_observation_row(row) for row in rows)
        source_key = f"observed_{source_index:02d}_{path.name}"
        records[source_key] = source_record(repo_root, configured_path, path)
        records[source_key]["source_kind"] = measurement
        records[source_key]["manual_bridge_applied"] = False
        records[source_key]["raster_inward_offset_m"] = 0.0
        audit_rows.append({
            "source_index": source_index,
            "source_file": path.name,
            "configured_path": configured_path,
            "source_kind": measurement,
            "sha256": records[source_key]["sha256"],
            "total_rows": len(rows),
            "direct_rows": counts["direct"],
            "generated_rows": counts["generated"],
            "interpolated_rows": counts["interpolated"],
            "fit_support_rows": counts["direct"],
            "audit_only_rows": counts["generated"] + counts["interpolated"],
            "manual_bridge_rows": 0,
            "canonical_inward_offset_m": 0.0,
        })
        for row_index, row in enumerate(rows):
            classification = classify_observation_row(row)
            if classification != "direct":
                continue
            enriched = dict(row)
            enriched.update({
                "_source_index": source_index,
                "_source_key": source_key,
                "_source_path": configured_path,
                "_source_file": path.name,
                "_source_hash": records[source_key]["sha256"],
                "_source_kind": measurement,
                "_source_row": row_index + 2,
                "_collection_side": str(row.get("side", "")).strip().upper(),
                "_lane_color": _color_for(row, source),
                "_group": _source_group(row, source, path.name),
            })
            direct_rows.append(enriched)
    return audit_rows, direct_rows, records


def _validate_input_contract(config: dict, audit_rows: list[dict]) -> None:
    expected = config["expected_rows"]
    totals = {
        "total": sum(int(row["total_rows"]) for row in audit_rows),
        "direct": sum(int(row["direct_rows"]) for row in audit_rows),
        "generated": sum(int(row["generated_rows"]) for row in audit_rows),
        "interpolated": sum(int(row["interpolated_rows"]) for row in audit_rows),
        "current_lidar_semantic": sum(
            int(row["direct_rows"]) for row in audit_rows
            if row["source_kind"] == "current_lidar_semantic"
        ),
        "current_gps_direct": sum(
            int(row["direct_rows"]) for row in audit_rows
            if row["source_kind"] == "current_gps_direct"
        ),
    }
    mismatches = {
        key: (int(expected[key]), totals[key]) for key in totals
        if int(expected[key]) != totals[key]
    }
    if mismatches:
        raise ValueError(f"observed row contract changed: {mismatches}")
    policy = config["promotion_policy"]
    forbidden = {
        "apply_generated_rows", "apply_interpolated_rows", "apply_manual_bridges",
        "apply_raster_inward_offset",
    }
    if any(policy.get(key) is not False for key in forbidden):
        raise ValueError("stage-2 policy must exclude generated/interpolated/manual/raster inputs")


def _connected_boundary_pairs(parent: dict, boundaries: list[dict]) -> set[frozenset[str]]:
    link_features = {
        feature["topology"]["current_link_ids"][0]: feature
        for feature in parent["features"] if feature["feature_type"] == "link_centerline"
    }
    connected_links: set[frozenset[str]] = set()
    for link_id, feature in link_features.items():
        topology = feature.get("topology") or {}
        for other in topology.get("predecessor_link_ids", []) + topology.get("successor_link_ids", []):
            if other in link_features:
                connected_links.add(frozenset((link_id, str(other))))
    pairs: set[frozenset[str]] = set()
    for first_index, first in enumerate(boundaries):
        first_links = set(first["topology"]["current_link_ids"])
        first_sides = first.get("attributes", {}).get("link_sides") or {}
        for second in boundaries[first_index + 1:]:
            second_links = set(second["topology"]["current_link_ids"])
            second_sides = second.get("attributes", {}).get("link_sides") or {}
            if any(
                frozenset((a, b)) in connected_links
                and first_sides.get(a) in {"left", "right"}
                and first_sides.get(a) == second_sides.get(b)
                for a in first_links for b in second_links
            ):
                pairs.add(frozenset((first["id"], second["id"])))
    return pairs


def _observation_segments(direct_rows: list[dict], config: dict) -> list[dict]:
    grouped: dict[tuple[str, str, str], list[dict]] = defaultdict(list)
    for row in direct_rows:
        grouped[(row["_group"], row["_collection_side"], row["_lane_color"])].append(row)
    segmentation = config["segmentation"]
    segments: list[dict] = []
    segment_id = 0
    for group_key in sorted(grouped):
        rows = grouped[group_key]
        for split in split_ordered_observations(
            rows,
            max_time_gap_sec=float(segmentation["max_time_gap_sec"]),
            max_point_gap_m=float(segmentation["max_point_gap_m"]),
        ):
            if len(split) < 2:
                continue
            points, tangents = resample_observation_rows(
                split, float(segmentation["resample_step_m"]),
            )
            segments.append({
                "segment_id": f"obs_{segment_id:05d}",
                "source_group": group_key[0],
                "source_file": split[0]["_source_file"],
                "source_path": split[0]["_source_path"],
                "source_hash": split[0]["_source_hash"],
                "source_kind": split[0]["_source_kind"],
                "collection_side": group_key[1],
                "lane_color": group_key[2],
                "raw_row_count": len(split),
                "raw_row_first": split[0]["_source_row"],
                "raw_row_last": split[-1]["_source_row"],
                "points": points,
                "tangents": tangents,
                "observed_length_m": float(cumulative_distance(points)[-1]) if len(points) else 0.0,
            })
            segment_id += 1
    return segments


def _profile_metrics(projections: list, config: dict) -> tuple[list[dict], np.ndarray, dict]:
    profile_config = config["profile"]
    stations = np.asarray([item.station_m for item in projections], dtype=np.float64)
    offsets = np.asarray([item.signed_offset_m for item in projections], dtype=np.float64)
    profile, inliers = robust_station_profile(
        stations, offsets,
        bin_size_m=float(profile_config["station_bin_m"]),
        mad_multiplier=float(profile_config["mad_multiplier"]),
    )
    residuals = []
    for station, offset, valid in zip(stations, offsets, inliers):
        if not valid:
            continue
        fitted = interpolate_profile(
            profile, float(station), float(profile_config["maximum_interpolation_gap_m"]),
        )
        if fitted is not None:
            residuals.append(abs(float(offset) - fitted))
    slopes = [
        abs((second["offset_m"] - first["offset_m"]) /
            max(second["station_m"] - first["station_m"], 1e-9))
        for first, second in zip(profile[:-1], profile[1:])
        if second["station_m"] - first["station_m"] <= float(
            profile_config["maximum_interpolation_gap_m"]
        ) + 1e-9
    ]
    metrics = {
        "fit_residual_p95_m": quantile_or_none(np.asarray(residuals), 0.95),
        "outlier_ratio": float(1.0 - np.mean(inliers)) if len(inliers) else 1.0,
        "maximum_abs_correction_m": max((abs(row["offset_m"]) for row in profile), default=math.inf),
        "maximum_offset_slope_m_per_m": max(slopes, default=0.0),
    }
    return profile, inliers, metrics


def _gate_status(metrics: dict, config: dict) -> tuple[str, str]:
    automatic = config["automatic_gate"]
    checks = {
        "observed_length": metrics["observed_length_m"] >= float(automatic["minimum_observed_length_m"]),
        "resampled_points": metrics["resampled_points"] >= int(automatic["minimum_resampled_points"]),
        "coverage": metrics["coverage"] >= float(automatic["minimum_coverage"]),
        "nominal_distance": metrics["nominal_distance_p95_m"] is not None and metrics["nominal_distance_p95_m"] <= float(automatic["maximum_nominal_distance_p95_m"]),
        "fit_residual": metrics["fit_residual_p95_m"] is not None and metrics["fit_residual_p95_m"] <= float(automatic["maximum_fit_residual_p95_m"]),
        "outlier_ratio": metrics["outlier_ratio"] <= float(automatic["maximum_outlier_ratio"]),
        "correction": metrics["maximum_abs_correction_m"] <= float(automatic["maximum_abs_correction_m"]),
        "offset_slope": metrics["maximum_offset_slope_m_per_m"] <= float(automatic["maximum_offset_slope_m_per_m"]),
    }
    if all(checks.values()):
        return "automatic", "all_automatic_gates_passed"
    review = config["review_gate"]
    review_checks = {
        "coverage": metrics["coverage"] >= float(review["minimum_coverage"]),
        "nominal_distance": metrics["nominal_distance_p95_m"] is not None and metrics["nominal_distance_p95_m"] <= float(review["maximum_nominal_distance_p95_m"]),
        "correction": metrics["maximum_abs_correction_m"] <= float(review["maximum_abs_correction_m"]),
    }
    failed = ";".join(sorted(key for key, passed in checks.items() if not passed))
    if all(review_checks.values()):
        return "review_required", f"automatic_failed:{failed}"
    return "rejected", f"automatic_failed:{failed}"


def _match_segments(segments: list[dict], boundaries: list[dict], parent: dict, config: dict) -> tuple[list[dict], list[dict]]:
    polylines = {feature["id"]: feature["geometry"]["coordinates"] for feature in boundaries}
    index = PolylineProjectionIndex(polylines, cell_size_m=float(config["matching"]["search_distance_m"]))
    connected_pairs = _connected_boundary_pairs(parent, boundaries)
    report_rows: list[dict] = []
    accepted_profiles: list[dict] = []
    for segment in segments:
        candidate_steps = [
            index.query(
                point, tangent,
                max_distance_m=float(config["matching"]["search_distance_m"]),
                max_heading_deg=float(config["matching"]["max_heading_deg"]),
            )
            for point, tangent in zip(segment["points"], segment["tangents"])
        ]
        assignments = associate_projection_sequence(
            candidate_steps, connected_pairs,
            search_distance_m=float(config["matching"]["search_distance_m"]),
            max_heading_deg=float(config["matching"]["max_heading_deg"]),
            connected_transition_cost=float(config["matching"]["connected_transition_cost"]),
        )
        matched_total = sum(item is not None for item in assignments)
        coverage = matched_total / max(len(assignments), 1)
        run_start = 0
        emitted = False
        while run_start < len(assignments):
            owner = assignments[run_start].owner_id if assignments[run_start] is not None else None
            run_end = run_start + 1
            while run_end < len(assignments):
                next_owner = assignments[run_end].owner_id if assignments[run_end] is not None else None
                if next_owner != owner:
                    break
                run_end += 1
            if owner is not None:
                projections = [item for item in assignments[run_start:run_end] if item is not None]
                profile, _, fit_metrics = _profile_metrics(projections, config)
                distances = np.asarray([item.distance_m for item in projections])
                metrics = {
                    "observed_length_m": max(0.0, (len(projections) - 1) * float(config["segmentation"]["resample_step_m"])),
                    "resampled_points": len(projections),
                    "coverage": coverage,
                    "nominal_distance_p50_m": quantile_or_none(distances, 0.50),
                    "nominal_distance_p95_m": quantile_or_none(distances, 0.95),
                    **fit_metrics,
                }
                status, reason = _gate_status(metrics, config)
                report = {
                    "segment_id": segment["segment_id"],
                    "run_index": len([row for row in report_rows if row["segment_id"] == segment["segment_id"]]),
                    "source_group": segment["source_group"],
                    "source_file": segment["source_file"],
                    "source_kind": segment["source_kind"],
                    "collection_side": segment["collection_side"],
                    "lane_color": segment["lane_color"],
                    "parent_boundary_id": owner,
                    "match_status": status,
                    "reason": reason,
                    "station_start_m": min(item.station_m for item in projections),
                    "station_end_m": max(item.station_m for item in projections),
                    **metrics,
                }
                report_rows.append(report)
                emitted = True
                if status == "automatic" and profile:
                    accepted_profiles.append({
                        "segment_id": segment["segment_id"],
                        "report_index": len(report_rows) - 1,
                        "parent_boundary_id": owner,
                        "source_file": segment["source_file"],
                        "source_path": segment["source_path"],
                        "source_hash": segment["source_hash"],
                        "source_kind": segment["source_kind"],
                        "lane_color": segment["lane_color"],
                        "profile": profile,
                    })
            run_start = run_end
        if not emitted:
            report_rows.append({
                "segment_id": segment["segment_id"], "run_index": 0,
                "source_group": segment["source_group"], "source_file": segment["source_file"],
                "source_kind": segment["source_kind"], "collection_side": segment["collection_side"],
                "lane_color": segment["lane_color"], "parent_boundary_id": "",
                "match_status": "unmatched", "reason": "no_compatible_nominal_boundary",
                "station_start_m": None, "station_end_m": None,
                "observed_length_m": segment["observed_length_m"],
                "resampled_points": len(segment["points"]), "coverage": coverage,
                "nominal_distance_p50_m": None, "nominal_distance_p95_m": None,
                "fit_residual_p95_m": None, "outlier_ratio": None,
                "maximum_abs_correction_m": None,
                "maximum_offset_slope_m_per_m": None,
            })
    return report_rows, accepted_profiles


def _combine_profiles(accepted_profiles: list[dict], config: dict, report_rows: list[dict]) -> tuple[dict[str, list[dict]], list[dict]]:
    bin_size = float(config["profile"]["station_bin_m"])
    conflict_limit = float(config["profile"]["maximum_source_disagreement_p95_m"])
    by_parent: dict[str, list[dict]] = defaultdict(list)
    for candidate in accepted_profiles:
        by_parent[candidate["parent_boundary_id"]].append(candidate)
    combined: dict[str, list[dict]] = {}
    profile_rows: list[dict] = []
    for parent_id in sorted(by_parent):
        candidates = by_parent[parent_id]
        conflict_bins: set[int] = set()
        for first_index, first in enumerate(candidates):
            first_bins = {row["bin_index"]: row for row in first["profile"]}
            for second in candidates[first_index + 1:]:
                second_bins = {row["bin_index"]: row for row in second["profile"]}
                overlap = sorted(set(first_bins).intersection(second_bins))
                if not overlap:
                    continue
                differences = np.asarray([
                    abs(first_bins[key]["offset_m"] - second_bins[key]["offset_m"])
                    for key in overlap
                ])
                incompatible_color = first["lane_color"] != second["lane_color"]
                if incompatible_color or float(np.quantile(differences, 0.95)) > conflict_limit:
                    conflict_bins.update(overlap)
        values_by_bin: dict[int, list[tuple[dict, dict]]] = defaultdict(list)
        for candidate in candidates:
            for row in candidate["profile"]:
                values_by_bin[int(row["bin_index"])].append((candidate, row))
        rows: list[dict] = []
        for bin_id in sorted(values_by_bin):
            values = values_by_bin[bin_id]
            is_conflict = bin_id in conflict_bins or len({item[0]["lane_color"] for item in values}) > 1
            offsets = [item[1]["offset_m"] for item in values]
            contributors = sorted({item[0]["source_file"] for item in values})
            source_kinds = sorted({item[0]["source_kind"] for item in values})
            colors = sorted({item[0]["lane_color"] for item in values})
            station = float(np.median([item[1]["station_m"] for item in values]))
            combined_row = {
                "parent_boundary_id": parent_id,
                "bin_index": bin_id,
                "station_m": station,
                "offset_m": float(np.median(offsets)),
                "support": sum(int(item[1]["support"]) for item in values),
                "lane_color": colors[0] if len(colors) == 1 else "CONFLICT",
                "source_files": contributors,
                "source_kinds": source_kinds,
                "conflict": is_conflict,
                "applied": not is_conflict,
            }
            profile_rows.append(combined_row)
            if not is_conflict:
                rows.append(combined_row)
            else:
                for candidate, _ in values:
                    report_rows[candidate["report_index"]]["match_status"] = "review_required"
                    report_rows[candidate["report_index"]]["reason"] = "source_conflict_not_applied"
        combined[parent_id] = rows
    return combined, profile_rows


def _aggregate_hash(values: list[str]) -> str:
    digest = hashlib.sha256()
    for value in sorted(set(values)):
        digest.update(value.encode("ascii"))
        digest.update(b"\n")
    return digest.hexdigest()


def _copy_feature(feature: dict, old_map_id: str, new_map_id: str) -> dict:
    copied = json.loads(json.dumps(feature))
    prefix = old_map_id + ":"
    suffix = feature["id"][len(prefix):] if feature["id"].startswith(prefix) else feature["id"]
    copied["id"] = f"{new_map_id}:{suffix}"
    copied["status"] = "candidate"
    if copied["feature_type"] in {"lane_boundary", "stopline"}:
        copied.setdefault("validation", {})["operational"] = False
    return copied


def _split_boundaries(parent: dict, boundaries: list[dict], combined: dict[str, list[dict]], accepted_profiles: list[dict], config: dict) -> tuple[list[dict], dict[str, list[tuple[float, float]]], dict[str, np.ndarray]]:
    old_map_id = str(parent["map_id"])
    new_map_id = str(config["map_id"])
    max_gap = float(config["profile"]["maximum_interpolation_gap_m"])
    candidate_by_parent: dict[str, list[dict]] = defaultdict(list)
    for candidate in accepted_profiles:
        candidate_by_parent[candidate["parent_boundary_id"]].append(candidate)
    output: list[dict] = []
    applied_ranges: dict[str, list[tuple[float, float]]] = defaultdict(list)
    full_adjusted: dict[str, np.ndarray] = {}
    for parent_feature in boundaries:
        parent_id = parent_feature["id"]
        raw_points = parent_feature["geometry"]["coordinates"]
        full_points, full_tangents = resample_polyline(raw_points, 0.25)
        full_stations = cumulative_distance(full_points)
        profile = combined.get(parent_id, [])
        runs = [run for run in profile_runs(profile, max_gap) if len(run) >= 2]
        ranges = [(float(run[0]["station_m"]), float(run[-1]["station_m"])) for run in runs]
        applied_ranges[parent_id] = ranges
        adjusted_offsets = [interpolate_profile(profile, float(station), max_gap) for station in full_stations]
        full_adjusted[parent_id] = offset_interval(
            full_points, full_tangents,
            [0.0 if value is None else value for value in adjusted_offsets],
        )
        length = float(full_stations[-1])
        intervals: list[tuple[float, float, str, list[dict]]] = []
        cursor = 0.0
        for run, (start, end) in zip(runs, ranges):
            start = max(cursor, start)
            end = min(length, end)
            if start > cursor + 1e-6:
                intervals.append((cursor, start, "link_nominal", []))
            if end > start + 1e-6:
                intervals.append((start, end, "observed_calibrated", run))
            cursor = max(cursor, end)
        if cursor < length - 1e-6:
            intervals.append((cursor, length, "link_nominal", []))
        if not intervals:
            intervals = [(0.0, length, "link_nominal", [])]
        for interval_index, (start, end, geometry_status, run) in enumerate(intervals):
            points, tangents, stations = extract_polyline_interval(raw_points, start, end, 0.25)
            feature = _copy_feature(parent_feature, old_map_id, new_map_id)
            parent_suffix = parent_id.split(":lane_boundary:", 1)[-1]
            feature["id"] = f"{new_map_id}:lane_boundary_interval:{parent_suffix}:{interval_index:03d}"
            feature["attributes"]["parent_boundary_id"] = parent_id
            feature["attributes"]["parent_station_start_m"] = start
            feature["attributes"]["parent_station_end_m"] = end
            feature["attributes"]["station_interval_convention"] = "closed_geometry_half_open_audit"
            feature["attributes"]["geometry_status"] = geometry_status
            feature["attributes"]["lane_shape"] = "unknown"
            feature.setdefault("validation", {})["operational"] = False
            feature["validation"]["observed_calibration_stage"] = 2
            if geometry_status == "observed_calibrated":
                offsets = [interpolate_profile(profile, float(station), max_gap) for station in stations]
                if any(value is None for value in offsets):
                    raise RuntimeError(f"profile interpolation gap inside interval: {parent_id}")
                points = offset_interval(points, tangents, offsets)
                colors = sorted({row["lane_color"] for row in run})
                source_files = sorted({name for row in run for name in row["source_files"]})
                source_kinds = sorted({name for row in run for name in row["source_kinds"]})
                hashes = [
                    item["source_hash"] for item in candidate_by_parent[parent_id]
                    if item["source_file"] in source_files
                ]
                representative_kind = source_kinds[0]
                feature["attributes"]["lane_color"] = colors[0] if len(colors) == 1 else "unknown"
                feature["attributes"]["width_source"] = "observed_boundary_offset"
                feature["attributes"]["observed_source_files"] = source_files
                feature["attributes"]["observed_source_kinds"] = source_kinds
                feature["provenance"] = {
                    "source_kind": representative_kind,
                    "source_id": "+".join(source_files),
                    "source_sha256": _aggregate_hash(hashes),
                    "transform": "robust_signed_normal_offset_from_parent_boundary",
                }
                feature["validation"].update({
                    "painted_lane_ground_truth": False,
                    "source_kind_is_representative_when_fused": len(source_kinds) > 1,
                    "contributing_source_kinds": source_kinds,
                    "conflict_bins_applied": False,
                    "extrapolated": False,
                })
            feature["geometry"]["coordinates"] = _plain(points)
            output.append(feature)
    return output, dict(applied_ranges), full_adjusted


def _station_in_ranges(station: float, ranges: list[tuple[float, float]]) -> bool:
    return any(start - 1e-6 <= station <= end + 1e-6 for start, end in ranges)


def _width_profiles(parent: dict, boundaries: list[dict], applied_ranges: dict[str, list[tuple[float, float]]], adjusted: dict[str, np.ndarray]) -> list[dict]:
    by_link_side: dict[tuple[str, str], str] = {}
    for boundary in boundaries:
        for link_id, side in (boundary.get("attributes", {}).get("link_sides") or {}).items():
            by_link_side[(str(link_id), str(side))] = boundary["id"]
    rows: list[dict] = []
    for link in sorted(
        (feature for feature in parent["features"] if feature["feature_type"] == "link_centerline"),
        key=lambda item: item["id"],
    ):
        link_id = str(link["topology"]["current_link_ids"][0])
        left_id = by_link_side.get((link_id, "left"))
        right_id = by_link_side.get((link_id, "right"))
        if not left_id or not right_id or not applied_ranges.get(left_id) or not applied_ranges.get(right_id):
            continue
        left_index = PolylineProjectionIndex({left_id: adjusted[left_id]}, cell_size_m=2.0)
        right_index = PolylineProjectionIndex({right_id: adjusted[right_id]}, cell_size_m=2.0)
        centers, tangents = resample_polyline(link["geometry"]["coordinates"], 1.0)
        center_stations = cumulative_distance(centers)
        for center, tangent, center_station in zip(centers, tangents, center_stations):
            left = left_index.query(center, tangent, max_distance_m=5.0, max_heading_deg=45.0)
            right = right_index.query(center, tangent, max_distance_m=5.0, max_heading_deg=45.0)
            if not left or not right:
                continue
            left_projection, right_projection = left[0], right[0]
            if not _station_in_ranges(left_projection.station_m, applied_ranges[left_id]):
                continue
            if not _station_in_ranges(right_projection.station_m, applied_ranges[right_id]):
                continue
            tangent_xy = np.asarray(tangent[:2], dtype=np.float64)
            tangent_xy /= max(float(np.linalg.norm(tangent_xy)), 1e-12)
            normal_xy = np.asarray([-tangent_xy[1], tangent_xy[0]])
            left_delta = np.asarray(left_projection.point_xyz[:2]) - center[:2]
            right_delta = np.asarray(right_projection.point_xyz[:2]) - center[:2]
            left_lateral = float(np.dot(left_delta, normal_xy))
            right_lateral = float(np.dot(right_delta, normal_xy))
            # A width sample is valid only when both calibrated boundaries
            # bracket this centerline station along the current link normal.
            # This rejects endpoint-nearest projections and nearby curves that
            # happen to belong to the same parent boundary IDs.
            if left_lateral <= 0.25 or right_lateral >= -0.25:
                continue
            if abs(float(np.dot(left_delta, tangent_xy))) > 1.0:
                continue
            if abs(float(np.dot(right_delta, tangent_xy))) > 1.0:
                continue
            width = left_lateral - right_lateral
            rows.append({
                "current_link_id": link_id,
                "link_station_m": float(center_station),
                "observed_width_m": width,
                "left_parent_boundary_id": left_id,
                "right_parent_boundary_id": right_id,
                "left_boundary_station_m": left_projection.station_m,
                "right_boundary_station_m": right_projection.station_m,
                "width_status": "both_sides_observed_calibrated",
            })
    return rows


def _render_overlay(path: Path, parent_features: list[dict], result_features: list[dict], segments: list[dict], report_rows: list[dict], link_roles: set[str] | None = None, review_only: bool = False) -> None:
    selected_parent = []
    if link_roles:
        link_ids = {
            str(feature["topology"]["current_link_ids"][0])
            for feature in parent_features if feature["feature_type"] == "link_centerline"
            and link_roles.intersection(feature.get("attributes", {}).get("corridor_roles", []))
        }
        selected_parent = [
            feature for feature in parent_features if feature["feature_type"] == "lane_boundary"
            and link_ids.intersection(feature["topology"]["current_link_ids"])
        ]
    else:
        selected_parent = [feature for feature in parent_features if feature["feature_type"] == "lane_boundary"]
    parent_ids = {feature["id"] for feature in selected_parent}
    selected_result = [
        feature for feature in result_features if feature["feature_type"] == "lane_boundary"
        and feature["attributes"].get("parent_boundary_id") in parent_ids
    ]
    # A source segment can traverse several parent boundaries.  Show its worst
    # run status so an accepted sub-run cannot hide a rejected or unmatched
    # portion in the review overlay.
    status_priority = {"automatic": 0, "review_required": 1, "rejected": 2, "unmatched": 3}
    report_status: dict[str, str] = {}
    for row in report_rows:
        segment_id = row["segment_id"]
        status = row["match_status"]
        previous = report_status.get(segment_id)
        if previous is None or status_priority.get(status, 99) > status_priority.get(previous, 99):
            report_status[segment_id] = status
    if review_only:
        selected_segments = [segment for segment in segments if report_status.get(segment["segment_id"]) in {"review_required", "rejected", "unmatched"}]
    else:
        selected_segments = [segment for segment in segments if any(
            row["segment_id"] == segment["segment_id"] and (not parent_ids or row["parent_boundary_id"] in parent_ids)
            for row in report_rows
        )]
    coordinate_sets = [points_xyz(feature["geometry"]["coordinates"]) for feature in selected_parent]
    coordinate_sets += [segment["points"] for segment in selected_segments]
    coordinate_sets = [value for value in coordinate_sets if len(value)]
    if not coordinate_sets:
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(image, "NO FEATURES", (30, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
        cv2.imwrite(str(path), image)
        return
    cloud = np.vstack(coordinate_sets)
    low = np.min(cloud[:, :2], axis=0) - 8.0
    high = np.max(cloud[:, :2], axis=0) + 8.0
    scale = min(1800.0 / max(high[0] - low[0], 1.0), 1400.0 / max(high[1] - low[1], 1.0), 5.0)
    width = max(640, int(math.ceil((high[0] - low[0]) * scale)))
    height = max(480, int(math.ceil((high[1] - low[1]) * scale)))
    image = np.full((height, width, 3), 24, dtype=np.uint8)

    def pixels(raw: Any) -> np.ndarray:
        points = points_xyz(raw)
        result = np.empty((len(points), 2), dtype=np.int32)
        result[:, 0] = np.round((points[:, 0] - low[0]) * scale).astype(np.int32)
        result[:, 1] = np.round((high[1] - points[:, 1]) * scale).astype(np.int32)
        return result

    for feature in selected_parent:
        cv2.polylines(image, [pixels(feature["geometry"]["coordinates"])], False, (100, 100, 100), 1, cv2.LINE_AA)
    for feature in selected_result:
        if feature["attributes"].get("geometry_status") == "observed_calibrated":
            color = (40, 220, 40) if feature["attributes"].get("lane_color") == "WHITE" else (20, 210, 240)
            cv2.polylines(image, [pixels(feature["geometry"]["coordinates"])], False, color, 2, cv2.LINE_AA)
    for segment in selected_segments:
        status = report_status.get(segment["segment_id"], "unmatched")
        color = {"automatic": (255, 180, 40), "review_required": (0, 165, 255), "rejected": (40, 40, 240), "unmatched": (180, 60, 220)}.get(status, (180, 180, 180))
        cv2.polylines(image, [pixels(segment["points"])], False, color, 1, cv2.LINE_AA)
    legend = (
        "gray: nominal  green/yellow: calibrated",
        "blue: raw-auto  orange: review",
        "red: rejected  magenta: unmatched",
    )
    for line_index, line in enumerate(legend):
        cv2.putText(
            image, line, (12, 24 + 20 * line_index),
            cv2.FONT_HERSHEY_SIMPLEX, 0.42, (235, 235, 235), 1, cv2.LINE_AA,
        )
    path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(path), image)


def build_observed_calibration(config_path: Path, repo_root: Path, output_dir: Path) -> dict:
    with config_path.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream) or {}
    if config.get("schema_version") != CONFIG_SCHEMA:
        raise ValueError("unsupported observed calibration config schema")
    if len(config.get("sources") or []) != 19:
        raise ValueError("observed calibration requires exactly 19 explicit sources")
    parent_path = resolve_source(repo_root, config["parent"]["path"])
    actual_parent_hash = sha256_file(parent_path)
    if actual_parent_hash != str(config["parent"]["sha256"]):
        raise ValueError(f"parent candidate hash mismatch: {actual_parent_hash}")
    with parent_path.open("r", encoding="utf-8") as stream:
        parent = json.load(stream)
    if parent.get("map_id") != config["parent"]["expected_map_id"]:
        raise ValueError("unexpected parent candidate map_id")
    if parent.get("build", {}).get("observed_or_legacy_inputs_used") is not False:
        raise ValueError("parent must be current-only")
    audit_rows, direct_rows, observed_records = _load_observations(config, repo_root)
    _validate_input_contract(config, audit_rows)
    segments = _observation_segments(direct_rows, config)
    boundaries = [feature for feature in parent["features"] if feature["feature_type"] == "lane_boundary"]
    match_rows, accepted_profiles = _match_segments(segments, boundaries, parent, config)
    combined, profile_rows = _combine_profiles(accepted_profiles, config, match_rows)
    calibrated_boundaries, applied_ranges, adjusted = _split_boundaries(
        parent, boundaries, combined, accepted_profiles, config,
    )
    width_rows = _width_profiles(parent, boundaries, applied_ranges, adjusted)

    old_map_id = str(parent["map_id"])
    new_map_id = str(config["map_id"])
    static_features = [
        _copy_feature(feature, old_map_id, new_map_id)
        for feature in parent["features"] if feature["feature_type"] != "lane_boundary"
    ]
    features = static_features + calibrated_boundaries
    parent_record = source_record(repo_root, config["parent"]["path"], parent_path)
    canonical = {
        "schema_version": SCHEMA_VERSION,
        "map_id": new_map_id,
        "frame": parent["frame"],
        "sources": {"parent_nominal_candidate_map": parent_record, **observed_records},
        "build": {
            "build_id": config["build_id"],
            "stage": "observed_lane_boundary_calibration_candidate",
            "operational": False,
            "parent_map_id": old_map_id,
            "parent_sha256": actual_parent_hash,
            "observed_inputs_used": True,
            "legacy_katri_inputs_used": False,
            "manual_bridges_used": False,
            "raster_inward_offset_used": False,
        },
        "features": sorted(features, key=lambda feature: feature["id"]),
    }
    validate_canonical_document(canonical)

    status_counts = Counter(row["match_status"] for row in match_rows)
    calibrated_intervals = [
        feature for feature in calibrated_boundaries
        if feature["attributes"]["geometry_status"] == "observed_calibrated"
    ]
    nominal_intervals = [
        feature for feature in calibrated_boundaries
        if feature["attributes"]["geometry_status"] == "link_nominal"
    ]
    parent_counts = Counter(feature["feature_type"] for feature in parent["features"])
    result_counts = Counter(feature["feature_type"] for feature in features)
    summary = {
        "build_id": config["build_id"],
        "map_id": new_map_id,
        "status": "candidate_observed_calibration",
        "operational": False,
        "parent": {"map_id": old_map_id, "sha256": actual_parent_hash},
        "input_contract": {
            "source_count": len(audit_rows),
            "total_rows": sum(row["total_rows"] for row in audit_rows),
            "direct_rows": sum(row["direct_rows"] for row in audit_rows),
            "generated_audit_only_rows": sum(row["generated_rows"] for row in audit_rows),
            "interpolated_audit_only_rows": sum(row["interpolated_rows"] for row in audit_rows),
            "current_lidar_semantic_rows": sum(row["direct_rows"] for row in audit_rows if row["source_kind"] == "current_lidar_semantic"),
            "current_gps_direct_rows": sum(row["direct_rows"] for row in audit_rows if row["source_kind"] == "current_gps_direct"),
            "manual_bridges_used": False,
            "raster_inward_offset_m": 0.0,
        },
        "segmentation": {
            "segment_count": len(segments),
            "resampled_point_count": sum(len(segment["points"]) for segment in segments),
        },
        "matching": {
            "run_status_counts": dict(sorted(status_counts.items())),
            "automatic_profile_count_before_conflict": len(accepted_profiles),
            "profile_bin_count": len(profile_rows),
            "applied_profile_bin_count": sum(bool(row["applied"]) for row in profile_rows),
            "conflict_profile_bin_count": sum(bool(row["conflict"]) for row in profile_rows),
        },
        "candidate_map": {
            "parent_feature_counts": dict(sorted(parent_counts.items())),
            "result_feature_counts": dict(sorted(result_counts.items())),
            "calibrated_interval_count": len(calibrated_intervals),
            "nominal_remainder_interval_count": len(nominal_intervals),
            "calibrated_parent_boundary_count": sum(bool(ranges) for ranges in applied_ranges.values()),
            "link_width_sample_count": len(width_rows),
            "route_link_count_unchanged": parent_counts["link_centerline"] == result_counts["link_centerline"] == 70,
            "stopline_count_unchanged": parent_counts["stopline"] == result_counts["stopline"] == 13,
        },
        "limitations": [
            "lane shape remains unknown",
            "one-sided observations are not lane-width measurements",
            "review_required and conflict profiles are not applied",
            "candidate map is non-operational",
        ],
    }

    _write_json(output_dir / "summary.json", summary)
    _write_json(output_dir / "observed_candidate_map.json", canonical)
    _write_csv(output_dir / "source_audit.csv", audit_rows, [
        "source_index", "source_file", "configured_path", "source_kind", "sha256",
        "total_rows", "direct_rows", "generated_rows", "interpolated_rows",
        "fit_support_rows", "audit_only_rows", "manual_bridge_rows",
        "canonical_inward_offset_m",
    ])
    match_fields = [
        "segment_id", "run_index", "source_group", "source_file", "source_kind",
        "collection_side", "lane_color", "parent_boundary_id", "match_status", "reason",
        "station_start_m", "station_end_m", "observed_length_m", "resampled_points",
        "coverage", "nominal_distance_p50_m", "nominal_distance_p95_m",
        "fit_residual_p95_m", "outlier_ratio", "maximum_abs_correction_m",
        "maximum_offset_slope_m_per_m",
    ]
    _write_csv(output_dir / "segment_matches.csv", match_rows, match_fields)
    flattened_profiles = [{
        **row,
        "source_files": ";".join(row["source_files"]),
        "source_kinds": ";".join(row["source_kinds"]),
    } for row in profile_rows]
    _write_csv(output_dir / "boundary_offset_profiles.csv", flattened_profiles, [
        "parent_boundary_id", "bin_index", "station_m", "offset_m", "support",
        "lane_color", "source_files", "source_kinds", "conflict", "applied",
    ])
    _write_csv(output_dir / "link_width_profiles.csv", width_rows, [
        "current_link_id", "link_station_m", "observed_width_m",
        "left_parent_boundary_id", "right_parent_boundary_id",
        "left_boundary_station_m", "right_boundary_station_m", "width_status",
    ])
    overlay_dir = output_dir / "overlays"
    _render_overlay(overlay_dir / "full_corridor.png", parent["features"], features, segments, match_rows)
    _render_overlay(overlay_dir / "high_speed.png", parent["features"], features, segments, match_rows, {"high_speed_parallel"})
    _render_overlay(overlay_dir / "shadow_and_static.png", parent["features"], features, segments, match_rows, {"shadow_primary", "shadow_opposite", "shadow_connector", "static_obstacle_primary", "static_obstacle_opposite", "static_obstacle_connector"})
    _render_overlay(overlay_dir / "review_required.png", parent["features"], features, segments, match_rows, review_only=True)
    return summary
