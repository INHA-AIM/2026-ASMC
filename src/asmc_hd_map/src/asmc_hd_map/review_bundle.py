"""Tri-source review bundle for current, observed, and legacy HD-map inputs."""

from __future__ import annotations

from collections import Counter, defaultdict
import csv
import json
import math
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import yaml

from .current_builder import _write_csv, _write_json
from .frames import LocalMapFrame, origin_translation
from .geometry import cumulative_distance, points_xyz, quantile_or_none, resample_polyline
from .io_mgeo import load_json, resolve_source, sha256_file
from .observed_builder import _load_observations, _observation_segments
from .spatial import GridIndex


CONFIG_SCHEMA = "asmc.hd_map.candidate_review_config/v1"
STATUS_PRIORITY = {"automatic": 0, "review_required": 1, "rejected": 2, "unmatched": 3}


def _read_csv(path: Path) -> list[dict]:
    with path.open("r", encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))


def _canonical_points(feature: dict) -> np.ndarray:
    return points_xyz((feature.get("geometry") or {}).get("coordinates", ()))


def _legacy_color(feature: dict) -> str:
    values = feature.get("lane_color") or []
    if isinstance(values, str):
        values = [values]
    lowered = {str(value).strip().lower() for value in values}
    for known in ("yellow", "white", "blue"):
        if known in lowered:
            return known
    return "unknown"


def _load_legacy_references(config: dict, repo_root: Path) -> list[dict]:
    legacy_config_path = resolve_source(repo_root, config["sources"]["legacy_config"])
    with legacy_config_path.open("r", encoding="utf-8") as stream:
        legacy_config = yaml.safe_load(stream) or {}
    current_global = resolve_source(
        repo_root, legacy_config["sources"]["current"]["global_info"],
    )
    legacy_global = resolve_source(
        repo_root, legacy_config["sources"]["legacy"]["global_info"],
    )
    translation = origin_translation(
        LocalMapFrame.from_global_info(load_json(legacy_global)),
        LocalMapFrame.from_global_info(load_json(current_global)),
    )
    lane_path = resolve_source(
        repo_root, legacy_config["sources"]["legacy"]["lane_boundary_set"],
    )
    lanes_by_id = {str(row.get("idx")): row for row in load_json(lane_path)}
    audit_path = resolve_source(repo_root, config["sources"]["legacy_lane_audit"])
    references: list[dict] = []
    for audit in _read_csv(audit_path):
        tier = str(audit.get("tier", "D"))
        if tier not in {"A", "B"}:
            continue
        source_id = str(audit["source_id"])
        source = lanes_by_id.get(source_id)
        if source is None:
            raise ValueError(f"legacy lane audit references missing feature: {source_id}")
        coordinates = points_xyz(source.get("points", ())).copy()
        coordinates += translation.reshape(1, 3)
        raw_lane_type = source.get("lane_type", "unknown")
        lane_types = raw_lane_type if isinstance(raw_lane_type, (list, tuple)) else [raw_lane_type]
        references.append({
            "id": source_id,
            "tier": tier,
            "color": _legacy_color(source),
            "shape": ";".join(str(value) for value in source.get("lane_shape") or ["unknown"]),
            "lane_type": ";".join(str(value) for value in lane_types),
            "dash_length_m": float(source.get("dash_interval_L1", 3.0)),
            "dash_gap_m": float(source.get("dash_interval_L2", 3.0)),
            "line_width_m": float(source.get("lane_width", 0.15)),
            "target_current_link_ids": [
                value for value in str(audit.get("target_current_ids", "")).split(";") if value
            ],
            "coordinates": coordinates,
        })
    return references


def _worst_segment_status(match_rows: list[dict]) -> dict[str, str]:
    result: dict[str, str] = {}
    for row in match_rows:
        segment_id = str(row["segment_id"])
        status = str(row["match_status"])
        previous = result.get(segment_id)
        if previous is None or STATUS_PRIORITY.get(status, 99) > STATUS_PRIORITY.get(previous, 99):
            result[segment_id] = status
    return result


def load_review_layers(config_path: Path, repo_root: Path) -> dict:
    with config_path.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream) or {}
    if config.get("schema_version") != CONFIG_SCHEMA:
        raise ValueError("unsupported candidate review config schema")
    source_paths = {
        key: resolve_source(repo_root, raw) for key, raw in config["sources"].items()
    }
    nominal = load_json(source_paths["nominal_map"])
    observed = load_json(source_paths["observed_map"])
    adopted = load_json(source_paths["legacy_adopted_map"])
    with source_paths["observed_config"].open("r", encoding="utf-8") as stream:
        observed_config = yaml.safe_load(stream) or {}
    _, direct_rows, _ = _load_observations(observed_config, repo_root)
    observation_segments = _observation_segments(direct_rows, observed_config)
    match_rows = _read_csv(source_paths["segment_matches"])
    segment_status = _worst_segment_status(match_rows)
    for segment in observation_segments:
        segment["review_status"] = segment_status.get(segment["segment_id"], "unmatched")
    route_points = []
    with source_paths["competition_route"].open("r", encoding="utf-8") as stream:
        for line in stream:
            values = line.strip().replace(",", " ").split()
            if len(values) >= 2:
                route_points.append([float(values[0]), float(values[1]), float(values[2]) if len(values) > 2 else 0.0])
    current_links = [
        feature for feature in nominal["features"] if feature["feature_type"] == "link_centerline"
    ]
    legacy_references = _load_legacy_references(config, repo_root)
    comparison = config.get("comparison") or {}
    if bool(comparison.get("legacy_current_corridor_only", False)):
        corridor_ids = {
            str(feature["topology"]["current_link_ids"][0]) for feature in current_links
        }
        legacy_references = [
            feature for feature in legacy_references
            if set(feature["target_current_link_ids"]) & corridor_ids
        ]
    return {
        "config": config,
        "source_paths": source_paths,
        "source_hashes": {key: sha256_file(path) for key, path in source_paths.items()},
        "nominal_document": nominal,
        "observed_document": observed,
        "current_links": current_links,
        "nominal_boundaries": [feature for feature in nominal["features"] if feature["feature_type"] == "lane_boundary"],
        "observed_boundaries": [feature for feature in observed["features"] if feature["feature_type"] == "lane_boundary" and feature.get("attributes", {}).get("geometry_status") == "observed_calibrated"],
        # Stop-line node flags need painted-line review.  The latest reviewed
        # candidate may therefore contain fewer stop lines than the stage-2
        # observed document.
        "stoplines": [feature for feature in adopted["features"] if feature["feature_type"] == "stopline"],
        "adopted_legacy_boundaries": [
            feature for feature in adopted["features"]
            if feature["feature_type"] == "lane_marking"
            or (
                feature["feature_type"] == "lane_boundary" and (
                str(feature.get("attributes", {}).get("geometry_status", "")).startswith("legacy_validated")
                or str(feature.get("attributes", {}).get("geometry_status", "")).startswith("current_reconstructed")
                )
            )
        ],
        "changed_legacy_candidates": list(
            (adopted.get("review_layers") or {}).get("changed_region_legacy_candidates", [])
        ) + list(
            (adopted.get("review_layers") or {}).get("legacy_template_review_required", [])
        ),
        "legacy_references": legacy_references,
        "observation_segments": observation_segments,
        "match_rows": match_rows,
        "route": np.asarray(route_points, dtype=np.float64),
    }


def lane_change_reachable_high_speed_links(current_links: list[dict]) -> set[str]:
    by_id = {
        str(feature["topology"]["current_link_ids"][0]): feature
        for feature in current_links
    }
    high_speed_ids = {
        link_id for link_id, feature in by_id.items()
        if "high_speed_parallel" in feature.get("attributes", {}).get("corridor_roles", [])
    }
    seeds = {
        link_id for link_id in high_speed_ids
        if "official_route" in by_id[link_id].get("attributes", {}).get("corridor_roles", [])
    }
    reached = set(seeds)
    frontier = set(seeds)
    while frontier:
        next_frontier: set[str] = set()
        for link_id in frontier:
            topology = by_id[link_id].get("topology") or {}
            for key in ("left_adjacent_link_id", "right_adjacent_link_id"):
                neighbor = topology.get(key)
                if neighbor in high_speed_ids and neighbor not in reached:
                    reached.add(str(neighbor))
                    next_frontier.add(str(neighbor))
        frontier = next_frontier
    return reached


def high_speed_inventory(layers: dict) -> list[dict]:
    current_links = layers["current_links"]
    high_speed = [
        feature for feature in current_links
        if "high_speed_parallel" in feature.get("attributes", {}).get("corridor_roles", [])
    ]
    reachable = lane_change_reachable_high_speed_links(current_links)
    nominal_by_link: dict[str, list[dict]] = defaultdict(list)
    for boundary in layers["nominal_boundaries"]:
        for link_id in boundary["topology"]["current_link_ids"]:
            nominal_by_link[str(link_id)].append(boundary)
    observed_by_link: dict[str, list[dict]] = defaultdict(list)
    for boundary in layers["observed_boundaries"]:
        for link_id in boundary["topology"]["current_link_ids"]:
            observed_by_link[str(link_id)].append(boundary)
    legacy_by_link: dict[str, list[dict]] = defaultdict(list)
    for reference in layers["legacy_references"]:
        for link_id in reference["target_current_link_ids"]:
            legacy_by_link[link_id].append(reference)
    rows: list[dict] = []
    for feature in sorted(high_speed, key=lambda item: item["id"]):
        link_id = str(feature["topology"]["current_link_ids"][0])
        attributes = feature["attributes"]
        points = _canonical_points(feature)
        nominal = nominal_by_link.get(link_id, [])
        observed = observed_by_link.get(link_id, [])
        nominal_length = sum(
            float(cumulative_distance(_canonical_points(boundary))[-1])
            for boundary in nominal if len(_canonical_points(boundary))
        )
        observed_length = sum(
            max(
                0.0,
                float(boundary.get("attributes", {}).get("parent_station_end_m", 0.0))
                - float(boundary.get("attributes", {}).get("parent_station_start_m", 0.0)),
            )
            for boundary in observed
        )
        sides = sorted({
            boundary.get("attributes", {}).get("link_sides", {}).get(link_id, "unknown")
            for boundary in nominal
        })
        rows.append({
            "current_link_id": link_id,
            "road_id": attributes.get("road_id"),
            "ego_lane": attributes.get("ego_lane"),
            "relative_direction": attributes.get("relative_direction"),
            "official_route": "official_route" in attributes.get("corridor_roles", []),
            "lane_change_reachable": link_id in reachable,
            "geometry_scope": attributes.get("geometry_scope"),
            "exported_length_m": float(cumulative_distance(points)[-1]) if len(points) else 0.0,
            "nominal_boundary_count": len(nominal),
            "nominal_boundary_sides": ";".join(sides),
            "observed_calibrated_interval_count": len(observed),
            "observed_calibrated_length_m": observed_length,
            "observed_boundary_coverage_ratio": (
                min(1.0, observed_length / nominal_length) if nominal_length > 0.0 else 0.0
            ),
            "legacy_tier_a_count": sum(item["tier"] == "A" for item in legacy_by_link.get(link_id, [])),
            "legacy_tier_b_count": sum(item["tier"] == "B" for item in legacy_by_link.get(link_id, [])),
            "inventory_status": (
                "centerline_only_boundary_review"
                if not nominal else
                "observed_calibrated" if observed else "nominal_boundary_only"
            ),
        })
    return rows


def observed_legacy_agreement(layers: dict, allowed_link_ids: set[str]) -> dict:
    """Measure observed-to-legacy proximity for review, never as a promotion gate."""

    distances: list[float] = []
    sample_count = 0
    interval_count = 0
    comparable_interval_count = 0
    for feature in layers["observed_boundaries"]:
        feature_links = {
            str(value) for value in feature.get("topology", {}).get("current_link_ids", [])
        }.intersection(allowed_link_ids)
        if not feature_links:
            continue
        interval_count += 1
        references = [
            reference for reference in layers["legacy_references"]
            if feature_links.intersection(reference["target_current_link_ids"])
        ]
        if not references:
            continue
        observed_points, observed_tangents = resample_polyline(
            _canonical_points(feature), 0.25,
        )
        reference_samples = [resample_polyline(reference["coordinates"], 0.25) for reference in references]
        reference_samples = [value for value in reference_samples if len(value[0])]
        if not len(observed_points) or not reference_samples:
            continue
        reference_points = np.vstack([value[0] for value in reference_samples])
        reference_tangents = np.vstack([value[1] for value in reference_samples])
        index = GridIndex(reference_points, reference_tangents, cell_size_m=2.0)
        comparable_interval_count += 1
        sample_count += len(observed_points)
        for point, tangent in zip(observed_points, observed_tangents):
            distance, _, _ = index.query(
                point, tangent, max_distance_m=3.0, max_heading_deg=30.0,
            )
            if math.isfinite(distance):
                distances.append(distance)
    values = np.asarray(distances, dtype=np.float64)
    return {
        "interpretation": "review_only_observed_to_legacy_nearest_distance",
        "promotion_gate": False,
        "observed_interval_count": interval_count,
        "comparable_interval_count": comparable_interval_count,
        "sample_count": sample_count,
        "matched_within_3m_count": len(values),
        "matched_within_3m_ratio": len(values) / sample_count if sample_count else 0.0,
        "distance_p50_m": quantile_or_none(values, 0.50),
        "distance_p95_m": quantile_or_none(values, 0.95),
        "within_0_30m_ratio": float(np.mean(values <= 0.30)) if len(values) else None,
        "within_0_75m_ratio": float(np.mean(values <= 0.75)) if len(values) else None,
    }


def _feature_in_region(feature: dict, road_ids: set[str], roles: set[str]) -> bool:
    attributes = feature.get("attributes") or {}
    return (
        (road_ids and str(attributes.get("road_id")) in road_ids)
        or (roles and bool(roles.intersection(attributes.get("corridor_roles", []))))
    )


def _region_bounds(layers: dict, region: dict) -> tuple[np.ndarray, np.ndarray]:
    road_ids = {str(value) for value in region.get("road_ids", [])}
    roles = {str(value) for value in region.get("corridor_roles", [])}
    selected = [
        _canonical_points(feature) for feature in layers["current_links"]
        if _feature_in_region(feature, road_ids, roles)
    ]
    selected = [points for points in selected if len(points)]
    if not selected:
        raise ValueError(f"review region selected no current links: {region}")
    cloud = np.vstack(selected)[:, :2]
    margin = float(region.get("margin_m", 12.0))
    return np.min(cloud, axis=0) - margin, np.max(cloud, axis=0) + margin


def _intersects(points: np.ndarray, low: np.ndarray, high: np.ndarray) -> bool:
    if len(points) == 0:
        return False
    point_low = np.min(points[:, :2], axis=0)
    point_high = np.max(points[:, :2], axis=0)
    return bool(np.all(point_high >= low) and np.all(point_low <= high))


def _draw_polyline(image: np.ndarray, points: np.ndarray, transform, color, thickness: int, dashed: bool = False) -> None:
    if len(points) < 2:
        return
    pixels = transform(points)
    if dashed:
        for index in range(0, len(pixels) - 1, 2):
            cv2.line(image, tuple(pixels[index]), tuple(pixels[index + 1]), color, thickness, cv2.LINE_AA)
    else:
        cv2.polylines(image, [pixels], False, color, thickness, cv2.LINE_AA)


def render_review_overlay(
    path: Path,
    layers: dict,
    low: np.ndarray,
    high: np.ndarray,
    title: str,
    visual_override: dict | None = None,
) -> dict:
    visual = dict(layers["config"]["visualization"])
    visual.update(visual_override or {})
    width, height, padding = int(visual["width_px"]), int(visual["height_px"]), int(visual["padding_px"])
    span = np.maximum(high - low, 1.0)
    scale = min((width - 2 * padding) / span[0], (height - 2 * padding) / span[1])
    center = 0.5 * (low + high)
    draw_low = center - np.asarray([(width - 2 * padding) / scale, (height - 2 * padding) / scale]) / 2.0
    draw_high = center + np.asarray([(width - 2 * padding) / scale, (height - 2 * padding) / scale]) / 2.0
    image = np.full((height, width, 3), 22, dtype=np.uint8)

    def transform(points: np.ndarray) -> np.ndarray:
        result = np.empty((len(points), 2), dtype=np.int32)
        result[:, 0] = np.round(padding + (points[:, 0] - draw_low[0]) * scale).astype(np.int32)
        result[:, 1] = np.round(padding + (draw_high[1] - points[:, 1]) * scale).astype(np.int32)
        return result

    counts = Counter()
    for feature in layers["current_links"]:
        points = _canonical_points(feature)
        if _intersects(points, low, high):
            _draw_polyline(image, points, transform, (180, 110, 30), 1)
            counts["current_link"] += 1
    for feature in layers["nominal_boundaries"]:
        points = _canonical_points(feature)
        if _intersects(points, low, high):
            _draw_polyline(image, points, transform, (100, 100, 100), 1)
            counts["nominal_boundary"] += 1
    for feature in layers["adopted_legacy_boundaries"]:
        points = _canonical_points(feature)
        if _intersects(points, low, high):
            lane_color = str(feature.get("attributes", {}).get("lane_color", "unknown")).lower()
            color = (245, 245, 245) if lane_color == "white" else (20, 215, 245) if lane_color == "yellow" else (50, 220, 50)
            _draw_polyline(image, points, transform, color, 3)
            counts["adopted_legacy_boundary"] += 1
    for reference in layers["legacy_references"]:
        points = reference["coordinates"]
        if _intersects(points, low, high):
            color = (230, 60, 210) if reference["tier"] == "A" else (170, 70, 150)
            _draw_polyline(image, points, transform, color, 1, dashed=True)
            counts[f"legacy_tier_{reference['tier'].lower()}"] += 1
    observation_colors = {
        "automatic": (220, 150, 30),
        "review_required": (0, 150, 255),
        "rejected": (30, 30, 230),
        "unmatched": (150, 40, 230),
    }
    for segment in layers["observation_segments"]:
        points = segment["points"]
        if _intersects(points, low, high):
            status = segment["review_status"]
            _draw_polyline(image, points, transform, observation_colors[status], 1)
            counts[f"observation_{status}"] += 1
    for feature in layers["observed_boundaries"]:
        points = _canonical_points(feature)
        if _intersects(points, low, high):
            lane_color = str(feature.get("attributes", {}).get("lane_color", "unknown")).lower()
            color = (245, 245, 245) if lane_color == "white" else (20, 215, 245) if lane_color == "yellow" else (50, 220, 50)
            _draw_polyline(image, points, transform, color, 3)
            counts["observed_calibrated"] += 1
    for feature in layers["stoplines"]:
        points = _canonical_points(feature)
        if _intersects(points, low, high):
            _draw_polyline(image, points, transform, (20, 40, 255), 3)
            counts["stopline"] += 1
    if _intersects(layers["route"], low, high):
        _draw_polyline(image, layers["route"], transform, (60, 220, 60), 2)
    legend = [
        title,
        "cyan=current center  gray=nominal  white/yellow=reviewed hybrid candidate",
        "orange=observed validation only",
        "red=stopline  green=organizer route  magenta dashed=legacy A/B reference",
        "raw: blue=automatic  orange=review  red/magenta=reject/unmatched",
        "REVIEW ONLY: legacy and raw observations are not operational map geometry",
    ]
    for index, line in enumerate(legend):
        cv2.putText(image, line, (14, 22 + index * 19), cv2.FONT_HERSHEY_SIMPLEX, 0.43, (238, 238, 238), 1, cv2.LINE_AA)
    path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(path), image):
        raise IOError(f"failed to write overlay: {path}")
    return dict(sorted(counts.items()))


def build_candidate_review(config_path: Path, repo_root: Path, output_dir: Path) -> dict:
    layers = load_review_layers(config_path, repo_root)
    config = layers["config"]
    inventory = high_speed_inventory(layers)
    high_speed_ids = {row["current_link_id"] for row in inventory}
    reachable = {row["current_link_id"] for row in inventory if row["lane_change_reachable"]}
    missing_boundary = [row["current_link_id"] for row in inventory if row["nominal_boundary_count"] == 0]
    high_index = load_json(layers["source_paths"]["high_speed_legacy_index"])
    indexed_ids = {str(value) for value in high_index["current_link_ids"]}
    if high_speed_ids != indexed_ids:
        raise ValueError("stage-1 and legacy-audit high-speed link inventories differ")

    overlays: dict[str, dict] = {}
    for name, region in config["regions"].items():
        low, high = _region_bounds(layers, region)
        counts = render_review_overlay(
            output_dir / "overlays" / f"{name}.png", layers, low, high, name,
            {key: region[key] for key in ("width_px", "height_px") if key in region},
        )
        overlays[name] = {
            "bounds_xy": [low.tolist(), high.tolist()],
            "layer_counts": counts,
        }
    all_current = np.vstack([_canonical_points(feature) for feature in layers["current_links"]])
    overview_low = np.min(all_current[:, :2], axis=0) - 12.0
    overview_high = np.max(all_current[:, :2], axis=0) + 12.0
    overlays["competition_corridor"] = {
        "bounds_xy": [overview_low.tolist(), overview_high.tolist()],
        "layer_counts": render_review_overlay(
            output_dir / "overlays" / "competition_corridor.png",
            layers, overview_low, overview_high, "competition_corridor",
        ),
    }
    summary = {
        "review_id": config["review_id"],
        "review_config_sha256": sha256_file(config_path),
        "status": "review_only_non_operational",
        "operational": False,
        "source_hashes": layers["source_hashes"],
        "high_speed_inventory": {
            "parallel_current_link_count": len(high_speed_ids),
            "lane_change_reachable_link_count": len(reachable),
            "all_lane_change_reachable_links_present": (
                len(high_speed_ids) == int(config["expected"]["high_speed_parallel_links"])
                and len(reachable) == int(config["expected"]["lane_change_reachable_links"])
                and reachable.issubset(high_speed_ids)
            ),
            "same_direction_count": sum(row["relative_direction"] == "same_route_direction" for row in inventory),
            "opposite_direction_count": sum(row["relative_direction"] == "opposite_route_direction" for row in inventory),
            "centerline_only_boundary_review_link_ids": missing_boundary,
            "legacy_high_speed_candidate_count": len(high_index["features"]),
            "legacy_high_speed_tiers": dict(sorted(Counter(item["tier"] for item in high_index["features"]).items())),
            "lane_change_reachable_with_observed_calibration_count": sum(
                bool(row["observed_calibrated_interval_count"])
                for row in inventory if row["lane_change_reachable"]
            ),
            "lane_change_reachable_without_observed_calibration_link_ids": [
                row["current_link_id"] for row in inventory
                if row["lane_change_reachable"] and not row["observed_calibrated_interval_count"]
            ],
        },
        "observed_legacy_high_speed_agreement": observed_legacy_agreement(
            layers, high_speed_ids,
        ),
        "geometry_usage": {
            "current_link_node_topology_used": True,
            "observed_calibrated_geometry_used": False,
            "observed_calibrated_geometry_shown_as_validation_reference": True,
            "legacy_geometry_merged_into_candidate_map": True,
            "legacy_geometry_shown_as_review_reference": True,
            "reviewed_legacy_adoption_candidate_available": True,
            "current_reconstructed_geometry_used": True,
            "painted_stoplines_after_driving_review": len(layers["stoplines"]),
            "observed_geometry_demoted_to_validation_reference": True,
        },
        "transfer_recommendation": {
            "current_topology_remains_authoritative": True,
            "stable_high_speed_legacy_tier_a": "adopted_in_non_operational_hybrid_candidate",
            "legacy_tier_b": "manual_review_or_current_observation_support_required",
            "changed_or_branching_sections": "do_not_copy_without_current_geometry_validation",
        },
        "overlay_regions": overlays,
        "decision": (
            "all lane-change-reachable high-speed centerlines are retained; stable legacy "
            "paint geometry and current reconstructed 2025 gaps form a non-operational hybrid candidate"
        ),
    }
    _write_json(output_dir / "summary.json", summary)
    _write_csv(output_dir / "high_speed_inventory.csv", inventory, [
        "current_link_id", "road_id", "ego_lane", "relative_direction",
        "official_route", "lane_change_reachable", "geometry_scope",
        "exported_length_m", "nominal_boundary_count", "nominal_boundary_sides",
        "observed_calibrated_interval_count", "observed_calibrated_length_m",
        "observed_boundary_coverage_ratio", "legacy_tier_a_count",
        "legacy_tier_b_count", "inventory_status",
    ])
    return summary
