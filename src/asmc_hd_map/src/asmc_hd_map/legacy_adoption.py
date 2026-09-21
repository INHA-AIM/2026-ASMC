"""Adopt visually approved legacy lane geometry without replacing current topology."""

from __future__ import annotations

from collections import Counter
import copy
import csv
import math
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import yaml

from .current_builder import _write_csv, _write_json
from .frames import LocalMapFrame, origin_translation
from .geometry import cumulative_distance, points_xyz, quantile_or_none, resample_polyline
from .io_mgeo import load_json, resolve_source, sha256_file, source_record
from .schema import SCHEMA_VERSION, make_candidate_feature, validate_canonical_document
from .spatial import GridIndex


CONFIG_SCHEMA = "asmc.hd_map.legacy_adoption_config/v1"


def _read_csv(path: Path) -> list[dict]:
    with path.open("r", encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))


def _feature_points(feature: dict) -> np.ndarray:
    return points_xyz((feature.get("geometry") or {}).get("coordinates", ()))


def _lane_color(lane: dict) -> str:
    values = lane.get("lane_color") or []
    if isinstance(values, str):
        values = [values]
    lowered = {str(value).strip().lower() for value in values}
    for name in ("yellow", "white", "blue"):
        if name in lowered:
            return name
    return "unknown"


def _retag_parent_features(features: list[dict], map_id: str) -> list[dict]:
    result = copy.deepcopy(features)
    for feature in result:
        suffix = str(feature["id"]).split(":", 1)[-1]
        feature["id"] = f"{map_id}:{suffix}"
    return result


def _close_short_false_runs(mask: np.ndarray, maximum_gap_samples: int) -> np.ndarray:
    result = np.asarray(mask, dtype=bool).copy()
    if maximum_gap_samples <= 0 or len(result) < 3:
        return result
    index = 0
    while index < len(result):
        if result[index]:
            index += 1
            continue
        start = index
        while index < len(result) and not result[index]:
            index += 1
        if start > 0 and index < len(result) and index - start <= maximum_gap_samples:
            result[start:index] = True
    return result


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


def _station_is_approved(station_m: float, intervals: list[dict]) -> tuple[bool, str | None]:
    for interval in intervals:
        if float(interval["start_s_m"]) <= station_m <= float(interval["end_s_m"]):
            return True, str(interval["name"])
    return False, None


def _current_link_index(features: list[dict], step_m: float) -> tuple[GridIndex, np.ndarray]:
    points: list[np.ndarray] = []
    tangents: list[np.ndarray] = []
    owners: list[str] = []
    stations: list[float] = []
    for feature in features:
        sampled, sampled_tangents = resample_polyline(_feature_points(feature), step_m)
        if not len(sampled):
            continue
        link_id = str(feature["topology"]["current_link_ids"][0])
        attributes = feature.get("attributes") or {}
        start_s = attributes.get("route_s_min_m")
        end_s = attributes.get("route_s_max_m")
        if start_s is None or end_s is None:
            continue
        points.extend(sampled)
        tangents.extend(sampled_tangents)
        owners.extend([link_id] * len(sampled))
        stations.extend(np.linspace(float(start_s), float(end_s), len(sampled)).tolist())
    if not points:
        raise ValueError("current candidate contains no indexed link centerlines")
    return (
        GridIndex(np.asarray(points), np.asarray(tangents), owners, cell_size_m=3.0),
        np.asarray(stations, dtype=np.float64),
    )


def _make_legacy_segment(
    *, map_id: str, lane: dict, lane_sha: str, points: np.ndarray,
    owners: list[str], distances: np.ndarray, stations: np.ndarray,
    source_tier: str, segment_index: int, interval_names: list[str],
    geometry_status: str, topology_binding: str = "current_boundary_owner",
) -> dict:
    source_id = str(lane["idx"])
    feature = make_candidate_feature(
        feature_type="lane_boundary",
        canonical_id=(
            f"{map_id}:lane_boundary:legacy:{source_id}:segment:{segment_index:02d}"
        ),
        points=points,
        attributes={
            "geometry_status": geometry_status,
            "lane_color": _lane_color(lane),
            "mgeo": {
                key: lane.get(key) for key in (
                    "lane_type", "lane_sub_type", "lane_color", "lane_shape",
                    "lane_width", "dash_interval_L1", "dash_interval_L2",
                    "double_line_interval", "pass_restr",
                )
            },
        },
        source_id=source_id,
        source_sha256=lane_sha,
        target_link_ids=(sorted(set(owners)) if topology_binding == "current_boundary_owner" else []),
        validation={
            "legacy_audit_tier": source_tier,
            "driving_review_date": "2026-09-16",
            "driving_review_decision": (
                "approved_unchanged_region"
                if geometry_status.startswith("legacy_validated")
                else "changed_region_review_required"
            ),
            "approved_interval_names": sorted(set(interval_names)),
            "route_s_min_m": float(np.min(stations)),
            "route_s_max_m": float(np.max(stations)),
            "current_link_center_distance_p50_m": quantile_or_none(distances, 0.50),
            "current_link_center_distance_p95_m": quantile_or_none(distances, 0.95),
            "reviewed": geometry_status.startswith("legacy_validated"),
        },
    )
    feature["topology"]["binding_status"] = topology_binding
    if topology_binding == "route_context_only":
        feature["topology"]["route_context_link_ids"] = sorted(set(owners))
    feature["status"] = "reviewed" if geometry_status.startswith("legacy_validated") else "candidate"
    return feature


def _draw_polyline(image: np.ndarray, points: np.ndarray, transform, color, thickness: int, dashed: bool = False) -> None:
    if len(points) < 2:
        return
    pixels = transform(points)
    if dashed:
        for index in range(0, len(pixels) - 1, 2):
            cv2.line(image, tuple(pixels[index]), tuple(pixels[index + 1]), color, thickness, cv2.LINE_AA)
    else:
        cv2.polylines(image, [pixels], False, color, thickness, cv2.LINE_AA)


def _render_overlay(path: Path, title: str, bounds: tuple[np.ndarray, np.ndarray], layers: dict, visual: dict) -> dict:
    low, high = bounds
    width, height = int(visual["width_px"]), int(visual["height_px"])
    padding = int(visual["padding_px"])
    span = np.maximum(high - low, 1.0)
    scale = min((width - 2 * padding) / span[0], (height - 2 * padding) / span[1])
    center = 0.5 * (low + high)
    draw_span = np.asarray([(width - 2 * padding) / scale, (height - 2 * padding) / scale])
    draw_low, draw_high = center - draw_span / 2.0, center + draw_span / 2.0
    image = np.full((height, width, 3), 20, dtype=np.uint8)

    def transform(points: np.ndarray) -> np.ndarray:
        result = np.empty((len(points), 2), dtype=np.int32)
        result[:, 0] = np.round(padding + (points[:, 0] - draw_low[0]) * scale).astype(np.int32)
        result[:, 1] = np.round(padding + (draw_high[1] - points[:, 1]) * scale).astype(np.int32)
        return result

    def intersects(points: np.ndarray) -> bool:
        return bool(len(points) and np.all(np.max(points[:, :2], axis=0) >= low) and np.all(np.min(points[:, :2], axis=0) <= high))

    counts = Counter()
    for feature in layers["current_links"]:
        points = _feature_points(feature)
        if intersects(points):
            _draw_polyline(image, points, transform, (190, 115, 30), 1)
            counts["current_link"] += 1
    for feature in layers["nominal"]:
        points = _feature_points(feature)
        if intersects(points):
            _draw_polyline(image, points, transform, (90, 90, 90), 1)
            counts["nominal_reference"] += 1
    for feature in layers["observed"]:
        points = _feature_points(feature)
        if intersects(points):
            _draw_polyline(image, points, transform, (0, 135, 255), 1)
            counts["observed_validation_reference"] += 1
    for feature in layers["review_required"]:
        points = _feature_points(feature)
        if intersects(points):
            _draw_polyline(image, points, transform, (220, 50, 220), 1, dashed=True)
            counts["changed_region_review_required"] += 1
    for feature in layers["adopted"]:
        points = _feature_points(feature)
        if intersects(points):
            color = (245, 245, 245) if feature["attributes"]["lane_color"] == "white" else (20, 220, 245)
            _draw_polyline(image, points, transform, color, 3)
            counts["legacy_validated"] += 1
    for feature in layers["stoplines"]:
        points = _feature_points(feature)
        if intersects(points):
            _draw_polyline(image, points, transform, (25, 40, 255), 3)
            counts["current_stopline"] += 1
    route = layers["route"]
    if intersects(route):
        _draw_polyline(image, route, transform, (50, 210, 50), 2)

    legend = [
        title,
        "cyan=current center  gray=nominal reference  orange=observed validation only",
        "white/yellow=adopted legacy  magenta dashed=changed-region review only",
        "red=current stopline  green=organizer route",
        "CANDIDATE: current topology retained; adopted geometry is not operational yet",
    ]
    for index, line in enumerate(legend):
        cv2.putText(image, line, (14, 22 + index * 19), cv2.FONT_HERSHEY_SIMPLEX, 0.43, (238, 238, 238), 1, cv2.LINE_AA)
    path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(path), image):
        raise IOError(f"failed to write overlay: {path}")
    return dict(sorted(counts.items()))


def _route_points(path: Path) -> np.ndarray:
    values = []
    with path.open("r", encoding="utf-8") as stream:
        for line in stream:
            row = line.strip().replace(",", " ").split()
            if len(row) >= 2:
                values.append([float(row[0]), float(row[1]), float(row[2]) if len(row) > 2 else 0.0])
    return points_xyz(values)


def build_legacy_adoption(config_path: Path, repo_root: Path, output_dir: Path) -> dict:
    with config_path.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream) or {}
    if config.get("schema_version") != CONFIG_SCHEMA:
        raise ValueError("unsupported legacy adoption config schema")
    if config["promotion_policy"].get("operational") is not False:
        raise ValueError("legacy adoption must remain non-operational until final driving review")

    observed_path = resolve_source(repo_root, config["sources"]["observed_candidate_map"])
    legacy_config_path = resolve_source(repo_root, config["sources"]["legacy_transfer_config"])
    audit_path = resolve_source(repo_root, config["sources"]["legacy_lane_audit"])
    observed_document = load_json(observed_path)
    with legacy_config_path.open("r", encoding="utf-8") as stream:
        legacy_config = yaml.safe_load(stream) or {}
    legacy_lane_path = resolve_source(repo_root, legacy_config["sources"]["legacy"]["lane_boundary_set"])
    legacy_global_path = resolve_source(repo_root, legacy_config["sources"]["legacy"]["global_info"])
    current_global_path = resolve_source(repo_root, legacy_config["sources"]["current"]["global_info"])
    route_path = resolve_source(repo_root, legacy_config["sources"]["current"]["route"])
    translation = origin_translation(
        LocalMapFrame.from_global_info(load_json(legacy_global_path)),
        LocalMapFrame.from_global_info(load_json(current_global_path)),
    )

    current_links = [feature for feature in observed_document["features"] if feature["feature_type"] == "link_centerline"]
    nominal = [feature for feature in observed_document["features"] if feature["feature_type"] == "lane_boundary" and feature.get("attributes", {}).get("geometry_status") == "link_nominal"]
    observed = [feature for feature in observed_document["features"] if feature["feature_type"] == "lane_boundary" and feature.get("attributes", {}).get("geometry_status") == "observed_calibrated"]
    stoplines = [feature for feature in observed_document["features"] if feature["feature_type"] == "stopline"]
    selection = config["selection"]
    link_index, link_stations = _current_link_index(current_links, float(selection["current_link_sample_step_m"]))
    route = _route_points(route_path)
    route_samples, route_tangents = resample_polyline(route, 0.5)
    route_station = cumulative_distance(route_samples)
    official_ranges = [
        (
            float(feature["attributes"]["route_s_min_m"]),
            float(feature["attributes"]["route_s_max_m"]),
            str(feature["topology"]["current_link_ids"][0]),
        )
        for feature in current_links
        if "official_route" in feature.get("attributes", {}).get("corridor_roles", [])
    ]
    route_owners = []
    for station in route_station:
        matching = [link_id for start, end, link_id in official_ranges if start - 0.6 <= station <= end + 0.6]
        route_owners.append(matching[0] if matching else "")
    route_index = GridIndex(
        route_samples, route_tangents, route_owners, cell_size_m=3.0,
    )
    tiers = {row["source_id"]: row["tier"] for row in _read_csv(audit_path)}
    excluded_colors = {str(value).lower() for value in selection["excluded_colors"]}
    excluded_types = {int(value) for value in selection["excluded_lane_types"]}
    lane_sha = sha256_file(legacy_lane_path)
    adopted: list[dict] = []
    adopted_context: list[dict] = []
    review_required: list[dict] = []
    inventory: list[dict] = []
    excluded = Counter()
    gap_samples = int(math.floor(float(selection["close_internal_gap_m"]) / float(selection["sample_step_m"])))
    output_stride = max(
        1,
        int(round(float(selection["output_step_m"]) / float(selection["sample_step_m"]))),
    )

    for lane in load_json(legacy_lane_path):
        source_id = str(lane.get("idx", ""))
        color = _lane_color(lane)
        lane_types = {int(value) for value in lane.get("lane_type") or []}
        if color in excluded_colors:
            excluded["excluded_color"] += 1
            continue
        if lane_types.intersection(excluded_types):
            excluded["current_stopline_authority"] += 1
            continue
        sampled, tangents = resample_polyline(points_xyz(lane.get("points", ())) + translation, float(selection["sample_step_m"]))
        if len(sampled) < 2:
            excluded["invalid_geometry"] += 1
            continue
        valid = np.zeros(len(sampled), dtype=bool)
        approved = np.zeros(len(sampled), dtype=bool)
        owner_values: list[str | None] = [None] * len(sampled)
        distance_values = np.full(len(sampled), np.nan)
        station_values = np.full(len(sampled), np.nan)
        interval_values: list[str | None] = [None] * len(sampled)
        route_valid = np.zeros(len(sampled), dtype=bool)
        route_approved = np.zeros(len(sampled), dtype=bool)
        route_station_values = np.full(len(sampled), np.nan)
        route_interval_values: list[str | None] = [None] * len(sampled)
        route_owner_values: list[str | None] = [None] * len(sampled)
        for index, (point, tangent) in enumerate(zip(sampled, tangents)):
            _, _, route_match = route_index.query(
                point, tangent,
                max_distance_m=float(selection["maximum_route_context_distance_m"]),
                max_heading_deg=float(selection["maximum_heading_difference_deg"]),
            )
            if route_match is not None:
                route_valid[index] = True
                route_station_values[index] = route_station[route_match]
                route_owner_values[index] = str(route_index.owners[route_match]) or None
                route_approved[index], route_interval_values[index] = _station_is_approved(
                    route_station[route_match], config["approved_route_intervals"],
                )
            distance, _, match = link_index.query(
                point, tangent,
                max_distance_m=float(selection["maximum_link_center_distance_m"]),
                max_heading_deg=float(selection["maximum_heading_difference_deg"]),
            )
            if match is None:
                continue
            valid[index] = True
            owner_values[index] = str(link_index.owners[match])
            distance_values[index] = distance
            station_values[index] = link_stations[match]
            approved[index], interval_values[index] = _station_is_approved(
                link_stations[match], config["approved_route_intervals"],
            )
        adopted_mask = _close_short_false_runs(valid & approved, gap_samples)
        context_mask = _close_short_false_runs(
            route_valid & route_approved & ~valid, gap_samples,
        )
        context_mask &= ~adopted_mask
        review_mask = _close_short_false_runs(valid & ~approved, gap_samples)
        source_tier = tiers.get(source_id, "D")
        lane_rows = []
        next_segment = 0
        for mask, status, output, station_source, interval_source, owner_source, binding in (
            (
                adopted_mask, "legacy_validated", adopted,
                station_values, interval_values, owner_values, "current_boundary_owner",
            ),
            (
                context_mask, "legacy_validated_route_context", adopted_context,
                route_station_values, route_interval_values, route_owner_values, "route_context_only",
            ),
            (
                review_mask, "changed_region_review_required", review_required,
                station_values, interval_values, owner_values, "current_boundary_owner",
            ),
        ):
            for start, end in _true_runs(mask):
                run_points = sampled[start:end]
                if len(run_points) < 2 or cumulative_distance(run_points)[-1] < float(selection["minimum_segment_length_m"]):
                    continue
                output_points = run_points[::output_stride]
                if not np.array_equal(output_points[-1], run_points[-1]):
                    output_points = np.vstack((output_points, run_points[-1]))
                indices = np.arange(start, end)
                finite = np.isfinite(station_source[indices])
                indices = indices[finite]
                if not len(indices):
                    continue
                feature = _make_legacy_segment(
                    map_id=config["map_id"], lane=lane, lane_sha=lane_sha,
                    points=output_points,
                    owners=[owner_source[index] for index in indices if owner_source[index] is not None],
                    distances=(
                        distance_values[indices]
                        if binding == "current_boundary_owner"
                        else np.asarray([
                            route_index.query(
                                sampled[index], tangents[index],
                                max_distance_m=float(selection["maximum_route_context_distance_m"]),
                                max_heading_deg=float(selection["maximum_heading_difference_deg"]),
                            )[0]
                            for index in indices
                        ])
                    ),
                    stations=station_source[indices],
                    source_tier=source_tier, segment_index=next_segment,
                    interval_names=[interval_source[index] for index in indices if interval_source[index]],
                    geometry_status=status, topology_binding=binding,
                )
                output.append(feature)
                lane_rows.append(feature)
                next_segment += 1
        if lane_rows:
            inventory.append({
                "legacy_id": source_id,
                "legacy_audit_tier": source_tier,
                "lane_color": color,
                "lane_type": ";".join(str(value) for value in sorted(lane_types)),
                "adopted_segment_count": sum(f["attributes"]["geometry_status"].startswith("legacy_validated") for f in lane_rows),
                "route_context_segment_count": sum(f["attributes"]["geometry_status"] == "legacy_validated_route_context" for f in lane_rows),
                "review_required_segment_count": sum(f["attributes"]["geometry_status"] != "legacy_validated" for f in lane_rows),
                "target_current_link_ids": ";".join(sorted({link for f in lane_rows for link in f["topology"]["current_link_ids"]})),
                "route_context_link_ids": ";".join(sorted({
                    link for f in lane_rows
                    for link in f["topology"].get("route_context_link_ids", [])
                })),
            })

    all_adopted = adopted + adopted_context
    core_features = _retag_parent_features(current_links + stoplines, config["map_id"]) + all_adopted
    document = {
        "schema_version": SCHEMA_VERSION,
        "map_id": config["map_id"],
        "frame": observed_document["frame"],
        "sources": {
            "parent_observed_candidate_map": source_record(repo_root, config["sources"]["observed_candidate_map"], observed_path),
            "legacy_lane_boundary_set": source_record(repo_root, legacy_config["sources"]["legacy"]["lane_boundary_set"], legacy_lane_path),
            "legacy_lane_audit": source_record(repo_root, config["sources"]["legacy_lane_audit"], audit_path),
        },
        "build": {
            "build_id": config["build_id"],
            "operational": False,
            "current_link_node_topology_authority": True,
            "current_stoplines_authority": True,
            "legacy_geometry_preserved_without_snapping": True,
            "observed_geometry_is_validation_reference_only": True,
            "lane_geometry_completeness": "candidate_requires_final_gap_and_transition_review",
            "approved_route_intervals": config["approved_route_intervals"],
        },
        "features": core_features,
        "review_layers": {
            "nominal_boundary_references": nominal,
            "observed_validation_references": observed,
            "changed_region_legacy_candidates": review_required,
        },
    }
    validate_canonical_document(document)
    _write_json(output_dir / "legacy_adopted_candidate_map.json", document)
    _write_csv(output_dir / "adoption_inventory.csv", sorted(inventory, key=lambda row: row["legacy_id"]), [
        "legacy_id", "legacy_audit_tier", "lane_color", "lane_type",
        "adopted_segment_count", "route_context_segment_count",
        "review_required_segment_count", "target_current_link_ids", "route_context_link_ids",
    ])

    layers = {
        "current_links": current_links, "nominal": nominal, "observed": observed,
        "stoplines": stoplines, "adopted": all_adopted,
        "review_required": review_required, "route": route,
    }
    margin = float(config["visualization"]["corridor_margin_m"])
    overlay_ranges = {
        "full_corridor": (0.0, float(route_station[-1])),
        "early_intersections": (0.0, 630.3767103356735),
        "roundabout_to_high_speed": (576.61869488622, 1118.741751169101),
        "high_speed_to_tollgate": (1118.741751169101, 1783.0572973249689),
        "tollgate_to_shadow_excluded": (1783.0572973249689, 1851.0567272113842),
        "shadow_and_tail": (1851.0567272113842, float(route_station[-1])),
    }
    overlays = {}
    for name, (start_s, end_s) in overlay_ranges.items():
        mask = (route_station >= start_s) & (route_station <= end_s)
        selected = route_samples[mask]
        if not len(selected):
            continue
        low = np.min(selected[:, :2], axis=0) - margin
        high = np.max(selected[:, :2], axis=0) + margin
        overlays[name] = {
            "route_s_m": [start_s, end_s],
            "bounds_xy": [low.tolist(), high.tolist()],
            "layer_counts": _render_overlay(
                output_dir / "overlays" / f"{name}.png", name, (low, high), layers,
                config["visualization"],
            ),
        }

    summary = {
        "build_id": config["build_id"],
        "map_id": config["map_id"],
        "status": "candidate_legacy_adopted_non_operational",
        "operational": False,
        "coordinate_transform": {
            "translation_legacy_to_current_xyz_m": translation.tolist(),
            "rotation_deg": 0.0,
            "scale": 1.0,
        },
        "policy": config["promotion_policy"],
        "approved_route_intervals": config["approved_route_intervals"],
        "counts": {
            "legacy_source_lanes": len(load_json(legacy_lane_path)),
            "legacy_source_lanes_with_any_output": len(inventory),
            "adopted_legacy_source_lanes": len({f["provenance"]["source_id"] for f in all_adopted}),
            "adopted_segment_count": len(all_adopted),
            "current_link_bound_segment_count": len(adopted),
            "route_context_segment_count": len(adopted_context),
            "route_context_source_lanes": len({f["provenance"]["source_id"] for f in adopted_context}),
            "changed_region_review_segment_count": len(review_required),
            "rescued_tier_d_source_lanes": len({f["provenance"]["source_id"] for f in all_adopted if f["validation"]["legacy_audit_tier"] == "D"}),
            "current_link_center_count": len(current_links),
            "current_stopline_count": len(stoplines),
            "nominal_reference_count": len(nominal),
            "observed_validation_reference_count": len(observed),
        },
        "adopted_by_audit_tier": dict(sorted(Counter(f["validation"]["legacy_audit_tier"] for f in all_adopted).items())),
        "adopted_by_color": dict(sorted(Counter(f["attributes"]["lane_color"] for f in all_adopted).items())),
        "excluded_source_lanes": dict(sorted(excluded.items())),
        "overlays": overlays,
        "decision": (
            "legacy geometry is adopted only where it remains locally compatible with current "
            "corridor links inside driving-reviewed route intervals; current topology and stoplines remain authoritative"
        ),
    }
    _write_json(output_dir / "summary.json", summary)
    return summary
