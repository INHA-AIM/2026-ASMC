"""Reproducible legacy-KATRI to K-City 2025 transfer audit."""

from __future__ import annotations

from collections import Counter, defaultdict
import csv
import json
import math
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from .correspondence import (
    MatchPolicy,
    boolean_intervals,
    coalesce_boolean_runs,
    match_feature,
    match_samples,
)
from .frames import LocalMapFrame, origin_translation
from .geometry import cumulative_distance, offset_polyline, quantile_or_none, resample_polyline
from .io_mgeo import (
    feature_cloud,
    first_existing_source,
    load_json,
    resolve_source,
    source_record,
)
from .schema import SCHEMA_VERSION, make_candidate_feature, validate_canonical_document
from .spatial import GridIndex


def _plain(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(key): _plain(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_plain(item) for item in value]
    if isinstance(value, np.ndarray):
        return _plain(value.tolist())
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def _write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as stream:
        json.dump(_plain(value), stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.write("\n")


def _write_csv(path: Path, rows: list[dict], fieldnames: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames, lineterminator="\n")
        writer.writeheader()
        for row in rows:
            writer.writerow({key: _plain(row.get(key)) for key in fieldnames})


def _load_route(path: Path) -> list[list[float]]:
    points: list[list[float]] = []
    with path.open("r", encoding="utf-8") as stream:
        for line in stream:
            values = line.strip().replace(",", " ").split()
            if len(values) >= 2:
                points.append([float(values[0]), float(values[1]), float(values[2]) if len(values) > 2 else 0.0])
    if len(points) < 2:
        raise ValueError(f"route has fewer than two points: {path}")
    return points


def _policy(config: dict) -> MatchPolicy:
    matching = config["matching"]
    return MatchPolicy(**{
        key: float(matching[key])
        for key in MatchPolicy.__dataclass_fields__
        if key in matching
    })


def _target_ids(match: dict) -> list[str]:
    return [str(item["id"]) for item in match.get("targets", [])]


def _feature_rows(matches: list[dict]) -> list[dict]:
    rows: list[dict] = []
    for match in matches:
        rows.append({
            "source_id": match["source_id"],
            "tier": match["tier"],
            "sample_count": match["sample_count"],
            "strict_coverage": round(match["strict_coverage"], 6),
            "relaxed_coverage": round(match["relaxed_coverage"], 6),
            "strict_distance_p50_m": match["strict_distance_p50_m"],
            "strict_distance_p95_m": match["strict_distance_p95_m"],
            "relaxed_distance_p95_m": match["relaxed_distance_p95_m"],
            "target_current_ids": ";".join(_target_ids(match)),
        })
    return rows


def _route_audit(
    route_raw: list[list[float]],
    legacy_link_index: GridIndex,
    current_link_index: GridIndex,
    policy: MatchPolicy,
    sample_step_m: float,
    close_gap_m: float,
    remove_island_m: float,
) -> dict:
    route, tangents = resample_polyline(route_raw, sample_step_m)
    matched = match_samples(route, tangents, legacy_link_index, policy)
    station = cumulative_distance(route)
    reuse = coalesce_boolean_runs(
        matched["strict_mask"], station, close_gap_m, remove_island_m,
    )
    intervals = boolean_intervals(reuse, route)

    legacy_owner: list[str | None] = []
    current_owner: list[str | None] = []
    z_residual: list[float] = []
    xy_residual: list[list[float]] = []
    for point, tangent, keep in zip(route, tangents, matched["strict_mask"]):
        distance, heading, old_index = legacy_link_index.query(
            point, tangent, policy.search_distance_m, policy.search_heading_deg,
        )
        _, _, current_index = current_link_index.query(point, tangent, 5.0, 45.0)
        legacy_owner.append(
            None if old_index is None else str(legacy_link_index.owners[old_index])
        )
        current_owner.append(
            None if current_index is None else str(current_link_index.owners[current_index])
        )
        if keep and old_index is not None:
            residual = legacy_link_index.points[old_index] - point
            xy_residual.append([float(residual[0]), float(residual[1])])
            z_residual.append(float(residual[2]))

    for interval in intervals:
        start, end = interval["start_index"], interval["end_index"] + 1
        interval["current_link_ids"] = [
            key for key, _ in Counter(x for x in current_owner[start:end] if x).most_common(8)
        ]
        interval["legacy_link_ids"] = [
            key for key, _ in Counter(x for x in legacy_owner[start:end] if x).most_common(8)
        ]
        finite = matched["distances"][start:end]
        interval["distance_p50_m"] = quantile_or_none(finite, 0.50)
        interval["distance_p95_m"] = quantile_or_none(finite, 0.95)

    weights = np.diff(station, prepend=station[0])
    reuse_length = float(np.sum(weights[reuse]))
    residual_xy = np.asarray(xy_residual, dtype=np.float64)
    return {
        "points": route,
        "tangents": tangents,
        "station_m": station,
        "reuse_mask": reuse,
        "current_owner": current_owner,
        "legacy_owner": legacy_owner,
        "intervals": intervals,
        "total_length_m": float(station[-1]),
        "reuse_length_m": reuse_length,
        "reuse_length_ratio": reuse_length / float(station[-1]),
        "origin_only_xy_residual_median": (
            None if len(residual_xy) == 0 else np.median(residual_xy, axis=0).tolist()
        ),
        "origin_only_xy_residual_norm_p95_m": (
            None if len(residual_xy) == 0 else float(np.quantile(np.linalg.norm(residual_xy, axis=1), 0.95))
        ),
        "origin_only_z_residual_median_m": quantile_or_none(np.asarray(z_residual), 0.50),
        "origin_only_z_residual_p05_m": quantile_or_none(np.asarray(z_residual), 0.05),
        "origin_only_z_residual_p95_m": quantile_or_none(np.asarray(z_residual), 0.95),
    }


def _region_station_bounds(route: dict, region: dict) -> tuple[float, float]:
    if "start_s_m" in region:
        return float(region["start_s_m"]), float(region["end_s_m"])
    owners = np.asarray(route["current_owner"], dtype=object)
    station = route["station_m"]
    start = np.where(owners == str(region["start_link"]))[0]
    end = np.where(owners == str(region["end_link"]))[0]
    if len(start) == 0 or len(end) == 0:
        raise ValueError(f"route region links not found: {region}")
    return float(station[start[0]]), float(station[end[-1]])


def _current_links_in_route_corridor(
    current_links: list[dict],
    route_points: np.ndarray,
    route_tangents: np.ndarray,
    half_width_m: float,
) -> set[str]:
    """Collect parallel current links inside a route corridor.

    The route itself names only the driven lane.  HD-map transfer must also
    retain adjacent lanes, while rejecting perpendicular roads that merely
    cross the corridor.
    """

    route_index = GridIndex(route_points, route_tangents, cell_size_m=4.0)
    selected: set[str] = set()
    for link in current_links:
        points, tangents = resample_polyline(link.get("points", ()), 1.0)
        support = 0
        for point, tangent in zip(points, tangents):
            distance, heading, _ = route_index.query(
                point, tangent, max_distance_m=half_width_m, max_heading_deg=30.0,
            )
            if distance <= half_width_m and heading <= 30.0:
                support += 1
        if support >= 3:
            selected.add(str(link.get("idx", "")))
    return selected


def _current_boundary_proxy(
    links: list[dict], step_m: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    points: list[np.ndarray] = []
    tangents: list[np.ndarray] = []
    owners: list[str] = []
    for link in links:
        sampled, heading = resample_polyline(link.get("points", ()), step_m)
        if len(sampled) == 0:
            continue
        width = float(link.get("width_start") or link.get("width_end") or 3.5)
        for sign in (-1.0, 1.0):
            boundary = offset_polyline(sampled, heading, sign * width / 2.0)
            points.extend(boundary)
            tangents.extend(heading)
            owners.extend([str(link.get("idx", ""))] * len(boundary))
    return np.asarray(points), np.asarray(tangents), np.asarray(owners, dtype=object)


def _lane_match(
    lane: dict,
    translation: np.ndarray,
    step_m: float,
    boundary_index: GridIndex,
    matching: dict,
) -> dict:
    policy = MatchPolicy(
        search_distance_m=max(5.0, float(matching["lane_tier_b_distance_m"])),
        search_heading_deg=30.0,
        strict_distance_m=float(matching["lane_tier_a_distance_m"]),
        strict_heading_deg=float(matching["lane_tier_a_heading_deg"]),
        tier_a_coverage=float(matching["tier_a_coverage"]),
        tier_a_p95_m=float(matching["lane_tier_a_distance_m"]),
        relaxed_distance_m=float(matching["lane_tier_b_distance_m"]),
        relaxed_heading_deg=float(matching["lane_tier_b_heading_deg"]),
        tier_b_coverage=float(matching["tier_b_coverage"]),
    )
    result = match_feature(lane, translation, step_m, boundary_index, policy)
    result["lane_color"] = ",".join(str(v).lower() for v in lane.get("lane_color", []))
    result["lane_shape"] = ",".join(str(v) for v in lane.get("lane_shape", []))
    result["lane_type"] = ",".join(str(v) for v in lane.get("lane_type", []))
    result["lane_width_m"] = float(lane.get("lane_width") or 0.0)
    return result


def _observed_lane_audit(
    repo_root: Path,
    configured_glob: str,
    lanes: list[dict],
    translation: np.ndarray,
    lane_step_m: float,
) -> list[dict]:
    import glob

    grids: dict[str, tuple[GridIndex, np.ndarray]] = {}
    for color in ("white", "yellow"):
        selected = [
            lane for lane in lanes
            if color in {str(v).lower() for v in lane.get("lane_color", [])}
        ]
        points, tangents, owners = feature_cloud(selected, lane_step_m, translation)
        grids[color] = (GridIndex(points, tangents, owners, 3.0), owners)

    rows: list[dict] = []
    for raw_path in sorted(glob.glob(str(resolve_source(repo_root, configured_glob)))):
        path = Path(raw_path)
        values: list[float] = []
        sources: Counter[str] = Counter()
        seen: set[tuple] = set()
        with path.open("r", encoding="utf-8", newline="") as stream:
            for row in csv.DictReader(stream):
                color = str(row.get("lane_color", "")).lower()
                if color not in grids:
                    continue
                try:
                    point = np.asarray(
                        [float(row["lane_map_x"]), float(row["lane_map_y"]), 0.0],
                        dtype=np.float64,
                    )
                except (KeyError, TypeError, ValueError):
                    continue
                key = (color, round(float(point[0]), 2), round(float(point[1]), 2))
                if key in seen:
                    continue
                seen.add(key)
                index, _ = grids[color]
                distance, _, _ = index.query(point, max_distance_m=12.0, max_heading_deg=180.0)
                values.append(distance)
                sources[str(row.get("collection_source") or "LIDAR_SEMANTIC").strip()] += 1
        if not values:
            continue
        distances = np.asarray(values, dtype=np.float64)
        name = path.name
        category = (
            "high_speed" if name.startswith("hi_")
            else "shadow_end" if name.startswith("turnel") or name.startswith("end_")
            else "other"
        )
        rows.append({
            "source_file": name,
            "category": category,
            "sample_count": len(distances),
            "collection_sources": ";".join(f"{key}:{value}" for key, value in sorted(sources.items())),
            "distance_p50_m": quantile_or_none(distances, 0.50),
            "distance_p95_m": quantile_or_none(distances, 0.95),
            "within_0_50_ratio": float(np.mean(distances <= 0.50)),
            "within_0_75_ratio": float(np.mean(distances <= 0.75)),
            "within_1_00_ratio": float(np.mean(distances <= 1.00)),
        })
    return rows


def _raster_metrics(old: np.ndarray, current: np.ndarray, mask: np.ndarray) -> dict:
    old_mask = (old > 0) & (mask > 0)
    current_mask = (current > 0) & (mask > 0)
    intersection = old_mask & current_mask
    union = old_mask | current_mask
    return {
        "iou": float(np.sum(intersection) / max(1, np.sum(union))),
        "legacy_covered_by_current": float(np.sum(intersection) / max(1, np.sum(old_mask))),
        "current_proxy_covered_by_legacy": float(np.sum(intersection) / max(1, np.sum(current_mask))),
    }


def _road_mesh_audit(
    mesh_path: Path | None,
    current_links: list[dict],
    translation: np.ndarray,
    route: dict,
    route_regions: dict,
    pixels_per_meter: float,
) -> dict:
    if mesh_path is None:
        return {"available": False}
    try:
        import cv2
    except ImportError:
        return {"available": True, "evaluated": False, "reason": "python3-opencv unavailable"}

    mesh = load_json(mesh_path)
    current_points, _, _ = feature_cloud(current_links, 2.0)
    legacy_points = []
    for feature in mesh:
        legacy_points.extend(np.asarray(feature.get("points", ()), dtype=np.float64) + translation)
        for interior in feature.get("interiors", []):
            legacy_points.extend(np.asarray(interior.get("points", ()), dtype=np.float64) + translation)
    legacy_points_array = np.asarray(legacy_points, dtype=np.float64)
    all_points = np.vstack((current_points, legacy_points_array, route["points"]))
    minimum = np.min(all_points[:, :2], axis=0) - 25.0
    maximum = np.max(all_points[:, :2], axis=0) + 25.0
    width = int(math.ceil((maximum[0] - minimum[0]) * pixels_per_meter))
    height = int(math.ceil((maximum[1] - minimum[1]) * pixels_per_meter))
    if width * height > 80_000_000:
        raise ValueError("road mesh audit raster exceeds safety limit")

    def pixels(points) -> np.ndarray:
        points = np.asarray(points, dtype=np.float64)
        xy = np.column_stack((
            (points[:, 0] - minimum[0]) * pixels_per_meter,
            (maximum[1] - points[:, 1]) * pixels_per_meter,
        ))
        return np.round(xy).astype(np.int32).reshape((-1, 1, 2))

    legacy = np.zeros((height, width), dtype=np.uint8)
    for feature in mesh:
        exterior = np.asarray(feature.get("points", ()), dtype=np.float64) + translation
        cv2.fillPoly(legacy, [pixels(exterior)], 255)
        for interior in feature.get("interiors", []):
            hole = np.asarray(interior.get("points", ()), dtype=np.float64) + translation
            cv2.fillPoly(legacy, [pixels(hole)], 0)

    current = np.zeros_like(legacy)
    for link in current_links:
        points = np.asarray(link.get("points", ()), dtype=np.float64)
        width_m = float(link.get("width_start") or link.get("width_end") or 3.5)
        cv2.polylines(
            current, [pixels(points)], False, 255,
            max(1, int(round(width_m * pixels_per_meter))), cv2.LINE_8,
        )
    current = cv2.morphologyEx(current, cv2.MORPH_CLOSE, np.ones((5, 5), dtype=np.uint8))

    def corridor(start_s: float, end_s: float, width_m: float = 24.0) -> np.ndarray:
        selected = route["points"][(route["station_m"] >= start_s) & (route["station_m"] <= end_s)]
        result = np.zeros_like(legacy)
        if len(selected) >= 2:
            cv2.polylines(
                result, [pixels(selected)], False, 255,
                max(1, int(round(width_m * pixels_per_meter))), cv2.LINE_8,
            )
        return result

    entire_route = np.zeros_like(legacy)
    cv2.polylines(
        entire_route, [pixels(route["points"])], False, 255,
        max(1, int(round(30.0 * pixels_per_meter))), cv2.LINE_8,
    )
    metrics = {"route_30m_corridor": _raster_metrics(legacy, current, entire_route)}
    for name, region in route_regions.items():
        start_s, end_s = _region_station_bounds(route, region)
        metrics[name] = {
            "start_s_m": start_s,
            "end_s_m": end_s,
            **_raster_metrics(legacy, current, corridor(start_s, end_s)),
        }
    rebuild = [item for item in route["intervals"] if item["status"] == "rebuild"]
    if rebuild:
        largest = max(rebuild, key=lambda item: item["length_m"])
        metrics["largest_rebuild_interval"] = {
            "start_s_m": largest["start_s_m"],
            "end_s_m": largest["end_s_m"],
            **_raster_metrics(
                legacy, current,
                corridor(largest["start_s_m"], largest["end_s_m"]),
            ),
        }
    return {
        "available": True,
        "evaluated": True,
        "feature_count": len(mesh),
        "pixels_per_meter": pixels_per_meter,
        "comparison_target": "current_link_width_proxy_not_ground_truth",
        "metrics": metrics,
    }


def _auxiliary_audit(
    legacy_sources: dict,
    repo_root: Path,
    link_matches: dict[str, dict],
    route_points: np.ndarray,
    translation: np.ndarray,
) -> tuple[dict, list[dict]]:
    route_index = GridIndex(route_points, cell_size_m=5.0)
    summary: dict[str, dict] = {}
    rows: list[dict] = []
    for key in (
        "surface_marking_set", "traffic_light_set", "traffic_sign_set", "singlecrosswalk_set",
    ):
        path = resolve_source(repo_root, legacy_sources[key])
        features = load_json(path)
        tier_counter: Counter[str] = Counter()
        near_counter: Counter[str] = Counter()
        for feature in features:
            references = feature.get("link_id_list") or []
            tiers = [link_matches[str(ref)]["tier"] for ref in references if str(ref) in link_matches]
            tier = (
                "A" if tiers and all(item == "A" for item in tiers)
                else "B" if tiers and all(item in {"A", "B"} for item in tiers)
                else "D"
            )
            point = feature.get("point")
            if point is None and feature.get("points"):
                point = np.mean(np.asarray(feature["points"], dtype=np.float64), axis=0)
            distance = math.inf
            if point is not None:
                current_point = np.asarray(point, dtype=np.float64) + translation
                distance, _, _ = route_index.query(current_point, max_distance_m=40.0)
            tier_counter[tier] += 1
            for threshold in (5, 10, 15, 25):
                if distance <= threshold:
                    near_counter[str(threshold)] += 1
            rows.append({
                "layer": key,
                "source_id": str(feature.get("idx", "")),
                "reference_tier": tier,
                "route_distance_m": None if not math.isfinite(distance) else distance,
                "legacy_link_ids": ";".join(str(v) for v in references),
                "sub_type": str(feature.get("sub_type", feature.get("sign_type", ""))),
            })
        summary[key] = {
            "total": len(features),
            "reference_tiers": dict(tier_counter),
            "within_route_m": dict(near_counter),
            "note": "candidate count only; physical asset and current ID still require review",
        }
    return summary, rows


def run_audit(config_path: Path, repo_root: Path, output_dir: Path) -> dict:
    with config_path.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if config.get("schema_version") != "asmc.hd_map.transfer_config/v1":
        raise ValueError("unsupported transfer config schema")

    sources = config["sources"]
    current_paths = {
        key: resolve_source(repo_root, value) for key, value in sources["current"].items()
    }
    legacy_paths = {
        key: resolve_source(repo_root, value)
        for key, value in sources["legacy"].items()
        if key != "road_mesh_candidates"
    }
    for path in [*current_paths.values(), *legacy_paths.values()]:
        if not path.is_file():
            raise FileNotFoundError(path)
    mesh_path, mesh_candidate_index = first_existing_source(
        repo_root, sources["legacy"].get("road_mesh_candidates", []),
    )

    current_global = load_json(current_paths["global_info"])
    legacy_global = load_json(legacy_paths["global_info"])
    current_frame = LocalMapFrame.from_global_info(current_global)
    legacy_frame = LocalMapFrame.from_global_info(legacy_global)
    translation = origin_translation(legacy_frame, current_frame)

    current_links = load_json(current_paths["link_set"])
    legacy_links = load_json(legacy_paths["link_set"])
    legacy_lanes = load_json(legacy_paths["lane_boundary_set"])
    sampling = config["sampling"]
    matching = config["matching"]
    policy = _policy(config)

    current_link_points, current_link_tangents, current_link_owners = feature_cloud(
        current_links, float(sampling["link_step_m"]),
    )
    legacy_link_points, legacy_link_tangents, legacy_link_owners = feature_cloud(
        legacy_links, float(sampling["link_step_m"]), translation,
    )
    current_link_index = GridIndex(
        current_link_points, current_link_tangents, current_link_owners, 4.0,
    )
    legacy_link_index = GridIndex(
        legacy_link_points, legacy_link_tangents, legacy_link_owners, 4.0,
    )

    link_matches = [
        match_feature(
            link, translation, float(sampling["link_step_m"]), current_link_index, policy,
        )
        for link in legacy_links
    ]
    link_matches_by_id = {item["source_id"]: item for item in link_matches}

    route = _route_audit(
        _load_route(current_paths["route"]), legacy_link_index, current_link_index,
        policy, float(sampling["route_step_m"]),
        float(matching["close_route_gap_m"]), float(matching["remove_route_island_m"]),
    )
    route_regions = config.get("route_regions", {})
    region_bounds = {
        name: _region_station_bounds(route, region) for name, region in route_regions.items()
    }
    high_start, high_end = region_bounds["high_speed"]
    high_mask = (route["station_m"] >= high_start) & (route["station_m"] <= high_end)
    driven_high_current_links = {
        str(owner) for owner, keep in zip(route["current_owner"], high_mask) if keep and owner
    }
    high_current_links = _current_links_in_route_corridor(
        current_links,
        route["points"][high_mask],
        route["tangents"][high_mask],
        float(route_regions["high_speed"].get("corridor_half_width_m", 15.0)),
    )

    boundary_points, boundary_tangents, boundary_owners = _current_boundary_proxy(
        current_links, float(sampling["lane_step_m"]),
    )
    boundary_index = GridIndex(boundary_points, boundary_tangents, boundary_owners, 3.0)
    lane_matches = [
        _lane_match(
            lane, translation, float(sampling["lane_step_m"]), boundary_index, matching,
        )
        for lane in legacy_lanes
    ]
    lane_matches_by_id = {item["source_id"]: item for item in lane_matches}

    observed_rows = _observed_lane_audit(
        repo_root, sources["current_observed_lane_glob"], legacy_lanes,
        translation, float(sampling["lane_step_m"]),
    )
    road_mesh = _road_mesh_audit(
        mesh_path, current_links, translation, route, route_regions,
        float(sampling["road_raster_pixels_per_meter"]),
    )
    auxiliary_summary, auxiliary_rows = _auxiliary_audit(
        sources["legacy"], repo_root, link_matches_by_id, route["points"], translation,
    )

    excluded_colors = {
        str(value).lower() for value in config["promotion_policy"]["excluded_operational_lane_colors"]
    }
    high_candidates: list[dict] = []
    for lane in legacy_lanes:
        match = lane_matches_by_id[str(lane.get("idx", ""))]
        colors = {str(value).lower() for value in lane.get("lane_color", [])}
        targets = _target_ids(match)
        if (
            match["tier"] in {"A", "B"}
            and any(target in high_current_links for target in targets)
            and not colors.intersection(excluded_colors)
        ):
            high_candidates.append({
                "candidate_id": f"{config['map_id']}:lane_boundary:legacy:{lane['idx']}",
                "legacy_id": str(lane["idx"]),
                "tier": match["tier"],
                "target_current_link_ids": targets,
                "strict_coverage": match["strict_coverage"],
                "strict_distance_p95_m": match["strict_distance_p95_m"],
                "lane_color": list(lane.get("lane_color", [])),
                "lane_shape": list(lane.get("lane_shape", [])),
                "lane_type": list(lane.get("lane_type", [])),
                "status": "candidate",
            })

    source_records = {
        "current": {
            key: source_record(repo_root, sources["current"][key], path)
            for key, path in current_paths.items()
        },
        "legacy": {
            key: source_record(repo_root, sources["legacy"][key], path)
            for key, path in legacy_paths.items()
        },
    }
    if mesh_path is not None and mesh_candidate_index is not None:
        configured = sources["legacy"]["road_mesh_candidates"][mesh_candidate_index]
        source_records["legacy"]["road_mesh"] = source_record(repo_root, configured, mesh_path)

    lane_tiers = Counter(item["tier"] for item in lane_matches)
    link_tiers = Counter(item["tier"] for item in link_matches)
    lane_attributes: dict[str, Counter[str]] = {
        "color": Counter(), "shape": Counter(), "type": Counter(),
    }
    for item in lane_matches:
        if item["tier"] not in {"A", "B"}:
            continue
        lane_attributes["color"][item["lane_color"]] += 1
        lane_attributes["shape"][item["lane_shape"]] += 1
        lane_attributes["type"][item["lane_type"]] += 1

    summary = {
        "audit_id": config["audit_id"],
        "map_id": config["map_id"],
        "status": "analysis_only_no_operational_map_mutation",
        "coordinate_contract": {
            "formula": "target_local = source_local + source_origin - target_origin",
            "translation_legacy_to_current_xyz_m": translation.tolist(),
            "fitted_rotation_deg": 0.0,
            "fitted_scale": 1.0,
            "reason": "declared origins are authoritative; changed areas must not bias ICP",
        },
        "source_records": source_records,
        "counts": {
            "current_links": len(current_links),
            "legacy_links": len(legacy_links),
            "legacy_lane_boundaries": len(legacy_lanes),
        },
        "link_correspondence": {
            "tiers": dict(link_tiers),
            "mapped_feature_count": sum(bool(item["targets"]) for item in link_matches),
        },
        "route": {
            key: value for key, value in route.items()
            if key not in {"points", "tangents", "station_m", "reuse_mask", "current_owner", "legacy_owner"}
        },
        "route_regions": {
            name: {"start_s_m": bounds[0], "end_s_m": bounds[1]}
            for name, bounds in region_bounds.items()
        },
        "lane_boundary_candidates": {
            "tiers": dict(lane_tiers),
            "tier_a_or_b_attributes": {
                key: dict(counter) for key, counter in lane_attributes.items()
            },
            "high_speed_candidate_count": len(high_candidates),
            "comparison_target": "current_link_width_proxy_not_ground_truth",
        },
        "observed_lane_comparison": observed_rows,
        "road_mesh": road_mesh,
        "auxiliary_layers": auxiliary_summary,
        "decisions": {
            "current_link_node_are_topology_authority": True,
            "legacy_mgeo_is_input_adapter_not_runtime_schema": True,
            "high_speed_is_first_transfer_pilot": True,
            "legacy_features_remain_candidate_until_review": True,
            "layer_specific_gates_required": True,
        },
    }

    _write_json(output_dir / "summary.json", summary)
    _write_csv(
        output_dir / "link_correspondence.csv", _feature_rows(link_matches),
        [
            "source_id", "tier", "sample_count", "strict_coverage", "relaxed_coverage",
            "strict_distance_p50_m", "strict_distance_p95_m", "relaxed_distance_p95_m",
            "target_current_ids",
        ],
    )
    lane_rows = _feature_rows(lane_matches)
    for row, match in zip(lane_rows, lane_matches):
        row.update({
            "lane_color": match["lane_color"], "lane_shape": match["lane_shape"],
            "lane_type": match["lane_type"], "lane_width_m": match["lane_width_m"],
        })
    _write_csv(
        output_dir / "lane_transfer_candidates.csv", lane_rows,
        [
            "source_id", "tier", "sample_count", "strict_coverage", "relaxed_coverage",
            "strict_distance_p50_m", "strict_distance_p95_m", "relaxed_distance_p95_m",
            "target_current_ids", "lane_color", "lane_shape", "lane_type", "lane_width_m",
        ],
    )
    route_rows = [
        {
            "status": item["status"], "start_s_m": item["start_s_m"],
            "end_s_m": item["end_s_m"], "length_m": item["length_m"],
            "start_x": item["start_xy"][0], "start_y": item["start_xy"][1],
            "end_x": item["end_xy"][0], "end_y": item["end_xy"][1],
            "distance_p50_m": item["distance_p50_m"], "distance_p95_m": item["distance_p95_m"],
            "current_link_ids": ";".join(item["current_link_ids"]),
            "legacy_link_ids": ";".join(item["legacy_link_ids"]),
        }
        for item in route["intervals"] if item["length_m"] >= 3.0
    ]
    _write_csv(
        output_dir / "route_intervals.csv", route_rows,
        [
            "status", "start_s_m", "end_s_m", "length_m", "start_x", "start_y",
            "end_x", "end_y", "distance_p50_m", "distance_p95_m",
            "current_link_ids", "legacy_link_ids",
        ],
    )
    _write_csv(
        output_dir / "observed_lane_alignment.csv", observed_rows,
        [
            "source_file", "category", "sample_count", "collection_sources",
            "distance_p50_m", "distance_p95_m", "within_0_50_ratio",
            "within_0_75_ratio", "within_1_00_ratio",
        ],
    )
    _write_csv(
        output_dir / "auxiliary_transfer_candidates.csv", auxiliary_rows,
        [
            "layer", "source_id", "reference_tier", "route_distance_m",
            "legacy_link_ids", "sub_type",
        ],
    )
    _write_json(
        output_dir / "high_speed_candidate_index.json",
        {
            "map_id": config["map_id"],
            "status": "candidate",
            "operational": False,
            "route_region": {"start_s_m": high_start, "end_s_m": high_end},
            "driven_current_link_ids": sorted(driven_high_current_links),
            "current_link_ids": sorted(high_current_links),
            "features": high_candidates,
        },
    )

    # A compact, schema-valid example demonstrates the future canonical format
    # without copying the entire unreviewed legacy geometry into the repository.
    legacy_lane_sha = source_records["legacy"]["lane_boundary_set"]["sha256"]
    examples: list[dict] = []
    lanes_by_id = {str(lane["idx"]): lane for lane in legacy_lanes}
    for candidate in high_candidates[:3]:
        lane = lanes_by_id[candidate["legacy_id"]]
        coordinates = np.asarray(lane["points"], dtype=np.float64) + translation
        examples.append(make_candidate_feature(
            feature_type="lane_boundary",
            canonical_id=candidate["candidate_id"],
            points=coordinates,
            attributes={
                "mgeo": {
                    "lane_color": lane.get("lane_color", []),
                    "lane_shape": lane.get("lane_shape", []),
                    "lane_type": lane.get("lane_type", []),
                    "lane_width": lane.get("lane_width"),
                },
            },
            source_id=candidate["legacy_id"],
            source_sha256=legacy_lane_sha,
            target_link_ids=candidate["target_current_link_ids"],
            validation={
                "tier": candidate["tier"],
                "strict_coverage": candidate["strict_coverage"],
                "strict_distance_p95_m": candidate["strict_distance_p95_m"],
                "reviewed": False,
            },
        ))
    canonical_example = {
        "schema_version": SCHEMA_VERSION,
        "map_id": config["map_id"],
        "frame": {
            "frame_id": "map",
            "crs": current_frame.crs,
            "local_origin_in_global": list(current_frame.origin_xyz),
        },
        "sources": source_records,
        "features": examples,
    }
    validate_canonical_document(canonical_example)
    _write_json(output_dir / "canonical_candidate_example.json", canonical_example)
    return summary
