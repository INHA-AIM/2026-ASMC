"""Build the current-link/node-only competition corridor baseline."""

from __future__ import annotations

from collections import defaultdict
import csv
import json
import math
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from .corridor import (
    clip_parallel_link_points,
    corridor_relations,
    expand_context_links,
    polyline_length,
    route_centerline,
    route_link_records,
    route_segment,
    select_parallel_links,
    topology_indices,
    validate_official_route,
)
from .frames import LocalMapFrame
from .geometry import cumulative_distance, resample_polyline
from .io_mgeo import load_json, resolve_source, source_record
from .nominal import build_nominal_boundaries, build_route_stoplines
from .schema import SCHEMA_VERSION, validate_canonical_document
from .spatial import GridIndex


CONFIG_SCHEMA = "asmc.hd_map.corridor_config/v1"
ALLOWED_SOURCE_KEYS = {
    "current_global_info", "current_link_set", "current_node_set",
    "route_links", "organizer_route",
}


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


def _write_csv(path: Path, rows: list[dict], fields: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        for row in rows:
            writer.writerow({key: _plain(row.get(key)) for key in fields})


def _load_text_route(path: Path) -> list[list[float]]:
    points: list[list[float]] = []
    with path.open("r", encoding="utf-8") as stream:
        for line in stream:
            values = line.strip().replace(",", " ").split()
            if len(values) >= 2:
                points.append([
                    float(values[0]), float(values[1]),
                    float(values[2]) if len(values) >= 3 else 0.0,
                ])
    if len(points) < 2:
        raise ValueError(f"organizer route has fewer than two points: {path}")
    return points


def _load_route_link_ids(path: Path) -> list[str]:
    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    values = (data.get("route") or {}).get("links") or []
    if not values:
        raise ValueError(f"route.links is empty: {path}")
    return [str(value) for value in values]


def _project_link_to_route(link: dict, route_points: np.ndarray) -> dict:
    sampled_route, _ = resample_polyline(route_points, 1.0)
    route_station = cumulative_distance(sampled_route)
    route_index = GridIndex(sampled_route, cell_size_m=4.0)
    points, _ = resample_polyline(link.get("points", ()), 1.0)
    distances: list[float] = []
    stations: list[float] = []
    for point in points:
        distance, _, index = route_index.query(point, max_distance_m=50.0)
        if index is not None:
            distances.append(float(distance))
            stations.append(float(route_station[index]))
    if not distances:
        return {
            "route_s_min_m": None, "route_s_max_m": None,
            "route_distance_p50_m": None, "route_distance_p95_m": None,
        }
    return {
        "route_s_min_m": float(min(stations)),
        "route_s_max_m": float(max(stations)),
        "route_distance_p50_m": float(np.quantile(distances, 0.50)),
        "route_distance_p95_m": float(np.quantile(distances, 0.95)),
    }


def _canonical_feature(
    *,
    map_id: str,
    suffix: str,
    feature_type: str,
    points: np.ndarray | list,
    attributes: dict,
    topology: dict,
    source_kind: str,
    source_id: str,
    source_sha256: str,
    transform: str,
    validation: dict,
) -> dict:
    return {
        "id": f"{map_id}:{suffix}",
        "feature_type": feature_type,
        "frame_id": "map",
        "geometry": {
            "type": "LineString",
            "coordinates": _plain(np.asarray(points, dtype=np.float64)),
        },
        "attributes": _plain(attributes),
        "topology": _plain(topology),
        "provenance": {
            "source_kind": source_kind,
            "source_id": source_id,
            "source_sha256": source_sha256,
            "transform": transform,
        },
        "validation": _plain(validation),
        "status": "candidate",
    }


def _relative_direction(corridor_roles: set[str]) -> str:
    if "high_speed_opposite_direction" in corridor_roles:
        return "opposite_route_direction"
    if "high_speed_same_direction" in corridor_roles:
        return "same_route_direction"
    if any(role.endswith("_opposite") for role in corridor_roles):
        return "opposite_route_direction"
    if "official_route" in corridor_roles or any(
        role.endswith("_primary") for role in corridor_roles
    ):
        return "same_route_direction"
    return "topology_context_unknown"


def build_current_corridor(config_path: Path, repo_root: Path, output_dir: Path) -> dict:
    with config_path.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream) or {}
    if config.get("schema_version") != CONFIG_SCHEMA:
        raise ValueError("unsupported current corridor config schema")
    source_config = config.get("sources") or {}
    unknown_sources = set(source_config).difference(ALLOWED_SOURCE_KEYS)
    missing_sources = ALLOWED_SOURCE_KEYS.difference(source_config)
    if unknown_sources or missing_sources:
        raise ValueError(
            f"current-only source contract violation: unknown={sorted(unknown_sources)} "
            f"missing={sorted(missing_sources)}"
        )
    if config["promotion_policy"].get("observed_or_legacy_inputs_allowed") is not False:
        raise ValueError("nominal corridor baseline must forbid observed and legacy inputs")

    paths = {
        key: resolve_source(repo_root, raw) for key, raw in source_config.items()
    }
    for path in paths.values():
        if not path.is_file():
            raise FileNotFoundError(path)

    global_info = load_json(paths["current_global_info"])
    current_frame = LocalMapFrame.from_global_info(global_info)
    links = load_json(paths["current_link_set"])
    nodes = load_json(paths["current_node_set"])
    links_by_id = {str(link["idx"]): link for link in links}
    route_ids = _load_route_link_ids(paths["route_links"])
    expected_route_count = int(config["route"]["expected_link_count"])
    if len(route_ids) != expected_route_count:
        raise ValueError(f"expected {expected_route_count} route links, got {len(route_ids)}")
    route_records = route_link_records(route_ids, links_by_id)
    current_route = route_centerline(route_ids, links_by_id)
    official_route = _load_text_route(paths["organizer_route"])
    route_validation = validate_official_route(
        current_route, official_route,
        sample_step_m=float(config["route"]["validation_sample_step_m"]),
    )
    if route_validation["distance_p95_m"] > float(config["route"]["maximum_distance_p95_m"]):
        raise ValueError(f"route p95 distance exceeded: {route_validation}")
    if abs(route_validation["length_difference_m"]) > float(
        config["route"]["maximum_length_difference_m"]
    ):
        raise ValueError(f"route length difference exceeded: {route_validation}")

    incoming, outgoing = topology_indices(links)
    roles: dict[str, set[str]] = defaultdict(set)
    for link_id in route_ids:
        roles[link_id].add("official_route")

    high_config = config["corridors"]["high_speed"]
    high_points, high_tangents = route_segment(
        route_ids, links_by_id,
        str(high_config["start_link"]), str(high_config["end_link"]),
    )
    high_ids = select_parallel_links(
        links, high_points, high_tangents,
        half_width_m=float(high_config["half_width_m"]),
        max_heading_deg=float(high_config["max_heading_deg"]),
        minimum_support=int(high_config["minimum_support"]),
        undirected_heading=True,
    )
    high_same_direction_ids = select_parallel_links(
        links, high_points, high_tangents,
        half_width_m=float(high_config["half_width_m"]),
        max_heading_deg=float(high_config["max_heading_deg"]),
        minimum_support=int(high_config["minimum_support"]),
        undirected_heading=False,
    )
    for link_id in high_ids:
        roles[link_id].add("high_speed_parallel")
        roles[link_id].add(
            "high_speed_same_direction"
            if link_id in high_same_direction_ids
            else "high_speed_opposite_direction"
        )

    for name in ("shadow", "static_obstacle"):
        section = config["corridors"][name]
        primary = {str(value) for value in section["primary_links"]}
        opposite = {str(value) for value in section["opposite_links"]}
        seeds = primary | opposite
        missing = seeds.difference(links_by_id)
        if missing:
            raise ValueError(f"{name} seed links missing: {sorted(missing)}")
        expanded = expand_context_links(
            seeds, links_by_id, incoming, outgoing,
            hops=int(section["topology_hops"]),
            max_heading_deg=float(section["max_heading_deg"]),
        )
        for link_id in primary:
            roles[link_id].add(f"{name}_primary")
        for link_id in opposite:
            roles[link_id].add(f"{name}_opposite")
        for link_id in expanded.difference(seeds):
            roles[link_id].add(f"{name}_connector")

    corridor_ids = set(roles)

    # A parallel current link may only touch the high-speed route for a short
    # interval and then continue into an unrelated road.  Keep the current link
    # ID/topology authoritative, but export only the route-corridor portion for
    # high-speed-only context links.  Official-route and mission links retain
    # their complete current geometry.
    geometry_links_by_id = dict(links_by_id)
    geometry_scope: dict[str, dict] = {}
    for link_id in corridor_ids:
        source_link = links_by_id[link_id]
        scope = {
            "geometry_scope": "full_current_link",
            "original_length_m": float(polyline_length(source_link.get("points", ()))),
            "corridor_length_m": float(polyline_length(source_link.get("points", ()))),
            "valid_run_count": 1,
            "discarded_valid_run_count": 0,
        }
        if (
            "high_speed_parallel" in roles[link_id]
            and roles[link_id].issubset({
                "high_speed_parallel",
                "high_speed_same_direction",
                "high_speed_opposite_direction",
            })
        ):
            clipped_points, clip_stats = clip_parallel_link_points(
                source_link,
                high_points,
                high_tangents,
                half_width_m=float(high_config["half_width_m"]),
                max_heading_deg=float(high_config["max_heading_deg"]),
                sample_step_m=float(high_config["clip_sample_step_m"]),
                minimum_segment_length_m=float(high_config["minimum_clip_length_m"]),
            )
            clipped_link = dict(source_link)
            clipped_link["points"] = clipped_points.tolist()
            geometry_links_by_id[link_id] = clipped_link
            scope = {"geometry_scope": "high_speed_route_corridor_clip", **clip_stats}
        geometry_scope[link_id] = scope

    explicit_opposites = {
        str(key): str(value) for key, value in config.get("explicit_opposite_pairs", {}).items()
    }
    relations = {
        link_id: corridor_relations(
            link_id, corridor_ids, links_by_id, incoming, outgoing, explicit_opposites,
        )
        for link_id in corridor_ids
    }
    exact_route = {record["link_id"]: record for record in route_records}
    corridor_rows: list[dict] = []
    for link_id in sorted(corridor_ids):
        projection = _project_link_to_route(geometry_links_by_id[link_id], current_route)
        route_record = exact_route.get(link_id)
        if route_record:
            projection.update({
                "route_s_min_m": route_record["start_s_m"],
                "route_s_max_m": route_record["end_s_m"],
                "route_distance_p50_m": 0.0,
                "route_distance_p95_m": 0.0,
            })
        corridor_rows.append({
            "link_id": link_id,
            "roles": ";".join(sorted(roles[link_id])),
            "relative_direction": _relative_direction(roles[link_id]),
            "road_id": str(links_by_id[link_id].get("road_id", "")),
            "ego_lane": links_by_id[link_id].get("ego_lane"),
            "link_type": str(links_by_id[link_id].get("link_type", "")),
            "width_start_m": links_by_id[link_id].get("width_start"),
            "width_end_m": links_by_id[link_id].get("width_end"),
            **geometry_scope[link_id],
            **projection,
            "predecessors": ";".join(relations[link_id]["predecessor_link_ids"]),
            "successors": ";".join(relations[link_id]["successor_link_ids"]),
            "left_adjacent": relations[link_id]["left_adjacent_link_id"],
            "right_adjacent": relations[link_id]["right_adjacent_link_id"],
            "opposite": relations[link_id]["opposite_link_id"],
            "is_merge_target": relations[link_id]["is_merge_target"],
            "is_split_source": relations[link_id]["is_split_source"],
        })

    lane_config = config["nominal_lane_boundary"]
    nominal_boundaries, nominal_stats = build_nominal_boundaries(
        corridor_ids, geometry_links_by_id, relations,
        step_m=float(lane_config["resample_step_m"]),
        minimum_link_length_m=float(lane_config["minimum_link_length_m"]),
        excluded_link_types={str(value) for value in lane_config["excluded_link_types"]},
        shared_boundary_max_p95_m=float(lane_config["shared_boundary_max_p95_m"]),
    )
    stop_config = config["stopline"]
    stoplines, stopline_stats = build_route_stoplines(
        route_records, route_ids, links, nodes,
        group_radius_m=float(stop_config["group_radius_m"]),
        max_heading_diff_deg=float(stop_config["max_heading_diff_deg"]),
        max_longitudinal_m=float(stop_config["max_longitudinal_m"]),
    )
    if stopline_stats["route_stop_node_count"] != int(stop_config["expected_route_stop_node_count"]):
        raise ValueError(f"route stop-node count changed: {stopline_stats}")

    source_records = {
        key: source_record(repo_root, source_config[key], path)
        for key, path in paths.items()
    }
    link_sha = source_records["current_link_set"]["sha256"]
    node_sha = source_records["current_node_set"]["sha256"]
    map_id = str(config["map_id"])
    features: list[dict] = []
    rows_by_id = {row["link_id"]: row for row in corridor_rows}
    for link_id in sorted(corridor_ids):
        link = geometry_links_by_id[link_id]
        row = rows_by_id[link_id]
        features.append(_canonical_feature(
            map_id=map_id,
            suffix=f"link_centerline:{link_id}",
            feature_type="link_centerline",
            points=link["points"],
            attributes={
                "corridor_roles": sorted(roles[link_id]),
                "relative_direction": row["relative_direction"],
                "road_id": link.get("road_id"),
                "ego_lane": link.get("ego_lane"),
                "link_type": str(link.get("link_type", "")),
                "nominal_width_start_m": link.get("width_start"),
                "nominal_width_end_m": link.get("width_end"),
                "width_status": "nominal_width",
                "geometry_scope": row["geometry_scope"],
                "original_length_m": row["original_length_m"],
                "exported_length_m": row["corridor_length_m"],
                "route_s_min_m": row["route_s_min_m"],
                "route_s_max_m": row["route_s_max_m"],
            },
            topology={"current_link_ids": [link_id], **relations[link_id]},
            source_kind="current_mgeo",
            source_id=link_id,
            source_sha256=link_sha,
            transform=(
                "current_geometry_high_speed_corridor_clip"
                if row["geometry_scope"] == "high_speed_route_corridor_clip"
                else "none"
            ),
            validation={
                "topology_authority": True,
                "geometry_is_painted_lane_ground_truth": False,
                "discarded_valid_run_count": row["discarded_valid_run_count"],
            },
        ))
    for boundary in nominal_boundaries:
        features.append(_canonical_feature(
            map_id=map_id,
            suffix=boundary["id_suffix"],
            feature_type="lane_boundary",
            points=boundary["points"],
            attributes={
                "boundary_role": boundary["boundary_role"],
                "link_sides": boundary["link_sides"],
                "lane_color": "unknown",
                "lane_shape": "unknown",
                "width_source": "current_link_width_start_end",
                "nominal_width_start_m": boundary["width_start_m"],
                "nominal_width_end_m": boundary["width_end_m"],
                "geometry_status": boundary["geometry_status"],
            },
            topology={"current_link_ids": boundary["current_link_ids"]},
            source_kind="link_width_proxy",
            source_id="+".join(boundary["current_link_ids"]),
            source_sha256=link_sha,
            transform="current_link_normal_offset",
            validation={
                "operational": False,
                "painted_lane_ground_truth": False,
                "shared_boundary_residual": boundary["shared_boundary_residual"],
            },
        ))
    for stopline in stoplines:
        features.append(_canonical_feature(
            map_id=map_id,
            suffix=stopline["id_suffix"],
            feature_type="stopline",
            points=stopline["points"],
            attributes={
                "node_ids": stopline["node_ids"],
                "route_node_ids": stopline["route_node_ids"],
                "route_s_m": stopline["route_s_m"],
                "length_m": stopline["length_m"],
                "heading_xy": stopline["heading_xy"],
                "bar_direction_xy": stopline["bar_direction_xy"],
                "traffic_light_ids": stopline["traffic_light_ids"],
                "signal_controlled": stopline["signal_controlled"],
                "geometry_status": stopline["geometry_status"],
            },
            topology={"current_link_ids": stopline["current_link_ids"]},
            source_kind="current_mgeo",
            source_id="+".join(stopline["node_ids"]),
            source_sha256=node_sha,
            transform="current_node_plus_link_heading_and_width",
            validation={
                "operational": False,
                "painted_stopline_ground_truth": False,
                "legacy_or_camera_validated": False,
            },
        ))

    canonical = {
        "schema_version": SCHEMA_VERSION,
        "map_id": map_id,
        "frame": {
            "frame_id": "map",
            "crs": current_frame.crs,
            "local_origin_in_global": list(current_frame.origin_xyz),
        },
        "sources": source_records,
        "build": {
            "build_id": config["build_id"],
            "stage": "current_link_node_nominal_baseline",
            "operational": False,
            "observed_or_legacy_inputs_used": False,
        },
        "features": features,
    }
    validate_canonical_document(canonical)

    role_counts: dict[str, int] = defaultdict(int)
    for values in roles.values():
        for role in values:
            role_counts[role] += 1
    width_pairs = {
        (float(link.get("width_start") or 3.5), float(link.get("width_end") or 3.5))
        for link in links
    }
    summary = {
        "build_id": config["build_id"],
        "map_id": map_id,
        "status": "candidate_current_link_node_nominal_baseline",
        "operational": False,
        "route": {
            "link_count": len(route_ids),
            "closed_loop": bool(config["route"]["closed_loop"]),
            "validation": route_validation,
        },
        "corridor": {
            "unique_link_count": len(corridor_ids),
            "role_counts": dict(sorted(role_counts.items())),
            "high_speed_parallel_link_count": len(high_ids),
            "high_speed_same_direction_link_count": len(high_same_direction_ids),
            "high_speed_opposite_direction_link_count": len(
                high_ids.difference(high_same_direction_ids)
            ),
            "high_speed_clipped_link_count": sum(
                item["geometry_scope"] == "high_speed_route_corridor_clip"
                for item in geometry_scope.values()
            ),
            "high_speed_discarded_valid_run_count": sum(
                int(item["discarded_valid_run_count"])
                for item in geometry_scope.values()
            ),
            "shadow_seed_link_ids": sorted(
                set(config["corridors"]["shadow"]["primary_links"])
                | set(config["corridors"]["shadow"]["opposite_links"])
            ),
            "static_obstacle_seed_link_ids": sorted(
                set(config["corridors"]["static_obstacle"]["primary_links"])
                | set(config["corridors"]["static_obstacle"]["opposite_links"])
            ),
        },
        "nominal_lane_boundaries": {
            **nominal_stats,
            "width_pairs_in_full_current_map_m": [list(pair) for pair in sorted(width_pairs)],
            "painted_lane_ground_truth": False,
        },
        "stoplines": {
            **stopline_stats,
            "painted_stopline_ground_truth": False,
        },
        "source_contract": {
            "current_only": True,
            "observed_or_legacy_inputs_used": False,
            "source_records": source_records,
        },
        "next_stage": "observed_lane_calibration_and_legacy_stable_gap_fill",
    }

    _write_json(output_dir / "summary.json", summary)
    _write_json(output_dir / "nominal_candidate_map.json", canonical)
    _write_csv(
        output_dir / "corridor_links.csv", corridor_rows,
        [
            "link_id", "roles", "relative_direction", "road_id", "ego_lane", "link_type",
            "width_start_m", "width_end_m", "geometry_scope",
            "original_length_m", "corridor_length_m", "valid_run_count",
            "discarded_valid_run_count", "route_s_min_m", "route_s_max_m",
            "route_distance_p50_m", "route_distance_p95_m", "predecessors",
            "successors", "left_adjacent", "right_adjacent", "opposite",
            "is_merge_target", "is_split_source",
        ],
    )
    boundary_rows = []
    for boundary in nominal_boundaries:
        residual = boundary["shared_boundary_residual"] or {}
        boundary_rows.append({
            "candidate_id": f"{map_id}:{boundary['id_suffix']}",
            "geometry_status": boundary["geometry_status"],
            "boundary_role": boundary["boundary_role"],
            "current_link_ids": ";".join(boundary["current_link_ids"]),
            "link_sides": json.dumps(boundary["link_sides"], sort_keys=True),
            "nominal_width_start_m": ";".join(str(value) for value in boundary["width_start_m"]),
            "nominal_width_end_m": ";".join(str(value) for value in boundary["width_end_m"]),
            "shared_distance_p95_m": residual.get("distance_p95_m"),
            "point_count": len(boundary["points"]),
        })
    _write_csv(
        output_dir / "nominal_lane_boundaries.csv", boundary_rows,
        [
            "candidate_id", "geometry_status", "boundary_role", "current_link_ids",
            "link_sides", "nominal_width_start_m", "nominal_width_end_m",
            "shared_distance_p95_m", "point_count",
        ],
    )
    stopline_rows = [{
        "candidate_id": f"{map_id}:{item['id_suffix']}",
        "route_s_m": item["route_s_m"],
        "route_node_ids": ";".join(item["route_node_ids"]),
        "group_node_ids": ";".join(item["node_ids"]),
        "current_link_ids": ";".join(item["current_link_ids"]),
        "length_m": item["length_m"],
        "traffic_light_ids": ";".join(item["traffic_light_ids"]),
        "signal_controlled": item["signal_controlled"],
        "geometry_status": item["geometry_status"],
    } for item in stoplines]
    _write_csv(
        output_dir / "route_stopline_candidates.csv", stopline_rows,
        [
            "candidate_id", "route_s_m", "route_node_ids", "group_node_ids",
            "current_link_ids", "length_m", "traffic_light_ids",
            "signal_controlled", "geometry_status",
        ],
    )
    return summary
