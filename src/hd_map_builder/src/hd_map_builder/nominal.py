"""Nominal lane-boundary and stop-line candidates from current link/node data."""

from __future__ import annotations

from collections import defaultdict
import math

import numpy as np

from .corridor import (
    heading_difference_deg,
    polyline_length,
    topology_indices,
    unit_heading,
)
from .geometry import cumulative_distance, resample_polyline
from .spatial import GridIndex


def variable_width_boundaries(link: dict, step_m: float) -> dict[str, np.ndarray]:
    points, tangents = resample_polyline(link.get("points", ()), step_m)
    if len(points) < 2:
        return {"left": np.zeros((0, 3)), "right": np.zeros((0, 3))}
    station = cumulative_distance(points)
    ratio = station / max(float(station[-1]), 1e-9)
    width_start = float(link.get("width_start") or link.get("width_end") or 3.5)
    width_end = float(link.get("width_end") or link.get("width_start") or 3.5)
    half_width = 0.5 * (width_start + (width_end - width_start) * ratio)
    normals = np.column_stack((-tangents[:, 1], tangents[:, 0]))
    left = points.copy()
    right = points.copy()
    left[:, :2] += half_width[:, None] * normals
    right[:, :2] -= half_width[:, None] * normals
    return {"left": left, "right": right}


def _shared_boundary_geometry(first: np.ndarray, second: np.ndarray) -> tuple[np.ndarray, dict]:
    first_points, first_tangents = resample_polyline(first, 0.5)
    second_points, second_tangents = resample_polyline(second, 0.5)
    if len(first_points) < 2 or len(second_points) < 2:
        return first, {"sample_count": 0, "distance_p95_m": None, "averaged": False}
    index = GridIndex(second_points, second_tangents, cell_size_m=2.0)
    result = first_points.copy()
    distances: list[float] = []
    averaged = np.zeros(len(first_points), dtype=bool)
    for row, (point, tangent) in enumerate(zip(first_points, first_tangents)):
        distance, heading, match = index.query(
            point, tangent, max_distance_m=3.0, max_heading_deg=30.0,
        )
        if match is None or distance > 3.0 or heading > 30.0:
            continue
        result[row] = 0.5 * (point + second_points[match])
        distances.append(float(distance))
        averaged[row] = True
    if len(distances) < 2:
        return first_points, {
            "sample_count": len(distances), "distance_p95_m": None, "averaged": False,
        }
    return result, {
        "sample_count": len(distances),
        "distance_p50_m": float(np.quantile(distances, 0.50)),
        "distance_p95_m": float(np.quantile(distances, 0.95)),
        "averaged": bool(np.mean(averaged) >= 0.80),
        "averaged_coverage": float(np.mean(averaged)),
    }


def build_nominal_boundaries(
    corridor_ids: set[str],
    links_by_id: dict[str, dict],
    relations_by_id: dict[str, dict],
    *,
    step_m: float,
    minimum_link_length_m: float,
    excluded_link_types: set[str],
    shared_boundary_max_p95_m: float,
) -> tuple[list[dict], dict]:
    raw: dict[tuple[str, str], dict] = {}
    skipped: dict[str, str] = {}
    for link_id in sorted(corridor_ids):
        link = links_by_id[link_id]
        length = polyline_length(link.get("points", ()))
        link_type = str(link.get("link_type", ""))
        if length < minimum_link_length_m:
            skipped[link_id] = "short_link"
            continue
        if link_type in excluded_link_types:
            skipped[link_id] = "intersection_connector"
            continue
        geometry = variable_width_boundaries(link, step_m)
        relation = relations_by_id[link_id]
        for side, adjacent_key in (
            ("left", "left_adjacent_link_id"),
            ("right", "right_adjacent_link_id"),
        ):
            raw[(link_id, side)] = {
                "link_id": link_id,
                "side": side,
                "adjacent_link_id": relation.get(adjacent_key),
                "points": geometry[side],
                "width_start_m": float(link.get("width_start") or link.get("width_end") or 3.5),
                "width_end_m": float(link.get("width_end") or link.get("width_start") or 3.5),
                "link_type": link_type,
                "review_required": bool(
                    link_type not in {"6"}
                    or relation.get("is_merge_target")
                    or relation.get("is_split_source")
                ),
            }

    features: list[dict] = []
    consumed: set[tuple[str, str]] = set()
    for key in sorted(raw):
        if key in consumed:
            continue
        item = raw[key]
        adjacent = item["adjacent_link_id"]
        opposite_side = "right" if item["side"] == "left" else "left"
        other_key = (str(adjacent), opposite_side) if adjacent else None
        if other_key is not None and other_key in raw:
            other = raw[other_key]
            pair = sorted((item["link_id"], other["link_id"]))
            geometry, residual = _shared_boundary_geometry(item["points"], other["points"])
            review = (
                item["review_required"]
                or other["review_required"]
                or residual.get("distance_p95_m") is None
                or not bool(residual.get("averaged"))
                or float(residual["distance_p95_m"]) > shared_boundary_max_p95_m
            )
            features.append({
                "id_suffix": f"lane_boundary:shared:{pair[0]}:{pair[1]}",
                "points": geometry,
                "current_link_ids": pair,
                "link_sides": {
                    item["link_id"]: item["side"],
                    other["link_id"]: other["side"],
                },
                "boundary_role": "shared",
                "geometry_status": "review_required" if review else "link_nominal",
                "width_start_m": [item["width_start_m"], other["width_start_m"]],
                "width_end_m": [item["width_end_m"], other["width_end_m"]],
                "shared_boundary_residual": residual,
            })
            consumed.add(key)
            consumed.add(other_key)
            continue

        features.append({
            "id_suffix": f"lane_boundary:{item['link_id']}:{item['side']}",
            "points": item["points"],
            "current_link_ids": [item["link_id"]],
            "link_sides": {item["link_id"]: item["side"]},
            "boundary_role": "corridor_edge",
            "geometry_status": "review_required" if item["review_required"] else "link_nominal",
            "width_start_m": [item["width_start_m"]],
            "width_end_m": [item["width_end_m"]],
            "shared_boundary_residual": None,
        })
        consumed.add(key)

    return features, {
        "eligible_link_count": len({item["link_id"] for item in raw.values()}),
        "raw_side_count": len(raw),
        "deduplicated_boundary_count": len(features),
        "shared_boundary_count": sum(x["boundary_role"] == "shared" for x in features),
        "review_required_count": sum(x["geometry_status"] == "review_required" for x in features),
        "skipped_links": skipped,
    }


def node_heading_and_width(
    node_id: str,
    links_by_id: dict[str, dict],
    incoming: dict[str, list[str]],
    outgoing: dict[str, list[str]],
) -> tuple[np.ndarray, float] | None:
    for link_ids, at_end in ((incoming.get(node_id, []), True), (outgoing.get(node_id, []), False)):
        headings: list[np.ndarray] = []
        widths: list[float] = []
        for link_id in link_ids:
            link = links_by_id[link_id]
            heading = unit_heading(link.get("points", ()), at_end=at_end)
            if heading is not None:
                headings.append(heading)
                widths.append(float(link.get("width_end" if at_end else "width_start") or 3.5))
        if headings:
            mean = np.mean(np.stack(headings), axis=0)
            norm = float(np.linalg.norm(mean))
            if norm < 1e-6:
                mean = headings[0]
                norm = 1.0
            return mean / norm, float(np.mean(widths))
    return None


def _stopline_components(
    stop_nodes: list[dict],
    headings: dict[str, np.ndarray],
    *,
    group_radius_m: float,
    max_heading_diff_deg: float,
    max_longitudinal_m: float,
) -> list[list[str]]:
    parent = {str(node["idx"]): str(node["idx"]) for node in stop_nodes}

    def find(value: str) -> str:
        while parent[value] != value:
            parent[value] = parent[parent[value]]
            value = parent[value]
        return value

    def union(first: str, second: str) -> None:
        root_first, root_second = find(first), find(second)
        if root_first != root_second:
            parent[root_second] = root_first

    for index, first in enumerate(stop_nodes):
        first_id = str(first["idx"])
        first_heading = headings.get(first_id)
        if first_heading is None:
            continue
        first_point = np.asarray(first["point"], dtype=np.float64)[:2]
        for second in stop_nodes[index + 1:]:
            second_id = str(second["idx"])
            second_heading = headings.get(second_id)
            if second_heading is None:
                continue
            first_signal = first.get("traffic_light_id")
            second_signal = second.get("traffic_light_id")
            if first_signal and second_signal and first_signal != second_signal:
                continue
            if heading_difference_deg(first_heading, second_heading) > max_heading_diff_deg:
                continue
            offset = np.asarray(second["point"], dtype=np.float64)[:2] - first_point
            if float(np.linalg.norm(offset)) > group_radius_m:
                continue
            mean_heading = first_heading + second_heading
            norm = float(np.linalg.norm(mean_heading))
            if norm < 1e-6:
                continue
            if abs(float(np.dot(offset, mean_heading / norm))) > max_longitudinal_m:
                continue
            union(first_id, second_id)

    groups: dict[str, list[str]] = defaultdict(list)
    for node_id in parent:
        groups[find(node_id)].append(node_id)
    return [sorted(group) for group in groups.values()]


def build_route_stoplines(
    route_link_records: list[dict],
    route_link_ids: list[str],
    links: list[dict],
    nodes: list[dict],
    *,
    group_radius_m: float,
    max_heading_diff_deg: float,
    max_longitudinal_m: float,
) -> tuple[list[dict], dict]:
    links_by_id = {str(link["idx"]): link for link in links}
    nodes_by_id = {str(node["idx"]): node for node in nodes}
    incoming, outgoing = topology_indices(links)
    station_by_node: dict[str, list[float]] = defaultdict(list)
    for record in route_link_records:
        link = links_by_id[record["link_id"]]
        station_by_node[str(link["from_node_idx"])].append(float(record["start_s_m"]))
        station_by_node[str(link["to_node_idx"])].append(float(record["end_s_m"]))
    route_stop_ids = {
        node_id for node_id in station_by_node
        if bool(nodes_by_id[node_id].get("on_stop_line"))
    }

    stop_nodes = [node for node in nodes if bool(node.get("on_stop_line"))]
    heading_width: dict[str, tuple[np.ndarray, float]] = {}
    for node in stop_nodes:
        value = node_heading_and_width(str(node["idx"]), links_by_id, incoming, outgoing)
        if value is not None:
            heading_width[str(node["idx"])] = value
    components = _stopline_components(
        stop_nodes, {key: value[0] for key, value in heading_width.items()},
        group_radius_m=group_radius_m,
        max_heading_diff_deg=max_heading_diff_deg,
        max_longitudinal_m=max_longitudinal_m,
    )

    features: list[dict] = []
    for group in components:
        route_members = sorted(set(group).intersection(route_stop_ids))
        if not route_members:
            continue
        valid_members = [node_id for node_id in group if node_id in heading_width]
        if not valid_members:
            continue
        centers = np.asarray([nodes_by_id[node_id]["point"][:2] for node_id in valid_members])
        headings = np.asarray([heading_width[node_id][0] for node_id in valid_members])
        mean_heading = np.mean(headings, axis=0)
        mean_heading /= max(float(np.linalg.norm(mean_heading)), 1e-9)
        nominal_bar = np.asarray([-mean_heading[1], mean_heading[0]])
        if len(valid_members) >= 2:
            _, _, vt = np.linalg.svd(centers - np.mean(centers, axis=0))
            bar = vt[0]
            if float(np.dot(bar, nominal_bar)) < 0:
                bar = -bar
            perpendicular_error = abs(90.0 - math.degrees(math.acos(
                min(1.0, abs(float(np.dot(bar, mean_heading))))
            )))
            if perpendicular_error > 20.0:
                bar = nominal_bar
        else:
            bar = nominal_bar

        normal = np.asarray([-bar[1], bar[0]])
        endpoint_projection: list[float] = []
        normal_projection: list[float] = []
        for node_id, center in zip(valid_members, centers):
            width = heading_width[node_id][1]
            endpoint_projection.extend([
                float(np.dot(center - 0.5 * width * bar, bar)),
                float(np.dot(center + 0.5 * width * bar, bar)),
            ])
            normal_projection.append(float(np.dot(center, normal)))
        low, high = min(endpoint_projection), max(endpoint_projection)
        normal_value = float(np.mean(normal_projection))
        start = low * bar + normal_value * normal
        end = high * bar + normal_value * normal
        z = float(np.mean([nodes_by_id[node_id]["point"][2] for node_id in valid_members]))
        signals = sorted({
            str(nodes_by_id[node_id].get("traffic_light_id"))
            for node_id in valid_members if nodes_by_id[node_id].get("traffic_light_id")
        })
        associated_links = sorted({
            link_id for node_id in valid_members
            for link_id in incoming.get(node_id, []) + outgoing.get(node_id, [])
        })
        route_stations = [
            value for node_id in route_members for value in station_by_node[node_id]
        ]
        # The route is closed; a stop line at the closure would have both 0 and
        # total-length stations.  Prefer the smaller representation.
        route_s = float(min(route_stations))
        suffix = "+".join(route_members)
        features.append({
            "id_suffix": f"stopline:{suffix}",
            "points": np.asarray([[start[0], start[1], z], [end[0], end[1], z]]),
            "current_link_ids": associated_links,
            "node_ids": valid_members,
            "route_node_ids": route_members,
            "route_s_m": route_s,
            "length_m": float(high - low),
            "heading_xy": mean_heading.tolist(),
            "bar_direction_xy": bar.tolist(),
            "traffic_light_ids": signals,
            "signal_controlled": bool(signals),
            "geometry_status": "link_nominal",
        })

    features.sort(key=lambda feature: feature["route_s_m"])
    return features, {
        "current_stop_node_count": len(stop_nodes),
        "route_stop_node_count": len(route_stop_ids),
        "route_stopline_feature_count": len(features),
        "route_stop_node_ids": sorted(route_stop_ids),
        "unresolved_route_stop_node_ids": sorted(
            route_stop_ids.difference({
                node_id for feature in features for node_id in feature["route_node_ids"]
            })
        ),
    }
