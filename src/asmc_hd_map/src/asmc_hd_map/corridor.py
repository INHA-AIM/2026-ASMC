"""Current-map competition corridor and topology helpers.

This module deliberately depends only on the K-City 2025 link/node data and
the organizer route.  Legacy KATRI geometry and observed lane CSVs belong to
later calibration stages and must not influence this nominal baseline.
"""

from __future__ import annotations

from collections import defaultdict
import math
from typing import Iterable

import numpy as np

from .geometry import cumulative_distance, points_xyz, resample_polyline
from .spatial import GridIndex


def polyline_length(points: Iterable[Iterable[float]]) -> float:
    values = points_xyz(points)
    if len(values) < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(values[:, :2], axis=0), axis=1).sum())


def unit_heading(points: Iterable[Iterable[float]], *, at_end: bool) -> np.ndarray | None:
    values = points_xyz(points)
    if len(values) < 2:
        return None
    pairs = zip(values[-2::-1], values[:0:-1]) if at_end else zip(values[:-1], values[1:])
    for start, end in pairs:
        vector = end[:2] - start[:2]
        norm = float(np.linalg.norm(vector))
        if norm > 1e-9:
            return vector / norm
    return None


def heading_difference_deg(first: np.ndarray | None, second: np.ndarray | None) -> float:
    if first is None or second is None:
        return math.inf
    dot = float(np.clip(np.dot(first, second), -1.0, 1.0))
    return math.degrees(math.acos(dot))


def topology_indices(links: list[dict]) -> tuple[dict[str, list[str]], dict[str, list[str]]]:
    incoming: dict[str, list[str]] = defaultdict(list)
    outgoing: dict[str, list[str]] = defaultdict(list)
    for link in links:
        link_id = str(link["idx"])
        incoming[str(link.get("to_node_idx", ""))].append(link_id)
        outgoing[str(link.get("from_node_idx", ""))].append(link_id)
    return dict(incoming), dict(outgoing)


def validate_route_chain(route_link_ids: list[str], links_by_id: dict[str, dict]) -> None:
    if not route_link_ids:
        raise ValueError("route link list is empty")
    if len(route_link_ids) != len(set(route_link_ids)):
        raise ValueError("route link list contains duplicate IDs")
    missing = [link_id for link_id in route_link_ids if link_id not in links_by_id]
    if missing:
        raise ValueError(f"route links missing from current map: {missing}")
    pairs = list(zip(route_link_ids, route_link_ids[1:] + route_link_ids[:1]))
    for first_id, second_id in pairs:
        first, second = links_by_id[first_id], links_by_id[second_id]
        if str(first.get("to_node_idx")) != str(second.get("from_node_idx")):
            raise ValueError(
                f"route topology gap: {first_id}:{first.get('to_node_idx')} -> "
                f"{second_id}:{second.get('from_node_idx')}"
            )


def route_link_records(route_link_ids: list[str], links_by_id: dict[str, dict]) -> list[dict]:
    validate_route_chain(route_link_ids, links_by_id)
    station = 0.0
    records: list[dict] = []
    for order, link_id in enumerate(route_link_ids):
        length = polyline_length(links_by_id[link_id].get("points", ()))
        records.append({
            "link_id": link_id,
            "route_order": order,
            "start_s_m": station,
            "end_s_m": station + length,
            "length_m": length,
        })
        station += length
    return records


def route_centerline(route_link_ids: list[str], links_by_id: dict[str, dict]) -> np.ndarray:
    points: list[list[float]] = []
    for link_id in route_link_ids:
        for point in links_by_id[link_id].get("points", ()):
            row = [float(value) for value in point[:3]]
            if len(row) == 2:
                row.append(0.0)
            if not points or float(np.linalg.norm(np.asarray(points[-1][:2]) - row[:2])) > 1e-7:
                points.append(row)
    return points_xyz(points)


def validate_official_route(
    current_route: np.ndarray,
    official_route: Iterable[Iterable[float]],
    *,
    sample_step_m: float = 1.0,
) -> dict:
    current_points, current_tangents = resample_polyline(current_route, sample_step_m)
    official_points, official_tangents = resample_polyline(official_route, sample_step_m)
    current_index = GridIndex(current_points, current_tangents, cell_size_m=4.0)
    distances: list[float] = []
    headings: list[float] = []
    for point, tangent in zip(official_points, official_tangents):
        distance, heading, _ = current_index.query(
            point, tangent, max_distance_m=10.0, max_heading_deg=90.0,
        )
        distances.append(distance)
        headings.append(heading)
    finite = np.asarray(distances, dtype=np.float64)
    finite = finite[np.isfinite(finite)]
    if len(finite) != len(official_points):
        raise ValueError("official route contains points farther than 10m from current route chain")
    official_length = float(cumulative_distance(official_points)[-1])
    current_length = float(cumulative_distance(current_points)[-1])
    return {
        "official_sample_count": len(official_points),
        "current_sample_count": len(current_points),
        "official_length_m": official_length,
        "current_chain_length_m": current_length,
        "length_difference_m": current_length - official_length,
        "distance_p50_m": float(np.quantile(finite, 0.50)),
        "distance_p95_m": float(np.quantile(finite, 0.95)),
        "distance_max_m": float(np.max(finite)),
        "heading_p95_deg": float(np.quantile(np.asarray(headings), 0.95)),
    }


def route_segment(
    route_link_ids: list[str],
    links_by_id: dict[str, dict],
    start_link: str,
    end_link: str,
) -> tuple[np.ndarray, np.ndarray]:
    start_index = route_link_ids.index(start_link)
    end_index = route_link_ids.index(end_link)
    if end_index < start_index:
        selected = route_link_ids[start_index:] + route_link_ids[:end_index + 1]
    else:
        selected = route_link_ids[start_index:end_index + 1]
    points = route_centerline(selected, links_by_id)
    return resample_polyline(points, 1.0)


def select_parallel_links(
    links: list[dict],
    route_points: np.ndarray,
    route_tangents: np.ndarray,
    *,
    half_width_m: float,
    max_heading_deg: float,
    minimum_support: int,
    undirected_heading: bool = False,
) -> set[str]:
    route_index = GridIndex(route_points, route_tangents, cell_size_m=4.0)
    selected: set[str] = set()
    for link in links:
        points, tangents = resample_polyline(link.get("points", ()), 1.0)
        support = 0
        for point, tangent in zip(points, tangents):
            distance, heading, _ = route_index.query(
                point, tangent,
                max_distance_m=half_width_m,
                max_heading_deg=max_heading_deg,
                undirected_heading=undirected_heading,
            )
            if distance <= half_width_m and heading <= max_heading_deg:
                support += 1
        if support >= minimum_support:
            selected.add(str(link["idx"]))
    return selected


def clip_parallel_link_points(
    link: dict,
    route_points: np.ndarray,
    route_tangents: np.ndarray,
    *,
    half_width_m: float,
    max_heading_deg: float,
    sample_step_m: float,
    minimum_segment_length_m: float,
) -> tuple[np.ndarray, dict]:
    """Return the longest contiguous part of a link inside a route corridor.

    Parallel-link discovery is intentionally permissive: a current link is a
    useful topology candidate as soon as a short portion runs alongside the
    competition route.  Exporting that link's *entire* geometry, however, can
    leak unrelated roads into a route-only map.  This helper preserves the
    authoritative source link ID while limiting its exported centerline to the
    contiguous geometry that passes the same distance and heading gate.
    """

    points, tangents = resample_polyline(link.get("points", ()), sample_step_m)
    if len(points) < 2:
        raise ValueError(f"link {link.get('idx')} has fewer than two geometry points")
    route_index = GridIndex(route_points, route_tangents, cell_size_m=4.0)
    valid = np.zeros(len(points), dtype=bool)
    distances = np.full(len(points), np.inf, dtype=np.float64)
    headings = np.full(len(points), np.inf, dtype=np.float64)
    for index, (point, tangent) in enumerate(zip(points, tangents)):
        distance, heading, match = route_index.query(
            point,
            tangent,
            max_distance_m=half_width_m,
            max_heading_deg=max_heading_deg,
            undirected_heading=True,
        )
        if match is not None and distance <= half_width_m and heading <= max_heading_deg:
            valid[index] = True
            distances[index] = distance
            headings[index] = heading

    runs: list[tuple[int, int]] = []
    start: int | None = None
    for index, accepted in enumerate(valid):
        if accepted and start is None:
            start = index
        if start is not None and (not accepted or index == len(valid) - 1):
            end = index if accepted and index == len(valid) - 1 else index - 1
            if end > start:
                runs.append((start, end))
            start = None
    if not runs:
        raise ValueError(f"selected parallel link {link.get('idx')} has no contiguous valid run")

    station = cumulative_distance(points)
    longest = max(runs, key=lambda item: float(station[item[1]] - station[item[0]]))
    run_length = float(station[longest[1]] - station[longest[0]])
    if run_length < minimum_segment_length_m:
        raise ValueError(
            f"selected parallel link {link.get('idx')} corridor segment is only "
            f"{run_length:.3f}m"
        )
    selected = points[longest[0]:longest[1] + 1]
    selected_valid = valid[longest[0]:longest[1] + 1]
    return selected, {
        "original_length_m": polyline_length(link.get("points", ())),
        "corridor_length_m": run_length,
        "valid_run_count": len(runs),
        "discarded_valid_run_count": max(0, len(runs) - 1),
        "distance_p95_m": float(np.quantile(distances[valid], 0.95)),
        "heading_p95_deg": float(np.quantile(headings[valid], 0.95)),
        "sample_count": int(np.sum(selected_valid)),
    }


def continuous_topology_neighbors(
    link_id: str,
    links_by_id: dict[str, dict],
    incoming: dict[str, list[str]],
    outgoing: dict[str, list[str]],
    *,
    max_heading_deg: float,
) -> set[str]:
    link = links_by_id[link_id]
    start_heading = unit_heading(link.get("points", ()), at_end=False)
    end_heading = unit_heading(link.get("points", ()), at_end=True)
    neighbors: set[str] = set()
    for candidate_id in incoming.get(str(link.get("from_node_idx", "")), []):
        candidate = links_by_id[candidate_id]
        candidate_heading = unit_heading(candidate.get("points", ()), at_end=True)
        if heading_difference_deg(candidate_heading, start_heading) <= max_heading_deg:
            neighbors.add(candidate_id)
    for candidate_id in outgoing.get(str(link.get("to_node_idx", "")), []):
        candidate = links_by_id[candidate_id]
        candidate_heading = unit_heading(candidate.get("points", ()), at_end=False)
        if heading_difference_deg(end_heading, candidate_heading) <= max_heading_deg:
            neighbors.add(candidate_id)
    return neighbors


def expand_context_links(
    seed_ids: set[str],
    links_by_id: dict[str, dict],
    incoming: dict[str, list[str]],
    outgoing: dict[str, list[str]],
    *,
    hops: int,
    max_heading_deg: float,
) -> set[str]:
    selected = set(seed_ids)
    frontier = set(seed_ids)
    for _ in range(max(0, hops)):
        new_frontier: set[str] = set()
        for link_id in frontier:
            new_frontier.update(continuous_topology_neighbors(
                link_id, links_by_id, incoming, outgoing,
                max_heading_deg=max_heading_deg,
            ))
        new_frontier -= selected
        selected.update(new_frontier)
        frontier = new_frontier
    return selected


def corridor_relations(
    link_id: str,
    corridor_ids: set[str],
    links_by_id: dict[str, dict],
    incoming: dict[str, list[str]],
    outgoing: dict[str, list[str]],
    explicit_opposites: dict[str, str],
) -> dict:
    link = links_by_id[link_id]
    all_predecessors = incoming.get(str(link.get("from_node_idx", "")), [])
    all_successors = outgoing.get(str(link.get("to_node_idx", "")), [])
    left = link.get("left_lane_change_dst_link_idx")
    right = link.get("right_lane_change_dst_link_idx")
    return {
        "predecessor_link_ids": sorted(set(all_predecessors).intersection(corridor_ids)),
        "successor_link_ids": sorted(set(all_successors).intersection(corridor_ids)),
        "left_adjacent_link_id": left if left in corridor_ids else None,
        "right_adjacent_link_id": right if right in corridor_ids else None,
        "external_left_adjacent_link_id": left if left and left not in corridor_ids else None,
        "external_right_adjacent_link_id": right if right and right not in corridor_ids else None,
        "opposite_link_id": explicit_opposites.get(link_id),
        "is_merge_target": len(all_predecessors) > 1,
        "is_split_source": len(all_successors) > 1,
    }
