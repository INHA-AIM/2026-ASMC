"""Mission-zone scenario spec and sampling helpers for scenario generators.

This module has no ROS imports. It builds the spec on demand from the committed
competition metadata, the compact drivable product and the MGeo link_set, plus a
team-editable source that maps each zone to the extra lanes a generator may use.
Nothing here is pinned: edit the source or the metadata and the next build
reflects it. Reference problems are returned as warnings so that an in-progress
edit never blocks a run or the debug visualization.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
import random
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import numpy as np
import yaml

from .competition_metadata import project_point_to_polyline
from .runtime_bundle import point_at_station, polyline_stations

SOURCE_SCHEMA_VERSION = "asmc.hd_map.mission_scenario_source/v1"
SPEC_SCHEMA_VERSION = "asmc.hd_map.mission_scenario_spec/v1"
DEFAULT_SOURCE_REL = Path("src/asmc_hd_map/config/kcity2025_mission_scenario_source.yaml")
ACTOR_ROLES = ("static_obstacle", "dynamic_obstacle", "npc_vehicle")
# Zone types whose spatial surface matters to a scenario generator. Speed,
# start and perception-policy zones keep only their route span.
SURFACE_MISSION_TYPES = {
    "gps_shadow",
    "static_obstacle",
    "dynamic_obstacle",
    "roundabout",
    "surrounding_vehicle",
    "highway_merge",
}


class MissionScenarioError(ValueError):
    """Raised when the spec inputs cannot be read at all."""


def _load_yaml(path: Path) -> Mapping[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        value = yaml.safe_load(stream)
    if not isinstance(value, dict):
        raise MissionScenarioError("%s must contain a mapping" % path)
    return value


def _load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def _xyz(points: Iterable[Sequence[Any]]) -> List[List[float]]:
    result = []
    for point in points:
        z = float(point[2]) if len(point) > 2 else 0.0
        result.append([round(float(point[0]), 4), round(float(point[1]), 4), round(z, 4)])
    return result


def _ring_xy(raw: Iterable[Any]) -> List[Tuple[float, float]]:
    return [
        (float(point["x"]), float(point["y"])) if isinstance(point, dict) else (float(point[0]), float(point[1]))
        for point in raw
    ]


def _tangent_heading_deg(points: Sequence[Sequence[float]], stations: Sequence[float], station: float) -> float:
    length = float(stations[-1])
    span = min(1.0, max(0.05, 0.25 * length))
    before = point_at_station(points, stations, max(0.0, station - span))
    after = point_at_station(points, stations, min(length, station + span))
    return math.degrees(math.atan2(after[1] - before[1], after[0] - before[0]))


def _route_polyline(
    route_link_ids: Sequence[str], links: Mapping[str, Mapping[str, Any]]
) -> List[List[float]]:
    points: List[List[float]] = []
    for link_id in route_link_ids:
        link_points = _xyz(links[link_id]["points"])
        if points and math.hypot(points[-1][0] - link_points[0][0], points[-1][1] - link_points[0][1]) < 1e-6:
            link_points = link_points[1:]
        points.extend(link_points)
    return points


def _direction_vs_route(
    centerline: Sequence[Sequence[float]], route_points: Sequence[Sequence[float]]
) -> str:
    stations = polyline_stations(centerline)
    middle = 0.5 * stations[-1]
    position = point_at_station(centerline, stations, middle)
    heading = math.radians(_tangent_heading_deg(centerline, stations, middle))
    projection = project_point_to_polyline(position, route_points)
    index = int(projection["segment_index"])
    start, end = route_points[index], route_points[index + 1]
    route_heading = math.atan2(end[1] - start[1], end[0] - start[0])
    dot = math.cos(heading - route_heading)
    if dot >= 0.5:
        return "same"
    if dot <= -0.5:
        return "opposite"
    return "crossing"


def _distance_to_span(
    centerline: Sequence[Sequence[float]], span_points: Sequence[Sequence[float]]
) -> float:
    if len(span_points) < 2:
        return 0.0
    return min(
        float(project_point_to_polyline(point, span_points)["error_xy_m"])
        for point in (centerline[0], centerline[len(centerline) // 2], centerline[-1])
    )


def merged_outline(
    rings: Iterable[Iterable[Any]],
    close_m: float = 0.2,
    min_hole_m2: float = 5.0,
    simplify_m: float = 0.1,
) -> Dict[str, Any]:
    """Union drivable polygons into clean display outlines.

    Adjacent lane polygons meet with hairline gaps, so a plain union leaves
    dozens of sliver rings that render as broken line fragments. A small
    closing (grow then shrink by close_m) seals those gaps, holes smaller than
    min_hole_m2 are dropped and the result is simplified. Real holes such as a
    roundabout island survive.
    """

    from shapely.geometry import Polygon
    from shapely.ops import unary_union

    polygons = []
    for raw in rings:
        polygon = Polygon(_ring_xy(raw))
        if not polygon.is_valid:
            polygon = polygon.buffer(0)
        if not polygon.is_empty:
            polygons.append(polygon)
    if not polygons:
        return {"area_m2": 0.0, "piece_count": 0, "outlines": []}
    union = unary_union(polygons)
    if close_m > 0.0:
        union = union.buffer(close_m, join_style=2).buffer(-close_m, join_style=2)
    pieces = []
    for piece in getattr(union, "geoms", [union]):
        holes = [ring for ring in piece.interiors if Polygon(ring).area >= min_hole_m2]
        pieces.append(Polygon(piece.exterior, holes).simplify(simplify_m))
    outlines = []
    for piece in pieces:
        for ring in [piece.exterior] + list(piece.interiors):
            outlines.append([[round(x, 3), round(y, 3)] for x, y in ring.coords])
    return {
        "area_m2": round(float(sum(piece.area for piece in pieces)), 3),
        "piece_count": len(pieces),
        "outlines": outlines,
    }


def _surface(
    areas: Sequence[Mapping[str, Any]], link_ids: Iterable[str]
) -> Optional[Dict[str, Any]]:
    allowed = set(link_ids)
    selected = [
        area for area in areas
        if area.get("link_ids") and set(area["link_ids"]).issubset(allowed)
    ]
    if not selected:
        return None
    merged = merged_outline(area["polygon"] for area in selected)
    merged["area_ids"] = sorted(str(area["id"]) for area in selected)
    return merged


def _adjacent_lane_chain(
    route_lanes: Sequence[Mapping[str, Any]],
    lane_by_id: Mapping[str, Mapping[str, Any]],
    side: str,
) -> List[str]:
    keys = {"left": ("left_adjacent_lane_id",), "right": ("right_adjacent_lane_id",)}.get(
        side, ("left_adjacent_lane_id", "right_adjacent_lane_id")
    )
    route_ids = {str(lane["id"]) for lane in route_lanes}
    found: List[str] = []
    for lane in route_lanes:
        for key in keys:
            current = lane
            while True:
                neighbor = str(current.get(key) or "")
                if not neighbor or neighbor not in lane_by_id or neighbor in route_ids or neighbor in found:
                    break
                found.append(neighbor)
                current = lane_by_id[neighbor]
    return [str(lane_by_id[lane_id]["link_id"]) for lane_id in found]


def _display_path(path: Path, repo_root: Path) -> str:
    try:
        return str(Path(path).resolve().relative_to(repo_root))
    except ValueError:
        return str(Path(path).resolve())


def build_mission_scenario_spec(
    repo_root: Path, source_path: Optional[Path] = None
) -> Dict[str, Any]:
    """Build the per-zone scenario spec from the current committed inputs."""

    repo_root = Path(repo_root).resolve()
    source_path = Path(source_path) if source_path else repo_root / DEFAULT_SOURCE_REL
    source = _load_yaml(source_path)
    if source.get("schema_version") != SOURCE_SCHEMA_VERSION:
        raise MissionScenarioError("unsupported mission scenario source schema")
    inputs = source.get("inputs") or {}
    metadata = _load_yaml(repo_root / str(inputs["competition_metadata"]))
    compact = _load_json(repo_root / str(inputs["compact_drivable"]))
    raw_links = _load_json(repo_root / str(inputs["mgeo_link_set"]))
    links = {str(item["idx"]): item for item in raw_links}
    lanes = list(compact.get("lanes") or [])
    lane_by_id = {str(lane["id"]): lane for lane in lanes}
    lane_by_link = {str(lane["link_id"]): lane for lane in lanes}
    areas = list(compact.get("drivable_areas") or [])
    route = metadata["route"]
    route_link_ids = [str(value) for value in route["link_ids"]]
    route_points = _route_polyline(route_link_ids, links)
    route_stations = polyline_stations(route_points)
    speed_by_link = {str(item["link_id"]): item for item in metadata.get("route_speed_limits") or []}
    max_distance = float(source.get("actor_link_max_distance_m", 100.0))
    zone_sources = source.get("zones") or {}
    warnings: List[str] = []
    zone_ids = {str(item["id"]) for item in metadata.get("mission_zones") or []}
    for unknown in sorted(set(zone_sources) - zone_ids):
        warnings.append("source zone %s is not a metadata mission zone" % unknown)

    zones: List[Dict[str, Any]] = []
    for mission in metadata.get("mission_zones") or []:
        zone_id = str(mission["id"])
        mission_type = str(mission["mission_type"])
        payload = json.loads(str(mission.get("rule_payload_json") or "{}"))
        zone_source = zone_sources.get(zone_id) or {}
        start = float(mission["route_station_start_m"])
        end = float(mission["route_station_end_m"])
        zone_links = [str(value) for value in mission.get("link_ids") or []]
        route_lanes = [lane_by_link[link_id] for link_id in zone_links if link_id in lane_by_link]
        span_points = [
            point_at_station(route_points, route_stations, station)
            for station in np.linspace(start, max(end, start + 1e-3), max(2, int((end - start) / 2.0) + 2))
        ]

        maneuver_lanes = []
        for group in zone_source.get("maneuver_groups") or []:
            members = [lane for lane in lanes if lane.get("drivable_group") == group]
            if not members:
                warnings.append("%s: maneuver group %s has no compact lanes" % (zone_id, group))
            for lane in members:
                centerline = _xyz(links[lane["link_id"]]["points"]) if lane["link_id"] in links else _xyz(lane["centerline"])
                maneuver_lanes.append({
                    "lane_id": str(lane["id"]),
                    "link_id": str(lane["link_id"]),
                    "group": str(group),
                    "on_route": str(lane["link_id"]) in route_link_ids,
                    "direction_vs_route": _direction_vs_route(centerline, route_points),
                    "length_m": round(polyline_stations(centerline)[-1], 3),
                })

        actor_sources = []
        for entry in zone_source.get("actor_sources") or []:
            role = str(entry.get("role", ""))
            if role not in ACTOR_ROLES:
                warnings.append("%s: unknown actor role %r" % (zone_id, role))
                continue
            link_ids = [str(value) for value in entry.get("link_ids") or []]
            if entry.get("adjacent_to_route"):
                link_ids += _adjacent_lane_chain(route_lanes, lane_by_id, str(entry["adjacent_to_route"]))
            if not link_ids:
                warnings.append("%s: %s actor source resolved to no links" % (zone_id, role))
            for link_id in link_ids:
                link = links.get(link_id)
                if link is None:
                    warnings.append("%s: actor link %s is not in the MGeo link_set" % (zone_id, link_id))
                    continue
                centerline = _xyz(link["points"])
                distance = _distance_to_span(centerline, span_points)
                if distance > max_distance:
                    warnings.append(
                        "%s: actor link %s is %.1f m from the zone span" % (zone_id, link_id, distance)
                    )
                lane = lane_by_link.get(link_id)
                actor_sources.append({
                    "role": role,
                    "link_id": link_id,
                    "lane_id": str(lane["id"]) if lane else "",
                    "in_compact_product": lane is not None,
                    "direction_vs_route": _direction_vs_route(centerline, route_points),
                    "length_m": round(polyline_stations(centerline)[-1], 3),
                    "mgeo_max_speed_kph": float(link.get("max_speed") or 0.0),
                    "from_node_id": str(link.get("from_node_idx", "")),
                    "to_node_id": str(link.get("to_node_idx", "")),
                    "note": str(entry.get("note", "")),
                    "centerline": centerline,
                })

        surface = None
        if mission_type in SURFACE_MISSION_TYPES and end > start:
            # The zone surface is where the ego may drive inside the zone: its
            # route lanes, the maneuver lanes, and optionally the same-direction
            # lanes beside the route (e.g. every lane of the high-speed road).
            surface_links = set(zone_links) | {lane["link_id"] for lane in maneuver_lanes}
            if zone_source.get("surface_adjacent_to_route"):
                surface_links |= set(_adjacent_lane_chain(
                    route_lanes, lane_by_id, str(zone_source["surface_adjacent_to_route"])
                ))
            for link_id in zone_source.get("surface_link_ids") or []:
                if str(link_id) not in lane_by_link:
                    warnings.append("%s: surface link %s is not a compact lane" % (zone_id, link_id))
                surface_links.add(str(link_id))
            surface = _surface(areas, surface_links)
        zones.append({
            "id": zone_id,
            "mission_type": mission_type,
            "status": str(mission["status"]),
            "authority": str(payload.get("authority", mission["status"])),
            "description": str(mission.get("description", "")),
            "route_station_start_m": start,
            "route_station_end_m": end,
            "route_link_ids": zone_links,
            "route_lane_ids": [str(lane["id"]) for lane in route_lanes],
            "checkpoint_ids": [str(value) for value in mission.get("checkpoint_ids") or []],
            "speed_limits": [
                {
                    "link_id": link_id,
                    "effective_limit_kph": speed_by_link[link_id]["effective_limit_kph"],
                    "competition_cap_kph": speed_by_link[link_id]["competition_cap_kph"],
                }
                for link_id in zone_links if link_id in speed_by_link
            ],
            "rule": payload,
            "maneuver_lanes": maneuver_lanes,
            "actor_sources": actor_sources,
            "surface": surface,
        })
    return {
        "schema_version": SPEC_SCHEMA_VERSION,
        "frame_id": str(metadata.get("frame_id", "map")),
        "yaw_convention": "degrees, atan2(dy, dx) in the map frame (MORAI checkpoint convention)",
        "source": _display_path(source_path, repo_root),
        "inputs": {name: str(value) for name, value in inputs.items()},
        "route_length_m": round(route_stations[-1], 6),
        "zones": zones,
        "warnings": warnings,
    }


class MissionScenarioSpec:
    """Query and sampling API over a built mission scenario spec."""

    def __init__(self, document: Mapping[str, Any], repo_root: Path) -> None:
        self.document = document
        self.repo_root = Path(repo_root).resolve()
        self._zones = {str(zone["id"]): zone for zone in document["zones"]}
        source = _load_yaml(self.repo_root / str(document["source"]))  # absolute paths survive "/"
        inputs = source.get("inputs") or {}
        metadata = _load_yaml(self.repo_root / str(inputs["competition_metadata"]))
        links = {str(item["idx"]): item for item in _load_json(self.repo_root / str(inputs["mgeo_link_set"]))}
        self.route_points = _route_polyline([str(value) for value in metadata["route"]["link_ids"]], links)
        self.route_stations = polyline_stations(self.route_points)
        compact = _load_json(self.repo_root / str(inputs["compact_drivable"]))
        grid = compact["grid"]
        self._grid = np.load(self.repo_root / str(inputs["compact_grid"]))["occupancy"]
        self._grid_origin = (float(grid["origin_x"]), float(grid["origin_y"]))
        self._grid_resolution = float(grid["resolution_m"])

    @classmethod
    def load(cls, repo_root: Path, source_path: Optional[Path] = None) -> "MissionScenarioSpec":
        return cls(build_mission_scenario_spec(repo_root, source_path), repo_root)

    @property
    def warnings(self) -> List[str]:
        return list(self.document.get("warnings") or [])

    def zones(self) -> List[Mapping[str, Any]]:
        return list(self.document["zones"])

    def zone(self, zone_id: str) -> Mapping[str, Any]:
        return self._zones[zone_id]

    def actor_roles(self, zone_id: str) -> List[str]:
        return sorted({str(item["role"]) for item in self.zone(zone_id)["actor_sources"]})

    def is_drivable(self, x_m: float, y_m: float) -> bool:
        column = int(round((float(x_m) - self._grid_origin[0]) / self._grid_resolution))
        row = int(round((float(y_m) - self._grid_origin[1]) / self._grid_resolution))
        if not (0 <= row < self._grid.shape[0] and 0 <= column < self._grid.shape[1]):
            return False
        return int(self._grid[row, column]) == 0

    def route_station(self, x_m: float, y_m: float) -> Dict[str, float]:
        projection = project_point_to_polyline((x_m, y_m, 0.0), self.route_points)
        return {"station_m": float(projection["station_m"]), "lateral_error_m": float(projection["error_xy_m"])}

    def route_pose(self, station_m: float, lateral_offset_m: float = 0.0) -> Dict[str, float]:
        return self._pose_on(self.route_points, self.route_stations, station_m, lateral_offset_m)

    @staticmethod
    def _pose_on(
        points: Sequence[Sequence[float]],
        stations: Sequence[float],
        station_m: float,
        lateral_offset_m: float,
    ) -> Dict[str, float]:
        station = max(0.0, min(float(station_m), float(stations[-1])))
        center = point_at_station(points, stations, station)
        yaw_deg = _tangent_heading_deg(points, stations, station)
        yaw = math.radians(yaw_deg)
        return {
            "x": center[0] - math.sin(yaw) * lateral_offset_m,
            "y": center[1] + math.cos(yaw) * lateral_offset_m,
            "z": center[2],
            "yaw_deg": yaw_deg,
        }

    def sample_actor_pose(
        self,
        zone_id: str,
        role: str,
        rng: Optional[random.Random] = None,
        longitudinal_margin_m: float = 2.0,
        lateral_jitter_m: float = 0.0,
    ) -> Dict[str, Any]:
        """Draw one spawn pose on a zone's actor link for MORAI or a planner check."""

        rng = rng or random.Random()
        sources = [item for item in self.zone(zone_id)["actor_sources"] if item["role"] == role]
        if not sources:
            raise KeyError("%s has no %s actor source" % (zone_id, role))
        weights = [max(float(item["length_m"]), 1e-3) for item in sources]
        source = rng.choices(sources, weights=weights, k=1)[0]
        centerline = source["centerline"]
        stations = polyline_stations(centerline)
        length = float(stations[-1])
        margin = min(float(longitudinal_margin_m), 0.5 * length)
        link_station = rng.uniform(margin, length - margin)
        lateral = rng.uniform(-lateral_jitter_m, lateral_jitter_m) if lateral_jitter_m > 0.0 else 0.0
        pose = self._pose_on(centerline, stations, link_station, lateral)
        route = self.route_station(pose["x"], pose["y"])
        return {
            "zone_id": zone_id,
            "role": role,
            "link_id": source["link_id"],
            "lane_id": source["lane_id"],
            "link_station_m": round(link_station, 3),
            "lateral_offset_m": round(lateral, 3),
            "x": round(pose["x"], 4),
            "y": round(pose["y"], 4),
            "z": round(pose["z"], 4),
            "yaw_deg": round(pose["yaw_deg"], 4),
            "in_compact_drivable": self.is_drivable(pose["x"], pose["y"]),
            "route_station_m": round(route["station_m"], 3),
            "route_lateral_error_m": round(route["lateral_error_m"], 3),
        }
