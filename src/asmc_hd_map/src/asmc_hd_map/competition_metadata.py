"""Deterministic P1-C checkpoint and mission metadata builder.

The module imports organizer checkpoint/path files read-only, projects them to
the reviewed route/link decomposition, and keeps official, derived-official,
and provisional authority levels distinct. It never edits HD-map geometry.
"""

from __future__ import annotations

from collections import Counter
import hashlib
import json
import math
import os
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import yaml


SOURCE_SCHEMA_VERSION = "asmc.competition_metadata.source/v1"
METADATA_SCHEMA_VERSION = "asmc.competition_metadata/v1"
# Rule book v1.2 (2026-09-30): START/END plus checkpoints 1-14.
OFFICIAL_CHECKPOINT_COUNT = 15
# team_final: the organizer gives no numeric boundary, but the team signed the
# boundary off for the competition, so planning may enforce it. The frozen
# MissionZone message carries it as STATUS_DERIVED_OFFICIAL with
# rule_payload.authority=team_final. Only provisional blocks planning.
ALLOWED_MISSION_STATUS = {"official", "derived_official", "team_final", "provisional"}
ALLOWED_MISSION_TYPES = {
    "start_success",
    "speed_limit",
    "high_speed_exception",
    "gps_shadow",
    "static_obstacle",
    "dynamic_obstacle",
    "roundabout",
    "highway_merge",
    "surrounding_vehicle",
    "traffic_light",
}


class CompetitionMetadataError(ValueError):
    """Raised when an official input or metadata invariant is invalid."""


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def default_data_root(repo_root: Path) -> Path:
    configured = os.environ.get("ASMC_DATA")
    if configured:
        return Path(configured).expanduser().resolve()
    sibling = (repo_root.parent / "data").resolve()
    if sibling.is_dir():
        return sibling
    return Path("/data")


def _resolve_input(repo_root: Path, data_root: Path, raw: str) -> Path:
    prefix = "$ASMC_DATA/"
    if raw.startswith(prefix):
        return (data_root / raw[len(prefix) :]).resolve()
    expanded = Path(os.path.expanduser(os.path.expandvars(raw)))
    if "$" in str(expanded):
        raise CompetitionMetadataError("unresolved variable in input path: %s" % raw)
    return expanded.resolve() if expanded.is_absolute() else (repo_root / expanded).resolve()


def _require_file(path: Path, label: str) -> Path:
    if not path.is_file():
        raise CompetitionMetadataError("missing %s: %s" % (label, path))
    return path


def _load_yaml(path: Path) -> Mapping[str, Any]:
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise CompetitionMetadataError("YAML root must be a mapping: %s" % path)
    return value


def _load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def _finite(value: Any, label: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        raise CompetitionMetadataError("%s must be numeric" % label)
    if not math.isfinite(result):
        raise CompetitionMetadataError("%s must be finite" % label)
    return result


def _point_xyz(raw: Sequence[Any], label: str) -> Tuple[float, float, float]:
    if len(raw) < 2:
        raise CompetitionMetadataError("%s must contain at least x and y" % label)
    z = raw[2] if len(raw) > 2 else 0.0
    return (_finite(raw[0], label + ".x"), _finite(raw[1], label + ".y"), _finite(z, label + ".z"))


def _polyline(raw: Iterable[Sequence[Any]], label: str) -> List[Tuple[float, float, float]]:
    points = [_point_xyz(row, "%s[%d]" % (label, index)) for index, row in enumerate(raw)]
    if len(points) < 2:
        raise CompetitionMetadataError("%s must have at least two points" % label)
    return points


def polyline_length_xy(points: Sequence[Sequence[float]]) -> float:
    return sum(
        math.hypot(right[0] - left[0], right[1] - left[1])
        for left, right in zip(points, points[1:])
    )


def project_point_to_polyline(
    point: Sequence[float], points: Sequence[Sequence[float]]
) -> Dict[str, Any]:
    """Project a point onto a polyline using XY distance and return station."""

    if len(points) < 2:
        raise CompetitionMetadataError("cannot project onto a polyline with fewer than two points")
    query = _point_xyz(point, "projection.point")
    cumulative = 0.0
    best: Optional[Dict[str, Any]] = None
    for segment_index, (start, end) in enumerate(zip(points, points[1:])):
        dx = float(end[0] - start[0])
        dy = float(end[1] - start[1])
        length_sq = dx * dx + dy * dy
        if length_sq <= 1e-20:
            continue
        fraction = ((query[0] - start[0]) * dx + (query[1] - start[1]) * dy) / length_sq
        fraction = max(0.0, min(1.0, fraction))
        projected = (
            start[0] + fraction * (end[0] - start[0]),
            start[1] + fraction * (end[1] - start[1]),
            start[2] + fraction * (end[2] - start[2]),
        )
        error = math.hypot(query[0] - projected[0], query[1] - projected[1])
        segment_length = math.sqrt(length_sq)
        candidate = {
            "station_m": cumulative + fraction * segment_length,
            "error_xy_m": error,
            "projected_xyz": list(projected),
            "segment_index": segment_index,
            "segment_fraction": fraction,
        }
        # Strict comparison preserves the first occurrence at the duplicated
        # START/END point of the closed route.
        if best is None or error < best["error_xy_m"] - 1e-12:
            best = candidate
        cumulative += segment_length
    if best is None:
        raise CompetitionMetadataError("polyline contains no non-zero XY segment")
    return best


def _stable_float(value: float, digits: int = 9) -> float:
    return float(round(float(value), digits))


def _source_record(raw: str, path: Path, authority: str) -> Dict[str, Any]:
    return {
        "path": raw,
        "sha256": sha256_file(path),
        "bytes": path.stat().st_size,
        "authority": authority,
    }


def _load_global_path(path: Path) -> List[Tuple[float, float, float]]:
    points: List[Tuple[float, float, float]] = []
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        values = line.split()
        if not values:
            continue
        if len(values) != 3:
            raise CompetitionMetadataError("invalid global path line %d" % line_number)
        points.append(_point_xyz(values, "global_path[%d]" % line_number))
    if len(points) < 2:
        raise CompetitionMetadataError("global path has fewer than two points")
    return points


def _route_ranges(
    route_link_ids: Sequence[str],
    link_by_id: Mapping[str, Mapping[str, Any]],
    continuity_tolerance_m: float,
) -> Tuple[List[Dict[str, Any]], Dict[str, List[Tuple[float, float, float]]]]:
    ranges: List[Dict[str, Any]] = []
    geometry: Dict[str, List[Tuple[float, float, float]]] = {}
    station = 0.0
    previous_end: Optional[Tuple[float, float, float]] = None
    for order, link_id in enumerate(route_link_ids):
        if link_id not in link_by_id:
            raise CompetitionMetadataError("route link is missing from link_set: %s" % link_id)
        points = _polyline(link_by_id[link_id].get("points", []), "link.%s.points" % link_id)
        if previous_end is not None:
            gap = math.hypot(points[0][0] - previous_end[0], points[0][1] - previous_end[1])
            if gap > continuity_tolerance_m:
                raise CompetitionMetadataError(
                    "route link discontinuity %s -> %s: %.9f m"
                    % (route_link_ids[order - 1], link_id, gap)
                )
        length = polyline_length_xy(points)
        ranges.append(
            {
                "link_id": link_id,
                "order": order,
                "start_station_m": station,
                "end_station_m": station + length,
                "length_m": length,
            }
        )
        geometry[link_id] = points
        station += length
        previous_end = points[-1]
    return ranges, geometry


def _range_by_id(ranges: Sequence[Mapping[str, Any]]) -> Dict[str, Mapping[str, Any]]:
    return {str(item["link_id"]): item for item in ranges}


def _route_link_at_station(ranges: Sequence[Mapping[str, Any]], station_m: float) -> str:
    for item in ranges:
        if station_m <= float(item["end_station_m"]) + 1e-6:
            return str(item["link_id"])
    return str(ranges[-1]["link_id"])


def _route_slice(
    route_link_ids: Sequence[str], start_link_id: str, end_link_id: str
) -> List[str]:
    try:
        start = route_link_ids.index(start_link_id)
        end = route_link_ids.index(end_link_id)
    except ValueError as exc:
        raise CompetitionMetadataError("mission references a non-route link: %s" % exc)
    if end < start:
        raise CompetitionMetadataError("mission link span is reversed: %s -> %s" % (start_link_id, end_link_id))
    return list(route_link_ids[start : end + 1])


def _yaw_quaternion(yaw_deg: float) -> Dict[str, float]:
    half = math.radians(yaw_deg) * 0.5
    return {
        "x": 0.0,
        "y": 0.0,
        "z": _stable_float(math.sin(half), 12),
        "w": _stable_float(math.cos(half), 12),
    }


def _build_checkpoints(
    official: Mapping[str, Any],
    global_path: Sequence[Sequence[float]],
    route_ranges: Sequence[Mapping[str, Any]],
    link_geometry: Mapping[str, Sequence[Sequence[float]]],
    assignment: Mapping[str, str],
    validation: Mapping[str, Any],
) -> List[Dict[str, Any]]:
    expected_count = int(validation["checkpoint_count"])
    pass_radius = _finite(official.get("pass_radius_m"), "official.pass_radius_m")
    if abs(pass_radius - float(validation["pass_radius_m"])) > 1e-12:
        raise CompetitionMetadataError("official checkpoint pass radius is not the required 3 m")
    if official.get("yaw_unit") != "deg":
        raise CompetitionMetadataError("official checkpoint yaw_unit must be deg")
    raw_items = official.get("checkpoints")
    if not isinstance(raw_items, list) or len(raw_items) != expected_count:
        raise CompetitionMetadataError("expected %d official checkpoints" % expected_count)

    expected_ids = ["START_END"] + [str(value) for value in range(1, expected_count)]
    actual_ids = [str(item.get("id")) for item in raw_items]
    if actual_ids != expected_ids:
        raise CompetitionMetadataError("checkpoint IDs/order changed: %s" % actual_ids)
    if set(assignment) != set(expected_ids):
        raise CompetitionMetadataError("checkpoint_link_assignment must cover every checkpoint exactly")

    ranges = _range_by_id(route_ranges)
    route_max = float(validation["checkpoint_route_projection_max_error_m"])
    link_max = float(validation["checkpoint_link_projection_max_error_m"])
    checkpoints: List[Dict[str, Any]] = []
    previous_station = -math.inf
    for sequence, item in enumerate(raw_items):
        checkpoint_id = str(item["id"])
        position = (
            _finite(item.get("x"), checkpoint_id + ".x"),
            _finite(item.get("y"), checkpoint_id + ".y"),
            _finite(item.get("z"), checkpoint_id + ".z"),
        )
        yaw_deg = _finite(item.get("yaw"), checkpoint_id + ".yaw")
        route_projection = project_point_to_polyline(position, global_path)
        link_id = str(assignment[checkpoint_id])
        if link_id not in link_geometry or link_id not in ranges:
            raise CompetitionMetadataError("checkpoint %s uses non-route link %s" % (checkpoint_id, link_id))
        link_projection = project_point_to_polyline(position, link_geometry[link_id])
        if route_projection["error_xy_m"] > route_max:
            raise CompetitionMetadataError("checkpoint %s is outside the route projection tolerance" % checkpoint_id)
        if link_projection["error_xy_m"] > link_max:
            raise CompetitionMetadataError("checkpoint %s is outside link %s" % (checkpoint_id, link_id))
        expected_route_station = float(ranges[link_id]["start_station_m"]) + float(link_projection["station_m"])
        if abs(expected_route_station - float(route_projection["station_m"])) > 1e-5:
            raise CompetitionMetadataError("checkpoint %s route/link station disagreement" % checkpoint_id)
        station = float(route_projection["station_m"])
        if station + 1e-9 < previous_station:
            raise CompetitionMetadataError("checkpoint route stations are not monotonic")
        previous_station = station
        checkpoints.append(
            {
                "id": checkpoint_id,
                "label": str(item.get("label", checkpoint_id)),
                "sequence": sequence,
                "is_start": checkpoint_id == "START_END",
                "is_finish": checkpoint_id == "START_END",
                "pose": {
                    "position": {"x": position[0], "y": position[1], "z": position[2]},
                    "orientation": _yaw_quaternion(yaw_deg),
                },
                "official_yaw_deg": yaw_deg,
                "pass_radius_m": pass_radius,
                "route_station_m": _stable_float(station),
                "link_id": link_id,
                "link_station_m": _stable_float(link_projection["station_m"]),
                "source_reference": "official_checkpoints",
                "projection": {
                    "route_error_xy_m": _stable_float(route_projection["error_xy_m"], 12),
                    "link_error_xy_m": _stable_float(link_projection["error_xy_m"], 12),
                },
            }
        )
    return checkpoints


def _mission_bounds(
    spec: Mapping[str, Any],
    route_length: float,
    route_link_ids: Sequence[str],
    route_ranges: Sequence[Mapping[str, Any]],
) -> Tuple[float, float, str, str, List[str]]:
    bounds = spec.get("bounds")
    if not isinstance(bounds, dict):
        raise CompetitionMetadataError("mission %s has no bounds mapping" % spec.get("id"))
    kind = bounds.get("kind")
    ranges = _range_by_id(route_ranges)
    if kind == "all_route":
        return 0.0, route_length, route_link_ids[0], route_link_ids[-1], list(route_link_ids)
    if kind == "route_fraction":
        start = _finite(bounds.get("start_fraction"), "start_fraction") * route_length
        end = _finite(bounds.get("end_fraction"), "end_fraction") * route_length
        return (
            start,
            end,
            _route_link_at_station(route_ranges, start),
            _route_link_at_station(route_ranges, end),
            _route_slice(
                route_link_ids,
                _route_link_at_station(route_ranges, start),
                _route_link_at_station(route_ranges, end),
            ),
        )
    if kind == "link_span":
        start_link = str(bounds.get("start_link_id"))
        end_link = str(bounds.get("end_link_id"))
        links = _route_slice(route_link_ids, start_link, end_link)
        return (
            float(ranges[start_link]["start_station_m"]),
            float(ranges[end_link]["end_station_m"]),
            start_link,
            end_link,
            links,
        )
    if kind == "after_link_to_link_end":
        after_link = str(bounds.get("after_link_id"))
        end_link = str(bounds.get("end_link_id"))
        if after_link not in ranges:
            raise CompetitionMetadataError("mission after_link is not on route: %s" % after_link)
        after_index = route_link_ids.index(after_link)
        if after_index + 1 >= len(route_link_ids):
            raise CompetitionMetadataError("mission cannot start after final route link")
        start_link = route_link_ids[after_index + 1]
        links = _route_slice(route_link_ids, start_link, end_link)
        return (
            float(ranges[after_link]["end_station_m"]),
            float(ranges[end_link]["end_station_m"]),
            start_link,
            end_link,
            links,
        )
    raise CompetitionMetadataError("unsupported mission bounds kind: %s" % kind)


def _team_final_payload(
    label: str, payload: Mapping[str, Any], review: Any
) -> Dict[str, Any]:
    if not isinstance(review, dict) or not review.get("reviewed_on") or not review.get("decision"):
        raise CompetitionMetadataError("team_final %s needs review.reviewed_on and review.decision" % label)
    if payload.get("requires_official_boundary_confirmation"):
        raise CompetitionMetadataError("team_final %s still requires boundary confirmation" % label)
    result = dict(payload)
    result["authority"] = "team_final"
    result["review"] = {"reviewed_on": str(review["reviewed_on"]), "decision": str(review["decision"])}
    return result


def _checkpoint_ids_in_span(
    checkpoints: Sequence[Mapping[str, Any]], start: float, end: float
) -> List[str]:
    return [
        str(item["id"])
        for item in checkpoints
        if start - 1e-6 <= float(item["route_station_m"]) <= end + 1e-6
    ]


def _build_authored_missions(
    mission_specs: Sequence[Mapping[str, Any]],
    route_length: float,
    route_link_ids: Sequence[str],
    route_ranges: Sequence[Mapping[str, Any]],
    checkpoints: Sequence[Mapping[str, Any]],
) -> List[Dict[str, Any]]:
    result: List[Dict[str, Any]] = []
    for spec in mission_specs:
        mission_id = str(spec.get("id", ""))
        mission_type = str(spec.get("mission_type", ""))
        status = str(spec.get("status", ""))
        if not mission_id:
            raise CompetitionMetadataError("mission id is required")
        if mission_type not in ALLOWED_MISSION_TYPES:
            raise CompetitionMetadataError("unknown mission type: %s" % mission_type)
        if status not in ALLOWED_MISSION_STATUS:
            raise CompetitionMetadataError("mission %s has invalid status" % mission_id)
        start, end, start_link, end_link, links = _mission_bounds(
            spec, route_length, route_link_ids, route_ranges
        )
        payload = spec.get("rule_payload") or {}
        if not isinstance(payload, dict):
            raise CompetitionMetadataError("mission rule_payload must be a mapping: %s" % mission_id)
        if status == "team_final":
            payload = _team_final_payload(mission_id, payload, spec.get("review"))
        has_speed_limit = "speed_limit_kph" in spec
        speed_limit = _finite(spec.get("speed_limit_kph", 0.0), mission_id + ".speed_limit_kph")
        result.append(
            {
                "id": mission_id,
                "mission_type": mission_type,
                "status": status,
                "source_reference": str(spec.get("source_reference", "")),
                "description": str(spec.get("description", "")),
                "route_station_start_m": _stable_float(start),
                "route_station_end_m": _stable_float(end),
                "start_link_id": start_link,
                "end_link_id": end_link,
                "link_ids": links,
                "checkpoint_ids": _checkpoint_ids_in_span(checkpoints, start, end),
                "polygon": [],
                "has_speed_limit": has_speed_limit,
                "speed_limit_kph": speed_limit,
                "rule_payload_json": json.dumps(
                    payload, ensure_ascii=False, sort_keys=True, separators=(",", ":")
                ),
                "planning_enforcement_allowed": status != "provisional",
            }
        )
    return result


def _build_traffic_light_missions(
    config: Mapping[str, Any],
    hd_map: Mapping[str, Any],
    route_ranges: Sequence[Mapping[str, Any]],
    checkpoints: Sequence[Mapping[str, Any]],
) -> List[Dict[str, Any]]:
    if not config.get("derive_from_reviewed_stoplines", False):
        return []
    status = str(config.get("status", ""))
    if status not in ALLOWED_MISSION_STATUS:
        raise CompetitionMetadataError("traffic_light_zones status is invalid")
    if status == "provisional" and config.get("planning_enforcement_allowed") is not False:
        raise CompetitionMetadataError("provisional traffic-light zones must disable planning enforcement")
    if status == "team_final" and config.get("planning_enforcement_allowed") is not True:
        raise CompetitionMetadataError("team_final traffic-light zones must enable planning enforcement")

    grouped: Dict[str, List[Mapping[str, Any]]] = {}
    for feature in hd_map.get("features", []):
        if feature.get("feature_type") != "stopline":
            continue
        attributes = feature.get("attributes") or {}
        traffic_light_id = str(attributes.get("traffic_light_id") or "")
        if not traffic_light_id:
            continue
        route_s = (feature.get("validation") or {}).get("route_s_m")
        if route_s is None:
            raise CompetitionMetadataError("traffic-light stopline has no route_s_m: %s" % feature.get("id"))
        grouped.setdefault(traffic_light_id, []).append(feature)

    missions: List[Dict[str, Any]] = []
    for traffic_light_id, features in grouped.items():
        stations = [_finite((item.get("validation") or {}).get("route_s_m"), "stopline.route_s_m") for item in features]
        station = sum(stations) / len(stations)
        if max(stations) - min(stations) > 0.05:
            raise CompetitionMetadataError("grouped stoplines disagree in station: %s" % traffic_light_id)
        owner = _route_link_at_station(route_ranges, station)
        feature_ids = sorted(str(item.get("id")) for item in features)
        topology_ids = sorted(
            {
                str(link_id)
                for item in features
                for link_id in (item.get("topology") or {}).get("current_link_ids", [])
            }
        )
        shared_payload = config.get("rule_payload") or {}
        if not isinstance(shared_payload, dict):
            raise CompetitionMetadataError("traffic_light_zones.rule_payload must be a mapping")
        payload = dict(shared_payload)
        payload.update({
            "boundary_only": True,
            "hd_map_stopline_ids": feature_ids,
            "stopline_topology_link_ids": topology_ids,
            "traffic_light_id": traffic_light_id,
        })
        if status == "team_final":
            payload = _team_final_payload(traffic_light_id, payload, config.get("review"))
        missions.append(
            {
                "id": "traffic_light_%s" % traffic_light_id,
                "mission_type": "traffic_light",
                "status": status,
                "source_reference": str(config.get("source_reference", "")),
                "description": "검수된 정지선에서 파생한 신호등 경계 후보 %s" % traffic_light_id,
                "route_station_start_m": _stable_float(station),
                "route_station_end_m": _stable_float(station),
                "start_link_id": owner,
                "end_link_id": owner,
                "link_ids": [owner],
                "checkpoint_ids": _checkpoint_ids_in_span(checkpoints, station, station),
                "polygon": [],
                "has_speed_limit": False,
                "speed_limit_kph": 0.0,
                "rule_payload_json": json.dumps(
                    payload, ensure_ascii=False, sort_keys=True, separators=(",", ":")
                ),
                "planning_enforcement_allowed": bool(config.get("planning_enforcement_allowed", False)),
            }
        )
    missions.sort(key=lambda item: (item["route_station_start_m"], item["id"]))
    return missions


def _build_route_speed_limits(
    route_ranges: Sequence[Mapping[str, Any]],
    link_by_id: Mapping[str, Mapping[str, Any]],
    missions: Sequence[Mapping[str, Any]],
) -> List[Dict[str, Any]]:
    """Cap each route link's MGeo max_speed by the competition speed profile.

    The organizer MGeo keeps 70 km/h links inside the 60 km/h zone and 50/120
    km/h links inside the uncapped high-speed exception, so planning reads the
    effective per-link limit here instead of the raw link attribute.
    """

    profile = [
        item for item in missions
        if item["mission_type"] in ("speed_limit", "high_speed_exception")
    ]
    result: List[Dict[str, Any]] = []
    for item in route_ranges:
        link_id = str(item["link_id"])
        zones = [zone for zone in profile if link_id in zone["link_ids"]]
        if len(zones) != 1:
            raise CompetitionMetadataError("route link %s is in %d speed zones" % (link_id, len(zones)))
        zone = zones[0]
        mgeo = _finite(link_by_id[link_id].get("max_speed"), "link.%s.max_speed" % link_id)
        cap = float(zone["speed_limit_kph"]) if zone["has_speed_limit"] else None
        result.append(
            {
                "link_id": link_id,
                "route_station_start_m": _stable_float(item["start_station_m"]),
                "route_station_end_m": _stable_float(item["end_station_m"]),
                "speed_zone_id": str(zone["id"]),
                "mgeo_max_speed_kph": _stable_float(mgeo),
                "competition_cap_kph": cap,
                "effective_limit_kph": _stable_float(mgeo if cap is None else min(mgeo, cap)),
            }
        )
    return result


def _validate_route_speed_limits(
    limits: Any, route: Mapping[str, Any], speed_profile: Sequence[Mapping[str, Any]]
) -> int:
    ranges = route.get("link_ranges") or []
    if not isinstance(limits, list) or [str(item.get("link_id")) for item in limits] != [
        str(item["link_id"]) for item in ranges
    ]:
        raise CompetitionMetadataError("route_speed_limits must list every route link in order")
    zones = {str(item["id"]): item for item in speed_profile}
    capped = 0
    for limit, link_range in zip(limits, ranges):
        link_id = str(limit["link_id"])
        if (
            abs(float(limit["route_station_start_m"]) - float(link_range["start_station_m"])) > 1e-6
            or abs(float(limit["route_station_end_m"]) - float(link_range["end_station_m"])) > 1e-6
        ):
            raise CompetitionMetadataError("route speed limit stations drift: %s" % link_id)
        zone = zones.get(str(limit.get("speed_zone_id")))
        if zone is None or link_id not in zone.get("link_ids", []):
            raise CompetitionMetadataError("route speed limit has no owning speed zone: %s" % link_id)
        mgeo = _finite(limit.get("mgeo_max_speed_kph"), link_id + ".mgeo_max_speed_kph")
        cap = limit.get("competition_cap_kph")
        if zone.get("has_speed_limit"):
            if cap is None or abs(float(cap) - float(zone["speed_limit_kph"])) > 1e-9:
                raise CompetitionMetadataError("route speed cap differs from its zone: %s" % link_id)
            expected = min(mgeo, float(cap))
        else:
            if cap is not None:
                raise CompetitionMetadataError("high-speed exception link must be uncapped: %s" % link_id)
            expected = mgeo
        if abs(_finite(limit.get("effective_limit_kph"), link_id + ".effective") - expected) > 1e-9:
            raise CompetitionMetadataError("effective speed limit is not min(MGeo, cap): %s" % link_id)
        capped += int(cap is not None and mgeo > float(cap))
    return capped


def validate_competition_metadata(document: Mapping[str, Any]) -> Dict[str, Any]:
    if document.get("schema_version") != METADATA_SCHEMA_VERSION:
        raise CompetitionMetadataError("unsupported competition metadata schema")
    if document.get("frame_id") != "map":
        raise CompetitionMetadataError("competition metadata frame_id must be map")
    route = document.get("route")
    if not isinstance(route, dict) or route.get("closed_loop") is not True:
        raise CompetitionMetadataError("competition route must be a closed loop")
    route_length = _finite(route.get("length_xy_m"), "route.length_xy_m")
    expected_start_success = route_length * 0.05
    if abs(_finite(route.get("start_success_5_percent_station_m"), "route.start_success") - expected_start_success) > 1e-6:
        raise CompetitionMetadataError("start-success station is not 5% of route length")

    route_link_ids = route.get("link_ids")
    if not isinstance(route_link_ids, list) or not route_link_ids:
        raise CompetitionMetadataError("route.link_ids is empty")
    valid_links = set(str(value) for value in route_link_ids)
    checkpoints = document.get("checkpoints")
    if not isinstance(checkpoints, list) or len(checkpoints) != OFFICIAL_CHECKPOINT_COUNT:
        raise CompetitionMetadataError(
            "metadata must contain %d checkpoints" % OFFICIAL_CHECKPOINT_COUNT
        )
    expected_ids = ["START_END"] + [str(value) for value in range(1, OFFICIAL_CHECKPOINT_COUNT)]
    if [str(item.get("id")) for item in checkpoints] != expected_ids:
        raise CompetitionMetadataError("checkpoint IDs/order are invalid")
    previous_station = -math.inf
    max_route_error = 0.0
    max_link_error = 0.0
    for sequence, checkpoint in enumerate(checkpoints):
        if checkpoint.get("sequence") != sequence:
            raise CompetitionMetadataError("checkpoint sequence mismatch")
        if abs(_finite(checkpoint.get("pass_radius_m"), "checkpoint.radius") - 3.0) > 1e-12:
            raise CompetitionMetadataError("checkpoint pass radius must be 3 m")
        station = _finite(checkpoint.get("route_station_m"), "checkpoint.route_station_m")
        if station + 1e-9 < previous_station:
            raise CompetitionMetadataError("checkpoint stations are not monotonic")
        previous_station = station
        if str(checkpoint.get("link_id")) not in valid_links:
            raise CompetitionMetadataError("checkpoint references a non-route link")
        _finite(checkpoint.get("official_yaw_deg"), "checkpoint.official_yaw_deg")
        projection = checkpoint.get("projection") or {}
        max_route_error = max(max_route_error, _finite(projection.get("route_error_xy_m"), "route_error"))
        max_link_error = max(max_link_error, _finite(projection.get("link_error_xy_m"), "link_error"))
        orientation = (checkpoint.get("pose") or {}).get("orientation") or {}
        norm = math.sqrt(sum(_finite(orientation.get(axis), "orientation.%s" % axis) ** 2 for axis in ("x", "y", "z", "w")))
        if abs(norm - 1.0) > 1e-9:
            raise CompetitionMetadataError("checkpoint quaternion is not unit length")
    start_end = checkpoints[0]
    if start_end.get("is_start") is not True or start_end.get("is_finish") is not True:
        raise CompetitionMetadataError("START_END must be both start and finish")
    if abs(float(start_end["route_station_m"])) > 1e-9:
        raise CompetitionMetadataError("START_END must use route station zero")

    missions = document.get("mission_zones")
    if not isinstance(missions, list) or not missions:
        raise CompetitionMetadataError("mission_zones is empty")
    ids: set = set()
    statuses: Counter = Counter()
    speed_profile: List[Mapping[str, Any]] = []
    traffic_activation_policies: List[Tuple[Mapping[str, Any], Mapping[str, Any]]] = []
    traffic_light_boundaries: List[Mapping[str, Any]] = []
    gps_shadow_policies: List[Tuple[Mapping[str, Any], Mapping[str, Any]]] = []
    for mission in missions:
        mission_id = str(mission.get("id", ""))
        if not mission_id or mission_id in ids:
            raise CompetitionMetadataError("mission IDs must be non-empty and unique")
        ids.add(mission_id)
        status = str(mission.get("status", ""))
        if status not in ALLOWED_MISSION_STATUS:
            raise CompetitionMetadataError("mission status is required and must be authoritative")
        statuses[status] += 1
        if mission.get("mission_type") not in ALLOWED_MISSION_TYPES:
            raise CompetitionMetadataError("invalid mission type")
        start = _finite(mission.get("route_station_start_m"), "mission.start")
        end = _finite(mission.get("route_station_end_m"), "mission.end")
        if start < -1e-6 or end > route_length + 1e-6 or end + 1e-9 < start:
            raise CompetitionMetadataError("mission station bounds are invalid: %s" % mission_id)
        if str(mission.get("start_link_id")) not in valid_links or str(mission.get("end_link_id")) not in valid_links:
            raise CompetitionMetadataError("mission references a non-route boundary link: %s" % mission_id)
        if not str(mission.get("source_reference", "")):
            raise CompetitionMetadataError("mission source_reference is required: %s" % mission_id)
        try:
            payload = json.loads(str(mission.get("rule_payload_json", "")))
        except json.JSONDecodeError as exc:
            raise CompetitionMetadataError("mission rule_payload_json is invalid: %s" % exc)
        if not isinstance(payload, dict):
            raise CompetitionMetadataError("mission rule payload must be a JSON object")
        if status == "provisional" and mission.get("planning_enforcement_allowed") is not False:
            raise CompetitionMetadataError("provisional mission cannot enable planning enforcement")
        if status == "team_final" and (
            mission.get("planning_enforcement_allowed") is not True
            or payload.get("authority") != "team_final"
            or not (payload.get("review") or {}).get("reviewed_on")
            or payload.get("requires_official_boundary_confirmation")
        ):
            raise CompetitionMetadataError("team_final mission lacks its review authority: %s" % mission_id)
        if mission.get("mission_type") in ("speed_limit", "high_speed_exception"):
            speed_profile.append(mission)
        if mission.get("mission_type") == "traffic_light":
            if payload.get("runtime_activation_hint") == "enable_within_zone_only":
                traffic_activation_policies.append((mission, payload))
            elif abs(end - start) <= 1e-9:
                traffic_light_boundaries.append(mission)
        if mission.get("mission_type") == "gps_shadow":
            gps_shadow_policies.append((mission, payload))

    # Planning receives an effective, non-overlapping profile instead of an
    # all-route 60 km/h rule plus a priority-based exception.  Together these
    # segments must describe every station exactly once.
    speed_profile.sort(key=lambda item: float(item["route_station_start_m"]))
    if not speed_profile:
        raise CompetitionMetadataError("effective speed profile is empty")
    expected_start = 0.0
    for segment in speed_profile:
        mission_id = str(segment["id"])
        start = float(segment["route_station_start_m"])
        end = float(segment["route_station_end_m"])
        if abs(start - expected_start) > 1e-6:
            raise CompetitionMetadataError(
                "effective speed profile has a gap/overlap before %s" % mission_id
            )
        if segment["mission_type"] == "speed_limit":
            if segment.get("has_speed_limit") is not True:
                raise CompetitionMetadataError("speed-limit segment has no limit: %s" % mission_id)
            if _finite(segment.get("speed_limit_kph"), mission_id + ".speed_limit_kph") <= 0.0:
                raise CompetitionMetadataError("speed-limit segment is non-positive: %s" % mission_id)
        elif segment.get("has_speed_limit") is not False:
            raise CompetitionMetadataError(
                "high-speed exception must have no competition speed cap: %s" % mission_id
            )
        expected_start = end
    if abs(expected_start - route_length) > 1e-6:
        raise CompetitionMetadataError("effective speed profile does not cover the full route")

    if len(traffic_activation_policies) != 1:
        raise CompetitionMetadataError("exactly one traffic-light activation policy is required")
    activation, activation_payload = traffic_activation_policies[0]
    activation_start = float(activation["route_station_start_m"])
    activation_end = float(activation["route_station_end_m"])
    if (
        activation.get("start_link_id") != "A2256W000236"
        or activation.get("end_link_id") != "A2256W000308"
        or activation_payload.get("spatial_mission") is not False
        or activation_payload.get("policy_scope") != "link_span"
        or activation_payload.get("disable_outside_zone") is not True
    ):
        raise CompetitionMetadataError("traffic-light activation policy must be A236 through A308")
    if not traffic_light_boundaries or any(
        not (
            activation_start - 1e-6
            <= float(boundary["route_station_start_m"])
            <= activation_end + 1e-6
        )
        for boundary in traffic_light_boundaries
    ):
        raise CompetitionMetadataError("traffic-light boundary lies outside its activation policy")
    if len(gps_shadow_policies) != 1:
        raise CompetitionMetadataError("exactly one GPS-shadow policy is required")
    gps_shadow, gps_payload = gps_shadow_policies[0]
    if (
        gps_shadow.get("start_link_id") != "A2256W000126"
        or gps_shadow.get("end_link_id") != "A2256W000126"
        or gps_payload.get("usable_opposite_lane_link_ids") != ["A2256W000131"]
        or gps_payload.get("drivable_area_bridge_id") != "gps_shadow_A126_A131"
    ):
        raise CompetitionMetadataError("GPS-shadow policy must bind A126 to opposite lane A131")
    capped_link_count = _validate_route_speed_limits(
        document.get("route_speed_limits"), route, speed_profile
    )

    return {
        "passed": True,
        "checkpoint_count": len(checkpoints),
        "mission_zone_count": len(missions),
        "mission_status_counts": dict(sorted(statuses.items())),
        "speed_profile_segment_count": len(speed_profile),
        "speed_profile_full_route_coverage": True,
        "route_speed_limit_count": len(document.get("route_speed_limits") or []),
        "route_speed_links_capped_below_mgeo": capped_link_count,
        "traffic_light_activation_start_m": _stable_float(activation_start),
        "traffic_light_activation_end_m": _stable_float(activation_end),
        "traffic_light_boundary_count": len(traffic_light_boundaries),
        "max_checkpoint_route_projection_error_m": _stable_float(max_route_error, 12),
        "max_checkpoint_link_projection_error_m": _stable_float(max_link_error, 12),
    }


def build_competition_metadata(
    config_path: Path, repo_root: Path, data_root: Optional[Path] = None
) -> Dict[str, Any]:
    repo_root = repo_root.resolve()
    data_root = (data_root or default_data_root(repo_root)).resolve()
    config = _load_yaml(config_path.resolve())
    if config.get("schema_version") != SOURCE_SCHEMA_VERSION:
        raise CompetitionMetadataError("unsupported competition metadata source schema")
    inputs = config.get("inputs")
    if not isinstance(inputs, dict):
        raise CompetitionMetadataError("inputs mapping is required")

    authorities = {
        "official_checkpoints": "official",
        "official_global_path": "official",
        "current_link_set": "organizer_map",
        "route_links": "team_reviewed_route_decomposition",
        "reviewed_hd_map": "team_reviewed_candidate",
    }
    expected_sha = config.get("input_sha256")
    if not isinstance(expected_sha, dict) or set(expected_sha) != set(authorities):
        raise CompetitionMetadataError("input_sha256 must pin every input exactly")
    paths: Dict[str, Path] = {}
    sources: Dict[str, Dict[str, Any]] = {}
    for source_id, authority in authorities.items():
        raw = str(inputs.get(source_id, ""))
        path = _require_file(_resolve_input(repo_root, data_root, raw), source_id)
        actual_sha = sha256_file(path)
        if actual_sha != str(expected_sha[source_id]):
            raise CompetitionMetadataError(
                "frozen input SHA changed for %s: expected %s, got %s"
                % (source_id, expected_sha[source_id], actual_sha)
            )
        paths[source_id] = path
        sources[source_id] = _source_record(raw, path, authority)

    official = _load_json(paths["official_checkpoints"])
    global_path = _load_global_path(paths["official_global_path"])
    raw_links = _load_json(paths["current_link_set"])
    if not isinstance(raw_links, list):
        raise CompetitionMetadataError("current link_set root must be a list")
    link_by_id = {str(item.get("idx")): item for item in raw_links}
    route_config = _load_yaml(paths["route_links"])
    route_link_ids = [str(value) for value in ((route_config.get("route") or {}).get("links") or [])]
    if not route_link_ids or len(route_link_ids) != len(set(route_link_ids)):
        raise CompetitionMetadataError("route link list is empty or contains duplicates")
    validation = config.get("validation")
    if not isinstance(validation, dict):
        raise CompetitionMetadataError("validation mapping is required")

    route_ranges, link_geometry = _route_ranges(
        route_link_ids,
        link_by_id,
        float(validation["link_continuity_tolerance_m"]),
    )
    route_length = polyline_length_xy(global_path)
    link_route_length = float(route_ranges[-1]["end_station_m"])
    expected_length = float(validation["expected_route_length_m"])
    length_tolerance = float(validation["route_length_tolerance_m"])
    if abs(route_length - expected_length) > length_tolerance:
        raise CompetitionMetadataError("official global path length changed")
    if abs(link_route_length - route_length) > length_tolerance:
        raise CompetitionMetadataError("link decomposition length differs from official path")
    closure = math.hypot(
        global_path[-1][0] - global_path[0][0],
        global_path[-1][1] - global_path[0][1],
    )
    if closure > float(validation["route_closure_tolerance_m"]):
        raise CompetitionMetadataError("official route is not closed")

    assignment = config.get("checkpoint_link_assignment")
    if not isinstance(assignment, dict):
        raise CompetitionMetadataError("checkpoint_link_assignment mapping is required")
    checkpoints = _build_checkpoints(
        official,
        global_path,
        route_ranges,
        link_geometry,
        {str(key): str(value) for key, value in assignment.items()},
        validation,
    )
    mission_specs = config.get("mission_zones")
    if not isinstance(mission_specs, list):
        raise CompetitionMetadataError("mission_zones must be a list")
    missions = _build_authored_missions(
        mission_specs, route_length, route_link_ids, route_ranges, checkpoints
    )
    traffic_config = config.get("traffic_light_zones") or {}
    if not isinstance(traffic_config, dict):
        raise CompetitionMetadataError("traffic_light_zones must be a mapping")
    reviewed_map = _load_json(paths["reviewed_hd_map"])
    missions.extend(
        _build_traffic_light_missions(traffic_config, reviewed_map, route_ranges, checkpoints)
    )

    route_ranges_output = [
        {
            "link_id": item["link_id"],
            "order": item["order"],
            "start_station_m": _stable_float(item["start_station_m"]),
            "end_station_m": _stable_float(item["end_station_m"]),
            "length_m": _stable_float(item["length_m"]),
        }
        for item in route_ranges
    ]
    document: Dict[str, Any] = {
        "schema_version": METADATA_SCHEMA_VERSION,
        "metadata_id": str(config.get("metadata_id", "")),
        "contract_id": str(config.get("contract_id", "")),
        "frame_id": str(config.get("frame_id", "")),
        "sources": sources,
        "official_notice": {
            "source": official.get("source"),
            "author": official.get("author"),
            "announced": official.get("announced"),
            "map": official.get("map"),
            "notes": list(official.get("notes") or []),
        },
        "route": {
            "route_id": "2026_molit_comp_global_path",
            "closed_loop": True,
            "point_count": len(global_path),
            "length_xy_m": _stable_float(route_length),
            "start_success_5_percent_station_m": _stable_float(route_length * 0.05),
            "closure_error_xy_m": _stable_float(closure, 12),
            "link_ids": route_link_ids,
            "link_ranges": route_ranges_output,
        },
        "checkpoints": checkpoints,
        "mission_zones": missions,
        "route_speed_limits": _build_route_speed_limits(route_ranges, link_by_id, missions),
    }
    document["validation"] = validate_competition_metadata(document)
    return document


def dump_metadata_yaml(document: Mapping[str, Any]) -> str:
    return yaml.safe_dump(
        dict(document),
        allow_unicode=True,
        sort_keys=False,
        default_flow_style=False,
        width=120,
    )
