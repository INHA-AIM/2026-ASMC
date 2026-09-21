#!/usr/bin/env python3
"""Publish the frozen P1-D HD-map and competition metadata read-only."""

from __future__ import annotations

from collections import defaultdict
import hashlib
import json
import math
import os
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Sequence, Tuple

from asmc_msgs.msg import (
    CompetitionCheckpoint,
    CompetitionCheckpointArray,
    HdMapMeta,
    DrivableArea,
    DrivableAreaArray,
    Lane,
    LaneArray,
    LaneMarking,
    LaneMarkingArray,
    MissionZone,
    MissionZoneArray,
)
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rospy
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray

from asmc_hd_map.map_overlay import dashed_polyline_segments
from asmc_hd_map.runtime_bundle import (
    RuntimeBundleError,
    choose_label_position,
    load_runtime_bundle,
    marking_role_name,
    point_at_station,
    polyline_slice,
    short_link_id,
    source_type_name,
)


COLOR_VALUES = {
    "white": LaneMarking.COLOR_WHITE,
    "yellow": LaneMarking.COLOR_YELLOW,
}
PATTERN_VALUES = {
    "solid": LaneMarking.PATTERN_SOLID,
    "broken": LaneMarking.PATTERN_BROKEN,
}
ROLE_VALUES = {
    "road_center": LaneMarking.ROLE_ROAD_CENTER,
    "lane_divider": LaneMarking.ROLE_LANE_DIVIDER,
    "road_edge": LaneMarking.ROLE_ROAD_EDGE,
    "stop_line": LaneMarking.ROLE_STOP_LINE,
    "keep_out": LaneMarking.ROLE_KEEP_OUT,
    "island_edge": LaneMarking.ROLE_ISLAND_EDGE,
}
SOURCE_VALUES = {
    "survey": LaneMarking.SOURCE_SURVEY,
    "legacy_map": LaneMarking.SOURCE_LEGACY_MAP,
    "preplan": LaneMarking.SOURCE_PREPLAN,
    "curated": LaneMarking.SOURCE_CURATED,
    "derived": LaneMarking.SOURCE_DERIVED,
}
MISSION_TYPE_VALUES = {
    "start_success": MissionZone.TYPE_START_SUCCESS,
    "speed_limit": MissionZone.TYPE_SPEED_LIMIT,
    "high_speed_exception": MissionZone.TYPE_HIGH_SPEED_EXCEPTION,
    "gps_shadow": MissionZone.TYPE_GPS_SHADOW,
    "static_obstacle": MissionZone.TYPE_STATIC_OBSTACLE,
    "dynamic_obstacle": MissionZone.TYPE_DYNAMIC_OBSTACLE,
    "roundabout": MissionZone.TYPE_ROUNDABOUT,
    "highway_merge": MissionZone.TYPE_HIGHWAY_MERGE,
    "surrounding_vehicle": MissionZone.TYPE_SURROUNDING_VEHICLE,
    "traffic_light": MissionZone.TYPE_TRAFFIC_LIGHT,
}
MISSION_STATUS_VALUES = {
    "official": MissionZone.STATUS_OFFICIAL,
    "derived_official": MissionZone.STATUS_DERIVED_OFFICIAL,
    "provisional": MissionZone.STATUS_PROVISIONAL,
}
LANE_TYPE_VALUES = {
    "driving": Lane.TYPE_DRIVING,
    "merge": Lane.TYPE_MERGE,
    "diverge_toll": Lane.TYPE_TOLL,
    "intersection": Lane.TYPE_INTERSECTION,
}
DRIVABLE_TYPE_VALUES = {
    "lane": DrivableArea.TYPE_LANE,
    "junction": DrivableArea.TYPE_JUNCTION,
    "merge": DrivableArea.TYPE_MERGE,
    "diverge": DrivableArea.TYPE_DIVERGE,
    "unmarked_road": DrivableArea.TYPE_UNMARKED_ROAD,
}
DRIVABLE_SOURCE_VALUES = {
    "road_mesh": DrivableArea.SOURCE_ROAD_MESH,
    "lane_clipped": DrivableArea.SOURCE_LANE_CLIPPED,
    "hybrid": DrivableArea.SOURCE_HYBRID,
}


def _point(values: Sequence[float], z_offset: float = 0.0) -> Point:
    message = Point()
    message.x = float(values[0])
    message.y = float(values[1])
    message.z = (float(values[2]) if len(values) > 2 else 0.0) + z_offset
    return message


def _color(red: float, green: float, blue: float, alpha: float = 1.0) -> ColorRGBA:
    message = ColorRGBA()
    message.r, message.g, message.b, message.a = red, green, blue, alpha
    return message


def _header(stamp, frame_id: str = "map") -> Header:
    message = Header()
    message.stamp = stamp
    message.frame_id = frame_id
    return message


def _feature_message(feature: Mapping[str, Any]) -> LaneMarking:
    attributes = feature.get("attributes") or {}
    provenance = feature.get("provenance") or {}
    message = LaneMarking()
    message.id = str(feature.get("id", ""))
    message.color = COLOR_VALUES.get(str(attributes.get("color", "")).lower(), LaneMarking.COLOR_UNKNOWN)
    message.pattern = PATTERN_VALUES.get(
        str(attributes.get("pattern", "")).lower(), LaneMarking.PATTERN_UNKNOWN
    )
    message.physical_type = LaneMarking.PHYSICAL_PAINTED
    message.role = ROLE_VALUES.get(marking_role_name(feature), LaneMarking.ROLE_UNKNOWN)
    message.width_m = float(attributes.get("line_width_m", 0.0))
    message.polyline = [_point(values) for values in (feature.get("geometry") or {}).get("coordinates", [])]
    # Lane adjacency is intentionally empty in P1-D. P2 creates reviewed Lane
    # topology; nearest-neighbour inference here could bridge a real paint gap.
    message.left_lane_ids = []
    message.right_lane_ids = []
    message.source_type = SOURCE_VALUES.get(source_type_name(feature), LaneMarking.SOURCE_UNKNOWN)
    message.source_dataset = str(attributes.get("source_display", provenance.get("source_kind", "")))
    message.source_feature_id = str(provenance.get("source_id", ""))
    return message


def _checkpoint_message(item: Mapping[str, Any]) -> CompetitionCheckpoint:
    message = CompetitionCheckpoint()
    message.id = str(item["id"])
    message.sequence = int(item["sequence"])
    message.is_start = bool(item["is_start"])
    message.is_finish = bool(item["is_finish"])
    position = item["pose"]["position"]
    orientation = item["pose"]["orientation"]
    message.pose.position = _point([position["x"], position["y"], position["z"]])
    message.pose.orientation.x = float(orientation["x"])
    message.pose.orientation.y = float(orientation["y"])
    message.pose.orientation.z = float(orientation["z"])
    message.pose.orientation.w = float(orientation["w"])
    message.official_yaw_deg = float(item["official_yaw_deg"])
    message.pass_radius_m = float(item["pass_radius_m"])
    message.route_station_m = float(item["route_station_m"])
    message.link_id = str(item["link_id"])
    message.link_station_m = float(item["link_station_m"])
    message.source_reference = str(item["source_reference"])
    return message


def _mission_message(item: Mapping[str, Any]) -> MissionZone:
    mission_type = str(item.get("mission_type", ""))
    status = str(item.get("status", ""))
    if mission_type not in MISSION_TYPE_VALUES:
        raise RuntimeBundleError("unsupported mission type for ROS publication: %s" % mission_type)
    if status not in MISSION_STATUS_VALUES:
        raise RuntimeBundleError("unsupported mission status for ROS publication: %s" % status)
    if status == "provisional" and bool(item.get("planning_enforcement_allowed")):
        raise RuntimeBundleError("provisional mission enables planning: %s" % item.get("id"))
    message = MissionZone()
    message.id = str(item["id"])
    message.mission_type = MISSION_TYPE_VALUES[mission_type]
    message.status = MISSION_STATUS_VALUES[status]
    message.source_reference = str(item["source_reference"])
    message.description = str(item["description"])
    message.route_station_start_m = float(item["route_station_start_m"])
    message.route_station_end_m = float(item["route_station_end_m"])
    message.start_link_id = str(item["start_link_id"])
    message.end_link_id = str(item["end_link_id"])
    message.link_ids = [str(value) for value in item.get("link_ids", [])]
    message.checkpoint_ids = [str(value) for value in item.get("checkpoint_ids", [])]
    message.polygon = [_point(value) for value in item.get("polygon", [])]
    message.has_speed_limit = bool(item.get("has_speed_limit"))
    message.speed_limit_kph = float(item.get("speed_limit_kph", 0.0))
    message.rule_payload_json = str(item.get("rule_payload_json", ""))
    return message


def _lane_message(item: Mapping[str, Any]) -> Lane:
    message = Lane()
    message.id = str(item["id"])
    message.link_id = str(item["link_id"])
    message.lane_type = LANE_TYPE_VALUES.get(str(item.get("lane_type")), Lane.TYPE_UNKNOWN)
    message.direction = Lane.DIRECTION_FORWARD
    message.lane_number = int(item.get("lane_number", 0))
    message.lane_number_scope_id = str(item.get("lane_number_scope_id", ""))
    message.route_station_start_m = float(item.get("route_station_start_m", -1.0))
    message.route_station_end_m = float(item.get("route_station_end_m", -1.0))
    message.left_boundary_id = str(item.get("left_boundary_id", ""))
    message.right_boundary_id = str(item.get("right_boundary_id", ""))
    message.left_adjacent_lane_id = str(item.get("left_adjacent_lane_id", ""))
    message.right_adjacent_lane_id = str(item.get("right_adjacent_lane_id", ""))
    message.predecessor_lane_ids = [str(value) for value in item.get("predecessor_lane_ids", [])]
    message.successor_lane_ids = [str(value) for value in item.get("successor_lane_ids", [])]
    message.centerline = [_point(value) for value in item.get("centerline", [])]
    message.polygon = [_point(value) for value in item.get("polygon", [])]
    message.source_type = Lane.SOURCE_DERIVED
    message.source_dataset = str(item.get("source_dataset", ""))
    message.source_feature_id = str(item.get("source_feature_id", ""))
    return message


def _drivable_message(item: Mapping[str, Any]) -> DrivableArea:
    message = DrivableArea()
    message.id = str(item["id"])
    message.area_type = DRIVABLE_TYPE_VALUES.get(str(item.get("area_type")), DrivableArea.TYPE_UNKNOWN)
    message.validity = (
        DrivableArea.VALIDITY_VALIDATED
        if str(item.get("validity")) == "validated"
        else DrivableArea.VALIDITY_PROVISIONAL
    )
    message.source_type = DRIVABLE_SOURCE_VALUES.get(
        str(item.get("source_type")), DrivableArea.SOURCE_UNKNOWN
    )
    message.polygon = [_point(value) for value in item.get("polygon", [])]
    message.lane_ids = [str(value) for value in item.get("lane_ids", [])]
    message.link_ids = [str(value) for value in item.get("link_ids", [])]
    message.intersection_guarded = bool(item.get("intersection_guarded"))
    message.source_dataset = str(item.get("source_dataset", ""))
    message.source_feature_id = str(item.get("source_feature_id", ""))
    return message


def build_static_messages(
    bundle: Mapping[str, Any], stamp, product: Mapping[str, Any] | None = None,
    occupancy: np.ndarray | None = None,
) -> Dict[str, Any]:
    map_document = bundle["map"]
    metadata = bundle["metadata"]
    build = map_document.get("build") or {}
    frame = map_document.get("frame") or {}
    markings = [_feature_message(feature) for feature in bundle["features"]]

    meta = HdMapMeta()
    meta.header = _header(stamp)
    meta.schema_major = 1
    meta.schema_minor = 0
    meta.contract_id = str(metadata["contract_id"])
    meta.build_id = str(build["build_id"])
    meta.map_id = str(map_document["map_id"])
    meta.geometry_sha256 = str(bundle["geometry_sha256"])
    meta.manifest_sha256 = str(bundle["manifest_sha256"])
    meta.generated_at = rospy.Time(0)
    meta.scope = HdMapMeta.SCOPE_GLOBAL
    meta.operational = bool(build.get("operational", False))
    meta.complete = product is not None
    if product is not None:
        meta.build_id = str(product["build_id"])
        meta.manifest_sha256 = str(product["product_sha256"])
        meta.operational = True
    meta.coordinate_system = str(frame.get("crs", ""))
    meta.source_ids = ["reviewed_hd_map", "competition_metadata", "official_global_path"]
    meta.source_sha256 = [
        str(bundle["map_sha256"]),
        str(bundle["metadata_sha256"]),
        str((metadata.get("sources") or {}).get("official_global_path", {}).get("sha256", "")),
    ]
    if product is not None:
        meta.source_ids.extend(["compact_drivable_product", "legacy_drivable_h5"])
        meta.source_sha256.extend([
            str(product["product_sha256"]),
            str((product.get("sources") or {}).get("legacy_drivable_h5", {}).get("sha256", "")),
        ])
    meta.lane_marking_count = len(markings)
    meta.lane_count = len(product["lanes"]) if product is not None else 0
    meta.drivable_area_count = len(product["drivable_areas"]) if product is not None else 0
    meta.checkpoint_count = len(bundle["checkpoints"])
    meta.mission_zone_count = len(bundle["mission_zones"])

    lane_array = LaneMarkingArray()
    lane_array.header = _header(stamp)
    lane_array.build_id = meta.build_id
    lane_array.map_id = meta.map_id
    lane_array.geometry_sha256 = meta.geometry_sha256
    lane_array.complete = True
    lane_array.markings = markings

    checkpoint_array = CompetitionCheckpointArray()
    checkpoint_array.header = _header(stamp)
    checkpoint_array.build_id = meta.build_id
    checkpoint_array.route_id = str(metadata["route"]["route_id"])
    checkpoint_array.source_sha256 = str(
        (metadata.get("sources") or {}).get("official_checkpoints", {}).get("sha256", "")
    )
    checkpoint_array.route_length_m = float(metadata["route"]["length_xy_m"])
    checkpoint_array.checkpoints = [_checkpoint_message(item) for item in bundle["checkpoints"]]

    mission_array = MissionZoneArray()
    mission_array.header = _header(stamp)
    mission_array.build_id = meta.build_id
    mission_array.route_id = checkpoint_array.route_id
    mission_array.source_sha256 = str(bundle["metadata_sha256"])
    mission_array.zones = [_mission_message(item) for item in bundle["mission_zones"]]
    result = {
        "meta": meta,
        "lane_markings": lane_array,
        "checkpoints": checkpoint_array,
        "mission_zones": mission_array,
    }
    if product is not None:
        lane_array_message = LaneArray()
        lane_array_message.header = _header(stamp)
        lane_array_message.build_id = meta.build_id
        lane_array_message.map_id = meta.map_id
        lane_array_message.geometry_sha256 = meta.geometry_sha256
        lane_array_message.complete = True
        lane_array_message.lanes = [_lane_message(item) for item in product["lanes"]]
        drivable_array = DrivableAreaArray()
        drivable_array.header = _header(stamp)
        drivable_array.build_id = meta.build_id
        drivable_array.map_id = meta.map_id
        drivable_array.geometry_sha256 = meta.geometry_sha256
        drivable_array.complete = True
        drivable_array.areas = [_drivable_message(item) for item in product["drivable_areas"]]
        if occupancy is None:
            raise RuntimeBundleError("P2 product requires an occupancy grid")
        grid = OccupancyGrid()
        grid.header = _header(stamp)
        grid.info.map_load_time = stamp
        grid.info.resolution = float(product["grid"]["resolution_m"])
        grid.info.width = int(product["grid"]["width"])
        grid.info.height = int(product["grid"]["height"])
        grid.info.origin.position.x = float(product["grid"]["origin_x"])
        grid.info.origin.position.y = float(product["grid"]["origin_y"])
        grid.info.origin.orientation.w = 1.0
        if tuple(occupancy.shape) != (grid.info.height, grid.info.width):
            raise RuntimeBundleError("compact occupancy shape differs from product metadata")
        grid.data = occupancy.reshape(-1).astype(np.int8).tolist()
        result.update({"lanes": lane_array_message, "drivable_areas": drivable_array, "grid": grid})
    return result


def _marker(namespace: str, marker_id: int, marker_type: int, stamp) -> Marker:
    value = Marker()
    value.header = _header(stamp)
    value.ns = namespace
    value.id = marker_id
    value.type = marker_type
    value.action = Marker.ADD
    value.pose.orientation.w = 1.0
    return value


def _segments_for_feature(feature: Mapping[str, Any]) -> Iterable[Tuple[Sequence[float], Sequence[float]]]:
    attributes = feature.get("attributes") or {}
    coordinates = (feature.get("geometry") or {}).get("coordinates", [])
    if len(coordinates) < 2:
        return []
    if str(attributes.get("pattern", "solid")).lower() != "broken":
        return list(zip(coordinates, coordinates[1:]))
    segments = dashed_polyline_segments(
        [coordinates],
        float(attributes.get("dash_length_m", 3.0)),
        float(attributes.get("dash_gap_m", 3.0)),
        float(attributes.get("dash_phase_m", 0.0)),
    )
    return [(segment[0], segment[1]) for segment in np.asarray(segments)]


def _line_color(color_name: str, stopline: bool = False) -> ColorRGBA:
    if stopline:
        return _color(0.35, 1.0, 1.0, 1.0)
    if color_name == "yellow":
        return _color(1.0, 0.78, 0.0, 1.0)
    return _color(0.95, 0.95, 0.95, 1.0)


def _mission_color(status: str) -> ColorRGBA:
    if status == "official":
        return _color(0.1, 0.9, 0.2, 0.75)
    if status == "derived_official":
        return _color(0.1, 0.65, 1.0, 0.75)
    return _color(1.0, 0.35, 0.05, 0.60)


MISSION_LABELS = {
    "start_success": "START 5%",
    "speed_limit": "SPEED LIMIT",
    "high_speed_exception": "COMP LIMIT NONE",
    "gps_shadow": "GPS SHADOW",
    "static_obstacle": "STATIC OBS",
    "dynamic_obstacle": "DYNAMIC OBS",
    "roundabout": "ROUNDABOUT",
    "highway_merge": "MERGE",
    "surrounding_vehicle": "SURROUNDING",
    "traffic_light": "TRAFFIC LIGHT",
}
MISSION_STATUS_LABELS = {
    "official": "OFF",
    "derived_official": "DER",
    "provisional": "PROV",
}


def _mission_rule_payload(item: Mapping[str, Any]) -> Mapping[str, Any]:
    try:
        payload = json.loads(str(item.get("rule_payload_json", "{}")))
    except ValueError as error:
        raise RuntimeBundleError("invalid mission rule payload: %s" % error)
    if not isinstance(payload, dict):
        raise RuntimeBundleError("mission rule payload must be an object")
    return payload


def _mission_label(item: Mapping[str, Any], start: float, end: float) -> str:
    mission_type = str(item["mission_type"])
    name = MISSION_LABELS.get(mission_type, mission_type.upper())
    payload = _mission_rule_payload(item)
    if mission_type == "speed_limit" and bool(item.get("has_speed_limit")):
        name = "SPEED %.0f" % float(item.get("speed_limit_kph", 0.0))
    if mission_type == "traffic_light" and start == end:
        identifier = str(item.get("id", ""))
        name = "TL " + identifier[-3:]
    elif payload.get("runtime_activation_hint") == "enable_within_zone_only":
        name = "TL ACTIVE"
    status = MISSION_STATUS_LABELS.get(str(item["status"]), "?")
    if abs(end - start) <= 1e-6:
        return "%s [%s]\n%.0f m" % (name, status, start)
    return "%s [%s]\n%.0f–%.0f m" % (name, status, start, end)


def _speed_color(has_limit: bool) -> ColorRGBA:
    if has_limit:
        return _color(0.1, 0.9, 1.0, 0.82)
    return _color(0.8, 0.3, 1.0, 0.90)


def _policy_color() -> ColorRGBA:
    return _color(0.45, 1.0, 0.2, 0.88)


def _is_policy_only(item: Mapping[str, Any]) -> bool:
    return _mission_rule_payload(item).get("spatial_mission") is False


def build_debug_markers(
    bundle: Mapping[str, Any], stamp, product: Mapping[str, Any] | None = None
) -> MarkerArray:
    markers: List[Marker] = []
    # Text labels share one collision set even though their marker namespaces
    # remain independently toggleable in RViz.  This keeps a checkpoint at a
    # link boundary from being drawn directly over the link or mission label.
    occupied_text_xy: List[List[float]] = []
    if product is not None:
        lane_centerlines = _marker("compact_lane_centerlines", 0, Marker.LINE_LIST, stamp)
        lane_centerlines.scale.x = 0.13
        lane_centerlines.color = _color(0.05, 0.95, 0.95, 0.92)
        lane_polygons = _marker("compact_lane_polygons", 0, Marker.LINE_LIST, stamp)
        lane_polygons.scale.x = 0.08
        lane_polygons.color = _color(0.15, 0.8, 0.95, 0.72)
        for lane in product.get("lanes", []):
            centerline = lane.get("centerline", [])
            for start, end in zip(centerline, centerline[1:]):
                lane_centerlines.points.extend((_point(start, 0.16), _point(end, 0.16)))
            polygon = lane.get("polygon", [])
            for start, end in zip(polygon, polygon[1:] + polygon[:1]):
                lane_polygons.points.extend((_point(start, 0.13), _point(end, 0.13)))
        markers.extend((lane_polygons, lane_centerlines))

        group_colors = {
            "route_only_connectors": _color(0.2, 0.75, 1.0, 0.95),
            "static_obstacle_maneuver": _color(1.0, 0.55, 0.05, 0.98),
            "dynamic_obstacle_maneuver": _color(1.0, 0.1, 0.7, 0.98),
            "non_mission_route_turns": _color(0.45, 1.0, 0.25, 0.95),
            "gps_shadow_opposite_lane": _color(0.65, 0.45, 1.0, 0.98),
            "roundabout_maneuver": _color(0.15, 0.65, 1.0, 0.98),
        }
        for marker_id, (group, color) in enumerate(group_colors.items()):
            grouped_marker = _marker("drivable_group_%s" % group, marker_id, Marker.LINE_LIST, stamp)
            grouped_marker.scale.x = 0.20
            grouped_marker.color = color
            for lane in product.get("lanes", []):
                if lane.get("drivable_group") != group:
                    continue
                polygon = lane.get("polygon", [])
                for start, end in zip(polygon, polygon[1:] + polygon[:1]):
                    grouped_marker.points.extend((_point(start, 0.24), _point(end, 0.24)))
            markers.append(grouped_marker)
            if group == "roundabout_maneuver":
                for label_id, lane in enumerate(
                    value for value in product.get("lanes", [])
                    if value.get("drivable_group") == group
                ):
                    centerline = lane.get("centerline", [])
                    if not centerline:
                        continue
                    label = _marker(
                        "drivable_group_roundabout_labels", label_id,
                        Marker.TEXT_VIEW_FACING, stamp,
                    )
                    label.pose.position = _point(centerline[len(centerline) // 2], 0.9)
                    label.scale.z = 1.25
                    label.color = color
                    label.text = "%s ROUNDABOUT DRIVABLE" % lane.get("link_id", "A134")
                    markers.append(label)

        surface_styles = {
            "selected_lane_topology_corridor_fill": (
                "compact_corridor_fills", _color(1.0, 0.75, 0.15, 0.92)
            ),
            "legacy_h5_clipped_to_selected_link_envelope": (
                "compact_intersection_surface_patches", _color(1.0, 0.25, 0.05, 0.95)
            ),
        }
        surface_markers = {}
        for marker_id, (dataset, (namespace, color)) in enumerate(surface_styles.items()):
            value = _marker(namespace, marker_id, Marker.LINE_LIST, stamp)
            value.scale.x = 0.16
            value.color = color
            surface_markers[dataset] = value
            markers.append(value)
        roundabout_surface = _marker(
            "compact_roundabout_surface", len(surface_markers), Marker.LINE_LIST, stamp
        )
        roundabout_surface.scale.x = 0.22
        roundabout_surface.color = _color(0.1, 0.75, 1.0, 0.98)
        markers.append(roundabout_surface)
        for area in product.get("drivable_areas", []):
            marker = (
                roundabout_surface
                if area.get("surface_group") == "roundabout_maneuver"
                else surface_markers.get(str(area.get("source_dataset")))
            )
            if marker is None:
                continue
            polygon = area.get("polygon", [])
            for start, end in zip(polygon, polygon[1:] + polygon[:1]):
                marker.points.extend((_point(start, 0.22), _point(end, 0.22)))

        junctions = _marker("compact_guarded_junctions", 0, Marker.LINE_LIST, stamp)
        junctions.scale.x = 0.12
        junctions.color = _color(1.0, 0.55, 0.05, 0.9)
        for area in product.get("drivable_areas", []):
            if not area.get("intersection_guarded"):
                continue
            polygon = area.get("polygon", [])
            for start, end in zip(polygon, polygon[1:] + polygon[:1]):
                junctions.points.extend((_point(start, 0.18), _point(end, 0.18)))
        markers.append(junctions)
    grouped: Dict[Tuple[str, str, str, float], List[Tuple[Sequence[float], Sequence[float]]]] = defaultdict(list)
    for feature in bundle["features"]:
        attributes = feature.get("attributes") or {}
        key = (
            str(attributes.get("color", "unknown")).lower(),
            str(attributes.get("pattern", "unknown")).lower(),
            "stopline" if feature.get("feature_type") == "stopline" else "lane",
            float(attributes.get("line_width_m", 0.15)),
        )
        grouped[key].extend(_segments_for_feature(feature))
    for marker_id, (key, segments) in enumerate(sorted(grouped.items())):
        color_name, pattern, layer, width = key
        value = _marker("marking_%s_%s_%s" % (layer, color_name, pattern), marker_id, Marker.LINE_LIST, stamp)
        value.scale.x = max(0.03, width)
        value.color = _line_color(color_name, layer == "stopline")
        for start, end in segments:
            value.points.extend((_point(start, 0.04), _point(end, 0.04)))
        markers.append(value)

    route = _marker("route", 0, Marker.LINE_STRIP, stamp)
    route.scale.x = 0.12
    route.color = _color(0.1, 0.45, 1.0, 0.75)
    route.points = [_point(value, 0.08) for value in bundle["route_points"]]
    markers.append(route)

    for index, item in enumerate(bundle["metadata"]["route"]["link_ranges"]):
        start = float(item["start_station_m"])
        end = float(item["end_station_m"])
        middle = 0.5 * (start + end)
        center = point_at_station(bundle["route_points"], bundle["route_stations"], middle)
        ahead = point_at_station(
            bundle["route_points"], bundle["route_stations"], min(end, middle + 2.5)
        )
        arrow = _marker("route_link_direction", index, Marker.ARROW, stamp)
        arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.18, 0.55, 0.8
        arrow.color = _color(0.15, 0.55, 1.0, 0.85)
        arrow.points = [_point(center, 0.30), _point(ahead, 0.30)]
        markers.append(arrow)
        label_position = choose_label_position(
            bundle["route_points"],
            bundle["route_stations"],
            middle,
            (5.0, 10.0, -5.0, -10.0, 15.0, -15.0, 20.0, -20.0, 25.0, -25.0),
            occupied_text_xy,
            11.0,
        )
        occupied_text_xy.append(label_position)
        label = _marker("route_link_labels", index, Marker.TEXT_VIEW_FACING, stamp)
        label.pose.position = _point(label_position, 1.0)
        label.scale.z = 1.15
        label.color = _color(0.45, 0.75, 1.0, 0.95)
        label.text = "%s\n%.0f–%.0f m" % (short_link_id(item["link_id"]), start, end)
        markers.append(label)

    route_length = float(bundle["metadata"]["route"]["length_xy_m"])
    for index, station in enumerate(np.arange(0.0, route_length, 100.0)):
        center = point_at_station(bundle["route_points"], bundle["route_stations"], float(station))
        tick = _marker("route_station", index, Marker.SPHERE, stamp)
        tick.pose.position = _point(center, 0.25)
        tick.scale.x = tick.scale.y = tick.scale.z = 0.45
        tick.color = _color(0.2, 0.55, 1.0, 0.9)
        markers.append(tick)
        # A visible tick remains every 100 m, while text is limited to 200 m
        # to keep the overview readable.
        if index % 2:
            continue
        label_position = choose_label_position(
            bundle["route_points"],
            bundle["route_stations"],
            float(station),
            (-3.0, -7.0, 3.0, 7.0, -12.0, 12.0),
            occupied_text_xy,
            9.0,
        )
        occupied_text_xy.append(label_position)
        text = _marker("route_station_text", index, Marker.TEXT_VIEW_FACING, stamp)
        text.pose.position = _point(label_position, 1.1)
        text.scale.z = 0.9
        text.color = _color(0.5, 0.8, 1.0, 0.95)
        text.text = "%d m" % int(station)
        markers.append(text)

    for index, item in enumerate(bundle["checkpoints"]):
        position = item["pose"]["position"]
        center = [position["x"], position["y"], position["z"]]
        radius = float(item["pass_radius_m"])
        ring = _marker("checkpoints_radius", index, Marker.LINE_STRIP, stamp)
        ring.scale.x = 0.20
        ring.color = _color(0.2, 1.0, 0.25, 0.95)
        ring.points = [
            _point(
                [
                    center[0] + radius * math.cos(2.0 * math.pi * step / 64.0),
                    center[1] + radius * math.sin(2.0 * math.pi * step / 64.0),
                    center[2],
                ],
                0.20,
            )
            for step in range(65)
        ]
        markers.append(ring)
        yaw = math.radians(float(item["official_yaw_deg"]))
        arrow_end = [center[0] + 4.0 * math.cos(yaw), center[1] + 4.0 * math.sin(yaw), center[2]]
        arrow = _marker("checkpoints_heading", index, Marker.ARROW, stamp)
        arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.22, 0.70, 0.95
        arrow.color = _color(0.1, 1.0, 0.2, 0.95)
        arrow.points = [_point(center, 0.3), _point(arrow_end, 0.3)]
        markers.append(arrow)
        label_position = choose_label_position(
            bundle["route_points"],
            bundle["route_stations"],
            float(item["route_station_m"]),
            (-7.0, -12.0, 7.0, 12.0, -17.0, 17.0, -22.0, 22.0),
            occupied_text_xy,
            11.0,
        )
        occupied_text_xy.append(label_position)
        text = _marker("checkpoints_text", index, Marker.TEXT_VIEW_FACING, stamp)
        text.pose.position = _point(label_position, 2.0)
        text.scale.z = 1.25
        text.color = _color(0.15, 1.0, 0.25, 1.0)
        text.text = "CP %s\n%s · %.1f m" % (
            item.get("label", item["id"]),
            short_link_id(item["link_id"]),
            float(item["route_station_m"]),
        )
        markers.append(text)

    for index, item in enumerate(bundle["mission_zones"]):
        start = float(item["route_station_start_m"])
        end = float(item["route_station_end_m"])
        mission_type = str(item["mission_type"])
        policy_only = _is_policy_only(item)
        policy_payload = _mission_rule_payload(item)
        policy_corridor = policy_only and policy_payload.get("policy_scope") == "link_span"
        speed_rule = mission_type in ("speed_limit", "high_speed_exception")
        color = (
            _policy_color()
            if policy_only
            else _speed_color(bool(item.get("has_speed_limit")))
            if speed_rule
            else _mission_color(str(item["status"]))
        )
        if policy_corridor:
            coordinates = polyline_slice(bundle["route_points"], bundle["route_stations"], start, end)
            zone_marker = _marker("perception_activation_corridors", index, Marker.LINE_STRIP, stamp)
            zone_marker.scale.x = 0.34
            zone_marker.points = [_point(value, 0.34) for value in coordinates]
            center = point_at_station(
                bundle["route_points"], bundle["route_stations"], 0.5 * (start + end)
            )
        elif policy_only:
            center = point_at_station(bundle["route_points"], bundle["route_stations"], 0.0)
        elif end - start <= 1e-6:
            center = point_at_station(bundle["route_points"], bundle["route_stations"], start)
            zone_marker = _marker("mission_boundaries", index, Marker.SPHERE, stamp)
            zone_marker.pose.position = _point(center, 0.5)
            zone_marker.scale.x = zone_marker.scale.y = zone_marker.scale.z = 1.2
        else:
            coordinates = polyline_slice(bundle["route_points"], bundle["route_stations"], start, end)
            namespace = (
                "speed_limit_corridors"
                if mission_type == "speed_limit"
                else "speed_limit_exceptions"
                if mission_type == "high_speed_exception"
                else "mission_corridors"
            )
            zone_marker = _marker(namespace, index, Marker.LINE_STRIP, stamp)
            zone_marker.scale.x = 0.42 if speed_rule else 0.28
            z_offset = 0.12 if speed_rule else 0.26 + 0.04 * (index % 5)
            zone_marker.points = [_point(value, z_offset) for value in coordinates]
            center = point_at_station(
                bundle["route_points"], bundle["route_stations"], 0.5 * (start + end)
            )
        if not policy_only or policy_corridor:
            zone_marker.color = color
            markers.append(zone_marker)
        label_station = (
            0.5 * (start + end)
            if policy_corridor
            else 0.0
            if policy_only
            else start
            if abs(end - start) <= 1e-6
            else 0.5 * (start + end)
        )
        label_position = choose_label_position(
            bundle["route_points"],
            bundle["route_stations"],
            label_station,
            (30.0, -30.0, 36.0, -36.0)
            if policy_only and not policy_corridor
            else (12.0, 17.0, -12.0, -17.0, 22.0, -22.0, 27.0, -27.0, 32.0, -32.0),
            occupied_text_xy,
            13.0,
        )
        occupied_text_xy.append(label_position)
        text_namespace = "policy_text" if policy_only else "speed_text" if speed_rule else "mission_text"
        text = _marker(text_namespace, index, Marker.TEXT_VIEW_FACING, stamp)
        text.pose.position = _point(label_position, 2.8 + 0.15 * (index % 4))
        text.scale.z = 1.05
        text.color = color
        text.text = _mission_label(item, start, end)
        markers.append(text)

    origin = choose_label_position(
        bundle["route_points"],
        bundle["route_stations"],
        0.0,
        (18.0, -18.0, 24.0, -24.0),
        occupied_text_xy,
        12.0,
    )
    info = _marker("build_info", 0, Marker.TEXT_VIEW_FACING, stamp)
    info.pose.position = _point(origin, 6.0)
    info.scale.z = 1.35
    info.color = _color(1.0, 0.85, 0.25, 1.0)
    info.text = (
        "%s\ngeometry %s\nP2-B lanes %d | areas %d"
        % (
            product["build_id"], str(bundle["geometry_sha256"])[:16],
            len(product["lanes"]), len(product["drivable_areas"]),
        )
        if product is not None
        else "%s\ngeometry %s\nP1-D read-only | Lane topology pending P2" % (
            bundle["map"]["build"]["build_id"], str(bundle["geometry_sha256"])[:16]
        )
    )
    markers.append(info)
    output = MarkerArray()
    output.markers = markers
    return output


class CompetitionMapPublisher:
    def __init__(self) -> None:
        map_path = Path(rospy.get_param("~map_path"))
        metadata_path = Path(rospy.get_param("~metadata_path"))
        baseline_raw = str(rospy.get_param("~baseline_path", "")).strip()
        data_root_raw = str(rospy.get_param("~data_root", os.environ.get("ASMC_DATA", "/data"))).strip()
        bundle = load_runtime_bundle(
            map_path,
            metadata_path,
            Path(baseline_raw) if baseline_raw else None,
            Path(data_root_raw) if data_root_raw else None,
        )
        self.bundle = bundle
        product_path_raw = str(rospy.get_param("~drivable_product_path", "")).strip()
        self.product = None
        self.occupancy = None
        if product_path_raw:
            product_path = Path(product_path_raw)
            self.product = json.loads(product_path.read_text(encoding="utf-8"))
            product_payload = json.dumps(
                {key: value for key, value in self.product.items() if key != "product_sha256"},
                ensure_ascii=False, sort_keys=True, separators=(",", ":"),
            ).encode("utf-8")
            if hashlib.sha256(product_payload).hexdigest() != self.product.get("product_sha256"):
                raise RuntimeBundleError("compact product self-hash mismatch")
            if self.product.get("reviewed_geometry_sha256") != bundle["geometry_sha256"]:
                raise RuntimeBundleError("compact product/reviewed geometry SHA mismatch")
            grid_path = Path(str(self.product["grid"]["path"]))
            if not grid_path.is_absolute():
                sibling = product_path.parent / grid_path.name
                if sibling.is_file():
                    grid_path = sibling
                else:
                    # Development-space generated paths are repository-relative.
                    package_root = Path(__file__).resolve().parents[1]
                    repo_root = package_root.parents[1]
                    grid_path = repo_root / grid_path
            digest = hashlib.sha256(grid_path.read_bytes()).hexdigest()
            if digest != self.product["grid"].get("sha256"):
                raise RuntimeBundleError("compact occupancy grid SHA mismatch")
            with np.load(str(grid_path), allow_pickle=False) as archive:
                self.occupancy = np.asarray(archive["occupancy"], dtype=np.int8)
        self.meta_pub = rospy.Publisher("/hd_map/meta", HdMapMeta, queue_size=1, latch=True)
        self.markings_pub = rospy.Publisher(
            "/hd_map/global/lane_markings", LaneMarkingArray, queue_size=1, latch=True
        )
        self.checkpoints_pub = rospy.Publisher(
            "/hd_map/global/checkpoints", CompetitionCheckpointArray, queue_size=1, latch=True
        )
        self.missions_pub = rospy.Publisher(
            "/hd_map/global/mission_zones", MissionZoneArray, queue_size=1, latch=True
        )
        self.lanes_pub = None
        self.drivable_pub = None
        self.grid_pub = None
        if self.product is not None:
            self.lanes_pub = rospy.Publisher(
                "/hd_map/global/lanes", LaneArray, queue_size=1, latch=True
            )
            self.drivable_pub = rospy.Publisher(
                "/hd_map/global/drivable_areas", DrivableAreaArray, queue_size=1, latch=True
            )
            self.grid_pub = rospy.Publisher(
                "/hd_map/global/drivable_grid", OccupancyGrid, queue_size=1, latch=True
            )
        # The debug topic follows the P1-B non-latched contract and is
        # periodically republished so late RViz subscribers receive it.
        self.debug_pub = rospy.Publisher(
            "/hd_map/debug/markers", MarkerArray, queue_size=1, latch=False
        )
        stamp = rospy.Time.now()
        messages = build_static_messages(bundle, stamp, self.product, self.occupancy)
        self.meta_pub.publish(messages["meta"])
        self.markings_pub.publish(messages["lane_markings"])
        self.checkpoints_pub.publish(messages["checkpoints"])
        self.missions_pub.publish(messages["mission_zones"])
        if self.product is not None:
            self.lanes_pub.publish(messages["lanes"])
            self.drivable_pub.publish(messages["drivable_areas"])
            self.grid_pub.publish(messages["grid"])
        self.debug_pub.publish(build_debug_markers(bundle, stamp, self.product))
        republish_hz = float(rospy.get_param("~debug_republish_hz", 1.0))
        if republish_hz <= 0.0:
            raise RuntimeBundleError("debug_republish_hz must be positive")
        self.timer = rospy.Timer(rospy.Duration(1.0 / republish_hz), self._publish_debug)
        rospy.loginfo(
            "competition map published: build=%s geometry=%s markings=%d lanes=%d areas=%d "
            "checkpoints=%d missions=%d operational=%s complete=%s",
            messages["meta"].build_id,
            messages["meta"].geometry_sha256,
            len(messages["lane_markings"].markings),
            messages["meta"].lane_count,
            messages["meta"].drivable_area_count,
            len(messages["checkpoints"].checkpoints),
            len(messages["mission_zones"].zones),
            messages["meta"].operational,
            messages["meta"].complete,
        )

    def _publish_debug(self, _event) -> None:
        self.debug_pub.publish(build_debug_markers(
            self.bundle, rospy.Time.now(), self.product
        ))


def main() -> None:
    rospy.init_node("competition_map_publisher")
    try:
        CompetitionMapPublisher()
    except (RuntimeBundleError, KeyError, TypeError, ValueError) as error:
        rospy.logfatal("P1-D map publisher refused inconsistent input: %s", error)
        raise
    rospy.spin()


if __name__ == "__main__":
    main()
