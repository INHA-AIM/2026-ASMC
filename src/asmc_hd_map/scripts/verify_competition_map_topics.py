#!/usr/bin/env python3
"""Runtime smoke check for the integrated phase-2 ROS topics."""

from collections import Counter
import cv2
import json
import math
import numpy as np
from pathlib import Path
import sys

from asmc_msgs.msg import (
    CompetitionCheckpointArray,
    HdMapMeta,
    DrivableAreaArray,
    LaneArray,
    LaneAssociation,
    LaneMarking,
    LaneMarkingArray,
    MissionZone,
    MissionZoneArray,
)
from nav_msgs.msg import OccupancyGrid
import rospy
from visualization_msgs.msg import Marker, MarkerArray


EXPECTED_TYPES = {
    "/hd_map/meta": "asmc_msgs/HdMapMeta",
    "/hd_map/global/lane_markings": "asmc_msgs/LaneMarkingArray",
    "/hd_map/global/checkpoints": "asmc_msgs/CompetitionCheckpointArray",
    "/hd_map/global/mission_zones": "asmc_msgs/MissionZoneArray",
    "/hd_map/global/lanes": "asmc_msgs/LaneArray",
    "/hd_map/global/drivable_areas": "asmc_msgs/DrivableAreaArray",
    "/hd_map/global/drivable_grid": "nav_msgs/OccupancyGrid",
    "/hd_map/local/lanes": "asmc_msgs/LaneArray",
    "/hd_map/local/drivable_grid": "nav_msgs/OccupancyGrid",
    "/hd_map/local/current_lane": "asmc_msgs/LaneAssociation",
    "/hd_map/debug/markers": "visualization_msgs/MarkerArray",
}


def _require(condition, message):
    if not condition:
        raise RuntimeError(message)


def _unique_nonempty(values, label):
    _require(all(values), "%s contains an empty ID" % label)
    _require(len(values) == len(set(values)), "%s IDs are not unique" % label)


def _finite_points(points, minimum, label):
    _require(len(points) >= minimum, "%s has too few points" % label)
    _require(
        all(math.isfinite(point.x) and math.isfinite(point.y) and math.isfinite(point.z)
            for point in points),
        "%s contains a non-finite point" % label,
    )


def _rasterized_area_union(values, resolution=0.2):
    polygons = [
        np.asarray([[point.x, point.y] for point in value.polygon], dtype=np.float64)
        for value in values
    ]
    _require(polygons, "surface area selection is empty")
    points = np.vstack(polygons)
    origin = points.min(axis=0) - 1.0
    extent = points.max(axis=0) - origin + 1.0
    shape = tuple(np.ceil(extent[::-1] / resolution).astype(int))
    mask = np.zeros(shape, dtype=np.uint8)
    for polygon in polygons:
        pixels = np.rint((polygon - origin[None, :]) / resolution).astype(np.int32)
        cv2.fillPoly(mask, [pixels.reshape((-1, 1, 2))], 1, cv2.LINE_8)
    return mask


def main() -> int:
    rospy.init_node("verify_competition_map_topics", anonymous=True)
    timeout = float(rospy.get_param("~timeout_s", 10.0))
    published = dict(rospy.get_published_topics())
    for topic, expected_type in EXPECTED_TYPES.items():
        _require(published.get(topic) == expected_type, "%s type is %s, expected %s" % (
            topic, published.get(topic), expected_type
        ))

    meta = rospy.wait_for_message("/hd_map/meta", HdMapMeta, timeout=timeout)
    markings = rospy.wait_for_message(
        "/hd_map/global/lane_markings", LaneMarkingArray, timeout=timeout
    )
    checkpoints = rospy.wait_for_message(
        "/hd_map/global/checkpoints", CompetitionCheckpointArray, timeout=timeout
    )
    missions = rospy.wait_for_message(
        "/hd_map/global/mission_zones", MissionZoneArray, timeout=timeout
    )
    lanes = rospy.wait_for_message("/hd_map/global/lanes", LaneArray, timeout=timeout)
    areas = rospy.wait_for_message(
        "/hd_map/global/drivable_areas", DrivableAreaArray, timeout=timeout
    )
    grid = rospy.wait_for_message(
        "/hd_map/global/drivable_grid", OccupancyGrid, timeout=timeout
    )
    debug = rospy.wait_for_message("/hd_map/debug/markers", MarkerArray, timeout=timeout)

    for name, message in (
        ("meta", meta),
        ("lane markings", markings),
        ("checkpoints", checkpoints),
        ("mission zones", missions),
        ("lanes", lanes),
        ("drivable areas", areas),
        ("drivable grid", grid),
    ):
        _require(message.header.frame_id == "map", "%s left the map frame" % name)

    _require(meta.geometry_sha256 == "01ec671fe402baf339f8f6bcfb83a97213d560f7311961c5f072a47d654bcfbf", "geometry SHA drift")
    _require(meta.build_id == "kcity2025_static_junction_widened_drivable_v6", "P2-B v6 build ID drift")
    _require(meta.operational and meta.complete, "P2-B map must be operational and complete")
    _require(meta.lane_count == 70 and meta.drivable_area_count == 102, "P2-B v6 product count drift")
    _require(
        "compact_drivable_product" in meta.source_ids and "legacy_drivable_h5" in meta.source_ids,
        "P2-B provenance is missing from HdMapMeta",
    )
    _require(meta.lane_marking_count == 128, "metadata marking count must include 121 lanes + 7 stoplines")
    _require(markings.complete and len(markings.markings) == 128, "global marking array is incomplete")
    _require(markings.build_id == meta.build_id and markings.map_id == meta.map_id,
             "marking/meta identity mismatch")
    _require(markings.geometry_sha256 == meta.geometry_sha256, "marking/meta geometry SHA mismatch")
    _require(lanes.complete and len(lanes.lanes) == 70, "global lane product is incomplete")
    _require(areas.complete and len(areas.areas) == 102, "global drivable vector product is incomplete")
    _require(lanes.build_id == meta.build_id and lanes.map_id == meta.map_id,
             "lane/meta identity mismatch")
    _require(areas.build_id == meta.build_id and areas.map_id == meta.map_id,
             "drivable/meta identity mismatch")
    _require(checkpoints.build_id == meta.build_id and missions.build_id == meta.build_id,
             "metadata array build identity mismatch")
    _require(checkpoints.route_id == missions.route_id and checkpoints.route_id,
             "checkpoint/mission route identity mismatch")
    _require(lanes.geometry_sha256 == meta.geometry_sha256, "lane/meta geometry SHA mismatch")
    _require(areas.geometry_sha256 == meta.geometry_sha256, "drivable/meta geometry SHA mismatch")
    _require(abs(grid.info.resolution - 0.2) < 1e-6, "drivable grid resolution drift")
    _require(grid.info.width == 3085 and grid.info.height == 7232, "drivable grid dimensions drift")
    _require(len(grid.data) == grid.info.width * grid.info.height, "drivable grid payload size mismatch")
    _require(set(grid.data) == {0, 100}, "drivable grid must use only free=0/occupied=100")
    _unique_nonempty([value.id for value in markings.markings], "LaneMarking")
    _unique_nonempty([value.id for value in lanes.lanes], "Lane")
    _unique_nonempty([value.id for value in areas.areas], "DrivableArea")
    _unique_nonempty([value.id for value in checkpoints.checkpoints], "Checkpoint")
    _unique_nonempty([value.id for value in missions.zones], "MissionZone")
    for value in markings.markings:
        _finite_points(value.polyline, 2, "marking %s" % value.id)
        _require(value.color != LaneMarking.COLOR_UNKNOWN, "marking color is unknown: %s" % value.id)
        _require(value.pattern != LaneMarking.PATTERN_UNKNOWN, "marking pattern is unknown: %s" % value.id)
        _require(value.physical_type != LaneMarking.PHYSICAL_UNKNOWN,
                 "marking physical type is unknown: %s" % value.id)
        _require(value.source_type != LaneMarking.SOURCE_UNKNOWN,
                 "marking source is unknown: %s" % value.id)
        _require(value.width_m > 0.0 and value.source_dataset and value.source_feature_id,
                 "marking width/provenance is incomplete: %s" % value.id)
    lane_by_link = {value.link_id: value for value in lanes.lanes}
    _require(len(lane_by_link) == 70, "lane link IDs are not unique")
    _require("A2256W000427" not in lane_by_link, "unused toll lane A427 must remain excluded")
    _require("A2256W000179" not in lane_by_link, "exit-only lane A179 must remain excluded")
    excluded_bus_links = {
        "A2256W000429", "A2256W000433", "A2256W000421",
        "A2256W000425", "A2256W000424",
    }
    _require(
        not (excluded_bus_links & set(lane_by_link)),
        "bus-only high-speed lane leaked into the competition lane product",
    )
    expected_high_speed_lanes = {
        "A2256W000418": 2,
        "A2256W000410": 3,
        "A2256W000409": 4,
        "A2256W000411": 5,
    }
    _require(
        {
            link_id: int(lane_by_link[link_id].lane_number)
            for link_id in expected_high_speed_lanes
        } == expected_high_speed_lanes,
        "high-speed traversable lane numbers must remain 2/3/4 plus merge lane 5",
    )
    _require("A2256W000131" in lane_by_link, "shadow-zone opposite lane A131 is missing")
    _require("A2256W000432" in lane_by_link, "usable toll lane A432 is missing")
    expected_extra_intersections = {
        "A2256W000066", "A2256W000087", "A2256W000100", "A2256W000111",
        "A2256W000206", "A2256W000211", "A2256W000221", "A2256W000226",
        "A2256W000227", "A2256W000319", "A2256W000636", "A2256W000746",
        "A2256W001124", "A2256W001125", "A2256W000134",
    }
    route_links = {
        "A2256W000748", "A2256W000182", "A2256W000329", "A2256W000236",
        "A2256W000219", "A2256W000215", "A2256W000213", "A2256W000222",
        "A2256W000220", "A2256W000202", "A2256W000728", "A2256W000599",
        "A2256W000207", "A2256W000205", "A2256W000315", "A2256W000318",
        "A2256W000308", "A2256W000083", "A2256W000304", "A2256W000148",
        "A2256W000146", "A2256W000151", "A2256W000866", "A2256W000054",
        "A2256W000846", "A2256W000411", "A2256W000420", "A2256W000408",
        "A2256W000445", "A2256W000153", "A2256W000451", "A2256W000446",
        "A2256W000448", "A2256W000126", "A2256W000128", "A2256W000333",
        "A2256W000154", "A2256W000332", "A2256W000751",
    }
    # Runtime derives this from lane_type rather than carrying the build-only
    # group field in the frozen Lane message.
    actual_extra_intersections = {
        value.link_id for value in lanes.lanes
        if value.lane_type == value.TYPE_INTERSECTION and value.link_id not in route_links
    }
    _require(
        actual_extra_intersections == expected_extra_intersections,
        "non-route intersection allowlist drift",
    )
    lane_ids = {value.id for value in lanes.lanes}
    for value in lanes.lanes:
        _finite_points(value.centerline, 2, "lane centerline %s" % value.id)
        _finite_points(value.polygon, 3, "lane polygon %s" % value.id)
        _require(value.lane_type != value.TYPE_UNKNOWN, "lane type is unknown: %s" % value.id)
        _require(value.direction != value.DIRECTION_UNKNOWN, "lane direction is unknown: %s" % value.id)
        _require(value.lane_number > 0 and value.lane_number_scope_id,
                 "lane number/scope is incomplete: %s" % value.id)
        _require(value.source_type != value.SOURCE_UNKNOWN
                 and value.source_dataset and value.source_feature_id,
                 "lane provenance is incomplete: %s" % value.id)
        references = (
            [value.left_adjacent_lane_id] if value.left_adjacent_lane_id else []
        ) + (
            [value.right_adjacent_lane_id] if value.right_adjacent_lane_id else []
        ) + list(value.predecessor_lane_ids) + list(value.successor_lane_ids)
        _require(set(references).issubset(lane_ids),
                 "lane topology points outside the published set: %s" % value.id)

    def flat_caps_on_endpoint_normals(value):
        if len(value.polygon) % 2 != 0 or len(value.centerline) < 2:
            return False
        start_dx = value.centerline[1].x - value.centerline[0].x
        start_dy = value.centerline[1].y - value.centerline[0].y
        end_dx = value.centerline[-1].x - value.centerline[-2].x
        end_dy = value.centerline[-1].y - value.centerline[-2].y
        start_norm = max(math.hypot(start_dx, start_dy), 1e-9)
        end_norm = max(math.hypot(end_dx, end_dy), 1e-9)
        start_tangent = (start_dx / start_norm, start_dy / start_norm)
        end_tangent = (end_dx / end_norm, end_dy / end_norm)
        half = len(value.polygon) // 2
        start_cap = (value.polygon[0], value.polygon[-1])
        end_cap = (value.polygon[half - 1], value.polygon[half])
        # The polygon cap uses the builder's 0.25 m local tangent while the
        # frozen Lane message exposes the coarser MGeo centerline samples.
        # Curved endpoints therefore differ by up to 6.1 cm even though both
        # cap vertices share the exact builder node-normal cross-section.
        # Keep the tolerance below half of the 0.2 m occupancy-grid cell.
        cap_projection_tolerance_m = 0.075
        return all(
            abs(
                (point.x - center.x) * tangent[0]
                + (point.y - center.y) * tangent[1]
            ) < cap_projection_tolerance_m
            for points, center, tangent in (
                (start_cap, value.centerline[0], start_tangent),
                (end_cap, value.centerline[-1], end_tangent),
            )
            for point in points
        )

    _require(
        all(flat_caps_on_endpoint_normals(value) for value in lanes.lanes),
        "lane polygons must terminate with flat caps on endpoint-node normals",
    )
    dynamic_ids = {
        "A2256W000066", "A2256W000087", "A2256W000206", "A2256W000211",
        "A2256W000599", "A2256W000636", "A2256W001124", "A2256W001125",
    }
    dynamic_areas = [
        value
        for value in areas.areas
        if value.link_ids and set(value.link_ids).issubset(dynamic_ids)
    ]
    _require(len(dynamic_areas) == 12, "dynamic surface must contain eight lanes and four patches")
    dynamic_mask = _rasterized_area_union(dynamic_areas)
    component_count, _ = cv2.connectedComponents(dynamic_mask, connectivity=8)
    _require(component_count - 1 == 1, "dynamic junction drivable union is disconnected")
    static_ids = {
        "A2256W000100", "A2256W000111", "A2256W000221", "A2256W000222",
        "A2256W000226", "A2256W000227", "A2256W000746",
    }
    static_areas = [
        value for value in areas.areas
        if value.link_ids and set(value.link_ids).issubset(static_ids)
    ]
    _require(len(static_areas) == 14, "static surface must contain lanes, bridge, and patches")
    for name, mask in (("static", _rasterized_area_union(static_areas)), ("dynamic", dynamic_mask)):
        component_count, _ = cv2.connectedComponents(mask, connectivity=8)
        _require(component_count - 1 == 1, "%s obstacle surface is disconnected" % name)
        _, hierarchy = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        hole_count = 0 if hierarchy is None else sum(value[3] >= 0 for value in hierarchy[0])
        _require(hole_count == 0, "%s obstacle surface still contains holes" % name)
    roundabout_areas = [
        value for value in areas.areas
        if (
            set(value.link_ids) == {"A2256W000134"}
            or value.source_feature_id.startswith("surface:roundabout_maneuver:")
        )
    ]
    _require(
        len(roundabout_areas) == 5,
        "A134/A151 roundabout stitch must contain one lane and four H5 patches",
    )
    roundabout_mask = _rasterized_area_union(roundabout_areas)
    component_count, _ = cv2.connectedComponents(roundabout_mask, connectivity=8)
    _require(component_count - 1 == 1, "A134 roundabout surface is disconnected")

    def grid_value(x_m, y_m):
        x = int(round((x_m - grid.info.origin.position.x) / grid.info.resolution))
        y = int(round((y_m - grid.info.origin.position.y) / grid.info.resolution))
        return grid.data[y * grid.info.width + x]

    _require(grid_value(-102.0, 340.0) == 100, "roundabout center island became drivable")
    _require(grid_value(-102.97, 325.54) == 0, "A134 centerline is not drivable")
    _require(grid_value(-54.38, -131.38) == 0 and grid_value(-55.07, -149.22) == 0,
             "A222 junction inner-corner pavement is not drivable")
    _require(grid_value(-53.72, -130.50) == 100 and grid_value(-54.21, -150.27) == 100,
             "A222 junction widening crossed the reviewed curb margin")
    source_counts = Counter(value.source_dataset for value in areas.areas)
    _require(
        source_counts["selected_lane_topology_corridor_fill"] == 18
        and source_counts["legacy_h5_clipped_to_selected_link_envelope"] == 14,
        "surface fill source counts drift",
    )
    _require(
        any(
            value.source_feature_id == "gps_shadow_A126_A131"
            and set(value.link_ids) == {"A2256W000126", "A2256W000131"}
            for value in areas.areas
        ),
        "shadow-zone A126/A131 surface bridge is missing",
    )
    marking_ids = {value.id for value in markings.markings}
    _require(
        all(
            (not value.left_boundary_id or value.left_boundary_id in marking_ids)
            and (not value.right_boundary_id or value.right_boundary_id in marking_ids)
            for value in lanes.lanes
        ),
        "lane boundary references must point to reviewed LaneMarking IDs",
    )
    for value in areas.areas:
        _finite_points(value.polygon, 3, "drivable polygon %s" % value.id)
        _require(value.area_type != value.TYPE_UNKNOWN,
                 "drivable area type is unknown: %s" % value.id)
        _require(value.validity == value.VALIDITY_VALIDATED,
                 "runtime drivable area is not validated: %s" % value.id)
        _require(value.source_type != value.SOURCE_UNKNOWN
                 and value.source_dataset and value.source_feature_id,
                 "drivable provenance is incomplete: %s" % value.id)
        _require(value.lane_ids and set(value.lane_ids).issubset(lane_ids),
                 "drivable area has invalid lane references: %s" % value.id)
        _require(value.link_ids and set(value.link_ids).issubset(lane_by_link),
                 "drivable area has invalid link references: %s" % value.id)
    role_counts = Counter(value.role for value in markings.markings)
    color_counts = Counter(value.color for value in markings.markings)
    pattern_counts = Counter(value.pattern for value in markings.markings)
    _require(role_counts[LaneMarking.ROLE_STOP_LINE] == 7, "expected seven stoplines")
    _require(color_counts[LaneMarking.COLOR_WHITE] == 101, "expected 94 white lanes + 7 stoplines")
    _require(color_counts[LaneMarking.COLOR_YELLOW] == 27, "expected 27 yellow markings")
    _require(pattern_counts[LaneMarking.PATTERN_SOLID] == 97, "expected 90 solid lanes + 7 stoplines")
    _require(pattern_counts[LaneMarking.PATTERN_BROKEN] == 31, "expected 31 broken markings")

    _require(len(checkpoints.checkpoints) == 15, "expected 15 checkpoints (rule book v1.2)")
    _require(
        [value.sequence for value in checkpoints.checkpoints] == list(range(15)),
        "checkpoint sequence must be contiguous 0..14",
    )
    _require(
        all(value.link_id in lane_by_link and value.source_reference
            and 0.0 <= value.route_station_m <= checkpoints.route_length_m
            for value in checkpoints.checkpoints),
        "checkpoint link/station/provenance is incomplete",
    )
    _require(all(abs(value.pass_radius_m - 3.0) < 1e-9 for value in checkpoints.checkpoints), "checkpoint radius drift")
    _require(abs(checkpoints.route_length_m - 2184.611723336) < 1e-6, "route length drift")
    _require(len(missions.zones) == 15, "expected 15 mission zones")
    _require(
        all(value.mission_type != MissionZone.TYPE_UNKNOWN
            and value.status != MissionZone.STATUS_UNSPECIFIED
            and value.source_reference and value.description
            and 0.0 <= value.route_station_start_m <= value.route_station_end_m
            <= checkpoints.route_length_m
            and value.start_link_id in lane_by_link and value.end_link_id in lane_by_link
            and value.link_ids and set(value.link_ids).issubset(lane_by_link)
            for value in missions.zones),
        "mission type/status/range/link/provenance is incomplete",
    )
    status_counts = Counter(value.status for value in missions.zones)
    _require(status_counts[MissionZone.STATUS_OFFICIAL] == 0, "official mission count drift")
    # 4 derived-official rules plus 11 team-final zones, which the frozen
    # message carries as DERIVED_OFFICIAL with rule_payload.authority=team_final.
    _require(status_counts[MissionZone.STATUS_DERIVED_OFFICIAL] == 15, "derived mission count drift")
    _require(status_counts[MissionZone.STATUS_PROVISIONAL] == 0, "provisional mission count drift")
    mission_by_id = {value.id: value for value in missions.zones}
    team_final = [
        value.id for value in missions.zones
        if json.loads(value.rule_payload_json).get("authority") == "team_final"
    ]
    _require(len(team_final) == 11, "team-final mission authority count drift")
    shadow_payload = json.loads(mission_by_id["gps_shadow_candidate"].rule_payload_json)
    _require(
        shadow_payload.get("usable_opposite_lane_link_ids") == ["A2256W000131"]
        and shadow_payload.get("drivable_area_bridge_id") == "gps_shadow_A126_A131",
        "GPS-shadow policy must expose A131 and its A126 bridge",
    )
    _require(
        mission_by_id["static_obstacle_candidate"].link_ids == ["A2256W000222"],
        "static obstacle bounds drift",
    )
    _require(
        mission_by_id["dynamic_obstacle_candidate"].link_ids
        == ["A2256W000728", "A2256W000599", "A2256W000207", "A2256W000205"],
        "dynamic obstacle bounds drift",
    )
    _require("highway_merge_candidate" not in mission_by_id, "A054/A846 must not be a mission zone")
    monitoring_policy = mission_by_id["traffic_light_activation_A236_to_A308"]
    monitoring_payload = json.loads(monitoring_policy.rule_payload_json)
    _require(
        monitoring_policy.start_link_id == "A2256W000236"
        and monitoring_policy.end_link_id == "A2256W000308"
        and monitoring_payload.get("spatial_mission") is False
        and monitoring_payload.get("runtime_activation_hint") == "enable_within_zone_only"
        and monitoring_payload.get("disable_outside_zone") is True,
        "traffic-light activation policy must be non-spatial A236 through A308",
    )
    traffic_boundaries = [
        value
        for value in missions.zones
        if value.mission_type == MissionZone.TYPE_TRAFFIC_LIGHT
        and abs(value.route_station_end_m - value.route_station_start_m) < 1e-6
    ]
    _require(
        len(traffic_boundaries) == 5
        and all(
            monitoring_policy.route_station_start_m
            <= value.route_station_start_m
            <= monitoring_policy.route_station_end_m
            for value in traffic_boundaries
        ),
        "traffic-light boundaries must stay inside the activation policy",
    )
    speed_profile = sorted(
        (
            value
            for value in missions.zones
            if value.mission_type
            in (MissionZone.TYPE_SPEED_LIMIT, MissionZone.TYPE_HIGH_SPEED_EXCEPTION)
        ),
        key=lambda value: value.route_station_start_m,
    )
    _require(
        [value.id for value in speed_profile]
        == [
            "speed_limit_60_before_high_speed",
            "high_speed_exception",
            "speed_limit_60_after_high_speed",
        ],
        "effective speed profile IDs/order drift",
    )
    _require(
        [value.speed_limit_kph if value.has_speed_limit else None for value in speed_profile]
        == [60.0, None, 60.0],
        "effective speed profile values drift",
    )
    _require(
        abs(speed_profile[0].route_station_start_m) < 1e-6
        and abs(speed_profile[-1].route_station_end_m - checkpoints.route_length_m) < 1e-6
        and all(
            abs(left.route_station_end_m - right.route_station_start_m) < 1e-6
            for left, right in zip(speed_profile, speed_profile[1:])
        ),
        "effective speed profile does not cover the route exactly once",
    )
    detail = str(rospy.get_param("/competition_map_publisher/debug_marker_detail", "lite"))
    _require(len(debug.markers) > 50, "debug marker set is unexpectedly small")
    namespaces = {value.ns for value in debug.markers}
    review_layers = (
        "route",
        "route_link_direction",
        "compact_lane_centerlines",
        "compact_lane_polygons",
        "compact_guarded_junctions",
        "drivable_group_static_obstacle_maneuver",
        "drivable_group_dynamic_obstacle_maneuver",
        "drivable_group_route_only_connectors",
        "drivable_group_non_mission_route_turns",
        "drivable_group_gps_shadow_opposite_lane",
        "compact_corridor_fills",
        "compact_intersection_surface_patches",
    )
    required = [
        "checkpoints_radius",
        "speed_limit_corridors",
        "speed_limit_exceptions",
        "perception_activation_corridors",
        "policy_text",
        "build_info",
    ]
    if detail == "full":
        required.extend(review_layers)
    else:
        required.append("drivable_area_outline")
        _require(
            not namespaces.intersection(review_layers)
            and "route_link_labels" not in namespaces
            and not any(namespace.startswith("mission_actors_") for namespace in namespaces),
            "lite debug markers must leave out the per-lane, per-link and actor review layers",
        )
    for namespace in required:
        _require(namespace in namespaces, "missing debug namespace: %s" % namespace)
    _require(
        sum(value.ns == "speed_limit_corridors" for value in debug.markers) == 2,
        "expected two 60 km/h speed corridors",
    )
    _require(
        sum(value.ns == "speed_limit_exceptions" for value in debug.markers) == 1,
        "expected one no-competition-cap corridor",
    )
    _require(
        sum(value.ns == "perception_activation_corridors" for value in debug.markers) == 1,
        "expected one traffic-light perception activation corridor",
    )
    policy_index = next(
        index
        for index, value in enumerate(missions.zones)
        if value.id == "traffic_light_activation_A236_to_A308"
    )
    _require(
        not any(
            value.ns == "mission_traffic_light" and value.id == policy_index
            for value in debug.markers
        ),
        "traffic-light activation policy must not paint a mission corridor",
    )
    # Mission corridors are drawn per mission type. Expected counts come from
    # the received zones and the current scenario spec, so editing zones or
    # actor links does not require editing this verifier.
    type_names = {
        getattr(MissionZone, name): name[len("TYPE_"):].lower()
        for name in dir(MissionZone) if name.startswith("TYPE_")
    }
    expected_corridors = Counter(
        type_names[value.mission_type]
        for value in missions.zones
        if value.route_station_end_m - value.route_station_start_m > 1e-6
        and value.mission_type not in (MissionZone.TYPE_SPEED_LIMIT, MissionZone.TYPE_HIGH_SPEED_EXCEPTION)
        and json.loads(value.rule_payload_json).get("spatial_mission") is not False
    )
    for mission_type, count in expected_corridors.items():
        _require(
            sum(value.ns == "mission_%s" % mission_type for value in debug.markers) == count,
            "mission_%s corridor count differs from the published zones" % mission_type,
        )
    import rospkg
    from asmc_hd_map.mission_scenario import build_mission_scenario_spec

    repo_root = Path(rospkg.RosPack().get_path("asmc_hd_map")).resolve().parents[1]
    scenario = build_mission_scenario_spec(repo_root)
    for zone in scenario["zones"]:
        if zone["surface"]:
            _require(
                "mission_surface_%s" % zone["mission_type"] in namespaces,
                "zone %s surface outline is missing" % zone["id"],
            )
        if zone["actor_sources"] and detail == "full":
            _require(
                sum(value.ns == "mission_actors_%s" % zone["mission_type"]
                    and value.type == Marker.LINE_STRIP for value in debug.markers)
                >= len(zone["actor_sources"]),
                "zone %s actor links are missing" % zone["id"],
            )
    text_markers = [value for value in debug.markers if value.type == Marker.TEXT_VIEW_FACING]
    _require(
        detail != "full"
        or sum(value.ns == "route_link_labels" for value in text_markers) == 39,
        "every route link needs one compact debug label",
    )
    _require(
        sum(value.ns == "checkpoints_text" for value in text_markers) == 15,
        "every checkpoint needs one debug label",
    )
    _require(
        all("A2256W" not in value.text for value in text_markers),
        "debug labels must use compact link IDs",
    )
    text_distances = [
        math.hypot(
            left.pose.position.x - right.pose.position.x,
            left.pose.position.y - right.pose.position.y,
        )
        for index, left in enumerate(text_markers)
        for right in text_markers[index + 1 :]
    ]
    _require(
        text_distances and min(text_distances) >= 9.0,
        "debug text labels overlap or are unexpectedly close",
    )

    print(
        "P2-B topics valid: build=%s geometry=%s markings=%d lanes=%d areas=%d "
        "grid=%dx%d checkpoints=%d missions=%d markers=%d"
        % (
            meta.build_id,
            meta.geometry_sha256,
            len(markings.markings),
            len(lanes.lanes),
            len(areas.areas),
            grid.info.width,
            grid.info.height,
            len(checkpoints.checkpoints),
            len(missions.zones),
            len(debug.markers),
        )
    )
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:
        print("P2-B topic verification failed: %s" % error, file=sys.stderr)
        sys.exit(1)
