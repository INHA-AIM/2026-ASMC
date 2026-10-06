#!/usr/bin/env python3
"""Development smoke check for phase-2 planner-facing local HD-map crops."""

from __future__ import annotations

import math
import threading
import sys
import time

from asmc_msgs.msg import HdMapMeta, LaneArray, LaneAssociation
from morai_msgs.msg import EgoVehicleStatus
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rospy

from asmc_hd_map.local_crop import (
    crop_occupancy_grid,
    EgoWindow,
    crop_ego_occupancy_grid,
    select_ego_bounds,
    geometry_bounds,
    select_intersecting_bounds,
)


def _require(condition, message):
    if not condition:
        raise RuntimeError(message)


def _lane_bounds(lane):
    return geometry_bounds((
        [[point.x, point.y] for point in lane.polygon],
        [[point.x, point.y] for point in lane.centerline],
    ))


def main() -> int:
    rospy.init_node("verify_local_map_topics", anonymous=True)
    timeout = float(rospy.get_param("~timeout_s", 30.0))
    mode = str(rospy.get_param("~crop_mode", "ego"))
    window = EgoWindow(**{key: float(rospy.get_param("~" + key, value)) for key, value in
                          dict(front_m=90., rear_m=30., left_m=30., right_m=30.).items()})
    extent_m = float(rospy.get_param("~extent_m", 120.0))
    test_x = float(rospy.get_param("~test_x", -131.68979755061446))
    test_y = float(rospy.get_param("~test_y", -428.3310229377821))
    ego_topic = str(rospy.get_param("~ego_topic", "/Ego_topic"))

    published = dict(rospy.get_published_topics())
    _require(
        published.get("/hd_map/local/lanes") == "asmc_msgs/LaneArray",
        "local lane topic is missing or has the wrong type",
    )
    _require(
        published.get("/hd_map/local/drivable_grid") == "nav_msgs/OccupancyGrid",
        "local grid topic is missing or has the wrong type",
    )
    _require(
        published.get("/hd_map/local/current_lane") == "asmc_msgs/LaneAssociation",
        "current-lane topic is missing or has the wrong type",
    )

    meta = rospy.wait_for_message("/hd_map/meta", HdMapMeta, timeout=timeout)
    global_lanes = rospy.wait_for_message(
        "/hd_map/global/lanes", LaneArray, timeout=timeout
    )
    global_grid = rospy.wait_for_message(
        "/hd_map/global/drivable_grid", OccupancyGrid, timeout=timeout
    )

    received = {}
    ready = threading.Event()

    def callback(message, key):
        received[key] = message
        if set(received) == {"lanes", "grid", "association"}:
            ready.set()

    subscribers = [
        rospy.Subscriber(
            "/hd_map/local/lanes", LaneArray, callback,
            callback_args="lanes", queue_size=1,
        ),
        rospy.Subscriber(
            "/hd_map/local/drivable_grid", OccupancyGrid, callback,
            callback_args="grid", queue_size=1,
        ),
        rospy.Subscriber(
            "/hd_map/local/current_lane", LaneAssociation, callback,
            callback_args="association", queue_size=1,
        ),
    ]
    publisher = rospy.Publisher(ego_topic, EgoVehicleStatus, queue_size=1)
    deadline = time.monotonic() + timeout
    while publisher.get_num_connections() == 0 and time.monotonic() < deadline:
        rospy.sleep(0.05)
    _require(publisher.get_num_connections() > 0, "local crop node did not subscribe to Ego pose")

    pose = EgoVehicleStatus()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "map"
    pose.position.x = test_x
    pose.position.y = test_y
    pose.heading = 61.298126221
    publisher.publish(pose)
    _require(ready.wait(timeout), "timed out waiting for both local HD-map topics")

    local_lanes = received["lanes"]
    local_grid = received["grid"]
    association = received["association"]
    _require(not local_lanes.complete, "local LaneArray must use complete=false")
    _require(local_lanes.header.frame_id == "map", "local lanes left the map frame")
    _require(local_grid.header.frame_id == "map", "local grid left the map frame")
    _require(local_lanes.build_id == meta.build_id, "local/global build ID mismatch")
    _require(association.build_id == meta.build_id, "association/global build ID mismatch")
    _require(
        association.reference_type == LaneAssociation.REFERENCE_BASE_LINK_REAR_AXLE,
        "current-lane association uses the wrong vehicle reference point",
    )
    _require(association.header.frame_id == "map", "current-lane reference left map frame")
    _require(local_lanes.map_id == meta.map_id, "local/global map ID mismatch")
    _require(
        local_lanes.geometry_sha256 == meta.geometry_sha256,
        "local/global geometry SHA mismatch",
    )

    bounds = [_lane_bounds(lane) for lane in global_lanes.lanes]
    yaw = math.radians(pose.heading)
    indices = (select_ego_bounds(bounds, (test_x, test_y), yaw, window) if mode == "ego"
               else select_intersecting_bounds(bounds, (test_x, test_y), extent_m))
    expected_lane_ids = [global_lanes.lanes[index].id for index in indices]
    _require(
        [lane.id for lane in local_lanes.lanes] == expected_lane_ids,
        "local lane crop differs from the deterministic global selection",
    )
    _require(0 < len(local_lanes.lanes) < len(global_lanes.lanes), "lane crop is not local")

    global_occupancy = np.asarray(global_grid.data, dtype=np.int8).reshape(
        int(global_grid.info.height), int(global_grid.info.width)
    )
    crop_args = (global_occupancy, global_grid.info.resolution,
                 (global_grid.info.origin.position.x, global_grid.info.origin.position.y),
                 (test_x, test_y))
    expected_grid = (crop_ego_occupancy_grid(*crop_args, yaw, window) if mode == "ego"
                     else crop_occupancy_grid(*crop_args, extent_m))
    expected_yaw = yaw if mode == "ego" else 0.
    _require(abs(local_grid.info.origin.orientation.z - math.sin(expected_yaw / 2)) < 1e-8,
             "local grid orientation is incorrect")
    _require(abs(local_grid.info.origin.orientation.w - math.cos(expected_yaw / 2)) < 1e-8,
             "local grid orientation is incorrect")
    actual_grid = np.asarray(local_grid.data, dtype=np.int8).reshape(
        int(local_grid.info.height), int(local_grid.info.width)
    )
    _require(
        actual_grid.shape == expected_grid.data.shape,
        "local grid dimensions differ from deterministic crop",
    )
    _require(np.array_equal(actual_grid, expected_grid.data), "local/global grid parity failed")
    _require(
        abs(local_grid.info.origin.position.x - expected_grid.origin_x) < 1e-9
        and abs(local_grid.info.origin.position.y - expected_grid.origin_y) < 1e-9,
        "local grid map origin differs from the requested window",
    )
    _require(set(actual_grid.reshape(-1)).issubset({-1, 0, 100}), "invalid occupancy value")

    print(
        "phase-2 local topics valid: build=%s lanes=%d grid=%dx%d@%.2fm origin=(%.1f,%.1f)"
        % (
            meta.build_id,
            len(local_lanes.lanes),
            local_grid.info.width,
            local_grid.info.height,
            local_grid.info.resolution,
            local_grid.info.origin.position.x,
            local_grid.info.origin.position.y,
        )
    )
    # Keep references alive until validation has completed.
    _ = subscribers
    return 0


if __name__ == "__main__":
    try:
        sys_exit = main()
    except Exception as error:
        print("phase-2 local topic verification failed: %s" % error, file=sys.stderr)
        sys_exit = 1
    raise SystemExit(sys_exit)
