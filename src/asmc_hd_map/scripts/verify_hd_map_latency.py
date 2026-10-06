#!/usr/bin/env python3
"""Development-only synthetic Ego measurement on an isolated ego_status launch.

Requires a fresh test ROS master, not a vehicle/simulator master. Publishes
100 poses at 20 Hz, checks stale rejection and reports first-delivery latency.
"""
import math
import sys
import threading
import time

import numpy as np
import rospy
from asmc_msgs.msg import HdMapMeta, LaneArray, LaneAssociation, LaneMarkingArray, MissionZoneArray
from morai_msgs.msg import EgoVehicleStatus
from nav_msgs.msg import OccupancyGrid

from asmc_hd_map.semantic_query import SemanticMapIndex


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def main():
    rospy.init_node("verify_hd_map_latency", anonymous=True)
    require(rospy.get_param("/local_map_crop/pose_source", "") == "ego_status",
            "run a separate test master with local_pose_source:=ego_status")
    require(rospy.get_param("/local_map_crop/crop_mode", "ego") == "ego",
            "this measurement checks the default ego crop")
    layers = {}
    meta = rospy.wait_for_message("/hd_map/meta", HdMapMeta, timeout=10)
    for name, topic, kind in (
        ("missions", "mission_zones", MissionZoneArray),
        ("markings", "lane_markings", LaneMarkingArray), ("lanes", "lanes", LaneArray),
    ):
        layers[name] = rospy.wait_for_message("/hd_map/global/" + topic, kind, timeout=10)
    index = SemanticMapIndex(meta, layers)
    samples = {"lane": [], "grid": []}
    seen = {"lane": set(), "grid": set()}
    published_stamps = set()
    last_lane = None
    grid_count = 0
    lock = threading.Lock()

    def callback(message, key):
        nonlocal last_lane, grid_count
        with lock:
            if key == "lane":
                last_lane = message
            stamp = message.header.stamp.to_nsec()
            if stamp not in published_stamps:
                return
            if key == "grid":
                grid_count += 1
                require(message.info.width == 600 and message.info.height == 300,
                        "measurement expects the default 90/30/30/30 m window")
            if stamp not in seen[key]:
                seen[key].add(stamp)
                samples[key].append((rospy.Time.now() - message.header.stamp).to_sec() * 1000.)

    subscribers = [
        rospy.Subscriber("/hd_map/local/current_lane", LaneAssociation, callback,
                         callback_args="lane", queue_size=1),
        rospy.Subscriber("/hd_map/local/drivable_grid", OccupancyGrid, callback,
                         callback_args="grid", queue_size=1, buff_size=1024 * 1024),
    ]
    topic = rospy.get_param("/local_map_crop/ego_topic", "/Ego_topic")
    publisher = rospy.Publisher(topic, EgoVehicleStatus, queue_size=1)
    deadline = time.monotonic() + 5.
    while publisher.get_num_connections() == 0 and time.monotonic() < deadline:
        time.sleep(.05)
    require(publisher.get_num_connections() > 0, "no test pose subscriber")
    time.sleep(.3)
    for i in range(100):
        message = EgoVehicleStatus()
        message.position.x, message.position.y = -131.68979755061446, -428.3310229377821
        message.heading = 61.298126221 + (i % 4) * 10
        message.header.stamp = rospy.Time.now()
        with lock:
            published_stamps.add(message.header.stamp.to_nsec())
        publisher.publish(message)
        time.sleep(.05)
    time.sleep(.15)
    for key, values in samples.items():
        require(len(values) > 70, key + " delivered fewer than 70 distinct test poses")
        print("%s samples=%d latency_ms p50=%.2f p95=%.2f max=%.2f" % (
            key, len(values), *np.percentile(values, [50, 95, 100])))
    timeout = float(rospy.get_param("/local_map_crop/pose_stale_timeout_sec", .5))
    time.sleep(timeout + .2)
    with lock:
        count = grid_count
    time.sleep(.2)
    with lock:
        require(last_lane is not None and last_lane.state == LaneAssociation.STATE_STALE
                and not last_lane.valid, "missing stale association")
        require(grid_count == count, "expired crop kept refreshing")
    old = EgoVehicleStatus()
    old.position.x, old.position.y = 100., 100.
    old.header.stamp = rospy.Time.now() - rospy.Duration(5.)
    publisher.publish(old)
    time.sleep(.15)
    with lock:
        require(last_lane.state == LaneAssociation.STATE_STALE and last_lane.reference_point.x < 0,
                "delayed source pose was accepted")
    timings = []
    for _ in range(500):
        start = time.perf_counter()
        index.spatial((-131.69, -428.33), math.radians(61.3), layers=["lanes", "markings"])
        index.speed_limit("A2256W000202")
        index.signals_ahead(400.)
        timings.append((time.perf_counter() - start) * 1000.)
    print("selected vector query_ms p50=%.3f p95=%.3f max=%.3f" % tuple(
        np.percentile(timings, [50, 95, 100])))
    print("stale and delayed replay rejection valid")
    _ = subscribers


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print("HD-map latency verification failed: %s" % error, file=sys.stderr)
        raise SystemExit(1)
