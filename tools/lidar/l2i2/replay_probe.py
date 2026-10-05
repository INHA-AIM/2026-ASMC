#!/usr/bin/env python3
"""Read-only probe for isolated asmc_lidar bag/live acceptance.

Run alongside rosbag play. It never publishes, changes parameters, or treats
map-unavailable/invalid as an observed empty frame.
"""

import argparse
import collections
import json
import statistics
import time

import rospy
from asmc_msgs.msg import DetectedObjectArray, LidarObjectArray, TrackedObjectArray, VehicleState
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import PointCloud2
from morai_msgs.msg import EgoVehicleStatus, GPSMessage


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=30.0)
    args = parser.parse_args()
    rospy.init_node("asmc_lidar_replay_probe", anonymous=True)
    counts = collections.Counter()
    statuses = collections.Counter()
    reasons = collections.Counter()
    validity = collections.Counter()
    first_stamp = {}
    last_stamp = {}
    first_wall = {}
    last_wall = {}
    stamp_gaps_ms = collections.defaultdict(list)
    times_ms = []
    map_pose_diagnostics = {}
    pipeline_diagnostics = {}
    track_ids = set()
    max_objects = 0

    def counted(name, msg):
        nonlocal max_objects
        counts[name] += 1
        now = time.monotonic()
        first_wall.setdefault(name, now)
        last_wall[name] = now
        stamp = msg.header.stamp.to_nsec()
        if stamp:
            if name in last_stamp and stamp > last_stamp[name]:
                stamp_gaps_ms[name].append((stamp-last_stamp[name])*1e-6)
            first_stamp.setdefault(name, stamp)
            last_stamp[name] = stamp
        if isinstance(msg, LidarObjectArray):
            statuses[name, msg.status] += 1
            if msg.reason:
                reasons[name, msg.reason] += 1
            max_objects = max(max_objects, len(msg.objects))
            if name == "local":
                track_ids.update(o.track_id for o in msg.objects)
                for obj in msg.objects:
                    if obj.observation_state != obj.OBSERVED:
                        validity["predicted"] += 1
                        continue
                    validity["observed"] += 1
                    for key, ok in (("relative_velocity", obj.relative_velocity_valid),
                                    ("absolute_velocity", obj.absolute_velocity_valid),
                                    ("dynamic", obj.dynamic_valid),
                                    ("yaw_rate", obj.yaw_rate_valid),
                                    ("dimensions", min(obj.length, obj.width, obj.height) > .05)):
                        if ok:
                            validity[key] += 1

    subscriptions = [
        ("scan", "/velodyne_points", PointCloud2),
        ("local", "/perception/lidar/objects_local", LidarObjectArray),
        ("map", "/perception/lidar/objects_map", LidarObjectArray),
        ("compat", "/perception/lidar/tracked_objects", TrackedObjectArray),
        ("planner_candidate", "/perception/objects_lidar_candidate", DetectedObjectArray),
        ("vehicle", "/vehicle/state", VehicleState),
        ("gps", "/gps", GPSMessage),
        ("ego", "/Ego_topic", EgoVehicleStatus),
    ]
    for name, topic, kind in subscriptions:
        if name == "vehicle":
            rospy.Subscriber(topic, kind, lambda msg: (counted("vehicle", msg),
                            validity.update(["valid_vehicle"] if msg.valid else ["invalid_vehicle"])),
                             queue_size=100)
        elif name == "gps":
            rospy.Subscriber(topic, kind, lambda msg: (counted("gps", msg),
                            validity.update(["valid_gps"] if msg.status > 0 else ["invalid_gps"])),
                             queue_size=100)
        else:
            rospy.Subscriber(topic, kind, lambda msg, name=name: counted(name, msg), queue_size=100)

    def diagnostics(message):
        for item in message.status:
            if item.name == "asmc_lidar/ground_live":
                for field in item.values:
                    if field.key in ("received", "processed", "invalid", "queue_dropped"):
                        pipeline_diagnostics[field.key] = field.value
                    if field.key == "compute_ms":
                        try:
                            times_ms.append(float(field.value))
                        except ValueError:
                            pass
            elif item.name == "asmc_lidar/map_pose":
                map_pose_diagnostics.update({field.key: field.value for field in item.values})

    rospy.Subscriber("/lidar_candidate/ground_live/status", DiagnosticArray,
                     diagnostics, queue_size=100)
    rospy.Subscriber("/perception/lidar/map_status", DiagnosticArray,
                     diagnostics, queue_size=100)
    stop = time.monotonic() + args.seconds
    while not rospy.is_shutdown() and time.monotonic() < stop:
        time.sleep(0.05)
    rates = {name: (counts[name] - 1) * 1e9 / (last_stamp[name] - first_stamp[name])
             for name in last_stamp if counts[name] > 1 and last_stamp[name] > first_stamp[name]}
    wall_rates = {name: (counts[name] - 1) / (last_wall[name] - first_wall[name])
                  for name in last_wall if counts[name] > 1 and last_wall[name] > first_wall[name]}
    ordered = sorted(times_ms)
    percentile = lambda q: ordered[min(len(ordered) - 1, int((len(ordered) - 1) * q))] if ordered else None
    result = {
        "counts": dict(counts), "source_stamp_hz": rates, "arrival_wall_hz": wall_rates,
        "statuses": {f"{name}:{status}": count for (name, status), count in statuses.items()},
        "reasons": {f"{name}:{reason}": count for (name, reason), count in reasons.items()},
        "unique_local_track_ids": len(track_ids), "max_objects_per_frame": max_objects,
        "object_validity_counts": dict(validity),
        "map_pose_diagnostics": map_pose_diagnostics,
        "pipeline_diagnostics": pipeline_diagnostics,
        "source_gap_ms": {name: {"p95": sorted(gaps)[int(.95*(len(gaps)-1))],
                                  "max": max(gaps), "over_100ms": sum(x>100 for x in gaps)}
                          for name, gaps in stamp_gaps_ms.items() if gaps},
        "compute_ms": {"p50": percentile(.5), "p95": percentile(.95), "p99": percentile(.99)},
    }
    print(json.dumps(result, sort_keys=True, indent=2), flush=True)


if __name__ == "__main__":
    main()
