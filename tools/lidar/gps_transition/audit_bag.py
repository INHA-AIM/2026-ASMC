#!/usr/bin/env python3
"""Read-only arrival-order GPS gate audit; does not infer a map pose from GPS."""

import argparse
from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path

from core import GpsGate


def file_sha(path):
    digest = hashlib.sha256()
    with open(path, "rb") as source:
        for chunk in iter(lambda: source.read(4 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def scene_at(ns, scenes):
    for scene in scenes:
        if (round(scene["measurement_start_wall_s"] * 1e9) <= ns <=
                round(scene["measurement_end_wall_s"] * 1e9)):
            return scene["name"]
    return "transition_or_outside"


def analyze(bag_path, scene_path):
    import rosbag
    from pyproj import Transformer

    manifest = json.loads(scene_path.read_text())
    expected = next((b["sha256"] for b in manifest["bags"]
                     if b["path"] == bag_path.name), None)
    actual = file_sha(bag_path)
    if expected != actual:
        raise ValueError("recorded bag SHA differs from scene manifest")
    utm = Transformer.from_crs("EPSG:4326", "EPSG:32652", always_xy=True)
    gate = GpsGate()
    by_scene = defaultdict(lambda: {"gps": Counter(), "scan_gate": Counter(),
                                    "output_messages": Counter()})
    transitions = Counter()
    last_state = gate.state
    seen = Counter()
    topics = ["/gps", "/velodyne_points", "/last_lidar", "/lidar_nogps"]
    with rosbag.Bag(str(bag_path)) as bag:
        for topic, msg, arrival in bag.read_messages(topics=topics):
            ns = arrival.to_nsec()
            scene = scene_at(ns, manifest["scenes"])
            bucket = by_scene[scene]
            seen[topic] += 1
            if topic == "/gps":
                zero = (msg.latitude == 0 and msg.longitude == 0)
                bucket["gps"]["status_" + str(msg.status)] += 1
                if zero:
                    bucket["gps"]["zero_lat_lon"] += 1
                xy = None
                if (not zero and math.isfinite(msg.latitude) and
                        math.isfinite(msg.longitude) and -90 <= msg.latitude <= 90 and
                        -180 <= msg.longitude <= 180):
                    east, north = utm.transform(msg.longitude, msg.latitude)
                    xy = (east - msg.eastOffset, north - msg.northOffset)
                state = gate.feed(arrival_ns=ns, stamp_ns=msg.header.stamp.to_nsec(),
                                  clock="ros_receive", status=int(msg.status),
                                  latitude=msg.latitude, longitude=msg.longitude,
                                  altitude=msg.altitude, xy_m=xy)
                bucket["gps"]["gate_" + state["state"]] += 1
                bucket["gps"]["reason_" + state["reason"]] += 1
            elif topic == "/velodyne_points":
                state = gate.tick(ns)
                bucket["scan_gate"][state["state"]] += 1
                # Neither the internal leveling snapshot nor a trusted observation-
                # time rear-axle pose was recorded. Do not manufacture map output.
                bucket["scan_gate"]["map_pose_not_assessed"] += 1
            else:
                bucket["output_messages"][topic] += 1
                continue
            if state["state"] != last_state:
                transitions[last_state + "->" + state["state"]] += 1
                last_state = state["state"]
    return {"schema": "asmc-lidar-l1b2-bag-audit-v1",
            "bag": {"path": str(bag_path), "bytes": bag_path.stat().st_size,
                    "sha256": actual},
            "scene_manifest_sha256": file_sha(scene_path),
            "policy": {"gps_max_age_ns": gate.policy.max_age_ns,
                       "recovery_fixes": gate.policy.recovery_fixes,
                       "jump_max_speed_mps": gate.policy.max_speed_mps,
                       "jump_slack_m": gate.policy.jump_slack_m},
            "topic_messages": dict(sorted(seen.items())),
            "transitions": dict(sorted(transitions.items())),
            "scenes": {name: {key: dict(sorted(counts.items())) for key, counts in values.items()}
                       for name, values in sorted(by_scene.items())},
            "limits": ["fixed_stationary_scenes_with_teleport_transitions_not_continuous_driving",
                       "gps_status_does_not_prove_coordinate_validity",
                       "no_observation_time_rear_axle_pose_or_leveling_snapshot_recorded",
                       "not_a_detector_or_tracker_accuracy_evaluation"]}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True, type=Path)
    parser.add_argument("--scene-manifest", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("output exists; select a new run path")
    report = analyze(args.bag, args.scene_manifest)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x") as out:
        json.dump(report, out, indent=2, sort_keys=True)
        out.write("\n")
    print(json.dumps({"report": str(args.output), "topic_messages": report["topic_messages"],
                      "transitions": report["transitions"]}, sort_keys=True))


if __name__ == "__main__":
    main()
