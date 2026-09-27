#!/usr/bin/env python3
"""Audit collector scene measurement windows, excluding teleports and settling.

Bag times must overlap recorded wall-clock windows. This tool never treats a
stationary teleport fixture as an actual driving/tunnel handoff trajectory.
"""
import argparse
import json
from pathlib import Path

from audit_tracking_bag import DEFAULT_CLOUDS, DIAGNOSTIC_TOPICS, TrackingAudit, normalize


def measurement_windows(manifest):
    windows = []
    for scene in manifest["scenes"]:
        if "measurement_start_wall_s" not in scene or "measurement_end_wall_s" not in scene:
            raise ValueError("incomplete scene: " + scene["name"])
        start, end = (int(scene[key] * 1e9) for key in ("measurement_start_wall_s", "measurement_end_wall_s"))
        if end <= start or (windows and start <= windows[-1][2]):
            raise ValueError("invalid/overlapping measurement windows")
        windows.append((scene["name"], start, end))
    if not windows:
        raise ValueError("manifest has no scenes")
    return windows


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--sample-points-per-cloud", type=int, default=300)
    args = parser.parse_args(argv)
    if args.output.exists():
        parser.error("output exists; choose a new report path")
    if args.sample_points_per_cloud < 0:
        parser.error("sample count must be nonnegative")
    manifest = json.loads(args.manifest.read_text())
    if not manifest.get("recording_complete"):
        parser.error("manifest does not certify a completed recording")
    if not manifest.get("collection_complete"):
        parser.error("manifest does not certify every planned scene completed")
    windows = measurement_windows(manifest)
    try:
        import rosbag
    except ImportError:
        parser.error("run inside ROS Noetic after sourcing its environment")
    audits = {name: TrackingAudit() for name, _, _ in windows}
    full = TrackingAudit()
    topics = list(DEFAULT_CLOUDS) + ["/last_lidar", "/lidar_nogps", "/lidar/tracked_objects_map", "/gps", "/imu", "/Ego_topic",
                                   "/YOLO/BBox", "/camera/image/compressed"] + list(DIAGNOSTIC_TOPICS)
    for source in manifest["bags"]:
        path = args.manifest.parent / source["path"]
        with rosbag.Bag(str(path), "r") as bag:
            for topic, message, time in bag.read_messages(topics=topics):
                record = normalize(topic, message, time.to_nsec(), args.sample_points_per_cloud)
                full.add(record)
                for name, start, end in windows:
                    if start <= record["bag_ns"] < end:
                        audits[name].add(record)
                        break
    scenes = {}
    for name, start, end in windows:
        audit = audits[name]
        if not audit.records:
            parser.error("no ROS bag samples overlap scene wall time: " + name)
        report = audit.report()
        gps = audit.records.get("/gps", [])
        report["raw_gps"] = {"samples": len(gps), "valid_samples": sum(record.get("gps_valid", False) for record in gps)}
        report["measurement_duration_sec"] = (end - start) / 1e9
        scenes[name] = report
    result = {"schema_version": "asmc-lidar-stationary-scene-audit-1",
              "manifest": str(args.manifest), "scenes": scenes,
              "full_recording": full.report(),
              "limitations": ["These are stationary fixtures, not a continuous driving or mode-switch acceptance test.",
                              "Scene windows use wall time; bag receipt timestamps must share that clock.",
                              "Asynchronous actor snapshots are not synchronized object/point labels.",
                              "Object count or point reduction is not detection accuracy without annotated truth."]}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x") as stream:
        json.dump(result, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
    for name, report in scenes.items():
        cloud = report["topics"].get("/velodyne_points", {})
        print(json.dumps({"scene": name, "cloud_messages": cloud.get("messages"), "cloud_hz": cloud.get("bag_rate_hz"),
                          "gps": report["raw_gps"], "handoff": report["handoff"]}, sort_keys=True))


if __name__ == "__main__":
    main()
