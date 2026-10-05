#!/usr/bin/env python3
"""Freeze a historical shadow-route bag for pose-aware ground diagnostics.

This is a qualitative auxiliary input: older sensor profiles must never be
pooled with the Autonav v3 GT denominator. Read-only source bag; new output.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path

import numpy as np


def digest(path):
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(block)
    return value.hexdigest()


def rpy(message):
    x, y, z, w = [np.float32(getattr(message.orientation, k))
                  for k in ("x", "y", "z", "w")]
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    return float(np.float32(roll)), float(np.float32(pitch))


def xyzi(message):
    if message.is_bigendian:
        raise ValueError("big-endian PointCloud2 is unsupported")
    fields = {field.name: field for field in message.fields}
    names = ("x", "y", "z", "intensity")
    if any(name not in fields or fields[name].datatype != 7 or fields[name].count != 1
           for name in names):
        raise ValueError("expected scalar FLOAT32 XYZI")
    return np.column_stack([
        np.ndarray((message.height, message.width), dtype="<f4", buffer=message.data,
                   offset=fields[name].offset,
                   strides=(message.row_step, message.point_step)).reshape(-1)
        for name in names]).astype("<f4")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--sensor-profile", required=True,
                        help="Provenance label; never inferred from the cloud shape")
    args = parser.parse_args()
    import rosbag

    args.output.mkdir(parents=True, exist_ok=False)
    (args.output / "clouds").mkdir()
    stats = Counter()
    imu = None
    last_stamp = 0
    phase = args.bag.stem.replace(".", "_")
    with rosbag.Bag(str(args.bag)) as source, \
            (args.output / "snapshots.jsonl").open("x") as snapshots:
        for topic, message, arrival in source.read_messages(
                topics=["/velodyne_points", "/imu"]):
            if topic == "/imu":
                stamp = message.header.stamp.to_nsec()
                if stamp > 0:
                    imu = (stamp, arrival.to_nsec(), rpy(message))
                continue
            stats["scans"] += 1
            stamp = message.header.stamp.to_nsec()
            if stamp <= last_stamp or stamp <= 0:
                stats["invalid_scan_clock"] += 1
                continue
            last_stamp = stamp
            if imu is None:
                stats["missing_imu"] += 1
                continue
            imu_stamp, imu_arrival, attitude = imu
            gap = stamp - imu_stamp
            if gap < 0 or imu_arrival > arrival.to_nsec():
                stats["future_imu"] += 1
                continue
            if gap > 100_000_000:
                stats["stale_imu"] += 1
                continue
            if not all(math.isfinite(v) for v in attitude):
                stats["invalid_imu"] += 1
                continue
            cloud = xyzi(message)
            if not np.isfinite(cloud).all():
                stats["invalid_cloud"] += 1
                continue
            identity = f"shadow_bag_{stats['scans']:06d}"
            relative = Path("clouds") / (identity + ".bin")
            cloud.tofile(args.output / relative)
            row = {"input_id": identity, "phase": phase, "cloud_path": str(relative),
                   "scan_effective_stamp_ns": stamp, "scan_arrival_ns": arrival.to_nsec(),
                   "imu_stamp_ns": imu_stamp, "imu_arrival_ns": imu_arrival,
                   "imu_gap_ms": gap / 1e6, "roll": attitude[0], "pitch": attitude[1],
                   "geometry_valid": True, "measurement": True,
                   "source_frame": message.header.frame_id,
                   "source_profile": args.sensor_profile,
                   "cloud_sha256": digest(args.output / relative)}
            snapshots.write(json.dumps(row, sort_keys=True) + "\n")
            stats["accepted"] += 1
    metadata = {"source_bag": str(args.bag), "source_bag_sha256": digest(args.bag),
                "sensor_profile": args.sensor_profile, "stats": dict(stats),
                "snapshots_sha256": digest(args.output / "snapshots.jsonl"),
                "limitations": ["ROS/bridge receive stamps are not per-ray capture times",
                                "historical sensor profile is not Autonav v3 GT",
                                "no point semantic or instance GT in this bag"]}
    (args.output / "source.json").write_text(json.dumps(metadata, indent=2) + "\n")
    if stats["accepted"] < 2:
        raise ValueError("too few pose-valid scans")
    print(json.dumps(metadata))


if __name__ == "__main__":
    main()
