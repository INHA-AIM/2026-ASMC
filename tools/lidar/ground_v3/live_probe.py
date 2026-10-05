#!/usr/bin/env python3
"""Bounded, read-only ground-review diagnostic/rate sampler."""
import argparse
from collections import Counter
import json
from pathlib import Path
import time

import rospy
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import PointCloud2


def percentile(values, q):
    if not values:
        return None
    values = sorted(values)
    index = (len(values) - 1) * q / 100
    lower = int(index)
    fraction = index - lower
    return values[lower] * (1 - fraction) + values[min(lower + 1, len(values) - 1)] * fraction


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=Path)
    parser.add_argument("--duration", type=float, default=31.0)
    parser.add_argument("--method", choices=["all", "jaeho", "patchwork", "linefit"], required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    print("initializing ROS probe", flush=True)
    rospy.init_node("ground_v3_live_probe", anonymous=True)
    received = []
    statuses = []
    def cloud(_):
        received.append(time.monotonic())
    def status(message):
        for row in message.status:
            if row.name == "asmc_lidar/ground_live":
                statuses.append((time.monotonic(), row.message,
                                 {v.key: v.value for v in row.values}))
    rospy.Subscriber("/velodyne_points", PointCloud2, cloud, queue_size=100)
    rospy.Subscriber("/lidar_candidate/ground_live/status", DiagnosticArray, status, queue_size=200)
    print("sampling live diagnostics", flush=True)
    time.sleep(args.duration)
    print("summarizing live diagnostics", flush=True)
    valid = [data for _, state, data in statuses if state == "valid"]
    first, last = (valid[0], valid[-1]) if valid else ({}, {})
    result = {"method": args.method, "duration_sec": args.duration,
              "scan_count": len(received), "valid_status_count": len(valid),
              "states": dict(Counter(state for _, state, _ in statuses)),
              "observed_scan_hz": (len(received) - 1) / (received[-1] - received[0])
              if len(received) > 1 else None,
              "first_counters": {k: first.get(k) for k in
                                 ("received", "processed", "invalid", "queue_dropped")},
              "last_counters": {k: last.get(k) for k in
                                ("received", "processed", "invalid", "queue_dropped")},
              "stamp_provenance": last.get("stamp_provenance"),
              "config_id": last.get("config_id"),
              "config_sha256": last.get("config_sha256"),
              "leveling_mode": last.get("leveling_mode"),
              "latency_ms": {key: {f"p{p}": percentile([float(row[key]) for row in valid if key in row], p)
                                    for p in (50, 95, 99)}
                             for key in ("queue_wait_ms", "prepare_ms", "compute_ms",
                                         "publish_ms", "receipt_to_publish_ms")}}
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"method": args.method, "scan_hz": result["observed_scan_hz"],
                      "states": result["states"], "latency_ms": result["latency_ms"]["compute_ms"]}))


if __name__ == "__main__":
    main()
