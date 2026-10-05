#!/usr/bin/env python3
"""Synchronize a 31-second diagnostic sample with controlled ROS bag playback.

Run with an explicit ROS master and exactly one ground review node, no UDP bridge.
"""
import argparse
import json
from pathlib import Path
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--method", required=True, choices=("all", "jaeho", "patchwork", "linefit"))
    parser.add_argument("--rate", type=float, default=1.15)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    probe = subprocess.Popen([sys.executable, "-u", str(Path(__file__).with_name("live_probe.py")),
                              str(args.output), "--method", args.method, "--duration", "31"],
                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    bag = None
    try:
        while True:
            line = probe.stdout.readline()
            if not line:
                raise RuntimeError("probe exited before ready")
            print(line, end="", flush=True)
            if "sampling live diagnostics" in line:
                break
        bag = subprocess.Popen(["rosbag", "play", "--quiet", "--rate", str(args.rate),
                                "--duration", "40", str(args.bag),
                                "--topics", "/imu", "/velodyne_points"],
                               stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
        stdout, _ = probe.communicate(timeout=44)
        print(stdout, end="", flush=True)
        if probe.returncode:
            raise RuntimeError(f"probe exited {probe.returncode}")
        _, stderr = bag.communicate(timeout=48)
        if bag.returncode:
            raise RuntimeError(f"rosbag exited {bag.returncode}: {stderr[-800:]}")
        result = json.loads(args.output.read_text())
        if result["observed_scan_hz"] is None:
            raise RuntimeError("bag produced no LiDAR scans")
        print(json.dumps({"method": args.method, "scan_hz": result["observed_scan_hz"],
                          "valid": result["valid_status_count"],
                          "queue_last": result["last_counters"]["queue_dropped"]}))
    finally:
        if bag is not None and bag.poll() is None:
            bag.terminate()
            bag.wait(timeout=5)
        if probe.poll() is None:
            probe.terminate()
            probe.wait(timeout=5)


if __name__ == "__main__":
    main()
