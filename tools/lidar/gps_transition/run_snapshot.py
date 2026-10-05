#!/usr/bin/env python3
"""Same L1-B1 geometry snapshot under isolated GPS/consumer state cases."""

import argparse
import copy
import hashlib
import json
from pathlib import Path

from core import GpsGate, select_output


def fix(gate, stamp, x=.0, **changes):
    event = dict(arrival_ns=stamp, stamp_ns=stamp, clock="ros_receive",
                 status=1, latitude=37.0, longitude=127.0, altitude=28.0, xy_m=(x, 0.))
    event.update(changes)
    gate.feed(**event)


def cases(snapshot):
    t = snapshot["stamp_ns"]
    if snapshot["input_id"] != "roll_pitch_mount" or snapshot["pose_reason"] != "valid":
        raise ValueError("expected pinned roll_pitch_mount geometry snapshot")
    scenarios = []
    for name in ("ready", "invalid_zero_status1", "stale", "recovering",
                 "jump_rejected", "pose_untrusted", "vehicle_unavailable",
                 "trajectory_unavailable", "detector_not_run", "observed_empty",
                 "sensor_missing"):
        gate = GpsGate()
        if name == "stale":
            for dt, x in ((-1_000_000_000, 0.), (-800_000_000, .2), (-600_000_000, .4)):
                fix(gate, t+dt, x)
        elif name == "recovering":
            for dt, x in ((-1_400_000_000, 0.), (-1_200_000_000, .2), (-1_000_000_000, .4)):
                fix(gate, t+dt, x)
            fix(gate, t-600_000_000, latitude=0., longitude=0.)
            fix(gate, t-400_000_000, .6)
            fix(gate, t-200_000_000, .8)
        else:
            for dt, x in ((-400_000_000, 0.), (-200_000_000, .2), (0, .4)):
                fix(gate, t+dt, x)
            if name == "invalid_zero_status1":
                fix(gate, t+1, latitude=0., longitude=0.)
            if name == "jump_rejected":
                fix(gate, t+1, x=200.)
        payload = None if name == "sensor_missing" else copy.deepcopy(snapshot)
        if name == "observed_empty":
            payload["objects"] = []
        result = select_output(payload, gate, processing_ns=t+1,
                               detector_ran=name != "detector_not_run",
                               pose_trusted=name != "pose_untrusted",
                               vehicle_state_ready=name != "vehicle_unavailable",
                               trajectory_ready=name != "trajectory_unavailable")
        result["case"] = name
        scenarios.append(result)
    expected = {"ready": ("objects", True), "invalid_zero_status1": ("gps_invalid", False),
                "stale": ("gps_stale", False), "recovering": ("gps_recovering", False),
                "jump_rejected": ("gps_jump_rejected", False),
                "pose_untrusted": ("pose_unavailable", False),
                "vehicle_unavailable": ("objects", False),
                "trajectory_unavailable": ("objects", False),
                "detector_not_run": ("unassessed_detector_not_run", False),
                "observed_empty": ("observed_empty", True),
                "sensor_missing": ("sensor_missing", False)}
    for row in scenarios:
        if (row["map"]["status"], row["planner_ready"]) != expected[row["case"]]:
            raise AssertionError(row["case"] + ": unexpected selection")
        if row["local"]["objects"] and row["local"]["objects"][0]["id"] != 42:
            raise AssertionError("track identity changed in selector")
    return scenarios


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("output exists; select a new run path")
    data = args.input.read_bytes()
    sample = next((json.loads(line) for line in data.splitlines()
                   if json.loads(line).get("input_id") == "roll_pitch_mount"), None)
    if sample is None:
        raise ValueError("roll_pitch_mount missing")
    output = {"schema": "asmc-lidar-l1b2-snapshot-v1",
              "source_result_sha256": hashlib.sha256(data).hexdigest(),
              "scenarios": cases(sample)}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x") as target:
        json.dump(output, target, indent=2, sort_keys=True, allow_nan=False)
        target.write("\n")
    print(json.dumps({"output": str(args.output), "cases": len(output["scenarios"])},
                     sort_keys=True))


if __name__ == "__main__":
    main()
