#!/usr/bin/env python3
"""Read-only diagnostic monitor for an isolated asmc_lidar ROS replay."""
import json
from pathlib import Path
import sys
import time

import rospy
from diagnostic_msgs.msg import DiagnosticArray


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(fraction * (len(ordered) - 1)))]


def main():
    if len(sys.argv) != 3:
        raise SystemExit("usage: monitor.py OUTPUT_JSON DURATION_SEC")
    path = Path(sys.argv[1])
    if path.exists():
        raise SystemExit("output exists")
    seconds = float(sys.argv[2])
    rospy.init_node("asmc_lidar_l2i1_monitor", anonymous=True, disable_signals=True)
    states = {}
    compute_ms = []
    observation_ns = []
    last_ground = {}
    last_candidate = {}

    def fields(item):
        return {value.key: value.value for value in item.values}

    def on_ground(message):
        for item in message.status:
            if item.name != "asmc_lidar/ground_live":
                continue
            values = fields(item)
            last_ground.update(values)
            if item.message == "valid":
                compute_ms.append(float(values["compute_ms"]))

    def on_candidate(message):
        for item in message.status:
            if item.name != "asmc_lidar/unified":
                continue
            states[item.message] = states.get(item.message, 0) + 1
            values = fields(item)
            last_candidate.update(values)
            if item.message == "valid":
                observation_ns.append(int(values["observation_stamp_ns"]))

    rospy.Subscriber("/lidar_candidate/ground_live/status", DiagnosticArray, on_ground, queue_size=40)
    rospy.Subscriber("/lidar_candidate/unified/status", DiagnosticArray, on_candidate, queue_size=40)
    end = time.monotonic() + seconds
    while time.monotonic() < end and not rospy.is_shutdown():
        time.sleep(.05)
    result = {
        "states": states,
        "compute_ms_p50_p95_p99": [percentile(compute_ms, p) for p in (.5, .95, .99)],
        "valid_observations": len(observation_ns),
        "unique_stamps": len(observation_ns) == len(set(observation_ns)),
        "increasing_stamps": all(a < b for a, b in zip(observation_ns, observation_ns[1:])),
        "last_ground": last_ground,
        "last_candidate": last_candidate,
        "scope": "isolated bag replay, not simulator end-to-end latency",
    }
    path.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")


if __name__ == "__main__":
    main()
