#!/usr/bin/env python3
"""Record system resources and ROS diagnostics during a UDP/ROS transport pilot."""

from __future__ import print_function

import argparse
import json
import os
import signal
import sys
import threading
import time
from datetime import datetime, timezone

import psutil
import rospy
from diagnostic_msgs.msg import DiagnosticArray


def utc_now_iso():
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


class DiagnosticSnapshot(object):
    def __init__(self, topics):
        self.lock = threading.Lock()
        self.values = {}
        self.subscribers = [
            rospy.Subscriber(
                topic, DiagnosticArray, self.callback,
                callback_args=topic, queue_size=1
            )
            for topic in topics
        ]

    def callback(self, message, topic):
        value = {}
        for status in message.status:
            value[status.name] = {
                "level": int(status.level),
                "message": status.message,
                "values": dict((item.key, item.value) for item in status.values),
            }
        with self.lock:
            self.values[topic] = value

    def snapshot(self):
        with self.lock:
            return dict(self.values)

    def close(self):
        for subscriber in self.subscribers:
            subscriber.unregister()


def matching_processes():
    needles = (
        "morai_sensor_bridge", "morai_udp_bridge", "rosbridge_websocket",
        "semantic_inference_node", "rosbag record", "rosbag play",
    )
    result = []
    for process in psutil.process_iter(["pid", "name", "cmdline"]):
        try:
            command = " ".join(process.info.get("cmdline") or [])
            if any(needle in command for needle in needles):
                result.append(process)
        except (psutil.AccessDenied, psutil.NoSuchProcess):
            continue
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=300.0)
    parser.add_argument("--interval", type=float, default=1.0)
    parser.add_argument("--json-out", required=True)
    parser.add_argument("--label", required=True, help="for example udp_static or ros_drive")
    args = parser.parse_args()
    if args.duration <= 0 or args.interval <= 0:
        raise SystemExit("duration and interval must be positive")

    output = os.path.abspath(os.path.expanduser(args.json_out))
    os.makedirs(os.path.dirname(output), exist_ok=True)
    if os.path.exists(output):
        raise SystemExit("refusing to overwrite: {}".format(output))

    rospy.init_node("transport_pilot_monitor", anonymous=True, disable_signals=True)
    diagnostics = DiagnosticSnapshot(
        (
            "/morai_sensor_bridge/diagnostics",
            "/morai_udp_bridge/diagnostics",
            "/perception/camera/diagnostics",
        )
    )
    stop = threading.Event()

    def request_stop(_signal, _frame):
        stop.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    processes = dict((process.pid, process) for process in matching_processes())
    for process in processes.values():
        try:
            process.cpu_percent(None)
        except (psutil.AccessDenied, psutil.NoSuchProcess):
            pass
    psutil.cpu_percent(None)
    network_previous = psutil.net_io_counters()
    wall_previous = time.monotonic()
    started = wall_previous

    with open(output, "x") as stream:
        while not stop.is_set() and not rospy.is_shutdown():
            elapsed = time.monotonic() - started
            if elapsed >= args.duration:
                break
            stop.wait(min(args.interval, args.duration - elapsed))
            now = time.monotonic()
            delta = max(1e-6, now - wall_previous)
            network = psutil.net_io_counters()
            memory = psutil.virtual_memory()
            process_values = []
            for process in matching_processes():
                if process.pid not in processes:
                    processes[process.pid] = process
                    try:
                        process.cpu_percent(None)
                    except (psutil.AccessDenied, psutil.NoSuchProcess):
                        pass
            for process in list(processes.values()):
                try:
                    process_values.append(
                        {
                            "pid": process.pid,
                            "name": process.name(),
                            "cpu_percent": process.cpu_percent(None),
                            "rss_bytes": process.memory_info().rss,
                        }
                    )
                except (psutil.AccessDenied, psutil.NoSuchProcess):
                    processes.pop(process.pid, None)
                    continue
            record = {
                "schema_version": "asmc-transport-pilot-1.0.0",
                "recorded_at_utc": utc_now_iso(),
                "elapsed_seconds": round(now - started, 3),
                "label": args.label,
                "system": {
                    "cpu_percent": psutil.cpu_percent(None),
                    "memory_percent": memory.percent,
                    "memory_available_bytes": memory.available,
                    "network_rx_bytes_per_sec": round(
                        (network.bytes_recv - network_previous.bytes_recv) / delta, 3
                    ),
                    "network_tx_bytes_per_sec": round(
                        (network.bytes_sent - network_previous.bytes_sent) / delta, 3
                    ),
                },
                "processes": process_values,
                "diagnostics": diagnostics.snapshot(),
            }
            stream.write(json.dumps(record, ensure_ascii=False, sort_keys=True) + "\n")
            stream.flush()
            network_previous = network
            wall_previous = now
    diagnostics.close()
    print(output)
    return 0


if __name__ == "__main__":
    sys.exit(main())
