#!/usr/bin/env python3
"""Observe bridge diagnostics, console cadence and LiDAR ROS arrivals.

Uses the selected ROS master. This subscriber never starts a bridge or sends
control. Pair with measure_lidar_ingress.py in separate, exclusive runs.
"""
import argparse
import json
import statistics
import threading
import time
from pathlib import Path

import rospy
from diagnostic_msgs.msg import DiagnosticArray
from rosgraph_msgs.msg import Log
from sensor_msgs.msg import PointCloud2


def intervals(times):
    gaps = [b - a for a, b in zip(times, times[1:])]
    return dict(sample_count=len(times), hz=(len(times) - 1) / (times[-1] - times[0]) if len(times) > 1 else 0.,
                interval_median_sec=statistics.median(gaps) if gaps else None,
                interval_min_sec=min(gaps) if gaps else None,
                interval_max_sec=max(gaps) if gaps else None)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=30)
    parser.add_argument('--json-out', type=Path, required=True)
    args = parser.parse_args()
    if not 0 < args.duration <= 600:
        parser.error('duration must be in (0, 600]')
    rospy.init_node('bridge_reporting_probe', anonymous=True, disable_signals=True)
    lock = threading.Lock()
    diagnostics = {}
    clouds = {}
    console = {'sensors': [], 'network': []}
    report_errors = []
    def diagnostic(message, topic):
        row = dict(received_monotonic=time.monotonic(), streams={
            status.name: dict(level=status.level, message=status.message,
                             values={value.key: value.value for value in status.values})
            for status in message.status})
        with lock:
            diagnostics.setdefault(topic, []).append(row)
    def cloud(message, topic):
        with lock:
            clouds.setdefault(topic, []).append((time.monotonic(), message.header.stamp.to_nsec()))
    def log(message):
        with lock:
            for prefix in console:
                if message.msg.startswith(prefix + '['):
                    console[prefix].append(dict(received_monotonic=time.monotonic(), text=message.msg))
            if 'diagnostics failed; retrying' in message.msg or 'deque mutated' in message.msg:
                report_errors.append(message.msg)
    subscribers = []
    for topic in ('/morai_sensor_bridge/diagnostics', '/morai_udp_bridge/diagnostics'):
        subscribers.append(rospy.Subscriber(topic, DiagnosticArray, diagnostic, callback_args=topic, queue_size=1))
    for topic in ('/velodyne_points', '/velodyne_points_instance'):
        subscribers.append(rospy.Subscriber(topic, PointCloud2, cloud, callback_args=topic, queue_size=1))
    subscribers.append(rospy.Subscriber('/rosout_agg', Log, log, queue_size=100))
    started = time.monotonic()
    try:
        while not rospy.is_shutdown() and time.monotonic() - started < args.duration:
            time.sleep(.05)
    finally:
        for subscriber in subscribers:
            subscriber.unregister()
    with lock:
        result = dict(duration_sec=time.monotonic() - started, report_errors=list(report_errors),
            diagnostics={topic: dict(**intervals([row['received_monotonic'] for row in rows]), samples=rows)
                         for topic, rows in diagnostics.items()},
            clouds={topic: dict(**intervals([row[0] for row in rows]),
                duplicate_stamps=len(rows) - len({row[1] for row in rows}),
                regressing_stamps=sum(b[1] < a[1] for a, b in zip(rows, rows[1:])))
                for topic, rows in clouds.items()},
            console={prefix: dict(**intervals([row['received_monotonic'] for row in rows]), lines=rows)
                     for prefix, rows in console.items()})
    args.json_out.parent.mkdir(parents=True, exist_ok=True)
    args.json_out.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({key: value if key not in ('diagnostics', 'console') else {
        name: {k: v for k, v in row.items() if k not in ('samples', 'lines')}
        for name, row in value.items()} for key, value in result.items()}, indent=2))


if __name__ == '__main__':
    main()
