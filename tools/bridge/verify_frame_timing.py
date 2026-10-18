#!/usr/bin/env python3
"""Observe exact sensor/sidecar joins on a ROS master; publishes no control.

Bounded pending maps and rolling statistics. Complete matched timing records
are streamed to JSONL for later source-time analysis, without retaining payloads.
"""
import argparse
from collections import Counter, OrderedDict, defaultdict, deque
import json
from pathlib import Path
import threading
import time

import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage, Imu, PointCloud2
from asmc_msgs.msg import SensorFrameTiming
from diagnostic_msgs.msg import DiagnosticArray


def stats(values):
    values = np.asarray(values, dtype=float)
    return dict(samples=len(values), **({'p50': float(np.percentile(values, 50)),
        'p95': float(np.percentile(values, 95)), 'max': float(values.max())} if len(values) else {}))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=30)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not 0 < args.duration <= 600:
        parser.error('duration must be in (0,600]')
    args.output.mkdir(parents=True, exist_ok=False)
    rospy.init_node('sensor_timing_probe', anonymous=True, disable_signals=True)
    lock = threading.Lock(); stopping = threading.Event(); pending = ({}, {})
    counts = defaultdict(Counter); windows = defaultdict(lambda: defaultdict(lambda: deque(maxlen=4096)))
    previous = {}; diagnostics = {}; records = (args.output / 'matched.jsonl').open('w')
    raw_timing = (args.output / 'timing.jsonl').open('w')
    sensor_headers = (args.output / 'sensor-headers.jsonl').open('w')
    def timing_dict(message):
        row = {slot: getattr(message, slot) for slot in message.__slots__}
        for field in ('published_stamp', 'publish_ros_time'):
            row[field] = row[field].to_nsec()
        return row
    def key(topic, header):
        return topic, header.frame_id, header.stamp.to_nsec(), header.seq
    def add(side, stream, join_key, value):
        table = pending[side].setdefault(stream, OrderedDict())
        if join_key in table:
            counts[stream]['ambiguous_keys'] += 1
            # Fail closed: never overwrite a colliding key and pick arbitrarily.
            table[join_key] = (time.monotonic(), None)
        else:
            table[join_key] = (time.monotonic(), value)
        other = pending[1 - side].setdefault(stream, OrderedDict())
        if join_key in other:
            _, first = table.pop(join_key); _, second = other.pop(join_key)
            if first is None or second is None:
                counts[stream]['ambiguous_matches'] += 1
                return
            sensor, timing = (first, second) if side == 0 else (second, first)
            counts[stream]['matched'] += 1
            row = timing_dict(timing)
            row['sensor_received_monotonic_ns'] = sensor
            records.write(json.dumps(row) + '\n')
            windows[stream]['assembly_ms'].append((timing.rx_last_monotonic_ns - timing.rx_first_monotonic_ns) / 1e6)
            windows[stream]['last_rx_to_sidecar_ms'].append((timing.publish_monotonic_ns - timing.rx_last_monotonic_ns) / 1e6)
        now = time.monotonic()
        for table in (table, other):
            while table and (len(table) > 256 or now - next(iter(table.values()))[0] > 2):
                table.popitem(last=False)
                counts[stream]['expired_sensor' if table is pending[0][stream] else 'expired_timing'] += 1
    def sensor(message, pair):
        stream, topic = pair
        with lock:
            if stopping.is_set():
                return
            counts[stream]['sensor_messages'] += 1
            sensor_headers.write(json.dumps(dict(stream_id=stream, key=key(topic, message.header))) + '\n')
            add(0, stream, key(topic, message.header), time.monotonic_ns())
    def timing(message):
        with lock:
            if stopping.is_set():
                return
            stream = message.stream_id; count = counts[stream]
            raw_timing.write(json.dumps(timing_dict(message)) + '\n')
            count['timing_messages'] += 1
            for field in ('source_duplicate', 'source_conflict', 'source_regression', 'device_hour_rollover',
                          'rx_userspace_fallback_packets', 'ancillary_truncated_packets', 'device_tick_duplicates'):
                count[field] += int(getattr(message, field))
            count['invalid_source'] += int(not message.source_valid)
            identity = message.bridge_session_id, message.frame_sequence
            if stream in previous and identity[0] == previous[stream][0]:
                count['timing_sequence_gaps'] += max(0, identity[1] - previous[stream][1] - 1)
                count['timing_sequence_regressions'] += int(identity[1] <= previous[stream][1])
            elif stream in previous:
                count['session_changes'] += 1
                for table in pending:
                    table.pop(stream, None)
            previous[stream] = identity
            if message.sensor_wire_sequence_valid:
                count['joinable_metadata'] += 1
                join_key = message.sensor_topic, message.sensor_frame_id, message.published_stamp.to_nsec(), message.sensor_wire_sequence
                add(1, stream, join_key, message)
            else:
                count['sensor_not_serialized'] += 1
    def diagnostic(message):
        with lock:
            for status in message.status:
                diagnostics[status.name] = {item.key: item.value for item in status.values}
    subscribers = [rospy.Subscriber('/morai_sensor_bridge/frame_timing', SensorFrameTiming, timing, queue_size=256),
                   rospy.Subscriber('/morai_sensor_bridge/diagnostics', DiagnosticArray, diagnostic, queue_size=1)]
    streams = [('cam1', '/image_jpeg/compressed', CompressedImage),
               ('cam2', '/cam_left/image_jpeg/compressed', CompressedImage),
               ('cam3', '/cam_right/image_jpeg/compressed', CompressedImage),
               ('cam4', '/cam_extra/image_jpeg/compressed', CompressedImage),
               ('imu', '/imu', Imu), ('lidar', '/velodyne_points', PointCloud2)]
    for stream, topic, kind in streams:
        subscribers.append(rospy.Subscriber(topic, kind, sensor, callback_args=(stream, topic), queue_size=256,
                                             buff_size=8 * 1024 * 1024))
    started = time.monotonic()
    try:
        while not rospy.is_shutdown() and time.monotonic() - started < args.duration:
            time.sleep(.05)
    finally:
        stopping.set()
        for subscriber in subscribers:
            subscriber.unregister()
        with lock:
            records.close()
            raw_timing.close(); sensor_headers.close()
            result = dict(duration_sec=time.monotonic() - started, streams={
                stream: dict(count, pending_sensor=len(pending[0].get(stream, {})),
                             pending_timing=len(pending[1].get(stream, {})),
                             statistics={field: stats(values) for field, values in windows[stream].items()})
                for stream, count in counts.items()}, diagnostics=diagnostics,
                limitations=['Probe receive time is user space, not kernel or exposure time.',
                             'Matched JSONL excludes unmatched/ambiguous inputs; counts report losses.',
                             'Initial subscriptions and final in-flight records can remain unmatched.'])
            (args.output / 'report.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result['streams'], indent=2))


if __name__ == '__main__':
    main()
