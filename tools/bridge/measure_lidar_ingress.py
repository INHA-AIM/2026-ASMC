#!/usr/bin/env python3
"""Measure MORAI Velodyne ingress without ROS conversion or control output.

Run only while the bridge is stopped: exclusive UDP binds intentionally refuse
to compete with an existing receiver. No sensor settings are changed.
"""
import argparse
import json
import select
import socket
import statistics
import struct
import sys
import time
from pathlib import Path


def udp_counters():
    lines = Path('/proc/net/snmp').read_text().splitlines()
    for index, line in enumerate(lines):
        if line.startswith('Udp:') and index + 1 < len(lines):
            return dict(zip(line.split()[1:], map(int, lines[index + 1].split()[1:])))
    return {}


def measure(ports, duration):
    streams = {}
    sockets = []
    before = udp_counters()
    try:
        for port in ports:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sockets.append(sock)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 << 20)
            sock.setsockopt(socket.SOL_SOCKET, 40, 1)  # Linux SO_RXQ_OVFL
            sock.bind(('0.0.0.0', port))  # no SO_REUSEADDR
            streams[sock] = dict(port=port, packets=0, bytes=0, wraps=0,
                malformed=0, socket_drops=0, device_regressions=0,
                previous_azimuth=None, previous_wrap_wall=None,
                previous_wrap_device=None, previous_device=None,
                first_device=None, last_device=None, first_wall=None,
                last_wall=None, wall_periods=[], device_periods=[],
                packet_counts=[], revolution_packets=0, azimuth_step_histogram={},
                effective_rcvbuf=sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF))
        started = time.monotonic()
        while time.monotonic() - started < duration:
            ready, _, _ = select.select(sockets, [], [], min(.2, max(0, duration - (time.monotonic() - started))))
            for sock in ready:
                packet, control, flags, _ = sock.recvmsg(65535, socket.CMSG_SPACE(4))
                now = time.monotonic()
                row = streams[sock]
                row['packets'] += 1
                row['bytes'] += len(packet)
                for level, kind, data in control:
                    if level == socket.SOL_SOCKET and kind == 40 and len(data) >= 4:
                        row['socket_drops'] = struct.unpack('=I', data[:4])[0]
                if len(packet) != 1206 or flags & socket.MSG_TRUNC:
                    row['malformed'] += 1
                    continue
                azimuth = struct.unpack_from('<H', packet, 2)[0]
                device = struct.unpack_from('<I', packet, 1200)[0]
                if not 0 <= device < 3600000000:
                    row['malformed'] += 1
                    continue
                if row['previous_device'] is not None and device < row['previous_device'] and row['previous_device'] - device < 1800000000:
                    row['device_regressions'] += 1
                if row['first_wall'] is None:
                    row['first_wall'], row['first_device'] = now, device
                row['last_wall'], row['last_device'] = now, device
                if row['previous_azimuth'] is not None:
                    step = (azimuth - row['previous_azimuth']) % 36000
                    histogram = row['azimuth_step_histogram']
                    histogram[step] = histogram.get(step, 0) + 1
                if row['previous_azimuth'] is not None and row['previous_azimuth'] - azimuth > 18000:
                    row['wraps'] += 1
                    if row['previous_wrap_wall'] is not None:
                        row['wall_periods'].append(now - row['previous_wrap_wall'])
                        delta = (device - row['previous_wrap_device']) % 3600000000 / 1e6
                        if 0 < delta < 10:
                            row['device_periods'].append(delta)
                        row['packet_counts'].append(row['revolution_packets'])
                    row['previous_wrap_wall'], row['previous_wrap_device'] = now, device
                    row['revolution_packets'] = 0
                row['revolution_packets'] += 1
                row['previous_azimuth'], row['previous_device'] = azimuth, device
        elapsed = time.monotonic() - started
        result = dict(duration_sec=elapsed, streams=[])
        for row in streams.values():
            output = {key: row[key] for key in ('port', 'packets', 'bytes', 'wraps', 'malformed', 'socket_drops', 'device_regressions', 'effective_rcvbuf')}
            output['raw_packet_hz'] = row['packets'] / elapsed
            for label in ('wall', 'device'):
                periods = row[label + '_periods']
                output[label + '_rotation_hz'] = len(periods) / sum(periods) if periods else None
                output[label + '_rotation_period_median_ms'] = statistics.median(periods) * 1000 if periods else None
                output[label + '_rotation_period_min_ms'] = min(periods) * 1000 if periods else None
                output[label + '_rotation_period_max_ms'] = max(periods) * 1000 if periods else None
            output['packets_per_rotation_median'] = statistics.median(row['packet_counts']) if row['packet_counts'] else None
            output['azimuth_step_histogram_hundredths_degree'] = row['azimuth_step_histogram']
            if row['first_wall'] is not None and row['last_wall'] > row['first_wall']:
                output['device_time_per_wall_time'] = ((row['last_device'] - row['first_device']) % 3600000000 / 1e6) / (row['last_wall'] - row['first_wall'])
            result['streams'].append(output)
        after = udp_counters()
        result['linux_udp_counter_delta'] = {key: after[key] - before.get(key, 0) for key in after}
        return result
    finally:
        for sock in sockets:
            sock.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ports', type=int, nargs='+', default=[9299, 9311])
    parser.add_argument('--duration', type=float, default=30)
    parser.add_argument('--json-out', type=Path)
    args = parser.parse_args()
    if not 0 < args.duration <= 3600 or len(set(args.ports)) != len(args.ports):
        parser.error('duration must be in (0, 3600]; ports must be unique')
    result = measure(args.ports, args.duration)
    body = json.dumps(result, indent=2) + '\n'
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(body)
    sys.stdout.write(body)


if __name__ == '__main__':
    main()
