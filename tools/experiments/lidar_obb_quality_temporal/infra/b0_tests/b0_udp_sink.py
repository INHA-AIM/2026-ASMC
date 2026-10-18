#!/usr/bin/env python3
"""LABELLED POSITIVE CONTROL ONLY (orchestrator decision 7). Runs inside the separate unprivileged user+net
namespace of a positive-control run (namespace-private loopback, 0 routes): binds 127.0.0.1:PORT (planner UDP
7510, setting.yaml PlannerInterface) and decodes every datagram Main's PlannerObjectSender sends
(PlannerObjectSender.cpp Send: 'LIDAR_OBJECT_INFO' header, data offset 38, 106-byte slots, int16 id+1 at +0,
int16 type at +2, float32 center xyz +4/+8/+12, yaw deg +16, l/w/h +20/+24/+28, velocity km/h x/y +44/+48).
Writes DIR/udp_sink.jsonl, one line per datagram with CLOCK_MONOTONIC arrival. Sends nothing.
usage: b0_udp_sink.py --out DIR [--port 7510] [--duration-s 600]"""
import argparse
import json
import os
import select
import socket
import struct
import time


def decode(data):
    out = dict(bytes=len(data), header=data[:17].decode(errors='replace'), slots=[])
    if len(data) < 38:
        return out
    n = (len(data) - 38) // 106
    for i in range(n):
        b = 38 + i * 106
        wire_id, typ = struct.unpack_from('<hh', data, b)
        if wire_id == 0:
            continue
        cx, cy, cz, yaw, l, w, h = struct.unpack_from('<7f', data, b + 4)
        vx, vy = struct.unpack_from('<2f', data, b + 44)
        out['slots'].append(dict(id=wire_id - 1, type=typ, c=[round(cx, 3), round(cy, 3), round(cz, 3)],
                                 yaw_deg=round(yaw, 2), lwh=[round(l, 3), round(w, 3), round(h, 3)],
                                 vx_kph=round(vx, 3), vy_kph=round(vy, 3), speed_kph=round((vx * vx + vy * vy) ** .5, 3)))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--port', type=int, default=7510)
    ap.add_argument('--duration-s', type=float, default=600.0)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(('127.0.0.1', a.port))
    log = open(os.path.join(a.out, 'udp_sink.jsonl'), 'w', buffering=1)
    log.write(json.dumps(dict(ev='bound', addr='127.0.0.1', port=a.port, mono_ns=time.monotonic_ns(),
                              label='POSITIVE_CONTROL_namespace_private_loopback')) + '\n')
    end = time.monotonic() + a.duration_s
    count = 0
    while time.monotonic() < end:
        r, _, _ = select.select([s], [], [], 0.2)
        if not r:
            if os.path.exists(os.path.join(a.out, 'udp_sink.stop')):
                break
            continue
        data, src = s.recvfrom(65535)
        count += 1
        row = decode(data)
        row.update(ev='datagram', n=count, mono_ns=time.monotonic_ns(), src='%s:%d' % src)
        log.write(json.dumps(row) + '\n')
    log.write(json.dumps(dict(ev='end', datagrams=count, mono_ns=time.monotonic_ns())) + '\n')
    log.close()


if __name__ == '__main__':
    main()
