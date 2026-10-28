#!/usr/bin/env python3
"""Extract small, causally joined highway inputs from an existing ROS1 bag.

Uses ROS rosbag when available, otherwise rosbags. Raw LiDAR/images are never
decoded or copied. Each selected object arrival uses the last received ego;
this is not a reconstruction of the original planning timer. The reference JSON comes from highway_merge_preflight.
"""
import argparse
import json
import math
from pathlib import Path


def stamp(message):
    value = message.header.stamp
    return float(getattr(value, 'secs', getattr(value, 'sec', 0))) + 1e-9 * float(
        getattr(value, 'nsecs', getattr(value, 'nanosec', 0)))


def messages(path, state_topic, objects_topic):
    try:
        import rosbag
    except ImportError:
        from rosbags.rosbag1 import Reader
        from rosbags.typesys import Stores, get_typestore, get_types_from_msg
        with Reader(path) as bag:
            store = get_typestore(Stores.ROS1_NOETIC)
            connections = [c for c in bag.connections if c.topic in (state_topic, objects_topic)]
            for connection in connections:
                store.register(get_types_from_msg(connection.msgdef.data, connection.msgtype))
            for connection, time, raw in bag.messages(connections=connections):
                yield connection.topic, time * 1e-9, store.deserialize_ros1(raw, connection.msgtype)
    else:
        with rosbag.Bag(str(path)) as bag:
            for topic, message, time in bag.read_messages(topics=[state_topic, objects_topic]):
                yield topic, time.to_sec(), message


def object_row(obj):
    return {
        'id': int(obj.id), 'type': int(obj.type), 'motion_state': int(obj.motion_state),
        'x': float(obj.center.x), 'y': float(obj.center.y), 'z': float(obj.center.z),
        'yaw': float(obj.yaw), 'vx': float(obj.vx), 'vy': float(obj.vy),
        'yaw_rate': float(obj.yaw_rate), 'width': float(obj.width),
        'length': float(obj.length), 'height': float(obj.height),
        'corners': [[float(p.x), float(p.y), float(p.z)] for p in obj.obb_corners],
    }


def extract(path, reference, output, state_topic, objects_topic, hz):
    import numpy as np
    points = np.asarray(reference['points'], dtype=float)
    spans = [(z['start_s'] - 50, z['completion_s'] + 30) for z in reference['zones']]
    current_objects = None
    current_state = None
    objects_received = None
    previous = -math.inf
    frames = []
    for topic, received, message in messages(path, state_topic, objects_topic):
        if topic == state_topic:
            current_state = message
            continue
        if topic != objects_topic or current_state is None or (hz > 0 and received - previous < 1.0 / hz):
            continue
        current_objects, objects_received = message, received
        message = current_state
        if not message.valid or message.header.frame_id != 'map':
            continue
        x, y = float(message.x), float(message.y)
        if not all(math.isfinite(v) for v in (x, y, message.yaw, message.speed)):
            continue
        index = int(np.argmin((points[:, 0] - x) ** 2 + (points[:, 1] - y) ** 2))
        station = float(points[index, 2])
        if not any(start <= station <= end for start, end in spans):
            continue
        previous = received
        frames.append({
            't': received, 'ego_stamp': stamp(message), 'object_stamp': stamp(current_objects),
            'object_receipt_age_s': received - objects_received,
            'object_frame': current_objects.header.frame_id,
            'ego': {'x': x, 'y': y, 'yaw': float(message.yaw),
                    'speed': float(message.speed), 'acceleration': float(message.acceleration),
                    'yaw_rate': float(message.yaw_rate)},
            'objects': [object_row(obj) for obj in current_objects.objects],
        })
    output.write_text(json.dumps(frames, allow_nan=False), encoding='utf-8')
    return len(frames)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('reference', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--state-topic', default='/planning/vehicle/state')
    parser.add_argument('--objects-topic', default='/planning/objects')
    parser.add_argument('--hz', type=float, default=0.0)
    args = parser.parse_args()
    if not math.isfinite(args.hz) or args.hz < 0:
        parser.error('--hz must be non-negative; 0 keeps all object arrivals')
    count = extract(args.bag, json.loads(args.reference.read_text()), args.output,
                    args.state_topic, args.objects_topic, args.hz)
    print('%d causal highway frames; %d bytes' % (count, args.output.stat().st_size))
    return 0 if count else 1


if __name__ == '__main__':
    raise SystemExit(main())
