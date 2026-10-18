#!/usr/bin/env python3
"""B0 output recorder (rospy). SKELETON - run ONLY inside ns_exec_b0.sh, BEFORE the feeder starts.

Subscribes to every final / legacy / visualization / diagnostic output of Main and writes DIR/recorder.jsonl with
one compact line per received message: key, topic, arrival mono_ns (CLOCK_MONOTONIC, comparable with the feeder),
ros_ns, header stamp_ns, frame_id and a type-specific summary. Sets the ROS param --ready-param once Main is a
connected publisher of every REQUIRED topic (or after --ready-timeout with the missing list recorded).
Subscribing to ALL typed topics is deliberate: Main serializes typed outputs only for subscribed topics
(FrameBoundaryPublisher.cpp:67-68,90-91), so exclusivity is tested in the worst case (both subscribed).
"""
import argparse
import json
import math
import os
import signal
import struct
import threading
import time

import rospy
from diagnostic_msgs.msg import DiagnosticArray
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import MarkerArray

try:
    from asmc_msgs.msg import LidarObjectArrayV2           # added by B0 (A)
except ImportError:                                         # pre-B0 build -> RED: v2 topics recorded as missing
    LidarObjectArrayV2 = None
from asmc_msgs.msg import LidarObjectArray
from Msg.msg import TrackedObjectArray

V2 = LidarObjectArrayV2 or rospy.AnyMsg
TOPICS = [  # key, topic, type, required-for-ready
    ('local_v2', '/lidar/local_object', V2, True),
    ('map_v2', '/lidar/map_object', V2, True),
    ('local_v1', '/perception/lidar/objects_local', LidarObjectArray, False),
    ('map_v1', '/perception/lidar/objects_map', LidarObjectArray, False),
    ('diag', '/perception/lidar/diagnostics', DiagnosticArray, True),
    ('last_lidar', '/last_lidar', TrackedObjectArray, True),
    ('tracked_map', '/lidar/tracked_objects_map', TrackedObjectArray, True),
    ('motion_debug', '/lidar/motion_debug', String, True),
    ('last_markers', '/last_lidar_markers', MarkerArray, True),
    ('map_markers', '/lidar/tracking_markers_map', MarkerArray, True),
    ('local_markers', '/lidar_tracking_markers', MarkerArray, False),   # G4: visualization only, not a gate
    ('object_boxes', '/lidar/object_boxes', MarkerArray, False),        # enable_sensor_fusion only (FusionLiDAR.cpp:21)
    ('object_cloud', '/lidar/object', rospy.AnyMsg, False),              # wall-clock stamps (verify_runtime.txt D3)
    ('debug_roi', '/lidar/debug_roi', rospy.AnyMsg, False),              # processing evidence (LidarProcess.cpp:67-72)
    ('debug_clustered_map', '/lidar/debug_clustered_map', rospy.AnyMsg, False),
    ('tf', '/tf', TFMessage, False),
    ('ctrl_discard', '/b0_test/discarded_ctrl_cmd_0', rospy.AnyMsg, False),   # Parser.cpp:499 remapped by the test launch
    ('ctrl_public', '/ctrl_cmd_0', rospy.AnyMsg, False),                       # must stay 0 (remap proof)
    ('planning_tracked', '/planning/input/tracked_objects', rospy.AnyMsg, False),
    ('planning_objects', '/planning/objects', rospy.AnyMsg, False),
]
ACTIONS = {0: 'add', 2: 'delete', 3: 'deleteall'}


def any_header(buff):
    sec, nsec = struct.unpack_from('<II', buff, 4)
    flen = struct.unpack_from('<I', buff, 12)[0]
    frame = buff[16:16 + flen].decode(errors='replace')
    return sec * 1000000000 + nsec, frame, 16 + flen


def summarize(key, msg):
    row = {}
    if isinstance(msg, rospy.AnyMsg):
        hdr = msg._connection_header or {}
        row.update(type=hdr.get('type'), md5=hdr.get('md5sum'), bytes=len(msg._buff))
        if key in ('object_cloud', 'debug_roi', 'debug_clustered_map') and len(msg._buff) >= 24:
            st, frame, off = any_header(msg._buff)
            h, w = struct.unpack_from('<II', msg._buff, off)
            row.update(stamp_ns=st, frame=frame, points=h * w)
        elif key in ('local_v2', 'map_v2'):
            row['v2_type_missing'] = True
        return row
    if key in ('local_v2', 'map_v2', 'local_v1', 'map_v1'):
        row.update(stamp_ns=msg.header.stamp.to_nsec(), frame=msg.header.frame_id, status=msg.status, reason=msg.reason,
                   prov=msg.stamp_provenance, n=len(msg.objects))
        objs = []
        for o in msg.objects:
            r = dict(id=o.track_id, c=[round(o.center.x, 3), round(o.center.y, 3), round(o.center.z, 3)], yaw=round(o.yaw, 4),
                     lwh=[round(o.length, 3), round(o.width, 3), round(o.height, 3)], cls=o.classification,
                     dv=o.dynamic_valid, dyn=o.is_dynamic, avv=o.absolute_velocity_valid, age=o.age)
            if hasattr(o, 'motion_state'):
                r.update(mv=o.motion_valid, ms=o.motion_state)
            objs.append(r)
        row['objects'] = objs
    elif key in ('last_lidar', 'tracked_map'):
        row.update(stamp_ns=msg.header.stamp.to_nsec(), frame=msg.header.frame_id, n=len(msg.objects),
                   objects=[dict(id=o.id, kph=round(o.speed_kph, 3), dyn=o.is_dynamic, age=o.age, cls=o.classification,
                                 c=[round(o.obb_center.x, 3), round(o.obb_center.y, 3), round(o.obb_center.z, 3)])
                            for o in msg.objects])
    elif key == 'motion_debug':
        try:
            j = json.loads(msg.data)
            row.update(stamp_ns=int(j['stamp_secs']) * 1000000000 + int(j['stamp_nsecs']), frame=j.get('frame_id'),
                       objects=[dict(id=o.get('id'), state=o.get('motion_state_code'), keep=o.get('final_keep'))
                                for o in j.get('objects', [])])
        except Exception as exc:                              # malformed JSON is itself a finding
            row.update(parse_error=str(exc)[:120])
    elif key in ('last_markers', 'map_markers', 'local_markers', 'object_boxes'):
        acts = {}
        for mk in msg.markers:
            acts[ACTIONS.get(mk.action, str(mk.action))] = acts.get(ACTIONS.get(mk.action, str(mk.action)), 0) + 1
        row.update(actions=acts, frames=sorted({mk.header.frame_id for mk in msg.markers}),
                   stamp_ns=msg.markers[0].header.stamp.to_nsec() if msg.markers else None)
    elif key == 'diag':
        for s in msg.status:
            if s.name == 'lidar_frame_boundary':
                row.update(stamp_ns=msg.header.stamp.to_nsec(), level=s.level, message=s.message,
                           values={v.key: v.value for v in s.values})
    elif key == 'tf':
        row['leveled'] = [t.header.stamp.to_nsec() for t in msg.transforms if t.child_frame_id == 'lidar_leveled']
        if not row['leveled']:
            return None
    return row


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--ready-param', default='/b0_recorder/ready')
    ap.add_argument('--ready-timeout', type=float, default=45.0)
    ap.add_argument('--only', default='', help='comma list of keys (subscriber-set variants); default all')
    ap.add_argument('--record-inputs', action='store_true', help='bag smoke: also log /gps,/imu,/Ego_topic,/velodyne_points')
    ap.add_argument('--optional-v2', action='store_true',
                    help='G4: production main.launch (common geometry OFF) has no V2/diagnostics publisher')
    a = ap.parse_args(rospy.myargv()[1:])
    os.makedirs(a.out, exist_ok=True)
    rospy.init_node('b0_recorder', disable_signals=True)
    out = open(os.path.join(a.out, 'recorder.jsonl'), 'w', buffering=1)
    lock = threading.Lock()
    only = set(filter(None, a.only.split(',')))

    def cb(msg, args):          # G4 fix: rospy passes callback_args as ONE argument (the (key, topic) tuple)
        key, topic = args
        t_mono, t_ros = time.monotonic_ns(), rospy.Time.now().to_nsec()
        try:
            row = summarize(key, msg)
        except Exception as exc:
            row = dict(summary_error=str(exc)[:160])
        if row is None:
            return
        row.update(k=key, topic=topic, mono_ns=t_mono, ros_ns=t_ros)
        with lock:
            out.write(json.dumps(row) + '\n')

    subs = []
    if a.record_inputs:   # bag smoke: reconstruct feeder-format input events so the same oracle can run
        from morai_msgs.msg import EgoVehicleStatus, GPSMessage
        from sensor_msgs.msg import Imu
        inp = open(os.path.join(a.out, 'inputs.jsonl'), 'w', buffering=1)
        counter = {'k': -1}

        def icb(msg, kind):
            t_mono, t_ros = time.monotonic_ns(), rospy.Time.now().to_nsec()
            if kind == 'lidar':
                st, frame, _ = any_header(msg._buff)
                counter['k'] += 1
                e = dict(ev='lidar', k=counter['k'], phase=0, i=counter['k'], mode='bag', stamp_ns=st, objects=[])
            else:
                st = msg.header.stamp.to_nsec()
                e = dict(ev=kind, k=counter['k'] + 1, phase=0, i=counter['k'] + 1, mode='bag', stamp_ns=st)
                if kind == 'imu':
                    q = msg.orientation
                    e.update(quat=[q.x, q.y, q.z, q.w], yaw=math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
                elif kind == 'gps':
                    e.update(lat=msg.latitude, lon=msg.longitude, alt=msg.altitude, east_offset=msg.eastOffset,
                             north_offset=msg.northOffset, status=msg.status)
                elif kind == 'can':
                    e.update(speed=math.sqrt(msg.velocity.x ** 2 + msg.velocity.y ** 2 + msg.velocity.z ** 2))
            e.update(mono_ns=t_mono, ros_ns=t_ros)
            with lock:
                inp.write(json.dumps(e, default=float) + '\n')
        subs.append(('in_imu', '/imu', rospy.Subscriber('/imu', Imu, icb, callback_args='imu', queue_size=500)))
        subs.append(('in_gps', '/gps', rospy.Subscriber('/gps', GPSMessage, icb, callback_args='gps', queue_size=500)))
        subs.append(('in_can', '/Ego_topic', rospy.Subscriber('/Ego_topic', EgoVehicleStatus, icb, callback_args='can', queue_size=500)))
        subs.append(('in_lidar', '/velodyne_points', rospy.Subscriber('/velodyne_points', rospy.AnyMsg, icb,
                                                                       callback_args='lidar', queue_size=50, buff_size=1 << 25)))
    for key, topic, typ, _ in TOPICS:
        if only and key not in only:
            continue
        subs.append((key, topic, rospy.Subscriber(topic, typ, cb, callback_args=(key, topic), queue_size=200,
                                                  buff_size=1 << 24, tcp_nodelay=True)))
    required = {k for k, _, _, r in TOPICS if r and (not only or k in only)}
    if a.optional_v2:
        required -= {'local_v2', 'map_v2', 'diag'}
    deadline = time.monotonic() + a.ready_timeout
    while time.monotonic() < deadline:
        missing = sorted(k for k, _, s in subs if k in required and s.get_num_connections() == 0)
        if not missing:
            break
        time.sleep(0.1)
    meta = dict(ev='ready', mono_ns=time.monotonic_ns(), missing_required=missing,
                v2_type_available=LidarObjectArrayV2 is not None,
                connections={k: s.get_num_connections() for k, _, s in subs})
    with lock:
        out.write(json.dumps(meta) + '\n')
    json.dump(meta, open(os.path.join(a.out, 'recorder_ready.json'), 'w'))
    rospy.set_param(a.ready_param, True)
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    while not stop.is_set() and not rospy.is_shutdown():
        stop.wait(0.2)
    with lock:
        out.write(json.dumps(dict(ev='end', mono_ns=time.monotonic_ns())) + '\n')
        out.close()
    rospy.signal_shutdown('done')


if __name__ == '__main__':
    main()
