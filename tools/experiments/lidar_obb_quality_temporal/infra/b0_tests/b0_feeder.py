#!/usr/bin/env python3
"""B0 synthetic sensor feeder with fault injection (rospy). SKELETON - run ONLY inside ns_exec_b0.sh.

usage: b0_feeder.py --catalogue fault_catalogue.json --scenario S01_gps_status0 --out DIR [--ready-param /b0_recorder/ready]
Publishes exactly what main.launch wires into Main (CameraMode/LiDARMode = ROS, setting.yaml:104,150):
  /velodyne_points  sensor_msgs/PointCloud2 (frame velodyne)       -> Parser.cpp:258
  /imu              sensor_msgs/Imu                                 -> Parser.cpp:554
  /gps              morai_msgs/GPSMessage                           -> Parser.cpp:386
  /Ego_topic        morai_msgs/EgoVehicleStatus (main.launch:125)   -> Parser.cpp:498
  /YOLO/BBox        Msg/Yolo_bbox_arr (only scenarios with yolo_stub)
Every stamp is on a 1 ms grid so ms-truncated legacy stamps associate exactly (verify_runtime.txt C7).
Writes DIR/feeder.jsonl: one line per published message (ev,k,phase,i,mode,stamp_ns,ros_ns,mono_ns) + 'truth' lines.
"""
import argparse
import json
import math
import os
import sys
import time

import rosgraph
import rospy
from morai_msgs.msg import EgoVehicleStatus, GPSMessage
from sensor_msgs.msg import Imu, PointCloud2, PointField

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import b0_scenario as bs  # noqa: E402

MAIN_INPUTS = ('/velodyne_points', '/imu', '/gps', '/Ego_topic')


def stamp(msg_header, ns):
    msg_header.stamp = rospy.Time(0) if ns == 0 else rospy.Time(ns // 1000000000, ns % 1000000000)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--catalogue', required=True)
    ap.add_argument('--scenario', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--ready-param', default='/b0_recorder/ready')
    ap.add_argument('--startup-timeout', type=float, default=60.0)
    ap.add_argument('--tail-s', type=float, default=2.5, help='keep running after the last scan (>1 diagnostics tick)')
    a = ap.parse_args(rospy.myargv()[1:])
    cat = json.load(open(a.catalogue))
    sc = next(s for s in cat['scenarios'] if s['id'] == a.scenario)
    d = dict(cat['defaults'])
    rospy.init_node('b0_feeder', disable_signals=True)
    if rospy.get_param('/use_sim_time', False):
        raise SystemExit('use_sim_time must be false: stamps are wall-clock ROS time like the MORAI bridge')
    pubs = dict(lidar=rospy.Publisher('/velodyne_points', PointCloud2, queue_size=5),
                imu=rospy.Publisher('/imu', Imu, queue_size=50),
                gps=rospy.Publisher('/gps', GPSMessage, queue_size=50),
                can=rospy.Publisher('/Ego_topic', EgoVehicleStatus, queue_size=50))
    if sc.get('yolo_stub'):
        from Msg.msg import Yolo_bbox_arr
        pubs['yolo'] = rospy.Publisher('/YOLO/BBox', Yolo_bbox_arr, queue_size=10)
    os.makedirs(a.out, exist_ok=True)
    log = open(os.path.join(a.out, 'feeder.jsonl'), 'w', buffering=1)
    # ---- wait for Main's subscriptions and the recorder (typed outputs publish only with subscribers,
    #      FrameBoundaryPublisher.cpp:67-68,90) ----
    master, deadline = rosgraph.Master('/b0_feeder'), time.monotonic() + a.startup_timeout
    while True:
        subs = {t: n for t, n in master.getSystemState()[1]}
        main_ok = all('/Main' in subs.get(t, []) for t in MAIN_INPUTS)
        conn_ok = all(p.get_num_connections() > 0 for k, p in pubs.items() if k != 'yolo')
        if main_ok and conn_ok and rospy.get_param(a.ready_param, False):
            break
        if time.monotonic() > deadline:
            log.write(json.dumps(dict(ev='startup_timeout', main_ok=main_ok, conn_ok=conn_ok)) + '\n')
            raise SystemExit(3)
        time.sleep(0.05)
    time.sleep(0.5)
    # G4: the synthetic clouds are generated before t0 (a cloud takes tens of ms in Python, longer than the
    # 15 ms between the last input sample and the LiDAR publication).
    scene_pre = bs.Scene(sc, d)
    clouds = {}
    for e in bs.plan(sc, d, 0):
        if e['ev'] == 'lidar':
            pts = scene_pre.cloud(e['objects'], e['t_truth'])
            clouds[e['k']] = (len(pts), bs.pack_cloud(pts))
    per_ns = int(round(d['period_s'] * 1e9))
    now = rospy.Time.now().to_nsec()
    t0 = ((now + 1000000000) // per_ns + 1) * per_ns          # aligned to the period grid (multiple of 1 ms)
    mono_at_t0 = time.monotonic_ns() + (t0 - now)
    log.write(json.dumps(dict(ev='start', scenario=sc['id'], t0_ns=t0, period_ns=per_ns, mono_at_t0=mono_at_t0)) + '\n')
    scene = bs.Scene(sc, d)
    fields = [PointField('x', 0, PointField.FLOAT32, 1), PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1), PointField('intensity', 12, PointField.FLOAT32, 1)]
    late = []
    for e in bs.plan(sc, d, t0):
        # G4: every sample at its own nominal time + 2 ms (b0_scenario.plan pub_ns), in publication order.
        target = mono_at_t0 + (e['pub_ns'] - t0)
        while time.monotonic_ns() < target:
            time.sleep(max(0.0, (target - time.monotonic_ns()) / 1e9 - 0.0005))
        if time.monotonic_ns() - target > 5000000:
            late.append((e['k'], e['ev'], (time.monotonic_ns() - target) // 1000000))
        if e['ev'] == 'lidar':
            n_points, data = clouds[e['k']]
            pts = range(n_points)
            m = PointCloud2()
            stamp(m.header, e['stamp_ns'])
            m.header.frame_id = 'velodyne'
            m.height, m.width, m.fields, m.is_bigendian = 1, n_points, fields, False
            m.point_step, m.row_step, m.is_dense = 16, 16 * n_points, True
            m.data = data
            log.write(json.dumps(dict(ev='truth', k=e['k'], stamp_ns=e['stamp_ns'], objects=e['truth'])) + '\n')
            pubs['lidar'].publish(m)
            e = dict(e, points=len(pts))
            e.pop('truth', None)
        elif e['ev'] == 'imu':
            m = Imu()
            stamp(m.header, e['stamp_ns'])
            m.header.frame_id = 'imu'
            m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w = e['quat']
            m.angular_velocity.z = e['yaw_rate']
            m.linear_acceleration.z = 9.81
            pubs['imu'].publish(m)
        elif e['ev'] == 'gps':
            m = GPSMessage()
            stamp(m.header, e['stamp_ns'])
            m.header.frame_id = 'gps'
            m.latitude, m.longitude, m.altitude = e['lat'], e['lon'], e['alt']
            m.eastOffset, m.northOffset, m.status = e['east_offset'], e['north_offset'], e['status']
            pubs['gps'].publish(m)
        elif e['ev'] == 'can':
            m = EgoVehicleStatus()
            stamp(m.header, e['stamp_ns'])
            yaw = scene.ego_pose(e['t_truth'])[2]
            m.velocity.x = e['speed']          # Main uses |velocity| (Parser.cpp:521-522)
            m.heading = math.degrees(yaw)
            pubs['can'].publish(m)
        elif e['ev'] == 'yolo':
            m = pubs['yolo'].data_class()
            stamp(m.header, e['stamp_ns'])
            m.objc = 0
            pubs['yolo'].publish(m)
        e['ros_ns'], e['mono_ns'] = rospy.Time.now().to_nsec(), time.monotonic_ns()
        e.pop('quat', None) if e['ev'] != 'imu' else None
        log.write(json.dumps(e, default=float) + '\n')
    time.sleep(a.tail_s)
    log.write(json.dumps(dict(ev='end', mono_ns=time.monotonic_ns(), late_publications_over_5ms=late)) + '\n')
    log.close()


if __name__ == '__main__':
    main()
