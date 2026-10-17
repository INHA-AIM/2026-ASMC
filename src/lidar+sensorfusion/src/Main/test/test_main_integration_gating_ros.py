#!/usr/bin/env python3
"""B0 runtime gate test: the REAL Main node through main_integration.launch (main_integration_gating.test,
planner_udp_enabled:=false), fed synthetic LiDAR/IMU/GPS/CAN at 10 Hz.

Each sample is published 2 ms after its source stamp, like the vehicle bridge: IMU at S-40/S-5 ms, GPS at
S-30 ms, CAN at S-5 ms, LiDAR at S+12 ms (S on a 100 ms grid). Phases: healthy, GPS status 0 (denied),
healthy (3-sample recovery), IMU silent (stale local input), healthy.

Checked per LiDAR stamp: /lidar/local_object is the continuous valid-local output and /lidar/map_object is
additionally published only with a valid map pose; no map_v2 and no legacy output
(/last_lidar, /lidar/tracked_objects_map) during the GPS denial; no V2 output at all while
the IMU is stale; the first two scans after the denial are local-only; legacy frames only on map_v2 scans
whose objects are all motion_valid; frames base_link/map; no zero-center objects. Scans the node skipped
under load are tolerated by counting, never by relaxing the per-stamp rules.
Must run inside an isolated network namespace with a private master (see the B0 harness)."""
import math
import struct
import time
import unittest

import rospy
import rostest
from asmc_msgs.msg import LidarObjectArrayV2
from diagnostic_msgs.msg import DiagnosticArray
from morai_msgs.msg import EgoVehicleStatus, GPSMessage
from Msg.msg import TrackedObjectArray
from sensor_msgs.msg import Imu, PointCloud2, PointField

MS = 1000000
ORIGIN_E, ORIGIN_N = 302595.0, 4124145.0                 # main.launch drivable origin / HD map origin
EGO_XY, EGO_YAW = (-100.0, 150.0), 0.3
LIDAR_IN_BASE, GPS_IN_BASE = (0.80, 0.0, 1.45), (0.35, 0.0, 1.25)
BOXES = [((22.0, 3.4), (4.0, 1.8, 1.5)), ((12.0, -3.0), (4.2, 1.8, 1.5))]   # ego frame, l/w/h
PHASES = [('healthy', 30, {}), ('gps_denied', 10, {'gps_status': 0}), ('recovery', 15, {}),
          ('imu_silent', 6, {'imu_silent': True}), ('healthy_end', 15, {})]

# UTM zone 52 (Krueger series, as GeographicLib in Parser.cpp) for synthetic GPS fixes.
_F = 1 / 298.257223563
_N = _F / (2 - _F)
_A = 6378137.0 / (1 + _N) * (1 + _N ** 2 / 4 + _N ** 4 / 64)
_AL = (_N / 2 - 2 * _N ** 2 / 3 + 5 * _N ** 3 / 16, 13 * _N ** 2 / 48 - 3 * _N ** 3 / 5, 61 * _N ** 3 / 240)
_BE = (_N / 2 - 2 * _N ** 2 / 3 + 37 * _N ** 3 / 96, _N ** 2 / 48 + _N ** 3 / 15, 17 * _N ** 3 / 480)
_DE = (2 * _N - 2 * _N ** 2 / 3 - 2 * _N ** 3, 7 * _N ** 2 / 3 - 8 * _N ** 3 / 5, 56 * _N ** 3 / 15)
_K0, _LON0 = 0.9996, math.radians(129.0)


def utm_forward(lat, lon):
    phi, lam = math.radians(lat), math.radians(lon) - _LON0
    c = 2 * math.sqrt(_N) / (1 + _N)
    t = math.sinh(math.atanh(math.sin(phi)) - c * math.atanh(c * math.sin(phi)))
    xi, eta = math.atan2(t, math.cos(lam)), math.atanh(math.sin(lam) / math.sqrt(1 + t * t))
    e = 500000 + _K0 * _A * (eta + sum(a * math.cos(2 * (j + 1) * xi) * math.sinh(2 * (j + 1) * eta) for j, a in enumerate(_AL)))
    n = _K0 * _A * (xi + sum(a * math.sin(2 * (j + 1) * xi) * math.cosh(2 * (j + 1) * eta) for j, a in enumerate(_AL)))
    return e, n


def utm_inverse(e, n):
    xi, eta = n / (_K0 * _A), (e - 500000) / (_K0 * _A)
    xp = xi - sum(b * math.sin(2 * (j + 1) * xi) * math.cosh(2 * (j + 1) * eta) for j, b in enumerate(_BE))
    ep = eta - sum(b * math.cos(2 * (j + 1) * xi) * math.sinh(2 * (j + 1) * eta) for j, b in enumerate(_BE))
    chi = math.asin(math.sin(xp) / math.cosh(ep))
    lat = math.degrees(chi + sum(d * math.sin(2 * (j + 1) * chi) for j, d in enumerate(_DE)))
    lon = math.degrees(_LON0 + math.atan2(math.sinh(ep), math.cos(xp)))
    for _ in range(3):
        fe, fn = utm_forward(lat, lon)
        lat += (n - fn) / 111132.0
        lon += (e - fe) / (111320.0 * math.cos(math.radians(lat)))
    return lat, lon


def static_cloud():
    """Ground grid (road z=0) and two boxes, in the LiDAR frame of a level, stationary ego."""
    pts = []
    for iu in range(77):
        for iv in range(61):
            pts.append((2.0 + 0.5 * iu, -15.0 + 0.5 * iv, 0.0))
    for (cx, cy), (l, w, h) in BOXES:
        n_l, n_w, n_h = int(l / 0.15) + 1, int(w / 0.15) + 1, int(h / 0.15) + 1
        xs = [cx - l / 2 + l * i / (n_l - 1) for i in range(n_l)]
        ys = [cy - w / 2 + w * i / (n_w - 1) for i in range(n_w)]
        zs = [h * i / (n_h - 1) for i in range(n_h)]
        pts += [(x, y, h) for x in xs for y in ys]
        pts += [(x, s, z) for x in xs for z in zs for s in (cy - w / 2, cy + w / 2)]
        pts += [(s, y, z) for y in ys for z in zs for s in (cx - l / 2, cx + l / 2)]
    data = b''.join(struct.pack('<ffff', x - LIDAR_IN_BASE[0], y - LIDAR_IN_BASE[1], z - LIDAR_IN_BASE[2], 1.0)
                    for x, y, z in pts)
    return len(pts), data


def quaternion(yaw):
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)


class MainIntegrationGatingTest(unittest.TestCase):
    def setUp(self):
        self.received = {'local_v2': [], 'map_v2': [], 'last_lidar': [], 'tracked_map': [], 'diag': []}
        self.subs = [
            rospy.Subscriber('/lidar/local_object', LidarObjectArrayV2, self.received['local_v2'].append, queue_size=100),
            rospy.Subscriber('/lidar/map_object', LidarObjectArrayV2, self.received['map_v2'].append, queue_size=100),
            rospy.Subscriber('/last_lidar', TrackedObjectArray, self.received['last_lidar'].append, queue_size=100),
            rospy.Subscriber('/lidar/tracked_objects_map', TrackedObjectArray, self.received['tracked_map'].append, queue_size=100),
            rospy.Subscriber('/perception/lidar/diagnostics', DiagnosticArray, self.received['diag'].append, queue_size=20)]
        self.pubs = dict(lidar=rospy.Publisher('/velodyne_points', PointCloud2, queue_size=5),
                         imu=rospy.Publisher('/imu', Imu, queue_size=50),
                         gps=rospy.Publisher('/gps', GPSMessage, queue_size=50),
                         can=rospy.Publisher('/Ego_topic', EgoVehicleStatus, queue_size=50))

    def wait_for_graph(self, timeout=60.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if all(p.get_num_connections() > 0 for p in self.pubs.values()) and \
                    all(s.get_num_connections() > 0 for s in self.subs):
                time.sleep(0.5)
                return True
            time.sleep(0.05)
        return False

    def test_continuous_local_and_valid_map_gps_denial_recovery_and_legacy_gate(self):
        self.assertTrue(self.wait_for_graph(), 'Main did not connect to the synthetic inputs and outputs')
        n_points, cloud = static_cloud()
        fields = [PointField('x', 0, PointField.FLOAT32, 1), PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1), PointField('intensity', 12, PointField.FLOAT32, 1)]
        gx = EGO_XY[0] + math.cos(EGO_YAW) * GPS_IN_BASE[0]
        gy = EGO_XY[1] + math.sin(EGO_YAW) * GPS_IN_BASE[0]
        lat, lon = utm_inverse(ORIGIN_E + gx, ORIGIN_N + gy)
        period = 100 * MS
        now = rospy.Time.now().to_nsec()
        t0 = (now // period + 15) * period
        mono0 = time.monotonic_ns() + (t0 - now)
        phase_of = {}
        events = []
        k = 0
        for name, scans, faults in PHASES:
            for _ in range(scans):
                s = t0 + k * period
                phase_of[s // MS] = name
                for off in (-40, -5):
                    events.append((s + (off + 2) * MS, 'imu', s + off * MS, faults))
                events.append((s - 28 * MS, 'gps', s - 30 * MS, faults))
                events.append((s - 3 * MS, 'can', s - 5 * MS, faults))
                events.append((s + 12 * MS, 'lidar', s, faults))
                k += 1
        events.sort(key=lambda e: e[0])
        for pub_ns, kind, stamp_ns, faults in events:
            target = mono0 + (pub_ns - t0)
            while time.monotonic_ns() < target:
                time.sleep(max(0.0, (target - time.monotonic_ns()) / 1e9 - 0.0005))
            stamp = rospy.Time(stamp_ns // 1000000000, stamp_ns % 1000000000)
            if kind == 'imu':
                if faults.get('imu_silent'):
                    continue
                m = Imu()
                m.header.stamp, m.header.frame_id = stamp, 'imu'
                m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w = quaternion(EGO_YAW)
                m.linear_acceleration.z = 9.81
                self.pubs['imu'].publish(m)
            elif kind == 'gps':
                m = GPSMessage()
                m.header.stamp, m.header.frame_id = stamp, 'gps'
                m.latitude, m.longitude, m.altitude = lat, lon, GPS_IN_BASE[2]
                m.eastOffset, m.northOffset = ORIGIN_E, ORIGIN_N
                m.status = faults.get('gps_status', 1)
                self.pubs['gps'].publish(m)
            elif kind == 'can':
                m = EgoVehicleStatus()
                m.header.stamp = stamp
                m.heading = math.degrees(EGO_YAW)
                self.pubs['can'].publish(m)
            else:
                m = PointCloud2()
                m.header.stamp, m.header.frame_id = stamp, 'velodyne'
                m.height, m.width, m.fields, m.is_bigendian = 1, n_points, fields, False
                m.point_step, m.row_step, m.is_dense, m.data = 16, 16 * n_points, True, cloud
                self.pubs['lidar'].publish(m)
        time.sleep(2.5)                      # last scans and one more 1 Hz diagnostics tick

        def stamps(key):
            return [m.header.stamp.to_nsec() // MS for m in self.received[key]]
        local, mapped = stamps('local_v2'), stamps('map_v2')
        legacy = set(stamps('last_lidar')) | set(stamps('tracked_map'))
        in_phase = {name: [ms for ms, p in sorted(phase_of.items()) if p == name] for name, _, _ in PHASES}
        summary = {key: len(v) for key, v in self.received.items()}
        rospy.loginfo('received %s', summary)
        # Every map frame has the same-scan local frame; local is the primary valid-scan output.
        self.assertTrue(set(mapped) <= set(local), 'map output existed without same-stamp local output')
        self.assertEqual(len(local), len(set(local)))
        self.assertEqual(len(mapped), len(set(mapped)))
        for ms in local + mapped:
            self.assertIn(ms, phase_of, 'V2 stamp that is not a fed scan')
        # GPS denial: local_v2 only, never map_v2 or legacy.
        denied = set(in_phase['gps_denied'])
        self.assertFalse(denied & set(mapped))
        self.assertFalse(denied & legacy)
        self.assertGreaterEqual(len(denied & set(local)), 7, summary)
        # Recovery needs three consecutive good GPS samples: the first two scans are not map.
        recovery = in_phase['recovery']
        self.assertFalse(set(recovery[:2]) & set(mapped))
        self.assertFalse(set(recovery[:2]) & legacy)
        self.assertGreaterEqual(len(set(recovery[:2]) & set(local)), 1, summary)
        self.assertGreaterEqual(len(set(recovery[2:]) & set(mapped)), 9, summary)
        # IMU silent: from the second scan on the last IMU sample is older than 0.12 s, so the local input is
        # invalid: nothing on V2 at all (no empty or dummy array), no legacy.
        invalid = set(in_phase['imu_silent'][1:])
        self.assertFalse(invalid & (set(local) | set(mapped)))
        self.assertFalse(invalid & legacy)
        # Map output and legacy available in the healthy phases.
        self.assertGreaterEqual(len(set(in_phase['healthy']) & set(mapped)), 20, summary)
        self.assertGreaterEqual(len(set(in_phase['healthy_end'][2:]) & set(mapped)), 9, summary)
        self.assertGreaterEqual(len(set(in_phase['healthy']) & set(local)), 20, summary)
        self.assertGreater(len(legacy), 0, summary)
        # Legacy only with map_v2 and the frame-wide motion gate (no warm-up object on the legacy wire).
        by_map = {m.header.stamp.to_nsec() // MS: m for m in self.received['map_v2']}
        for ms in legacy:
            self.assertIn(ms, by_map, 'legacy frame without map_v2')
            self.assertTrue(all(o.motion_valid for o in by_map[ms].objects), 'legacy frame while a track warms up')
        warmup = [ms for ms, m in by_map.items() if any(not o.motion_valid for o in m.objects)]
        self.assertTrue(warmup, 'no warm-up frame observed (startup should have some)')
        self.assertFalse(set(warmup) & legacy)
        # Frames, status, motion encoding and no zero-coordinate dummy objects.
        for key, frame in (('local_v2', 'base_link'), ('map_v2', 'map')):
            for m in self.received[key]:
                self.assertEqual(frame, m.header.frame_id)
                self.assertIn(m.status, (LidarObjectArrayV2.OBSERVED, LidarObjectArrayV2.OBSERVED_EMPTY))
                self.assertTrue(m.stamp_provenance)
                for o in m.objects:
                    self.assertFalse(o.center.x == 0.0 and o.center.y == 0.0 and o.center.z == 0.0)
                    self.assertEqual(o.dynamic_valid, o.motion_valid)
                    self.assertEqual(o.is_dynamic, o.motion_state == o.MOTION_DYNAMIC)
                    if not o.motion_valid:
                        self.assertEqual(o.MOTION_UNKNOWN, o.motion_state)
        # Main's own counters agree: none frames were counted, nothing was suppressed silently.
        values = {}
        for d in self.received['diag']:
            for status in d.status:
                if status.name == 'lidar_frame_boundary':
                    values = {v.key: v.value for v in status.values}
        self.assertGreaterEqual(int(values.get('v2_none_frames', '0')), 4, values)
        self.assertEqual(values.get('v2_stamp_errors'), '0')
        self.assertEqual(values.get('roll_leveling'), 'true')


if __name__ == '__main__':
    rospy.init_node('main_integration_gating_test')
    rostest.rosrun('Main', 'main_integration_gating', MainIntegrationGatingTest)
