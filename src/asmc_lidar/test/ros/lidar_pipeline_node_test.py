#!/usr/bin/env python3
"""rostest node for lidar_pipeline_node, synthetic inputs only.

A background thread publishes a box object on a flat ground plane (PointCloud2,
10 Hz), sensor_msgs/Imu and morai EgoVehicleStatus (50 Hz), morai GPSMessage
(10 Hz) and, per phase, canonical camera status/observations (10 Hz). The test
methods run in name order against the same node and switch the inputs:

  01 A normal: frames, stamps, observed box, map == map_from_base * local
  02 B/C GPS invalid (status 0) -> map POSE_UNAVAILABLE, local keeps the track;
     recovery needs >= 3 consecutive normal samples, invalid positions unused
  03 B/C GPS silent -> gps_stale, then the same recovery rule
  04 E camera: vehicle after 5 observations (raw label kept), empty, invalid,
     STANDBY, session reset, duplicate/regressing observations, camera stopped
  05/06 D IMU stale / invalid quaternion -> INPUT_INVALID with no objects
  07/08 F isolation (master API) and the operational-output guard

The synthetic camera input is an interface/synchronization fixture, not YOLO
accuracy.
"""
import hashlib
import json
import math
import subprocess
import threading
import time
import unittest
import uuid

import numpy as np
import rosgraph
import roslib.packages
import rospy
import rostest
import yaml
from asmc_msgs.msg import (CameraObjectBox, CameraObjectObservation, InferenceStatus,
                           LidarObjectArray)
from diagnostic_msgs.msg import DiagnosticArray
from morai_msgs.msg import EgoVehicleStatus, GPSMessage
from sensor_msgs.msg import Imu, PointCloud2, PointField
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

PKG = 'asmc_lidar'
OBSERVED, OBSERVED_EMPTY, INPUT_INVALID, POSE_UNAVAILABLE = 1, 2, 3, 4

# HD-map origin (config/contracts/hd_map_topic_contract_v1.yaml) = GPSMessage offsets.
MAP_ORIGIN = (302595.0, 4124145.0)
ANTENNA_LOCAL = (25.0, 40.0)   # inside Main's GPS sanity bounds
ANTENNA_ALTITUDE = 0.5
GPS_IN_BASE = (float(np.float32(0.35)), 0.0, float(np.float32(1.25)))  # setting.yaml Vehicle.GPS (float)
WHEEL_RADIUS = 0.36            # base_link height above the road (cameras_v3.yaml ground_plane)
BOX_CENTER = (10.0, 0.0)       # base_link
BOX_SIZE = (4.0, 1.8)          # x, y
BOX_Z = (-WHEEL_RADIUS + 0.4, -WHEEL_RADIUS + 1.9)
OPERATIONAL = ('/last_lidar', '/lidar_nogps', '/YOLO/BBox', '/perception/lidar/objects_local',
               '/perception/lidar/objects_map', '/perception/lidar/diagnostics',
               '/perception/lidar/objects')


# --- UTM 52N (Krueger series, 6th order; GeographicLib's default accuracy) ----
def utm_forward(lat_deg, lon_deg, zone=52):
    a, f, k0 = 6378137.0, 1 / 298.257223563, 0.9996
    n = f / (2 - f)
    big_a = a / (1 + n) * (1 + n ** 2 / 4 + n ** 4 / 64 + n ** 6 / 256)
    alpha = (n / 2 - 2 * n ** 2 / 3 + 5 * n ** 3 / 16 + 41 * n ** 4 / 180 - 127 * n ** 5 / 288
             + 7891 * n ** 6 / 37800,
             13 * n ** 2 / 48 - 3 * n ** 3 / 5 + 557 * n ** 4 / 1440 + 281 * n ** 5 / 630
             - 1983433 * n ** 6 / 1935360,
             61 * n ** 3 / 240 - 103 * n ** 4 / 140 + 15061 * n ** 5 / 26880 + 167603 * n ** 6 / 181440,
             49561 * n ** 4 / 161280 - 179 * n ** 5 / 168 + 6601661 * n ** 6 / 7257600,
             34729 * n ** 5 / 80640 - 3418889 * n ** 6 / 1995840,
             212378941 * n ** 6 / 319334400)
    lat = math.radians(lat_deg)
    dlon = math.radians(lon_deg - (zone * 6 - 183))
    c = 2 * math.sqrt(n) / (1 + n)
    t = math.sinh(math.atanh(math.sin(lat)) - c * math.atanh(c * math.sin(lat)))
    xi = math.atan2(t, math.cos(dlon))
    eta = math.atanh(math.sin(dlon) / math.sqrt(1 + t * t))
    east = eta + sum(alpha[j - 1] * math.cos(2 * j * xi) * math.sinh(2 * j * eta) for j in range(1, 7))
    north = xi + sum(alpha[j - 1] * math.sin(2 * j * xi) * math.cosh(2 * j * eta) for j in range(1, 7))
    return 500000.0 + k0 * big_a * east, k0 * big_a * north


def utm_inverse(east, north):
    lat, lon = 37.24, 126.77
    for _ in range(20):
        e0, n0 = utm_forward(lat, lon)
        h = 1e-7
        e1, n1 = utm_forward(lat + h, lon)
        e2, n2 = utm_forward(lat, lon + h)
        jac = np.array([[(e1 - e0) / h, (e2 - e0) / h], [(n1 - n0) / h, (n2 - n0) / h]])
        step = np.linalg.solve(jac, np.array([east - e0, north - n0]))
        lat, lon = lat + step[0], lon + step[1]
        if abs(step[0]) + abs(step[1]) < 1e-13:
            break
    return lat, lon


def rpy(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch),
                              math.cos(yaw), math.sin(yaw))
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return rz @ ry @ rx


def xyz(point):
    return np.array([point.x, point.y, point.z])


class LidarPipelineNodeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.node = rospy.get_param('~node_name', '/lidar_pipeline')
        cls.pipeline_config = rospy.get_param('~pipeline_config')
        cls.expect_tf = rospy.get_param('~expect_tf', False)
        mount = yaml.safe_load(open(rospy.get_param('~mount_config')))['sensor_pose_base_link']
        cls.mount = np.array([mount['x_m'], mount['y_m'], mount['z_m']])
        mount_rotation = rpy(*(math.radians(mount[k]) for k in ('roll_deg', 'pitch_deg', 'yaw_deg')))
        cls.mount_rotation = mount_rotation

        # GPS: a fixed antenna position; expected map_from_base (speed 0, level IMU, yaw 0).
        cls.lat, cls.lon = utm_inverse(MAP_ORIGIN[0] + ANTENNA_LOCAL[0], MAP_ORIGIN[1] + ANTENNA_LOCAL[1])
        east, north = utm_forward(cls.lat, cls.lon)
        cls.antenna = np.array([east - MAP_ORIGIN[0], north - MAP_ORIGIN[1], ANTENNA_ALTITUDE])
        e0, n0 = utm_forward(cls.lat - 1e-5, cls.lon)
        e1, n1 = utm_forward(cls.lat + 1e-5, cls.lon)
        offset = math.atan2(n1 - n0, e1 - e0) - math.pi / 2  # Main GridImu
        cls.yaw_grid = float(np.float32(math.remainder(0.0 + offset, 2 * math.pi)))
        cls.map_rotation = rpy(0.0, 0.0, cls.yaw_grid)
        cls.map_translation = cls.antenna - cls.map_rotation @ np.array(GPS_IN_BASE)
        cls.bad_lat, cls.bad_lon = utm_inverse(MAP_ORIGIN[0] + ANTENNA_LOCAL[0] + 50.0,
                                               MAP_ORIGIN[1] + ANTENNA_LOCAL[1])

        cls.cloud = cls.make_cloud()
        cls.load_camera(rospy.get_param('~camera_calibration_config'))

        cls.lock = threading.Lock()
        cls.state = dict(imu='normal', gps='normal', camera=None)
        cls.scans, cls.gps_log, cls.imu_log, cls.obs_log = [], [], [], []
        cls.local, cls.map, cls.diagnostics, cls.tf, cls.traces = [], [], [], [], []
        rospy.Subscriber('/migration/lidar/objects_local', LidarObjectArray, cls.store(cls.local), queue_size=50)
        rospy.Subscriber('/migration/lidar/objects_map', LidarObjectArray, cls.store(cls.map), queue_size=50)
        rospy.Subscriber('/migration/lidar/diagnostics', DiagnosticArray, cls.store(cls.diagnostics),
                         queue_size=5)
        rospy.Subscriber('/tf', TFMessage, cls.store(cls.tf), queue_size=50)
        cls.pub = dict(
            points=rospy.Publisher('/velodyne_points', PointCloud2, queue_size=2),
            imu=rospy.Publisher('/imu', Imu, queue_size=10),
            can=rospy.Publisher('/Ego_topic', EgoVehicleStatus, queue_size=10),
            gps=rospy.Publisher('/gps', GPSMessage, queue_size=10),
            status=rospy.Publisher('/perception/camera/objects/status', InferenceStatus, queue_size=5),
            obs=rospy.Publisher('/perception/camera/objects/observations', CameraObjectObservation,
                                queue_size=10))
        cls.running = True
        cls.thread = threading.Thread(target=cls.publish_loop, daemon=True)
        cls.thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.running = False
        cls.thread.join(timeout=2.0)

    # ------------------------------------------------------------- fixtures --
    @classmethod
    def store(cls, target):
        def callback(msg):
            with cls.lock:
                target.append(msg)
        return callback

    @classmethod
    def make_cloud(cls):
        """Ground plane and a closed box, sensor frame, x/y/z/intensity float32."""
        ground_z = -WHEEL_RADIUS - cls.mount[2]
        gx, gy = np.meshgrid(np.arange(-8.0, 30.01, 0.4), np.arange(-12.0, 12.01, 0.4))
        ground = np.stack([gx.ravel(), gy.ravel(), np.full(gx.size, ground_z)], axis=1)

        def span(lo, hi):
            return np.linspace(lo, hi, int(math.ceil((hi - lo) / 0.15)) + 1)

        x0, x1 = BOX_CENTER[0] - BOX_SIZE[0] / 2, BOX_CENTER[0] + BOX_SIZE[0] / 2
        y0, y1 = BOX_CENTER[1] - BOX_SIZE[1] / 2, BOX_CENTER[1] + BOX_SIZE[1] / 2
        z0, z1 = BOX_Z
        faces = []
        for x in (x0, x1):
            yy, zz = np.meshgrid(span(y0, y1), span(z0, z1))
            faces.append(np.stack([np.full(yy.size, x), yy.ravel(), zz.ravel()], axis=1))
        for y in (y0, y1):
            xx, zz = np.meshgrid(span(x0, x1), span(z0, z1))
            faces.append(np.stack([xx.ravel(), np.full(xx.size, y), zz.ravel()], axis=1))
        xx, yy = np.meshgrid(span(x0, x1), span(y0, y1))
        faces.append(np.stack([xx.ravel(), yy.ravel(), np.full(xx.size, z1)], axis=1))
        box_base = np.concatenate(faces)
        # base -> sensor: p_sensor = R_mount^T (p_base - t_mount)
        box = (box_base - cls.mount) @ cls.mount_rotation
        points = np.concatenate([ground, box])
        data = np.concatenate([points, np.full((len(points), 1), 10.0)], axis=1).astype(np.float32)
        cloud = PointCloud2()
        cloud.header.frame_id = 'velodyne'
        cloud.height, cloud.width = 1, len(data)
        cloud.fields = [PointField(name, 4 * i, PointField.FLOAT32, 1)
                        for i, name in enumerate(('x', 'y', 'z', 'intensity'))]
        cloud.is_bigendian, cloud.point_step = False, 16
        cloud.row_step, cloud.is_dense = 16 * len(data), True
        cloud.data = data.tobytes()
        return cloud

    @classmethod
    def load_camera(cls, path):
        """Front calibration id (camera_calibration.cpp digest) and the box's image rectangle."""
        document = yaml.safe_load(open(path))
        row = document['cameras']['front']
        cls.calibration_id = 'sha256:' + hashlib.sha256(json.dumps(
            dict(camera=row, source_sha256=document['source_sha256']), sort_keys=True,
            separators=(',', ':'), allow_nan=False).encode()).hexdigest()
        cls.image = (row['width'], row['height'])
        focal = row['width'] / (2 * math.tan(math.radians(row['horizontal_fov_deg']) / 2))
        rotation = rpy(*(math.radians(v) for v in row['rotation_deg']))
        translation = np.array(row['translation_m'])
        pixels = []
        for x in (BOX_CENTER[0] - BOX_SIZE[0] / 2, BOX_CENTER[0] + BOX_SIZE[0] / 2):
            for y in (BOX_CENTER[1] - BOX_SIZE[1] / 2, BOX_CENTER[1] + BOX_SIZE[1] / 2):
                for z in BOX_Z:
                    cam = rotation.T @ (np.array([x, y, z]) - translation)
                    pixels.append((focal * -cam[1] / cam[0] + row['width'] / 2,
                                   focal * -cam[2] / cam[0] + row['height'] / 2))
        pixels = np.array(pixels)
        cls.box_pixels = (max(0.0, pixels[:, 0].min()), max(0.0, pixels[:, 1].min()),
                          min(float(row['width']), pixels[:, 0].max()),
                          min(float(row['height']), pixels[:, 1].max()))

    @classmethod
    def observation(cls, stamp, camera, content):
        msg = CameraObjectObservation()
        d = msg.detections
        d.header.stamp, d.header.frame_id = stamp, 'camera_front'
        d.image_width, d.image_height = cls.image
        d.valid = content != 'invalid'
        if content == 'vehicle':
            box = CameraObjectBox()
            box.xmin, box.ymin, box.xmax, box.ymax = cls.box_pixels
            box.class_name, box.source_class_name = 'vehicle', 'car'
            box.confidence, box.confidence_valid = 0.9, True
            d.boxes = [box]
        msg.producer_session_id, msg.generation = camera['session'], camera['generation']
        msg.calibration_id = cls.calibration_id
        msg.model_id, msg.config_id = 'synthetic_fixture_model', 'synthetic_fixture_config'
        return msg

    @classmethod
    def publish_loop(cls):
        rate = rospy.Rate(100)
        tick = 0
        while cls.running and not rospy.is_shutdown():
            with cls.lock:
                state = dict(cls.state)
                camera = dict(state['camera']) if state['camera'] else None
            now = rospy.Time.now()
            if tick % 2 == 0 and state['imu'] != 'stopped':
                imu = Imu()
                imu.header.stamp, imu.header.frame_id = now, 'imu'
                imu.orientation.w = 1.0 if state['imu'] == 'normal' else 0.5
                imu.linear_acceleration.z = 9.81
                cls.pub['imu'].publish(imu)
                with cls.lock:
                    cls.imu_log.append((now.to_nsec(), state['imu']))
            if tick % 2 == 0:
                can = EgoVehicleStatus()
                can.header.stamp = now
                cls.pub['can'].publish(can)
            if tick % 10 == 0 and state['gps'] != 'silent':
                gps = GPSMessage()
                gps.header.stamp = now
                normal = state['gps'] == 'normal'
                gps.latitude, gps.longitude = (cls.lat, cls.lon) if normal else (cls.bad_lat, cls.bad_lon)
                gps.altitude = ANTENNA_ALTITUDE
                gps.eastOffset, gps.northOffset = MAP_ORIGIN
                gps.status = 1 if normal else 0
                cls.pub['gps'].publish(gps)
                with cls.lock:
                    cls.gps_log.append((now.to_nsec(), state['gps']))
            if tick % 10 == 0 and camera and camera.get('status', True):
                status = InferenceStatus()
                status.header.stamp = now
                status.model = 'yolo'
                status.requested_mode = status.mode = camera['mode']
                status.generation = camera['generation']
                status.producer_session_id = camera['session']
                status.managed = status.loaded = status.warmed_up = True
                status.prepared = status.pipeline_ready = camera['mode'] == InferenceStatus.ACTIVE
                cls.pub['status'].publish(status)
            if tick % 10 == 5:
                if camera and camera.get('content'):
                    msg = cls.observation(now, camera, camera['content'])
                    cls.pub['obs'].publish(msg)
                    if camera.get('duplicate'):
                        cls.pub['obs'].publish(msg)  # identical observation
                        old = cls.observation(now - rospy.Duration(0.03), camera, camera['content'])
                        cls.pub['obs'].publish(old)  # regressing observation
                    with cls.lock:
                        cls.obs_log.append((now.to_nsec(), camera['tag']))
                cls.cloud.header.stamp = now
                cls.pub['points'].publish(cls.cloud)
                with cls.lock:
                    cls.scans.append(now.to_nsec())
            tick += 1
            rate.sleep()

    # -------------------------------------------------------------- helpers --
    def set_state(self, **kwargs):
        with self.lock:
            self.state.update(kwargs)

    def set_camera(self, **kwargs):
        with self.lock:
            camera = dict(self.state['camera'] or {})
            camera.update(kwargs)
            self.state['camera'] = camera

    def snapshot(self, name):
        with self.lock:
            return list(getattr(self, name))

    def wait_for(self, predicate, timeout, message):
        deadline = time.time() + timeout
        while time.time() < deadline and not rospy.is_shutdown():
            result = predicate()
            if result:
                return result
            time.sleep(0.05)
        self.fail('timeout after %.1f s: %s' % (timeout, message))

    @staticmethod
    def stamp(msg):
        return msg.header.stamp.to_nsec()

    def box(self, array):
        """The fixture box among the array's objects (base_link), or None."""
        for obj in array.objects:
            if math.hypot(obj.center.x - BOX_CENTER[0], obj.center.y - BOX_CENTER[1]) < 1.0:
                return obj
        return None

    def local_after(self, stamp_ns):
        return [m for m in self.snapshot('local') if self.stamp(m) > stamp_ns]

    def map_after(self, stamp_ns):
        return [m for m in self.snapshot('map') if self.stamp(m) > stamp_ns]

    def log_after(self, name, stamp_ns, kind):
        return [s for s, k in self.snapshot(name) if s > stamp_ns and k == kind]

    def latest_scan(self):
        scans = self.snapshot('scans')
        return scans[-1] if scans else 0

    def wait_map_observed_after(self, stamp_ns, timeout=8.0):
        return self.wait_for(lambda: [m for m in self.map_after(stamp_ns) if m.status == OBSERVED],
                             timeout, 'map OBSERVED after %d' % stamp_ns)

    def wait_local_observed_after(self, stamp_ns, timeout=8.0):
        return self.wait_for(lambda: [m for m in self.local_after(stamp_ns)
                                      if m.status == OBSERVED and self.box(m)],
                             timeout, 'local OBSERVED box after %d' % stamp_ns)

    def assert_map_matches_local(self, local, mapped):
        """map geometry == map_from_base(published GPS pose) * local geometry."""
        by_id = {o.track_id: o for o in local.objects}
        self.assertTrue(mapped.objects)
        for obj in mapped.objects:
            self.assertIn(obj.track_id, by_id)
            base = by_id[obj.track_id]
            expected = self.map_rotation @ xyz(base.center) + self.map_translation
            np.testing.assert_allclose(xyz(obj.center), expected, atol=2e-3)
            for corner_map, corner_base in zip(obj.obb_corners, base.obb_corners):
                np.testing.assert_allclose(
                    xyz(corner_map), self.map_rotation @ xyz(corner_base) + self.map_translation, atol=2e-3)
            yaw_error = math.remainder(obj.yaw - base.yaw - self.yaw_grid, 2 * math.pi)
            self.assertLess(abs(yaw_error), 1e-3)
            self.assertEqual((obj.classification, obj.camera_class), (base.classification, base.camera_class))

    def check_pairs(self, local_arrays, map_arrays, minimum):
        local_by_stamp = {self.stamp(m): m for m in local_arrays}
        pairs = [(local_by_stamp[self.stamp(m)], m) for m in map_arrays
                 if m.status == OBSERVED and self.stamp(m) in local_by_stamp]
        self.assertGreaterEqual(len(pairs), minimum)
        for local, mapped in pairs:
            self.assert_map_matches_local(local, mapped)

    def classification_after(self, stamp_ns):
        """[(stamp, classification, camera_class)] of the box after stamp_ns."""
        out = []
        for m in self.local_after(stamp_ns):
            obj = self.box(m)
            if m.status == OBSERVED and obj:
                out.append((self.stamp(m), obj.classification, obj.camera_class))
        return out

    def wait_class(self, stamp_ns, wanted, timeout):
        def first():
            hits = [entry for entry in self.classification_after(stamp_ns) if entry[1] == wanted]
            return hits[0] if hits else None
        return self.wait_for(first, timeout, 'box classification %s after %d' % (wanted, stamp_ns))

    def tagged_obs(self, tag):
        return [s for s, t in self.snapshot('obs_log') if t == tag]

    def confirm_vehicle(self, tag, **camera):
        start = self.latest_scan()
        self.set_camera(content='vehicle', tag=tag, duplicate=False, **camera)
        stamp, _, raw = self.wait_class(start, 'vehicle', 8.0)
        observations = self.tagged_obs(tag)
        self.assertGreaterEqual(len(observations), 5)
        # Main fusion_initial_confirmations = 5: never before the 5th observation.
        self.assertGreaterEqual(stamp, observations[4], 'vehicle before the 5th observation')
        self.assertEqual(raw, 'car')  # original detector label kept as camera_class
        return stamp

    def assert_lidar_continues(self, start, end, minimum):
        arrays = [m for m in self.local_after(start) if self.stamp(m) <= end]
        self.assertGreaterEqual(len(arrays), minimum)
        for m in arrays:
            self.assertEqual(m.status, OBSERVED, m.reason)
            self.assertIsNotNone(self.box(m))

    # ---------------------------------------------------------------- tests --
    def test_01_normal(self):
        self.wait_for(lambda: len([m for m in self.snapshot('map') if m.status == OBSERVED]) >= 5, 60.0,
                      'five OBSERVED map arrays after start-up')
        start = self.latest_scan()
        time.sleep(2.5)
        end = self.latest_scan()
        local = [m for m in self.local_after(start) if self.stamp(m) <= end]
        mapped = [m for m in self.map_after(start) if self.stamp(m) <= end]
        self.assertGreaterEqual(len(local), 15)
        published = set(self.snapshot('scans'))
        ids = set()
        for arrays, frame in ((local, 'base_link'), (mapped, 'map')):
            stamps = [self.stamp(m) for m in arrays]
            self.assertEqual(stamps, sorted(set(stamps)), 'stamps strictly increasing')
            for m in arrays:
                self.assertEqual(m.header.frame_id, frame)
                self.assertIn(self.stamp(m), published, 'header stamp == PointCloud2 source stamp')
                self.assertEqual(m.status, OBSERVED, m.reason)
                self.assertTrue(m.stamp_provenance)
        for m in local:
            obj = self.box(m)
            self.assertIsNotNone(obj, 'observed box present')
            dims = sorted((obj.length, obj.width))
            self.assertAlmostEqual(dims[1], max(BOX_SIZE), delta=0.5)
            self.assertAlmostEqual(dims[0], min(BOX_SIZE), delta=0.5)
            self.assertAlmostEqual(obj.height, BOX_Z[1] - BOX_Z[0], delta=0.4)
            self.assertFalse(obj.relative_velocity_valid or obj.absolute_velocity_valid or obj.dynamic_valid)
            ids.add(obj.track_id)
        self.assertEqual(len(ids), 1, 'one stable track id')
        type(self).track_id = ids.pop()
        self.check_pairs(local, mapped, 10)

        diagnostics = self.wait_for(lambda: self.snapshot('diagnostics'), 5.0, 'diagnostics')[-1]
        values = {kv.key: kv.value for kv in diagnostics.status[0].values}
        self.assertEqual(values['velocity_status'], 'unproven_moving_sensor_derivative')
        self.assertTrue(values['config_sha256'])
        self.assertEqual(values['roll_leveling'], 'true')
        self.assertEqual(values['tunnel_filter'], 'false')
        self.assertEqual(values['camera_fusion'], 'enabled')
        for key in ('processed', 'invalid', 'stamp_errors', 'overwritten_scans', 'rejected_scans',
                    'processing_ms_p50', 'processing_ms_p95', 'processing_ms_max', 'fusion_pending_scans'):
            self.assertIn(key, values)

        traces = []
        subscriber = rospy.Subscriber('/migration/lidar/trace', String, traces.append, queue_size=5)
        try:
            self.wait_for(lambda: traces, 5.0, 'trace message while subscribed')
        finally:
            subscriber.unregister()
        line = json.loads(traces[-1].data)
        self.assertEqual(line['schema'], 'asmc_lidar_scan_trace_v1')
        self.assertTrue(line['accepted'] and line['pose_ready'] and line['processed'])
        self.assertIn('typed', line)
        self.assertIn('tracks', line)

        if self.expect_tf:
            transforms = [t for msg in self.snapshot('tf') for t in msg.transforms]
            self.assertTrue(transforms)
            for t in transforms:
                self.assertEqual(t.header.frame_id, 'base_link')
                self.assertEqual(t.child_frame_id, 'lidar_pipeline/lidar_leveled')
            last = transforms[-1].transform.translation
            np.testing.assert_allclose([last.x, last.y, last.z], self.mount, atol=1e-6)

    def test_02_gps_invalid_then_recovery(self):
        self.wait_map_observed_after(self.latest_scan())
        start = self.latest_scan()
        self.set_state(gps='invalid')
        time.sleep(1.5)
        resume = self.latest_scan()
        self.set_state(gps='normal')
        observed = self.wait_map_observed_after(resume)[0]
        first_invalid = self.log_after('gps_log', start, 'invalid')[0]
        recovery = self.log_after('gps_log', first_invalid, 'normal')
        self.assertGreaterEqual(len(recovery), 3)
        # B: margin for the in-flight invalid sample; up to the first recovery sample.
        window_start = first_invalid + 150000000
        mapped = [m for m in self.map_after(window_start) if self.stamp(m) < recovery[0]]
        self.assertGreaterEqual(len(mapped), 5)
        for m in mapped:
            self.assertEqual((m.status, m.reason, len(m.objects)), (POSE_UNAVAILABLE, 'gps_invalid', 0))
        self.assert_lidar_continues(window_start, recovery[0], 5)
        for m in self.local_after(window_start):
            if self.stamp(m) < recovery[0]:
                self.assertEqual(self.box(m).track_id, self.track_id, 'local keeps the same track')
        # C: OBSERVED only once three consecutive normal samples exist.
        first_observed = min(self.stamp(m) for m in self.map_after(first_invalid) if m.status == OBSERVED)
        self.assertGreaterEqual(first_observed, recovery[2])
        for m in self.map_after(first_invalid):
            if self.stamp(m) < recovery[2]:
                self.assertNotEqual(m.status, OBSERVED)
        time.sleep(0.5)
        later = [m for m in self.map_after(first_observed - 1) if m.status == OBSERVED]
        local = self.local_after(first_observed - 1)
        self.check_pairs(local, later, 3)  # recovered pose, never the invalid-period position

    def test_03_gps_silent_then_recovery(self):
        self.wait_map_observed_after(self.latest_scan())
        start = self.latest_scan()
        self.set_state(gps='silent')
        time.sleep(1.5)
        last_normal = [s for s, k in self.snapshot('gps_log') if k == 'normal' and s <= start + 200000000][-1]
        self.set_state(gps='normal')
        self.wait_map_observed_after(self.latest_scan())
        recovery = self.log_after('gps_log', last_normal, 'normal')
        window_start = last_normal + 350000000  # 300 ms bound + margin
        mapped = [m for m in self.map_after(window_start) if self.stamp(m) < recovery[0]]
        self.assertGreaterEqual(len(mapped), 5)
        for m in mapped:
            self.assertEqual((m.status, m.reason, len(m.objects)), (POSE_UNAVAILABLE, 'gps_stale', 0))
        self.assert_lidar_continues(window_start, recovery[0], 5)
        for m in self.local_after(window_start):
            if self.stamp(m) < recovery[0]:
                self.assertEqual(self.box(m).track_id, self.track_id, 'local keeps the same track')
        first_observed = min(self.stamp(m) for m in self.map_after(window_start) if m.status == OBSERVED)
        self.assertGreaterEqual(first_observed, recovery[2])

    def test_04a_camera_vehicle_after_five_observations(self):
        type(self).session = str(uuid.uuid4())
        type(self).generation = 1
        start = self.latest_scan()
        self.set_camera(session=self.session, generation=1, mode=InferenceStatus.ACTIVE, status=True,
                        content=None, tag='E1-status')
        time.sleep(0.6)  # status gate open before the first observation
        self.assertTrue(all(c == 'unknown' for _, c, _ in self.classification_after(start)))
        self.confirm_vehicle('E1')

    def test_04b_camera_empty_observations_reset_class(self):
        start = self.latest_scan()
        self.set_camera(content='empty', tag='E2')
        stamp, _, raw = self.wait_class(start, 'unknown', 6.0)
        empties = self.tagged_obs('E2')
        # class_miss_grace = 3: kept through 3 misses, reset on the 4th.
        self.assertGreaterEqual(stamp, empties[3])
        self.assertEqual(raw, 'unknown')
        self.assert_lidar_continues(start, stamp, 3)

    def test_04c_camera_invalid_observation(self):
        self.confirm_vehicle('E3v')
        start = self.latest_scan()
        self.set_camera(content='invalid', tag='E3i')
        stamp, _, _ = self.wait_class(start, 'unknown', 4.0)
        time.sleep(0.5)
        self.assert_lidar_continues(start, self.latest_scan(), 5)

    def test_04d_camera_standby(self):
        self.confirm_vehicle('E4v')
        start = self.latest_scan()
        type(self).generation += 1
        self.set_camera(generation=self.generation, mode=InferenceStatus.STANDBY, content=None, tag='E4s')
        self.wait_class(start, 'unknown', 4.0)
        time.sleep(0.5)
        self.assert_lidar_continues(start, self.latest_scan(), 5)

    def test_04e_camera_session_change_resets(self):
        type(self).generation += 1
        self.set_camera(generation=self.generation, mode=InferenceStatus.ACTIVE, content=None, tag='E5s')
        time.sleep(0.6)
        self.confirm_vehicle('E5a')
        start = self.latest_scan()
        type(self).session, type(self).generation = str(uuid.uuid4()), 1
        self.set_camera(session=self.session, generation=1, content='vehicle', tag='E5b')
        reset, _, _ = self.wait_class(start, 'unknown', 4.0)
        again, _, raw = self.wait_class(reset, 'vehicle', 8.0)
        observations = self.tagged_obs('E5b')
        self.assertGreaterEqual(again, observations[4], 'new session needs 5 new observations')
        self.assertEqual(raw, 'car')

    def test_04f_camera_duplicates_not_double_counted(self):
        start = self.latest_scan()
        type(self).generation += 1
        self.set_camera(generation=self.generation, content=None, tag='E6s')
        self.wait_class(start, 'unknown', 4.0)
        time.sleep(0.3)
        begin = self.latest_scan()
        self.set_camera(content='vehicle', tag='E6', duplicate=True)
        stamp, _, _ = self.wait_class(begin, 'vehicle', 8.0)
        distinct = self.tagged_obs('E6')
        self.assertGreaterEqual(stamp, distinct[4], 'duplicates/regressing observations counted')

    def test_04g_camera_stopped_lidar_continues(self):
        start = self.latest_scan()
        self.set_state(camera=None)
        self.wait_class(start, 'unknown', 4.0)
        time.sleep(1.5)
        self.assert_lidar_continues(start, self.latest_scan(), 10)

    def test_05_imu_stale_is_input_invalid(self):
        self.wait_local_observed_after(self.latest_scan())
        start = self.latest_scan()
        self.set_state(imu='stopped')
        time.sleep(1.2)
        resume = self.latest_scan()
        self.set_state(imu='normal')
        last_imu = [s for s, k in self.snapshot('imu_log') if s <= start + 100000000][-1]
        window_start = last_imu + 120000000 + 60000000  # imu max age + margin
        for arrays in (self.local_after(window_start), self.map_after(window_start)):
            stale = [m for m in arrays if self.stamp(m) <= resume]
            self.assertGreaterEqual(len(stale), 5)
            for m in stale:
                self.assertEqual((m.status, len(m.objects)), (INPUT_INVALID, 0), m.reason)
        self.wait_local_observed_after(resume)

    def test_06_imu_invalid_quaternion_is_input_invalid(self):
        start = self.latest_scan()
        self.set_state(imu='invalid')
        time.sleep(1.2)
        resume = self.latest_scan()
        self.set_state(imu='normal')
        first_invalid = self.log_after('imu_log', start, 'invalid')[0]
        window_start = first_invalid + 60000000
        for arrays in (self.local_after(window_start), self.map_after(window_start)):
            bad = [m for m in arrays if self.stamp(m) <= resume]
            self.assertGreaterEqual(len(bad), 5)
            for m in bad:
                self.assertEqual((m.status, len(m.objects)), (INPUT_INVALID, 0), m.reason)
        self.wait_local_observed_after(resume)

    def test_07_isolation(self):
        master = rosgraph.Master('/lidar_pipeline_node_test')
        publications, _, services = master.getSystemState()
        published = sorted(topic for topic, nodes in publications if self.node in nodes)
        allowed = {'/rosout'} | ({'/tf'} if self.expect_tf else set())
        self.assertTrue(published)
        for topic in published:
            self.assertTrue(topic.startswith('/migration/lidar/') or topic in allowed, topic)
        self.assertEqual(sorted(s for s, providers in services if self.node in providers),
                         sorted([self.node + '/get_loggers', self.node + '/set_logger_level']))

    def test_08_guard_refuses_operational_output(self):
        executable = rospy.get_param('~node_executable', '')
        if not executable:
            executable = roslib.packages.find_node(PKG, 'lidar_pipeline_node')[0]
        cases = [
            ['_objects_local_topic:=/perception/lidar/objects_local'],
            ['/migration/lidar/objects_map:=/perception/lidar/objects_map'],
            ['/migration/lidar/objects_local:=/last_lidar'],
            ['_diagnostics_topic:=/perception/lidar/diagnostics'],
            ['_trace_topic:=/planning/shadow_trace'],
            ['_tf_child_frame:=lidar_leveled'],
            ['_objects_local_topic:=/perception/lidar/objects'],
            ['_trace_topic:=/velodyne_points'],
            ['_tf_child_frame:=velodyne'],
        ]
        master = rosgraph.Master('/lidar_pipeline_node_test')
        for index, case in enumerate(cases):
            name = 'lidar_pipeline_guard_%d' % index
            command = [executable, '__name:=' + name, '_pipeline_config:=' + self.pipeline_config,
                       '_enable_camera_fusion:=false'] + case
            try:
                result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                        timeout=30, universal_newlines=True)
            except subprocess.TimeoutExpired:
                self.fail('guard did not refuse %s (node kept running)' % case)
            self.assertNotEqual(result.returncode, 0, '%s accepted:\n%s' % (case, result.stdout))
            self.assertIn('refused', result.stdout, case)
            publications, _, _ = master.getSystemState()
            for topic, nodes in publications:
                self.assertFalse(topic in OPERATIONAL or topic.startswith('/planning/'),
                                 'operational topic advertised: %s by %s' % (topic, nodes))


if __name__ == '__main__':
    rospy.init_node('lidar_pipeline_node_test')
    rostest.rosrun(PKG, 'lidar_pipeline_node', LidarPipelineNodeTest)
