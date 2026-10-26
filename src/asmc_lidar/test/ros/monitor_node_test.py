#!/usr/bin/env python3
"""rostest node for the asmc_lidar web monitor (synthetic inputs only).

A background thread publishes a box on a flat ground plane (PointCloud2, 10 Hz),
sensor_msgs/Imu and morai EgoVehicleStatus (50 Hz); no GPS and no camera. The
methods run in name order against the same shadow + monitor nodes:

  01 health, live status, typed local OBSERVED, map POSE_UNAVAILABLE, camera disabled
  02 one frame: stage arrays match the announced sample sizes, labels/clusters are
     consistent, typed local stamp == scene stamp, the observed track sits where the
     synthetic box is (base_link via base_from_leveled) with its size
  03 since filter returns the status without a scene
  04 only the fixed routes exist, no file browsing, no write methods
  05 IMU stopped -> INPUT_INVALID (scan notice without stages), recovery
  06 points stopped -> no LiDAR input / stale, ages grow, recovery
  07 the monitor node publishes nothing and offers no service beyond rospy defaults

The synthetic cloud is an interface fixture, not a perception accuracy test.
"""
import json
import math
import struct
import threading
import time
import unittest
import urllib.error
import urllib.request

import numpy as np
import rosgraph
import rospy
import rostest
import yaml
from morai_msgs.msg import EgoVehicleStatus
from sensor_msgs.msg import Imu, PointCloud2, PointField

PKG = 'asmc_lidar'
OBSERVED, OBSERVED_EMPTY, INPUT_INVALID, POSE_UNAVAILABLE = 1, 2, 3, 4
WHEEL_RADIUS = 0.36
BOX_CENTER = (10.0, 0.0)   # base_link
BOX_SIZE = (4.0, 1.8)
BOX_Z = (-WHEEL_RADIUS + 0.4, -WHEEL_RADIUS + 1.9)
MAGIC = b'ALMF'


def rpy(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch),
                              math.cos(yaw), math.sin(yaw))
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return rz @ ry @ rx


class MonitorNodeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.port = int(rospy.get_param('~port', 18091))
        cls.base = 'http://127.0.0.1:%d' % cls.port
        mount = yaml.safe_load(open(rospy.get_param('~mount_config')))['sensor_pose_base_link']
        cls.mount = np.array([mount['x_m'], mount['y_m'], mount['z_m']])
        cls.mount_rotation = rpy(*(math.radians(mount[k]) for k in ('roll_deg', 'pitch_deg', 'yaw_deg')))
        cls.cloud = cls.make_cloud()
        cls.lock = threading.Lock()
        cls.state = {'imu': 'normal', 'points': 'normal'}
        cls.pub = dict(points=rospy.Publisher('/velodyne_points', PointCloud2, queue_size=2),
                       imu=rospy.Publisher('/imu', Imu, queue_size=10),
                       can=rospy.Publisher('/Ego_topic', EgoVehicleStatus, queue_size=10))
        cls.running = True
        cls.thread = threading.Thread(target=cls.publish_loop, daemon=True)
        cls.thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.running = False
        cls.thread.join(timeout=2.0)

    @classmethod
    def make_cloud(cls):
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
        box = (np.concatenate(faces) - cls.mount) @ cls.mount_rotation
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
    def publish_loop(cls):
        rate = rospy.Rate(100)
        tick = 0
        while cls.running and not rospy.is_shutdown():
            with cls.lock:
                state = dict(cls.state)
            now = rospy.Time.now()
            if tick % 2 == 0 and state['imu'] == 'normal':
                imu = Imu()
                imu.header.stamp, imu.header.frame_id = now, 'imu'
                imu.orientation.w = 1.0
                imu.linear_acceleration.z = 9.81
                cls.pub['imu'].publish(imu)
            if tick % 2 == 0:
                can = EgoVehicleStatus()
                can.header.stamp = now
                cls.pub['can'].publish(can)
            if tick % 10 == 0 and state['points'] == 'normal':
                cls.cloud.header.stamp = now
                cls.pub['points'].publish(cls.cloud)
            tick += 1
            rate.sleep()

    # --------------------------------------------------------------- helpers --
    def get(self, path, method='GET'):
        request = urllib.request.Request(self.base + path, method=method)
        try:
            with urllib.request.urlopen(request, timeout=5) as response:
                return response.status, response.read()
        except urllib.error.HTTPError as error:
            return error.code, error.read()

    def status(self):
        code, body = self.get('/api/status')
        self.assertEqual(code, 200)
        return json.loads(body.decode())

    def frame(self, since=0):
        code, body = self.get('/api/frame?since=%d' % since)
        self.assertEqual(code, 200)
        self.assertEqual(body[:4], MAGIC)
        header_len, payload_len = struct.unpack('<II', body[4:12])
        header = json.loads(body[12:12 + header_len].decode())
        offset = 12 + header_len + ((-header_len) % 4)
        self.assertEqual(len(body), offset + payload_len)
        arrays = {}
        for name, entry in header['arrays'].items():
            dtype = {'f32': '<f4', 'u8': 'u1', 'i32': '<i4'}[entry['dtype']]
            arrays[name] = np.frombuffer(body, dtype=dtype, count=entry['length'], offset=offset + entry['offset'])
        return header, arrays

    def wait_until(self, predicate, timeout_s, what):
        deadline = time.time() + timeout_s
        last = None
        while time.time() < deadline:
            try:
                last = predicate()
                if last:
                    return last
            except (OSError, AssertionError, ValueError, KeyError) as exc:
                last = 'error: %r' % (exc,)
            time.sleep(0.25)
        try:
            snapshot = self.status()
            snapshot = {k: snapshot.get(k) for k in ('pipeline_state', 'pipeline_detail', 'debug_state', 'ages_ms',
                                                     'objects_local', 'rates', 'store', 'master')}
        except (OSError, AssertionError, ValueError):
            snapshot = 'status unavailable'
        self.fail('timeout waiting for %s (last=%r, status=%r)' % (what, last, snapshot))

    def set_state(self, **kwargs):
        with self.lock:
            self.state.update(kwargs)

    # ----------------------------------------------------------------- tests --
    def test_01_health_and_live_status(self):
        self.wait_until(lambda: self.get('/api/health')[0] == 200, 30, 'health')
        status = self.wait_until(
            lambda: (lambda s: s if s['pipeline_state'] == 'ok' and s['debug_state'] == 'live' else None)(self.status()),
            40, 'pipeline ok + debug live')
        self.assertEqual(status['objects_local']['status'], OBSERVED)
        self.assertGreaterEqual(status['objects_local']['count'], 1)
        self.assertEqual(status['map_state'], 'unavailable')
        self.assertEqual(status['objects_map']['status'], POSE_UNAVAILABLE)
        self.assertEqual(status['camera_state'], 'disabled')
        self.assertEqual(status['debug_enabled'], 'true')
        # The master poll is independent of diagnostics/scene reception. Its
        # first snapshot can see the fixture's points publisher before it sees
        # the pipeline. Wait for BOTH in the same snapshot, within the existing
        # timeout; pipeline_state=ok does not make that first snapshot current.
        master = self.wait_until(
            lambda: (lambda m: m if m['ok'] and m['points_publishers'] == 1
                     and '/migration/lidar/lidar_pipeline' in m['shadow_nodes'] else None)(self.status()['master']),
            10, 'points publisher and pipeline seen by master')
        self.assertTrue(master['ok'])
        self.assertIn('/migration/lidar/lidar_pipeline', master['shadow_nodes'])
        self.assertLess(status['ages_ms']['scan'], 1000)
        # Input/processing rates come from consecutive 1 Hz diagnostics samples.
        rates = self.wait_until(lambda: (lambda r: r if (r['input_hz'] or 0) > 5.0 else None)(self.status()['rates']),
                                10, 'input rate from diagnostics')
        self.assertGreater(rates['processed_hz'] or 0, 5.0)
        self.assertGreater(rates['debug_hz'] or 0, 1.0)

    def test_02_frame_scene_consistency(self):
        header, arrays = self.wait_until(
            lambda: (lambda f: f if f[0]['scene'] and f[0]['scene']['stages'] and not f[0]['scene'].get('missing') else None)(self.frame(0)),
            30, 'scene with stages')
        scene = header['scene']
        self.assertEqual(scene['schema'], 'asmc_lidar_debug_scan_v1')
        self.assertEqual(scene['frames']['sensor'], 'velodyne')
        self.assertEqual(scene['frames']['base'], 'base_link')
        self.assertEqual(scene['missing'], [])
        sampled, counts = scene['sampled'], scene['counts']
        for stage in ('raw', 'roi', 'voxel', 'leveled'):
            self.assertEqual(len(arrays[stage + '.xyzi']), 4 * sampled[stage]['sent'], stage)
            self.assertEqual(sampled[stage]['actual'], counts[stage], stage)
            self.assertEqual(sampled[stage]['sent'], math.ceil(counts[stage] / sampled[stage]['stride']), stage)
        self.assertEqual(sampled['raw']['actual'], self.cloud.width)
        self.assertLessEqual(sampled['raw']['sent'], 3000)      # debug_max_points of the launch
        self.assertEqual(counts['voxel'], counts['leveled'])
        label = arrays['leveled.label']
        cluster = arrays['leveled.cluster']
        self.assertEqual(len(label), sampled['leveled']['sent'])
        self.assertEqual(len(cluster), sampled['leveled']['sent'])
        self.assertTrue(set(np.unique(label)) <= {0, 1}, np.unique(label))  # tunnel OFF: no label 2
        self.assertTrue(set(np.unique(cluster)) <= set(range(-1, counts['clusters'])))
        removed_fraction = float(np.mean(label == 1))
        expected_fraction = (counts['leveled'] - counts['ground_kept']) / counts['leveled']
        self.assertAlmostEqual(removed_fraction, expected_fraction, delta=0.08)
        self.assertGreater(expected_fraction, 0.3)  # the plane is most of the scene
        self.assertTrue(np.all(cluster[label == 1] == -1))  # removed ground points are never clustered
        self.assertGreaterEqual(counts['clusters'], 1)
        self.assertEqual(len(scene['detections']), counts['detections'])
        self.assertEqual(len(scene['tracks']), counts['tracks'])
        typed = scene['typed']['local']
        self.assertTrue(typed['accepted'])
        self.assertEqual(typed['status'], OBSERVED)
        self.assertEqual(typed['stamp_ns'], scene['stamp_ns'])
        self.assertEqual(typed['frame'], 'base_link')
        self.assertEqual(scene['typed']['map']['status'], POSE_UNAVAILABLE)
        base_from_leveled = np.array(scene['base_from_leveled']).reshape(4, 4)
        observed = [t for t in scene['tracks'] if t['miss'] == 0]
        self.assertTrue(observed)
        box = max(observed, key=lambda t: t['points'])
        center = base_from_leveled @ np.array(box['obb']['center'] + [1.0])
        self.assertAlmostEqual(center[0], BOX_CENTER[0], delta=0.8)
        self.assertAlmostEqual(center[1], BOX_CENTER[1], delta=0.6)
        long_side, short_side = max(box['obb']['width'], box['obb']['height']), min(box['obb']['width'], box['obb']['height'])
        self.assertAlmostEqual(long_side, BOX_SIZE[0], delta=0.6)
        self.assertAlmostEqual(short_side, BOX_SIZE[1], delta=0.6)
        self.assertAlmostEqual(box['obb']['zmax'] - box['obb']['zmin'], BOX_Z[1] - BOX_Z[0], delta=0.4)
        self.assertEqual(box['class'], 'unknown')  # no camera: class evidence stays unknown
        published = [o for o in typed['objects'] if o['track_id'] == box['id']]
        self.assertEqual(len(published), 1)
        self.assertAlmostEqual(published[0]['center'][0], center[0], delta=1e-3)
        self.assertAlmostEqual(published[0]['center'][1], center[1], delta=1e-3)
        self.assertEqual(published[0]['points'], box['points'])
        self.assertEqual(scene['config']['tunnel_filter'], False)
        self.assertEqual(scene['config']['roll_leveling'], True)
        self.assertEqual(scene['vehicle']['wheelbase'], 3)

    def test_03_since_filter_and_status_only(self):
        header, _ = self.frame(0)
        seq = header['frame_seq']
        self.assertGreater(seq, 0)
        # Live input can advance between the two HTTP requests. A newer scene
        # is correct; validate it and retry against that sequence. Still require
        # a real unchanged/status-only response, never skip that assertion.
        for _ in range(20):
            same, arrays = self.frame(seq)
            self.assertIn('pipeline_state', same['status'])
            if same['frame_seq'] == seq:
                self.assertIsNone(same['scene'])
                self.assertEqual(arrays, {})
                break
            self.assertGreater(same['frame_seq'], seq)
            self.assertIsNotNone(same['scene'])
            seq = same['frame_seq']
        else:
            self.fail('no unchanged frame response in 20 consecutive requests')
        older, _ = self.frame(seq - 1)
        self.assertIsNotNone(older['scene'])

    def test_04_only_fixed_routes(self):
        code, body = self.get('/')
        self.assertEqual(code, 200)
        self.assertIn(b'<canvas id="bev">', body)
        self.assertIn(b'id="audit-summary"', body)
        self.assertEqual(self.get('/app.js')[0], 200)
        self.assertEqual(self.get('/app.css')[0], 200)
        for path in ('/scripts/asmc_lidar_monitor_core.py', '/../package.xml', '/web/index.html', '/api',
                     '/api/frame/x', '/index.html/../app.js'):
            self.assertEqual(self.get(path)[0], 404, path)
        for method in ('POST', 'PUT', 'DELETE'):
            self.assertEqual(self.get('/api/status', method=method)[0], 501, method)

    def test_05_imu_missing_gives_input_invalid_then_recovers(self):
        self.set_state(imu='stopped')
        status = self.wait_until(
            lambda: (lambda s: s if s['pipeline_state'] == 'input_invalid' else None)(self.status()),
            15, 'input_invalid')
        self.assertEqual(status['objects_local']['status'], INPUT_INVALID)
        self.assertIn('imu', status['pipeline_detail']['input_reason'].lower())
        header, arrays = self.wait_until(
            lambda: (lambda f: f if f[0]['scene'] and not f[0]['scene']['stages'] else None)(self.frame(0)),
            10, 'stage-less scan notice')
        scene = header['scene']
        self.assertFalse(scene['processed'])
        self.assertFalse(scene['pose_ready'])
        self.assertIn('imu', scene['reason'].lower())
        self.assertEqual(scene['typed']['local']['status'], INPUT_INVALID)
        self.assertEqual(arrays, {})
        self.set_state(imu='normal')
        self.wait_until(lambda: self.status()['pipeline_state'] == 'ok', 20, 'recovery after IMU')
        self.wait_until(lambda: (lambda f: f[0]['scene'] and f[0]['scene']['stages'])(self.frame(0)), 10,
                        'scene with stages after recovery')

    def test_06_points_stopped_gives_no_input_then_recovers(self):
        self.set_state(points='stopped')
        status = self.wait_until(
            lambda: (lambda s: s if s['pipeline_state'] in ('lidar_stale', 'no_lidar_input') else None)(self.status()),
            10, 'stale or no input')
        self.assertGreater(status['ages_ms']['scan'], 1000)
        time.sleep(2.5)
        status = self.status()
        self.assertEqual(status['pipeline_state'], 'no_lidar_input')  # diagnostics report input_silence
        self.assertEqual(status['debug_state'], 'stale')
        header, _ = self.frame(0)
        self.assertIsNotNone(header['scene'])  # the last scene stays available, flagged stale by the status
        self.set_state(points='normal')
        self.wait_until(lambda: self.status()['pipeline_state'] == 'ok', 20, 'recovery after points')

    def test_07_monitor_publishes_nothing(self):
        master = rosgraph.Master('/monitor_node_test')
        publishers, subscribers, services = master.getSystemState()
        for topic, nodes in publishers:
            if topic == '/rosout':  # every rospy node logs to /rosout
                continue
            self.assertNotIn('/asmc_lidar_monitor', nodes, 'monitor publishes %s' % topic)
        subscribed = {topic for topic, nodes in subscribers if '/asmc_lidar_monitor' in nodes}
        self.assertIn('/migration/lidar/debug/scan', subscribed)
        self.assertIn('/migration/lidar/debug/leveled', subscribed)
        self.assertIn('/migration/lidar/diagnostics', subscribed)
        offered = {service for service, nodes in services if '/asmc_lidar_monitor' in nodes}
        self.assertTrue(offered <= {'/asmc_lidar_monitor/get_loggers', '/asmc_lidar_monitor/set_logger_level'}, offered)


if __name__ == '__main__':
    rospy.init_node('monitor_node_test')
    rostest.rosrun(PKG, 'monitor_node', MonitorNodeTest)
