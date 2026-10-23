#!/usr/bin/env python3
"""Unit tests of the web monitor core (no ROS): cloud parsing, scene grouping by
stamp, binary frame layout, status classification, fixed HTTP routes, server
session across restarts, connection limit and deadlines, and the browser opener
decisions."""
import json
import os
from pathlib import Path
import socket
import struct
import sys
import tempfile
import threading
import time
import unittest
import urllib.error
import urllib.request

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'scripts'))
import asmc_lidar_monitor_core as core  # noqa: E402


def cloud_bytes(points, labelled):
    out = bytearray()
    for i, (x, y, z, w) in enumerate(points):
        out += struct.pack('<ffff', x, y, z, w)
        if labelled:
            out += struct.pack('<BxxxI', (i % 3), 0)  # label, pad, cluster placeholder
            out[-4:] = struct.pack('<i', i - 1)
    return bytes(out)


def fields(labelled):
    base = [('x', 0, 7, 1), ('y', 4, 7, 1), ('z', 8, 7, 1), ('intensity', 12, 7, 1)]
    if labelled:
        base += [('label', 16, 2, 1), ('cluster', 20, 5, 1)]
    return base


def scan_json(stamp, sent, stages=True, extra=None):
    doc = {'schema': core.SCHEMA, 'producer': 'test', 'seq': 1, 'stamp_ns': stamp, 'stages': stages,
           'sampled': {stage: {'actual': sent.get(stage, 0), 'sent': sent.get(stage, 0), 'stride': 1}
                       for stage in core.STAGES}}
    doc.update(extra or {})
    return json.dumps(doc)


class FakeClock(object):
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


class CloudParsingTest(unittest.TestCase):
    def test_xyzi_and_labels(self):
        pts = [(1.0, 2.0, 3.0, 9.0), (-1.5, 0.5, 0.25, 1.0), (0.0, 0.0, -1.0, 4.0)]
        cloud = core.parse_point_cloud(fields(True), 24, 3, 1, cloud_bytes(pts, True))
        self.assertEqual(cloud['count'], 3)
        np.testing.assert_array_equal(cloud['xyzi'], np.array(pts, dtype=np.float32))
        np.testing.assert_array_equal(cloud['label'], np.array([0, 1, 2], dtype=np.uint8))
        np.testing.assert_array_equal(cloud['cluster'], np.array([-1, 0, 1], dtype=np.int32))

    def test_plain_cloud_has_no_labels(self):
        cloud = core.parse_point_cloud(fields(False), 16, 2, 1, cloud_bytes([(1, 2, 3, 4), (5, 6, 7, 8)], False))
        self.assertIsNone(cloud['label'])
        self.assertIsNone(cloud['cluster'])
        self.assertEqual(cloud['xyzi'].shape, (2, 4))

    def test_rejects_missing_axis_and_short_data(self):
        with self.assertRaises(ValueError):
            core.parse_point_cloud([('x', 0, 7, 1), ('y', 4, 7, 1)], 8, 1, 1, b'\0' * 8)
        with self.assertRaises(ValueError):
            core.parse_point_cloud(fields(False), 16, 4, 1, b'\0' * 32)


class SceneStoreTest(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.store = core.SceneStore(max_scenes=4, complete_timeout_s=0.6, clock=self.clock)

    def cloud(self, n, labelled=False):
        pts = [(float(i), 0.0, 0.0, 1.0) for i in range(n)]
        return core.parse_point_cloud(fields(labelled), 24 if labelled else 16, n, 1, cloud_bytes(pts, labelled))

    def test_scene_waits_for_every_announced_stage(self):
        stamp = 1700000000000000000
        self.assertEqual(self.store.add_scan(scan_json(stamp, {'raw': 3, 'leveled': 2}))['stamp_ns'], stamp)
        self.assertIsNone(self.store.latest())
        self.store.add_cloud('raw', stamp, self.cloud(3))
        self.assertIsNone(self.store.latest())
        self.store.add_cloud('leveled', stamp, self.cloud(2, labelled=True))
        scene = self.store.latest()
        self.assertIsNotNone(scene)
        self.assertEqual(scene.frame_seq, 1)
        self.assertEqual(scene.missing, [])
        self.assertEqual(set(scene.arrays), {'raw.xyzi', 'leveled.xyzi', 'leveled.label', 'leveled.cluster'})
        for entry in scene.arrays.values():
            self.assertEqual(entry['offset'] % 4, 0)
        self.assertEqual(scene.arrays['raw.xyzi']['length'], 12)
        self.assertEqual(self.store.counters['complete'], 1)

    def test_cloud_before_scan_json_is_kept(self):
        stamp = 5
        self.store.add_cloud('voxel', stamp, self.cloud(1))
        self.store.add_scan(scan_json(stamp, {'voxel': 1}))
        self.assertEqual(self.store.latest().stamp_ns, stamp)

    def test_partial_scene_after_timeout(self):
        stamp = 7
        self.store.add_scan(scan_json(stamp, {'raw': 3}))
        self.assertIsNone(self.store.latest())
        self.clock.t += 0.7
        scene = self.store.latest()
        self.assertEqual(scene.missing, ['raw'])
        self.assertEqual(self.store.counters['partial'], 1)

    def test_scan_without_stages_is_ready_immediately(self):
        stamp = 9
        self.store.add_scan(scan_json(stamp, {}, stages=False, extra={'processed': False, 'reason': 'imu_stale'}))
        scene = self.store.latest()
        self.assertEqual(scene.stamp_ns, stamp)
        self.assertEqual(scene.payload, b'')

    def test_bounded_and_newest_wins(self):
        for k in range(6):
            stamp = 100 + k
            self.store.add_scan(scan_json(stamp, {'raw': 1}))
            if k % 2 == 0:
                self.store.add_cloud('raw', stamp, self.cloud(1))
        self.assertEqual(self.store.latest().stamp_ns, 104)
        self.assertEqual(self.store.counters['evicted_incomplete'], 1)  # stamp 101 never completed
        self.assertLessEqual(len(self.store._order), 4)

    def test_bad_scan_rejected(self):
        self.assertIsNone(self.store.add_scan('{"schema":"other","stamp_ns":1}'))
        self.assertIsNone(self.store.add_scan('not json'))
        self.assertEqual(self.store.counters['bad'], 2)


class FrameTest(unittest.TestCase):
    def test_layout_and_since_filter(self):
        clock = FakeClock()
        store = core.SceneStore(clock=clock)
        stamp = 42
        store.add_scan(scan_json(stamp, {'roi': 2}))
        pts = [(1.0, 2.0, 3.0, 4.0), (5.0, 6.0, 7.0, 8.0)]
        store.add_cloud('roi', stamp, core.parse_point_cloud(fields(False), 16, 2, 1, cloud_bytes(pts, False)))
        scene = store.latest()
        frame = core.build_frame({'pipeline_state': 'ok'}, scene, True)
        self.assertEqual(frame[:4], core.MAGIC)
        header_len, payload_len = struct.unpack('<II', frame[4:12])
        header = json.loads(frame[12:12 + header_len].decode('utf-8'))
        self.assertEqual(header['frame_seq'], 1)
        self.assertEqual(header['scene']['stamp_ns'], stamp)
        self.assertEqual(header['status'], {'pipeline_state': 'ok'})
        payload_offset = 12 + header_len + ((-header_len) % 4)
        self.assertEqual(payload_offset % 4, 0)
        self.assertEqual(len(frame), payload_offset + payload_len)
        entry = header['arrays']['roi.xyzi']
        values = np.frombuffer(frame, dtype='<f4', count=entry['length'], offset=payload_offset + entry['offset'])
        np.testing.assert_array_equal(values.reshape(2, 4), np.array(pts, dtype=np.float32))
        older = core.build_frame({}, scene, False)
        header_len2, payload_len2 = struct.unpack('<II', older[4:12])
        self.assertEqual(payload_len2, 0)
        self.assertIsNone(json.loads(older[12:12 + header_len2].decode('utf-8'))['scene'])


class StatusTest(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.store = core.SceneStore(clock=self.clock)
        self.state = core.MonitorState(self.store, clock=self.clock, wall=lambda: 1.5)

    def diag(self, message='valid', **values):
        base = {'input_reason': 'valid', 'map_reason': 'valid', 'camera_fusion': 'enabled', 'debug_enabled': 'true',
                'scans_received': '0', 'scans_processed': '0', 'debug_published': '0'}
        base.update({k: str(v) for k, v in values.items()})
        self.state.update_diagnostics(0, message, base)

    def test_states_in_order_of_severity(self):
        self.assertEqual(self.state.status()['pipeline_state'], 'no_shadow_node')
        self.diag('input_silence')
        self.assertEqual(self.state.status()['pipeline_state'], 'no_lidar_input')
        self.diag('imu_stale', input_reason='imu_stale')
        self.assertEqual(self.state.status()['pipeline_state'], 'input_invalid')
        self.assertEqual(self.state.status()['debug_state'], 'no_data')
        self.diag()
        self.state.note_scan(123)
        status = self.state.status()
        self.assertEqual(status['pipeline_state'], 'ok')
        self.assertEqual(status['debug_state'], 'live')
        self.assertEqual(status['scan_stamp_ns'], 123)
        self.assertEqual(status['now_wall_ms'], 1500)
        self.clock.t += 1.2
        self.diag()
        status = self.state.status()
        self.assertEqual(status['pipeline_state'], 'lidar_stale')
        self.assertEqual(status['debug_state'], 'stale')
        self.assertEqual(status['ages_ms']['scan'], 1200)
        self.clock.t += 3.5
        self.assertEqual(self.state.status()['pipeline_state'], 'no_shadow_node')
        self.state.update_master(False, 'refused')
        self.assertEqual(self.state.status()['pipeline_state'], 'ros_master_unreachable')

    def test_map_camera_and_debug_flags(self):
        self.diag(map_reason='gps_stale', camera_fusion='lidar_only', debug_enabled='false')
        status = self.state.status()
        self.assertEqual(status['map_state'], 'unknown')  # no typed map array and no scan yet
        self.assertEqual(status['map_reason'], 'gps_stale')
        self.state.note_scan(5, map_valid=True, map_reason='gps_bounded_prediction')
        status = self.state.status()
        self.assertEqual((status['map_state'], status['map_reason']), ('valid', 'gps_bounded_prediction'))
        self.state.update_objects_map(4, 'gps_missing', 5)
        status = self.state.status()
        self.assertEqual((status['map_state'], status['map_reason']), ('unavailable', 'gps_missing'))
        self.state.update_objects_map(2, 'observed_empty', 6)
        self.assertEqual(self.state.status()['map_state'], 'valid')
        self.state.update_objects_map(3, 'imu_stale', 7)
        self.assertEqual(self.state.status()['map_state'], 'input_invalid')
        self.assertEqual(status['camera_state'], 'lidar_only')
        self.assertEqual(status['debug_state'], 'disabled')

    def test_rates_from_diagnostics_samples(self):
        self.diag(scans_received=10, scans_processed=8, debug_published=2)
        self.clock.t += 2.0
        self.diag(scans_received=30, scans_processed=24, debug_published=12)
        rates = self.state.status()['rates']
        self.assertAlmostEqual(rates['input_hz'], 10.0)
        self.assertAlmostEqual(rates['processed_hz'], 8.0)
        self.assertAlmostEqual(rates['debug_hz'], 5.0)


class HttpTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.mkdtemp()
        for name in ('index.html', 'app.js', 'app.css', 'monitor_view.js'):
            with open(os.path.join(cls.tmp, name), 'w') as stream:
                stream.write('content of ' + name)
        with open(os.path.join(cls.tmp, 'secret.txt'), 'w') as stream:
            stream.write('never served')
        cls.store = core.SceneStore()
        cls.state = core.MonitorState(cls.store)
        cls.server = core.MonitorServer('127.0.0.1', 0, cls.state, cls.tmp, max_clients=2)
        cls.thread = threading.Thread(target=cls.server.serve_forever, kwargs={'poll_interval': 0.1}, daemon=True)
        cls.thread.start()
        cls.url = cls.server.url.rstrip('/')

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()
        cls.server.server_close()

    def get(self, path, method='GET'):
        request = urllib.request.Request(self.url + path, method=method)
        try:
            with urllib.request.urlopen(request, timeout=5) as response:
                return response.status, response.headers, response.read()
        except urllib.error.HTTPError as error:
            return error.code, error.headers, error.read()

    def test_drivable_read_only_endpoint(self):
        code, headers, body = self.get('/api/drivable')
        self.assertEqual(code, 200)
        self.assertFalse(json.loads(body)['available'])
        previous = self.server.drivable_body
        try:
            expected = {'available': True, 'polygons': [[[0, 0], [1, 0], [0, 1]]]}
            self.server.drivable_body = json.dumps(expected).encode()
            self.assertEqual(json.loads(self.get('/api/drivable')[2]), expected)
            self.assertEqual(self.get('/api/drivable', method='HEAD')[2], b'')
        finally:
            self.server.drivable_body = previous

    def test_assets_and_api(self):
        code, headers, body = self.get('/')
        self.assertEqual((code, body), (200, b'content of index.html'))
        self.assertIn('no-store', headers['Cache-Control'])
        self.assertEqual(self.get('/app.js')[2], b'content of app.js')
        self.assertEqual(self.get('/app.css')[0], 200)
        self.assertEqual(self.get('/monitor_view.js')[2], b'content of monitor_view.js')
        code, _, body = self.get('/api/health')
        self.assertEqual(code, 200)
        self.assertTrue(json.loads(body)['ok'])
        code, _, body = self.get('/api/status')
        self.assertEqual(json.loads(body)['pipeline_state'], 'no_shadow_node')
        code, headers, body = self.get('/api/frame?since=0')
        self.assertEqual(code, 200)
        self.assertEqual(headers['Content-Type'], 'application/octet-stream')
        self.assertEqual(body[:4], core.MAGIC)
        self.assertEqual(self.get('/api/frame?since=abc')[0], 200)
        self.assertEqual(self.get('/favicon.ico')[0], 204)

    def test_only_fixed_routes(self):
        for path in ('/secret.txt', '/../secret.txt', '/web/index.html', '/api', '/api/frame/x', '/index.html/..'):
            self.assertEqual(self.get(path)[0], 404, path)
        self.assertEqual(self.get('/', method='HEAD')[0], 200)
        self.assertEqual(self.get('/api/health', method='POST')[0], 501)
        self.assertEqual(self.get('/api/status', method='PUT')[0], 501)

    def test_client_limit_returns_503(self):
        self.server.limiter.acquire()
        self.server.limiter.acquire()
        try:
            self.assertEqual(self.get('/api/health')[0], 503)
        finally:
            self.server.limiter.release()
            self.server.limiter.release()
        self.assertEqual(self.get('/api/health')[0], 200)

    def test_url_uses_loopback_for_wildcard_bind(self):
        server = core.MonitorServer('0.0.0.0', 0, self.state, self.tmp)
        try:
            self.assertTrue(server.url.startswith('http://127.0.0.1:'))
        finally:
            server.server_close()


class SessionFrameTest(unittest.TestCase):
    """Server restart: a client cursor of another session gets the newest scene at once."""

    def setUp(self):
        self.clock = FakeClock()
        self.store = core.SceneStore(clock=self.clock)
        self.state = core.MonitorState(self.store, clock=self.clock, session_id='sess-b')
        self.tmp = tempfile.mkdtemp()
        self.server = core.MonitorServer('127.0.0.1', 0, self.state, self.tmp)
        self.thread = threading.Thread(target=self.server.serve_forever, kwargs={'poll_interval': 0.05}, daemon=True)
        self.thread.start()
        self.url = self.server.url.rstrip('/')

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()

    def frame(self, query):
        with urllib.request.urlopen(self.url + '/api/frame?' + query, timeout=5) as response:
            body = response.read()
        header_len = struct.unpack('<I', body[4:8])[0]
        return json.loads(body[12:12 + header_len].decode('utf-8'))

    def add_scenes(self, count):
        for i in range(count):
            self.store.add_scan(scan_json(1000 + i, {}, stages=False))

    def test_session_and_scene_age_in_header_and_status(self):
        self.assertIsNone(self.frame('since=0')['scene'])
        self.assertEqual(self.frame('since=0')['session'], 'sess-b')
        self.add_scenes(1)
        self.clock.t += 0.25
        header = self.frame('since=0')
        self.assertEqual((header['session'], header['frame_seq'], header['scene_age_ms']), ('sess-b', 1, 250))
        status = self.state.status()
        self.assertEqual(status['session'], 'sess-b')
        self.assertEqual(status['latest_scene'], {'frame_seq': 1, 'stamp_ns': 1000, 'age_ms': 250})
        self.assertIn('http', header['status'])
        self.assertEqual(header['status']['http']['max_clients'], 8)

    def test_restarted_server_serves_old_cursor_at_once(self):
        self.add_scenes(3)  # new server numbers its scenes 1..3
        # The browser still holds frame 163 of the previous server (session sess-a).
        header = self.frame('since=163&session=sess-a')
        self.assertIsNotNone(header['scene'])
        self.assertEqual((header['session'], header['frame_seq'], header['scene']['stamp_ns']), ('sess-b', 3, 1002))
        # Without a session parameter a cursor ahead of this server's numbering is another session too.
        self.assertIsNotNone(self.frame('since=163')['scene'])
        # Same session: only newer scenes.
        self.assertIsNone(self.frame('since=3&session=sess-b')['scene'])
        self.assertIsNotNone(self.frame('since=2&session=sess-b')['scene'])
        self.assertIsNone(self.frame('since=3')['scene'])

    def test_new_session_ids_differ(self):
        self.assertNotEqual(core.new_session_id(), core.new_session_id())
        self.assertNotEqual(core.MonitorState(self.store).session_id, core.MonitorState(self.store).session_id)


class HttpLimitTest(unittest.TestCase):
    """Connections are bounded before parsing; header and request deadlines free the slots."""
    MAX = 2
    HEADER_S = 0.5
    REQUEST_S = 1.5

    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        for name in ('index.html', 'app.js', 'app.css', 'monitor_view.js'):
            with open(os.path.join(self.tmp, name), 'w') as stream:
                stream.write('content of ' + name)
        self.store = core.SceneStore(max_scenes=64)
        self.state = core.MonitorState(self.store)
        self.baseline_threads = threading.active_count()
        self.server = core.MonitorServer('127.0.0.1', 0, self.state, self.tmp, max_clients=self.MAX,
                                         header_timeout_s=self.HEADER_S, request_timeout_s=self.REQUEST_S)
        self.thread = threading.Thread(target=self.server.serve_forever, kwargs={'poll_interval': 0.05}, daemon=True)
        self.thread.start()
        self.port = self.server.server_address[1]
        self.sockets = []

    def tearDown(self):
        for sock in self.sockets:
            sock.close()
        self.server.shutdown()
        self.server.server_close()

    def connect(self, rcvbuf=None):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        if rcvbuf:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, rcvbuf)
        sock.settimeout(5)
        sock.connect(('127.0.0.1', self.port))
        self.sockets.append(sock)
        return sock

    def get(self, path='/api/health'):
        try:
            with urllib.request.urlopen('http://127.0.0.1:%d%s' % (self.port, path), timeout=5) as response:
                return response.status
        except urllib.error.HTTPError as error:
            return error.code

    def wait(self, predicate, timeout_s, what):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.02)
        self.fail('timeout waiting for ' + what + ' ' + repr(self.server.stats()))

    def handler_threads(self):
        # Server internals besides handlers: serve_forever + reaper + rejecter.
        return threading.active_count() - self.baseline_threads - 3

    def read_all(self, sock, timeout_s=5.0):
        sock.settimeout(timeout_s)
        chunks = []
        try:
            while True:
                data = sock.recv(65536)
                if not data:
                    break
                chunks.append(data)
        except (ConnectionResetError, socket.timeout):
            pass
        return b''.join(chunks)

    def add_big_scene(self, points):
        self.store.add_scan(scan_json(7, {'raw': points}))
        self.store.add_cloud('raw', 7, {'xyzi': np.zeros((points, 4), dtype=np.float32), 'label': None,
                                        'cluster': None, 'count': points})
        self.assertIsNotNone(self.store.latest())

    def test_unfinished_headers_hold_only_max_clients_threads(self):
        held = []
        for _ in range(self.MAX):
            sock = self.connect()
            sock.sendall(b'GET /api/health HTTP/1.1\r\nHost: x\r\n')  # never ends the header block
            held.append(sock)
        self.wait(lambda: self.server.stats()['active'] == self.MAX, 2, 'slots taken')
        extra = []
        for _ in range(6):
            sock = self.connect()
            sock.sendall(b'GET /api/health HTTP/1.1\r\nHost: x\r\n')
            extra.append(sock)
        for sock in extra:  # answered 503 without a handler thread, then closed
            self.assertTrue(self.read_all(sock, 3).startswith(b'HTTP/1.1 503'))
        self.assertLessEqual(self.handler_threads(), self.MAX)
        self.assertEqual(self.get(), 503)  # complete request while the slots are held
        stats = self.server.stats()
        self.assertEqual(stats['active'], self.MAX)
        self.assertGreaterEqual(stats['rejected_busy'], 7)
        # Header deadline: the held connections are closed and the service recovers.
        started = time.monotonic()
        for sock in held:
            self.assertEqual(self.read_all(sock, 3), b'')
        self.assertLess(time.monotonic() - started, self.HEADER_S + 1.0)
        self.wait(lambda: self.server.stats()['active'] == 0, 2, 'slots freed')
        self.assertEqual(self.get(), 200)
        self.assertGreaterEqual(self.server.stats()['timed_out'], self.MAX)
        self.wait(lambda: self.handler_threads() <= 0, 2, 'handler threads gone')

    def test_trickled_header_is_cut_at_the_header_deadline(self):
        sock = self.connect()
        started = time.monotonic()
        closed_at = None
        for byte in b'GET /api/health HTTP/1.1\r\nHost: x\r\nX-Slow: 1234567890\r\n\r\n':
            try:
                sock.send(bytes([byte]))
            except OSError:
                closed_at = time.monotonic()
                break
            time.sleep(0.05)  # each byte well inside a per-recv timeout; the total exceeds the deadline
            if self.server.stats()['timed_out']:
                closed_at = time.monotonic()
                break
        self.assertIsNotNone(closed_at, 'slow header never cut')
        self.assertLess(closed_at - started, self.HEADER_S + 0.6)
        self.wait(lambda: self.server.stats()['active'] == 0, 2, 'slot freed')
        self.assertEqual(self.get(), 200)

    def test_unread_and_slowly_read_large_response_are_cut_at_the_request_deadline(self):
        self.add_big_scene(1500000)  # 24 MB payload, far beyond the socket buffers
        unread = self.connect(rcvbuf=4096)
        unread.sendall(b'GET /api/frame?since=0 HTTP/1.1\r\nHost: x\r\n\r\n')
        slow = self.connect(rcvbuf=4096)
        slow.sendall(b'GET /api/frame?since=0 HTTP/1.1\r\nHost: x\r\n\r\n')
        self.wait(lambda: self.server.stats()['active'] == 2, 2, 'both admitted')
        # ROS-side work is not held up by the blocked writers (older stamps: the big scene stays the newest).
        started = time.monotonic()
        for i in range(20):
            self.store.add_scan(scan_json(-1 - i, {}, stages=False))
            self.state.note_scan(-1 - i)
            self.state.status()
        self.assertLess(time.monotonic() - started, 0.5)
        received = 0
        deadline = time.monotonic() + self.REQUEST_S + 2.0
        slow.settimeout(0.3)
        while time.monotonic() < deadline and self.server.stats()['active']:
            try:
                data = slow.recv(512)
                if not data:
                    break
                received += len(data)
            except socket.timeout:
                pass
            time.sleep(0.05)
        self.wait(lambda: self.server.stats()['active'] == 0, 2, 'slots freed at the request deadline')
        self.assertGreater(received, 0)
        self.assertLess(received, 24000000)
        self.assertGreaterEqual(self.server.stats()['timed_out'], 2)
        self.assertEqual(self.get(), 200)

    def test_more_connections_than_the_limit_then_recovery_and_no_growth(self):
        for _ in range(5):  # repeated rounds: threads and tracked connections return to zero
            held = []
            for _ in range(self.MAX):
                sock = self.connect()
                sock.sendall(b'GET /api/health HTTP/1.1\r\n')
                held.append(sock)
            self.wait(lambda: self.server.stats()['active'] == self.MAX, 2, 'slots taken')
            others = [self.connect() for _ in range(10)]
            for sock in others:
                sock.sendall(b'GET /api/health HTTP/1.1\r\nHost: x\r\n\r\n')
            for sock in others:
                self.assertTrue(self.read_all(sock, 3).startswith(b'HTTP/1.1 503'))
            self.assertLessEqual(self.handler_threads(), self.MAX)
            for sock in held:  # clients give up: slots come back before any deadline
                sock.close()
            self.wait(lambda: self.server.stats()['active'] == 0, 1.0, 'slots freed by closing clients')
            self.assertEqual(self.get(), 200)
            for sock in held + others:
                self.sockets.remove(sock)
                sock.close()
        self.wait(lambda: self.handler_threads() <= 0, 2, 'no handler thread left')
        stats = self.server.stats()
        self.assertEqual((stats['active'], stats['reject_queue']), (0, 0))
        self.assertGreaterEqual(stats['rejected_busy'], 50)


class BrowserTest(unittest.TestCase):
    def test_docker_never_spawns(self):
        calls = []
        original = core._spawn
        core._spawn = lambda command: calls.append(command) or True
        try:
            opened, how = core.open_browser_once('http://127.0.0.1:1/', {'docker': True, 'wsl': False, 'display': False,
                                                                        'browser_env': ''}, which=lambda n: True)
        finally:
            core._spawn = original
        self.assertFalse(opened)
        self.assertIn('host', how)
        self.assertEqual(calls, [])

    def test_wsl_prefers_wslview_then_powershell(self):
        calls = []
        original = core._spawn
        core._spawn = lambda command: calls.append(command) or True
        try:
            info = {'docker': False, 'wsl': True, 'display': False, 'browser_env': ''}
            opened, how = core.open_browser_once('http://127.0.0.1:1/', info, which=lambda n: n == 'wslview')
            self.assertTrue(opened)
            self.assertIn('wslview', how)
            opened, how = core.open_browser_once('http://127.0.0.1:1/', info, which=lambda n: n == 'powershell.exe')
            self.assertTrue(opened)
            self.assertIn('powershell', how)
            opened, how = core.open_browser_once('http://127.0.0.1:1/', info, which=lambda n: False)
            self.assertFalse(opened)
        finally:
            core._spawn = original
        self.assertEqual(calls[0][0], 'wslview')
        self.assertEqual(calls[1][0], 'powershell.exe')

    def test_headless_linux_without_display(self):
        opened, how = core.open_browser_once('http://127.0.0.1:1/', {'docker': False, 'wsl': False, 'display': False,
                                                                    'browser_env': ''}, which=lambda n: False)
        self.assertFalse(opened)
        self.assertIn('manually', how)

    def test_wait_for_server_retries_until_healthy(self):
        clock = FakeClock()
        attempts = []

        class Response(object):
            status = 200

            def __enter__(self):
                return self

            def __exit__(self, *args):
                return False

        def opener(url, timeout):
            attempts.append(url)
            clock.t += 0.3
            if len(attempts) < 3:
                raise OSError('refused')
            return Response()

        self.assertTrue(core.wait_for_server('http://127.0.0.1:1/', timeout_s=5, interval_s=0, clock=clock, opener=opener))
        self.assertEqual(len(attempts), 3)
        self.assertTrue(attempts[0].endswith('/api/health'))
        def always_down(url, timeout):
            clock.t += 0.3
            raise OSError('refused')
        self.assertFalse(core.wait_for_server('http://127.0.0.1:1/', timeout_s=0.5, interval_s=0, clock=clock,
                                              opener=always_down))


if __name__ == '__main__':
    unittest.main()
