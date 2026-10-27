#!/usr/bin/env python3
"""Pure-Python core of the asmc_lidar read-only web monitor (no ROS import).

- parse_point_cloud: sensor_msgs/PointCloud2 fields/data -> numpy arrays
- SceneStore: groups the debug messages of one scan by source stamp
  (scan JSON + raw/roi/voxel/leveled clouds), latest-wins, bounded, and
  encodes each ready scene once into a binary frame payload
- MonitorState: status snapshot (ages, rates, pipeline/map/camera state)
- MonitorServer: HTTP server with fixed routes (web assets, /api/health,
  /api/status, /api/frame) bound to loopback by default; read-only. At most
  max_clients connections are handled at once (checked at accept, before any
  request byte is parsed; more get a bounded 503 or are closed), and every
  connection has a header deadline and a total deadline enforced by one reaper
  thread (a slow client only ever holds its own slot)
- open_browser_once / wait_for_server: browser opener used by the node

The frame format (/api/frame?since=<frame_seq>&session=<session>):
  b'ALMF' | uint32 header_len | uint32 payload_len | header JSON (utf-8) |
  zero padding to a multiple of 4 | payload (little-endian arrays, each
  4-byte aligned; the header's "arrays" table gives offset/length/dtype).
The header carries the server session (new per monitor process), frame_seq of
the newest ready scene and its age (monitor monotonic clock, ms since its first
part arrived). The scene is included when it is newer than `since` or when the
client's session differs (a restarted server numbers frames from 1 again).
"""
import http.server
import json
import os
import queue
import select
import socket
import struct
import subprocess
import threading
import time
import urllib.parse
import urllib.request
import webbrowser

import numpy as np

SCHEMA = 'asmc_lidar_debug_scan_v1'
STAGES = ('raw', 'roi', 'voxel', 'leveled')
MAGIC = b'ALMF'
WEB_FILES = {
    '/': ('index.html', 'text/html; charset=utf-8'),
    '/index.html': ('index.html', 'text/html; charset=utf-8'),
    '/app.js': ('app.js', 'application/javascript; charset=utf-8'),
    '/monitor_view.js': ('monitor_view.js', 'application/javascript; charset=utf-8'),
    '/app.css': ('app.css', 'text/css; charset=utf-8'),
}
POINT_FIELD_DTYPES = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}
DTYPE_NAMES = {np.dtype(np.float32): 'f32', np.dtype(np.uint8): 'u8', np.dtype(np.int32): 'i32'}


# --------------------------------------------------------------------------- clouds
def parse_point_cloud(fields, point_step, width, height, data, is_bigendian=False):
    """fields: iterable of (name, offset, datatype, count) as in sensor_msgs/PointField.

    Returns {'xyzi': float32 (n, 4), 'label': uint8 (n,) or None, 'cluster': int32 (n,) or None,
    'count': n}. Missing intensity is 0. Raises ValueError without x/y/z or on a short buffer.
    """
    n = int(width) * int(height)
    names, formats, offsets = [], [], []
    order = '>' if is_bigendian else '<'
    for name, offset, datatype, count in fields:
        if int(count) != 1 or int(datatype) not in POINT_FIELD_DTYPES:
            continue
        names.append(str(name))
        formats.append(order + POINT_FIELD_DTYPES[int(datatype)])
        offsets.append(int(offset))
    for key in ('x', 'y', 'z'):
        if key not in names:
            raise ValueError('point cloud without field ' + key)
    if n * int(point_step) > len(data):
        raise ValueError('point cloud data shorter than width*height*point_step')
    dtype = np.dtype({'names': names, 'formats': formats, 'offsets': offsets, 'itemsize': int(point_step)})
    arr = np.frombuffer(data, dtype=dtype, count=n)
    xyzi = np.empty((n, 4), dtype=np.float32)
    xyzi[:, 0] = arr['x']
    xyzi[:, 1] = arr['y']
    xyzi[:, 2] = arr['z']
    xyzi[:, 3] = arr['intensity'] if 'intensity' in names else 0.0
    label = np.ascontiguousarray(arr['label'], dtype=np.uint8) if 'label' in names else None
    cluster = np.ascontiguousarray(arr['cluster'], dtype=np.int32) if 'cluster' in names else None
    track = np.ascontiguousarray(arr['track'], dtype=np.int32) if 'track' in names else None
    return {'xyzi': xyzi, 'label': label, 'cluster': cluster, 'track':track, 'road':np.ascontiguousarray(arr['road'],dtype=np.uint8) if 'road' in names else None, 'count': n}


# --------------------------------------------------------------------------- scenes
class Scene(object):
    __slots__ = ('stamp_ns', 'meta', 'clouds', 'expected', 'created', 'ready', 'frame_seq', 'payload',
                 'arrays', 'missing', 'ready_at')

    def __init__(self, stamp_ns, created):
        self.stamp_ns = stamp_ns
        self.meta = None
        self.clouds = {}
        self.expected = set()
        self.created = created
        self.ready = False
        self.frame_seq = 0
        self.payload = b''
        self.arrays = {}
        self.missing = []
        self.ready_at = 0.0


def encode_scene(clouds):
    """Concatenate the stage arrays (4-byte aligned) and return (arrays table, payload bytes)."""
    parts, table, offset = [], {}, 0

    def put(name, array):
        nonlocal offset
        raw = np.ascontiguousarray(array).tobytes()
        pad = (-len(raw)) % 4
        table[name] = {'offset': offset, 'length': int(array.size), 'dtype': DTYPE_NAMES[np.dtype(array.dtype)]}
        parts.append(raw)
        if pad:
            parts.append(b'\0' * pad)
        offset += len(raw) + pad

    for stage in STAGES:
        cloud = clouds.get(stage)
        if cloud is None:
            continue
        put(stage + '.xyzi', cloud['xyzi'].astype(np.float32, copy=False))
        if cloud.get('road') is not None:
            put(stage + '.road', cloud['road'])
        if cloud.get('label') is not None:
            put(stage + '.label', cloud['label'])
        if cloud.get('cluster') is not None:
            put(stage + '.cluster', cloud['cluster'])
        if cloud.get('track') is not None:
            put(stage + '.track', cloud['track'])
    return table, b''.join(parts)


class SceneStore(object):
    """Groups debug messages by source stamp. Thread-safe. Keeps at most max_scenes scenes.

    A scene is ready when its scan JSON and every stage cloud the JSON announced
    (sampled.<stage>.sent > 0) arrived, or when complete_timeout_s passed since
    its first part (then it is ready with `missing` stages). The newest ready
    scene is served; older ones are evicted.
    """

    def __init__(self, max_scenes=4, complete_timeout_s=0.6, clock=time.monotonic, metadata_first=False):
        self.metadata_first = bool(metadata_first)
        self.max_scenes = max(1, int(max_scenes))
        self.complete_timeout_s = float(complete_timeout_s)
        self.clock = clock
        self._lock = threading.Lock()
        self._scenes = {}
        self._order = []
        self._frame_seq = 0
        self._latest = None
        self._producer_session=None;self._generation=0;self._retired_sessions=[];self._cloud_floor=-1
        self.counters = {'scan': 0, 'raw': 0, 'roi': 0, 'voxel': 0, 'leveled': 0, 'bad': 0,
                         'evicted_incomplete': 0, 'partial': 0, 'complete': 0}

    def add_scan(self, text):
        try:
            meta = json.loads(text)
        except ValueError:
            self.counters['bad'] += 1
            return None
        if not isinstance(meta, dict) or meta.get('schema') != SCHEMA or 'stamp_ns' not in meta:
            self.counters['bad'] += 1
            return None
        stamp = int(meta['stamp_ns'])
        meta['stamp_ns'] = stamp
        with self._lock:
            session=meta.get('producer_session_id');generation=int(meta.get('generation',1))
            if session:
                if session in self._retired_sessions or (session==self._producer_session and generation<self._generation):
                    return None
                if session!=self._producer_session or generation>self._generation:
                    if self._producer_session and session!=self._producer_session:
                        self._retired_sessions=(self._retired_sessions+[self._producer_session])[-16:]
                    # Debug clouds have only a stamp, no producer UUID. Discard
                    # the ambiguous boundary scan's clouds after a restart/reset.
                    self._scenes.clear();self._order.clear();self._latest=None;self._cloud_floor=stamp
                    self._producer_session=session;self._generation=generation
            self.counters['scan'] += 1
            scene = self._get(stamp)
            scene.meta = meta
            expected = set()
            if meta.get('stages'):
                sampled = meta.get('sampled') or {}
                for stage in STAGES:
                    if int((sampled.get(stage) or {}).get('sent', 0)) > 0:
                        expected.add(stage)
            scene.expected = expected
            self._check(scene)
            if self.metadata_first and not scene.ready and (self._latest is None or stamp > self._latest.stamp_ns):
                # Immutable metadata-only snapshot: show this scan's OBB/audit
                # immediately. Never reuse another scan's cloud or refresh age.
                preview=Scene(stamp,scene.created)
                preview.meta=meta;preview.expected=set(scene.expected)
                preview.ready=True;preview.missing=sorted(scene.expected)
                preview.ready_at=self.clock()
                self._frame_seq+=1;preview.frame_seq=self._frame_seq
                self._latest=preview
            self._sweep()
        return meta

    def add_cloud(self, stage, stamp_ns, cloud):
        if stage not in STAGES:
            self.counters['bad'] += 1
            return
        with self._lock:
            if int(stamp_ns)<=self._cloud_floor:return
            self.counters[stage] += 1
            scene = self._get(int(stamp_ns))
            scene.clouds[stage] = cloud
            self._check(scene)
            self._sweep()

    def latest(self):
        with self._lock:
            self._sweep()
            return self._latest

    def frame_seq(self):
        with self._lock:
            return self._frame_seq

    def scene_age_s(self, scene):
        """Seconds since the first part of `scene` reached the monitor (store clock)."""
        return max(0.0, self.clock() - scene.created)

    def latest_info(self):
        """frame_seq, stamp and age of the newest ready scene, or None."""
        scene = self.latest()
        if scene is None:
            return None
        return {'frame_seq': scene.frame_seq, 'stamp_ns': scene.stamp_ns,
                'age_ms': int(round(self.scene_age_s(scene) * 1000))}

    def _get(self, stamp):
        scene = self._scenes.get(stamp)
        if scene is None:
            scene = Scene(stamp, self.clock())
            self._scenes[stamp] = scene
            self._order.append(stamp)
            while len(self._order) > self.max_scenes:
                old = self._order.pop(0)
                dropped = self._scenes.pop(old)
                if not dropped.ready:
                    self.counters['evicted_incomplete'] += 1
                if self._latest is dropped:
                    self._latest = None
        return scene

    def _check(self, scene):
        if scene.ready or scene.meta is None:
            return
        if scene.expected <= set(scene.clouds):
            self._finish(scene, [])

    def _sweep(self):
        now = self.clock()
        for stamp in list(self._order):
            scene = self._scenes[stamp]
            if scene.ready or scene.meta is None:
                continue
            if now - scene.created >= self.complete_timeout_s:
                self._finish(scene, sorted(scene.expected - set(scene.clouds)))
                self.counters['partial'] += 1

    def _finish(self, scene, missing):
        scene.ready = True
        scene.missing = list(missing)
        scene.ready_at = self.clock()
        self._frame_seq += 1
        scene.frame_seq = self._frame_seq
        clouds = {stage: cloud for stage, cloud in scene.clouds.items() if stage in scene.expected}
        scene.arrays, scene.payload = encode_scene(clouds)
        if not missing:
            self.counters['complete'] += 1
        if self._latest is None or scene.stamp_ns >= self._latest.stamp_ns:
            self._latest = scene


def build_frame(status, scene, include_scene, session='', scene_age_s=None):
    """Binary frame: status always; the scene and its arrays only when include_scene.

    frame_seq and scene_age_ms describe `scene` (the newest ready scene) whether or
    not it is included, so a client can tell "no newer scene" from "not delivered".
    """
    header = {'session': str(session), 'frame_seq': scene.frame_seq if scene else 0,
              'scene_age_ms': None if scene is None or scene_age_s is None else int(round(scene_age_s * 1000)),
              'status': status, 'scene': None, 'arrays': {}}
    if scene is not None and include_scene:
        meta = dict(scene.meta or {})
        meta['missing'] = list(scene.missing)
        meta['frame_seq'] = scene.frame_seq
        header['scene'] = meta
        header['arrays'] = scene.arrays
        payload = scene.payload
    else:
        payload = b''
    head = json.dumps(header, separators=(',', ':'), allow_nan=False).encode('utf-8')
    pad = (-len(head)) % 4
    return b''.join([MAGIC, struct.pack('<II', len(head), len(payload)), head, b'\0' * pad, payload])


# --------------------------------------------------------------------------- status
def new_session_id(wall=time.time):
    """Wall start time (hex seconds) + 32 random bits: unique per monitor process."""
    return '%x-%s' % (int(wall()), os.urandom(4).hex())


class MonitorState(object):
    """Status inputs from the ROS node (any thread) and the status snapshot for the web page."""

    STALE_S = 1.0        # scan older than this: lidar_stale
    DIAG_LOST_S = 3.0    # no diagnostics: shadow node missing

    def __init__(self, store, clock=time.monotonic, wall=time.time, config=None, session_id=None):
        self.store = store
        self.clock = clock
        self.wall = wall
        self.config = dict(config or {})
        # Identifies this monitor process: frame_seq restarts at 1 in a new session.
        self.session_id = session_id or new_session_id(wall)
        self._lock = threading.Lock()
        self._diag = None
        self._diag_time = None
        self._diag_rate_samples = []
        self._objects = None
        self._objects_time = None
        self._objects_map = None
        self._objects_map_time = None
        self._scan_map = None  # (valid, reason) of the last scan JSON that reached the stages
        self._scan_time = None
        self._scan_stamp_ns = 0
        self._scan_count = 0
        self._master = {'checked': None, 'ok': None, 'error': '', 'points_publishers': 0,
                        'debug_publishers': 0, 'shadow_nodes': []}
        self.started = clock()

    def update_diagnostics(self, level, message, values):
        now = self.clock()
        with self._lock:
            self._diag = {'level': int(level), 'message': str(message), 'values': dict(values)}
            self._diag_time = now
            sample = (now, self._int(values.get('scans_received')), self._int(values.get('scans_processed')),
                      self._int(values.get('debug_published')))
            self._diag_rate_samples.append(sample)
            self._diag_rate_samples = self._diag_rate_samples[-6:]

    def update_objects_local(self, status, reason, stamp_ns, count):
        with self._lock:
            self._objects = {'status': int(status), 'reason': str(reason), 'stamp_ns': int(stamp_ns), 'count': int(count)}
            self._objects_time = self.clock()

    def update_objects_map(self, status, reason, stamp_ns):
        with self._lock:
            self._objects_map = {'status': int(status), 'reason': str(reason), 'stamp_ns': int(stamp_ns)}
            self._objects_map_time = self.clock()

    def note_scan(self, stamp_ns, map_valid=None, map_reason=None):
        with self._lock:
            self._scan_time = self.clock()
            self._scan_stamp_ns = int(stamp_ns)
            self._scan_count += 1
            if map_valid is not None:
                self._scan_map = (bool(map_valid), str(map_reason))

    def update_master(self, ok, error='', points_publishers=0, debug_publishers=0, shadow_nodes=()):
        with self._lock:
            self._master = {'checked': self.clock(), 'ok': bool(ok), 'error': str(error),
                            'points_publishers': int(points_publishers), 'debug_publishers': int(debug_publishers),
                            'shadow_nodes': list(shadow_nodes)}

    @staticmethod
    def _int(value):
        try:
            return int(float(value))
        except (TypeError, ValueError):
            return None

    @staticmethod
    def _float(value):
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    def _rates(self):
        samples = self._diag_rate_samples
        if len(samples) < 2:
            return {'input_hz': None, 'processed_hz': None, 'debug_hz': None}
        (t0, r0, p0, d0), (t1, r1, p1, d1) = samples[0], samples[-1]
        dt = t1 - t0
        if dt <= 0:
            return {'input_hz': None, 'processed_hz': None, 'debug_hz': None}

        def rate(a, b):
            return None if a is None or b is None else max(0.0, (b - a) / dt)
        return {'input_hz': rate(r0, r1), 'processed_hz': rate(p0, p1), 'debug_hz': rate(d0, d1)}

    def status(self):
        now = self.clock()
        with self._lock:
            diag = self._diag
            diag_age = None if self._diag_time is None else now - self._diag_time
            scan_age = None if self._scan_time is None else now - self._scan_time
            objects = self._objects
            objects_age = None if self._objects_time is None else now - self._objects_time
            objects_map = self._objects_map
            scan_map = self._scan_map
            master = dict(self._master)
            rates = self._rates()
            scan_stamp_ns = self._scan_stamp_ns
            scan_count = self._scan_count
        values = (diag or {}).get('values', {})
        master_age = None if master['checked'] is None else now - master['checked']
        # Pipeline/input state, most severe first.
        if master['ok'] is False:
            pipeline = 'ros_master_unreachable'
        elif diag is None or diag_age > self.DIAG_LOST_S:
            pipeline = 'no_shadow_node'
        elif diag['message'] == 'input_silence':
            pipeline = 'no_lidar_input'
        elif values.get('input_reason') not in (None, 'valid'):
            pipeline = 'input_invalid'
        elif scan_age is not None and scan_age > self.STALE_S:
            pipeline = 'lidar_stale'
        else:
            pipeline = 'ok'
        debug_enabled = values.get('debug_enabled')
        if diag is None:
            debug = 'unknown'
        elif debug_enabled == 'false':
            debug = 'disabled'
        elif scan_age is None:
            debug = 'no_data'
        elif scan_age > self.STALE_S:
            debug = 'stale'
        else:
            debug = 'live'
        # Map availability: the typed map array status (POSE_UNAVAILABLE = 4) is authoritative; the
        # per-scan map.valid of the debug JSON is the fallback. The reason string alone cannot
        # decide it: a valid bounded pose also carries a reason (e.g. gps_bounded_prediction).
        if objects_map is not None:
            map_state = 'valid' if objects_map['status'] in (1, 2) else 'unavailable' if objects_map['status'] == 4 else 'input_invalid'
            map_reason = objects_map['reason']
        elif scan_map is not None:
            map_state = 'valid' if scan_map[0] else 'unavailable'
            map_reason = scan_map[1]
        else:
            map_state = 'unknown'
            map_reason = values.get('map_reason', 'unknown')
        camera_fusion = values.get('camera_fusion', 'unknown')
        return {
            'session': self.session_id,
            'latest_scene': self.store.latest_info(),
            'now_wall_ms': int(self.wall() * 1000),
            'uptime_s': round(now - self.started, 1),
            'pipeline_state': pipeline,
            'pipeline_detail': {
                'message': (diag or {}).get('message'),
                'input_reason': values.get('input_reason'),
                'pose_reason': values.get('pose_reason'),
                'last_reject_reason': values.get('last_reject_reason'),
            },
            'map_state': map_state,
            'map_reason': map_reason,
            'objects_map': objects_map,
            'camera_state': camera_fusion,
            'camera_active': values.get('camera_active'),
            'camera_reason': values.get('camera_reason'),
            'tunnel_filter': values.get('tunnel_filter'),
            'roll_leveling': values.get('roll_leveling'),
            'debug_state': debug,
            'debug_enabled': debug_enabled,
            'ages_ms': {
                'diagnostics': None if diag_age is None else int(diag_age * 1000),
                'scan': None if scan_age is None else int(scan_age * 1000),
                'objects_local': None if objects_age is None else int(objects_age * 1000),
                'master': None if master_age is None else int(master_age * 1000),
            },
            'scan_stamp_ns': scan_stamp_ns,
            'scan_count': scan_count,
            'rates': rates,
            'diagnostics': values,
            'diagnostics_level': (diag or {}).get('level'),
            'objects_local': objects,
            'master': master,
            'store': dict(self.store.counters),
            'frame_seq': self.store.frame_seq(),
            'config': self.config,
        }


# --------------------------------------------------------------------------- http
class MonitorHandler(http.server.BaseHTTPRequestHandler):
    server_version = 'asmc_lidar_monitor/1'
    sys_version = ''
    protocol_version = 'HTTP/1.1'

    def log_message(self, fmt, *args):  # quiet; errors go through log_error
        logger = getattr(self.server, 'logger', None)
        if logger:
            logger('debug', '%s %s' % (self.address_string(), fmt % args))

    def log_error(self, fmt, *args):
        logger = getattr(self.server, 'logger', None)
        if logger:
            logger('warn', '%s %s' % (self.address_string(), fmt % args))

    def parse_request(self):
        ok = super().parse_request()
        if ok:
            self.server.request_parsed(self.connection)
        return ok

    def do_HEAD(self):
        self._handle(head=True)

    def do_GET(self):
        self._handle(head=False)

    def _handle(self, head):
        server = self.server
        path, _, query = self.path.partition('?')
        if path in WEB_FILES:
            self._file(path, head)
        elif path == '/favicon.ico':
            self._send(204, 'text/plain', b'', head)
        elif path == '/api/health':
            body = json.dumps({'ok': True, 'service': 'asmc_lidar_monitor', 'session': server.state.session_id}).encode()
            self._send(200, 'application/json', body, head)
        elif path == '/api/status':
            body = json.dumps(server.status_snapshot(), separators=(',', ':'), allow_nan=False).encode()
            self._send(200, 'application/json', body, head)
        elif path == '/api/drivable':
            self._send(200, 'application/json', server.drivable_body, head)
        elif path == '/api/frame':
            self._frame(query, head)
        else:
            self._send(404, 'text/plain; charset=utf-8', b'not found\n', head)

    def _frame(self, query, head):
        since, session = 0, ''
        for part in query.split('&'):
            key, _, value = part.partition('=')
            if key == 'since':
                try:
                    since = int(value)
                except ValueError:
                    since = 0
            elif key == 'session':
                session = urllib.parse.unquote(value)
        state = self.server.state
        scene = state.store.latest()
        # A cursor of another session (given, or ahead of this server's numbering) gets the scene at once.
        include = scene is not None and (scene.frame_seq > since or since > scene.frame_seq
                                         or (session != '' and session != state.session_id))
        age = state.store.scene_age_s(scene) if scene is not None else None
        body = build_frame(self.server.status_snapshot(), scene, include, state.session_id, age)
        self._send(200, 'application/octet-stream', body, head)

    def _file(self, path, head):
        name, content_type = WEB_FILES[path]
        full = os.path.join(self.server.web_dir, name)
        try:
            with open(full, 'rb') as stream:
                body = stream.read()
        except OSError:
            self._send(404, 'text/plain; charset=utf-8', ('missing web asset %s\n' % name).encode(), head)
            return
        self._send(200, content_type, body, head)

    def _send(self, code, content_type, body, head):
        self.close_connection = True
        try:
            self.send_response(code)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate')
            self.send_header('Connection', 'close')
            self.end_headers()
            if not head:
                self.wfile.write(body)
        except OSError:  # peer gone, write timeout or socket shut down at the deadline
            pass


class _Connection(object):
    __slots__ = ('accepted', 'deadline', 'parsed', 'timed_out')

    def __init__(self, accepted, deadline):
        self.accepted = accepted
        self.deadline = deadline
        self.parsed = False
        self.timed_out = False


class MonitorServer(http.server.HTTPServer):
    """HTTP server with a bounded number of handler threads and per-connection deadlines.

    A connection beyond max_clients never gets a handler thread: right after accept
    (before any request byte is read) it goes to a bounded queue of one rejecter
    thread that answers 503 without blocking, or is closed at once when that queue
    is full. An admitted connection must deliver its request line and headers within
    header_timeout_s of its accept and be finished (response written) within
    request_timeout_s; one reaper thread shuts the socket down at the deadline, which
    ends the handler and frees the slot. Socket reads and writes carry a timeout too.
    Handlers only take the store/state locks for short copies, so a slow client never
    holds up the ROS callbacks.
    """
    allow_reuse_address = True
    request_queue_size = 16
    REJECT_QUEUE = 32
    REJECT_WAIT_S = 0.2
    REAP_INTERVAL_S = 0.1
    REJECT_RESPONSE = (b'HTTP/1.1 503 Service Unavailable\r\nContent-Type: text/plain; charset=utf-8\r\n'
                       b'Content-Length: 17\r\nCache-Control: no-store\r\nConnection: close\r\n\r\ntoo many clients\n')

    def __init__(self, bind_address, port, state, web_dir, max_clients=8, logger=None,
                 header_timeout_s=5.0, request_timeout_s=20.0, drivable=None):
        self.state = state
        self.drivable_body = json.dumps(drivable or {'available': False, 'reason': '지도 미설정'},
                                        separators=(',', ':'), allow_nan=False).encode()
        self.web_dir = web_dir
        self.max_clients = max(1, int(max_clients))
        self.header_timeout_s = max(0.1, float(header_timeout_s))
        self.request_timeout_s = max(self.header_timeout_s, float(request_timeout_s))
        self.limiter = threading.BoundedSemaphore(self.max_clients)
        self.logger = logger
        self._connections = {}  # socket -> _Connection; a socket is closed only after leaving this map
        self._lock = threading.Lock()
        self._stats = {'accepted': 0, 'served': 0, 'timed_out': 0, 'rejected_busy': 0, 'rejected_dropped': 0,
                       'errors': 0}
        self._rejects = queue.Queue(maxsize=self.REJECT_QUEUE)
        self._stop = threading.Event()
        # TCPServer.__init__ calls our server_close() if bind/activate fails.
        # Initialise cleanup state before binding so the original OSError
        # reaches the node's clear port-conflict handler without being masked.
        self._workers = []
        super().__init__((bind_address, int(port)), MonitorHandler)
        self.socket.settimeout(None)
        self._workers = [threading.Thread(target=self._reap_loop, name='monitor_http_reaper', daemon=True),
                         threading.Thread(target=self._reject_loop, name='monitor_http_reject', daemon=True)]
        for worker in self._workers:
            worker.start()

    # -- admission (accept thread)
    def process_request(self, request, client_address):
        if not self.limiter.acquire(blocking=False):
            self._count('rejected_busy')
            try:
                self._rejects.put_nowait(request)
            except queue.Full:
                self._count('rejected_dropped')
                self.shutdown_request(request)
            return
        now = time.monotonic()
        with self._lock:
            self._connections[request] = _Connection(now, now + self.header_timeout_s)
            self._stats['accepted'] += 1
        try:
            request.settimeout(self.header_timeout_s)
            threading.Thread(target=self._serve, args=(request, client_address), daemon=True).start()
        except Exception:  # pylint: disable=broad-except
            self._count('errors')
            self._release(request)
            self.shutdown_request(request)

    def _serve(self, request, client_address):
        try:
            self.finish_request(request, client_address)
        except Exception:  # pylint: disable=broad-except
            self.handle_error(request, client_address)
        finally:
            self._release(request)
            self.shutdown_request(request)

    def _release(self, request):
        now = time.monotonic()
        with self._lock:
            conn = self._connections.pop(request, None)
            if conn is not None:
                # The socket timeouts end at the same deadline as the reaper; either one counts.
                self._stats['timed_out' if conn.timed_out or now >= conn.deadline else 'served'] += 1
        if conn is not None:
            self.limiter.release()

    def request_parsed(self, sock):
        """Headers complete: the response may take until request_timeout_s after accept."""
        with self._lock:
            conn = self._connections.get(sock)
            if conn is None:
                return
            conn.parsed = True
            conn.deadline = conn.accepted + self.request_timeout_s
            remaining = conn.deadline - time.monotonic()
        try:
            sock.settimeout(max(0.05, remaining))  # sendall: total duration of the write
        except OSError:
            pass

    # -- deadlines and rejections (one thread each)
    def _reap_loop(self):
        while not self._stop.wait(self.REAP_INTERVAL_S):
            now = time.monotonic()
            with self._lock:
                for sock, conn in self._connections.items():
                    if conn.timed_out or now < conn.deadline:
                        continue
                    conn.timed_out = True
                    try:
                        sock.shutdown(socket.SHUT_RDWR)  # wakes the blocked handler; it then closes and frees the slot
                    except OSError:
                        pass
                    if self.logger:
                        self.logger('debug', 'connection closed at its %s deadline' %
                                    ('request' if conn.parsed else 'header'))

    def _reject_loop(self):
        while not self._stop.is_set():
            try:
                sock = self._rejects.get(timeout=0.2)
            except queue.Empty:
                continue
            self._reject(sock)

    def _reject(self, sock):
        try:
            # Wait briefly for the request so closing does not reset the 503 away; never block on the write.
            readable, _, _ = select.select([sock], [], [], self.REJECT_WAIT_S)
            sock.setblocking(False)
            if readable:
                sock.recv(65536)
            sock.send(self.REJECT_RESPONSE)
        except (OSError, ValueError):
            pass
        finally:
            self.shutdown_request(sock)

    def _count(self, key):
        with self._lock:
            self._stats[key] += 1

    def stats(self):
        with self._lock:
            out = dict(self._stats)
            out['active'] = len(self._connections)
        out.update({'max_clients': self.max_clients, 'header_timeout_s': self.header_timeout_s,
                    'request_timeout_s': self.request_timeout_s, 'reject_queue': self._rejects.qsize()})
        return out

    def status_snapshot(self):
        status = self.state.status()
        status['http'] = self.stats()
        return status

    def handle_error(self, request, client_address):
        self._count('errors')
        if self.logger:
            self.logger('debug', 'client %s dropped' % (client_address,))

    def server_close(self):
        self._stop.set()
        super().server_close()
        with self._lock:
            for sock in self._connections:
                try:
                    sock.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
        while True:
            try:
                self.shutdown_request(self._rejects.get_nowait())
            except queue.Empty:
                break
        for worker in self._workers:
            worker.join(timeout=1.0)

    @property
    def url(self):
        host, port = self.server_address[:2]
        shown = '127.0.0.1' if host in ('0.0.0.0', '') else host
        return 'http://%s:%d/' % (shown, port)


# --------------------------------------------------------------------------- browser
def environment(proc_version='/proc/version', env=None):
    env = os.environ if env is None else env
    info = {'wsl': False, 'docker': False, 'display': bool(env.get('DISPLAY') or env.get('WAYLAND_DISPLAY')),
            'browser_env': env.get('BROWSER', '')}
    try:
        with open(proc_version) as stream:
            info['wsl'] = 'microsoft' in stream.read().lower()
    except OSError:
        pass
    info['docker'] = os.path.exists('/.dockerenv')
    if not info['docker']:
        try:
            with open('/proc/1/cgroup') as stream:
                info['docker'] = 'docker' in stream.read() or 'containerd' in stream.read()
        except OSError:
            pass
    return info


def _spawn(command):
    try:
        subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                         stderr=subprocess.DEVNULL, start_new_session=True)
        return True
    except (OSError, ValueError):
        return False


def open_browser_once(url, info=None, which=None):
    """Try to open url once. Returns (opened, how). Never retries by itself."""
    info = environment() if info is None else info
    which = which or (lambda name: subprocess.run(['which', name], stdout=subprocess.PIPE,
                                                 stderr=subprocess.DEVNULL).returncode == 0)
    if info['docker'] and not info['wsl']:
        return False, 'docker container: no browser here; open the URL on the host (host network mode shares 127.0.0.1)'
    if info['wsl']:
        if which('wslview') and _spawn(['wslview', url]):
            return True, 'wslview (Windows default browser)'
        if which('powershell.exe') and _spawn(['powershell.exe', '-NoProfile', '-Command', 'Start-Process', "'%s'" % url]):
            return True, 'powershell.exe Start-Process'
        if which('cmd.exe') and _spawn(['cmd.exe', '/c', 'start', '', url]):
            return True, 'cmd.exe start'
        return False, 'WSL: wslview/powershell.exe/cmd.exe not found; open the URL in a Windows browser'
    if info['browser_env'] or info['display']:
        if info['display'] and which('xdg-open') and _spawn(['xdg-open', url]):
            return True, 'xdg-open'
        try:
            if webbrowser.open(url, new=2):
                return True, 'python webbrowser'
        except Exception:  # pylint: disable=broad-except
            pass
        return False, 'no browser command succeeded; open the URL manually'
    return False, 'no DISPLAY/BROWSER: open the URL manually'


def wait_for_server(url, timeout_s=10.0, interval_s=0.2, clock=time.monotonic, opener=urllib.request.urlopen):
    """Poll <url>/api/health until it answers 200 or timeout_s passes."""
    deadline = clock() + timeout_s
    health = url.rstrip('/') + '/api/health'
    while True:
        try:
            with opener(health, timeout=1.0) as response:
                if response.status == 200:
                    return True
        except Exception:  # pylint: disable=broad-except
            pass
        if clock() >= deadline:
            return False
        time.sleep(interval_s)
