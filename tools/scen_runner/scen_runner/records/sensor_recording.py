"""Opt-in ROS evidence for a registered scenario run; never controls the vehicle.

The host storage manager reserves the run first. This component owns only its
rosbag process, uses bounded buffers, and preserves incomplete evidence on failure.
"""
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import time


SHADOW_TOPICS = (
    '/image_jpeg/compressed', '/cam_left/image_jpeg/compressed', '/cam_right/image_jpeg/compressed',
    '/velodyne_points', '/imu', '/gps', '/Ego_topic', '/morai_sensor_bridge/frame_timing',
    '/perception/bev/camera/drivable_probability', '/perception/bev/camera/coverage',
    '/CollisionData', '/morai_sensor_bridge/diagnostics', '/morai_udp_bridge/diagnostics',
    '/perception/camera/objects', '/perception/camera/objects/observations', '/perception/camera/objects/status',
    '/perception/camera/lane/status', '/perception/lane/boundary_family', '/perception/lane/ground_projection',
    '/perception/lidar/tracks_local', '/perception/lidar/planning_observations_map', '/perception/lidar/diagnostics',
    '/planning/camera_bbox_lane/frame', '/planning/camera_bbox_lane/result', '/planning/camera_bbox_lane/diagnostics',
    '/planning/camera_bbox_lane_change/diagnostics', '/planning/camera_bbox_lane_change/markers',
    '/planning/camera_bbox_lane_change/reference_lane', '/planning/objects', '/planning/vehicle/state',
    '/planning/mode', '/planning/control_selector/diagnostics', '/diagnostics',
    '/behavior/context', '/behavior/command_v2', '/planning/trajectory', '/planning/frenet_feedback_v2',
    '/ctrl_cmd/normal', '/ctrl_cmd/jamming', '/ctrl_cmd', '/rosout_agg',
)
REQUIRED_TOPICS = (
    '/image_jpeg/compressed', '/velodyne_points', '/imu', '/gps', '/morai_sensor_bridge/frame_timing',
    '/perception/camera/objects', '/perception/lane/boundary_family', '/perception/lane/ground_projection',
    '/perception/lidar/tracks_local', '/planning/camera_bbox_lane/result',
    '/planning/camera_bbox_lane_change/diagnostics', '/planning/control_selector/diagnostics',
    '/planning/mode', '/ctrl_cmd/jamming', '/ctrl_cmd',
)


def write_json(path, payload):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    temporary.replace(path)


def recording_directory(spec):
    run_id = spec['run_id']
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{1,100}', run_id):
        raise ValueError('invalid recording run ID')
    root = Path(spec['run_root']).resolve(strict=True)
    # Accept host/container paths, but no fallback directory or unregistered run.
    if root.name != run_id:
        raise ValueError('recording run ID/path mismatch')
    registration = json.loads((root/'registration.json').read_text())
    if (registration.get('run_id') != run_id or registration.get('status') != 'active' or
            registration.get('kind') != 'experiment' or registration.get('output') != 'artifacts/'+run_id):
        raise ValueError('active experiment registration required')
    return root/'ros'


class SensorRecording:
    def __init__(self, spec_path, notify=print):
        self.spec = json.loads(Path(spec_path).read_text())
        self.directory = recording_directory(self.spec)
        self.notify = notify
        self.process = self.log = None
        self.closed = False
        self.started = False
        self.last_check = 0.
        self.max_bytes = int(self.spec.get('max_bytes', 8*2**30))
        self.min_free = int(self.spec.get('minimum_remaining_bytes', 2*2**30))
        if self.max_bytes <= 0 or self.min_free < 0:
            raise ValueError('invalid recording storage limits')
        self.manifest = dict(schema='asmc.scenario_ros_recording.v1', run_id=self.spec['run_id'],
            status='created', topics=list(SHADOW_TOPICS), required_topics=list(REQUIRED_TOPICS),
            time_policy='bag receipt is ROS time; source header retained; monotonic used only for local durations',
            spec=self.spec)

    def start(self):
        self.directory.mkdir(exist_ok=False)
        self.started = True
        self.log = (self.directory/'rosbag.log').open('x')
        self.command = ['rosbag', 'record', '--lz4', '--buffsize=256', '--split', '--duration=60',
                        '-O', str(self.directory/'sensors'), *SHADOW_TOPICS,
                        '__name:=s1_recorder_'+self.spec['run_id'].replace('-', '_')]
        self.manifest.update(status='starting', started_unix=time.time(), command=self.command)
        write_json(self.directory/'manifest.json', self.manifest)
        try:
            self.process = subprocess.Popen(self.command, stdout=self.log, stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic()+10.
            while not list(self.directory.glob('*.bag.active')):
                self.check(force=True)
                if time.monotonic() >= deadline:
                    raise RuntimeError('rosbag did not open an output file')
                time.sleep(.05)
            self.manifest.update(status='recording', pid=self.process.pid)
            write_json(self.directory/'manifest.json', self.manifest)
        except Exception:
            self.close()
            raise
        self.notify('[센서 기록 시작] '+str(self.directory))

    def check(self, force=False):
        if self.process is None or self.process.poll() is not None:
            raise RuntimeError('sensor recorder exited unexpectedly')
        now = time.monotonic()
        if not force and now-self.last_check < 1.:
            return
        self.last_check = now
        used = sum(p.stat().st_size for p in self.directory.glob('*.bag*'))
        if used >= self.max_bytes or shutil.disk_usage(self.directory).free < self.min_free:
            raise RuntimeError('sensor recording storage budget exhausted; stop trial, retain evidence')

    def close(self):
        if self.closed:
            return self.manifest
        self.closed = True
        if not self.started:
            return self.manifest
        errors = []
        if self.process:
            if self.process.poll() is None:
                os.killpg(self.process.pid, signal.SIGINT)
                try:
                    self.process.wait(timeout=20)
                except subprocess.TimeoutExpired:
                    os.killpg(self.process.pid, signal.SIGKILL)
                    self.process.wait(timeout=5)
                    errors.append('recorder_shutdown_timeout')
            if self.process.returncode not in (0, -signal.SIGINT):
                errors.append('recorder_exit_'+str(self.process.returncode))
        else:
            errors.append('recorder_not_started')
        if self.log:
            self.log.close()
        # rosbag's Python launcher can return before its record child has
        # finished renaming the final split. Never inventory only the first bag.
        if self.process:
            deadline=time.monotonic()+20.
            while list(self.directory.glob('*.active')) and time.monotonic()<deadline:
                time.sleep(.05)
        bags = sorted(self.directory.glob('*.bag'))
        counts = {}
        types = {}
        files = []
        for path in bags:
            try:
                import rosbag
                with rosbag.Bag(str(path)) as bag:
                    for name, info in bag.get_type_and_topic_info().topics.items():
                        counts[name] = counts.get(name, 0)+info.message_count
                        types[name] = dict(type=info.msg_type)
                    for connection in bag._get_connections():
                        types.setdefault(connection.topic, {})['md5'] = connection.md5sum
                    files.append(dict(file=path.name, bytes=path.stat().st_size,
                                      start=bag.get_start_time(), end=bag.get_end_time()))
                digest = hashlib.sha256()
                with path.open('rb') as stream:
                    for block in iter(lambda: stream.read(4*2**20), b''):
                        digest.update(block)
                files[-1]['sha256'] = digest.hexdigest()
            except Exception as error:
                errors.append(path.name+': '+str(error))
        if {p.name:p.stat().st_size for p in self.directory.glob('*.bag')} != {f['file']:f['bytes'] for f in files}:
            errors.append('final_bag_inventory_changed')
        missing = [topic for topic in REQUIRED_TOPICS if not counts.get(topic)]
        if list(self.directory.glob('*.active')):
            errors.append('unclosed_bag')
        log_path = self.directory/'rosbag.log'
        if log_path.exists() and ('buffer exceeded' in log_path.read_text(errors='replace').lower()):
            errors.append('rosbag_buffer_overflow')
        self.manifest.update(status='incomplete' if errors or missing else 'complete',
                             ended_unix=time.time(), files=files, topic_counts=counts,
                             topic_types=types,
                             missing_required_topics=missing, errors=errors)
        write_json(self.directory/'manifest.json', self.manifest)
        self.notify('[센서 기록 종료] '+self.manifest['status']+' · '+str(self.directory))
        return self.manifest
