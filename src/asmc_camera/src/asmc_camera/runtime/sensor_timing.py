"""Bounded source-clock joins and bracketed attitude, independent of ROS.

No receive/source clock subtraction, extrapolation or latest-pose substitution.
Missing evidence is an explicit fixed-plane fallback, never a repaired timestamp.
"""
from bisect import bisect_left
from collections import Counter, OrderedDict, deque
from dataclasses import dataclass
import threading
import time

import numpy as np


def sensor_key(topic, header):
    return str(topic), str(header.frame_id), int(header.stamp.to_nsec()), int(header.seq)


def quaternion_rotation(q):
    q = np.asarray(q, dtype=float)
    norm = float(np.linalg.norm(q))
    if q.shape != (4,) or not np.isfinite(q).all() or abs(norm - 1.) > .02:
        raise ValueError('invalid IMU quaternion')
    x, y, z, w = q / norm
    return np.asarray([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                       [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                       [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def slerp(a, b, fraction):
    quaternion_rotation(a); quaternion_rotation(b)
    a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
    a, b = a / np.linalg.norm(a), b / np.linalg.norm(b)
    if a @ b < 0:
        b = -b
    angle = np.arccos(np.clip(a @ b, -1., 1.))
    q = ((1-fraction)*a + fraction*b if angle < 1e-6 else
         (np.sin((1-fraction)*angle)*a + np.sin(fraction*angle)*b) / np.sin(angle))
    return q / np.linalg.norm(q)


class SensorTimingIndex:
    def __init__(self, capacity=256, retention_sec=2., clock=time.monotonic):
        if capacity < 2 or retention_sec <= 0:
            raise ValueError('invalid timing index bounds')
        self.capacity, self.retention, self.clock = capacity, retention_sec, clock
        self.lock = threading.RLock()
        self.entries = OrderedDict()
        self.session = None
        self.retired = deque(maxlen=16)
        self.retired_keys = OrderedDict()
        self.counters = Counter()

    def _prune(self):
        now = self.clock()
        while self.entries and (len(self.entries) > self.capacity or
                now - next(iter(self.entries.values()))[0] > self.retention):
            self.entries.popitem(last=False)
        while self.retired_keys and (len(self.retired_keys) > 2*self.capacity or
                now - next(iter(self.retired_keys.values())) > self.retention):
            self.retired_keys.popitem(last=False)

    def add(self, message):
        with self.lock:
            session = message.bridge_session_id
            if not session or session in self.retired:
                self.counters['retired_session'] += 1
                return False
            if self.session != session:
                if self.session is not None:
                    self.retired.append(self.session)
                    self.counters['session_reset'] += 1
                self.session = session
                for key in self.entries:
                    self.retired_keys[key] = self.clock()
                    self.retired_keys.move_to_end(key)
                self.entries.clear()
            if not message.sensor_wire_sequence_valid:
                self.counters['no_wire_sequence'] += 1
                return False
            key = (message.sensor_topic, message.sensor_frame_id,
                   message.published_stamp.to_nsec(), message.sensor_wire_sequence)
            # A repeated key is ambiguous, even if received twice with equal data.
            self._prune()
            value = None if key in self.entries or key in self.retired_keys else message
            self.entries[key] = (self.clock(), value)
            self.entries.move_to_end(key)
            self._prune()
            return True

    def lookup(self, key, max_age_sec=None):
        with self.lock:
            self._prune()
            pair = self.entries.get(key)
            if pair is None or pair[1] is None:
                return None
            if max_age_sec is not None and self.clock() - pair[0] > max_age_sec:
                return None
            return pair[1]


def source_usable(message):
    return (message.source_clock_domain == 'morai_sensor' and message.source_valid and
            not (message.source_duplicate or message.source_conflict or message.source_regression) and
            message.source_sec > 0 and 0 <= message.source_nsec < 1000000000)


@dataclass(frozen=True)
class Attitude:
    quaternion: tuple
    source_ns: int
    bracket_ms: float
    bridge_session_id: str


class ImuTimeline:
    def __init__(self, capacity=256, max_age_sec=.35, max_bracket_ms=100.,
                 retention_sec=2., clock=time.monotonic):
        if max_age_sec <= 0 or max_bracket_ms <= 0:
            raise ValueError('attitude bounds must be positive')
        self.index = SensorTimingIndex(capacity, retention_sec, clock)
        self.clock, self.max_age, self.max_bracket = clock, max_age_sec, max_bracket_ms * 1e6
        self.capacity, self.retention = capacity, retention_sec
        self.lock = threading.RLock()
        self.pending = OrderedDict()
        self.poses = OrderedDict()
        self.blocked = deque(maxlen=capacity)
        self.session = None
        self.counters = Counter()
        self.last_bracket_failure = None

    def add_timing(self, message):
        if message.stream_id != 'imu':
            return
        with self.lock:
            if not self.index.add(message):
                return
            if self.session != self.index.session:
                if self.session is not None:
                    self.pending.clear()
                self.session = self.index.session
                self.poses.clear(); self.blocked.clear()
                # Only first-session signals may precede metadata. A restart
                # flushes pending data rather than guessing its session.
            key = (message.sensor_topic, message.sensor_frame_id,
                   message.published_stamp.to_nsec(), message.sensor_wire_sequence)
            if self.index.lookup(key) is None:
                ns = int(message.source_sec) * 1000000000 + int(message.source_nsec)
                self.poses.pop(ns, None); self.blocked.append(ns)
                self.counters['ambiguous_wire_key'] += 1
                return
            self._join(key)

    def add_imu(self, key, q, frame_id):
        with self.lock:
            try:
                quaternion_rotation(q)
            except ValueError:
                self.counters['invalid_quaternion'] += 1
                return
            self.pending[key] = (self.clock(), tuple(float(x) for x in q), frame_id)
            self.pending.move_to_end(key)
            self._join(key)
            self._prune()

    def _prune(self):
        for table in (self.pending, self.poses):
            while table and (len(table) > self.capacity or
                    self.clock() - next(iter(table.values()))[0] > self.retention):
                table.popitem(last=False)

    def _join(self, key):
        metadata = self.index.lookup(key, self.max_age)
        signal = self.pending.pop(key, None) if metadata is not None else None
        if metadata is None or signal is None:
            return
        received, q, frame_id = signal
        ns = int(metadata.source_sec) * 1000000000 + int(metadata.source_nsec)
        if metadata.source_conflict or metadata.source_regression:
            if metadata.source_regression:
                self.poses.clear()
            self.poses.pop(ns, None); self.blocked.append(ns)
            self.counters['invalid_source'] += 1
            return
        # Identical retransmissions keep receipt evidence but add no pose.
        if not source_usable(metadata) or ns in self.blocked:
            self.counters['duplicate_or_invalid_source'] += 1
            return
        if ns in self.poses and not np.allclose(self.poses[ns][1], q, atol=1e-12, rtol=0):
            self.poses.pop(ns); self.blocked.append(ns)
            self.counters['pose_conflict'] += 1
            return
        self.poses[ns] = (min(received, self.clock()), q, frame_id)
        self.poses.move_to_end(ns)
        self.counters['joined'] += 1
        self._prune()

    def attitude(self, camera_timing, imu_frame_id='imu'):
        with self.lock:
            self._prune()
            if camera_timing.bridge_session_id != self.session:
                return None, 'bridge_session_mismatch'
            if not source_usable(camera_timing):
                return None, 'camera_source_invalid'
            target = int(camera_timing.source_sec) * 1000000000 + int(camera_timing.source_nsec)
            if target in self.blocked:
                return None, 'imu_source_conflict'
            samples = sorted((ns, row) for ns, row in self.poses.items()
                             if self.clock() - row[0] <= self.max_age and row[2] == imu_frame_id)
            stamps = [ns for ns, _ in samples]; at = bisect_left(stamps, target)
            if at < len(samples) and samples[at][0] == target:
                return Attitude(samples[at][1][1], target, 0., self.session), 'exact'
            if at == 0 or at == len(samples):
                self.last_bracket_failure = dict(target_ns=target, fresh_samples=len(samples),
                    first_ns=stamps[0] if stamps else None, last_ns=stamps[-1] if stamps else None,
                    latest_receipt_age_ms=1000 * (self.clock() - samples[-1][1][0]) if samples else None)
                return None, 'imu_not_bracketed'
            a, b = samples[at-1:at+1]; span = b[0] - a[0]
            if any(a[0] < ns < b[0] for ns in self.blocked):
                return None, 'imu_conflict_in_bracket'
            if span > self.max_bracket:
                return None, 'imu_bracket_too_wide'
            q = slerp(a[1][1], b[1][1], (target-a[0])/span)
            return Attitude(tuple(q), target, span/1e6, self.session), 'interpolated'

    def snapshot(self):
        with self.lock:
            self._prune()
            return dict(session=self.session, poses=len(self.poses), pending=len(self.pending),
                        counters=dict(self.counters), last_bracket_failure=self.last_bracket_failure)
