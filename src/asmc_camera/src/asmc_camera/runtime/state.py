"""Thread-safe inference lifecycle, independent of ROS and mission policy."""
import math
import threading
import time
from collections import deque
from contextlib import contextmanager

STANDBY, PREPARE, ACTIVE = range(3)


class InferenceState:
    def __init__(self, views, managed=False, initial_mode=STANDBY, target_hz=10.,
                 warmed_up=True, freshness_sec=.35, clock=time.monotonic, trace=None):
        self.views = tuple(views)
        if not self.views or initial_mode not in (STANDBY, PREPARE, ACTIVE):
            raise ValueError('invalid views or initial mode')
        if not math.isfinite(target_hz) or target_hz <= 0 or freshness_sec <= 0:
            raise ValueError('invalid target rate or freshness')
        self.lock = threading.RLock()
        self.clock, self.freshness = clock, freshness_sec
        self.trace = trace
        self.managed = managed
        self.mode = initial_mode if managed else ACTIVE
        self.target_hz, self.warmed_up = target_hz, warmed_up
        self.generation = 0
        self.latest_input = {v: 0 for v in self.views}
        self.latest_output = {v: 0 for v in self.views}
        self.completed_at = {v: -float('inf') for v in self.views}
        self.hits = {v: 0 for v in self.views}
        self.cutoff, self.last_source, self.next_at = 0, 0, 0.
        self.samples = deque(maxlen=200)
        self.successful_batches = self.errors = 0
        self.last_output_valid = False
        self.reason = 'standby' if self.mode == STANDBY else 'awaiting_new_images'

    def observe(self, view, stamp_ns):
        with self.lock:
            if view not in self.latest_input or stamp_ns <= 0:
                return
            # New source epochs invalidate prepared history without inventing a stamp.
            if stamp_ns and stamp_ns < self.latest_input[view]:
                self._reset()
                self.cutoff = 0
                self.latest_input = {v: 0 for v in self.views}
                self.generation += 1
            self.latest_input[view] = stamp_ns
            self.record('input', view=view, source_stamp_ns=stamp_ns)

    def record(self, event, **values):
        if self.trace is not None:
            self.trace(event, generation=self.generation, mode=self.mode, **values)

    def _reset(self):
        self.hits = {v: 0 for v in self.views}
        self.latest_output = {v: 0 for v in self.views}
        self.completed_at = {v: -float('inf') for v in self.views}
        self.samples.clear()
        self.next_at = 0.
        self.last_output_valid = False

    def request(self, mode, target_hz=0.):
        with self.lock:
            if mode not in (STANDBY, PREPARE, ACTIVE):
                return False, self.generation, 'unknown_mode'
            if not math.isfinite(target_hz) or target_hz < 0 or target_hz > 120:
                return False, self.generation, 'target_hz_must_be_zero_or_in_(0,120]'
            target = target_hz or self.target_hz
            if self.managed and mode == self.mode and target == self.target_hz:
                return True, self.generation, 'unchanged'
            self.managed = True
            self.mode, self.target_hz = mode, target
            self.generation += 1
            self.cutoff = max(self.latest_input.values())
            self._reset()
            self.reason = 'standby' if mode == STANDBY else 'awaiting_new_images'
            self.record('request_accepted', target_hz=target, source_cutoff_ns=self.cutoff)
            return True, self.generation, 'accepted_not_ready'

    def begin(self, stamps):
        with self.lock:
            now = self.clock()
            if self.mode == STANDBY or (self.managed and now + .002 < self.next_at):
                if self.mode != STANDBY:
                    self.record('rate_wait', stamps=stamps, deadline=self.next_at)
                return None
            if not stamps or any(v not in self.views or n <= 0 for v, n in stamps.items()):
                return None
            if self.managed and any(n <= self.cutoff or n <= self.latest_output[v]
                                    for v, n in stamps.items()):
                return None
            if self.managed:
                self.next_at = (now + 1. / self.target_hz if not self.next_at
                                else max(self.next_at + 1. / self.target_hz, now + .001))
            self.record('begin', stamps=stamps, deadline=self.next_at)
            return self.generation

    @contextmanager
    def output_guard(self, generation):
        # A service cannot acknowledge a transition midway through this publication.
        with self.lock:
            yield generation == self.generation and self.mode != STANDBY

    def complete(self, generation, stamps, valid=True, error=''):
        with self.lock:
            if generation != self.generation or self.mode == STANDBY:
                return False
            now = self.clock()
            if error:
                self.errors += 1
                self.hits = {v: 0 for v in self.views}
                self.reason = error
                self.last_output_valid = False
                self.record('error', stamps=stamps, reason=error)
                return True
            for view, stamp in stamps.items():
                if stamp > self.latest_output[view]:
                    self.hits[view] += 1
                    self.latest_output[view] = stamp
                    self.completed_at[view] = now
            self.last_source = max(stamps.values())
            self.last_output_valid = bool(valid)
            self.successful_batches += 1
            self.samples.append(now)
            self.reason = 'running' if self.mode == ACTIVE else 'preparing'
            self.record('complete', stamps=stamps, hits=dict(self.hits), valid=bool(valid))
            return True

    def snapshot(self):
        with self.lock:
            now = self.clock()
            prepared = (self.mode != STANDBY and self.warmed_up and
                        all(self.hits[v] >= 3 and now - self.completed_at[v] <= self.freshness
                            for v in self.views))
            samples = [t for t in self.samples if now - t <= 5.]
            hz = ((len(samples) - 1) / (samples[-1] - samples[0])
                  if len(samples) > 1 and samples[-1] > samples[0] else 0.)
            reason = self.reason
            if self.mode != STANDBY and any(now - t > self.freshness
                                           for t in self.completed_at.values()):
                reason = 'input_or_result_stale'
            return dict(mode=self.mode, requested_mode=self.mode, generation=self.generation,
                        managed=self.managed, loaded=True, warmed_up=self.warmed_up,
                        prepared=prepared, last_output_valid=self.last_output_valid,
                        source_cutoff=self.cutoff, last_source_stamp=self.last_source,
                        target_hz=self.target_hz, processing_hz=hz,
                        successful_batches=self.successful_batches, errors=self.errors,
                        reason=reason)
