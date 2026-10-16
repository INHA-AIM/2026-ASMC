"""Detect sustained low position change without speed or driving-policy guesses."""
import math
from pathlib import Path

from scen_runner.config import TOOL_ROOT, load_yaml


class StagnationMonitor:
    def __init__(self, settings):
        if not isinstance(settings,dict) or not {'enabled','duration_s','position_radius_m','max_sample_gap_s'}<=settings.keys():
            raise ValueError('정체 YAML에 enabled/duration_s/position_radius_m/max_sample_gap_s가 필요합니다.')
        if type(settings.get('enabled')) is not bool:
            raise ValueError('정체 감시 enabled는 true/false여야 합니다.')
        self.enabled = settings['enabled']
        try:
            self.duration = float(settings['duration_s'])
            self.radius = float(settings['position_radius_m'])
            self.max_gap = float(settings['max_sample_gap_s'])
        except (TypeError,ValueError) as error:raise ValueError('정체 시간·위치 범위·관측 공백은 숫자여야 합니다.') from error
        if not all(math.isfinite(v) and v > 0 for v in (self.duration, self.radius, self.max_gap)):
            raise ValueError('정체 시간·위치 범위·관측 공백은 유한한 양수여야 합니다.')
        self.reset()

    @classmethod
    def load(cls, path=None):
        return cls(load_yaml(Path(path) if path is not None else TOOL_ROOT/'config/stagnation.yaml'))

    def reset(self, xy=None, now=None):
        self.anchor = tuple(xy) if xy is not None else None
        self.started = now if xy is not None else None
        self.last_at = now if xy is not None else None
        self.maximum = 0.
        self.maximum_gap = 0.
        self.samples = 0
        self.elapsed = 0.
        self.detected = False

    def update(self, state, now):
        if not self.enabled or self.detected:
            return None
        p = state.transform.location
        xy = (p.x, p.y)
        if not all(math.isfinite(value) for value in (*xy, now)):
            self.reset()
            return None
        gap = None if self.last_at is None else now-self.last_at
        if self.anchor is None or gap is None or gap <= 0 or gap > self.max_gap:
            self.reset(xy, now)
            gap = 0.
        distance = math.dist(self.anchor, xy)
        if distance > self.radius:
            self.reset(xy, now)
            distance = 0.
            gap = 0.
        self.last_at = now
        self.samples += 1
        self.maximum = max(self.maximum, distance)
        self.maximum_gap = max(self.maximum_gap, gap or 0.)
        self.elapsed = now-self.started
        if self.elapsed < self.duration:
            return None
        self.detected = True
        return dict(duration_s=self.elapsed, threshold_s=self.duration, position_radius_m=self.radius,
                    maximum_displacement_m=self.maximum, anchor_xy=list(self.anchor),
                    last_xy=list(xy), window_started_at=self.started, detected_at=now,
                    sample_count=self.samples, maximum_sample_gap_s=self.maximum_gap)

    def snapshot(self):
        return dict(enabled=self.enabled, elapsed_s=self.elapsed, threshold_s=self.duration,
                    position_radius_m=self.radius, maximum_displacement_m=self.maximum,
                    detected=self.detected)
