"""Finite driving batches. Count only successful ego driving releases."""
from pathlib import Path
import yaml

from scen_runner.config import TOOL_ROOT, load_yaml


DEFAULT_COUNTS = TOOL_ROOT / 'config/run_counts.yaml'
SECTIONS = ('static', 'dynamic', 'roundabout', 'high_speed', 'tunnel')


class RunBudget:
    def __init__(self, settings, mode='scene', section='static', path=None):
        if not isinstance(settings, dict) or set(settings) != {'scene_runs', 'section_runs'}:
            raise ValueError('주행 횟수 YAML에는 scene_runs와 section_runs가 필요합니다.')
        sections = settings['section_runs']
        if not isinstance(sections, dict) or set(sections) != set(SECTIONS):
            raise ValueError('section_runs에는 static/dynamic/roundabout/high_speed/tunnel 횟수가 필요합니다.')
        for key, value in [('scene_runs', settings['scene_runs'])] + list(sections.items()):
            if type(value) is not int or value < 1:
                raise ValueError(f'주행 횟수 {key}는 최초 주행을 포함한 1 이상의 정수여야 합니다.')
        self.settings = dict(scene_runs=settings['scene_runs'], section_runs=dict(sections))
        self.path = str(path) if path is not None else None
        self.reset(mode, section)

    @classmethod
    def load(cls, mode='scene', section='static', path=None):
        path = Path(path) if path is not None else DEFAULT_COUNTS
        try:
            return cls(load_yaml(path), mode, section, path)
        except (ValueError, yaml.YAMLError) as error:
            raise ValueError(f'{path}: {error}') from error

    def reset(self, mode, section):
        if mode not in ('scene', 'section') or section not in SECTIONS:
            raise ValueError('지원하지 않는 주행 횟수 모드/구간입니다.')
        self.mode, self.section = mode, section
        self.limit = self.settings['scene_runs'] if mode == 'scene' else self.settings['section_runs'][section]
        self.started = 0

    @property
    def can_start(self):
        return self.started < self.limit

    def start_run(self):
        if not self.can_start:
            raise RuntimeError('설정한 총 주행 횟수를 모두 사용했습니다.')
        self.started += 1

    def snapshot(self):
        return dict(started=self.started, limit=self.limit, remaining=self.limit-self.started, path=self.path)
