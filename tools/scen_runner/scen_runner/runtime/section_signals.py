"""Maintain a green intersection phase independently of object/GUI preparation."""
import copy
import threading
import time

from scen_runner.config import TOOL_ROOT, load_yaml


class SectionSignals:
    def __init__(self, link):
        self.link = link
        self.original = {}  # Intersection phases, captured before our first write.
        self.settings = dict(policy='unchanged', colors={})
        self.stopping = threading.Event()
        self.thread = None
        self.error = None
        self.last_verified = None
        self.interval = 1.

    def begin(self, mode, section, config, replay=None):
        if self.original or (self.thread and self.thread.is_alive()):
            raise RuntimeError('이전 구간의 신호등 고정 해제가 완료되지 않았습니다.')
        # Old/scene replay logs did not force green. Do not silently introduce
        # today's section policy into a saved case with different conditions.
        if replay is not None:
            settings = copy.deepcopy(replay)
        elif mode == 'section' and config.get('enabled', False):
            settings = dict(policy='green', colors={
                aid: config.get('color', 48) for aid in config.get('ids', {}).get(section, [])})
        else:
            settings = dict(policy='unchanged', colors={})
        colors = settings.get('colors', {})
        if settings.get('policy') not in ('green', 'unchanged') or not isinstance(colors, dict):
            raise ValueError('지원하지 않는 구간 신호등 설정입니다.')
        if settings['policy'] == 'unchanged' and colors:
            raise ValueError('변경하지 않는 신호등 설정에 고정 색상이 있습니다.')
        if any(not isinstance(aid, str) or not aid or type(color) is not int or color not in (16, 48)
               for aid, color in colors.items()):
            raise ValueError('구간 신호등은 유효한 ID와 직진(16)/직진·좌회전(48) 초록이어야 합니다.')
        self.settings = dict(policy='unchanged', colors={})
        self.error = None
        self.last_verified = None
        self.stopping.clear()
        if not colors:
            return
        controllers = settings.get('controllers')
        if controllers is None:
            controllers = config.get('controllers', {}).get(section)
        if controllers is None and replay is not None:
            # Early green replay records only stored signal IDs/colors. Resolve
            # their known K-city approach using today's controller mapping.
            defaults = load_yaml(TOOL_ROOT/'config/validation.yaml')['section_traffic_lights']
            if set(colors) <= set(defaults['ids'].get(section, [])):
                controllers = defaults['controllers'].get(section)
        if not isinstance(controllers, dict) or not controllers or any(
                not isinstance(aid, str) or not aid or type(phase) is not int or phase < 0
                for aid, phase in controllers.items()):
            raise ValueError('구간 신호등의 교차로 단계 정보가 없습니다.')
        self.interval = float(config.get('refresh_interval_s', 1.))
        if not .1 <= self.interval <= 2.:
            raise ValueError('구간 신호 갱신 주기는 0.1~2초여야 합니다.')
        settings['controllers'] = dict(controllers)
        # Check every intersection/target before mutating any of them.
        states = {aid: self.link.intersection_state(aid) for aid in controllers}
        covered = set()
        for aid, state in states.items():
            relevant = set(colors).intersection(state['colors'])
            if not relevant:
                raise RuntimeError('대상 신호등이 없는 교차로: '+aid)
            covered.update(relevant)
        if covered != set(colors):
            raise RuntimeError('교차로에 대상 신호등이 없습니다: '+', '.join(sorted(set(colors)-covered)))
        # Track before the RPC: a deadline can arrive after MORAI applied it.
        self.original = {aid: state['phase'] for aid, state in states.items()}
        self._apply(settings)
        self.settings = copy.deepcopy(settings)
        self._start_monitor()
        print(f'[구간 신호등] {section} · 초록 확인 · 교차로 단계 {controllers} · '
              f'{self.interval:g}초마다 유지·조회', flush=True)

    def _apply(self, settings):
        # MORAI's SetIntersectionPhase resets that phase's elapsed time to zero.
        # Reset before the green interval expires; the intersection cannot then
        # overwrite an individually paused light at its next phase transition.
        for aid, phase in settings['controllers'].items():
            self.link.set_intersection_phase(aid, phase)
        for aid, color in settings['colors'].items():
            self.link.set_traffic_light_color(aid, color, impulse=True)
        self.link.verify_traffic_light_colors(settings['colors'])
        self.last_verified = time.monotonic()

    def _start_monitor(self):
        self.thread = threading.Thread(target=self._monitor, name='section-signals', daemon=True)
        self.thread.start()

    def _monitor(self):
        while not self.stopping.wait(self.interval):
            try:
                self._apply(self.settings)
            except Exception as error:
                self.error = str(error)
                print('[구간 신호 유지 실패] '+self.error, flush=True)
                return

    def check(self):
        if self.error:
            raise RuntimeError('구간 신호 유지 실패 · '+self.error)
        if self.settings.get('colors') and (self.last_verified is None or
                time.monotonic()-self.last_verified > 10.):
            raise RuntimeError('구간 신호 유지 확인이 10초 이상 지연됐습니다.')

    def verify(self):
        self.check()
        colors = self.settings['colors']
        if colors:
            self.link.verify_traffic_light_colors(colors)

    def restore(self):
        self.stopping.set()
        # No late refresh is allowed after restoring the original phase or
        # switching sections. Bounded RPCs ensure this wait can terminate.
        if self.thread:
            self.thread.join(timeout=12.)
            if self.thread.is_alive():
                raise RuntimeError('구간 신호 갱신 종료 미확인 · 교차로 복귀 보류')
            self.thread = None
        errors = []
        for aid, phase in list(self.original.items()):
            try:
                self.link.set_intersection_phase(aid, phase)
            except Exception as error:
                errors.append(f'{aid}: {error}')
            else:
                del self.original[aid]
                print(f'[구간 신호등 해제] {aid} · 단계 {phase}에서 자동 신호 주기 재개', flush=True)
        if errors:
            raise RuntimeError('구간 신호등 고정 해제 실패 · '+'; '.join(errors))
        self.settings = dict(policy='unchanged', colors={})
        self.error = None
        self.last_verified = None

    def snapshot(self):
        return copy.deepcopy(self.settings)
