"""One writer for controlled signal trials; simulator truth is evaluation only.

Holding a green phase is suspended for a forced yellow/red interval. Otherwise
the old green refresher would silently overwrite the aspect under test.
"""
from __future__ import annotations

import time

RED, YELLOW, STRAIGHT_GREEN, LEFT_GREEN = 1, 4, 16, 33


class SignalControl:
    def __init__(self, link, emit=lambda event: None, clock=time.monotonic):
        self.link, self.emit, self.clock = link, emit, clock
        self.original = {}
        self.active = {}
        self.tick_cursor = 0

    def begin(self, light_id, intersection_id, green_phase, green_color, phase_duration):
        if light_id in self.active:
            raise RuntimeError('signal already owned by this trial: '+light_id)
        if green_color not in (16, 32, 33, 48) or phase_duration <= 10:
            raise ValueError('reviewed green aspect and phase longer than 10s required')
        state = self.link.intersection_state(intersection_id)
        if light_id not in state['colors']:
            raise ValueError('signal does not belong to the reviewed intersection')
        self.original.setdefault(intersection_id, dict(state))
        self.emit(dict(event='signal_phase_snapshot',intersection_id=intersection_id,
            phase=state['phase'],colors=state['colors']))
        self.active[light_id] = dict(intersection=intersection_id, phase=green_phase,
            green=green_color, duration=phase_duration, color=green_color, phase_at=None)
        self._green(light_id)

    def _green(self, aid):
        state = self.active[aid]
        self._phase(state['intersection'],state['phase'])
        state['phase_at'] = self.clock()
        self.set_aspect(aid, state['green'])

    def _phase(self, intersection, phase):
        self.link.set_intersection_phase(intersection,phase)
        deadline=time.monotonic()+5.
        while True:
            actual=self.link.intersection_state(intersection)
            if actual['phase']==phase:
                return actual
            if time.monotonic()>=deadline:
                raise RuntimeError(f'intersection phase readback differs: {intersection}, requested={phase}, actual={actual["phase"]}')
            time.sleep(.05)

    def set_aspect(self, aid, color):
        if aid not in self.active or color not in (1, 4, 16, 32, 33, 48):
            raise ValueError('unowned signal or unsupported trial aspect')
        state = self.active[aid]
        # An asynchronous RPC acknowledgement is not the rendered transition
        # time. Keep request/readback bounds; never fabricate an exact instant.
        started = self.clock()
        self.link.set_traffic_light_color(aid, color, impulse=False)
        self.link.verify_traffic_light_colors({aid:color})
        state['color'] = color
        self.emit(dict(event='signal_set_verified', light_id=aid, color=color,
            requested_monotonic=started, verified_monotonic=self.clock()))

    def tick(self):
        now = self.clock()
        # Check every deadline, but perform at most ONE signal's RPC work.
        # Four synchronous phase refreshes delayed pose sampling and the
        # requested yellow transition by >1m at approach speed.
        for state in self.active.values():
            if now-state['phase_at'] >= state['duration']-.5:
                raise RuntimeError('forced aspect would outlive its controlled intersection phase')
        if not self.active:return
        lights=list(self.active);aid=lights[self.tick_cursor%len(lights)]
        self.tick_cursor=(self.tick_cursor+1)%len(lights);state=self.active[aid]
        if state['color'] == state['green'] and now-state['phase_at'] >= 1.:
            self._green(aid)
        else:
            actual = self.link.traffic_light_color(aid)
            if actual != state['color']:
                raise RuntimeError(f'trial aspect overwritten: {aid}, requested={state["color"]}, actual={actual}')

    def green(self, aid):
        self._green(aid)

    def restore(self):
        errors = []
        for intersection, original in list(self.original.items()):
            try:
                actual = self._phase(intersection,original['phase'])
                self.emit(dict(event='signal_phase_restored', intersection_id=intersection,
                    phase=actual['phase'], colors=actual['colors']))
            except Exception as error:
                errors.append(f'{intersection}: {error}')
            else:
                del self.original[intersection]
        if errors:
            raise RuntimeError('signal restoration incomplete: '+'; '.join(errors))
        self.active.clear()
