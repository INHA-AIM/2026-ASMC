"""Canonical scene actors/environment around an independently evaluated drive.

The episode worker never drives Ego or starts another planner. Preparation stays
held; the caller authorizes release after its operational inputs are ready.
"""
import threading
import time

from scen_runner.runtime.validation_episode import EpisodeController


class SignalTrialScenario:
    def __init__(self, link, route, seed, lease, emit, section=None):
        self.link, self.route, self.emit = link, route, emit
        self.episode = EpisodeController(link, route,
            mode='section' if section else 'scene', section=section or 'static',
            seed=seed, lease=lease)
        # Every section creates its own connection. Use the endpoint of this
        # trial, not a possibly different optional workstation configuration.
        self.episode.base['morai'].update(host=link.host, port=link.port)
        self.episode.on_lifecycle_observation = emit
        self.episode.on_progress = lambda detail: emit(dict(event='preparation', detail=detail))
        # Section actors are prepared while Ego is held. Repeated creation
        # starts only near their route section, so vehicles meant for the
        # roundabout/highway do not exhaust MORAI's actor slots while Ego is
        # still at the first traffic lights.
        self.episode.adapter_tick_guard = self._allow_section_tick
        self._window_open = {}
        self.thread = None
        self.error = None
        self.weather = self.simulation_time = None

    def _allow_section_tick(self, adapter, ego, now):
        if adapter.name not in ('roundabout','high_speed'):
            return True
        point=ego.transform.location
        station,_=self.route.curve.project(point.x,point.y)
        section=self.route.section(adapter.name)
        allowed=section['start']-150. <= float(station) <= section['end']
        if self._window_open.get(adapter.name)!=allowed:
            self._window_open[adapter.name]=allowed
            self.emit(dict(event='section_actor_window',section=adapter.name,
                           open=allowed,station=float(station),
                           start=section['start'],end=section['end']))
        return allowed

    def prepare(self, clock, deadline_s=240.):
        from proto.morai.common.type_pb2 import Empty
        stub = self.link._require()._environment_stub
        # Capture before begin(), including partial preparation failures.
        self.weather = stub.GetWeather(Empty(), timeout=3)
        self.simulation_time = stub.GetTime(Empty(), timeout=3)
        self.emit(dict(event='scene_environment_snapshot', weather_type=self.weather.type,
                       hour=self.simulation_time.hour))
        self.episode.begin(1)
        deadline = time.monotonic() + deadline_s
        while self.episode.pending:
            clock.check()
            if time.monotonic() > deadline:
                raise RuntimeError('canonical scene preparation deadline; Ego remains held')
            self.episode.step()
            time.sleep(.02)
        self.emit(dict(event='scene_prepared', **self.snapshot()))

    def start(self, clock):
        deadline = time.monotonic() + 15.
        while self.episode.phase == 'preparing':
            clock.check()
            if time.monotonic() > deadline:
                raise RuntimeError('canonical actor readiness deadline; Ego remains held')
            self.episode.step()
            time.sleep(.02)
        if self.episode.phase != 'running':
            raise RuntimeError('canonical scene did not enter running state')
        self.thread = threading.Thread(target=self._run, name='canonical-scene', daemon=True)
        self.thread.start()

    def _run(self):
        try:
            while not self.episode.cancelled.is_set():
                reason = self.episode.step()
                if reason:
                    self.emit(dict(event='scene_stopped', reason=reason, **self.snapshot()))
                    if reason != 'scene_end':
                        raise RuntimeError('canonical scene ended: ' + reason)
                    return
                self.episode.cancelled.wait(.05)
        except BaseException as error:
            self.error = error
            try:
                self.link.hold()
            except Exception:
                pass

    def check(self):
        if self.error is not None:
            raise RuntimeError('canonical scenario worker failed: ' + str(self.error))

    def snapshot(self):
        episode = self.episode
        return dict(seed=episode.seed, phase=episode.phase,
                    environment=episode.environment.snapshot(),
                    actor_replay={a.name: list(a.replay_history) for a in episode.adapters},
                    environment_timeline=list(episode.environment.timeline))

    def stop_worker(self):
        self.episode.cancelled.set()
        if self.thread is not None:
            self.thread.join(timeout=20.)
            if self.thread.is_alive():
                raise RuntimeError('canonical scenario worker did not stop; cleanup withheld')
        self.check()

    def cleanup(self):
        # Do not call EpisodeController.stop(): its GUI semantics release Ego.
        if self.thread is not None and self.thread.is_alive():
            raise RuntimeError('cannot clean actors while their worker is running')
        self.episode.cancelled.set()
        try:
            self.emit(dict(event='scene_final_snapshot', **self.snapshot()))
        finally:
            try:
                self.episode.cleanup()
            finally:
                self._restore_environment()

    def _restore_environment(self):
        if self.weather is None or self.simulation_time is None:
            return
        from proto.morai.common.type_pb2 import Empty
        stub = self.link._require()._environment_stub
        weather = type(self.weather)(); weather.CopyFrom(self.weather); weather.instantly = True
        stamp = type(self.simulation_time)(); stamp.CopyFrom(self.simulation_time)
        stamp.instantly = True; stamp.data_only = False
        self.link.check(stub.SetWeather(weather, timeout=3), 'trial weather restoration failed')
        self.link.check(stub.SetTime(stamp, timeout=3), 'trial time restoration failed')
        deadline = time.monotonic() + 3.
        while True:
            actual_weather = stub.GetWeather(Empty(), timeout=3)
            actual_time = stub.GetTime(Empty(), timeout=3)
            if actual_weather.type == weather.type and actual_time.hour == stamp.hour:
                self.emit(dict(event='scene_environment_restored', weather_type=weather.type, hour=stamp.hour))
                self.weather = self.simulation_time = None
                return
            if time.monotonic() >= deadline:
                raise RuntimeError('trial environment restoration readback mismatch')
            time.sleep(.05)
