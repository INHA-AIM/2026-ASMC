import sys
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from scen_runner.config import load_config, load_yaml
from scen_runner.morai.episode_link import SceneLink
from scen_runner.runtime.section_signals import SectionSignals

CONFIG = dict(enabled=True, color=48, ids=dict(static=['A'], dynamic=['B', 'C'], roundabout=[]),
              controllers=dict(static={'J1': 0}, dynamic={'J2': 2}))


class SignalsTest(unittest.TestCase):
    def setUp(self):
        self.link = Mock()
        self.link.intersection_state.side_effect = lambda aid: dict(
            J1=dict(phase=1, colors=dict(A=1)), J2=dict(phase=3, colors=dict(B=1, C=1)))[aid]
        self.signals = SectionSignals(self.link)
        self.signals._start_monitor = Mock()  # Most cases drive native updates deterministically.
        self.addCleanup(self.cleanup_signals)

    def cleanup_signals(self):
        self.link.set_intersection_phase.side_effect = None
        self.signals.restore()

    def test_scene_disabled_and_uncontrolled_section_send_no_writes(self):
        for mode, section, config in [('scene', 'static', CONFIG),
                                      ('section', 'static', dict(CONFIG, enabled=False)),
                                      ('section', 'roundabout', CONFIG)]:
            self.signals.begin(mode, section, config)
            self.signals.restore()
        self.link.intersection_state.assert_not_called()
        self.link.set_intersection_phase.assert_not_called()
        self.link.set_traffic_light_color.assert_not_called()

    def test_fixed_green_controls_only_selected_intersection_and_restores_its_phase(self):
        self.signals.begin('section', 'dynamic', CONFIG)
        self.assertEqual(self.signals.snapshot(), dict(policy='green', colors=dict(B=48, C=48), controllers={'J2': 2}))
        self.link.set_intersection_phase.assert_called_once_with('J2', 2)
        self.link.set_traffic_light_color.assert_any_call('B', 48, impulse=True)
        self.link.set_traffic_light_color.assert_any_call('C', 48, impulse=True)
        self.signals.restore()
        self.link.set_intersection_phase.assert_called_with('J2', 3)
        calls = self.link.set_intersection_phase.call_count
        self.signals.restore()
        self.assertEqual(self.link.set_intersection_phase.call_count, calls)

    def test_missing_target_prevents_writes_and_uncertain_phase_write_is_owned(self):
        self.link.intersection_state.side_effect = None
        self.link.intersection_state.return_value = dict(phase=3, colors=dict(B=1))
        with self.assertRaisesRegex(RuntimeError, '대상 신호등'):
            self.signals.begin('section', 'dynamic', CONFIG)
        self.link.set_intersection_phase.assert_not_called()
        self.link.intersection_state.return_value = dict(phase=3, colors=dict(B=1, C=1))
        self.link.set_intersection_phase.side_effect = TimeoutError('RPC timeout')
        with self.assertRaises(TimeoutError):
            self.signals.begin('section', 'dynamic', CONFIG)
        self.assertEqual(self.signals.original, dict(J2=3))
        self.link.set_intersection_phase.side_effect = None
        self.signals.restore()
        self.link.set_intersection_phase.assert_called_with('J2', 3)

    def test_failed_restore_retains_unconfirmed_controller_for_retry(self):
        self.signals.begin('section', 'dynamic', CONFIG)
        self.link.set_intersection_phase.side_effect = RuntimeError('offline')
        with self.assertRaisesRegex(RuntimeError, 'J2: offline'):
            self.signals.restore()
        with self.assertRaisesRegex(RuntimeError, '해제'):
            self.signals.begin('scene', 'static', CONFIG)
        self.link.set_intersection_phase.side_effect = None
        self.signals.restore()
        self.assertFalse(self.signals.original)

    def test_replay_uses_saved_policy_and_resolves_early_green_records(self):
        self.signals.begin('section', 'dynamic', CONFIG, replay=dict(policy='unchanged', colors={}))
        self.link.set_intersection_phase.assert_not_called()
        saved = dict(policy='green', colors=dict(A=16), controllers={'J1': 0})
        self.signals.begin('section', 'dynamic', CONFIG, replay=saved)
        self.link.set_intersection_phase.assert_called_once_with('J1', 0)
        self.assertEqual(self.signals.snapshot(), saved)
        self.signals.restore()
        self.link.intersection_state.side_effect = lambda _: dict(phase=1, colors={'C1256W000091': 1})
        self.signals.begin('section', 'static', {}, replay=dict(policy='green', colors={'C1256W000091': 48}))
        self.assertEqual(self.signals.snapshot()['controllers'], {'IntTL7': 0})

    def test_real_phase_transition_model_overrides_single_light_but_phase_refresh_keeps_green(self):
        state = dict(elapsed=0., color=1, phase=3)
        def advance(seconds):
            state['elapsed'] += seconds
            if state['elapsed'] >= 27.:
                state.update(phase=(state['phase']+1)%4, color=1, elapsed=0.)
        def set_phase(aid, phase):
            state.update(phase=phase, color=48 if phase == 2 else 1, elapsed=0.)
        self.link.set_intersection_phase.side_effect = set_phase
        self.link.set_traffic_light_color.side_effect = lambda aid, color, impulse: state.update(color=color)
        self.link.verify_traffic_light_colors.side_effect = lambda colors: self.assertEqual(state['color'], 48)
        # A successful one-time color change alone does not survive the controller.
        self.link.set_traffic_light_color('B', 48, impulse=False)
        advance(28.)
        self.assertEqual(state['color'], 1)
        self.signals.begin('section', 'dynamic', CONFIG)
        for _ in range(150):
            advance(1.)
            self.assertEqual(state['color'], 48)
            self.signals._apply(self.signals.settings)
        self.assertEqual(state['phase'], 2)

    def test_monitor_runs_without_episode_steps_and_reports_failure(self):
        # Object preparation/GUI/MPC waiting is deliberately not stepped here.
        self.signals._start_monitor = lambda: SectionSignals._start_monitor(self.signals)
        self.signals.begin('section', 'dynamic', dict(CONFIG, refresh_interval_s=.1))
        failed = threading.Event()
        def fail(_):
            failed.set()
            raise RuntimeError('native readback red')
        self.link.verify_traffic_light_colors.side_effect = fail
        self.assertTrue(failed.wait(2))
        self.signals.thread.join(2)
        with self.assertRaisesRegex(RuntimeError, 'native readback red'):
            self.signals.check()

    def test_stop_waits_for_inflight_refresh_before_restoring_no_late_writer(self):
        self.signals._start_monitor = lambda: SectionSignals._start_monitor(self.signals)
        self.signals.begin('section', 'dynamic', dict(CONFIG, refresh_interval_s=.1))
        entered, release = threading.Event(), threading.Event()
        writes = []
        def write(aid, phase):
            if phase == 2:
                entered.set()
                if not release.wait(3):raise RuntimeError('test did not release writer')
            writes.append(phase)
        self.link.set_intersection_phase.side_effect = write
        self.assertTrue(entered.wait(2))
        stopped = threading.Thread(target=self.signals.restore)
        stopped.start()
        self.assertTrue(self.signals.stopping.wait(1))
        self.assertNotIn(3, writes)
        release.set();stopped.join(2)
        self.assertFalse(stopped.is_alive())
        self.assertEqual(writes, [2, 3])
        self.assertIsNone(self.signals.thread)

    def test_selected_targets_follow_section_stoplines(self):
        import json
        from scen_runner.config import repo_path
        from scen_runner.runtime.validation_episode import EpisodeRoute
        route = EpisodeRoute()
        zones = load_yaml(repo_path(load_config()['map']['competition_metadata']))['mission_zones']
        ids = route.cfg['section_traffic_lights']['ids']
        for section in route.cfg['sections']:
            bounds = route.section(section)
            lights = [json.loads(z['rule_payload_json'])['traffic_light_id'] for z in zones
                      if z['id'].startswith('traffic_light_C') and
                      bounds['start'] <= z['route_station_start_m'] <= bounds['end']]
            for aid in lights:self.assertIn(aid, ids[section])
            if not lights:self.assertEqual(ids[section], [])


class SignalRpcTest(unittest.TestCase):
    def setUp(self):
        self.link = SceneLink(load_config())
        from proto.morai.infrastructure.traffic_light_pb2 import TrafficLightInfo
        from proto.morai.infrastructure.intersection_pb2 import IntersectionState
        self.response = TrafficLightInfo(color=1);self.response.id.value = 'A'
        self.intersection = IntersectionState(phase=2);self.intersection.traffic_lights.add().CopyFrom(self.response)
        self.stub = Mock();self.link._sa = S(_infrastructure_stub=self.stub)
        self.stub.SetTrafficLightState.return_value = S(status=1)
        self.stub.SetIntersectionPhase.return_value = S(status=1)
        self.stub.GetTrafficLightInfo.return_value = self.response
        self.stub.GetIntersectionTLInfo.return_value = self.intersection

    def test_selected_intersection_request_and_colors_use_bounded_rpcs(self):
        self.assertEqual(self.link.intersection_state('J'), dict(phase=2, colors={'A': 1}))
        self.link.set_intersection_phase('J', 2)
        param = self.stub.SetIntersectionPhase.call_args.args[0]
        self.assertEqual((param.id.value, param.phase), ('J', 2))
        self.assertEqual(self.stub.SetIntersectionPhase.call_args.kwargs['timeout'], 3)
        self.assertEqual(self.link.traffic_light_color('A'), 1)
        self.link.set_traffic_light_color('A', 48, impulse=True)
        param = self.stub.SetTrafficLightState.call_args.args[0]
        self.assertEqual(param.info.id.value, 'A');self.assertTrue(param.is_impulse)
        self.assertFalse(param.set_sibling)

    def test_ack_without_matching_readback_does_not_pass(self):
        with patch('scen_runner.morai.episode_link.time.sleep'):
            with self.assertRaisesRegex(RuntimeError, '요청=48, 실제=1'):
                self.link.verify_traffic_light_colors(dict(A=48))
        self.assertEqual(self.stub.GetTrafficLightInfo.call_count, 6)
        self.response.id.value = ''
        with self.assertRaisesRegex(RuntimeError, '미확인'):self.link.traffic_light_color('A')
        self.stub.SetIntersectionPhase.return_value = S(status=0, description='rejected')
        with self.assertRaisesRegex(RuntimeError, 'rejected'):self.link.set_intersection_phase('J', 2)

    def test_only_known_transient_read_handler_failure_is_retried_with_fresh_response(self):
        import grpc
        class HandlerError(grpc.RpcError):
            def code(self):return grpc.StatusCode.UNKNOWN
            def details(self):return 'Exception was thrown by handler.'
        self.stub.GetTrafficLightInfo.side_effect = [HandlerError(), self.response]
        with patch('scen_runner.morai.episode_link.time.sleep'):
            self.assertEqual(self.link.traffic_light_color('A'), 1)
        self.assertEqual(self.stub.GetTrafficLightInfo.call_count, 2)
        self.stub.GetTrafficLightInfo.reset_mock()
        self.stub.GetTrafficLightInfo.side_effect = HandlerError()
        with patch('scen_runner.morai.episode_link.time.sleep'):
            with self.assertRaises(HandlerError):self.link.traffic_light_color('A')
        self.assertEqual(self.stub.GetTrafficLightInfo.call_count, 3)
        self.stub.GetTrafficLightInfo.reset_mock()
        self.stub.GetTrafficLightInfo.side_effect = TimeoutError('offline')
        with self.assertRaises(TimeoutError):self.link.traffic_light_color('A')
        self.assertEqual(self.stub.GetTrafficLightInfo.call_count, 1)


if __name__ == '__main__':unittest.main()
