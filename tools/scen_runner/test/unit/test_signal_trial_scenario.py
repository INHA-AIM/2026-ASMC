import sys
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from scen_runner.runtime.signal_trial_scenario import SignalTrialScenario


class CanonicalScenarioTest(unittest.TestCase):
    def make_scenario(self):
        link = Mock(host='audited-host', port=7789)
        episode = Mock()
        episode.base = {'morai': {'host': 'different-host', 'port': 9999}}
        episode.adapters = []
        episode.seed = 17
        episode.phase = 'running'
        episode.environment.timeline = []
        episode.environment.snapshot.return_value = {}
        with patch('scen_runner.runtime.signal_trial_scenario.EpisodeController', return_value=episode):
            scenario = SignalTrialScenario(link, Mock(), 17, Mock(), Mock())
        return scenario, episode, link

    def test_all_section_connections_use_trial_endpoint_and_no_constructor_release(self):
        scenario, episode, link = self.make_scenario()
        self.assertEqual(episode.base['morai'], {'host': 'audited-host', 'port': 7789})
        self.assertEqual(episode.adapter_tick_guard,scenario._allow_section_tick)
        link.release.assert_not_called()
        episode.step.assert_not_called()

    def test_repeated_section_actors_wait_for_their_route_window(self):
        scenario, _, _ = self.make_scenario()
        scenario.route.section.side_effect=lambda name: {'roundabout':
            dict(start=840.,end=915.),'high_speed':dict(start=985.,end=1860.)}[name]
        ego=Mock();ego.transform.location.x=1.;ego.transform.location.y=2.
        adapter=Mock(name='roundabout');adapter.name='roundabout'
        scenario.route.curve.project.return_value=(300.,0.)
        self.assertFalse(scenario._allow_section_tick(adapter,ego,0.))
        scenario.route.curve.project.return_value=(700.,0.)
        self.assertTrue(scenario._allow_section_tick(adapter,ego,0.))
        scenario.route.curve.project.return_value=(920.,0.)
        self.assertFalse(scenario._allow_section_tick(adapter,ego,0.))
        adapter.name='high_speed'
        self.assertTrue(scenario._allow_section_tick(adapter,ego,0.))

    def test_worker_failure_holds_ego_and_remains_reported_after_join(self):
        scenario, episode, link = self.make_scenario()
        episode.cancelled.is_set.return_value = False
        episode.step.side_effect = RuntimeError('actor update failed')
        scenario._run()
        link.hold.assert_called_once()
        with self.assertRaisesRegex(RuntimeError, 'actor update failed'):
            scenario.stop_worker()
        link.release.assert_not_called()

    def test_cleanup_restores_environment_after_actor_failure_without_gui_stop(self):
        scenario, episode, link = self.make_scenario()
        scenario._restore_environment = Mock()
        episode.cleanup.side_effect = RuntimeError('actor deletion failed')
        with self.assertRaisesRegex(RuntimeError, 'actor deletion failed'):
            scenario.cleanup()
        scenario._restore_environment.assert_called_once()
        episode.stop.assert_not_called()
        link.release.assert_not_called()

    def test_live_worker_prevents_racing_cleanup(self):
        scenario, episode, _ = self.make_scenario()
        scenario.thread = Mock()
        scenario.thread.is_alive.return_value = True
        with self.assertRaisesRegex(RuntimeError, 'worker is running'):
            scenario.cleanup()
        episode.cleanup.assert_not_called()

    def test_snapshot_recording_failure_does_not_skip_owned_actor_cleanup(self):
        scenario, episode, link = self.make_scenario()
        scenario.emit.side_effect = TypeError('unsupported record')
        scenario._restore_environment = Mock()
        with self.assertRaisesRegex(TypeError, 'unsupported record'):
            scenario.cleanup()
        episode.cleanup.assert_called_once()
        scenario._restore_environment.assert_called_once()
        link.release.assert_not_called()


if __name__ == '__main__':
    unittest.main()
