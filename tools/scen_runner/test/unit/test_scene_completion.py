"""CP14 qualification, finish crossings, teleports and real closed-route replay."""
import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from scen_runner.runtime.scene_completion import SceneCompletion
from scen_runner.runtime.checkpoint_environment import CheckpointEnvironment


def ego(x, y=0., speed=20.):
    return S(transform=S(location=S(x=x, y=y)), velocity=S(x=speed, y=0., z=0.))


class SceneCompletionTest(unittest.TestCase):
    def tracker(self):
        # Deliberately supply CP14 only. CP1..13 and their order are irrelevant.
        tracker = SceneCompletion([dict(id='14', xy=(30., 0.), radius=3.)],
                                  dict(id='START_END', xy=(0., 0.), radius=3.))
        tracker.reset((0., 0.), 0.)
        return tracker

    def test_start_and_unqualified_return_do_not_complete(self):
        tracker = self.tracker()
        for time in range(1, 10):
            self.assertFalse(tracker.update(ego(0., speed=0.), time))
        self.assertFalse(tracker.update(ego(6.), 11.))
        self.assertFalse(tracker.update(ego(0.), 13.))
        self.assertFalse(tracker.checkpoint14_passed)

    def test_cp14_only_then_finish_emits_once(self):
        tracker = self.tracker()
        self.assertFalse(tracker.update(ego(30.), 10.))
        self.assertTrue(tracker.checkpoint14_passed)
        self.assertFalse(tracker.completed)
        self.assertTrue(tracker.update(ego(0.), 20.))
        self.assertTrue(tracker.completed)
        self.assertFalse(tracker.update(ego(0.), 21.))

    def test_fast_pass_detected_between_samples_outside_both_circles(self):
        tracker = self.tracker()
        self.assertFalse(tracker.update(ego(25., speed=80.), 1.))
        self.assertFalse(tracker.update(ego(35., speed=80.), 1.5))
        self.assertTrue(tracker.checkpoint14_passed)
        self.assertFalse(tracker.update(ego(5., speed=80.), 3.))
        self.assertTrue(tracker.update(ego(-5., speed=80.), 3.5))

    def test_teleport_to_cp14_and_remaining_inside_cannot_qualify(self):
        tracker = self.tracker()
        self.assertFalse(tracker.update(ego(30., speed=0.), .2))
        for time in (1., 2., 3.):
            self.assertFalse(tracker.update(ego(30., speed=0.), time))
        self.assertFalse(tracker.checkpoint14_passed)
        tracker.update(ego(35.), 4.)
        tracker.update(ego(30.), 5.)
        self.assertTrue(tracker.checkpoint14_passed)

    def test_teleport_after_cp14_revokes_qualification_even_at_finish(self):
        tracker = self.tracker()
        tracker.update(ego(30.), 10.)
        self.assertTrue(tracker.checkpoint14_passed)
        self.assertFalse(tracker.update(ego(0., speed=0.), 10.2))
        self.assertFalse(tracker.checkpoint14_passed)
        for time in (11., 12., 13.):
            self.assertFalse(tracker.update(ego(0., speed=0.), time))

    def test_first_sample_inside_cp14_is_not_a_pass(self):
        tracker = self.tracker()
        tracker.reset()
        tracker.update(ego(30.), 0.)
        tracker.update(ego(30.), 1.)
        self.assertFalse(tracker.checkpoint14_passed)

    def test_new_episode_clears_finished_and_qualification(self):
        tracker = self.tracker()
        tracker.update(ego(30.), 10.)
        self.assertTrue(tracker.update(ego(0.), 20.))
        tracker.reset((0., 0.), 21.)
        self.assertEqual(tracker.snapshot(), dict(checkpoint14_passed=False, completed=False))
        self.assertFalse(tracker.update(ego(0.), 22.))

    def test_invalid_sample_cannot_bridge_a_checkpoint_or_finish(self):
        tracker = self.tracker()
        tracker.update(ego(30.), 10.)
        self.assertFalse(tracker.update(ego(math.nan), 11.))
        self.assertFalse(tracker.checkpoint14_passed)
        self.assertFalse(tracker.update(ego(0.), 12.))

    def test_required_positions_are_validated(self):
        with self.assertRaises(ValueError):
            SceneCompletion([], dict(xy=(0., 0.), radius=3.))
        with self.assertRaises(ValueError):
            SceneCompletion([dict(id='14', xy=(30., 0.), radius=0.)], dict(xy=(0., 0.), radius=3.))

    def test_current_competition_route_finishes_with_cp14_only_at_80kmh(self):
        from scen_runner.runtime.validation_episode import EpisodeRoute
        route = EpisodeRoute()
        environment = CheckpointEnvironment(Mock())
        tracker = SceneCompletion(environment.checkpoints, environment.start_end)
        tracker.reset((route.scene_start['x'], route.scene_start['y']), 0.)
        completed_at = []
        qualification_at = None
        station, now = 0., .2
        while station < route.curve.length:
            x, y, _, _ = route.curve.pose(station)
            if tracker.update(ego(x, y, 80.), now):
                completed_at.append(station)
            if qualification_at is None and tracker.checkpoint14_passed:
                qualification_at = station
            station += 80./3.6*.2
            now += .2
        self.assertIsNotNone(qualification_at)
        self.assertGreater(qualification_at, 1900.)
        self.assertEqual(len(completed_at), 1)
        self.assertGreater(completed_at[0], route.curve.length-5.)


if __name__ == '__main__':
    unittest.main()
