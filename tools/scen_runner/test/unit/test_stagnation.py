"""Continuous low displacement, interrupted observation and movement excursions."""
from pathlib import Path
import sys
from types import SimpleNamespace as S
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from scen_runner.runtime.stagnation import StagnationMonitor


def ego(x=0.,y=0.):return S(transform=S(location=S(x=x,y=y)))


class StagnationTest(unittest.TestCase):
    def monitor(self):
        m=StagnationMonitor(dict(enabled=True,duration_s=30.,position_radius_m=1.,max_sample_gap_s=5.))
        m.reset((0.,0.),0.)
        return m

    def test_stationary_triggers_at_thirty_seconds_once(self):
        m=self.monitor()
        for t in range(1,30):self.assertIsNone(m.update(ego(),float(t)))
        event=m.update(ego(.2,.1),30.)
        self.assertEqual(event['duration_s'],30.)
        self.assertLessEqual(event['maximum_displacement_m'],1.)
        self.assertIsNone(m.update(ego(),31.))

    def test_one_metre_jitter_does_not_reset_window(self):
        m=self.monitor()
        for t in range(1,30):m.update(ego(.8 if t%2 else -.8),float(t))
        self.assertIsNotNone(m.update(ego(.8),30.))

    def test_movement_or_excursion_and_return_starts_new_window(self):
        m=self.monitor()
        for t in range(1,20):m.update(ego(),float(t))
        m.update(ego(2.),20.);m.update(ego(),21.)
        for t in range(22,51):self.assertIsNone(m.update(ego(),float(t)))
        self.assertIsNotNone(m.update(ego(),51.))

    def test_query_gap_is_not_counted_as_observed_standstill(self):
        m=self.monitor()
        for t in range(1,20):m.update(ego(),float(t))
        self.assertIsNone(m.update(ego(),30.))
        self.assertEqual(m.elapsed,0.)
        for t in range(31,60):self.assertIsNone(m.update(ego(),float(t)))
        event=m.update(ego(),60.)
        self.assertIsNotNone(event)
        self.assertLessEqual(event['maximum_sample_gap_s'],m.max_gap)

    def test_invalid_position_and_new_episode_clear_detection(self):
        m=self.monitor();m.update(ego(float('nan')),1.)
        self.assertIsNone(m.update(ego(),31.))
        m.reset((0.,0.),32.)
        self.assertFalse(m.detected);self.assertEqual(m.elapsed,0.)

    def test_disabled_monitor_and_bad_config(self):
        m=self.monitor();m.enabled=False
        self.assertIsNone(m.update(ego(),100.))
        for cfg in ({},dict(enabled=True,duration_s=0.,position_radius_m=1.,max_sample_gap_s=5.)):
            with self.assertRaises(ValueError):StagnationMonitor(cfg)


if __name__=='__main__':unittest.main()
