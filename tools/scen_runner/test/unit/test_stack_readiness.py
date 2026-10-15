import math
from pathlib import Path
import sys
from types import SimpleNamespace as S
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from scen_runner.orchestration.control import settings
from scen_runner.orchestration.readiness import Readiness


class ReadinessTest(unittest.TestCase):
    def setUp(self):
        self.probe = Readiness(settings())
        self.pose = dict(x=10., y=20., yaw=90.)
        self.probe.reset(self.pose, 100.)

    def sample(self, now, source=None, failure=0, x=10., speed=0., yaw=math.pi/2):
        source = now if source is None else source
        header = S(stamp=S(to_sec=lambda: source), frame_id='map')
        state = S(header=header, x=x, y=20., yaw=yaw, speed=speed, valid=True)
        rows = dict(state=state, objects=S(header=header, objects=[]),
                    trajectory=S(header=header, valid=True, failure_reason=failure,
                                 ego_at_plan=state, points=[S(x=x)]), control=S())
        for name, message in rows.items():
            self.probe.observe(name, message, now)
        return rows

    def test_fresh_consistent_inputs_must_remain_stable_before_release(self):
        for now in (100., 100.2, 100.4):
            self.sample(now)
            self.assertFalse(self.probe.evaluate(now, now))
        self.sample(100.6)
        self.assertTrue(self.probe.evaluate(100.6, 100.6))

    def test_old_scene_input_is_rejected_even_if_received_now(self):
        self.sample(101., source=99.99)
        self.assertFalse(self.probe.evaluate(101., 101.))
        self.assertIn('이전 회차', self.probe.detail)

    def test_missing_objects_are_distinct_from_valid_empty_observation(self):
        self.sample(101.)
        self.probe.messages.pop('objects')
        self.assertFalse(self.probe.evaluate(101., 101.))
        self.assertEqual(self.probe.detail, 'objects 미수신')

    def test_position_heading_speed_and_nonfinite_state_are_rejected(self):
        for change in (dict(x=100.), dict(yaw=0.), dict(speed=2.), dict(x=float('nan'))):
            with self.subTest(change=change):
                self.sample(101., **change)
                self.assertFalse(self.probe.evaluate(101., 101.))
                self.assertIn('시작 위치', self.probe.detail)

    def test_planner_failure_stop_does_not_qualify_as_ready(self):
        self.sample(101., failure=3)
        self.assertFalse(self.probe.evaluate(101., 101.))
        self.assertIn('failure_reason=3', self.probe.detail)

    def test_old_trajectory_origin_is_rejected_with_fresh_trajectory_header(self):
        rows = self.sample(101.)
        rows['trajectory'].ego_at_plan.header = S(stamp=S(to_sec=lambda: 99.), frame_id='map')
        # Use a separate valid fresh state, rather than sharing the origin.
        self.probe.observe('state', S(header=rows['objects'].header, valid=True,
            x=10., y=20., yaw=math.pi/2, speed=0.), 101.)
        self.assertFalse(self.probe.evaluate(101., 101.))

    def test_staleness_or_reset_clears_previously_stable_readiness(self):
        self.sample(101.); self.probe.evaluate(101., 101.)
        self.sample(102.); self.assertTrue(self.probe.evaluate(102., 102.))
        self.assertFalse(self.probe.evaluate(103., 103.))
        self.assertIsNone(self.probe.stable_since)
        self.probe.reset(self.pose, 200.)
        self.assertFalse(self.probe.messages)


if __name__ == '__main__':
    unittest.main()
