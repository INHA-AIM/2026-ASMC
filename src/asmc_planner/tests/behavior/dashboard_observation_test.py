#!/usr/bin/env python3
"""Pure bounded-observer contracts; no ROS master, controls or simulator."""
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as S
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'scripts'))
from dashboard_observation import LocalScene, family_payload, markers_payload, objects_payload


class SceneTest(unittest.TestCase):
    def setUp(self):
        self.now = 10.
        self.scene = LocalScene(clock=lambda: self.now)
        self.stamp = 1793000000000000000

    def put(self, seq=1, epoch=('producer', 1), stamp=None, **kwargs):
        self.scene.put('objects', {'count': 1}, self.stamp if stamp is None else stamp,
                       'base_link', epoch=epoch, sequence=seq, **kwargs)

    def current(self):
        return self.scene.snapshot(self.stamp)['objects']

    def test_ros_and_receipt_clocks_are_not_subtracted(self):
        self.put()
        self.assertTrue(self.current()['available'])
        self.now += .36  # ROS clock paused, receipt still expires.
        self.assertFalse(self.current()['available'])
        self.assertEqual(self.current()['source_age_s'], 0)

    def test_no_map_to_local_relabelling_and_future_source(self):
        self.scene.put('family', {'lines': [1]}, self.stamp, 'map')
        f = self.scene.snapshot(self.stamp)['family']
        self.assertFalse(f['available']); self.assertEqual(f['payload'], {})
        self.put(stamp=self.stamp+50000001)
        self.assertFalse(self.current()['available'])

    def test_sequence_watermark_survives_rejection(self):
        self.put(seq=10)
        self.put(seq=1)
        self.assertFalse(self.current()['available'])
        self.put(seq=2)
        self.assertFalse(self.current()['available'])
        self.put(seq=11)
        self.assertTrue(self.current()['available'])

    def test_restart_and_retired_generation(self):
        self.put(seq=10)
        self.put(seq=1, epoch=('producer', 2))
        self.assertTrue(self.current()['available'])
        self.put(seq=11, epoch=('producer', 1))
        self.assertEqual(self.current()['reason'], 'retired_epoch')
        self.put(seq=1, epoch=('new', 1))
        self.assertTrue(self.current()['available'])
        self.put(seq=12, epoch=('producer', 2))
        self.assertFalse(self.current()['available'])

    def test_invalid_replaces_good_geometry_and_bounded_latest(self):
        self.put()
        self.put(seq=2, valid=False, reason='imu_invalid')
        self.assertEqual(self.current()['payload'], {})
        for i in range(3, 1003): self.put(seq=i)
        self.assertEqual(len(self.scene.items), 1)
        self.assertEqual(len(self.scene.accepted), 1)

    def test_family_splits_unobserved_gaps(self):
        m = S(boundaries=[S(role=1, polyline=[S(x=i,y=1) for i in range(7)],
                            support_state=[1,1,0,1,1,2,2])],
              valid=True, config_sha256='a', deployment_allowed=False,
              status_reason='valid', valid_x_min_m=0, valid_x_max_m=6)
        p, v, _ = family_payload(m)
        self.assertTrue(v)
        self.assertEqual([x['points'] for x in p['lines']],
                         [[[0.,1.],[1.,1.]], [[3.,1.],[4.,1.]], [[5.,1.],[6.,1.]]])
        self.assertEqual([x['support'] for x in p['lines']], [1,1,2])
        m.boundaries[0].support_state.pop()
        self.assertFalse(family_payload(m)[1])

    def test_delete_all_clears_selected_path(self):
        self.assertFalse(markers_payload(S(markers=[]))[2])

    def test_one_hz_selector_does_not_blink_or_change_command_watchdog(self):
        self.scene.put('selector', {'command_fresh': 'false', 'mode': 'JAMMING'})
        self.now += 1.1
        state = self.scene.snapshot(self.stamp)['selector']
        self.assertTrue(state['available'])
        self.assertEqual(state['payload']['command_fresh'], 'false')
        self.now += 1.5
        self.assertFalse(self.scene.snapshot(self.stamp)['selector']['available'])

    def test_invalid_local_status_not_free_space(self):
        array = S(status=3, OBSERVED=1, OBSERVED_EMPTY=2, objects=[])
        message = S(observation=S(objects=array), tracking=[])
        self.assertFalse(objects_payload(message)[1])
        array.status = 2
        message.observation.producer_session_id='p'; message.observation.generation=1
        message.observation.scan_sequence=1; message.observation.config_id='c'
        message.observation.calibration_id='cal'
        self.assertTrue(objects_payload(message)[1])


if __name__ == '__main__': unittest.main()
