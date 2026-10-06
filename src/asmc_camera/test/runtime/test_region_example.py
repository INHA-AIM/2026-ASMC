import unittest
from types import SimpleNamespace
from asmc_camera.runtime.region_example import RegionExample


class Stamp:
    def __init__(self, t): self.t = t
    def to_sec(self): return self.t
    def to_nsec(self): return int(self.t * 1e9)


class RegionExampleTest(unittest.TestCase):
    def setUp(self):
        self.policy = RegionExample(dict(context_freshness_sec=.5, prepare_lead_sec=3.,
            prepare_min_distance_m=20., prepare_margin_m=5., recovery_hold_sec=.5,
            recovery_exit_margin_m=5., gps_loss_modes=dict(twinlite=2, yolo=2, resnet18=0),
            models=dict(twinlite=dict(zone_ids=['shadow']), yolo=dict(always_active=True),
                        resnet18=dict(zone_ids=['signal']))))
        self.zones = dict(shadow=(100., 200.), signal=(300., 400.))

    def context(self, station, now=1., valid=True):
        return SimpleNamespace(pose_valid=valid, lane_valid=valid, route_valid=valid,
            pose_stamp=Stamp(now), header=SimpleNamespace(stamp=Stamp(now)), route_station_m=station)

    def test_before_prepare_inside_and_other_model_standby(self):
        for station, mode in [(50, 0), (85, 1), (150, 2)]:
            result, _ = self.policy.decide(self.context(station), self.zones, 0, 1., 1.)
            self.assertEqual(mode, result['twinlite'])
            self.assertEqual(2, result['yolo'])
            self.assertEqual(0, result['resnet18'])

    def test_loss_preserves_lane_and_recovery_requires_new_pose_and_exit(self):
        self.policy.decide(self.context(150), self.zones, 0, 1., 1.)
        modes, _ = self.policy.decide(self.context(150, valid=False), self.zones, 0, 2., 2.)
        self.assertEqual(2, modes['twinlite'])
        for now, station, expected in [(3., 202., 2), (3.3, 204., 2), (3.6, 206., 0)]:
            modes, _ = self.policy.decide(self.context(station, now), self.zones, 0, now, now)
            self.assertEqual(expected, modes['twinlite'])

    def test_repeated_health_header_does_not_finish_recovery(self):
        self.policy.decide(None, self.zones, 0, 1., 1.)
        context = self.context(250, 2.)
        for now in [2., 2.2, 2.4]:
            modes, _ = self.policy.decide(context, self.zones, 0, now, now)
            self.assertEqual(2, modes['twinlite'])

    def test_policy_configuration_changes_region_and_prepare_distance(self):
        self.policy.config['prepare_min_distance_m'] = 40.
        modes, _ = self.policy.decide(self.context(65), self.zones, 0, 1., 1.)
        self.assertEqual(1, modes['twinlite'])
        self.policy.config['models']['twinlite']['zone_ids'] = ['signal']
        modes, _ = self.policy.decide(self.context(150), self.zones, 0, 1., 1.)
        self.assertEqual(0, modes['twinlite'])

    def test_unknown_context_does_not_invent_zone_or_signal(self):
        modes, reason = self.policy.decide(None, self.zones, 0, 1., 1.)
        self.assertEqual(2, modes['twinlite'])
        self.assertEqual(0, modes['resnet18'])
        self.assertIn('no_zone_inferred', reason)


if __name__ == '__main__':
    unittest.main()
