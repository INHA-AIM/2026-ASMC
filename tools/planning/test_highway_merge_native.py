#!/usr/bin/env python3
"""Negative geometry/config checks against the actual native planner."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
import yaml

ROOT = Path(__file__).resolve().parents[2]
NATIVE = os.environ.get('ASMC_HIGHWAY_PREFLIGHT', '')


@unittest.skipUnless(NATIVE, 'set ASMC_HIGHWAY_PREFLIGHT to the built native tool')
class NativePreflightTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.config = Path(self.temp.name) / 'params.yaml'
        package = ROOT / 'src/asmc_planner'
        self.settings = yaml.safe_load((package / 'config/frenet/params.yaml').read_text())
        self.settings['waypoint_file'] = str((package / 'src' / self.settings['waypoint_file']).resolve())
        self.settings['avoid_hd_map']['directory'] = str(package / 'config/frenet/hd_map')

    def check(self, expected_success, detail=None):
        self.config.write_text(yaml.safe_dump(self.settings))
        run = subprocess.run([NATIVE, str(self.config)], text=True,
                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=60)
        self.assertEqual(expected_success, run.returncode == 0, run.stdout)
        if detail:
            self.assertIn(detail, run.stdout)

    def test_actual_map_and_route(self):
        self.check(True, 'PREFLIGHT OK')

    def test_missing_route(self):
        self.settings['waypoint_file'] = str(Path(self.temp.name) / 'missing.txt')
        self.check(False)

    def test_invalid_speed_policy(self):
        self.settings['planner']['highway_merge']['speed_error_mps'] = -1
        self.check(False, 'invalid highway prediction/search bound')

    def test_invalid_motion_uncertainty_policy(self):
        policy=self.settings['planner']['highway_merge']
        for key,value in [('speed_error_floor_mps',-1),('speed_error_floor_mps',6),
                          ('motion_fit_min_span_s',0),('motion_fit_min_span_s',.6),
                          ('speed_error_correlation_s',0),('speed_error_correlation_s',2)]:
            original=policy[key]
            with self.subTest(key=key,value=value):
                policy[key]=value
                self.check(False,'invalid highway prediction/search bound')
            policy[key]=original

    def test_malformed_policy_type(self):
        self.settings['planner']['highway_merge']['speed_cap_mps'] = 'wrong'
        self.check(False)

    def test_invalid_forward_response_fails_before_driving(self):
        for value in (0, -1, float('nan')):
            with self.subTest(value=value):
                self.settings['planner']['highway_merge']['forward_accel_mps2'] = value
                self.check(False, 'invalid highway policy scalar')

    def test_older_config_uses_default_forward_response(self):
        del self.settings['planner']['highway_merge']['forward_accel_mps2']
        self.check(True, 'PREFLIGHT OK')

    def test_invalid_highway_curve_policy(self):
        for key in ('curve_lateral_accel_safety_factor', 'curve_comfortable_decel_factor'):
            for value in (0, 1.1, float('nan')):
                with self.subTest(key=key, value=value):
                    self.settings['planner']['highway_merge'][key] = value
                    self.check(False, 'invalid highway curve policy factor')
            self.settings['planner']['highway_merge'][key] = .5

    def test_missing_target_lane(self):
        self.settings['planner']['highway_merge']['zones'][0]['target_lanes'] = ['lane:missing']
        self.check(False)

    def test_invalid_handover_alignment_fails_before_driving(self):
        self.settings['planner']['highway_merge']['reference_handover']['max_pose_gap_s'] = 0
        self.check(False, 'invalid reference alignment policy')

    def test_invalid_handover_continuity_fails_before_driving(self):
        self.settings['planner']['highway_merge']['reference_handover']['continuity']['overlap_s'] = 0
        self.check(False, 'invalid prepared reference execution')

    def test_invalid_transition_tracking_fails_before_driving(self):
        tracking = self.settings['planner']['highway_merge']['transition_tracking']
        for key, value in [('position_m', 0), ('heading_rad', 1.6),
                           ('curvature_inv_m', float('nan'))]:
            original = tracking[key]
            with self.subTest(key=key):
                tracking[key] = value
                self.check(False, 'invalid highway')
            tracking[key] = original

    def test_transition_tracking_defaults_support_older_configs(self):
        del self.settings['planner']['highway_merge']['transition_tracking']
        self.check(True, 'PREFLIGHT OK')

    def test_disconnected_target_sequence(self):
        self.settings['planner']['highway_merge']['zones'][0]['target_lanes'] = [
            'lane:A2256W000409', 'lane:A2256W000153']
        self.check(False, 'disconnected target sequence')

    def test_completion_off_target(self):
        self.settings['planner']['highway_merge']['zones'][0]['completion_s'] = 1280
        self.settings['planner']['highway_merge']['zones'][0]['target_lanes'] = ['lane:A2256W000409']
        self.check(False, 'completion footprint outside')


if __name__ == '__main__':
    unittest.main()
