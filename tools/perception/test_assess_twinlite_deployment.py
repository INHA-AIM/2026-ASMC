import unittest
from tools.perception.assess_twinlite_deployment import assess


class DeploymentAcceptanceTest(unittest.TestCase):
    def report(self):
        supply = dict(five_second_hz=[15., 14.8], max_gap_sec=.15)
        return dict(complete=True, acceptance_eligible=True, failures=[], start=10., end=20.,
                    streams={'lane': dict(supply, timing_pass=True)},
                    lane_family=dict(valid_supply=supply, producer_supply=supply),
                    resources=dict(resource_coverage_pass=True, vram_peak_fraction=.6),
                    qt_inventory_before=[], qt_inventory_during=[{'owned_trial': True}],
                    qt_inventory_after=[])

    def test_accepts_valid_window_and_excludes_prepare_cleanup(self):
        rows = [dict(at=t, role='lane', pipeline_ready=ready, errors=0)
                for t, ready in [(9., False), (11., True), (20., False)]]
        self.assertTrue(assess(self.report(), rows)['pass_'])

    def test_execution_success_does_not_hide_readiness_drop(self):
        rows = [dict(at=11., role='lane', pipeline_ready=False, errors=0)]
        result = assess(self.report(), rows)
        self.assertFalse(result['pass_'])
        self.assertEqual(result['normal_lane_readiness_drops'], 1)

    def test_execution_success_does_not_hide_bad_rate_or_timing(self):
        report = self.report()
        report['streams']['lane'].update(five_second_hz=[9.2], timing_pass=False)
        rows = [dict(at=11., role='lane', pipeline_ready=True, errors=0)]
        self.assertEqual(assess(report, rows)['failures'], ['lane: rate', 'lane: latency/gap'])

    def test_missing_status_and_incomplete_window_fail(self):
        report = self.report()
        report['complete'] = False
        result = assess(report, [])
        self.assertFalse(result['pass_'])
        self.assertIn('normal ACTIVE readiness/errors', result['failures'])


if __name__ == '__main__':
    unittest.main()
