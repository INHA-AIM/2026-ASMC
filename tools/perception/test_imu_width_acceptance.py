import unittest
import numpy as np

from evaluate_imu_width_candidate import support_intervals, lost_support, summarize_quality, VARIANTS


class ImuWidthAcceptanceTest(unittest.TestCase):
    def test_coverage_preserves_holes_and_does_not_hide_loss_with_extra_tail(self):
        b = dict(xy=[[x, 0] for x in (0, 4, 5, 7, 8, 15)], support=[1, 1, 0, 0, 2, 2])
        self.assertEqual(support_intervals(b), [(0., 4.), (8., 15.)])
        self.assertEqual(lost_support([(0., 10.)], [(1., 15.)]), 1.)
        self.assertEqual(lost_support([(0., 4.), (8., 15.)], [(0., 15.)]), 0.)

    def records(self):
        rows, pairs = [], []
        for i, phase in enumerate(('accelerate', 'brake', 'accelerate2', 'brake2', 'hold', 'steer')):
            family = dict(valid=True, widths_m=[3.5, 3.5], boundaries=[dict(
                role=role, class_id=label, xy=[[0, y], [15, y]], support=[1, 1])
                for role, label, y in [('left_outer_boundary', 1, 3.5),
                                     ('yellow_centre_boundary', 2, 0), ('right_outer_boundary', 1, -3.5)]],
                semantic_model_px={'observed/right/1': dict(count=3, p50=0)})
            import copy
            rows.append(dict(index=i, phase=phase, stamp_ns=i * 10**9,
                             variants={n: copy.deepcopy(family) for n in VARIANTS}))
            pairs.append(dict(phase=phase, variants={n: dict(count=50, p95=.03 if n == VARIANTS[-1] else .1)
                                                     for n in VARIANTS}))
        return rows, pairs

    def test_candidate_with_motion_gain_still_fails_pixel_or_support_gate(self):
        rows, pairs = self.records()
        self.assertTrue(summarize_quality(rows, pairs)['passed'])
        for row in rows:
            row['variants'][VARIANTS[-1]]['semantic_model_px']['observed/right/1']['p50'] = 2
        report = summarize_quality(rows, pairs)
        self.assertTrue(report['checks']['motion'])
        self.assertFalse(report['passed'])
        self.assertFalse(report['checks']['observed_pixel_distance'])
        rows, pairs = self.records()
        rows[0]['variants'][VARIANTS[-1]]['boundaries'][0]['xy'][0][0] = .031
        self.assertFalse(summarize_quality(rows, pairs)['checks']['support_preserved'])

    def test_missing_samples_and_no_data_never_pass(self):
        with self.assertRaises(ValueError): summarize_quality([], [])
        rows, pairs = self.records()
        self.assertFalse(summarize_quality(rows, [])['checks']['motion'])
        rows[0]['variants'][VARIANTS[-1]]['semantic_model_px'] = {}
        self.assertFalse(summarize_quality(rows, pairs)['checks']['observed_pixel_distance'])


if __name__ == '__main__': unittest.main()
