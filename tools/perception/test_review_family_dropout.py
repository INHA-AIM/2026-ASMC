"""Safety and reporting regressions for offline lane-evidence removal."""
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from review_family_dropout import cases, describe, summarize
from asmc_camera.bev.lane_evidence import LaneNode
from asmc_camera.bev.lane_tracking import LaneTrack
from asmc_camera.bev.shared_lane_family import (
    ROLE_ORDER, _assign_observed_roles, _config, fit_shared_lane_family_with_audit,
)


def tracks():
    result = []
    for class_id, offset in ((1, 3.5), (2, 0.), (1, -3.5)):
        result.append(LaneTrack(tuple(
            LaneNode(np.array([x, offset]), class_id, .9, .9, 1, 4, i)
            for i, x in enumerate(np.arange(0., 18.01, .25)))))
    return result


class DropoutReviewTest(unittest.TestCase):
    def setUp(self):
        self.config = dict(same_frame_completion_enabled=True)
        self.assigned, _ = _assign_observed_roles(tracks(), _config(self.config))

    def test_whole_role_loss_is_inferred_and_two_losses_are_rejected(self):
        for name, values, removed in cases(self.assigned):
            family, audit = fit_shared_lane_family_with_audit(values, self.config)
            if name.startswith('drop:'):
                self.assertIsNotNone(family, name)
                missing = next(b for b in family.boundaries if b.role == removed)
                self.assertEqual(missing.state, 'structure_inferred')
                self.assertFalse(np.any(missing.sample_observed))
                self.assertEqual(len(audit['observed_roles']), 2)
            elif name.startswith('only:'):
                self.assertIsNone(family, name)
                self.assertEqual(audit['reason'], 'too_few_observed_roles')

    def test_local_gap_preserves_other_evidence_without_mutating_original(self):
        before = {r: t.xy.copy() for r, t in self.assigned.items()}
        for name, values, removed in cases(self.assigned):
            if not name.startswith('gap'):
                continue
            family, audit = fit_shared_lane_family_with_audit(values, self.config)
            self.assertIsNotNone(family)
            boundary = next(b for b in family.boundaries if b.role == removed)
            middle = (boundary.sample_points[:, 0] > 6.) & (boundary.sample_points[:, 0] < 9.)
            self.assertTrue(np.any(middle))
            self.assertTrue(np.all(boundary.sample_state[middle] == 2))
            self.assertEqual(len(audit['observed_roles']), 3)
        for role, value in before.items():
            np.testing.assert_array_equal(value, self.assigned[role].xy)

    def test_naturally_missing_role_removal_does_not_add_evidence(self):
        assigned = {r: t for r, t in self.assigned.items() if r != ROLE_ORDER[1]}
        case = next(c for c in cases(assigned) if c[0] == 'drop:' + ROLE_ORDER[1])
        self.assertEqual(len(case[1]), 2)
        family, audit = fit_shared_lane_family_with_audit(case[1], self.config)
        self.assertIsNotNone(family)
        self.assertNotIn(ROLE_ORDER[1], audit['observed_roles'])

    def test_short_valid_extent_is_not_extrapolated_to_full_consumer_zone(self):
        boundary = NS(role=ROLE_ORDER[0], state='observed',
                      sample_points=np.array([[4., 0.], [6., 0.], [8., 0.]]),
                      sample_state=np.array([1, 2, 0]), observed_length_m=0.,
                      inferred_length_m=2., invalid_length_m=2.)
        family = NS(boundaries=[boundary], valid_x_min_m=4., valid_x_max_m=6.)
        result = describe(family, dict(reason='accepted'))['boundaries'][ROLE_ORDER[0]]
        self.assertEqual(result['consumer_covered_x_m'], 2.)
        self.assertEqual(result['consumer_uncovered_x_m'], 13.)
        self.assertEqual(result['consumer_observed_x_m'], 0.)

    def test_summary_retains_failures_and_no_op_removals_per_condition(self):
        rows = [dict(run_id='sunny11', phase='entry', case='drop:left', affected=True,
                     result=dict(valid=False, reason='too_few_observed_roles')),
                dict(run_id='sunny11', phase='entry', case='drop:left', affected=False,
                     result=dict(valid=True, reason='accepted'))]
        row = summarize(rows)[0]
        self.assertEqual((row['frames'], row['valid'], row['affected_frames']), (2, 1, 1))
        self.assertEqual(row['reasons']['too_few_observed_roles'], 1)


if __name__ == '__main__':
    unittest.main()
