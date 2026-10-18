import concurrent.futures
import hashlib
import json
import unittest

import numpy as np

from asmc_camera.bev.family_fit_policy import FamilyFitPolicy
from asmc_camera.bev.lane_evidence import LaneNode
from asmc_camera.bev.lane_tracking import LaneTrack
from asmc_camera.bev.shared_lane_family import fit_shared_lane_family_with_audit


def tracks(include_right=True):
    result = []
    for label, offset in [(1, 3.0), (2, 0.0)] + ([(1, -3.2)] if include_right else []):
        result.append(LaneTrack(tuple(LaneNode(
            xy=np.array([x, offset]), class_id=label, confidence=.9,
            geometric_quality=.9, source_mask=1, support=4, x_bin=i)
            for i, x in enumerate(np.arange(0., 18.01, .5)))))
    return result


class FamilyFitPolicyTest(unittest.TestCase):
    def test_missing_imu_retains_exact_original_fit_across_transitions(self):
        original = dict(width_prior_strength=.35)
        policy = FamilyFitPolicy(original)
        reference, _ = fit_shared_lane_family_with_audit(tracks(), original)
        for reason in ('missing_source_timing', 'new_bridge', 'source_regression', 'disabled'):
            corrected = policy.select(True, 'corrected')
            self.assertEqual(corrected.parameters['width_prior_strength'], 0.)
            fallback = policy.select(False, reason)
            family, _ = fit_shared_lane_family_with_audit(tracks(), fallback.parameters)
            self.assertEqual(fallback.audit()['reason'], reason)
            for a, b in zip(reference.boundaries, family.boundaries):
                np.testing.assert_array_equal(a.sample_points, b.sample_points)
                np.testing.assert_array_equal(a.sample_state, b.sample_state)
        self.assertEqual(original, dict(width_prior_strength=.35))

    def test_relaxation_preserves_bounds_and_inferred_role(self):
        policy = FamilyFitPolicy({})
        family, _ = fit_shared_lane_family_with_audit(tracks(False), policy.select(True).parameters)
        self.assertIsNotNone(family)
        self.assertTrue(2.4 <= family.left_width_m <= 4.6)
        self.assertTrue(2.4 <= family.right_width_m <= 4.6)
        right = next(b for b in family.boundaries if b.role == 'right_outer_boundary')
        self.assertNotIn(1, right.sample_state)  # missing paint never becomes observed

    def test_frames_are_immutable_and_parallel_selections_are_independent(self):
        settings = dict(width_prior_strength=.35)
        policy = FamilyFitPolicy(settings)
        settings['width_prior_strength'] = 99
        with self.assertRaises(TypeError):
            policy.select(True).parameters['width_prior_strength'] = .2
        with concurrent.futures.ThreadPoolExecutor(max_workers=4) as executor:
            values = list(executor.map(lambda i: policy.select(i % 2 == 0).audit(), range(100)))
        self.assertEqual([v['width_prior_strength'] for v in values], [0., .35] * 50)

    def test_invalid_policies_reject_and_contract_identifies_both_branches(self):
        for value in (-1, .36, float('nan'), float('inf')):
            with self.assertRaises(ValueError): FamilyFitPolicy({}, value)
        def digest(value):
            return hashlib.sha256(json.dumps(value.contract(), sort_keys=True).encode()).hexdigest()
        self.assertNotEqual(digest(FamilyFitPolicy({}, 0)), digest(FamilyFitPolicy({}, .1)))
        self.assertNotEqual(digest(FamilyFitPolicy({}, 0)), digest(FamilyFitPolicy({'width_prior_strength': .3}, 0)))


if __name__ == '__main__': unittest.main()
