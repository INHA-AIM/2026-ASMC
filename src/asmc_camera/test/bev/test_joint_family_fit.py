import unittest
import numpy as np

from asmc_camera.bev.shared_lane_family import (
    _FitWorkspace, _fit_joint_width_controls, _config, LEFT_ROLE, RIGHT_ROLE,
    YELLOW_ROLE, fit_shared_lane_family_with_audit)
from asmc_camera.bev.lane_evidence import LaneNode
from asmc_camera.bev.lane_tracking import LaneTrack
from asmc_camera.bev.family_fit_policy import FamilyFitPolicy


class JointFamilyFitTest(unittest.TestCase):
    def solve(self, left, right):
        workspace = _FitWorkspace(7, 3)
        t = np.tile(np.linspace(0, 1, 80), 3)
        roles = np.repeat([LEFT_ROLE, YELLOW_ROLE, RIGHT_ROLE], 80)
        controls = np.column_stack((np.linspace(0, 18, 7), np.zeros(7)))
        normals = np.tile([0., 1.], (len(t), 1))
        points = workspace.basis(t) @ controls + normals * np.repeat([left, 0., -right], 80)[:, None]
        return _fit_joint_width_controls(points, t, normals, roles, np.ones(len(t)),
            _config(dict(width_prior_strength=0, spline_smoothing=0)), workspace)

    def test_joint_recovers_observed_widths_without_nominal_attraction(self):
        _, _, widths = self.solve(3.1, 3.8)
        self.assertAlmostEqual(widths[LEFT_ROLE], 3.1, places=9)
        self.assertAlmostEqual(widths[RIGHT_ROLE], 3.8, places=9)

    def test_width_bounds_resolve_controls_instead_of_clipping_after_fit(self):
        _, controls, widths = self.solve(4.9, 4.9)
        self.assertAlmostEqual(widths[LEFT_ROLE], 4.6)
        self.assertAlmostEqual(widths[RIGHT_ROLE], 4.6)
        np.testing.assert_allclose(controls[:, 1], 0, atol=1e-9)

    def test_missing_yellow_retains_exact_original_gauge_and_support(self):
        tracks = [LaneTrack(tuple(LaneNode(np.array([x, y]), 1, .9, .9, 1, 4, i)
                    for i, x in enumerate(np.arange(0, 18.01, .5)))) for y in (3.4, -3.6)]
        a, _ = fit_shared_lane_family_with_audit(tracks, {})
        b, _ = fit_shared_lane_family_with_audit(tracks, dict(joint_width_fit_enabled=True))
        for before, after in zip(a.boundaries, b.boundaries):
            np.testing.assert_array_equal(before.sample_points, after.sample_points)
            np.testing.assert_array_equal(before.sample_state, after.sample_state)

    def test_joint_solver_is_scoped_to_corrected_frames(self):
        policy = FamilyFitPolicy({}, 0, True)
        self.assertTrue(policy.select(True).parameters['joint_width_fit_enabled'])
        self.assertFalse(policy.select(False).parameters['joint_width_fit_enabled'])


if __name__ == '__main__': unittest.main()
