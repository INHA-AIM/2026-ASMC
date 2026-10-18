import unittest
from unittest.mock import patch
import numpy as np

from asmc_camera.bev.shared_lane_family import (
    _FitWorkspace, _fit_joint_width_controls, _config, LEFT_ROLE, RIGHT_ROLE,
    YELLOW_ROLE, fit_shared_lane_family_with_audit, _basis_derivative,
    _interpolate_track_y, _track_separation, _track_y_at_x)
from asmc_camera.bev.spline_fitting import _bspline_basis_uncached
from asmc_camera.bev.lane_evidence import LaneNode
from asmc_camera.bev.lane_tracking import LaneTrack
from asmc_camera.bev.family_fit_policy import FamilyFitPolicy


class JointFamilyFitTest(unittest.TestCase):
    def test_vector_interpolation_matches_scalar_sort_and_duplicate_policy(self):
        points = np.array([[3., 9.], [0., 1.], [3., 99.], [1., 2.], [2., 5.]])
        query = np.array([-1., 0., .1, 1.5, 3., 9.])
        # The first duplicate in stable input order remains authoritative.
        expected = [float(np.interp(float(np.clip(x, 0., 3.)),
                    [0., 1., 2., 3.], [1., 2., 5., 9.])) for x in query]
        np.testing.assert_array_equal(_interpolate_track_y(points, query), expected)

    def test_role_separation_matches_original_scalar_queries(self):
        def track(start, stop, offset):
            return LaneTrack(tuple(LaneNode(np.array([x, offset + .02*x*x]),
                1, .9, .9, 1, 4, i) for i, x in enumerate(np.linspace(start, stop, 60))))
        first = track(0., 22., -3.)
        for second in (track(1., 23., 3.), track(30., 40., 3.)):
            low, high = max(first.xy[:, 0].min(), second.xy[:, 0].min()), min(first.xy[:, 0].max(), second.xy[:, 0].max())
            query = np.linspace(low, high, max(3, int(np.ceil(high-low)) + 1)) if high-low >= 1. else [5.]
            expected = float(np.median([_track_y_at_x(second, x)-_track_y_at_x(first, x) for x in query]))
            self.assertEqual(_track_separation(first, second, 5.), expected)

    def test_dense_selection_reuses_exact_basis_and_derivative_rows(self):
        workspace = _FitWorkspace(12, 3)
        for indices in (np.array([0, 32, 32, 383, 1, 100]),
                        np.array([383, 0, 16])):
            parameters = workspace.select_dense(indices)
            expected_basis = _bspline_basis_uncached(parameters, workspace.knots, 3, 12)
            expected_derivative = _basis_derivative(parameters, workspace.knots, 3, 12)
            with patch('asmc_camera.bev.shared_lane_family._bspline_basis_uncached',
                       side_effect=AssertionError('selected rows must not be recomputed')):
                np.testing.assert_array_equal(workspace.basis(parameters), expected_basis)
                np.testing.assert_array_equal(workspace.derivative(parameters), expected_derivative)
        # An arbitrary later parameter array must still use the generic path.
        arbitrary = np.array([.123456, .654321])
        np.testing.assert_array_equal(workspace.basis(arbitrary),
            _bspline_basis_uncached(arbitrary, workspace.knots, 3, 12))
        np.testing.assert_array_equal(workspace.derivative(arbitrary),
            _basis_derivative(arbitrary, workspace.knots, 3, 12))

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
