"""Geometry-only regression for the offline clearance measurement, no ROS."""
import unittest
import numpy as np
from measure_tunnel_clearance import wall_band


class ClearanceTest(unittest.TestCase):
    def test_known_inner_surface_and_left_right_symmetry(self):
        x, z = np.meshgrid(np.linspace(2, 4, 35), np.linspace(-.1, 1.3, 12))
        for sign in (-1, 1):
            line = np.array([[1., sign * 2., 0.], [5., sign * 2., 0.]])
            points = np.column_stack([x.ravel(), np.full(x.size, sign * 3.2), z.ravel()])
            result = wall_band(points, line, 3., sign)
            self.assertIsNotNone(result)
            self.assertAlmostEqual(result['gap_m'], 1.2, places=8)

    def test_pole_or_horizontal_shelf_is_not_wall_evidence(self):
        line = np.array([[1., 2., 0.], [5., 2., 0.]])
        z = np.linspace(-.1, 1.3, 100)
        pole = np.column_stack([np.full(100, 3.), np.full(100, 3.2), z])
        self.assertIsNone(wall_band(pole, line, 3., 1))
        shelf = np.column_stack([np.linspace(2., 4., 100), np.full(100, 3.2), np.zeros(100)])
        self.assertIsNone(wall_band(shelf, line, 3., 1))


if __name__ == '__main__':
    unittest.main()
