import unittest
import numpy as np
from asmc_hd_map.route_reference import RouteReference


class RouteReferenceTest(unittest.TestCase):
    def test_station_xy_ignores_elevation_and_interpolates_exact_boundaries(self):
        route = RouteReference([[0, 0, 0], [3, 0, 9], [3, 4, 9]], [0, 3, 7])
        self.assertEqual(route.point_at(3), (3., 0., 9.))
        self.assertEqual(route.point_at(7), (3., 4., 9.))
        result = route.project((2, 1))
        self.assertEqual(result.station_m, 2)
        self.assertEqual(result.distance_m, 1)
        self.assertIsNone(route.project((30, 40)))
        with self.assertRaises(ValueError):
            RouteReference([[0, 0, 0], [3, 0, 9]], [0, np.sqrt(90)])

    def test_crossing_hint_preserves_progress_and_reset_reacquires(self):
        points = np.array([[-10, 0, 0], [10, 0, 0], [10, 10, 0], [0, 10, 0], [0, -10, 0]])
        route = RouteReference(points, [0, 20, 30, 40, 60])
        self.assertEqual(route.project((0, 0), hint_m=50, search_m=5).station_m, 50)
        self.assertEqual(route.project((0, 0)).station_m, 10)

    def test_duplicate_points_and_closed_route_seam(self):
        route = RouteReference([[0, 0, 0], [0, 0, 0], [10, 0, 0], [10, 10, 0], [0, 0, 0]], [0, 0, 10, 20, 20 + np.sqrt(200)])
        self.assertEqual(route.point_at(0), (0., 0., 0.))
        self.assertLess(route.project((.1, 0), hint_m=route.length_m, search_m=1).distance_m, .01)
        with self.assertRaises(ValueError):
            route.point_at(-1)
