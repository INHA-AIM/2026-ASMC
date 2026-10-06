import unittest
from pathlib import Path

import yaml

import numpy as np

from asmc_hd_map.local_crop import (
    bounds_intersect,
    EgoWindow,
    crop_ego_occupancy_grid,
    select_ego_bounds,
    crop_bounds,
    crop_occupancy_grid,
    geometry_bounds,
    select_intersecting_bounds,
)


class LocalCropTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.repo_root = Path(__file__).resolve().parents[3]

    def test_geometry_bounds_ignores_non_finite_points(self):
        bounds = geometry_bounds((
            [[1.0, 2.0], [3.0, 4.0]],
            [[float("nan"), 9.0], [-2.0, 5.0]],
        ))
        self.assertEqual(bounds, (-2.0, 2.0, 3.0, 5.0))
        self.assertIsNone(geometry_bounds(([],)))

    def test_lane_selection_uses_closed_map_frame_crop(self):
        window = crop_bounds((0.0, 0.0), 20.0)
        self.assertEqual(window, (-10.0, -10.0, 10.0, 10.0))
        self.assertTrue(bounds_intersect((-12.0, -1.0, -10.0, 1.0), window))
        self.assertFalse(bounds_intersect((10.1, -1.0, 11.0, 1.0), window))
        selected = select_intersecting_bounds(
            [(-12.0, -1.0, -10.0, 1.0), None, (10.1, -1.0, 11.0, 1.0)],
            (0.0, 0.0),
            20.0,
        )
        self.assertEqual(selected, [0])

    def test_occupancy_crop_preserves_source_cells_and_alignment(self):
        source = np.arange(20, dtype=np.int8).reshape(4, 5)
        result = crop_occupancy_grid(
            source, resolution=1.0, origin_xy=(10.0, 20.0),
            center_xy=(12.0, 22.0), extent_m=4.0,
        )
        self.assertEqual((result.height, result.width), (4, 4))
        self.assertEqual((result.origin_x, result.origin_y), (10.0, 20.0))
        self.assertTrue(np.array_equal(result.data, source[:, :4]))

    def test_outside_grid_is_unknown_with_fixed_dimensions(self):
        source = np.arange(20, dtype=np.int8).reshape(4, 5)
        result = crop_occupancy_grid(
            source, resolution=1.0, origin_xy=(10.0, 20.0),
            center_xy=(9.0, 19.0), extent_m=4.0,
        )
        self.assertEqual((result.height, result.width), (4, 4))
        self.assertEqual((result.origin_x, result.origin_y), (7.0, 17.0))
        self.assertEqual(int(result.data[3, 3]), int(source[0, 0]))
        self.assertEqual(int(np.count_nonzero(result.data == -1)), 15)

    def test_competition_default_is_600_cells_at_point_two_meters(self):
        source = np.zeros((7232, 3085), dtype=np.int8)
        result = crop_occupancy_grid(
            source, resolution=0.2, origin_xy=(-200.0, -600.0),
            center_xy=(-131.6898, -428.3310), extent_m=120.0,
        )
        self.assertEqual((result.height, result.width), (600, 600))
        self.assertTrue(np.all(result.data == 0))

    def test_invalid_inputs_are_rejected(self):
        with self.assertRaises(ValueError):
            crop_bounds((0.0, 0.0), 0.0)
        with self.assertRaises(ValueError):
            crop_occupancy_grid(
                np.zeros((2, 2), dtype=np.int8), 0.0, (0.0, 0.0), (0.0, 0.0), 2.0
            )

    def test_ego_window_rotates_and_retains_rear_context(self):
        window = EgoWindow(front_m=9, rear_m=3, left_m=2, right_m=2)
        # Heading north: retain y=-3..9 and x=-2..2, not eastward points.
        selected = select_ego_bounds([
            (-.1, 8, .1, 9), (-.1, -3, .1, -2),
            (3, 0, 4, 1), (-.1, -5, .1, -4),
        ], (0, 0), np.pi / 2, window)
        self.assertEqual(selected, [0, 1])
        # Diagonal heading: AABB corner outside the actual window is excluded.
        selected = select_ego_bounds([(0, 7, .1, 7.1)], (0, 0), np.pi/4, window)
        self.assertEqual(selected, [])

    def test_rotated_grid_samples_known_cells_and_unknown(self):
        source = np.array([[0, 100, -1], [100, -1, 0], [-1, 0, 100]], dtype=np.int8)
        window = EgoWindow(front_m=1, rear_m=1, left_m=1, right_m=1)
        result = crop_ego_occupancy_grid(source, 1, (0, 0), (1, 1), np.pi/2, window)
        self.assertTrue(np.array_equal(result.data, np.array([[100, -1], [0, 100]], dtype=np.int8)))
        self.assertAlmostEqual(result.origin_x, 2)
        self.assertAlmostEqual(result.origin_y, 0)
        outside = crop_ego_occupancy_grid(source, 1, (0, 0), (-10, -10), 0, window)
        self.assertTrue(np.all(outside.data == -1))

    def test_default_ego_grid_is_half_the_old_cell_count(self):
        result = crop_ego_occupancy_grid(np.zeros((2, 2), dtype=np.int8), .2,
                                         (0, 0), (0, 0), 0, EgoWindow())
        self.assertEqual(result.data.shape, (300, 600))
        self.assertEqual((result.origin_x, result.origin_y), (-30, -30))
        with self.assertRaises(ValueError):
            EgoWindow(front_m=float("nan"))
        with self.assertRaises(ValueError):
            crop_ego_occupancy_grid(np.zeros((2, 2)), 1, (0, 0), (0, 0), float("nan"), EgoWindow())

    def test_phase_two_launch_keeps_crop_optional(self):
        launch = (
            self.repo_root / "src/asmc_hd_map/launch/competition_map_debug.launch"
        ).read_text(encoding="utf-8")
        self.assertIn('name="publish_local" default="false"', launch)
        self.assertIn('type="local_map_crop_node.py"', launch)
        contract = yaml.safe_load((
            self.repo_root / "config/contracts/hd_map_topic_contract_v1.yaml"
        ).read_text(encoding="utf-8"))
        policy = contract["local_crop_policy"]
        self.assertEqual(policy["geometry_frame"], "map")
        self.assertEqual(policy["default_mode"], "ego")
        self.assertEqual(policy["front_m"], 90.0)
        self.assertEqual(policy["rear_m"], 30.0)
        self.assertEqual(policy["grid_width_cells"], 600)
        self.assertFalse(policy["lane_array_complete"])
        self.assertEqual(policy["default_pose_source"], "gps_imu")
        self.assertIn("gps_imu", policy["supported_pose_sources"])
        self.assertIn('name="local_pose_source" default="gps_imu"', launch)


if __name__ == "__main__":
    unittest.main()
