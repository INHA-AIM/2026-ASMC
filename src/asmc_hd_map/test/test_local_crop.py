import unittest
from pathlib import Path

import yaml

import numpy as np

from asmc_hd_map.local_crop import (
    downsample_occupancy,
    bounds_intersect,
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

    def test_display_downsample_keeps_any_drivable_cell_and_pads_with_non_drivable(self):
        occupancy = np.full((5, 7), 100, dtype=np.int8)
        occupancy[0, 0] = 0
        occupancy[4, 6] = 0
        coarse = downsample_occupancy(occupancy, 3)
        self.assertEqual(coarse.shape, (2, 3))
        self.assertEqual(coarse[0, 0], 0)
        self.assertEqual(coarse[1, 2], 0)
        self.assertEqual(int((coarse == 0).sum()), 2)
        np.testing.assert_array_equal(downsample_occupancy(occupancy, 1), occupancy)
        with self.assertRaises(ValueError):
            downsample_occupancy(occupancy, 0)

    def test_phase_two_launch_and_contract_enable_local_crop(self):
        launch = (
            self.repo_root / "src/asmc_hd_map/launch/competition_map_debug.launch"
        ).read_text(encoding="utf-8")
        self.assertIn('name="publish_local" default="true"', launch)
        self.assertIn('type="local_map_crop_node.py"', launch)
        contract = yaml.safe_load((
            self.repo_root / "config/contracts/hd_map_topic_contract_v1.yaml"
        ).read_text(encoding="utf-8"))
        policy = contract["local_crop_policy"]
        self.assertEqual(policy["geometry_frame"], "map")
        self.assertEqual(policy["extent_m"], 120.0)
        self.assertEqual(policy["grid_width_cells"], 600)
        self.assertFalse(policy["lane_array_complete"])
        self.assertEqual(policy["default_pose_source"], "gps_imu")
        self.assertIn("gps_imu", policy["supported_pose_sources"])
        self.assertIn('name="local_pose_source" default="gps_imu"', launch)


if __name__ == "__main__":
    unittest.main()
