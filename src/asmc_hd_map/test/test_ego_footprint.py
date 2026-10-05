#!/usr/bin/env python3

import math
from pathlib import Path
import unittest

import numpy as np
import yaml

from asmc_hd_map.ego_footprint import (
    IONIQ5_FOOTPRINT,
    VehicleFootprint,
    footprint_corners,
    footprint_drivable_summary,
)

RVIZ_DIR = Path(__file__).resolve().parents[1] / "rviz"


class EgoFootprintTest(unittest.TestCase):
    def test_ioniq5_footprint_matches_rule_book_dimensions(self):
        self.assertAlmostEqual(IONIQ5_FOOTPRINT.length_m, 4.635, places=9)
        self.assertAlmostEqual(IONIQ5_FOOTPRINT.width_m, 1.892, places=9)
        corners = footprint_corners(0.0, 0.0, 0.0)
        np.testing.assert_allclose(corners[0], (-0.79, 0.946), atol=1e-12)
        np.testing.assert_allclose(corners[2], (3.845, -0.946), atol=1e-12)

    def test_corners_rotate_about_base_link(self):
        corners = footprint_corners(10.0, 5.0, math.pi / 2.0)
        rear_left, front_left = corners[0], corners[1]
        self.assertAlmostEqual(rear_left[0], 10.0 - 0.946)
        self.assertAlmostEqual(rear_left[1], 5.0 - 0.79)
        self.assertAlmostEqual(front_left[1], 5.0 + 3.845)

    def test_outside_fraction_counts_non_drivable_cells(self):
        occupancy = np.full((100, 100), 100, dtype=np.int8)
        occupancy[:, :50] = 0  # x < 10 m drivable at 0.2 m/cell from origin (0, 0)
        inside = footprint_drivable_summary(3.0, 10.0, 0.0, occupancy, 0.2, (0.0, 0.0))
        self.assertEqual(inside["outside_fraction"], 0.0)
        self.assertEqual(inside["corner_drivable"], [True] * 4)
        straddle = footprint_drivable_summary(8.0, 10.0, 0.0, occupancy, 0.2, (0.0, 0.0))
        self.assertGreater(straddle["outside_fraction"], 0.2)
        self.assertEqual(straddle["corner_drivable"], [True, False, False, True])

    def test_invalid_dimensions_are_rejected(self):
        with self.assertRaises(ValueError):
            VehicleFootprint(width_m=0.0)

    def test_follow_ego_rviz_differs_only_in_the_current_view(self):
        overview = yaml.safe_load((RVIZ_DIR / "competition_map_debug.rviz").read_text(encoding="utf-8"))
        follow = yaml.safe_load((RVIZ_DIR / "competition_map_follow_ego.rviz").read_text(encoding="utf-8"))
        self.assertEqual(follow["Visualization Manager"]["Views"]["Current"]["Target Frame"], "base_link")
        overview["Visualization Manager"]["Views"].pop("Current")
        follow["Visualization Manager"]["Views"].pop("Current")
        self.assertEqual(overview, follow)


if __name__ == "__main__":
    unittest.main()
