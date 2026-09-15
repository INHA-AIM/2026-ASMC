#!/usr/bin/env python3

import unittest

from drivable_bev.pose_time_alignment import align_ego_pose, pose_delta
from drivable_bev.rgb_bev import StampedEgoPose


class PoseTimeAlignmentTest(unittest.TestCase):
    def setUp(self):
        self.poses = [
            StampedEgoPose(1_000_000_000, 0.0, 1.0, 179.0),
            StampedEgoPose(1_020_000_000, 2.0, 3.0, -179.0),
        ]

    def test_interpolates_xy_and_shortest_heading(self):
        value = align_ego_pose(self.poses, 1_010_000_000, 20_000_000, 40_000_000)
        self.assertEqual("interpolated", value.mode)
        self.assertAlmostEqual(1.0, value.pose.x_m)
        self.assertAlmostEqual(2.0, value.pose.y_m)
        self.assertAlmostEqual(-180.0, value.pose.heading_deg)
        self.assertAlmostEqual(20.0, value.bracket_span_ms)

    def test_falls_back_to_nearest_when_bracket_is_too_wide(self):
        value = align_ego_pose(self.poses, 1_005_000_000, 10_000_000, 5_000_000)
        self.assertEqual("nearest_fallback", value.mode)
        self.assertEqual(self.poses[0], value.pose)

    def test_nearest_mode_preserves_legacy_behavior(self):
        value = align_ego_pose(self.poses, 1_017_000_000, 10_000_000, 40_000_000, "nearest")
        self.assertEqual("nearest", value.mode)
        self.assertEqual(self.poses[1], value.pose)

    def test_rejects_stale_nearest(self):
        value = align_ego_pose(self.poses, 1_100_000_000, 10_000_000, 40_000_000)
        self.assertEqual("unavailable", value.mode)
        self.assertIsNone(value.pose)

    def test_reports_pose_delta(self):
        delta = pose_delta(self.poses[0], StampedEgoPose(2, 3.0, 5.0, -179.0))
        self.assertAlmostEqual(5.0, delta["translation_m"])
        self.assertAlmostEqual(2.0, delta["heading_deg"])


if __name__ == "__main__":
    unittest.main()
