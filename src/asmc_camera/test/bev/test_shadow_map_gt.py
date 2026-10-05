#!/usr/bin/env python3

import unittest

import numpy as np

from asmc_camera.offline.bev.shadow_map_gt import (
    SCHEMA_VERSION,
    align_opposing_polylines,
    build_shadow_lane_pseudo_gt,
    lidar_paint_alignment,
    map_xy_to_base,
    nearest_polyline_distance,
    nearest_polyline_signed_normal_distance,
    points_in_rotated_rectangle,
    resample_polyline,
)


class ShadowMapGtTest(unittest.TestCase):
    def test_map_to_base_uses_forward_and_vehicle_left_axes(self):
        value = map_xy_to_base(
            [[10.0, 21.0], [9.0, 20.0]],
            ego_xy=[10.0, 20.0],
            heading_deg=90.0,
        )
        np.testing.assert_allclose(value, [[1.0, 0.0], [0.0, 1.0]], atol=1e-12)

    def test_nearest_distance_and_lidar_reflectivity_contrast(self):
        reference = np.asarray([[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]])
        points = np.asarray(
            [
                [0.0, 0.05, 50.0],
                [1.0, -0.10, 45.0],
                [0.0, 0.80, 10.0],
                [2.0, -1.00, 11.0],
            ]
        )
        np.testing.assert_allclose(
            nearest_polyline_distance(points[:, :2], reference),
            [0.05, 0.10, 0.80, 1.00],
        )
        value = lidar_paint_alignment(points, reference)
        self.assertEqual(2, value["near_points"])
        self.assertEqual(2, value["near_high_points"])
        self.assertEqual(2, value["control_points"])
        self.assertEqual(0, value["control_high_points"])

    def test_polyline_family_does_not_connect_curve_ends(self):
        family = np.asarray(
            [
                [[0.0, 0.0], [1.0, 0.0]],
                [[10.0, 0.0], [11.0, 0.0]],
            ]
        )
        value = nearest_polyline_distance([[5.0, 0.0]], family)
        self.assertAlmostEqual(4.0, value[0])

    def test_signed_normal_distance_uses_increasing_x_orientation(self):
        reference = np.asarray([[2.0, 0.0], [1.0, 0.0], [0.0, 0.0]])
        value = nearest_polyline_signed_normal_distance(
            [[1.0, 0.25], [1.0, -0.40]], reference
        )
        np.testing.assert_allclose(value, [0.25, -0.40], atol=1e-12)

    def setUp(self):
        self.links = [
            {
                "idx": "a",
                "points": [[-5.0, 1.75, 2.0], [5.0, 1.75, 2.0]],
                "width_start": 3.5,
                "width_end": 3.5,
            },
            {
                "idx": "b",
                "points": [[5.0, -1.75, 2.0], [-5.0, -1.75, 2.0]],
                "width_start": 3.5,
                "width_end": 3.5,
            },
        ]
        self.scenario = {
            "shadedAreaList": [
                {
                    "UNIQUEID": 12,
                    "pos": {"x": 0.0, "y": 0.0},
                    "rot": {"yaw": 0.0},
                    "size": {"x": 10.0, "y": 7.2, "z": 3.0},
                }
            ]
        }
        self.config = {
            "schema_version": "asmc-shadow-zone-config-1.0.0",
            "map": "synthetic",
            "operating_domain": "shadow_two_way_v1",
            "shadow_area": {
                "unique_id": 12,
                "yaw_to_map_offset_deg": 0.0,
                "max_link_axis_error_deg": 5.0,
            },
            "links": {"ids": ["a", "b"]},
            "geometry": {
                "resample_spacing_m": 0.5,
                "expected_lane_width_m": 3.5,
                "lane_width_tolerance_m": 0.2,
                "clip_margin_m": 0.0,
            },
            "provenance": {"note": "synthetic"},
        }

    def test_aligns_reversed_opposing_link(self):
        second, reversed_second = align_opposing_polylines(
            self.links[0]["points"], self.links[1]["points"]
        )
        self.assertTrue(reversed_second)
        self.assertLess(second[0, 0], second[-1, 0])

    def test_resampling_preserves_endpoints(self):
        value = resample_polyline(self.links[0]["points"], 7)
        np.testing.assert_allclose(value[0], self.links[0]["points"][0])
        np.testing.assert_allclose(value[-1], self.links[0]["points"][-1])

    def test_builds_three_parallel_boundaries(self):
        document = build_shadow_lane_pseudo_gt(
            self.links, self.scenario, self.config
        )
        self.assertEqual(document["schema_version"], SCHEMA_VERSION)
        self.assertFalse(document["deployment_allowed"])
        self.assertEqual(document["status"], "map_pseudo_gt")
        yellow = np.asarray(document["boundaries"]["yellow_center"])
        positive = np.asarray(document["boundaries"]["white_positive_normal"])
        negative = np.asarray(document["boundaries"]["white_negative_normal"])
        np.testing.assert_allclose(yellow[:, 1], 0.0, atol=1e-9)
        np.testing.assert_allclose(
            np.linalg.norm(positive[:, :2] - yellow[:, :2], axis=1), 3.5
        )
        np.testing.assert_allclose(
            np.linalg.norm(negative[:, :2] - yellow[:, :2], axis=1), 3.5
        )
        self.assertAlmostEqual(
            document["alignment"]["center_separation_m"]["median"], 3.5
        )

    def test_applies_measured_center_offset_and_asymmetric_widths(self):
        self.config["geometry"].update(
            {
                "center_lateral_offset_m": -0.18,
                "positive_lane_width_m": 3.47,
                "negative_lane_width_m": 3.49,
                "calibration": {"method": "synthetic_test"},
            }
        )
        document = build_shadow_lane_pseudo_gt(
            self.links, self.scenario, self.config
        )
        yellow = np.asarray(document["boundaries"]["yellow_center"])
        positive = np.asarray(document["boundaries"]["white_positive_normal"])
        negative = np.asarray(document["boundaries"]["white_negative_normal"])
        np.testing.assert_allclose(yellow[:, 1], -0.18, atol=1e-9)
        np.testing.assert_allclose(
            np.linalg.norm(positive[:, :2] - yellow[:, :2], axis=1), 3.47
        )
        np.testing.assert_allclose(
            np.linalg.norm(negative[:, :2] - yellow[:, :2], axis=1), 3.49
        )
        self.assertEqual(
            "pending_camera_review_after_lidar_calibration",
            document["validation"]["review_status"],
        )

    def test_preserves_bounded_pose_heading_alignment(self):
        self.config["pose_alignment"] = {
            "ego_heading_offset_deg": -0.5,
            "calibration": {"method": "synthetic_heading_grid"},
        }
        document = build_shadow_lane_pseudo_gt(
            self.links, self.scenario, self.config
        )
        self.assertAlmostEqual(
            -0.5, document["pose_alignment"]["ego_heading_offset_deg"]
        )
        self.assertEqual(
            "synthetic_heading_grid",
            document["pose_alignment"]["calibration"]["method"],
        )

    def test_rejects_unbounded_pose_heading_alignment(self):
        self.config["pose_alignment"] = {"ego_heading_offset_deg": 5.1}
        with self.assertRaisesRegex(ValueError, "heading offset"):
            build_shadow_lane_pseudo_gt(self.links, self.scenario, self.config)

    def test_rotated_rectangle_membership(self):
        area = dict(self.scenario["shadedAreaList"][0])
        area["rot"] = {"yaw": 90.0}
        inside = points_in_rotated_rectangle([[0.0, 4.0], [0.0, -4.0]], area)
        outside = points_in_rotated_rectangle([[4.0, 0.0]], area)
        self.assertTrue(inside.all())
        self.assertFalse(outside.any())

    def test_rejects_wrong_lane_width_contract(self):
        self.config["geometry"]["expected_lane_width_m"] = 2.5
        with self.assertRaisesRegex(ValueError, "lane-width prior"):
            build_shadow_lane_pseudo_gt(self.links, self.scenario, self.config)


if __name__ == "__main__":
    unittest.main()
