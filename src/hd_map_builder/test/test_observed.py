import unittest

import numpy as np

from hd_map_builder.observed import (
    PolylineProjectionIndex,
    Projection,
    associate_projection_sequence,
    classify_observation_row,
    interpolate_profile,
    robust_station_profile,
    split_ordered_observations,
)


class ObservedGeometryTest(unittest.TestCase):
    def test_generated_and_interpolated_are_never_direct(self):
        self.assertEqual(classify_observation_row({"generated": "1"}), "generated")
        self.assertEqual(classify_observation_row({"interpolated": "1.0"}), "interpolated")
        self.assertEqual(classify_observation_row({"generated": "0", "interpolated": "0"}), "direct")

    def test_split_on_non_monotonic_time_time_gap_and_space_gap(self):
        rows = [
            {"timestamp": value[0], "lane_map_x": value[1], "lane_map_y": 0.0}
            for value in [(0.0, 0.0), (0.1, 0.1), (0.1, 0.2), (1.0, 0.3), (1.1, 3.0)]
        ]
        groups = split_ordered_observations(rows, max_time_gap_sec=0.5, max_point_gap_m=1.5)
        self.assertEqual([len(group) for group in groups], [2, 1, 1, 1])

    def test_signed_exact_projection_recovers_straight_offset(self):
        index = PolylineProjectionIndex({"boundary": [[0, 0, 0], [20, 0, 0]]})
        for x in (1.0, 10.0, 19.0):
            match = index.query([x, 0.4], [-1.0, 0.0], max_distance_m=2.0, max_heading_deg=30.0)[0]
            self.assertAlmostEqual(match.station_m, x, places=6)
            self.assertAlmostEqual(match.signed_offset_m, 0.4, places=6)
            self.assertAlmostEqual(match.distance_m, 0.4, places=6)

    def test_curved_projection_recovers_offset_within_five_centimeters(self):
        angles = np.linspace(0.0, np.pi / 2.0, 101)
        radius = 20.0
        curve = np.column_stack((radius * np.cos(angles), radius * np.sin(angles), np.zeros(len(angles))))
        index = PolylineProjectionIndex({"curve": curve})
        errors = []
        for angle in angles[5:-5:10]:
            point = [(radius + 0.35) * np.cos(angle), (radius + 0.35) * np.sin(angle)]
            tangent = [-np.sin(angle), np.cos(angle)]
            match = index.query(point, tangent, max_distance_m=1.0, max_heading_deg=10.0)[0]
            errors.append(abs(match.distance_m - 0.35))
        self.assertLessEqual(max(errors), 0.05)

    def test_unrelated_boundary_transition_is_not_selected(self):
        a = Projection("a", 0.0, 0.0, 0.0, 0.0, (0.0, 0.0, 0.0))
        b = Projection("b", 0.0, 0.0, 0.0, 0.0, (0.0, 0.0, 0.0))
        result = associate_projection_sequence(
            [[a], [b]], set(), search_distance_m=2.0, max_heading_deg=30.0,
            connected_transition_cost=0.25,
        )
        self.assertFalse(result[0] is not None and result[1] is not None)

    def test_connected_boundary_transition_is_allowed(self):
        a = Projection("a", 0.0, 0.0, 0.0, 0.0, (0.0, 0.0, 0.0))
        b = Projection("b", 0.0, 0.0, 0.0, 0.0, (0.0, 0.0, 0.0))
        result = associate_projection_sequence(
            [[a], [b]], {frozenset(("a", "b"))}, search_distance_m=2.0,
            max_heading_deg=30.0, connected_transition_cost=0.25,
        )
        self.assertEqual([item.owner_id for item in result], ["a", "b"])

    def test_robust_profile_rejects_outlier_without_extrapolation(self):
        stations = np.linspace(0.0, 10.0, 101)
        offsets = np.full(101, 0.3)
        offsets[50] = 2.0
        profile, inlier = robust_station_profile(
            stations, offsets, bin_size_m=1.0, mad_multiplier=3.0,
        )
        self.assertFalse(inlier[50])
        self.assertAlmostEqual(interpolate_profile(profile, 5.0, 2.0), 0.3, places=6)
        self.assertIsNone(interpolate_profile(profile, -0.1, 2.0))
        self.assertIsNone(interpolate_profile(profile, 10.1, 2.0))


if __name__ == "__main__":
    unittest.main()
