import unittest

from asmc_hd_map.review_context import LaneReviewIndex


class ReviewContextTest(unittest.TestCase):
    def test_nearest_lane_reports_width_boundaries_and_stopline(self):
        document = {"features": [
            {
                "id": "map:lane_boundary:L:left", "feature_type": "lane_boundary",
                "attributes": {"lane_color": "yellow", "pattern": "solid", "crossing_rule": "prohibited", "observed_fraction": 1.0},
            },
            {
                "id": "map:lane_boundary:L:right", "feature_type": "lane_boundary",
                "attributes": {"lane_color": "white", "pattern": "broken", "crossing_rule": "permitted", "observed_fraction": 1.0},
            },
            {
                "id": "map:lane_area:L", "feature_type": "lane_area",
                "attributes": {
                    "lane_id": "L", "centerline": [[0, 0, 0], [10, 0, 0]],
                    "width_profile": [{"station_m": 0, "width_m": 3.0}, {"station_m": 10, "width_m": 4.0}],
                    "left_boundary_id": "map:lane_boundary:L:left",
                    "right_boundary_id": "map:lane_boundary:L:right",
                    "geometry_quality": "direct_observed", "route_maneuver_roles": ["official_route"],
                    "route_s_min_m": 100.0, "route_s_max_m": 110.0,
                },
            },
            {
                "id": "map:stopline:N", "feature_type": "stopline",
                "attributes": {"route_s_m": 108.0, "node_ids": ["N"]},
            },
        ]}
        result = LaneReviewIndex(document).query(5.0, 0.5, 0.0)
        self.assertIsNotNone(result)
        self.assertEqual(result["lane_id"], "L")
        self.assertAlmostEqual(result["lane_width_m"], 3.5)
        self.assertAlmostEqual(result["route_s_m"], 105.0)
        self.assertEqual(result["left_boundary"]["crossing_rule"], "prohibited")
        self.assertEqual(result["right_boundary"]["crossing_rule"], "permitted")
        self.assertEqual(result["next_stopline"]["node_id"], "N")
        self.assertAlmostEqual(result["next_stopline"]["route_distance_m"], 3.0)


if __name__ == "__main__":
    unittest.main()
