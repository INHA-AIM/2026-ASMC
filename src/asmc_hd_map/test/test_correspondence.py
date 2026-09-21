import numpy as np
import unittest

from asmc_hd_map.correspondence import (
    MatchPolicy,
    boolean_intervals,
    coalesce_boolean_runs,
    match_feature,
)
from asmc_hd_map.geometry import resample_polyline
from asmc_hd_map.spatial import GridIndex


def _target_index():
    points, tangents = resample_polyline([[0, 0, 0], [20, 0, 0]], 0.5)
    owners = np.asarray(["current_renamed_link"] * len(points), dtype=object)
    return GridIndex(points, tangents, owners, 2.0)


class CorrespondenceTest(unittest.TestCase):
    def test_correspondence_ignores_identifier_name_and_uses_geometry(self):
        result = match_feature(
            {"idx": "legacy_unrelated_name", "points": [[0, 0, 0], [20, 0, 0]]},
            np.zeros(3), 0.5, _target_index(), MatchPolicy(),
        )
        self.assertEqual(result["tier"], "A")
        self.assertEqual(result["targets"][0]["id"], "current_renamed_link")
        self.assertEqual(result["strict_coverage"], 1.0)

    def test_one_lane_width_shift_is_not_promoted(self):
        result = match_feature(
            {"idx": "legacy_adjacent", "points": [[0, 3.5, 0], [20, 3.5, 0]]},
            np.zeros(3), 0.5, _target_index(), MatchPolicy(),
        )
        self.assertEqual(result["tier"], "D")
        self.assertEqual(result["targets"], [])

    def test_route_run_cleanup_and_intervals_use_metric_station(self):
        points = np.column_stack((np.arange(12.0), np.zeros(12), np.zeros(12)))
        values = np.asarray([1, 1, 1, 0, 0, 1, 1, 1, 1, 0, 0, 0], dtype=bool)
        station = np.arange(12.0)
        cleaned = coalesce_boolean_runs(values, station, close_false_gap_m=2.0, remove_true_island_m=0.0)
        self.assertTrue(cleaned[:9].all())
        intervals = boolean_intervals(cleaned, points)
        self.assertEqual(intervals[0]["status"], "reuse")
        self.assertEqual(intervals[0]["end_s_m"], 8.0)
        self.assertEqual(intervals[1]["status"], "rebuild")


if __name__ == "__main__":
    unittest.main()
