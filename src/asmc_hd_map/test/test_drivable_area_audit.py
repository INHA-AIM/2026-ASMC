import tempfile
import unittest
from pathlib import Path

import numpy as np

from asmc_hd_map.drivable_area_audit import (
    bilateral_boundary_coverage,
    complement_intervals,
    merge_intervals,
    render_report,
)


class DrivableAreaAuditUnitTest(unittest.TestCase):
    def test_boundary_coverage_requires_both_sides(self):
        center = np.array([[0.0, 0.0], [10.0, 0.0]])
        left = np.array([[0.0, 2.0], [10.0, 2.0]])
        right = np.array([[0.0, -2.0], [10.0, -2.0]])
        both = bilateral_boundary_coverage(
            center,
            [left, right],
            sample_step_m=1.0,
            marking_sample_step_m=0.5,
            minimum_lateral_distance_m=0.4,
            maximum_boundary_distance_m=8.0,
        )
        one = bilateral_boundary_coverage(
            center,
            [left],
            sample_step_m=1.0,
            marking_sample_step_m=0.5,
            minimum_lateral_distance_m=0.4,
            maximum_boundary_distance_m=8.0,
        )
        self.assertEqual(both["bilateral_coverage"], 1.0)
        self.assertEqual(one["left_coverage"], 1.0)
        self.assertEqual(one["right_coverage"], 0.0)
        self.assertEqual(one["bilateral_coverage"], 0.0)

    def test_guard_intervals_merge_reasons_and_cover_short_remainder(self):
        guards = merge_intervals(
            [(0.0, 4.0, "branch"), (3.0, 8.0, "intersection")], 10.0
        )
        candidates, guards = complement_intervals(guards, 10.0, 5.0)
        self.assertEqual(candidates, [])
        self.assertEqual(guards[0]["start_m"], 0.0)
        self.assertEqual(guards[0]["end_m"], 10.0)
        self.assertIn("clip_interval_below_minimum", guards[0]["reasons"])

    def test_sub_micrometre_rounding_tail_is_not_a_clip_candidate(self):
        candidates, _ = complement_intervals(
            [{"start_m": 0.0, "end_m": 22.325277, "reasons": ["intersection"]}],
            22.325277208,
            5.0,
        )
        self.assertEqual(candidates, [])

    def test_report_states_no_geometry_change(self):
        document = {
            "route_summary": {
                "route_length_m": 10.0,
                "guarded_length_m": 4.0,
                "lane_clip_candidate_length_m": 6.0,
            },
            "legacy_drivable_prior": {
                "road_area_m2": 20.0,
                "checkpoint_centers_inside": 1,
                "checkpoint_count": 1,
            },
            "reviewed_hd_map": {"geometry_sha256": "abc"},
            "route_links": [],
        }
        report = render_report(document)
        self.assertIn("geometry/publisher 변경 없음", report)
        self.assertIn("아직 road를 잘라낸 결과가 아니다", report)


if __name__ == "__main__":
    unittest.main()
