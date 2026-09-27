from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from summarize_teammate_offline import retained_objects, summarize_rows


class OfflineSummaryTest(unittest.TestCase):
    def test_removed_flag_not_wall_candidate_flag_controls_retention(self):
        wall_candidate = [0] * 19
        wall_candidate[15] = True
        removed = [0] * 19
        removed[17] = True
        self.assertEqual(retained_objects([wall_candidate, removed]), 1)

    def test_schema_change_fails_instead_of_inventing_metric(self):
        with self.assertRaises(ValueError):
            retained_objects([[0] * 18])

    def test_inactive_runtime_is_not_counted_as_tunnel_baseline(self):
        row = {"stamp": 10., "rec": 10.001, "gps_invalid": False, "gps_tunnel_latched": False,
               "A": {"ms": [20., 3.], "ransac": 500, "last_lidar": 3, "nogps": [], "replica": 1},
               "B": {"active": False}}
        report = summarize_rows([row])
        self.assertEqual(report["active_tunnel_frames"], 0)
        self.assertNotIn("tunnel", report)
        self.assertEqual(report["integrated_runtime"]["map_output_objects"]["p50"], 3)


if __name__ == "__main__":
    unittest.main()
