"""CLI tests; no ROS, simulator, or external data. FRAME_TIME_BUILD is required."""
import copy
import os
from pathlib import Path
import unittest

from run_evaluation import execute, snapshots, check_snapshots, strip_timing


class JsonlContract(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.binary = Path(os.environ["FRAME_TIME_BUILD"]) / "frame_time_eval"

    def one(self, row):
        return execute(self.binary, [row])[0]

    def test_reference_math_and_determinism(self):
        rows = snapshots()
        a, b = execute(self.binary, rows), execute(self.binary, rows)
        self.assertLessEqual(check_snapshots(rows, a), 1e-6)
        self.assertEqual(strip_timing(a), strip_timing(b))

    def test_large_integer_stamp(self):
        row = snapshots()[0]
        self.assertEqual(self.one(row)["stamp_ns"], 1790415998913611401)

    def test_duplicate_and_regressed_are_not_fresh_empty(self):
        row = snapshots()[0]
        old = copy.deepcopy(row)
        old["time"]["stamp_ns"] -= 1
        result = execute(self.binary, [row, row, old])
        self.assertTrue(result[0]["valid"])
        self.assertEqual(result[1]["reason"], "duplicate_stamp")
        self.assertEqual(result[2]["reason"], "regressed_stamp")
        self.assertFalse(result[1]["valid"])
        self.assertFalse(result[2]["valid"])

    def test_zero_stamp(self):
        row = snapshots()[0]
        row["time"]["stamp_ns"] = 0
        self.assertEqual(self.one(row)["reason"], "zero_or_negative_stamp")

    def test_bad_pose_time_preserves_local(self):
        row = snapshots()[0]
        row["pose_samples"][0]["arrival_ns"] = 0
        result = self.one(row)
        self.assertTrue(result["objects"][0]["valid"])
        self.assertIsNone(result["objects"][0]["map"])
        self.assertEqual(result["pose_reason"], "pose_time_invalid")

    def test_nan_pose_preserves_local(self):
        row = snapshots()[0]
        # YAML .nan string is deliberately accepted by yaml-cpp as double NaN;
        # valid JSON cannot contain a numeric NaN literal.
        row["pose_samples"][0]["pose"]["translation"][0] = ".nan"
        result = self.one(row)
        self.assertTrue(result["objects"][0]["valid"])
        self.assertIsNone(result["objects"][0]["map"])

    def test_missing_calibration_fails(self):
        row = snapshots()[0]
        row["calibration_source"] = ""
        self.assertFalse(self.one(row)["objects"][0]["valid"])

    def test_missing_twist_is_unknown_not_zero(self):
        row = next(r for r in snapshots() if r["input_id"] == "base_derivative")
        del row["twist"]
        self.assertIsNone(self.one(row)["objects"][0]["absolute_velocity_base"])

    def test_future_pose_excluded(self):
        row = snapshots()[0]
        row["pose_samples"][0]["arrival_ns"] = row["time"]["processing_ns"] + 1
        result = self.one(row)
        self.assertEqual(result["pose_sample_arrivals"], [])
        self.assertIsNone(result["objects"][0]["map"])


if __name__ == "__main__":
    unittest.main()
