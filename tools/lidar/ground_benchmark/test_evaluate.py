import importlib.util
from pathlib import Path
import unittest
import numpy as np

SPEC = importlib.util.spec_from_file_location("ground_evaluate", Path(__file__).with_name("evaluate.py"))
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class GroundEvaluationTest(unittest.TestCase):
    def test_unknown_never_inflates_ground_or_obstacle_metrics(self):
        result = MODULE.counts(np.array([127, 255, 125, 109, 153, 10, 0]),
                               np.array([1, 2, 2, 3, 2, 1, 1]))
        self.assertEqual(result["ground_points"], 2)
        self.assertEqual(result["ground_removed"], 1)
        self.assertEqual(result["obstacle_points"], 2)
        self.assertEqual(result["obstacle_retained"], 1)
        self.assertEqual(result["obstacle_structure_removed"], 1)
        self.assertEqual(result["excluded_unknown_points"], 2)

    def test_roi_rejects_sentinel_and_nonfinite(self):
        points = np.array([[0, 0, .04, 0], [10, 0, -1.45, 20],
                           [10, 0, np.nan, 20], [10, 21, 0, 20], [10, 0, 4, 20]])
        self.assertEqual(MODULE.roi_mask(points).tolist(), [False, True, False, False, False])

    def test_missing_manifest_and_rejected_runs_fail_closed(self):
        sample = {"frame": "accepted_000001", "xyz_exactly_equal": True,
                  "manifest": {"run": "accepted", "success": True, "ego": True}}
        inventory = {"runs": {"accepted": {"manifest_frame_set_equal": True}}}
        self.assertTrue(MODULE.admitted(sample, inventory, set()))
        self.assertFalse(MODULE.admitted(sample, inventory, {"accepted"}))
        sample["manifest"] = None
        self.assertFalse(MODULE.admitted(sample, inventory, set()))

    def test_incomplete_prediction_fails(self):
        with self.assertRaises(ValueError):
            MODULE.counts(np.array([127, 125]), np.array([1]))
        with self.assertRaises(ValueError):
            MODULE.counts(np.array([127]), np.array([9]))


if __name__ == "__main__":
    unittest.main()
