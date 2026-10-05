import unittest
from analyze_l1b4_fragments import supported_distance, summarize


class FragmentDiagnosticsTest(unittest.TestCase):
    def test_support_is_not_extended_and_removed_objects_are_not_counted(self):
        model = {"plane": [0, -1, 0, 3], "intervals": [[-2, 2]]}
        self.assertAlmostEqual(supported_distance([0, 2.9, 0], [model]), .1)
        self.assertIsNone(supported_distance([3, 2.9, 0], [model]))
        obj = {"center": [0, 2.9, 0], "dimensions": [1, .1, 2], "removed": False,
               "protected": False, "flags": {"wall_attached": True}}
        rows = [{"measurement": True, "scene": "shadow_test", "input_id": "scan",
                 "wall_models": [model], "geometry": {"objects": [obj, dict(obj, removed=True)]}}]
        result = summarize(rows)
        self.assertEqual(result["counts"]["final_objects"], 1)
        self.assertEqual(result["counts"]["center_within_0p3m"], 1)
        self.assertEqual(result["flags"]["wall_attached"], 1)

    def test_empty_and_inactive_are_not_phantom_detections(self):
        result = summarize([{"measurement": True, "scene": "shadow_test", "skipped_reason": "inactive"}])
        self.assertEqual(result["dimension_p50_p95_m"], {})
        self.assertEqual(result["counts"].get("final_objects", 0), 0)
