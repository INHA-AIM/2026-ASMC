import unittest
from tools.perception.analyze_fusion_identity import compare, compare_stable_observations
from tools.perception.analyze_fusion_trace import summarize


def frame(objects):
    return {"stamp_ns": 100, "objects": [[key, label, label] for key, label, center in objects],
            "centers": {str(key): center for key, label, center in objects}}


class FusionDiagnosticsTest(unittest.TestCase):
    def test_reassigned_ids_are_compared_by_mutually_unique_geometry(self):
        a = [frame([(1, "car", [0, 0, 0]), (2, "person", [3, 0, 0])])]
        b = [frame([(22, "car", [0, 0, 0]), (21, "person", [3, 0, 0])])]
        report = compare(a, b)
        self.assertEqual(2, report["matched_observations"])
        self.assertEqual(2, report["reassigned_ids"])
        self.assertEqual([], report["classification_differences"])

    def test_two_reference_clusters_cannot_claim_one_target(self):
        a = [frame([(1, "car", [0, 0, 0]), (2, "car", [.0005, 0, 0])])]
        b = [frame([(7, "car", [0, 0, 0])])]
        report = compare(a, b)
        self.assertEqual(0, report["matched_observations"])
        self.assertEqual(2, report["unmatched_observations"])
        self.assertEqual(1, report["unmatched_candidate_observations"])

    def test_stable_old_class_change_is_reported_despite_changed_id(self):
        a = [frame([(1, "car", [0, 0, 0])])]
        b = [frame([(8, "car", [0, 0, 0])])]
        new = [frame([(9, "unknown", [0, 0, 0])])]
        report = compare_stable_observations([a, b], [new, a])
        self.assertEqual(1, report["old_stable"])
        self.assertEqual(1, len(report["regressions"]))

    def test_duplicate_stamps_are_not_silently_overwritten(self):
        row = frame([(1, "car", [0, 0, 0])])
        with self.assertRaises(ValueError):
            compare([row, row], [row])

    def test_complete_trace_counts_final_partial_interval_and_decode_drop(self):
        rows = [{"event": "match_decision", "action": "match", "stamp_ns": 100,
                 "selected_yolo_stamp_ns": 110, "offset_ns": 10},
                {"event": "camera_decode_drop"},
                {"event": "fusion_summary", "matched": 0, "dropped": 1}]
        report = summarize(rows)
        self.assertTrue(report["complete"])
        self.assertTrue(report["accounting_consistent"])
        self.assertEqual(0, report["duplicate_consumed_yolo_stamps"])

    def test_missing_final_summary_is_incomplete(self):
        report = summarize([])
        self.assertFalse(report["complete"])
        self.assertFalse(report["accounting_consistent"])


if __name__ == "__main__":
    unittest.main()
