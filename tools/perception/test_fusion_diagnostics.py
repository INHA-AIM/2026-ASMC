import unittest
from tools.perception.analyze_fusion_identity import compare, compare_stable_observations
from tools.perception.analyze_fusion_trace import summarize, compare_boxes, object_state, publication_state
from tools.perception.fusion_log_accounting import parse_fusion_logs


def frame(objects):
    return {"stamp_ns": 100, "objects": [[key, label, label] for key, label, center in objects],
            "centers": {str(key): center for key, label, center in objects}}


class FusionDiagnosticsTest(unittest.TestCase):
    def test_publisher_input_retains_source_provenance_without_current_update(self):
        rows = [{'event': 'class_publish_input', 'stamp_ns': 200000000, 'generation': 3,
                 'objects': [{'id': 7, 'class': 'unknown'}],
                 'origins': [{'id': 7, 'source_lidar_stamp_ns': 100000000,
                              'updated_in_frame': False}]}]
        report = publication_state(rows, 200000000, 7)
        self.assertEqual(100, report['lidar_state_lag_ms'])
        self.assertFalse(report['origin']['updated_in_frame'])
        self.assertEqual(3, report['generation'])
        with self.assertRaises(ValueError):
            publication_state(rows * 2, 200000000, 7)

    def test_continuous_observation_missing_publication_is_not_complete(self):
        rows = [{'event': 'lidar_process_end', 'stamp_ns': 100,
                 'duration_ms': 1, 'processed': True, 'pose_ready': True},
                {'event': 'class_lidar_ready', 'stamp_ns': 100}]
        self.assertFalse(summarize(rows)['continuous_observation_complete'])
        rows.append({'event': 'class_publish_input', 'stamp_ns': 100, 'origins': []})
        self.assertTrue(summarize(rows)['continuous_observation_complete'])
        rows.append(rows[-1])
        self.assertFalse(summarize(rows)['continuous_observation_complete'])

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

    def test_box_comparison_keeps_missing_frames_and_rejects_duplicates(self):
        row = {"stamp_ns": 100, "frame_id": "camera_front", "objc": 1,
               "boxes": [["car", 1, 2, 3, 4]]}
        extra = dict(row, stamp_ns=200)
        report = compare_boxes([row], [row, extra])
        self.assertEqual(1, report["common_source_frames"])
        self.assertEqual(1, report["new_only_frames"])
        self.assertEqual([], report["different_boxes_or_headers"])
        with self.assertRaises(ValueError):
            compare_boxes([row, row], [row])

    def test_current_state_is_joined_to_the_historical_lidar_stamp(self):
        rows = [{"event": "class_after", "stamp_ns": 100000000,
                 "yolo_stamp_ns": 110000000, "objects": [{"id": 22, "class": "car"}]},
                {"event": "class_current", "stamp_ns": 175000000,
                 "yolo_stamp_ns": 110000000, "objects": [{"id": 22, "class": "car"}]}]
        report = object_state(rows, "class_current", 175000000, 22)
        self.assertEqual(100000000, report[0]["fused_lidar_stamp_ns"])
        self.assertEqual(75, report[0]["lidar_state_lag_ms"])
        self.assertEqual([], object_state(rows, "class_current", 175000000, 23))

    def test_tail_must_reconcile_with_total_including_final_drops(self):
        interval = "Fusion sync 5s: matched=397 dropped=0 offset median/p95/max=18.0/30.0/40.0 ms"
        tail = "Fusion sync tail: matched=104 dropped=1 offset median/p95/max=18.0/31.0/49.0 ms"
        total = "Fusion sync total: matched=501 dropped=1 pending lidar/yolo=1/1"
        rows, count, complete = parse_fusion_logs('\n'.join((interval, tail, total)))
        self.assertTrue(complete)
        self.assertEqual(501, sum(row['matched'] for row in rows))
        self.assertEqual(1, count['dropped'])
        self.assertFalse(parse_fusion_logs('\n'.join((interval, total)))[2])
        self.assertFalse(parse_fusion_logs('\n'.join((interval, tail, total, total)))[2])
        self.assertFalse(parse_fusion_logs('\n'.join((interval, tail, total.replace('501', '502'))))[2])


if __name__ == "__main__":
    unittest.main()
