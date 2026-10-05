import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

if __package__:
    from .admit import aligned_clouds, attitude, boxes, camera_fixture, projection_check, SIGNS
    from .score import mixed_voxels, object_metrics
else:
    from admit import aligned_clouds, attitude, boxes, camera_fixture, projection_check, SIGNS
    from score import mixed_voxels, object_metrics


class AdmissionTest(unittest.TestCase):
    def test_aligned_gt_and_intensity_are_separate(self):
        with tempfile.TemporaryDirectory() as tmp:
            paths = [Path(tmp) / (str(k) + ".bin") for k in range(3)]
            for k, path in enumerate(paths):
                np.array([[2, 3, 4, 255 if k == 0 else k]], dtype="<f4").tofile(path)
            self.assertEqual(aligned_clouds(paths)[0][0, 3], 255)

    def test_shifted_gt_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            paths = [Path(tmp) / (str(k) + ".bin") for k in range(3)]
            for k, path in enumerate(paths):
                np.array([[2 + k, 3, 4, 1]], dtype="<f4").tofile(path)
            with self.assertRaises(ValueError):
                aligned_clouds(paths)

    def test_noninteger_labels_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            paths = [Path(tmp) / (str(k) + ".bin") for k in range(3)]
            for k, path in enumerate(paths):
                np.array([[2, 3, 4, .2]], dtype="<f4").tofile(path)
            with self.assertRaises(ValueError):
                aligned_clouds(paths)

    def test_native_stamp_bug_does_not_affect_attitude_or_become_ros_time(self):
        imu = {"Orientation X": 0, "Orientation Y": 0, "Orientation Z": 0, "Orientation W": 1,
               "TimeStamp.Secs": 100, "TimeStamp.nSecs": 2_888_976_704}
        self.assertEqual(attitude(imu), (0, 0, 0))
        self.assertEqual(imu["TimeStamp.nSecs"], 2_888_976_704)

    def test_missing_bbox_file_is_not_empty_detector_evidence(self):
        with tempfile.TemporaryDirectory() as tmp:
            r = projection_check(Path(tmp) / "missing.txt", [], {})
            self.assertFalse(r[0]["file_available"])
            self.assertEqual(r[0]["camera_detection_state"], "unverified")

    def test_invalid_quaternion_rejected(self):
        with self.assertRaises(ValueError):
            attitude({"Orientation " + a: 0 for a in "XYZW"})

    def test_no_gt_boxes_in_camera_fixture(self):
        for state in ("empty", "missing"):
            result = camera_fixture(state, 1_790_000_000_000_000_003)
            self.assertEqual(result["boxes"], [])
            self.assertEqual(result["wait_ns"], 0)
            self.assertTrue(result["fixture_only_not_recorded_yolo"])
            self.assertEqual(json.loads(json.dumps(result))["scan_stamp_ns"], 1_790_000_000_000_000_003)
        self.assertIsNone(camera_fixture("missing", 3)["selected_yolo_index"])
        self.assertIsNotNone(camera_fixture("empty", 3)["selected_yolo_index"])

    def test_inconsistent_box_is_flagged_not_deleted_from_point_truth(self):
        with tempfile.TemporaryDirectory() as tmp:
            basic, corner = Path(tmp) / "boxes.txt", Path(tmp) / "corners.txt"
            basic.write_text("Obstacle 0 0 0 0 0 0 2 2 2 0 0 0 0 4\n")
            values = list(SIGNS.ravel()) + [0, 0, 0, 0, 0, 0, 0, 4]
            corner.write_text("Obstacle " + " ".join(map(str, values)) + "\n")
            gt = boxes(basic, corner, np.array([[2, 0, 0]]), np.array([4]))[0]
            self.assertFalse(gt["box_accuracy_admitted"])
            self.assertEqual(gt["outside_box_3cm"], 1)
            self.assertEqual(gt["visible_points"], 1)


class RawLineageMetricsTest(unittest.TestCase):
    @staticmethod
    def mode(labels, clusters, removed=False):
        return {"labels": labels, "cluster_id": clusters,
                "objects": [{"removed": removed, "center": [0, 0, 0], "dimensions": [1, 1, 1]}]}

    def test_mixed_voxel_contamination_cannot_be_majority_voted_away(self):
        ids = np.array([4, 4, 0])
        mapping = np.array([0, 0, 0])
        s = object_metrics(4, ids, mapping, [0], self.mode([0], [0]))
        self.assertEqual(s["wall_stage_preservation"], 1)
        self.assertEqual(s["final_target"]["completeness"], 1)
        self.assertAlmostEqual(s["final_target"]["purity"], 2 / 3)
        self.assertFalse(s["accepted_target"])
        self.assertEqual(mixed_voxels(ids, mapping)["target_raw_points_in_mixed_voxels"], 2)

    def test_roi_ground_wall_denominators_are_distinct(self):
        s = object_metrics(4, [4] * 4, [-1, 0, 1, 2], [2, 0, 0],
                           self.mode([2, 3, 0], [-1, -1, 0]))
        self.assertEqual(s["roi_excluded"], 1)
        self.assertEqual(s["upstream_ground_removed"], 1)
        self.assertEqual(s["wall_stage_target_points"], 2)
        self.assertEqual(s["wall_stage_preservation"], .5)
        self.assertEqual(s["wall_stage_removed"], 1)

    def test_rejected_obb_is_not_final_detection(self):
        s = object_metrics(4, [4], [0], [0], self.mode([0], [0], removed=True))
        self.assertEqual(s["wall_stage_kept"], 1)
        self.assertIsNone(s["final_target"])
        self.assertEqual(s["failure_reason"], "final_obb_rejected")

    def test_kept_but_unclustered_is_not_detection(self):
        s = object_metrics(4, [4], [0], [0], self.mode([0], [-1]))
        self.assertEqual(s["failure_reason"], "kept_but_no_cluster")
        self.assertFalse(s["sufficient_target_voxels_for_10_point_cluster"])

    def test_mapping_out_of_range_rejected(self):
        with self.assertRaises(ValueError):
            object_metrics(4, [4], [1], [0], self.mode([0], [0]))


if __name__ == "__main__":
    unittest.main()
