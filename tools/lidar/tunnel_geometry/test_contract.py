import unittest
import os
from pathlib import Path
import numpy as np
if __package__:
    from .evidence import camera_evidence
    from .metrics import score, promotion_gate
    from .git_evidence import git_output
else:
    from evidence import camera_evidence
    from metrics import score, promotion_gate
    from git_evidence import git_output


def decision(boxes=(), selected=True, **kw):
    return {"selected_yolo_index": 1 if selected else None, "boxes": list(boxes),
            "selected_yolo_stamp_ns": 10, "selected_yolo_arrival_ns": 11,
            "decision_ns": 12, "reason": "matched_boxes" if boxes else "matched_empty",
            "protection_enabled": True, **kw}


class EvidenceTest(unittest.TestCase):
    def test_empty_unsupported_missing_and_off_remain_distinct(self):
        unknown = {"class": "cone", "xmin": 0, "ymin": 0, "xmax": 3, "ymax": 4}
        values = [decision(), decision([unknown]), decision(selected=False, reason="missing_yolo"),
                  decision(protection_enabled=False)]
        self.assertEqual([camera_evidence(x)["state"] for x in values],
                         ["valid_empty", "unsupported_only", "missing", "protection_off"])
        self.assertTrue(all(not camera_evidence(x)["absence_is_free_space"] for x in values))

    def test_future_arrival_is_rejected(self):
        with self.assertRaises(ValueError):
            camera_evidence(decision(selected_yolo_arrival_ns=13))

    def test_selected_does_not_imply_actual_point_protection(self):
        box = {"class": "car", "xmin": 0, "ymin": 0, "xmax": 0, "ymax": 4}
        result = camera_evidence(decision([box]))
        self.assertEqual(result["state"], "invalid_boxes")
        self.assertEqual(result["invalid_box_count"], 1)
        self.assertNotIn("protected_points", result)

    def test_stale_and_bad_stamp(self):
        self.assertEqual(camera_evidence(decision(selected=False, unmatched_cause="outside_50ms"))["state"],
                         "outside_time_window")
        self.assertEqual(camera_evidence(decision(selected_yolo_stamp_ns=0))["state"], "invalid_stamp")


class MetricsTest(unittest.TestCase):
    def test_upstream_loss_is_separate_and_wall_merge_fails_purity(self):
        mode = {"labels": [2, 0, 0, 0, 0], "cluster_id": [-1, 0, 0, 0, 0],
                "objects": [{"removed": False, "center": [0, 0, 0],
                             "dimensions": [1, 1, 1], "yaw": 0}]}
        result = score(mode, [3, 3, 3, 1, 1], [2, 0, 0, 0, 0])
        self.assertEqual(result["wall_stage_target_points"], 2)
        self.assertEqual(result["target_removed_upstream"], 1)
        self.assertEqual(result["target_preservation"], 1)
        self.assertFalse(result["accepted_target"])

    def test_removed_obb_is_not_a_detection(self):
        mode = {"labels": [0, 0], "cluster_id": [0, 0],
                "objects": [{"removed": True, "center": [0, 0, 0],
                             "dimensions": [1, 1, 1], "yaw": 0}]}
        result = score(mode, [3, 3], [0, 0])
        self.assertIsNone(result["final_target"])
        self.assertEqual(len(result["target_clusters"]), 1)

    def test_mismatched_lineage_is_rejected(self):
        with self.assertRaises(ValueError):
            score({"labels": [0], "cluster_id": [0], "objects": []}, [3, 3], [0, 0])

    def test_missing_required_case_cannot_pass(self):
        self.assertFalse(promotion_gate({}, 0)["passed"])


class ProvenanceTest(unittest.TestCase):
    def test_readonly_revision_preserves_environment(self):
        environment = dict(os.environ)
        repo = Path(__file__).resolve().parents[3]
        revision = git_output(repo, ["rev-parse", "HEAD"]).strip()
        self.assertRegex(revision, r"^[0-9a-f]{40}$")
        self.assertEqual(dict(os.environ), environment)


if __name__ == "__main__":
    unittest.main()
