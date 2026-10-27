#!/usr/bin/env python3
"""Small CPU-only provenance regression; no private bags, GPU, ROS or downloads."""
import copy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

import review_lidar_research as review


class ResearchReviewTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.names = ["config/baselines/evidence/test/summary.json",
                      "config/baselines/evidence/test/index.html"]
        entries = []
        for name, data in zip(self.names, [b'{"historical":true}', b"historical gallery"]):
            path = self.root / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(data)
            entries.append({"path": name, "bytes": len(data),
                            "sha256": hashlib.sha256(data).hexdigest()})
        self.manifest = {"schema_version": 1, "groups": [
            {"id": "fixture", "summary": self.names[0], "gallery": self.names[1], "files": entries}]}

    def test_small_fixture_and_no_writes(self):
        before = {p: p.read_bytes() for p in self.root.rglob("*") if p.is_file()}
        self.assertEqual(review.verify(self.root, self.manifest), {"fixture": {"historical": True}})
        self.assertEqual(before, {p: p.read_bytes() for p in self.root.rglob("*") if p.is_file()})

    def test_corruption_is_failure(self):
        (self.root / self.names[1]).write_bytes(b"different gallery!")
        with self.assertRaisesRegex(ValueError, "changed"):
            review.verify(self.root, self.manifest)

    def test_missing_file_is_failure(self):
        (self.root / self.names[1]).unlink()
        with self.assertRaises(FileNotFoundError):
            review.verify(self.root, self.manifest)

    def test_duplicate_file_and_uncovered_summary_are_failure(self):
        group = self.manifest["groups"][0]
        group["files"].append(copy.deepcopy(group["files"][0]))
        with self.assertRaisesRegex(ValueError, "Duplicate"):
            review.verify(self.root, self.manifest)
        group["files"].pop()
        group["summary"] = "config/baselines/evidence/test/unlisted.json"
        with self.assertRaisesRegex(ValueError, "not covered"):
            review.verify(self.root, self.manifest)

    def test_outside_paths_and_symlinks_are_failure(self):
        for path in ["/etc/passwd", "config/baselines/evidence/../../outside"]:
            with self.assertRaises(ValueError):
                review.evidence_path(self.root, path)
        escaped = self.root / "outside"
        escaped.write_text("outside")
        link = self.root / "config/baselines/evidence/test/escape"
        link.symlink_to(escaped)
        with self.assertRaisesRegex(ValueError, "escapes"):
            review.evidence_path(self.root, str(link.relative_to(self.root)))

    def test_imported_report_keeps_limitations(self):
        manifest = json.loads(review.DEFAULT_MANIFEST.read_text())
        report = review.build_report(manifest, review.verify(review.ROOT, manifest))
        self.assertFalse(report["runtime_acceptance"])
        self.assertFalse(report["independent_accuracy_gt"])
        self.assertTrue(report["limitations"]["learned"])
        self.assertEqual(report["segmentation_label_evidence"]["sample_inspection_only"]
                         ["sampled_train_annotations"], 48)
        self.assertEqual(report["verified_files"], 15)


if __name__ == "__main__":
    unittest.main()
