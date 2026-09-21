import unittest
from pathlib import Path

import yaml

from asmc_hd_map.drivable_area_audit import (
    build_drivable_area_input_audit,
    render_report,
)


REPO_ROOT = Path(__file__).resolve().parents[3]
SOURCE_PATH = REPO_ROOT / "src/asmc_hd_map/config/kcity2025_drivable_area_audit_source.yaml"
MANIFEST_PATH = REPO_ROOT / "src/asmc_hd_map/config/kcity2025_drivable_area_input_audit.yaml"
REPORT_PATH = REPO_ROOT / "docs/baselines/kcity2025-drivable-area-input-audit-v1.md"


class DrivableAreaAuditWorkspaceTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.document = build_drivable_area_input_audit(REPO_ROOT, SOURCE_PATH)
        cls.by_link = {item["link_id"]: item for item in cls.document["route_links"]}

    def test_frozen_geometry_and_prior_invariants(self):
        self.assertEqual(
            self.document["reviewed_hd_map"]["geometry_sha256"],
            "01ec671fe402baf339f8f6bcfb83a97213d560f7311961c5f072a47d654bcfbf",
        )
        self.assertEqual(self.document["reviewed_hd_map"]["lane_marking_count"], 121)
        self.assertEqual(self.document["reviewed_hd_map"]["stopline_count"], 7)
        self.assertEqual(self.document["legacy_drivable_prior"]["checkpoint_centers_inside"], 16)
        self.assertEqual(self.document["legacy_drivable_prior"]["checkpoint_count"], 16)

    def test_route_partition_is_complete_and_conservative(self):
        summary = self.document["route_summary"]
        self.assertEqual(summary["link_count"], 39)
        self.assertTrue(summary["topologically_continuous"])
        self.assertTrue(summary["full_route_coverage"])
        self.assertAlmostEqual(
            summary["guarded_length_m"] + summary["lane_clip_candidate_length_m"],
            summary["route_length_m"],
            places=5,
        )
        self.assertEqual(
            summary["action_counts"],
            {
                "hybrid_guard_and_clip_candidate": 15,
                "lane_clip_candidate": 6,
                "preserve_prior": 18,
            },
        )
        self.assertEqual(self.by_link["A2256W000222"]["action"], "preserve_prior")
        self.assertEqual(self.by_link["A2256W000153"]["action"], "preserve_prior")
        self.assertEqual(self.by_link["A2256W000451"]["action"], "preserve_prior")
        self.assertEqual(
            self.by_link["A2256W000054"]["action"], "hybrid_guard_and_clip_candidate"
        )
        self.assertEqual(self.by_link["A2256W000448"]["action"], "lane_clip_candidate")

    def test_committed_manifest_and_report_are_deterministic(self):
        committed = yaml.safe_load(MANIFEST_PATH.read_text(encoding="utf-8"))
        self.assertEqual(committed, self.document)
        self.assertEqual(REPORT_PATH.read_text(encoding="utf-8"), render_report(self.document))


if __name__ == "__main__":
    unittest.main()
