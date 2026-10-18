#!/usr/bin/env python3

import copy
import sys
import tempfile
import unittest
from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "tools" / "audit"))

from validate_perception_topic_policy import (  # noqa: E402
    PolicyError,
    validate_policy,
)


class PerceptionTopicPolicyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.policy = (
            REPO_ROOT
            / "config"
            / "contracts"
            / "perception_topic_policy_v1.yaml"
        )

    def test_repository_policy_is_valid(self):
        result = validate_policy(REPO_ROOT, self.policy)
        self.assertEqual(33, result["family_count"])
        self.assertEqual(84, result["expanded_topic_count"])
        self.assertEqual(2, result["required_internal_family_count"])
        self.assertEqual(2, result["default_off_candidate_count"])

    def test_required_fusion_transport_cannot_be_called_disposable_debug(self):
        value = yaml.safe_load(self.policy.read_text(encoding="utf-8"))
        broken = copy.deepcopy(value)
        family = next(
            item
            for item in broken["topic_families"]
            if item["id"] == "per_view_projected_semantics"
        )
        family["classification"] = "debug_opt_in"
        family["p5b_action"] = "default_off_p5b"
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "broken.yaml"
            path.write_text(yaml.safe_dump(broken, sort_keys=False), encoding="utf-8")
            with self.assertRaises(PolicyError):
                validate_policy(REPO_ROOT, path)

    def test_bag_profile_drift_fails_closed(self):
        value = yaml.safe_load(self.policy.read_text(encoding="utf-8"))
        broken = copy.deepcopy(value)
        family = next(
            item
            for item in broken["topic_families"]
            if item["id"] == "fused_bev_source_view"
        )
        family["bag_profiles"] = ["camera_semantic_result.yaml"]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "broken.yaml"
            path.write_text(yaml.safe_dump(broken, sort_keys=False), encoding="utf-8")
            with self.assertRaises(PolicyError):
                validate_policy(REPO_ROOT, path)


if __name__ == "__main__":
    unittest.main()
