#!/usr/bin/env python3

import sys
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "tools" / "audit"))

from validate_perception_contract import ContractError, validate_contract  # noqa: E402


class PerceptionContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.contract = (
            REPO_ROOT
            / "config"
            / "contracts"
            / "perception_runtime_topic_contract_v1.yaml"
        )

    def test_repository_contract_is_valid(self):
        result = validate_contract(REPO_ROOT, self.contract)
        self.assertEqual(3, result["message_count"])
        self.assertEqual(3, result["canonical_topic_count"])
        self.assertEqual(4, result["compatibility_topic_count"])
        self.assertEqual(1, result["consumer_binding_count"])
        self.assertEqual(
            "6c96135eb03a3ee93bba474fa835f57d",
            result["message_md5"]["asmc_msgs/TrackedObject"],
        )

    def test_missing_contract_fails_closed(self):
        with self.assertRaises(OSError):
            validate_contract(REPO_ROOT, self.contract.with_name("missing.yaml"))


if __name__ == "__main__":
    unittest.main()
