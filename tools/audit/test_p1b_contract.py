#!/usr/bin/env python3

import sys
import unittest
from dataclasses import replace
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "tools" / "audit"))

from validate_p1b_contract import (  # noqa: E402
    ContractError,
    Field,
    all_message_md5,
    load_message_specs,
    ros1_md5,
    validate_contract,
)


class P1BContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.message_dir = REPO_ROOT / "src" / "asmc_msgs" / "msg"
        cls.contract = REPO_ROOT / "config" / "contracts" / "hd_map_topic_contract_v1.yaml"
        cls.baseline = REPO_ROOT / "config" / "baselines" / "runtime_baseline_v1.yaml"
        cls.specs = load_message_specs(cls.message_dir)

    def test_repository_contract_is_valid(self):
        result = validate_contract(REPO_ROOT, self.contract, self.baseline)
        self.assertEqual(result["message_count"], 28)
        self.assertEqual(result["contract_message_count"], 25)
        self.assertEqual(result["registered_message_count"], 28)
        self.assertEqual(result["legacy_message_count"], 13)
        self.assertEqual(result["new_message_count"], 12)
        self.assertEqual(result["topic_count"], 11)

    def test_legacy_ros_md5_compatibility(self):
        md5s = all_message_md5(self.specs)
        self.assertEqual(md5s["BehaviorCommand"], "fad196232c8f54cbbe2a1bced9d3972a")
        self.assertEqual(md5s["DetectedObjectArray"], "14fbd9679c2d87ae7498e4269d9678da")
        self.assertEqual(md5s["LaneBoundaryFamily"], "70ac79f98ca1244c782ea4067becd6af")

    def test_new_message_ros_md5_is_frozen(self):
        md5s = all_message_md5(self.specs)
        self.assertEqual(md5s["HdMapMeta"], "adb0d98c5dbd41055ebb929fcdf3af19")
        self.assertEqual(md5s["LaneMarkingArray"], "2eae16c9d4bf3385ac35eba4b3d1b823")
        self.assertEqual(md5s["LaneArray"], "95ad93ef58c8ff22f4069b8ca2388395")
        self.assertEqual(md5s["LaneAssociation"], "fac864677f1248f439252de3e651536d")
        self.assertEqual(md5s["MissionZoneArray"], "04325427bb7aedffd9a6ec4881ed46ce")

    def test_md5_changes_when_a_legacy_field_changes(self):
        original = self.specs["VehicleState"]
        changed = replace(original, fields=original.fields + (Field("float64", "compatibility_break"),))
        specs = dict(self.specs)
        specs["VehicleState"] = changed
        self.assertNotEqual(
            ros1_md5("VehicleState", specs),
            "7b5b81807cda33ae90638895dcf749f1",
        )

    def test_unknown_message_dependency_fails_closed(self):
        original = self.specs["HdMapMeta"]
        changed = replace(original, fields=original.fields + (Field("missing_pkg/Missing", "bad"),))
        specs = dict(self.specs)
        specs["HdMapMeta"] = changed
        with self.assertRaises(ContractError):
            ros1_md5("HdMapMeta", specs)


if __name__ == "__main__":
    unittest.main()
