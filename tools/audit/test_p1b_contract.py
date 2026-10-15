#!/usr/bin/env python3

import sys
import unittest
import yaml
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
    validate_lidar_object_v2,
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
        self.assertEqual(result["message_count"], 37)
        self.assertEqual(result["contract_message_count"], 25)
        self.assertEqual(result["registered_message_count"], 37)
        self.assertEqual(result["legacy_message_count"], 13)
        self.assertEqual(result["new_message_count"], 12)
        self.assertEqual(result["topic_count"], 11)

    def test_legacy_ros_md5_compatibility(self):
        md5s = all_message_md5(self.specs)
        self.assertEqual(md5s["BehaviorCommand"], "fad196232c8f54cbbe2a1bced9d3972a")
        self.assertEqual(md5s["DetectedObjectArray"], "14fbd9679c2d87ae7498e4269d9678da")
        self.assertEqual(md5s["LaneBoundaryFamily"], "70ac79f98ca1244c782ea4067becd6af")

    def test_lidar_handoff_md5_matches_message_sources(self):
        contract = yaml.safe_load((REPO_ROOT / "config/contracts/lidar_candidate_topic_contract_v1.yaml").read_text())
        md5s = all_message_md5(self.specs)
        for section in ("messages", "compatibility_messages"):
            for type_name, expected in contract[section].items():
                with self.subTest(type=type_name):
                    self.assertEqual(md5s[type_name.split("/")[-1]], expected)

    def test_planning_debug_md5_matches_message_sources(self):
        contract = yaml.safe_load((REPO_ROOT / "config/contracts/planning_debug_topic_contract_v1.yaml").read_text())
        definition = contract["messages"]["asmc_msgs/UrbanFeatureDebug"]
        self.assertEqual(ros1_md5("UrbanFeatureDebug", self.specs), definition["md5"])
        fields = {field.name for field in self.specs["UrbanFeatureDebug"].fields}
        self.assertTrue(set(definition["required_fields"]).issubset(fields))
        topic = contract["topics"]["/behavior/feature_debug"]
        self.assertEqual(topic["type"], "asmc_msgs/UrbanFeatureDebug")
        self.assertEqual(topic["md5"], definition["md5"])

    def test_camera_observation_md5_matches_message_sources(self):
        contract = yaml.safe_load((REPO_ROOT / "config/contracts/camera_object_observation_v1.yaml").read_text())
        definition = contract["messages"]["asmc_msgs/CameraObjectObservation"]
        self.assertEqual(ros1_md5("CameraObjectObservation", self.specs), definition["md5"])
        fields = {field.name for field in self.specs["CameraObjectObservation"].fields}
        self.assertTrue(set(definition["required_fields"]).issubset(fields))
        self.assertEqual(contract["observation"]["md5"], definition["md5"])

    def test_new_message_ros_md5_is_frozen(self):
        md5s = all_message_md5(self.specs)
        self.assertEqual(md5s["HdMapMeta"], "adb0d98c5dbd41055ebb929fcdf3af19")
        self.assertEqual(md5s["LaneMarkingArray"], "2eae16c9d4bf3385ac35eba4b3d1b823")
        self.assertEqual(md5s["LaneArray"], "95ad93ef58c8ff22f4069b8ca2388395")
        self.assertEqual(md5s["LaneAssociation"], "fac864677f1248f439252de3e651536d")
        self.assertEqual(md5s["MissionZoneArray"], "04325427bb7aedffd9a6ec4881ed46ce")

    def test_lidar_object_v1_stays_frozen_and_v2_is_pinned(self):
        md5s = all_message_md5(self.specs)
        self.assertEqual(md5s["LidarObject"], "b94cf82e0f325e9ce0364bb143df9f06")
        self.assertEqual(md5s["LidarObjectArray"], "5f774d07dffd4c7dfa93a8d4b62c60b4")
        self.assertEqual(md5s["TrackedObject"], "6c96135eb03a3ee93bba474fa835f57d")
        self.assertEqual(md5s["TrackedObjectArray"], "e127b2aabd7494ad01a5a56b005bf07e")
        self.assertEqual(md5s["LidarObjectV2"], "c22c9b52da848aeab0e2b4e73343d9f0")
        self.assertEqual(md5s["LidarObjectArrayV2"], "d728e7fd984c24641abe10dddfda6dd5")

    def test_lidar_object_v2_contract_version_owns_only_v2_messages(self):
        contracts = REPO_ROOT / "config" / "contracts"
        v1_path = "config/contracts/lidar_candidate_topic_contract_v1.yaml"
        v2_path = "config/contracts/lidar_candidate_topic_contract_v2.yaml"
        v1 = yaml.safe_load((REPO_ROOT / v1_path).read_text())
        v2 = yaml.safe_load((REPO_ROOT / v2_path).read_text())
        registry = yaml.safe_load((contracts / "asmc_msgs_contract_registry_v1.yaml").read_text())
        md5s = all_message_md5(self.specs)
        v2_types = {"asmc_msgs/LidarObjectV2", "asmc_msgs/LidarObjectArrayV2"}

        self.assertNotEqual(v2["contract_id"], v1["contract_id"])
        self.assertEqual(v2["base_contract"], v1_path)
        self.assertEqual(set(v2["messages"]), v2_types)
        for type_name, expected in v2["messages"].items():
            with self.subTest(type=type_name):
                self.assertEqual(md5s[type_name.split("/")[-1]], expected)
        for type_name, expected in v2["base_messages"].items():
            with self.subTest(base=type_name):
                self.assertEqual(v1["messages"][type_name], expected)
                self.assertEqual(md5s[type_name.split("/")[-1]], expected)
        self.assertFalse(v2_types & (set(v1["messages"]) | set(v1["compatibility_messages"])))
        rows = {entry["path"]: entry["message_sections"] for entry in registry["contracts"]}
        self.assertEqual(rows.get(v2_path), ["messages"])
        self.assertEqual(rows.get(v1_path), ["messages"])

    def test_lidar_object_v2_is_lossless_append_only_superset(self):
        validate_lidar_object_v2(self.specs)
        v1 = self.specs["LidarObject"]
        v2 = self.specs["LidarObjectV2"]
        self.assertEqual(v2.fields[: len(v1.fields)], v1.fields)
        self.assertEqual(
            {item.name: item.value for item in v2.constants if item.name.startswith("MOTION_")},
            {"MOTION_UNKNOWN": "0", "MOTION_LOW_THREAT": "1", "MOTION_DYNAMIC_PENDING": "2", "MOTION_DYNAMIC": "3"},
        )

    def test_lidar_object_v2_drift_fails_closed(self):
        v2 = self.specs["LidarObjectV2"]
        mutations = {
            "motion_value": replace(v2, constants=tuple(
                replace(item, value="4") if item.name == "MOTION_DYNAMIC" else item for item in v2.constants)),
            "dropped_v1_field": replace(v2, fields=tuple(item for item in v2.fields if item.name != "camera_class")),
            "reordered_v1_field": replace(v2, fields=v2.fields[1:2] + v2.fields[:1] + v2.fields[2:]),
            "bool_motion_state": replace(v2, fields=v2.fields[:-1] + (Field("bool", "motion_state"),)),
        }
        for label, changed in mutations.items():
            with self.subTest(mutation=label):
                specs = dict(self.specs)
                specs["LidarObjectV2"] = changed
                with self.assertRaises(ContractError):
                    validate_lidar_object_v2(specs)
        array_v2 = self.specs["LidarObjectArrayV2"]
        specs = dict(self.specs)
        specs["LidarObjectArrayV2"] = replace(
            array_v2, fields=array_v2.fields[:-1] + (Field("asmc_msgs/LidarObject[]", "objects"),))
        with self.assertRaises(ContractError):
            validate_lidar_object_v2(specs)

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

