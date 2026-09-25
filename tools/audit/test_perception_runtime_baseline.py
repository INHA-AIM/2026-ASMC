#!/usr/bin/env python3
"""Regression tests for the P4-A perception runtime baseline."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
import build_perception_runtime_baseline as baseline


REPO_ROOT = Path(__file__).resolve().parents[2]


class PerceptionRuntimeBaselineTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.manifest = yaml.safe_load(
            (REPO_ROOT / baseline.MANIFEST_REL).read_text(encoding="utf-8")
        )

    def test_scope_and_launch_inventory_is_complete(self) -> None:
        scopes = self.manifest["runtime_scopes"]
        self.assertEqual(17, scopes["scope_count"])
        self.assertGreaterEqual(scopes["tracked_file_count"], 560)
        ids = {item["id"] for item in scopes["items"]}
        self.assertIn("lidar_main", ids)
        self.assertIn("lidar_msg", ids)
        self.assertIn("lidar_morai_msgs", ids)
        self.assertIn("learning_by_cheating", ids)
        launches = self.manifest["launch_inventory"]
        self.assertGreaterEqual(launches["file_count"], 20)
        self.assertGreaterEqual(launches["node_declaration_count"], 20)
        self.assertGreaterEqual(self.manifest["declared_entrypoints"]["count"], 30)

    def test_coordinate_timestamp_and_topic_contract(self) -> None:
        self.assertEqual("1bd8ec9", self.manifest["input_commit"])
        contract = self.manifest["topic_contract"]
        outputs = {item["topic"]: item for item in contract["outputs"]}
        self.assertEqual(
            "base_link",
            outputs["/perception/lane/boundary_observations"]["frame_policy"],
        )
        self.assertEqual(
            "map",
            outputs["/lidar/tracked_objects_map"]["frame_policy"],
        )
        self.assertIn(
            "no source stamp",
            outputs["/traffic_light/state"]["timestamp_policy"],
        )
        self.assertFalse(outputs["/ctrl_cmd_0"]["deployment_allowed"])
        self.assertFalse(outputs["/perception/lane/boundary_family"]["deployment_allowed"])
        rates = self.manifest["rate_contract"]
        configured = rates["bridge_configured_or_nominal_hz"]
        observed = rates["bridge_local_observed_hz"]
        self.assertEqual(15.0, configured["vlp16_configured"])
        self.assertEqual(50.0, configured["imu_nominal"])
        self.assertEqual([42.0, 44.0], observed["imu_approx_range"])
        self.assertEqual(12.0, observed["vlp16_complete_scan_approx"])
        self.assertIn("12 Hz", rates["vlp16_acceptance"])

    def test_model_artifacts_are_exact(self) -> None:
        models = {item["id"]: item for item in self.manifest["model_artifacts"]["items"]}
        self.assertEqual(3, self.manifest["model_artifacts"]["selected_count"])
        self.assertEqual(
            "a26311b3af4e92880f632509eff3c81defc1b95135f5b463c7a08e5b6e23d24b",
            models["camera_twinlite_v6"]["sha256"],
        )
        self.assertFalse(models["lidar_camera_yolo11m_duplicate"]["selected"])

    def test_duplicate_message_md5_is_frozen(self) -> None:
        messages = {
            item["type"]: item["ros1_md5"]
            for item in self.manifest["duplicate_messages"]["items"]
        }
        self.assertEqual(4, len(messages))
        self.assertEqual(
            "e127b2aabd7494ad01a5a56b005bf07e",
            messages["Msg/TrackedObjectArray"],
        )

    def test_oracle_profiles_cannot_be_deployed(self) -> None:
        profiles = {item["name"]: item for item in self.manifest["bag_profiles"]["items"]}
        for name in ("dev_oracle_raw", "dev_oracle_udp", "lidar_dev_gt_udp"):
            self.assertFalse(profiles[name]["deployment_allowed"])
        self.assertTrue(profiles["dev_oracle_raw"]["privileged_gt"])
        self.assertTrue(profiles["dev_oracle_udp"]["privileged_gt"])
        self.assertTrue(profiles["lidar_dev_gt_udp"]["privileged_gt"])

    def test_known_migration_risks_are_not_silently_dropped(self) -> None:
        risk_ids = {item["id"] for item in self.manifest["known_integration_risks"]}
        self.assertIn("camera_calibration_v2_provenance", risk_ids)
        self.assertIn("lidar_stale_drivable_mask", risk_ids)
        self.assertIn("traffic_light_unheadered_output", risk_ids)
        self.assertIn("lidar_legacy_sensor_rate_provenance", risk_ids)
        self.assertIn("bag_profiles_v2_provenance", risk_ids)
        gates = {
            item["id"]: item
            for item in self.manifest["external_reference_gates"]["rules"]
        }
        self.assertGreater(gates["learning_by_cheating"]["blocking_external_file_count"], 0)
        self.assertGreater(gates["duplicate_Msg"]["blocking_external_file_count"], 0)

if __name__ == "__main__":
    unittest.main()
