#!/usr/bin/env python3

import sys
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "tools/audit"))
from validate_bridge_profiles import validate  # noqa: E402


class BridgeProfileContractTest(unittest.TestCase):
    def test_repository_contract_is_valid(self):
        result = validate(REPO_ROOT / "config/bridge/bridge_profiles_v1.json")
        self.assertEqual("morai_udp", result["runtime_protocol"])
        self.assertEqual("9093/9094 mode 2", result["ctrl"])
        self.assertEqual("9088/9099", result["ego_status"])
        self.assertEqual(["lan2lan", "local"], result["transport_profiles"])
        self.assertEqual(4, result["network_profiles"])
        self.assertEqual(14, result["published_topic_count"])
        self.assertEqual(
            ["integration_launch", "morai_udp_bridge", "morai_sensor_bridge", "vehicle_state"],
            result["packages"],
        )

    def test_10hz_baseline_and_15hz_candidate_are_distinct(self):
        result = validate(REPO_ROOT / "config/bridge/bridge_profiles_v1.json")
        self.assertAlmostEqual(0.1, result["sensor_sets"]["v2"]["lidar_period_s"])
        self.assertAlmostEqual(1.0 / 15.0, result["sensor_sets"]["v3"]["lidar_period_s"])


if __name__ == "__main__":
    unittest.main()
