"""Scoring payloads survive the actual publisher and ROS wire format."""
import importlib.util
from io import BytesIO
import json
import os
from pathlib import Path
import unittest

try:
    import rospy
    from asmc_msgs.msg import MissionZoneArray
    from asmc_hd_map.msg import HdMapRoute
except ImportError:
    rospy = None


@unittest.skipIf(rospy is None, "ROS Noetic generated messages required")
class ScoringROSTest(unittest.TestCase):
    def test_actual_publisher_roundtrip_preserves_rules_and_unknown_mapping(self):
        from asmc_hd_map.runtime_bundle import load_runtime_bundle, RUNTIME_PINS_REL
        root = Path(__file__).resolve().parents[3]
        spec = importlib.util.spec_from_file_location(
            "scoring_publisher", root / "src/asmc_hd_map/scripts/competition_map_publisher_node.py")
        publisher = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(publisher)
        bundle = load_runtime_bundle(
            root / "src/asmc_hd_map/reports/kcity2025_route_curation_20260917/route_curated_candidate_map.json",
            root / "src/asmc_hd_map/config/kcity2025_competition_metadata.yaml",
            root / RUNTIME_PINS_REL, Path(os.environ.get("ASMC_DATA", str(root.parent / "data"))))
        messages = publisher.build_static_messages(bundle, rospy.Time(123, 456))
        array = messages["mission_zones"]
        buffer = BytesIO()
        array.serialize(buffer)
        decoded = MissionZoneArray().deserialize(buffer.getvalue())
        self.assertEqual(decoded.header.frame_id, "map")
        self.assertEqual(decoded.header.stamp, rospy.Time(123, 456))
        self.assertEqual(len(decoded.zones), 15)
        boundaries = carriers = 0
        for zone in decoded.zones:
            payload = json.loads(zone.rule_payload_json)
            if payload.get("boundary_only"):
                boundaries += 1
                self.assertEqual(payload["violation_penalty_s"], 30)
                self.assertEqual(payload["judged_intersection_count"], 4)
                self.assertIsNone(payload["judged_intersection_id"])
                self.assertFalse(payload["intersection_failure_evaluation_available"])
                self.assertNotIn("red_without_deceleration", payload)
            if "route_rules" in payload:
                carriers += 1
                self.assertEqual(payload["route_rules"], bundle["metadata"]["route_rules"])
                self.assertTrue(payload["route_speed_limits"])
        self.assertEqual((boundaries, carriers), (5, 3))


if __name__ == "__main__":
    unittest.main()
