#!/usr/bin/env python3

import copy
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "tools" / "audit"))

from validate_perception_runtime_acceptance import (  # noqa: E402
    AcceptanceContractError,
    _validate_planning_signal_launch,
    expand_forbidden_topics,
    expand_topic_specs,
    load_contract,
    validate_contract,
)

sys.path.insert(0, str(REPO_ROOT / "scripts"))

from verify_perception_runtime import (  # noqa: E402
    call_with_timeout,
    interval_summary,
    snapshot_samples,
)


class PerceptionRuntimeAcceptanceTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.path = (
            REPO_ROOT
            / "config/contracts/perception_runtime_acceptance_v1.yaml"
        )
        cls.contract = load_contract(cls.path)

    def validate_copy(self, value):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "contract.yaml"
            path.write_text(
                yaml.safe_dump(value, sort_keys=False), encoding="utf-8"
            )
            return validate_contract(REPO_ROOT, path)

    def test_repository_contract_is_valid(self):
        result = validate_contract(REPO_ROOT, self.path)
        self.assertEqual(33, result["camera_lane_topic_count"])
        self.assertEqual(38, result["all_topic_count"])
        self.assertEqual(20, result["default_forbidden_topic_count"])
        self.assertEqual(8, result["static_binding_count"])
        self.assertEqual(2, result["optional_consumer_gate_count"])
        self.assertEqual(2, result["deferred_topic_count"])
        self.assertEqual(3, result["p5_closeout"]["phase_count"])
        self.assertEqual(2, result["p5_closeout"]["consumer_count"])
        self.assertEqual(
            293,
            result["p5_closeout"]["representative_lane_family_messages"],
        )

    def test_p5_handoff_keeps_typed_inputs_authoritative(self):
        handoff = self.contract["consumer_handoff"]
        self.assertEqual(
            "typed_authoritative_after_first_message",
            handoff["traffic_light"]["selection_policy"],
        )
        self.assertFalse(
            handoff["traffic_light"][
                "planning_launch_starts_producer_by_default"
            ]
        )
        self.assertEqual(
            "typed_family", handoff["lane_family"]["planning_input"]
        )
        self.assertFalse(handoff["lane_family"]["debug_is_planning_input"])


    def test_standalone_signal_launch_defaults(self):
        for name, default in (
            ("autonomy_no_bridges.launch", "false"),
            ("autonomy_no_bridges_v2.launch", "true"),
        ):
            launch = ET.parse(REPO_ROOT / "src/behavior_planner/launch" / name).getroot()
            _validate_planning_signal_launch(launch, name, default)

    def signal_launch(self):
        return ET.parse(
            REPO_ROOT / "src/behavior_planner/launch/autonomy_no_bridges_v2.launch"
        ).getroot()

    def test_v2_signal_default_cannot_be_disabled_silently(self):
        launch = self.signal_launch()
        launch.find("./arg[@name='traffic_light']").set("default", "false")
        with self.assertRaises(AcceptanceContractError):
            _validate_planning_signal_launch(launch, "autonomy_no_bridges_v2.launch", "true")

    def test_full_perception_pipeline_cannot_be_added_to_planning(self):
        launch = self.signal_launch()
        ET.SubElement(launch, "include", file="$(find asmc_perception)/launch/perception.launch")
        with self.assertRaises(AcceptanceContractError):
            _validate_planning_signal_launch(launch, "autonomy_no_bridges_v2.launch", "true")

    def test_camera_producer_cannot_be_added_to_planning(self):
        launch = self.signal_launch()
        ET.SubElement(launch, "node", pkg="asmc_perception", type="camera_bev_node.py")
        with self.assertRaises(AcceptanceContractError):
            _validate_planning_signal_launch(launch, "autonomy_no_bridges_v2.launch", "true")

    def test_external_signal_opt_out_cannot_be_removed(self):
        launch = self.signal_launch()
        launch.find("./group[@if='$(arg traffic_light)']").attrib.pop("if")
        with self.assertRaises(AcceptanceContractError):
            _validate_planning_signal_launch(launch, "autonomy_no_bridges_v2.launch", "true")

    def test_signal_producer_failure_cannot_be_hidden(self):
        launch = self.signal_launch()
        launch.find(".//node[@type='traffic_light_node.py']").set("required", "false")
        with self.assertRaises(AcceptanceContractError):
            _validate_planning_signal_launch(launch, "autonomy_no_bridges_v2.launch", "true")

    def test_signal_defaults_contract_cannot_drift(self):
        broken = copy.deepcopy(self.contract)
        broken["consumer_handoff"]["traffic_light"]["planning_launch_signal_defaults"][
            "autonomy_no_bridges_v2.launch"
        ] = False
        with self.assertRaises(AcceptanceContractError):
            self.validate_copy(broken)

    def test_incomplete_p5_phase_cannot_claim_closeout(self):
        broken = copy.deepcopy(self.contract)
        broken["phase_status"]["p5e_operational_closeout"] = "in_progress"
        with self.assertRaises(AcceptanceContractError):
            self.validate_copy(broken)

    def test_debug_marker_cannot_be_promoted_to_planning_input(self):
        broken = copy.deepcopy(self.contract)
        broken["consumer_handoff"]["lane_family"][
            "debug_is_planning_input"
        ] = True
        with self.assertRaises(AcceptanceContractError):
            self.validate_copy(broken)

    def test_operational_command_drift_fails_closeout(self):
        broken = copy.deepcopy(self.contract)
        broken["operational_closeout"]["commands"]["perception"] = (
            "./scripts/perception.sh"
        )
        with self.assertRaises(AcceptanceContractError):
            self.validate_copy(broken)

    def test_internal_transport_and_canonical_outputs_are_required(self):
        topics = {
            item["topic"]
            for item in expand_topic_specs(self.contract, ["camera_lane"])
        }
        self.assertIn(
            "/perception/camera/front/road_marking/model_grid/class_id",
            topics,
        )
        self.assertIn(
            "/perception/bev/debug/right/road_marking/confidence", topics
        )
        self.assertIn("/perception/bev/camera/drivable_probability", topics)
        self.assertIn("/perception/lane/boundary_family", topics)
        self.assertNotIn("/perception/lane/boundary_observations", topics)

    def test_default_off_topics_do_not_hide_internal_transport(self):
        required = {
            item["topic"]
            for item in expand_topic_specs(self.contract, ["camera_lane"])
        }
        forbidden = set(expand_forbidden_topics(self.contract))
        self.assertFalse(required & forbidden)
        self.assertIn("/perception/lane/pipeline_audit", forbidden)
        self.assertIn("/perception/bev/camera/source_count", forbidden)
        self.assertIn("/perception/lane/boundary_observations", forbidden)

    def test_stale_planning_anchor_fails_closed(self):
        broken = copy.deepcopy(self.contract)
        binding = next(
            item
            for item in broken["static_bindings"]
            if item["id"] == "planning_traffic_light_topics"
        )
        binding["anchors"][0] = "/wrong/typed/topic"
        with self.assertRaises(AcceptanceContractError):
            self.validate_copy(broken)

    def test_deferred_lidar_cannot_become_required_silently(self):
        broken = copy.deepcopy(self.contract)
        broken["topic_groups"]["camera_lane"]["topics"].append(
            {
                "pattern": "/perception/lidar/tracked_objects",
                "type": "asmc_msgs/TrackedObjectArray",
                "publisher": "/lidar_fusion",
                "min_rate_hz": 1.0,
            }
        )
        with self.assertRaises(AcceptanceContractError):
            self.validate_copy(broken)

    def test_host_wrapper_does_not_start_or_hide_perception(self):
        body = (REPO_ROOT / "scripts/verify_perception.sh").read_text(
            encoding="utf-8"
        )
        self.assertIn("already-running perception graph", body)
        self.assertIn("asmc-ros-noetic", body)
        self.assertIn("set --", body)
        self.assertIn('"${verifier_args[@]}"', body)
        self.assertNotIn("scripts/perception.sh", body)

    def test_runtime_binding_uses_ros_container_and_shared_family(self):
        bindings = {
            item["id"]: item for item in self.contract["static_bindings"]
        }
        self.assertNotIn("host_perception_entrypoint", bindings)
        self.assertEqual(
            "docker/ros-noetic/docker-compose.yaml",
            bindings["ros_runtime_inference_environment"]["file"],
        )
        gate = self.contract["optional_consumer_gates"]["lane_adapter"]
        self.assertEqual(
            "/perception/lane/boundary_family",
            gate["topics"][0]["pattern"],
        )
        self.assertEqual(
            ["/boundary_family_monitor"],
            gate["topics"][0]["subscribers"],
        )

    def test_interval_summary_is_deterministic(self):
        self.assertEqual(
            {
                "samples": 3,
                "p50_ms": 100.0,
                "p95_ms": 190.0,
                "max_ms": 200.0,
            },
            interval_summary([1.0, 1.1, 1.3, 1.4]),
        )

    def test_ros_master_calls_have_a_bounded_wait(self):
        self.assertEqual("ok", call_with_timeout("quick", lambda: "ok", 0.1))
        with self.assertRaises(TimeoutError):
            call_with_timeout("blocked", lambda: __import__("time").sleep(1), 0.01)

    def test_sample_snapshot_preserves_scalar_counters(self):
        result = snapshot_samples(
            {
                "/topic": {
                    "arrivals": [1.0, 2.0],
                    "zero_stamps": 3,
                    "frame_ids": {"base_link"},
                }
            }
        )
        self.assertEqual(3, result["/topic"]["zero_stamps"])
        self.assertEqual([1.0, 2.0], result["/topic"]["arrivals"])
        self.assertEqual(["base_link"], result["/topic"]["frame_ids"])


if __name__ == "__main__":
    unittest.main()
