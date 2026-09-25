#!/usr/bin/env python3

import hashlib
import json
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[3]
PACKAGE_ROOT = REPO_ROOT / "src" / "asmc_perception"
SENSOR_ROOT = REPO_ROOT / "config" / "morai" / "sensors"


def _camera_contract(camera):
    config = camera["cc"]
    return {
        "id": int(camera["m_SensorUniqueID"]),
        "pos": camera["pos"],
        "rot": camera["rot"],
        "period": float(config["sensorPeriod"]),
        "udp": config["udpConfig"],
        "width": int(config["cameraResWidth"]),
        "height": int(config["cameraResHeight"]),
        "fov": float(config["cameraFOV"]),
        "sensor_size": config["sensorSize"],
        "focal_mm": float(config["focalLengthmm"]),
        "focal_px": float(config["focalLengthpixel"]),
        "principal": config["principalPoint"],
        "shift": config["sensorShift"],
        "distortion": config["lensDistortion"],
    }


class PerceptionFacadeContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.launch_path = PACKAGE_ROOT / "launch" / "perception.launch"
        cls.launch_root = ET.parse(cls.launch_path).getroot()
        cls.runtime = yaml.safe_load(
            (PACKAGE_ROOT / "config" / "runtime.yaml").read_text(encoding="utf-8")
        )
        cls.calibration = yaml.safe_load(
            (PACKAGE_ROOT / "config" / "cameras_v3.yaml").read_text(encoding="utf-8")
        )

    def test_facade_has_one_explicit_component_switch_per_runtime_scope(self):
        arguments = {item.get("name"): item for item in self.launch_root.findall("arg")}
        expected = {
            "start_camera_lane": "true",
            "start_traffic_light": "false",
            "start_lidar_fusion": "false",
            "start_recorder": "false",
            "start_visualization": "false",
        }
        for name, default in expected.items():
            self.assertIn(name, arguments)
            self.assertEqual(default, arguments[name].get("default"))

    def test_facade_uses_canonical_runtime_executables(self):
        nodes = {
            (node.get("pkg"), node.get("type"))
            for node in self.launch_root.iter("node")
        }
        self.assertEqual(
            {
                ("asmc_perception", "camera_bev_node.py"),
                ("asmc_perception", "camera_bev_fusion_node.py"),
                ("asmc_perception", "lane_vectorizer_node.py"),
                ("asmc_perception", "traffic_light_node.py"),
            },
            nodes,
        )
        includes = {item.get("file", "") for item in self.launch_root.iter("include")}
        self.assertTrue(any("camera_lane.launch" in item for item in includes))
        self.assertTrue(any("lidar_legacy_launch" in item for item in includes))
        self.assertTrue(any("recorder.launch" in item for item in includes))
        self.assertTrue(any("visualization.launch" in item for item in includes))

    def test_camera_bev_and_recorder_implementations_are_physically_owned(self):
        implementations = {
            "semantic_inference_node.py",
            "semantic_visualizer_node.py",
            "camera_bev_node.py",
            "camera_bev_fusion_node.py",
            "camera_bev_visualizer_node.py",
            "lane_vectorizer_node.py",
            "lane_vector_visualizer_node.py",
            "bag_recorder_node.py",
        }
        canonical_root = PACKAGE_ROOT / "src" / "asmc_perception" / "nodes"
        for filename in implementations:
            body = (canonical_root / filename).read_text(encoding="utf-8")
            self.assertIn("def main", body, filename)

        compatibility_paths = {
            REPO_ROOT / "src/perception/camera_semantic_perception/scripts": {
                "semantic_inference_node.py",
                "semantic_visualizer_node.py",
            },
            REPO_ROOT / "src/perception/drivable_bev/scripts": {
                "camera_bev_node.py",
                "camera_bev_fusion_node.py",
                "camera_bev_visualizer_node.py",
                "lane_vectorizer_node.py",
                "lane_vector_visualizer_node.py",
            },
            REPO_ROOT / "src/data_collection/scripts": {"bag_recorder_node.py"},
        }
        for root, filenames in compatibility_paths.items():
            for filename in filenames:
                body = (root / filename).read_text(encoding="utf-8")
                self.assertIn("asmc_perception.nodes", body, str(root / filename))

    def test_runtime_support_and_config_are_canonically_owned(self):
        canonical_root = PACKAGE_ROOT / "src" / "asmc_perception"
        expected_support = {
            "camera": 8,
            "bev": 19,
            "recorder": 5,
        }
        for namespace, expected_count in expected_support.items():
            files = list((canonical_root / namespace).rglob("*.py"))
            self.assertEqual(expected_count, len(files), namespace)
            for path in files:
                body = path.read_text(encoding="utf-8")
                self.assertNotIn("from camera_semantic_perception", body, str(path))
                self.assertNotIn("from drivable_bev", body, str(path))
                self.assertNotIn("from data_collection", body, str(path))

        launch_body = self.launch_path.read_text(encoding="utf-8")
        for package in (
            "camera_semantic_perception",
            "drivable_bev",
            "data_collection",
        ):
            self.assertNotIn("$(find {})".format(package), launch_body)

        package_root = ET.parse(PACKAGE_ROOT / "package.xml").getroot()
        dependencies = {
            item.text for item in package_root if item.tag.endswith("depend")
        }
        self.assertFalse(
            dependencies
            & {"camera_semantic_perception", "drivable_bev", "data_collection"}
        )

    def test_compatibility_config_mirrors_are_byte_identical(self):
        evidence = yaml.safe_load(
            (
                REPO_ROOT
                / "config/baselines/perception_support_config_parity_v1.yaml"
            ).read_text(encoding="utf-8")
        )
        self.assertEqual(
            "p4c5_perception_support_config_ownership_v1",
            evidence["acceptance_id"],
        )
        self.assertEqual(21, len(evidence["config_mirrors"]))
        for item in evidence["config_mirrors"]:
            canonical = (REPO_ROOT / item["canonical"]).read_bytes()
            compatibility = (REPO_ROOT / item["compatibility"]).read_bytes()
            self.assertEqual(canonical, compatibility, item["canonical"])
            self.assertEqual(
                item["sha256"], hashlib.sha256(canonical).hexdigest()
            )

    def test_v3_calibration_is_bound_to_exact_sensor_artifact(self):
        sensor_path = SENSOR_ROOT / "ASMC_sensor_set_v3.json"
        digest = hashlib.sha256(sensor_path.read_bytes()).hexdigest()
        self.assertEqual(digest, self.calibration["source_sha256"])
        self.assertEqual(
            "config/morai/sensors/ASMC_sensor_set_v3.json",
            self.calibration["source_sensor_set"],
        )

    def test_v2_and_v3_camera_runtime_contracts_are_identical(self):
        values = []
        for name in ("ASMC_sensor_set_v2.json", "ASMC_sensor_set_v3.json"):
            document = json.loads((SENSOR_ROOT / name).read_text(encoding="utf-8"))
            values.append([_camera_contract(item) for item in document["cameraList"]])
        self.assertEqual(values[0], values[1])

    def test_typed_topic_cutover_status_matches_runtime(self):
        traffic = self.runtime["components"]["traffic_light"]
        lidar = self.runtime["components"]["lidar_fusion"]
        self.assertEqual("active_parallel_p4c", traffic["canonical_producer_status"])
        self.assertEqual(
            "config/baselines/perception_traffic_light_parity_v1.yaml",
            traffic["parity_evidence"],
        )
        self.assertEqual(
            "typed_preferred_legacy_fallback_p4c3",
            traffic["planning_consumer_cutover_status"],
        )
        self.assertEqual("declared_not_published_p4b", lidar["canonical_producer_status"])
        self.assertTrue(self.runtime["compatibility"]["preserve_legacy_topics"])
        self.assertTrue(self.runtime["compatibility"]["physical_source_moves"])
        self.assertFalse(self.runtime["compatibility"]["package_deletions"])
        self.assertFalse(self.runtime["compatibility"]["oracle_inputs_allowed"])
        consumer = self.runtime["compatibility"]["planning_consumer"]
        self.assertEqual(
            "/perception/traffic_light/detection", consumer["canonical_topic"]
        )
        self.assertEqual("unknown_without_fallback", consumer["invalid_or_stale_policy"])
        self.assertEqual(
            "physical_vehicle_closed_loop_p4c4",
            consumer["closed_loop_status"],
        )
        self.assertEqual(
            "p4c4_traffic_light_vehicle_closed_loop_v1",
            traffic["vehicle_closed_loop_acceptance"],
        )
        self.assertEqual(
            "config/baselines/perception_runtime_move_smoke_v1.yaml",
            self.runtime["components"]["recorder"]["live_smoke_evidence"],
        )
        self.assertEqual(
            "asmc_perception",
            self.runtime["components"]["camera_lane"]["support_library_owner"],
        )
        self.assertEqual(
            "asmc_perception", self.runtime["components"]["recorder"]["profile_owner"]
        )
        self.assertEqual(
            "passed_p4c5", self.runtime["compatibility"]["representative_bag_parity"]
        )

    def test_host_entrypoint_selects_the_gpu_container(self):
        script = (REPO_ROOT / "scripts" / "perception.sh").read_text(encoding="utf-8")
        self.assertIn("asmc-perception-train", script)
        self.assertIn("docker_perception_up.sh", script)
        self.assertIn("roslaunch asmc_perception perception.launch", script)

    def test_visualization_launch_uses_v3_calibration(self):
        visualizer = ET.parse(PACKAGE_ROOT / "launch" / "visualization.launch").getroot()
        values = [item.get("default", "") for item in visualizer.findall("arg")]
        self.assertTrue(any("cameras_v3.yaml" in value for value in values))
        node_types = {item.get("type") for item in visualizer.iter("node")}
        node_packages = {item.get("pkg") for item in visualizer.iter("node")}
        self.assertIn("camera_bev_visualizer_node.py", node_types)
        self.assertIn("lane_vector_visualizer_node.py", node_types)
        self.assertEqual({"asmc_perception"}, node_packages)

    def test_closed_loop_acceptance_is_isolated_from_control_outputs(self):
        launch = ET.parse(
            PACKAGE_ROOT / "launch" / "traffic_light_closed_loop_acceptance.launch"
        ).getroot()
        nodes = {(item.get("pkg"), item.get("type")) for item in launch.iter("node")}
        self.assertEqual(
            {
                ("behavior_planner", "behavior_node"),
                ("asmc_perception", "traffic_light_closed_loop_acceptance.py"),
            },
            nodes,
        )
        rendered = ET.tostring(launch, encoding="unicode")
        self.assertNotIn("mpc_controller", rendered)
        self.assertNotIn("frenet_planner", rendered)
        self.assertNotIn("asmc_bridge", rendered)

    def test_vehicle_acceptance_stack_never_starts_a_bridge(self):
        launch = ET.parse(
            PACKAGE_ROOT
            / "launch"
            / "traffic_light_vehicle_acceptance_stack.launch"
        ).getroot()
        rendered = ET.tostring(launch, encoding="unicode")
        self.assertIn("behavior_node_v2.launch", rendered)
        self.assertIn("frenet_planner.launch", rendered)
        self.assertIn("mpc_controller.launch", rendered)
        self.assertNotIn("asmc_bridge", rendered)
        self.assertIn("traffic_light_vehicle_acceptance.py", rendered)
        self.assertIn('/acceptance/ctrl_cmd', rendered)
        self.assertIn('default="false"', rendered)
        self.assertIn('name="allow_vehicle_control"', rendered)

    def test_parity_launch_compares_reference_canonical_and_typed_outputs(self):
        audit = ET.parse(
            PACKAGE_ROOT / "launch" / "traffic_light_parity.launch"
        ).getroot()
        nodes = {(item.get("pkg"), item.get("type")) for item in audit.iter("node")}
        self.assertIn(("traffic_light", "live_infer.py"), nodes)
        self.assertIn(("asmc_perception", "traffic_light_node.py"), nodes)
        self.assertIn(("asmc_perception", "traffic_light_parity_audit.py"), nodes)


if __name__ == "__main__":
    unittest.main()
