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

    def test_facade_reuses_existing_runtime_executables(self):
        nodes = {
            (node.get("pkg"), node.get("type"))
            for node in self.launch_root.iter("node")
        }
        self.assertEqual(
            {
                ("drivable_bev", "camera_bev_node.py"),
                ("drivable_bev", "camera_bev_fusion_node.py"),
                ("drivable_bev", "lane_vectorizer_node.py"),
                ("asmc_perception", "traffic_light_node.py"),
            },
            nodes,
        )
        includes = {item.get("file", "") for item in self.launch_root.iter("include")}
        self.assertTrue(any("camera_semantic_perception" in item for item in includes))
        self.assertTrue(any("lidar_legacy_launch" in item for item in includes))
        self.assertTrue(any("data_collection" in item for item in includes))
        self.assertTrue(any("visualization.launch" in item for item in includes))

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
            "pending_owner_integration", traffic["planning_consumer_cutover_status"]
        )
        self.assertEqual("declared_not_published_p4b", lidar["canonical_producer_status"])
        self.assertTrue(self.runtime["compatibility"]["preserve_legacy_topics"])
        self.assertFalse(self.runtime["compatibility"]["physical_source_moves"])
        self.assertFalse(self.runtime["compatibility"]["package_deletions"])
        self.assertFalse(self.runtime["compatibility"]["oracle_inputs_allowed"])

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
        self.assertIn("camera_bev_visualizer_node.py", node_types)
        self.assertIn("lane_vector_visualizer_node.py", node_types)

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
