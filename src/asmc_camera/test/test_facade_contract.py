#!/usr/bin/env python3

import hashlib
import json
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[3]
PACKAGE_ROOT = REPO_ROOT / "src" / "asmc_camera"
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
            "start_camera_lane": "$(eval arg('profile') in ['planning', 'debug', 'legacy'])",
            "start_traffic_light": "$(eval arg('profile') in ['planning', 'debug'])",
            "start_visualization": "$(eval arg('profile') == 'debug')",
            "visualization_legacy_rqt": "false",
            "visualization_calibration_grid": "false",
            "publish_bev_debug_metadata": "$(arg visualization_legacy_rqt)",
            "lane_enable_shared_family_debug": "$(eval arg('profile') in ['debug', 'legacy'])",
            "lane_enable_boundary_family_candidate": "true",
            "lane_enable_independent_boundary_pipeline": "false",
            "lane_publish_pipeline_audit": "false",
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
                ("asmc_camera", "camera_bev_node.py"),
                ("asmc_camera", "camera_bev_fusion_node.py"),
                ("asmc_camera", "lane_vectorizer_node.py"),
                ("asmc_camera", "traffic_light_node.py"),
            },
            nodes,
        )
        includes = {item.get("file", "") for item in self.launch_root.iter("include")}
        self.assertTrue(any("camera_lane.launch" in item for item in includes))
        self.assertTrue(any("visualization.launch" in item for item in includes))

    def test_camera_bev_lane_implementations_are_physically_owned(self):
        implementations = {
            "semantic_inference_node.py",
            "semantic_visualizer_node.py",
            "camera_bev_node.py",
            "camera_bev_fusion_node.py",
            "camera_bev_visualizer_node.py",
            "lane_vectorizer_node.py",
            "lane_vector_visualizer_node.py",
        }
        canonical_root = PACKAGE_ROOT / "src" / "asmc_camera" / "nodes"
        for filename in implementations:
            body = (canonical_root / filename).read_text(encoding="utf-8")
            self.assertIn("def main", body, filename)

        self.assertFalse((REPO_ROOT / "src/perception").exists())
        self.assertFalse((REPO_ROOT / "src/traffic_light").exists())
        self.assertTrue(
            (canonical_root.parent / "training" / "twinlite" / "training.py").is_file()
        )
        self.assertTrue(
            (canonical_root.parent / "offline" / "bev" / "bag_batch.py").is_file()
        )

    def test_runtime_support_and_config_are_canonically_owned(self):
        canonical_root = PACKAGE_ROOT / "src" / "asmc_camera"
        expected_support = {
            "camera": 8,
            "bev": 19,
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
        for package in ("camera_semantic_perception", "drivable_bev"):
            self.assertNotIn("$(find {})".format(package), launch_body)

        package_root = ET.parse(PACKAGE_ROOT / "package.xml").getroot()
        dependencies = {
            item.text for item in package_root if item.tag.endswith("depend")
        }
        self.assertFalse(dependencies & {"camera_semantic_perception", "drivable_bev"})

    def test_canonical_config_inventory_is_hash_guarded(self):
        evidence = yaml.safe_load(
            (
                REPO_ROOT
                / "config/baselines/perception_support_config_parity_v1.yaml"
            ).read_text(encoding="utf-8")
        )
        self.assertEqual(
            "p4c6_perception_package_closeout_v1",
            evidence["acceptance_id"],
        )
        self.assertEqual(12, len(evidence["canonical_configs"]))
        for item in evidence["canonical_configs"]:
            canonical = (REPO_ROOT / item["path"]).read_bytes()
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
        lane_family = self.runtime["components"]["camera_lane"]["lane_family"]
        self.assertTrue(lane_family["enabled_by_default"])
        self.assertEqual(
            "/perception/lane/boundary_family", lane_family["canonical_topic"]
        )
        self.assertFalse(lane_family["shared_family_debug_enabled_by_default"])
        self.assertEqual(
            "noncanonical_diagnostic_only",
            lane_family["compatibility_topic_policy"],
        )
        self.assertFalse(lane_family["compatibility_topic_enabled_by_default"])
        self.assertEqual(
            "lane_enable_independent_boundary_pipeline",
            lane_family["compatibility_launch_argument"],
        )

        traffic = self.runtime["components"]["traffic_light"]
        self.assertEqual("active_parallel_p4c", traffic["canonical_producer_status"])
        self.assertEqual(
            "config/baselines/perception_traffic_light_parity_v1.yaml",
            traffic["parity_evidence"],
        )
        self.assertEqual(
            "typed_preferred_legacy_fallback_p4c3",
            traffic["planning_consumer_cutover_status"],
        )
        self.assertTrue(self.runtime["compatibility"]["preserve_legacy_topics"])
        self.assertTrue(self.runtime["compatibility"]["physical_source_moves"])
        self.assertTrue(self.runtime["compatibility"]["package_deletions"])
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
            "asmc_camera",
            self.runtime["components"]["camera_lane"]["support_library_owner"],
        )
        self.assertEqual(
            "data_collection",
            self.runtime["external_components"]["recorder"]["owner_package"],
        )
        self.assertEqual(
            "src/lidar+sensorfusion",
            self.runtime["external_components"]["lidar_fusion"]["owner_workspace"],
        )
        self.assertEqual(
            "passed_p4c5", self.runtime["compatibility"]["representative_bag_parity"]
        )

    def test_runtime_inference_is_owned_by_ros_container(self):
        compose = yaml.safe_load(
            (
                REPO_ROOT / "docker" / "ros-noetic" / "docker-compose.yaml"
            ).read_text(encoding="utf-8")
        )
        service = compose["services"]["asmc-ros-noetic"]
        self.assertEqual("all", service["gpus"])
        self.assertIn(
            "TWINLITE_ROOT=/opt/baselines/TwinLiteNetPlus",
            service["environment"],
        )
        self.assertTrue(
            any(
                item.endswith(
                    ":/opt/baselines/TwinLiteNetPlus:ro"
                )
                for item in service["volumes"]
            )
        )
        self.assertFalse((REPO_ROOT / "scripts" / "perception.sh").exists())

        training_compose = yaml.safe_load(
            (
                REPO_ROOT / "docker" / "perception-train" / "docker-compose.yaml"
            ).read_text(encoding="utf-8")
        )
        training = training_compose["services"]["asmc-perception-train"]
        self.assertNotEqual("host", training.get("network_mode"))
        self.assertFalse(
            any(
                item.startswith(("ROS_MASTER_URI=", "ROS_IP="))
                for item in training["environment"]
            )
        )

    def test_visualization_launch_uses_v3_calibration(self):
        visualizer = ET.parse(PACKAGE_ROOT / "launch" / "visualization.launch").getroot()
        arguments = {
            item.get("name"): item.get("default", "")
            for item in visualizer.findall("arg")
        }
        values = list(arguments.values())
        self.assertTrue(any("cameras_v3.yaml" in value for value in values))
        self.assertEqual("false", arguments["legacy_rqt"])
        self.assertEqual("false", arguments["calibration_grid"])
        node_types = {item.get("type") for item in visualizer.iter("node")}
        node_packages = {item.get("pkg") for item in visualizer.iter("node")}
        self.assertIn("camera_bev_visualizer_node.py", node_types)
        self.assertIn("lane_vector_visualizer_node.py", node_types)
        self.assertEqual({"asmc_camera"}, node_packages)

    def test_runtime_visualization_defaults_are_compact(self):
        visualization = self.runtime["components"]["visualization"]
        self.assertEqual(
            [
                "/perception/camera/debug/mosaic/compressed",
                "/perception/bev/debug/comparison/compressed",
                "/perception/camera/objects/debug/image/compressed",
                "/perception/traffic_light/debug/image/compressed",
            ],
            visualization["image_topics"],
        )
        self.assertFalse(visualization["legacy_rqt"]["enabled_by_default"])
        self.assertEqual(
            "visualization_legacy_rqt",
            visualization["legacy_rqt"]["launch_argument"],
        )
        self.assertFalse(visualization["calibration_grid"]["enabled_by_default"])
        metadata = self.runtime["components"]["camera_lane"]["bev_debug_metadata"]
        self.assertFalse(metadata["enabled_by_default"])
        self.assertEqual("publish_bev_debug_metadata", metadata["launch_argument"])
        self.assertEqual(5, len(metadata["topics"]))
        audit = self.runtime["components"]["camera_lane"]["lane_pipeline_audit"]
        self.assertFalse(audit["enabled_by_default"])
        self.assertEqual("lane_publish_pipeline_audit", audit["launch_argument"])
        self.assertEqual("/perception/lane/pipeline_audit", audit["topic"])

    def test_bev_debug_metadata_is_opt_in_at_both_producers(self):
        camera_bev = next(
            node
            for node in self.launch_root.iter("node")
            if node.get("name") == "camera_bev"
        )
        fusion = next(
            node
            for node in self.launch_root.iter("node")
            if node.get("name") == "camera_bev_fusion"
        )
        camera_params = {
            item.get("name"): item.get("value")
            for item in camera_bev.findall("param")
        }
        fusion_params = {
            item.get("name"): item.get("value")
            for item in fusion.findall("param")
        }
        self.assertEqual(
            "$(arg publish_bev_debug_metadata)", camera_params["publish_coverage"]
        )
        self.assertEqual(
            "$(arg publish_bev_debug_metadata)",
            fusion_params["publish_debug_metadata"],
        )

    def test_lane_pipeline_audit_is_evaluation_opt_in(self):
        vectorizer = next(
            node
            for node in self.launch_root.iter("node")
            if node.get("name") == "camera_lane_vectorizer"
        )
        params = {
            item.get("name"): item.get("value")
            for item in vectorizer.findall("param")
        }
        self.assertEqual(
            "$(arg lane_publish_pipeline_audit)",
            params["publish_pipeline_audit"],
        )
        node_body = (
            PACKAGE_ROOT
            / "src/asmc_camera/nodes/lane_vectorizer_node.py"
        ).read_text(encoding="utf-8")
        self.assertIn(
            'rospy.get_param("~publish_pipeline_audit", False)', node_body
        )
        batch_body = (
            PACKAGE_ROOT / "tools/bev/run_shadow_lane_bag_batch.py"
        ).read_text(encoding="utf-8")
        self.assertIn('"lane_publish_pipeline_audit:=true"', batch_body)

    def test_closed_loop_acceptance_is_isolated_from_control_outputs(self):
        launch = ET.parse(
            PACKAGE_ROOT / "launch" / "traffic_light_closed_loop_acceptance.launch"
        ).getroot()
        nodes = {(item.get("pkg"), item.get("type")) for item in launch.iter("node")}
        self.assertEqual(
            {
                ("asmc_planner", "asmc_behavior_acceptance_node"),
                ("asmc_camera", "traffic_light_closed_loop_acceptance.py"),
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
        self.assertIn("planner_v2.launch", rendered)
        self.assertNotIn("frenet_planner.launch", rendered)
        self.assertIn("controller.launch", rendered)
        self.assertNotIn("asmc_bridge", rendered)
        self.assertIn("traffic_light_vehicle_acceptance.py", rendered)
        self.assertIn('/acceptance/ctrl_cmd', rendered)
        self.assertIn('default="false"', rendered)
        self.assertIn('name="allow_vehicle_control"', rendered)

    def test_traffic_light_model_is_canonically_owned(self):
        model = PACKAGE_ROOT / "models" / "traffic_light" / "best.pt"
        self.assertTrue(model.is_file())
        evidence = yaml.safe_load(
            (
                REPO_ROOT
                / "config/baselines/perception_traffic_light_parity_v1.yaml"
            ).read_text(encoding="utf-8")
        )
        self.assertEqual(
            "src/asmc_camera/models/traffic_light/best.pt",
            evidence["checkpoint"]["path"],
        )
        self.assertEqual(
            evidence["checkpoint"]["sha256"],
            hashlib.sha256(model.read_bytes()).hexdigest(),
        )

    def test_planning_launches_use_the_canonical_traffic_light_model(self):
        behavior_root = REPO_ROOT / "src" / "asmc_planner"
        expected = "$(find asmc_camera)/models/traffic_light/best.pt"
        for name in ("autonomy_no_bridges_v2.launch",):
            launch = ET.parse(behavior_root / "launch" / name).getroot()
            arguments = {
                item.get("name"): item.get("default", "")
                for item in launch.findall("arg")
            }
            self.assertEqual(expected, arguments["traffic_light_checkpoint"], name)
            self.assertEqual(
                "true",
                arguments["traffic_light"],
                name + " owns traffic light unless an external producer is selected",
            )
            traffic_arg = next(
                item
                for item in launch.findall("arg")
                if item.get("name") == "traffic_light"
            )
            self.assertIn("external traffic-light producer", traffic_arg.get("doc", ""), name)
            owned = launch.find("group[@if='$(arg traffic_light)']")
            self.assertIsNotNone(owned)
            producers = owned.findall("node[@name='traffic_light_infer']")
            self.assertEqual(1, len(producers))
            self.assertEqual("asmc_camera", producers[0].get("pkg"))

        package = ET.parse(behavior_root / "package.xml").getroot()
        dependencies = {
            item.text for item in package if item.tag.endswith("depend")
        }
        self.assertIn("asmc_camera", dependencies)
        self.assertNotIn("traffic_light", dependencies)


if __name__ == "__main__":
    unittest.main()
