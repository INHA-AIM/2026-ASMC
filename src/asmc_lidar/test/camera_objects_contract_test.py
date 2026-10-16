#!/usr/bin/env python3
"""Cross-check handoff references against the real camera producer, without ROS/GPU."""
import ast
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[3]
CAMERA = ROOT / 'src/asmc_camera'


def read_yaml(path):
    return yaml.safe_load(path.read_text())


class CameraObjectsContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = read_yaml(ROOT / 'src/asmc_lidar/config/camera_objects.yaml')
        cls.contract = read_yaml(ROOT / cls.config['observation_contract'])

    def camera_reference(self, name):
        reference = self.config[name]
        prefix = '$(find asmc_camera)/'
        self.assertTrue(reference.startswith(prefix))
        path = CAMERA / reference[len(prefix):]
        self.assertTrue(path.is_file(), str(path))
        return read_yaml(path)

    def test_observation_and_mode_paths_match_central_contract(self):
        self.assertEqual(self.config['observation_topic'], self.contract['observation']['topic'])
        self.assertEqual(self.config['status_topic'], self.contract['mode']['roles']['objects']['status'])
        self.assertTrue(self.contract['observation']['one_message_per_view'])
        self.assertEqual(self.contract['identity']['producer_session_id'], 'uuid_per_object_producer_start')
        self.assertIn('calibration_id', self.contract['identity']['deduplication_key'])

    def test_camera_ids_and_native_geometry_resolve_to_calibration(self):
        calibration = self.camera_reference('calibration_config')
        self.assertEqual(self.config['cameras'], {
            'front': 'camera_front', 'left': 'camera_left', 'right': 'camera_right'})
        self.assertEqual(set(self.config['cameras']), set(calibration['cameras']))
        expected = {'front': (1280, 720), 'left': (640, 480), 'right': (640, 480)}
        for view, (width, height) in expected.items():
            row = calibration['cameras'][view]
            self.assertEqual((row['width'], row['height']), (width, height))
            self.assertEqual(len(row['translation_m']), 3)
            self.assertEqual(len(row['rotation_deg']), 3)

    def test_labels_and_groups_match_actual_model_profiles(self):
        groups = self.camera_reference('class_groups_config')['class_groups']['groups']
        self.assertEqual(set(groups), {'vehicle', 'dynamic_obstacle', 'static_obstacle'})
        labels = [name for values in groups.values() for name in values]
        e8 = read_yaml(CAMERA / 'config/camera/yolo26l_mission16_e8_three_view.yaml')
        pretrained = read_yaml(CAMERA / 'config/camera/yolo26l_pretrained_mission_three_view.yaml')
        self.assertEqual(len(labels), len(set(labels)))
        self.assertEqual(set(labels), set(e8['allowed_names']))
        tree = ast.parse((CAMERA / 'src/asmc_camera/detection/mission.py').read_text())
        classes = next(ast.literal_eval(node.value) for node in tree.body if isinstance(node, ast.Assign)
                       and any(isinstance(target, ast.Name) and target.id == 'CLASSES' for target in node.targets))
        self.assertEqual(tuple(e8['allowed_names']), classes)
        self.assertTrue(set(pretrained['allowed_names']) <= set(labels))
        self.assertIn('motorcycle', groups['static_obstacle'])
        self.assertTrue({'bicycle', 'stroller'} <= set(groups['dynamic_obstacle']))

    def test_skeleton_does_not_claim_or_depend_on_runtime_fusion(self):
        self.assertEqual(self.config['implementation_status'], 'contract_only_receiver_not_implemented')
        compatibility = self.config['current_compatibility']
        self.assertEqual(compatibility['camera_scope'], 'front')
        self.assertEqual(compatibility['output_topic'], '/YOLO/BBox')
        package = ET.parse(ROOT / 'src/asmc_lidar/package.xml').getroot()
        dependencies = {node.text for node in package if node.tag.endswith('depend')}
        self.assertFalse({'Main', 'Msg', 'asmc_camera', 'tensorrt', 'torch'} & dependencies)
        self.assertFalse((ROOT / 'src/asmc_lidar/launch').exists())


if __name__ == '__main__':
    unittest.main()
