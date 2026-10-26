"""Main may adapt camera observations but must not install or start inference."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


MAIN = Path(__file__).resolve().parents[1]
ROOT = MAIN.parents[3]


class CameraInferenceOwnershipTest(unittest.TestCase):
    def test_main_has_one_model_free_front_adapter(self):
        launch = ET.parse(MAIN / 'main.launch').getroot()
        args = {node.get('name'): node.get('default') for node in launch.findall('arg')}
        self.assertNotIn('start_yolo', args)
        self.assertFalse({'yolo_device', 'yolo_require_cuda', 'yolo_perf_log_interval_sec'} & set(args))
        self.assertEqual(args['camera_class_profile'], 'mission16_v1')
        self.assertEqual(args['camera_objects_conf_thres'], '0.0')
        self.assertEqual(args['fusion_yolo_policy'], 'unique')
        self.assertEqual(args['fusion_max_time_offset_sec'], '0.05')
        self.assertEqual(args['fusion_sync_wait_timeout_sec'], '0.20')
        adapters = [node for node in launch.iter('node') if node.get('type') == 'camera_objects_yolo_bbox_adapter.py']
        self.assertEqual(len(adapters), 1)
        self.assertIsNone(adapters[0].get('unless'))
        self.assertFalse(any('yolo_detection' in node.get('type', '').lower() for node in launch.iter('node')))
        self.assertFalse(any('asmc_camera' in node.get('file', '') for node in launch.iter('include')))

    def test_no_installable_legacy_detector_or_weights(self):
        cmake = (MAIN / 'CMakeLists.txt').read_text()
        for relative in ('src/Camera/YOLO/YOLODetection.py', 'scripts/yolo_detection_node',
                         'YOLOWeight/yolo11m.pt', 'YOLOWeight/yolo11m(1).pt'):
            self.assertFalse((MAIN / relative).exists(), relative)
            self.assertNotIn(relative, cmake)
        self.assertIn('scripts/camera_objects_yolo_bbox_adapter.py', cmake)
        self.assertNotIn('import torch', (MAIN / 'scripts/camera_objects_yolo_bbox_adapter.py').read_text())

    def test_active_includers_do_not_pass_removed_arguments(self):
        for relative in ('tools/perception/b0_main.launch', 'tools/perception/coco7_preview_stack.launch',
                         'tools/scen_runner/launch/perception.launch',
                         'src/data_collection/launch/capture_collection_planner.launch'):
            tree = ET.parse(ROOT / relative)
            self.assertFalse(any(node.get('name') == 'start_yolo' for node in tree.iter('arg')), relative)
        script = (ROOT / 'src/lidar+sensorfusion/start_main.sh').read_text()
        self.assertIn('flock -n 9', script)
        self.assertIn('exec roslaunch Main main.launch', script)
        self.assertNotIn('import torch', script)
        self.assertNotIn('ASMC_YOLO_PYTHON', script)
        scenario = ET.parse(ROOT / 'tools/scen_runner/launch/perception.launch').getroot()
        camera = scenario.find('include')
        self.assertEqual(camera.get('file'), '$(find asmc_camera)/launch/camera.launch')
        self.assertEqual(camera.find('arg').get('value'), 'active')

    def test_jamming_converter_cannot_start_an_independent_detector(self):
        tree = ET.parse(ROOT / 'src/jamming_lane_boundary_adapter/launch/camera_bbox_lane.launch')
        args = {node.get('name'): node.get('default') for node in tree.getroot().findall('arg')}
        self.assertEqual(args['bbox_topic'], '/perception/camera/objects')
        self.assertEqual(args['conf_thres'], '0.7')
        nodes = list(tree.iter('node'))
        self.assertFalse(any(node.get('pkg') in ('Main','Msg','asmc_camera') for node in nodes))
        bridge=(ROOT/'src/jamming_lane_boundary_adapter/scripts/camera_bbox_frame_bridge.py').read_text()
        self.assertIn('from asmc_msgs.msg import CameraObjectBoxArray',bridge)
        self.assertNotIn('from Msg',bridge)
        self.assertFalse((ROOT/'src/jamming_lane_boundary_adapter/launch/camera_bbox_detector.launch').exists())


if __name__ == '__main__':
    unittest.main()
