"""Expanded observations preserve wire data and leave legacy defaults intact."""
import io
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import yaml
from std_msgs.msg import Header
from asmc_msgs.msg import CameraObjectBoxArray
from asmc_camera.detection.backend import Detection, allowed_class_ids
from asmc_camera.detection.classes import load_class_groups
from asmc_camera.detection.messages import build_box_array

PACKAGE = Path(__file__).resolve().parents[2]


class CocoProfilesTest(unittest.TestCase):
    def test_all_80_labels_have_a_group_and_infrastructure_is_preserved(self):
        config = yaml.safe_load((PACKAGE/'config/camera/yolo11m_coco_all.yaml').read_text())
        names = config['allowed_names']
        self.assertEqual(80, len(names))
        self.assertEqual(80, len(set(names)))
        groups = load_class_groups(yaml.safe_load((PACKAGE/'config/camera/object_classes_coco_all.yaml').read_text())['class_groups'])
        for label in names:
            self.assertIn(groups.group_of(label), ('person', 'vehicle', 'obstacle'))
        self.assertEqual('person', groups.group_of('bicycle'))
        self.assertEqual('vehicle', groups.group_of('motorcycle'))
        self.assertEqual('obstacle', groups.group_of('traffic light'))
        self.assertEqual('obstacle', groups.group_of('stop sign'))
        self.assertEqual(set(range(80)), allowed_class_ids(dict(enumerate(names)), names))

    def test_generated_message_round_trip_keeps_all_labels_and_coordinates(self):
        config = yaml.safe_load((PACKAGE/'config/camera/yolo11m_coco_all.yaml').read_text())
        groups = load_class_groups(yaml.safe_load((PACKAGE/'config/camera/object_classes_coco_all.yaml').read_text())['class_groups'])
        detections = [Detection(1.25, 2.5, 10.75, 20.0, label, .875) for label in config['allowed_names']]
        header = Header(seq=17, frame_id='camera_front')
        header.stamp.secs, header.stamp.nsecs = 42, 123456789
        original = build_box_array(header, 1280, 720, True, detections, groups)
        wire = io.BytesIO()
        original.serialize(wire)
        restored = CameraObjectBoxArray().deserialize(wire.getvalue())
        self.assertEqual(header, restored.header)
        self.assertEqual(config['allowed_names'], [b.source_class_name for b in restored.boxes])
        self.assertTrue(all((b.xmin, b.ymin, b.xmax, b.ymax, b.confidence)==(1.25, 2.5, 10.75, 20.0, .875) for b in restored.boxes))
        self.assertTrue(all(b.confidence_valid for b in restored.boxes))
        self.assertEqual([], build_box_array(header, 1280, 720, True, [], groups).boxes)
        invalid = build_box_array(header, 1280, 720, False, detections, groups)
        self.assertFalse(invalid.valid)
        self.assertEqual([], invalid.boxes)

    def test_baseline_and_all_change_only_class_selection(self):
        baseline = yaml.safe_load((PACKAGE/'config/camera/yolo11m_coco.yaml').read_text())
        expanded = yaml.safe_load((PACKAGE/'config/camera/yolo11m_coco_all.yaml').read_text())
        self.assertEqual(['bus', 'truck', 'person', 'car'], baseline.pop('allowed_names'))
        expanded.pop('allowed_names')
        self.assertEqual(baseline, expanded)
        groups = load_class_groups(yaml.safe_load((PACKAGE/'config/camera/object_classes.yaml').read_text())['class_groups'])
        self.assertIsNone(groups.group_of('traffic light'))
        self.assertIsNone(groups.group_of('stop sign'))

    def test_experiment_is_yolo_only_and_defaults_remain_baseline(self):
        root = ET.parse(PACKAGE/'launch/camera_objects_coco_experiment.launch').getroot()
        args = {a.get('name'): a.get('default') for a in root.findall('arg')}
        self.assertEqual('all', args['classes'])
        self.assertEqual('2', args['initial_mode'])
        passed = {a.get('name'): a.get('value') for a in root.find('include').findall('arg')}
        self.assertEqual('false', passed['start_camera_lane'])
        self.assertEqual('false', passed['start_traffic_light'])
        self.assertEqual('true', passed['start_camera_objects'])
        original = ET.parse(PACKAGE/'launch/camera_objects.launch').getroot()
        defaults = {a.get('name'): a.get('default') for a in original.findall('arg')}
        self.assertTrue(defaults['config'].endswith('/yolo11m_coco.yaml'))
        self.assertTrue(defaults['class_groups_config'].endswith('/object_classes.yaml'))


if __name__ == '__main__':
    unittest.main()
