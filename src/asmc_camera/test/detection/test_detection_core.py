#!/usr/bin/env python3
"""Object detection helpers: class groups, result filtering and messages."""

import unittest
from pathlib import Path

import yaml
from std_msgs.msg import Header

from asmc_camera.detection.backend import (
    Detection,
    allowed_class_ids,
    filter_result_boxes,
)
from asmc_camera.detection.classes import load_class_groups
from asmc_camera.detection.messages import build_box_array


PACKAGE_ROOT = Path(__file__).resolve().parents[2]
COCO_NAMES = {0: "person", 1: "bicycle", 2: "car", 3: "motorcycle", 5: "bus",
              7: "truck", 9: "traffic light", 11: "stop sign", 28: "suitcase"}


class FakeBox:
    def __init__(self, cls, conf, xyxy):
        self.cls = [cls]
        self.conf = [conf]
        self.xyxy = [xyxy]


class FakeResult:
    def __init__(self, boxes):
        self.boxes = boxes


def config_groups():
    config = yaml.safe_load(
        (PACKAGE_ROOT / "config/camera/object_classes.yaml").read_text(encoding="utf-8")
    )
    return load_class_groups(config["class_groups"])


class ClassGroupTest(unittest.TestCase):
    def test_agreed_groups(self):
        groups = config_groups()
        self.assertEqual("person", groups.group_of("person"))
        self.assertEqual("person", groups.group_of("Bicycle"))
        self.assertEqual("person", groups.group_of("Pedestrian"))
        for name in ("car", "truck", "bus", "motorcycle", "SUV", "Vehicle"):
            self.assertEqual("vehicle", groups.group_of(name), name)
        self.assertEqual("obstacle", groups.group_of("suitcase"))
        self.assertEqual("obstacle", groups.group_of("Obstacle"))
        self.assertIsNone(groups.group_of("traffic light"))
        self.assertIsNone(groups.group_of("stop sign"))

    def test_rejects_overlapping_or_missing_groups(self):
        with self.assertRaises(ValueError):
            load_class_groups({"groups": {"person": ["car"], "vehicle": ["car"]}})
        with self.assertRaises(ValueError):
            load_class_groups({"groups": {"person": ["person"]}})
        with self.assertRaises(ValueError):
            load_class_groups({"groups": {"person": ["a"], "vehicle": ["b"]},
                               "default_group": "unknown"})


class FilterTest(unittest.TestCase):
    def test_allowed_ids_match_names_case_insensitively(self):
        self.assertEqual({0, 2, 5, 7}, allowed_class_ids(COCO_NAMES, ["Bus", "truck", "PERSON", "car"]))

    def test_keeps_model_order_threshold_and_unrounded_coordinates(self):
        results = [FakeResult([
            FakeBox(2, 0.91, [10.7, 20.2, 30.9, 40.1]),
            FakeBox(0, 0.69, [1.0, 2.0, 3.0, 4.0]),
            FakeBox(0, 0.70, [5.5, 6.5, 7.5, 8.5]),
        ]), FakeResult([]), FakeResult(None)]
        detections = filter_result_boxes(results, COCO_NAMES, {0, 2}, ["person", "car"], 0.7)
        self.assertEqual(
            [Detection(10.7, 20.2, 30.9, 40.1, "car", 0.91),
             Detection(5.5, 6.5, 7.5, 8.5, "person", 0.70)],
            detections,
        )

    def test_name_fallback_when_no_class_id_matched(self):
        results = [FakeResult([FakeBox(28, 0.9, [0, 0, 1, 1]), FakeBox(0, 0.9, [0, 0, 2, 2])])]
        detections = filter_result_boxes(results, COCO_NAMES, set(), ["person"], 0.7)
        self.assertEqual(["person"], [item.source_class_name for item in detections])


class MessageTest(unittest.TestCase):
    def setUp(self):
        self.header = Header(frame_id="camera_front")
        self.groups = config_groups()

    def test_groups_keep_order_and_drop_excluded(self):
        detections = [
            Detection(1.5, 2.5, 3.5, 4.5, "truck", 0.8),
            Detection(0, 0, 1, 1, "traffic light", 0.9),
            Detection(5, 6, 7, 8, "bicycle", 0.75),
        ]
        message = build_box_array(self.header, 1280, 720, True, detections, self.groups)
        self.assertEqual("camera_front", message.header.frame_id)
        self.assertEqual((1280, 720, True), (message.image_width, message.image_height, message.valid))
        self.assertEqual(["vehicle", "person"], [box.class_name for box in message.boxes])
        self.assertEqual(["truck", "bicycle"], [box.source_class_name for box in message.boxes])
        self.assertAlmostEqual(1.5, message.boxes[0].xmin, places=5)
        self.assertTrue(all(box.confidence_valid for box in message.boxes))

    def test_valid_empty_frame_is_published_without_boxes(self):
        message = build_box_array(self.header, 1280, 720, True, [], self.groups)
        self.assertTrue(message.valid)
        self.assertEqual([], list(message.boxes))

    def test_invalid_frame_carries_no_boxes(self):
        detections = [Detection(1, 2, 3, 4, "car", 0.9)]
        message = build_box_array(self.header, 0, 0, False, detections, self.groups)
        self.assertFalse(message.valid)
        self.assertEqual([], list(message.boxes))


if __name__ == "__main__":
    unittest.main()
