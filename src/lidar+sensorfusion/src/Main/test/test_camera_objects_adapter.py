import importlib.util
import unittest
from pathlib import Path

from asmc_msgs.msg import CameraObjectBox, CameraObjectBoxArray
from rospy import Time
from std_msgs.msg import Header

spec = importlib.util.spec_from_file_location("camera_adapter", Path(__file__).parents[1] / "scripts/camera_objects_yolo_bbox_adapter.py")
adapter = importlib.util.module_from_spec(spec)
spec.loader.exec_module(adapter)


class AdapterTest(unittest.TestCase):
    def message(self):
        return CameraObjectBoxArray(header=Header(stamp=Time(42, 123), frame_id="camera_front"),
                                    image_width=1280, image_height=720, valid=True)

    def test_preserves_raw_label_order_and_truncates_coordinates(self):
        message = self.message()
        for label, confidence in [("truck", 0.9), ("person", 0.69), ("person", 0.7), ("car", 0.8)]:
            message.boxes.append(CameraObjectBox(xmin=1.9, ymin=2.1, xmax=30.9, ymax=40.2,
                                                 class_name="vehicle", source_class_name=label,
                                                 confidence=confidence, confidence_valid=True))
        result = adapter.convert(message)
        self.assertEqual(message.header, result.header)
        self.assertEqual(["truck", "person", "car"], [b.classes for b in result.yolo_bbox_arr])
        self.assertEqual(3, result.objc)
        self.assertEqual((1, 2, 30, 40), tuple(getattr(result.yolo_bbox_arr[0], k) for k in ["xMin", "yMin", "xMax", "yMax"]))

    def test_empty_valid_frame_and_invalid_inputs(self):
        message = self.message()
        self.assertEqual(0, adapter.convert(message).objc)
        for field, value in [("valid", False), ("image_width", 0)]:
            message = self.message()
            setattr(message, field, value)
            self.assertIsNone(adapter.convert(message))
        message = self.message()
        message.header.frame_id = "camera_left"
        self.assertIsNone(adapter.convert(message))
        message.header.frame_id = "camera_front"
        message.header.stamp = Time()
        self.assertIsNone(adapter.convert(message))

    def test_invalid_confidence_and_nonfinite_box_are_filtered(self):
        message = self.message()
        message.boxes = [CameraObjectBox(source_class_name="car", confidence=0.9, confidence_valid=False),
                         CameraObjectBox(source_class_name="car", confidence=float("nan"), confidence_valid=True),
                         CameraObjectBox(source_class_name="car", confidence=0.9, confidence_valid=True, xmin=float("inf"))]
        self.assertEqual(0, adapter.convert(message).objc)

    def test_old_build_fails_explicitly(self):
        with self.assertRaisesRegex(RuntimeError, "stale outer asmc_msgs"):
            adapter.check_message_build("old")

    def test_seven_raw_labels_and_explicit_legacy_four(self):
        message = self.message()
        message.boxes = [CameraObjectBox(xmin=1, ymin=2, xmax=20, ymax=30,
            class_name=label, source_class_name=label, confidence=.9, confidence_valid=True)
            for label in ("person", "dog", "bicycle", "car", "motorcycle", "bus", "truck")]
        self.assertEqual([b.source_class_name for b in message.boxes],
                         [b.classes for b in adapter.convert(message).yolo_bbox_arr])
        self.assertEqual(["person", "car", "bus", "truck"],
            [b.classes for b in adapter.convert(message, allowed_names=["bus", "truck", "person", "car"]).yolo_bbox_arr])
