import threading
import unittest
from unittest.mock import Mock, patch

import cv2
import numpy as np
from std_msgs.msg import Header
from rospy import Time

from asmc_camera.camera.frame_buffer import LatestFrameBuffer
from asmc_camera.detection.backend import Detection
from asmc_camera.detection.classes import load_class_groups
from asmc_camera.detection.processor import ObjectFrameProcessor
from asmc_camera.nodes.object_detection_node import check_message_build, CameraObjectBoxArray


class ProcessorTest(unittest.TestCase):
    def setUp(self):
        self.groups = load_class_groups({"groups": {"person": ["person"], "vehicle": ["car"]}})
        self.header = Header(seq=17, stamp=Time(123, 456), frame_id="camera_front")
        self.jpeg = cv2.imencode(".jpg", np.zeros((24, 32, 3), np.uint8))[1].tobytes()
        self.backend = Mock()
        self.processor = ObjectFrameProcessor(self.backend, self.groups)

    def test_detection_and_empty_frame_preserve_header_and_size(self):
        for detections in ([Detection(1.25, 2.5, 8.75, 12, "car", 0.9)], []):
            self.backend.detect.return_value = (detections, {"inference_ms": 2})
            message, _, error = self.processor.process(self.header, self.jpeg)
            self.assertIsNone(error)
            self.assertEqual(self.header, message.header)
            self.assertEqual((32, 24, True), (message.image_width, message.image_height, message.valid))
            self.assertEqual(len(detections), len(message.boxes))

    def test_bad_jpeg_and_inference_failure_are_invalid(self):
        message, _, error = self.processor.process(self.header, b"broken")
        self.assertFalse(message.valid)
        self.assertEqual("decode", error[0])
        self.backend.detect.assert_not_called()
        self.backend.detect.side_effect = RuntimeError("fixture failure")
        message, _, error = self.processor.process(self.header, self.jpeg)
        self.assertFalse(message.valid)
        self.assertEqual([], message.boxes)
        self.assertEqual(self.header, message.header)
        self.assertEqual("inference", error[0])

    def test_stale_generated_message_build_is_rejected(self):
        with patch.object(CameraObjectBoxArray, "_md5sum", "stale"):
            with self.assertRaisesRegex(RuntimeError, "stale asmc_msgs"):
                check_message_build()

    def test_buffer_close_releases_waiting_worker(self):
        buffer = LatestFrameBuffer(["front"])
        thread = threading.Thread(target=buffer.pop_batch)
        thread.start()
        buffer.close()
        thread.join(timeout=1)
        self.assertFalse(thread.is_alive())
