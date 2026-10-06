import time
import unittest
from unittest.mock import Mock, patch

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header

from asmc_camera.nodes.object_detection_node import ObjectDetectionNode


class NodeWorkerTest(unittest.TestCase):
    def setUp(self):
        self.outputs, self.diagnostics = [], []
        params = {"~class_groups": {"groups": {"person": ["person"], "vehicle": ["car"]}}}
        def publisher(topic, *args, **kwargs):
            return Mock(publish=(self.diagnostics if topic.endswith("diagnostics") else self.outputs).append)
        self.patches = [patch.object(rospy, "get_param", side_effect=lambda name, default=None: params.get(name, default)),
                        patch.object(rospy, "Publisher", side_effect=publisher),
                        patch.object(rospy, "Subscriber"), patch.object(rospy, "on_shutdown"),
                        patch.object(rospy, "is_shutdown", return_value=False),
                        patch.object(rospy.Time, "now", return_value=rospy.Time(100)),
                        patch.object(rospy, "loginfo"), patch.object(rospy, "logwarn_throttle"),
                        patch.object(rospy, "get_master", return_value=Mock(getSystemState=lambda: (1, "", [[], [], []])))]
        for item in self.patches:
            item.start()
        self.backend = Mock(info={"device": "fixture", "weights_sha256": "fixture"})
        self.backend.detect.return_value = ([], {"inference_ms": 1})
        self.node = ObjectDetectionNode(backend=self.backend)
        self.jpeg = cv2.imencode(".jpg", np.zeros((24, 32, 3), np.uint8))[1].tobytes()

    def tearDown(self):
        self.node.shutdown()
        for item in reversed(self.patches):
            item.stop()

    def send(self, seq, data, frame="camera_front", stamp=None):
        self.node._callback(CompressedImage(header=Header(seq=seq, stamp=rospy.Time(42, seq) if stamp is None else stamp,
                                                        frame_id=frame), data=data))

    def wait(self, predicate):
        deadline = time.monotonic() + 2
        while not predicate() and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertTrue(predicate())

    def test_worker_empty_decode_and_inference_failure_then_shutdown(self):
        self.send(1, self.jpeg)
        self.wait(lambda: len(self.outputs) == 1)
        self.assertTrue(self.outputs[0].valid)
        self.assertEqual([], self.outputs[0].boxes)
        self.assertEqual(1, self.outputs[0].header.seq)
        self.send(2, b"broken jpeg")
        self.wait(lambda: len(self.outputs) == 2)
        self.assertFalse(self.outputs[-1].valid)
        self.assertEqual(2, self.outputs[-1].header.seq)
        self.backend.detect.side_effect = RuntimeError("fixture failure")
        self.send(3, self.jpeg)
        self.wait(lambda: len(self.outputs) == 3)
        self.assertFalse(self.outputs[-1].valid)
        self.assertEqual((1, 1), (self.node.decode_errors, self.node.inference_errors))
        self.node.shutdown()
        self.assertFalse(self.node.worker.is_alive())
        self.assertFalse(self.node.monitor.is_alive())

    def test_invalid_header_is_withheld_and_input_silence_reported(self):
        self.send(1, self.jpeg, frame="camera_left")
        self.send(2, self.jpeg, stamp=rospy.Time(0))
        self.assertEqual(2, self.node.header_errors)
        self.wait(lambda: bool(self.diagnostics))
        self.assertEqual([], self.outputs)
        status = self.diagnostics[-1].status[0]
        self.assertGreater(status.level, 0)
        values = {v.key: v.value for v in status.values}
        self.assertEqual("2", values["header_errors"])
        self.assertGreaterEqual(float(values["input_silence_sec"]), 0.9)


if __name__ == "__main__":
    unittest.main()
