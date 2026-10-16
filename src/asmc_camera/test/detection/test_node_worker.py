import time
import threading
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
            if topic not in ("/perception/camera/objects", "/perception/camera/objects/diagnostics"):
                return Mock()
            return Mock(publish=(self.diagnostics if topic.endswith("diagnostics") else self.outputs).append)
        self.patches = [patch.object(rospy, "get_param", side_effect=lambda name, default=None: params.get(name, default)),
                        patch.object(rospy, "Publisher", side_effect=publisher),
                        patch.object(rospy, "Subscriber"), patch.object(rospy, "on_shutdown"),
                        patch.object(rospy, "Timer"), patch.object(rospy, "Service"),
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

    def request(self, mode, hz=20):
        from asmc_msgs.srv import SetInferenceModeRequest
        result = self.node.control._request(SetInferenceModeRequest(mode, hz))
        self.assertTrue(result.accepted)

    def test_standby_prepare_active_and_invalidation(self):
        self.request(0)
        self.send(1, self.jpeg)
        time.sleep(.1)
        self.backend.detect.assert_not_called()
        self.request(1)
        for n in range(2, 5):
            self.send(n, self.jpeg)
            self.wait(lambda: self.backend.detect.call_count >= n-1)
            time.sleep(.06)
        self.assertTrue(self.node.control.state.snapshot()['prepared'])
        self.assertEqual([], self.outputs)
        self.request(2)
        self.send(5, self.jpeg)
        self.wait(lambda: len(self.outputs) == 1)
        self.assertTrue(self.outputs[-1].valid)
        self.request(0)
        self.assertFalse(self.outputs[-1].valid)
        self.assertEqual(5, self.outputs[-1].header.seq)

    def test_transition_discards_inflight_result(self):
        entered, release = threading.Event(), threading.Event()
        def blocked(image):
            entered.set()
            release.wait(1)
            return [], {}
        self.backend.detect.side_effect = blocked
        self.send(1, self.jpeg)
        self.assertTrue(entered.wait(1))
        self.request(0)
        release.set()
        time.sleep(.1)
        self.assertEqual([], self.outputs)

    def test_mask_is_applied_during_prepare_and_active(self):
        from asmc_camera.detection.hood_mask import HoodMask
        self.node.processor.hood_mask=HoodMask([(0,.79),(1,.79),(1,1),(0,1)])
        jpeg=cv2.imencode('.jpg',np.full((24,32,3),200,np.uint8))[1].tobytes()
        self.request(1);self.send(1,jpeg)
        self.wait(lambda:self.backend.detect.call_count==1)
        self.assertFalse(self.backend.detect.call_args.args[0][19:].any())
        self.assertTrue(self.backend.detect.call_args.args[0][:19].any())
        self.assertEqual([],self.outputs)
        self.request(2);self.send(2,jpeg)
        self.wait(lambda:len(self.outputs)==1)
        self.assertTrue(self.outputs[0].valid)
        self.assertEqual((32,24),(self.outputs[0].image_width,self.outputs[0].image_height))
        self.assertFalse(self.backend.detect.call_args.args[0][19:].any())


if __name__ == "__main__":
    unittest.main()
