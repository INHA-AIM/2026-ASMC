#!/usr/bin/env python3
import time
import unittest

import rospy
import rostest
from asmc_msgs.msg import CameraObjectBox, CameraObjectBoxArray
from Msg.msg import Yolo_bbox_arr
from std_msgs.msg import Header


class AdapterRosTest(unittest.TestCase):
    def test_roundtrip_and_withheld_inputs(self):
        received = []
        subscriber = rospy.Subscriber("/YOLO/BBox", Yolo_bbox_arr, received.append, queue_size=5)
        publisher = rospy.Publisher("/perception/camera/objects", CameraObjectBoxArray, queue_size=5)
        deadline = time.monotonic() + 8
        while publisher.get_num_connections() == 0 and time.monotonic() < deadline:
            time.sleep(0.02)
        self.assertGreater(publisher.get_num_connections(), 0)
        time.sleep(0.2)
        for frame_id, valid, stamp in [("camera_left", True, 1), ("camera_front", False, 2), ("camera_front", True, 0)]:
            publisher.publish(CameraObjectBoxArray(header=Header(stamp=rospy.Time(stamp), frame_id=frame_id),
                                                     valid=valid, image_width=1280, image_height=720))
            time.sleep(0.05)
        message = CameraObjectBoxArray(header=Header(stamp=rospy.Time(42, 123), frame_id="camera_front"),
                                       valid=True, image_width=1280, image_height=720,
                                       boxes=[CameraObjectBox(xmin=1.9, ymin=2.1, xmax=30.9, ymax=40.2,
                                                              source_class_name="truck", class_name="vehicle",
                                                              confidence=0.9, confidence_valid=True)])
        publisher.publish(message)
        deadline = time.monotonic() + 5
        while not received and time.monotonic() < deadline:
            time.sleep(0.02)
        self.assertEqual(1, len(received))
        self.assertEqual(message.header.stamp, received[0].header.stamp)
        self.assertEqual("camera_front", received[0].header.frame_id)
        self.assertEqual("truck", received[0].yolo_bbox_arr[0].classes)
        self.assertEqual(1, received[0].yolo_bbox_arr[0].xMin)
        message.header.stamp = rospy.Time(43)
        message.boxes = []
        publisher.publish(message)
        deadline = time.monotonic() + 5
        while len(received) < 2 and time.monotonic() < deadline:
            time.sleep(0.02)
        self.assertEqual(2, len(received))
        self.assertEqual(0, received[-1].objc)
        subscriber.unregister()


if __name__ == "__main__":
    rospy.init_node("camera_objects_adapter_test")
    rostest.rosrun("Main", "camera_objects_adapter_test", AdapterRosTest)
