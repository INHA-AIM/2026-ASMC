#!/usr/bin/env python3
"""Real public camera wire, no Main/Msg dependency and no simulator connection."""
import time
import math
import unittest
import rospy
import rostest
from asmc_msgs.msg import CameraObjectBox, CameraObjectBoxArray
from lane_boundary_adapter.msg import CameraBBoxFrame
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header


class FrameBridgeTest(unittest.TestCase):
    def test_source_time_empty_invalid_and_missing_image(self):
        seen = []
        sub = rospy.Subscriber('/planning/camera_bbox_lane/frame', CameraBBoxFrame, seen.append)
        boxes = rospy.Publisher('/perception/camera/objects', CameraObjectBoxArray, queue_size=5)
        images = rospy.Publisher('/image_jpeg/compressed', CompressedImage, queue_size=5)
        deadline = time.monotonic() + 5
        while (not boxes.get_num_connections() or not images.get_num_connections()) and time.monotonic() < deadline:
            time.sleep(.02)
        self.assertGreater(boxes.get_num_connections(), 0)
        for case in ('box', 'empty', 'low_confidence', 'invalid_confidence', 'invalid', 'nan', 'missing', 'wrong_image_frame', 'side'):
            stamp = rospy.Time.now()
            header = Header(stamp=stamp, frame_id='camera_left' if case == 'side' else 'camera_front')
            b = CameraObjectBox(xmin=10, ymin=20, xmax=30, ymax=40,
                                source_class_name='car', class_name='vehicle', confidence=.9, confidence_valid=True)
            if case == 'nan': b.xmin = float('nan')
            if case == 'low_confidence': b.confidence = .5
            if case == 'invalid_confidence': b.confidence_valid = False
            m = CameraObjectBoxArray(header=header, image_width=1280, image_height=720,
                                     valid=case != 'invalid', boxes=[b] if case in ('box', 'nan', 'low_confidence', 'invalid_confidence') else [])
            if case != 'missing':
                ih = Header(stamp=stamp, frame_id='camera_left' if case == 'wrong_image_frame' else header.frame_id)
                images.publish(CompressedImage(header=ih, format='jpeg', data=b'fixture'))
            boxes.publish(m)
            deadline = time.monotonic() + .5
            while time.monotonic() < deadline and not any(x.header.stamp == stamp for x in seen):
                time.sleep(.01)
            matching = [x for x in seen if x.header.stamp == stamp]
            if case == 'side': self.assertEqual(matching, []); continue
            self.assertEqual(len(matching), 1, case)
            result = matching[0]
            self.assertEqual(result.valid, case in ('box', 'empty', 'low_confidence', 'nan'), (case, result.reason))
            self.assertEqual(result.header.stamp, header.stamp)
            self.assertEqual(result.header.frame_id, header.frame_id)
            if case in ('box', 'empty', 'low_confidence'):
                self.assertEqual(result.image.header.stamp, stamp)
                self.assertEqual(result.image.data, b'fixture')
            if case == 'box': self.assertEqual(result.boxes[0].label, 'car')
            if case == 'nan':
                # Preserve malformed evidence for the C++ per-box UNKNOWN gate.
                # test_camera_bbox_lane verifies it never becomes a clear lane.
                self.assertEqual(len(result.boxes), 1)
                self.assertTrue(math.isnan(result.boxes[0].xmin))
            if case in ('empty', 'low_confidence'): self.assertEqual(result.boxes, [])
            if case == 'missing': self.assertEqual(result.reason, 'matching_image_missing')
        sub.unregister()


if __name__ == '__main__':
    rospy.init_node('camera_frame_bridge_test')
    rostest.rosrun('lane_boundary_adapter', 'camera_frame_bridge', FrameBridgeTest)
