#!/usr/bin/env python3
import unittest
import rostest
from test_camera_bbox_lane import BboxLaneTest
from Msg.msg import Yolo_bbox
from lane_boundary_adapter.msg import CameraBBoxLaneResult as Result


class EgoMaskTest(unittest.TestCase):
    setUpClass = classmethod(BboxLaneTest.setUpClass.__func__)
    box = classmethod(BboxLaneTest.box.__func__)
    feed = BboxLaneTest.feed

    def test_occluded_contact_is_retained_as_unknown(self):
        self.feed([Yolo_bbox(480, 450, 800, 715, 'car')])
        self.assertFalse(self.result.valid)
        self.assertEqual(self.result.reason, "some_boxes_unclassified")
        self.assertEqual(list(self.result.assignments), [Result.UNKNOWN])
        self.assertEqual(list(self.result.box_reasons), ['bottom_contact_occluded_by_ego_mask'])
        self.feed([self.box(12., 0.)])
        self.assertEqual(list(self.result.assignments), [Result.OWN])


if __name__ == '__main__':
    rostest.rosrun('lane_boundary_adapter', 'camera_bbox_ego_mask', EgoMaskTest)
