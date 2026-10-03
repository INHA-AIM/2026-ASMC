#!/usr/bin/env python3
"""Clipped-box continuity: no new/ambiguous object is promoted to a known lane."""
import importlib.util
from pathlib import Path
import unittest
import rospy
import rostest
from Msg.msg import Yolo_bbox
from lane_boundary_adapter.msg import CameraBBoxLaneResult as Result
spec=importlib.util.spec_from_file_location('bbox_fixture',str(Path(__file__).with_name('test_camera_bbox_lane.py')))
fixture=importlib.util.module_from_spec(spec);spec.loader.exec_module(fixture)


class ClippedTrackingTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        fixture.BboxLaneTest.setUpClass()

    def setUp(self):
        self.b=fixture.BboxLaneTest()
        self.b.feed([],duration=.3)

    def normal(self):return Yolo_bbox(600,480,680,560,'car')
    def clipped(self):return Yolo_bbox(600,480,680,568,'car')

    def test_confirmed_track_continues_but_never_renews_ground_age(self):
        self.b.feed([self.clipped()])
        self.assertFalse(self.b.result.valid)
        self.b.feed([self.normal()])
        self.assertTrue(self.b.result.valid,self.b.result)
        self.b.feed([self.clipped()])
        self.assertTrue(self.b.result.valid,self.b.result)
        self.assertEqual(list(self.b.result.assignments),[Result.OWN])
        self.assertEqual(list(self.b.result.clipped_association),[True])
        self.b.feed([self.clipped()],duration=3.2)
        self.assertFalse(self.b.result.valid)
        self.b.feed([self.normal()])
        self.assertTrue(self.b.result.valid)
        self.assertEqual(list(self.b.result.clipped_association),[False])

    def test_ambiguous_new_or_lost_track_is_unknown(self):
        self.b.feed([self.normal()])
        self.b.feed([self.clipped(),self.clipped()])
        self.assertFalse(self.b.result.valid)
        self.assertEqual(list(self.b.result.assignments),[Result.UNKNOWN,Result.UNKNOWN])
        self.b.feed([self.normal()])
        self.b.feed([])
        self.b.feed([self.clipped()])
        self.assertFalse(self.b.result.valid)
        self.b.feed([self.normal()])
        self.b.feed([self.clipped()],image=False,duration=.5)
        self.b.feed([self.clipped()])
        self.assertFalse(self.b.result.valid)

    def test_changed_lane_overlap_and_out_of_crop_box_are_unknown(self):
        self.b.feed([self.normal()])
        self.b.feed([self.clipped()],shift=-1.75)
        self.assertFalse(self.b.result.valid)
        self.b.feed([self.normal()])
        box=self.clipped();box.yMax=650
        self.b.feed([box])
        self.assertFalse(self.b.result.valid)


if __name__=='__main__':
    rostest.rosrun('lane_boundary_adapter','camera_bbox_clipped',ClippedTrackingTest)
