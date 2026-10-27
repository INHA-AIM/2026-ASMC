#!/usr/bin/env python3
"""Closed ROS contract checks for the opt-in contact corridor experiment."""
import importlib.util
from pathlib import Path
import unittest
import subprocess
import roslib.packages
import rospy
import rostest
spec=importlib.util.spec_from_file_location('legacy',str(Path(__file__).with_name('test_camera_bbox_lane_change.py')))
legacy=importlib.util.module_from_spec(spec);spec.loader.exec_module(legacy)
class ContactTest(unittest.TestCase):
    setUpClass=legacy.LaneChangeTest.__dict__['setUpClass']
    feed=legacy.LaneChangeTest.feed
    following=legacy.LaneChangeTest.following
    stopped=legacy.LaneChangeTest.stopped
    status=legacy.LaneChangeTest.status
    def setUp(self):
        rospy.set_param('/camera_bbox_lane_change/contact_clearance_enabled',True)
        legacy.LaneChangeTest.setUp(self)
    def tearDown(self):
        legacy.LaneChangeTest.tearDown(self)
        rospy.delete_param('/camera_bbox_lane_change/contact_clearance_enabled')
    def test_recorded_drum_heading_has_feasible_candidate(self):
        fixture=Path(__file__).parent/'fixtures/contact-clearance'
        executable=roslib.packages.find_node('lane_boundary_adapter','contact_path_probe')[0]
        output=subprocess.check_output([executable,str(fixture/'family-probe.rosbin'),
            str(fixture/'bbox-probe.rosbin')],text=True,timeout=5)
        self.assertRegex(output,r'lane=0 .*road=1 clear=1')

    def test_contact_centerline_barrier_and_far_car(self):
        self.feed()
        self.feed([1.75],far=True,box_x=9.)
        self.following()
        self.assertTrue(self.status()[1]['contact_decision'].startswith('contact_checked:'))
        self.assertLessEqual(self.command.velocity,5.)
        self.assertEqual(self.status()[1]['locked_lane'],'1')
    def test_contact_own_obstacle_uses_collision_checked_change(self):
        self.feed()
        self.feed([0],box_x=12.,duration=.8)
        self.following()
        self.assertEqual(self.status()[1]['locked_lane'],'0')
        self.assertEqual(self.status()[1]['bbox_stop_suppressed'],'false')
        self.feed([0,3.5],box_x=7.,duration=.3)
        self.stopped('contact_target_path_blocked')
    def test_contact_invalid_and_input_loss_remain_stops(self):
        self.feed()
        self.feed(bbox=False,duration=.8)
        self.assertEqual(self.command.brake,1.)
        self.feed([0,3.5],box_x=6.,duration=.5)
        self.stopped('contact_all_paths_blocked')
        self.feed(gap=True)
        self.assertEqual(self.command.brake,1.)
if __name__=='__main__':rostest.rosrun('lane_boundary_adapter','contact_clearance_integration',ContactTest)
