#!/usr/bin/env python3
"""Closed ROS contract checks for the opt-in contact corridor experiment."""
import importlib.util
from pathlib import Path
import unittest
import subprocess
import roslib.packages
import rospy
import rostest
from morai_msgs.msg import EgoVehicleStatus
spec=importlib.util.spec_from_file_location('legacy',str(Path(__file__).with_name('test_camera_bbox_lane_change.py')))
legacy=importlib.util.module_from_spec(spec);spec.loader.exec_module(legacy)
class ContactTest(unittest.TestCase):
    setUpClass=legacy.LaneChangeTest.__dict__['setUpClass']
    def feed(self,*args,**kwargs):
        if not hasattr(self,'speed_pub'):
            self.speed_pub=rospy.Publisher('/Ego_topic',EgoVehicleStatus,queue_size=1)
            self.speed_value=1.0
            def speed(_):
                m=EgoVehicleStatus();m.header.stamp=rospy.Time.now();m.velocity.x=self.speed_value;self.speed_pub.publish(m)
            self.speed_timer=rospy.Timer(rospy.Duration(.025),speed)
        legacy.LaneChangeTest.feed(self,*args,**kwargs)
    following=legacy.LaneChangeTest.following
    stopped=legacy.LaneChangeTest.stopped
    status=legacy.LaneChangeTest.status
    def setUp(self):
        rospy.set_param('/camera_bbox_lane_change/contact_clearance_enabled',True)
        legacy.LaneChangeTest.setUp(self)
    def tearDown(self):
        if hasattr(self,'speed_timer'):self.speed_timer.shutdown()
        legacy.LaneChangeTest.tearDown(self)
        rospy.delete_param('/camera_bbox_lane_change/contact_clearance_enabled')
    def test_interpolation_equivalence(self):
        executable=roslib.packages.find_node('lane_boundary_adapter','contact_path_probe')[0]
        output=subprocess.check_output([executable,'--interpolation-regression'],text=True,timeout=5)
        self.assertIn('interpolation_equivalent samples=24220',output)

    def test_recorded_drum_heading_has_feasible_candidate(self):
        fixture=Path(__file__).parent/'fixtures/contact-clearance'
        executable=roslib.packages.find_node('lane_boundary_adapter','contact_path_probe')[0]
        output=subprocess.check_output([executable,str(fixture/'family-probe.rosbin'),
            str(fixture/'bbox-probe-v2.rosbin')],text=True,timeout=5)
        self.assertRegex(output,r'lane=0 .*road=1 clear=1')

    def test_recorded_near_corridor_retains_unchanged_clearance_margins(self):
        fixture=Path(__file__).parent/'fixtures/contact-clearance'
        executable=roslib.packages.find_node('lane_boundary_adapter','contact_path_probe')[0]
        output=subprocess.check_output([executable,str(fixture/'near-family.rosbin'),
            str(fixture/'near-bbox.rosbin'),'20','15'],text=True,timeout=5)
        self.assertIn('runtime_decision= candidates=',output)
        self.assertRegex(output,r'lane=1 offset=-0.35 length=3 road=1 clear=1')

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
    def test_actual_speed_must_be_low_and_fresh(self):
        self.feed();self.speed_value=4.0;self.feed(duration=.3)
        self.stopped('contact_actual_speed_above_trial_limit')
        self.speed_value=1.0;self.feed(duration=.3);self.following()
        self.speed_timer.shutdown();self.feed(duration=.5)
        self.stopped('contact_actual_speed_unavailable')

    def test_single_change_budget_is_not_bypassed_by_clearance(self):
        self.feed();self.feed([0],box_x=12.,duration=.8)
        self.assertEqual(self.status()[1]['locked_lane'],'0')
        self.feed([],shift=-3.5,duration=.7)
        self.assertEqual(self.status()[1]['change_completed'],'true')
        self.feed([3.5],shift=-3.5,box_x=12.,duration=.5)
        self.stopped('lane_change_already_completed')

    def test_contact_invalid_and_input_loss_remain_stops(self):
        self.feed()
        self.feed(bbox=False,duration=.8)
        self.assertEqual(self.command.brake,1.)
        self.feed([0,3.5],box_x=6.,duration=.5)
        self.stopped('contact_all_paths_blocked')
        self.feed(gap=True)
        self.assertEqual(self.command.brake,1.)
if __name__=='__main__':rostest.rosrun('lane_boundary_adapter','contact_clearance_integration',ContactTest)
