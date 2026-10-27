#!/usr/bin/env python3
"""Exact IMU context + delayed local scan: callback-driven bounded pairing."""
import time
import unittest
import rospy
import rostest
from std_msgs.msg import Header
from asmc_msgs.msg import LidarTrackingObservation
from test_camera_bbox_imu_projection import ImuBBoxTest

class PairedLocalTest(unittest.TestCase):
    setUpClass=classmethod(ImuBBoxTest.setUpClass.__func__)
    active=classmethod(ImuBBoxTest.active.__func__)
    sample=ImuBBoxTest.sample
    def test_later_arrival_keeps_exact_image_and_expires_without_local(self):
        pub=rospy.Publisher('/perception/lidar/tracks_local',LidarTrackingObservation,queue_size=8)
        end=time.monotonic()+3
        while not pub.get_num_connections() and time.monotonic()<end:time.sleep(.01)
        self.active();lane,context,frame=self.sample();self.results.clear()
        self.context_pub.publish(context);self.family_pub.publish(lane);self.frame_pub.publish(frame)
        time.sleep(.08)
        self.assertFalse(any(m.valid and m.header.stamp==frame.header.stamp for m in self.results))
        m=LidarTrackingObservation();o=m.observation;o.producer_session_id='delayed-test';o.generation=1;o.scan_sequence=1
        o.config_id='config';o.calibration_id='calibration';o.objects.header=Header(stamp=frame.header.stamp,frame_id='base_link');o.objects.status=o.objects.OBSERVED_EMPTY
        pub.publish(m)
        end=time.monotonic()+.15
        while time.monotonic()<end and not any(x.valid and x.header.stamp==frame.header.stamp for x in self.results):time.sleep(.005)
        paired=[x for x in self.results if x.valid and x.header.stamp==frame.header.stamp]
        self.assertTrue(paired,[x.reason for x in self.results]);self.assertTrue(paired[-1].local_obstacles_valid)
        self.assertEqual(paired[-1].local_obstacles_stamp,frame.header.stamp)
        time.sleep(.07);self.active();lane,context,frame=self.sample();self.results.clear()
        self.context_pub.publish(context);self.family_pub.publish(lane);self.frame_pub.publish(frame)
        time.sleep(.28);self.active();time.sleep(.08)
        self.assertFalse(any(x.valid and x.header.stamp==frame.header.stamp for x in self.results))
        self.assertTrue(any('waiting_for_local_contact_observation' in x.reason for x in self.results),[x.reason for x in self.results])

if __name__=='__main__':rostest.rosrun('lane_boundary_adapter','local_contact_pending',PairedLocalTest)
