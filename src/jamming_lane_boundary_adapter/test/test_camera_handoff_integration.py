#!/usr/bin/env python3
import importlib.util
from pathlib import Path
import time, unittest
import rospy, rostest
from asmc_msgs.msg import ControlHandoff
from morai_msgs.msg import CtrlCmd
from std_msgs.msg import UInt8
spec=importlib.util.spec_from_file_location('camera_fixture',str(Path(__file__).with_name('test_camera_bbox_lane_change.py')))
fixture=importlib.util.module_from_spec(spec);spec.loader.exec_module(fixture)
class HandoffTest(unittest.TestCase):
    def test_preparation_is_not_actuation_and_epochs_expire(self):
        fixture.LaneChangeTest.setUpClass()
        h=fixture.LaneChangeTest('runTest');h.handoff_enabled=True;h.setUp()
        candidates=[];commands=[]
        cp=rospy.Publisher('/planning/jamming/handoff_request',ControlHandoff,queue_size=1)
        cs=rospy.Subscriber('/planning/jamming/handoff_candidate',ControlHandoff,candidates.append,queue_size=100)
        ds=rospy.Subscriber('/bbox_change_test/command',CtrlCmd,commands.append,queue_size=100)
        epoch=[50]
        def request(_):
            m=ControlHandoff();m.header.stamp=rospy.Time.now();m.epoch=epoch[0];m.target_mode=1;m.valid=True;cp.publish(m)
        timer=None
        try:
            h.mode_pub.publish(UInt8(0));time.sleep(.15)
            timer=rospy.Timer(rospy.Duration(.04),request)
            h.feed(duration=1.2,speed=2.,imu=0.,lidar=[])
            self.assertTrue(any(m.valid and m.epoch==50 and m.state_stamp.to_sec()>0 for m in candidates))
            n=len(commands);h.feed(duration=.3,speed=2.,imu=0.,lidar=[]);self.assertEqual(len(commands),n)
            epoch[0]=49;start=len(candidates);h.feed(duration=.35,speed=2.,imu=0.,lidar=[])
            self.assertFalse(any(m.epoch==49 for m in candidates[start:]))
            epoch[0]=51;h.feed(duration=.6,speed=2.,imu=0.,lidar=[])
            self.assertTrue(any(m.valid and m.epoch==51 for m in candidates))
            h.mode_pub.publish(UInt8(1));h.feed(duration=.3,speed=2.,imu=0.,lidar=[])
            self.assertGreater(len(commands),n)
            h.mode_pub.publish(UInt8(0));timer.shutdown();timer=None;time.sleep(.4)
            n=len(candidates);h.feed(duration=.2,speed=2.,imu=0.,lidar=[]);self.assertEqual(len(candidates),n)
        finally:
            if timer:timer.shutdown()
            cs.unregister();ds.unregister();h.tearDown()
if __name__=='__main__':
    rospy.init_node('camera_handoff_integration_test')
    rostest.rosrun('lane_boundary_adapter','camera_handoff_integration',HandoffTest)
