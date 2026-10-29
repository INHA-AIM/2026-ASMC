#!/usr/bin/env python3
"""Real mode source and NORMAL MPC approach cap, without a simulator bridge."""
import json
import time
import unittest
import rospy
import rostest
from std_msgs.msg import Bool, String
from asmc_msgs.msg import VehicleState
from morai_msgs.msg import GPSMessage


class ApproachTest(unittest.TestCase):
    def test_prepare_before_boundary_hold_while_slowing_and_release(self):
        prepares=[];traces=[]
        rospy.Subscriber('/planning/localization/prepare',Bool,lambda m:prepares.append(m.data))
        rospy.Subscriber('/planning/localization/approach_trace',String,lambda m:traces.append(json.loads(m.data)))
        state=rospy.Publisher('/planning/vehicle/state',VehicleState,queue_size=1)
        gps=rospy.Publisher('/gps',GPSMessage,queue_size=1)
        def wait(predicate):
            end=time.monotonic()+5
            while not predicate():
                self.assertLess(time.monotonic(),end);time.sleep(.01)
        wait(lambda:state.get_num_connections() and gps.get_num_connections() and prepares and traces)
        gps.publish(GPSMessage(latitude=37.,longitude=126.,status=1))
        def send(x,speed,frame='map',age=.01):
            m=VehicleState(valid=True,x=x,y=0,yaw=0,speed=speed)
            m.header.frame_id=frame;m.header.stamp=rospy.Time.now()-rospy.Duration(age)
            state.publish(m)
        send(0,30./3.6);time.sleep(.25)
        self.assertFalse(prepares[-1]);self.assertEqual(traces[-1]['effective_cap_kph'],0.)
        send(15,30./3.6)
        wait(lambda:prepares[-1] and traces[-1]['effective_cap_kph']==15.)
        send(15,15./3.6);time.sleep(.25)
        self.assertTrue(prepares[-1],'slowing must not shrink and release the window')
        for x,frame,age in [(0,'odom',.01),(0,'map',1.),(float('nan'),'map',.01)]:
            send(x,15./3.6,frame,age);time.sleep(.08)
            self.assertTrue(prepares[-1],'invalid state cannot release a conservative cap')
        time.sleep(.3)
        self.assertEqual(traces[-1]['effective_cap_kph'],15.,'missing state does not lift the cap')
        send(0,15./3.6)
        wait(lambda:not prepares[-1] and traces[-1]['effective_cap_kph']==0.)


if __name__=='__main__':
    rospy.init_node('localization_approach_test')
    rostest.rosrun('asmc_planner','localization_approach',ApproachTest)
