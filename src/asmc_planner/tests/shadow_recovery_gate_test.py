#!/usr/bin/env python3
import time,unittest
import rospy,rostest
from std_msgs.msg import Bool,UInt8
from morai_msgs.msg import GPSMessage
from asmc_msgs.msg import FrenetFeedback

class RecoveryGate(unittest.TestCase):
    def test_unvalidated_and_stale_anchor_cannot_handoff(self):
        seen=[]
        sub=rospy.Subscriber('/planning/mode',UInt8,lambda m:seen.append(m.data),queue_size=10)
        gps=rospy.Publisher('/gps',GPSMessage,queue_size=10)
        ready=rospy.Publisher('/planning/localization/recovery_ready',Bool,queue_size=10)
        feedback=rospy.Publisher('/planning/frenet_feedback_v2',FrenetFeedback,queue_size=10)
        end=time.monotonic()+5
        while not seen or not gps.get_num_connections() or not ready.get_num_connections():
            self.assertLess(time.monotonic(),end);time.sleep(.02)
        def send_feedback():
            f=FrenetFeedback(command_accepted=True,plan_valid=True,trajectory_source=FrenetFeedback.TRAJECTORY_NEW)
            f.header.frame_id='map';f.header.stamp=rospy.Time.now();feedback.publish(f);time.sleep(.06)
        gps.publish(GPSMessage());time.sleep(.1);self.assertEqual(seen[-1],1)
        gps.publish(GPSMessage(latitude=37,longitude=126,status=1));time.sleep(.1)
        for _ in range(8):ready.publish(Bool(False));send_feedback()
        self.assertEqual(seen[-1],1,'deadline and valid normal plans cannot bypass invalid anchor')
        ready.publish(Bool(True));time.sleep(.3)
        send_feedback();send_feedback();self.assertEqual(seen[-1],1,'stale readiness is not permission to hand off')
        ready.publish(Bool(True));time.sleep(.02);send_feedback();self.assertEqual(seen[-1],1)
        ready.publish(Bool(True));send_feedback();self.assertEqual(seen[-1],0)
        gps.publish(GPSMessage());time.sleep(.1);self.assertEqual(seen[-1],1)
        gps.publish(GPSMessage(latitude=37,longitude=126,status=1));time.sleep(.1)
        send_feedback();send_feedback();self.assertEqual(seen[-1],1,'new loss requires new readiness')
        sub.unregister()

if __name__=='__main__':
    rospy.init_node('shadow_recovery_gate_test');rostest.rosrun('asmc_planner','shadow_recovery_gate',RecoveryGate)
