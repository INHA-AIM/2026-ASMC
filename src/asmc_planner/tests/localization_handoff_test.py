#!/usr/bin/env python3
import copy,time,unittest
import rospy,rostest
from std_msgs.msg import Bool,UInt8
from morai_msgs.msg import CtrlCmd,GPSMessage
from asmc_msgs.msg import ControlHandoff,FrenetFeedback


class HandoffTest(unittest.TestCase):
    def test_prepared_switch_epoch_freshness_return_and_fail_safe(self):
        commands=[];modes=[];requests=[]
        rospy.Subscriber('/ctrl_cmd',CtrlCmd,lambda m:commands.append((time.monotonic(),m)))
        rospy.Subscriber('/planning/mode',UInt8,lambda m:modes.append(m.data))
        rospy.Subscriber('/planning/jamming/handoff_request',ControlHandoff,lambda m:requests.append(m))
        normal=rospy.Publisher('/ctrl_cmd/normal',CtrlCmd,queue_size=1)
        anchor=rospy.Publisher('/planning/localization/anchor_ready',Bool,queue_size=1,latch=True)
        ready=rospy.Publisher('/planning/localization/recovery_ready',Bool,queue_size=1)
        gps=rospy.Publisher('/gps',GPSMessage,queue_size=1)
        candidate=rospy.Publisher('/planning/jamming/handoff_candidate',ControlHandoff,queue_size=4)
        feedback=rospy.Publisher('/planning/frenet_feedback_v2',FrenetFeedback,queue_size=4)
        def wait(predicate):
            end=time.monotonic()+5
            while not predicate():
                self.assertLess(time.monotonic(),end);time.sleep(.005)
        wait(lambda:modes and normal.get_num_connections() and gps.get_num_connections() and candidate.get_num_connections()>=2)
        normal.publish(CtrlCmd(longlCmdType=1,accel=.2));time.sleep(.06)
        self.assertTrue(commands and commands[-1][1].brake==1,'startup waits for the first anchor')
        anchor.publish(Bool(True));time.sleep(.03)
        normal.publish(CtrlCmd(longlCmdType=1,accel=.2));wait(lambda:commands[-1][1].accel==.2)
        gps.publish(GPSMessage());wait(lambda:requests and requests[-1].valid)
        request=requests[-1];self.assertEqual(modes[-1],0,'prepare before selecting')
        def evidence(req,age=0):
            c=ControlHandoff(epoch=req.epoch,target_mode=req.target_mode,valid=True,longl_cmd_type=1,accel=.2)
            c.header.stamp=rospy.Time.now()-rospy.Duration(age)
            c.header.frame_id='/mpc_controller' if c.target_mode==0 else rospy.get_param('/planning_mode/jamming_controller','/localization_controller')
            c.state_stamp=c.trajectory_stamp=c.header.stamp;c.state_frame='map' if c.target_mode==0 else rospy.get_param('/asmc_control_selector/jamming_frame_prefix','local/')+'test'
            return c
        old=evidence(request);old.epoch-=1;candidate.publish(old);time.sleep(.015);self.assertEqual(modes[-1],0)
        stale=evidence(request,.3);candidate.publish(stale);time.sleep(.015);self.assertEqual(modes[-1],0)
        first=len(commands);candidate.publish(evidence(request));wait(lambda:modes[-1]==1 and commands[-1][1].accel==.2)
        self.assertTrue(all(c.brake<1 for _,c in commands[first:]),'prepared entry inserts no emergency stop')
        entry_epoch=request.epoch
        gps.publish(GPSMessage(latitude=37,longitude=126,status=1));time.sleep(.12)
        ready.publish(Bool(True));time.sleep(.02)
        for _ in range(2):
            f=FrenetFeedback(command_accepted=True,plan_valid=True,trajectory_source=FrenetFeedback.TRAJECTORY_NEW)
            f.header.frame_id='map';f.header.stamp=rospy.Time.now();feedback.publish(f);time.sleep(.025)
        wait(lambda:requests[-1].valid and requests[-1].target_mode==0)
        request=requests[-1];self.assertNotEqual(request.epoch,entry_epoch)
        first=len(commands);candidate.publish(evidence(request));wait(lambda:modes[-1]==0 and commands[-1][1].accel==.2)
        self.assertTrue(all(c.brake<1 for _,c in commands[first:]),'prepared return inserts no emergency stop')
        normal.publish(CtrlCmd(longlCmdType=1,accel=.2));time.sleep(.02)
        gps.publish(GPSMessage());wait(lambda:requests[-1].target_mode==1 and requests[-1].epoch!=entry_epoch)
        current=requests[-1];bad=evidence(current);bad.epoch=entry_epoch;candidate.publish(bad)
        time.sleep(.4)
        self.assertEqual(commands[-1][1].brake,1,'unprepared target retains bounded fail-safe stop')
        # A detector clear/re-entry can occur while the deadline has selected
        # JAMMING logically but the selector is still waiting for its candidate.
        gps.publish(GPSMessage(latitude=37,longitude=126,status=1));time.sleep(.12)
        gps.publish(GPSMessage());time.sleep(.04)
        self.assertTrue(requests[-1].valid and requests[-1].target_mode==1,
                        'rapid re-entry preserves the incomplete JAMMING handoff')
        first=len(commands);deadline=time.monotonic()+1
        while not (len(commands)>first and commands[-1][1].accel==.2):
            self.assertLess(time.monotonic(),deadline)
            # Request and candidate travel on different connections. Mirror a
            # controller's fresh candidate cadence after producer re-entry.
            if requests[-1].valid:candidate.publish(evidence(requests[-1]))
            time.sleep(.02)
        time.sleep(.3);self.assertEqual(commands[-1][1].brake,1,'selected command timeout stays 250ms')


if __name__=='__main__':
    rospy.init_node('localization_handoff_test');rostest.rosrun('asmc_planner','localization_handoff',HandoffTest)
