#!/usr/bin/env python3
import time, unittest
import rospy, rostest
from std_msgs.msg import UInt8
from morai_msgs.msg import CtrlCmd
from asmc_msgs.msg import ControlHandoff

class ModeFirst(unittest.TestCase):
    def test_mode_before_request_has_no_brake_pulse(self):
        output=[]
        rospy.Subscriber('/ctrl_cmd',CtrlCmd,lambda m:output.append(m))
        mode=rospy.Publisher('/planning/mode',UInt8,queue_size=1)
        normal=rospy.Publisher('/ctrl_cmd/normal',CtrlCmd,queue_size=1)
        request=rospy.Publisher('/planning/jamming/handoff_request',ControlHandoff,queue_size=1)
        candidate=rospy.Publisher('/planning/jamming/handoff_candidate',ControlHandoff,queue_size=1)
        end=time.monotonic()+3
        while not all(p.get_num_connections() for p in [mode,normal,request,candidate]):
            self.assertLess(time.monotonic(),end);time.sleep(.005)
        mode.publish(UInt8(0));time.sleep(.02)
        normal.publish(CtrlCmd(longlCmdType=1,accel=.2));time.sleep(.03)
        self.assertTrue(output and output[-1].accel==.2)
        begin=len(output)
        mode.publish(UInt8(1));time.sleep(.02)
        r=ControlHandoff(epoch=1,target_mode=1,valid=True)
        r.header.stamp=rospy.Time.now();request.publish(r);time.sleep(.02)
        c=ControlHandoff(epoch=1,target_mode=1,valid=True,longl_cmd_type=1,accel=.3)
        c.header.stamp=rospy.Time.now();c.header.frame_id='/localization_controller'
        c.state_stamp=c.trajectory_stamp=c.header.stamp;c.state_frame='local/race/0'
        candidate.publish(c)
        end=time.monotonic()+.2
        while not (len(output)>begin and output[-1].accel==.3):
            self.assertLess(time.monotonic(),end);time.sleep(.002)
        self.assertTrue(all(m.brake<1 for m in output[begin:]),'mode-first ordering must not insert full brake')
        time.sleep(.3)
        self.assertEqual(output[-1].brake,1,'250ms watchdog remains active')

if __name__=='__main__':
    rospy.init_node('handoff_mode_first_test')
    rostest.rosrun('asmc_planner','handoff_mode_first',ModeFirst)
