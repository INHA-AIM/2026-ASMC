#!/usr/bin/env python3
"""Real incoming MPC keeps the applied scalar seed after a new pose frame."""
import time
import unittest
import rospy
import rostest
from nav_msgs.msg import Odometry
from morai_msgs.msg import CtrlCmd
from asmc_msgs.msg import ControlHandoff, LocalTrajectory, TrajectoryPoint


class SeedTest(unittest.TestCase):
    def test_seed_survives_first_frame_and_frame_restart(self):
        candidates=[]
        rospy.Subscriber('/planning/localization/handoff_candidate',ControlHandoff,
                         lambda m:candidates.append(m),queue_size=20)
        state=rospy.Publisher('/planning/local/odometry',Odometry,queue_size=1)
        path=rospy.Publisher('/planning/local/trajectory',LocalTrajectory,queue_size=1)
        applied=rospy.Publisher('/ctrl_cmd',CtrlCmd,queue_size=1)
        request=rospy.Publisher('/planning/localization/handoff_request',ControlHandoff,queue_size=1)
        deadline=time.monotonic()+5
        while not all(p.get_num_connections() for p in (state,path,applied,request)):
            self.assertLess(time.monotonic(),deadline);time.sleep(.01)
        for generation,(steering,speed) in enumerate(((.2,1.),(-.2,1.),(0.,2.))):
            frame='local/seed_fixture/'+str(generation)
            start=len(candidates);deadline=time.monotonic()+4
            # Same handoff epoch tests re-preparation after an input-frame reset.
            req=ControlHandoff(epoch=1,target_mode=1,valid=True)
            req.header.stamp=rospy.Time.now()
            while not any(m.valid and m.state_frame==frame for m in candidates[start:]):
                self.assertLess(time.monotonic(),deadline)
                applied.publish(CtrlCmd(longlCmdType=1,steering=steering))
                stamp=rospy.Time.now()-rospy.Duration(.01)
                m=Odometry();m.header.stamp=stamp;m.header.frame_id=frame
                m.child_frame_id='base_link';m.pose.pose.orientation.w=1.;m.twist.twist.linear.x=speed
                trajectory=LocalTrajectory(valid=True,reason='fixture',run_id='seed_test',plan_sequence=1,dt=.05)
                trajectory.header=m.header;trajectory.ego_at_plan=m;trajectory.scene_stamp=stamp
                trajectory.points=[TrajectoryPoint(x=speed*i*.05+.5*(i*.05)**2,
                                                   velocity=speed+i*.05,acceleration=1.) for i in range(80)]
                state.publish(m);path.publish(trajectory);request.publish(req);time.sleep(.02)
            first=next(m for m in candidates[start:] if m.valid and m.state_frame==frame)
            self.assertLessEqual(abs(first.steering-steering),.055,
                                 'first prepared command must respect steering slew from actual applied command')
            if speed*3.6>5:
                self.assertEqual(first.accel,0.,'an accelerating reference cannot request throttle above the measured cap')
                self.assertGreater(first.brake,0.,'the acceleration ceiling must brake inside the MPC solution')


if __name__=='__main__':
    rospy.init_node('localization_seed_test')
    rostest.rosrun('asmc_planner','localization_seed',SeedTest)
