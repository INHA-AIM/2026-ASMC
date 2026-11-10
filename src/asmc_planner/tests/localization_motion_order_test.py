#!/usr/bin/env python3
"""Exercise real speed/IMU callbacks, including the recorded 0.64ms reorder."""
import time,unittest
import rospy,rostest
from morai_msgs.msg import EgoVehicleStatus
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry


class MotionOrderTest(unittest.TestCase):
    def test_packet_reorder_does_not_erase_anchor_epoch_but_restart_and_gap_do(self):
        states=[]
        rospy.Subscriber('/planning/local/odometry',Odometry,states.append,queue_size=100)
        speed=rospy.Publisher('/Ego_topic',EgoVehicleStatus,queue_size=20)
        imu=rospy.Publisher('/imu',Imu,queue_size=20)
        deadline=time.monotonic()+5
        while not (speed.get_num_connections() and imu.get_num_connections()):
            self.assertLess(time.monotonic(),deadline);time.sleep(.01)
        def send(stamp,value=1.):
            v=EgoVehicleStatus();v.header.stamp=stamp;v.velocity.x=value;speed.publish(v)
            q=Imu();q.header.stamp=stamp;q.header.frame_id='imu';q.orientation.w=1.;imu.publish(q)
        def pump(seconds):
            end=time.monotonic()+seconds
            while time.monotonic()<end:
                send(rospy.Time.now()-rospy.Duration(.02));time.sleep(.02)
            time.sleep(.03)
        pump(.7);self.assertGreater(len(states),10)
        frame=states[-1].header.frame_id;old=states[-1].header.stamp-rospy.Duration(.00064)
        index=len(states);send(old,20.);pump(.5)
        self.assertTrue(states[index:]);self.assertTrue(all(s.header.frame_id==frame for s in states[index:]))
        self.assertTrue(all(abs(s.twist.twist.linear.x-1.)<1e-9 for s in states[index:]))
        # A true time epoch change still retires the old relative pose.
        send(states[-1].header.stamp-rospy.Duration(1.));pump(.4)
        self.assertNotEqual(states[-1].header.frame_id,frame)
        frame=states[-1].header.frame_id
        time.sleep(.2);pump(.4)
        self.assertNotEqual(states[-1].header.frame_id,frame,'forward input gaps still start a new invalidating epoch')


if __name__=='__main__':
    rospy.init_node('localization_motion_order_test')
    rostest.rosrun('asmc_planner','localization_motion_order',MotionOrderTest)
