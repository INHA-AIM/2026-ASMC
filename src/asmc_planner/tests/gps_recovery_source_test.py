#!/usr/bin/env python3
import time,unittest
import rospy,rostest
from morai_msgs.msg import GPSMessage
from asmc_msgs.msg import VehicleState
from std_msgs.msg import UInt8

class RecoveryTest(unittest.TestCase):
 def test_recovery_needs_consistent_position(self):
  modes=[];subscriber=rospy.Subscriber('/planning/mode',UInt8,lambda m:modes.append(m.data))
  gps=rospy.Publisher('/test/recovery/gps',GPSMessage,queue_size=1)
  state=rospy.Publisher('/test/recovery/state',VehicleState,queue_size=1)
  deadline=time.monotonic()+5
  while not (modes and gps.get_num_connections() and state.get_num_connections()):
   self.assertLess(time.monotonic(),deadline);time.sleep(.01)
  jam=GPSMessage();jam.header.stamp=rospy.Time.now();gps.publish(jam);time.sleep(.08)
  self.assertEqual(modes[-1],1)
  def observation(error):
   stamp=rospy.Time.now();fix=GPSMessage();fix.header.stamp=stamp
   # UTM 52N central meridian at the equator is exactly (500000,0).
   fix.latitude=0;fix.longitude=129;fix.altitude=1;fix.status=1;fix.eastOffset=500000
   ego=VehicleState();ego.header.stamp=stamp;ego.header.frame_id='map'
   ego.source=VehicleState.SOURCE_GPS_IMU;ego.valid=True;ego.x=-.35+error
   gps.publish(fix);time.sleep(.015);state.publish(ego);time.sleep(.045)
  for _ in range(4):observation(6.7)
  self.assertEqual(modes[-1],1)
  observation(0);self.assertEqual(modes[-1],1)
  observation(0);self.assertEqual(modes[-1],1)
  observation(0);self.assertEqual(modes[-1],0)

if __name__=='__main__':
 rospy.init_node('gps_recovery_source_test');rostest.rosrun('asmc_planner','gps_recovery_source_test',RecoveryTest)
