#!/usr/bin/env python3
"""Route entry is independent of GPS loss and prepared-command availability."""
import time,unittest,threading
import rospy,rostest
from std_msgs.msg import UInt8,Bool
from morai_msgs.msg import GPSMessage,CtrlCmd
from asmc_msgs.msg import VehicleState,ControlHandoff,FrenetFeedback
class Entry(unittest.TestCase):
 def test_route_entry_and_clear(self):
  modes=[];requests=[];commands=[];prepare=[]
  rospy.Subscriber('/planning/mode',UInt8,lambda m:modes.append(m.data))
  rospy.Subscriber('/planning/jamming/handoff_request',ControlHandoff,lambda m:requests.append(m))
  rospy.Subscriber('/planning/jamming/approach',Bool,lambda m:prepare.append(m.data))
  rospy.Subscriber('/ctrl_cmd',CtrlCmd,lambda m:commands.append(m))
  gps=rospy.Publisher('/gps',GPSMessage,queue_size=1)
  state=rospy.Publisher('/planning/vehicle/state',VehicleState,queue_size=1)
  candidate=rospy.Publisher('/planning/jamming/handoff_candidate',ControlHandoff,queue_size=4)
  normal=rospy.Publisher('/ctrl_cmd/normal',CtrlCmd,queue_size=1)
  ready=rospy.Publisher('/planning/localization/recovery_ready',Bool,queue_size=1)
  feedback=rospy.Publisher('/planning/frenet_feedback_v2',FrenetFeedback,queue_size=4)
  def wait(pred):
   end=time.monotonic()+4
   while not pred():
    self.assertLess(time.monotonic(),end);time.sleep(.005)
  wait(lambda:state.get_num_connections() and gps.get_num_connections() and modes)
  stop=threading.Event()
  def normal_heartbeat():
   while not stop.wait(.02):normal.publish(CtrlCmd(longlCmdType=1,accel=.1))
  heartbeat=threading.Thread(target=normal_heartbeat,daemon=True);heartbeat.start()
  self.addCleanup(stop.set)
  def sample(x):
   gps.publish(GPSMessage(latitude=37.,longitude=126.,status=1))
   m=VehicleState(valid=True,x=x,y=0.,yaw=0.,speed=2.)
   m.header.stamp=rospy.Time.now();m.header.frame_id='map';state.publish(m)
  sample(0);normal.publish(CtrlCmd(longlCmdType=1,accel=.1));time.sleep(.08)
  self.assertEqual(modes[-1],0);self.assertFalse(prepare[-1])
  sample(49.8);wait(lambda:requests and requests[-1].valid)
  request=requests[-1]
  def offer(req):
   c=ControlHandoff(epoch=req.epoch,target_mode=req.target_mode,valid=True,longl_cmd_type=1,accel=.1)
   c.header.stamp=rospy.Time.now();c.header.frame_id='/mpc_controller' if c.target_mode==0 else rospy.get_param('/planning_mode/jamming_controller')
   c.state_stamp=c.trajectory_stamp=c.header.stamp
   c.state_frame='map' if c.target_mode==0 else rospy.get_param('/asmc_control_selector/jamming_frame_prefix')+'fixture'
   candidate.publish(c)
  offer(request);time.sleep(.03);self.assertEqual(modes[-1],0,'a prepared command cannot enter before A448')
  normal.publish(CtrlCmd(longlCmdType=1,accel=.1));sample(50.2);time.sleep(.03)
  # A delayed prewarm may have expired under test/build load. Follow the
  # current request, as a real controller does; never reuse the old epoch.
  wait(lambda:requests[-1].valid and requests[-1].target_mode==1)
  first=len(commands);offer(requests[-1]);wait(lambda:modes[-1]==1)
  self.assertTrue(all(c.brake<1 for c in commands[first:]),'entry must not add a stop')
  sample(60);time.sleep(.10);self.assertEqual(modes[-1],1,'valid GPS inside the route does not end jamming ownership')
  sample(72);time.sleep(.12);ready.publish(Bool(True));time.sleep(.02)
  for _ in range(2):
   f=FrenetFeedback(command_accepted=True,plan_valid=True,trajectory_source=FrenetFeedback.TRAJECTORY_NEW)
   f.header.stamp=rospy.Time.now();f.header.frame_id='map';feedback.publish(f);time.sleep(.025)
  wait(lambda:requests[-1].valid and requests[-1].target_mode==0)
  offer(requests[-1]);wait(lambda:modes[-1]==0)
if __name__=='__main__':
 rospy.init_node('jamming_entry_test');rostest.rosrun('asmc_planner','jamming_entry',Entry)
