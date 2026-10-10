#!/usr/bin/env python3
import rospy,rosgraph,time,math,json,statistics,threading
from asmc_msgs.msg import VehicleState,BehaviorCommand,FrenetFeedback,Trajectory
from planning_inputs.msg import DetectedObjectArray
from morai_msgs.msg import ObjectStatusList
rospy.init_node('phase8_scheduling_test')
lock=threading.Lock(); seen={k:[] for k in ['behavior','feedback','trajectory']}
def receive(k):
 def callback(m):
  with lock: seen[k].append((time.monotonic(),m))
 return callback
subs=[rospy.Subscriber('/behavior/command_v2',BehaviorCommand,receive('behavior'),queue_size=100),rospy.Subscriber('/planning/frenet_feedback_v2',FrenetFeedback,receive('feedback'),queue_size=100),rospy.Subscriber('/planning/trajectory',Trajectory,receive('trajectory'),queue_size=100)]
pubs=[rospy.Publisher('/vehicle/state',VehicleState,queue_size=1),rospy.Publisher('/planning/objects',DetectedObjectArray,queue_size=1),rospy.Publisher('/Object_topic',ObjectStatusList,queue_size=1)]
rows=[list(map(float,s.split()[:2])) for s in open('/root/ws/R_KR_PG_KATRI/2026_molit_comp_global_path.txt') if s.strip()]
x,y=rows[400];nx,ny=rows[401];yaw=math.atan2(ny-y,nx-x)
start=time.monotonic();rate=rospy.Rate(50)
while not rospy.is_shutdown() and time.monotonic()-start<8:
 stamp=rospy.Time.now()
 state=VehicleState();state.header.stamp=stamp;state.header.frame_id='map';state.source=VehicleState.SOURCE_GPS_IMU;state.valid=True;state.x=x;state.y=y;state.yaw=yaw;state.speed=5
 objects=DetectedObjectArray();objects.header.stamp=stamp;objects.header.frame_id='map'
 morai=ObjectStatusList();morai.header.stamp=stamp
 for pub,m in zip(pubs,[state,objects,morai]):pub.publish(m)
 rate.sleep()
with lock: window={k:[v for v in a if start+3<=v[0]<=start+7] for k,a in seen.items()}
metrics={}
for k,target in [('behavior',10),('feedback',20),('trajectory',20)]:
 a=window[k];intervals=[q[0]-p[0] for p,q in zip(a,a[1:])]
 assert len(a)>=target*3,(k,'too few messages',len(a))
 hz=(len(a)-1)/(a[-1][0]-a[0][0]);median=statistics.median(intervals)
 metrics[k]={'count':len(a),'hz':round(hz,3),'median_interval':round(median,5)}
 assert abs(hz-target)<1.0,(k,hz)
 assert abs(median-1.0/target)<.01,(k,median)
master=rosgraph.Master(rospy.get_name());publishers,subscribers,_=master.getSystemState();pub=dict(publishers);sub=dict(subscribers)
for t in ['/behavior/command_v2','/planning/trajectory','/planning/feedback','/planning/frenet_feedback_v2','/planning/markers','/behavior/markers','/behavior/feature_debug']:
 assert pub.get(t)==['/asmc_planner'],(t,pub.get(t))
# Test config disconnects the ROS command input. Feedback is direct as well.
assert '/asmc_planner' not in sub.get('/behavior/command_v2',[]),sub
assert '/asmc_planner' not in sub.get('/planning/frenet_feedback_v2',[]),sub
assert '/asmc_planner' in sub.get('/test/disconnected_external_commands',[]),sub
ids={m.command_id for _,m in window['behavior']}
assert ids and any(m.command_id in ids and m.command_accepted for _,m in window['feedback'])
metrics['direct_with_command_ROS_input_disconnected']=True
metrics['feedback_has_no_internal_ROS_subscriber']=True
metrics['single_publisher_each_output']=True
metrics['accepted_command_ids']=sorted(ids)
metrics['valid_trajectories']=sum(m.valid for _,m in window['trajectory'])
print(json.dumps(metrics,indent=2))
