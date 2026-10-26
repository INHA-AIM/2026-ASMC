#!/usr/bin/env python3
"""Full autonomy launch subscription/typed boundary smoke on its own ROS master.

No bridge, simulator, sensor collection or vehicle transport is started.
"""
import collections
import copy
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time


def main():
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', 0)); port=probe.getsockname()[1]
    os.environ['ROS_MASTER_URI']='http://127.0.0.1:'+str(port)
    os.environ['ROS_HOSTNAME']='localhost'
    os.environ.pop('ROS_IP',None)
    import rosgraph
    import rospy
    from asmc_msgs.msg import LidarTrackingObservation, LidarObject, LidarTrackingState
    from geometry_msgs.msg import Point
    from planning_inputs.msg import DetectedObjectArray
    processes=[]
    def require(predicate, description, timeout=30):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            try:
                if predicate():print('PASS '+description,flush=True);return
            except (OSError,rosgraph.MasterException):pass
            time.sleep(.05)
        raise AssertionError(description)
    with tempfile.TemporaryDirectory(prefix='asmc-autonomy-boundary-') as folder:
        handles=[]
        def spawn(command,name):
            log=(Path(folder)/(name+'.log')).open('w');handles.append(log)
            p=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            processes.append(p);return p
        try:
            spawn(['roscore','-p',str(port)],'master')
            master=rosgraph.Master('/planning_boundary_test')
            require(lambda:bool(master.getPid()),'isolated master')
            rospy.init_node('planning_boundary_test',anonymous=True,disable_signals=True)
            launch=spawn(['roslaunch','asmc_planner','autonomy_no_bridges_v2.launch',
                          'state_source:=external'],'autonomy')
            def subscriptions():return dict(master.getSystemState()[1])
            require(lambda:{'/asmc_planner','/mpc_controller'}.issubset(
                subscriptions().get('/planning/vehicle/state',[])),
                'Behavior/Frenet/MPC use the existing private vehicle state')
            require(lambda:'/asmc_planner' in subscriptions().get('/planning/objects',[]),
                    'Behavior/Frenet subscribe to private objects')
            received=collections.deque(maxlen=128)
            sub=rospy.Subscriber('/planning/objects',DetectedObjectArray,received.append,queue_size=2)
            pub=rospy.Publisher('/perception/lidar/tracks_map',LidarTrackingObservation,queue_size=2)
            require(lambda:pub.get_num_connections()>0,'default common tracking subscriber connects')
            frame=LidarTrackingObservation();o=frame.observation
            o.producer_session_id='boundary';o.generation=1;o.config_id='config';o.calibration_id='calibration'
            o.objects.header.frame_id='map';o.objects.status=o.objects.OBSERVED
            obj=LidarObject(track_id=731,observation_state=LidarObject.OBSERVED,age=1,
                classification='dynamic_obstacle',camera_class='person',length=4,width=2,height=1,
                absolute_velocity_valid=True)
            obj.center=Point(100,100,.5);obj.absolute_velocity_mps.x=2.5
            obj.obb_corners=[Point(x,y,z) for z in (0,1) for x,y in ((98,99),(102,99),(102,101),(98,101))]
            o.objects.objects=[obj];frame.tracking=[LidarTrackingState(track_id=731,cluster_center=obj.center)]
            def send():
                o.scan_sequence+=1;o.objects.header.stamp=rospy.Time.now();pub.publish(frame)
            require(lambda:(send() or bool(received)),'common frame reaches full autonomy boundary',10)
            target=received[-1].objects[0]
            assert target.id==731 and target.center==obj.center and target.obb_corners==obj.obb_corners
            assert target.vx==2.5 and target.type==2 and target.motion_state==0
            print('PASS original geometry/person category/UNKNOWN applied estimate preserved',flush=True)
            # Invalid observations must not become false empty free-space frames.
            count=len(received);o.objects.status=o.objects.POSE_UNAVAILABLE
            for _ in range(5):send();time.sleep(.08)
            assert len(received)==count
            print('PASS invalid pose withheld without fabricated empty output',flush=True)
            o.objects.status=o.objects.OBSERVED_EMPTY;o.objects.objects=[];frame.tracking=[]
            require(lambda:(send() or bool(received and not received[-1].objects)),
                    'observed-empty clears objects using a new source stamp',10)
            pubs=dict(master.getSystemState()[0])
            assert pubs['/planning/objects']==['/planning_objects_adapter']
            assert not any(name.startswith('/Main') or 'wire_bridge' in name for names in pubs.values() for name in names)
            assert '/perception/camera/objects/observations' not in pubs
            assert launch.poll() is None
            print('PASS full launch has no legacy wire or duplicate perception producer',flush=True)
            pub.unregister();sub.unregister()
        finally:
            for p in reversed(processes):
                if p.poll() is None:
                    os.killpg(p.pid,signal.SIGINT)
                    try:p.wait(timeout=12)
                    except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5)
            for handle in handles:handle.close()
if __name__=='__main__':main()
