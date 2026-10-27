#!/usr/bin/env python3
"""Isolated real pipeline -> common planning observation -> input contract test.

Owns a fresh ROS master and only its own subprocess groups. No simulator,
vehicle-control launch, production master, data collection or model inference.
All synthetic input/logs/config stay in a temporary unit-test directory.
"""
import collections
import copy
import importlib.util
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import threading
import time


def require(predicate, description, timeout=12):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            print('PASS ' + description, flush=True)
            return
        time.sleep(.03)
    raise AssertionError(description)


def main():
    with socket.socket() as probe:
        probe.bind(('127.0.0.1', 0))
        port = probe.getsockname()[1]
    os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:' + str(port)
    os.environ['ROS_HOSTNAME'] = 'localhost'
    os.environ.pop('ROS_IP', None)
    import numpy as np
    import yaml
    import rospy
    import rosgraph
    from asmc_msgs.msg import LidarTrackingObservation, LidarPlanningObservation
    from planning_inputs.msg import DetectedObjectArray
    from sensor_msgs.msg import Imu, PointCloud2, PointField
    from morai_msgs.msg import GPSMessage, EgoVehicleStatus
    from std_srvs.srv import Trigger
    from diagnostic_msgs.msg import DiagnosticArray

    root = Path(__file__).resolve().parents[3]
    package = root/'src/asmc_lidar'
    processes, handles = [], []
    running = threading.Event()
    state = dict(gps=True, imu=True)
    lock = threading.RLock()
    local, mapped, output, diagnostic, pipeline_diagnostic = (collections.deque(maxlen=512) for _ in range(5))
    thread = None
    with tempfile.TemporaryDirectory(prefix='asmc-common-lidar-') as temporary:
        temporary = Path(temporary)

        def spawn(args, name):
            handle = (temporary/(name+'.log')).open('w')
            handles.append(handle)
            process = subprocess.Popen(args, stdout=handle, stderr=subprocess.STDOUT, start_new_session=True)
            processes.append(process)
            return process

        def stop(process):
            if process.poll() is not None:
                return
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=6)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=4)

        try:
            spawn(['roscore', '-p', str(port)], 'master')
            master = rosgraph.Master('/common_lidar_test')
            def master_ready():
                try:
                    return bool(master.getPid())
                except Exception:
                    return False
            require(master_ready, 'isolated master', 10)
            rospy.init_node('common_lidar_test', anonymous=True, disable_signals=True)
            config = yaml.safe_load((package/'config/lidar_pipeline.yaml').read_text())
            # Geometry contract fixture is independent of road placement; the
            # actual map/height delivery filters have their own runtime suite.
            config['hd_map_audit']['mode'] = 'off'
            config_path = temporary/'pipeline.yaml'
            config_path.write_text(yaml.safe_dump(config))
            ns = '/perception/lidar'
            def pipeline():
                return spawn(['roslaunch', 'asmc_lidar', 'lidar.launch',
                    'pipeline_config:='+str(config_path), 'fusion:=false',
                    'debug:=false', 'rviz:=false'], 'pipeline-'+str(len(processes)))
            node = pipeline()
            spawn(['roslaunch', 'planning_inputs', 'objects.launch',
                'output_topic:=/contract/planning_objects'], 'adapter')
            def store(target):
                def callback(message):
                    with lock:
                        target.append(copy.deepcopy(message))
                return callback
            subscribers = [
                rospy.Subscriber(ns+'/tracks_local', LidarTrackingObservation, store(local), queue_size=2),
                rospy.Subscriber(ns+'/planning_observations_map', LidarPlanningObservation, store(mapped), queue_size=2),
                rospy.Subscriber('/contract/planning_objects', DetectedObjectArray, store(output), queue_size=2),
                rospy.Subscriber('/planning_objects_adapter/diagnostics', DiagnosticArray, store(diagnostic), queue_size=1),
                rospy.Subscriber(ns+'/diagnostics', DiagnosticArray, store(pipeline_diagnostic), queue_size=1)]
            pubs = dict(points=rospy.Publisher('/velodyne_points', PointCloud2, queue_size=1),
                        imu=rospy.Publisher('/imu', Imu, queue_size=2),
                        can=rospy.Publisher('/Ego_topic', EgoVehicleStatus, queue_size=2),
                        gps=rospy.Publisher('/gps', GPSMessage, queue_size=2))
            require(lambda: all(p.get_num_connections() for p in pubs.values()), 'sensor subscriptions')
            # Pure UTM helper from the existing regression fixture; no live GT.
            spec = importlib.util.spec_from_file_location('lidar_fixture', package/'test/ros/lidar_pipeline_node_test.py')
            fixture = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(fixture)
            latitude, longitude = fixture.utm_inverse(302595.+.35, 4124145.)
            xx, yy = np.meshgrid(np.arange(-5,25,.4), np.arange(-8,8,.4))
            ground = np.stack([xx.ravel(), yy.ravel(), np.full(xx.size,-.36)], axis=1)
            faces = []
            for x in (7.,9.):
                y,z=np.meshgrid(np.linspace(-.8,.8,15),np.linspace(.1,1.2,12))
                faces.append(np.stack([np.full(y.size,x),y.ravel(),z.ravel()],axis=1))
            points = np.concatenate([ground,*faces]) - np.array([.8,0.,1.45])
            points = np.column_stack([points,np.ones(len(points))]).astype(np.float32)
            cloud=PointCloud2();cloud.header.frame_id='velodyne';cloud.height=1;cloud.width=len(points)
            cloud.fields=[PointField(name,4*i,PointField.FLOAT32,1) for i,name in enumerate(('x','y','z','intensity'))]
            cloud.point_step=16;cloud.row_step=16*len(points);cloud.is_dense=True;cloud.data=points.tobytes()

            def publish():
                tick=0
                while running.is_set() and not rospy.is_shutdown():
                    now=rospy.Time.now()
                    if state['imu']:
                        imu=Imu();imu.header.stamp=now;imu.orientation.w=1.;imu.linear_acceleration.z=9.81
                        pubs['imu'].publish(imu)
                    can=EgoVehicleStatus();can.header.stamp=now;pubs['can'].publish(can)
                    if tick%10==0:
                        gps=GPSMessage();gps.header.stamp=now;gps.status=1 if state['gps'] else 0
                        gps.latitude=latitude if state['gps'] else 0.;gps.longitude=longitude if state['gps'] else 0.
                        gps.altitude=1.25;gps.eastOffset=302595.;gps.northOffset=4124145.;pubs['gps'].publish(gps)
                    if tick%5==2:
                        cloud.header.stamp=now;pubs['points'].publish(cloud)
                    tick+=1;time.sleep(.02)
            running.set();thread=threading.Thread(target=publish,daemon=True);thread.start()

            def latest(messages):
                with lock:
                    return messages[-1] if messages else None
            require(lambda: latest(output) is not None and bool(latest(output).objects), 'real pipeline reaches private planning objects')
            require(lambda: len(output)>=12, 'continuous common observation consumption')
            with lock:
                by_stamp={m.observation.objects.header.stamp.to_nsec():m for m in mapped}
                pairs=[(by_stamp[o.header.stamp.to_nsec()],o) for o in output if o.header.stamp.to_nsec() in by_stamp]
            assert len(pairs)>=5
            for source,converted in pairs:
                assert len(source.motion_states)==len(source.observation.objects.objects)==len(converted.objects)
                for common,motion,private in zip(source.observation.objects.objects,source.motion_states,converted.objects):
                    assert private.id>=0  # private IDs intentionally change across producer epochs
                    assert common.center==private.center and common.obb_corners==private.obb_corners
                    assert motion==private.motion_state
                    assert private.velocity_unavailable== (not common.absolute_velocity_valid)
                    if common.absolute_velocity_valid and motion!=source.LOW_THREAT:
                        assert common.absolute_velocity_mps.x==private.vx and common.absolute_velocity_mps.y==private.vy
            print('PASS source stamp, IDs, full OBB, applied velocity and four-state delivery',flush=True)
            require(lambda: latest(pipeline_diagnostic) is not None, 'pipeline diagnostics without debug')
            values={v.key:v.value for s in latest(pipeline_diagnostic).status for v in s.values}
            assert values['debug_enabled']=='false'
            publications, subscriptions, _=master.getSystemState()
            assert not any(topic.startswith(ns+'/debug/') for topic,nodes in publications)
            assert not any(topic in (ns+'/trace',ns+'/objects_local',ns+'/objects_map')
                           for topic,nodes in subscriptions)
            # The planner alone must demand planning_observations_map: remove
            # the fixture's subscriptions, leaving only planning_inputs.
            subscribers[0].unregister();subscribers[1].unregister()
            before=len(output)
            require(lambda: len(output)>=before+10, 'planner-only demand supplies new objects without web/trace/RViz')
            print('PASS debug=false, no stage/trace/array consumer, geometry/velocity delivery continues',flush=True)
            subscribers[0]=rospy.Subscriber(ns+'/tracks_local', LidarTrackingObservation, store(local), queue_size=2)
            subscribers[1]=rospy.Subscriber(ns+'/planning_observations_map', LidarPlanningObservation, store(mapped), queue_size=2)
            state['gps']=False
            require(lambda: latest(mapped).observation.objects.status==4,'GPS0 reports POSE_UNAVAILABLE')
            time.sleep(.5)
            before=len(output);stamp=latest(local).observation.objects.header.stamp
            time.sleep(.5)
            assert len(output)==before
            assert latest(local).observation.objects.header.stamp>stamp
            assert latest(local).observation.objects.objects
            assert not latest(local).observation.objects.objects[0].absolute_velocity_valid
            print('PASS GPS loss preserves local and withholds map; no fabricated empty planning frame',flush=True)
            state['gps']=True
            require(lambda: len(output)>before,'GPS recovery resumes planner input')
            old=latest(local).observation
            reset=rospy.ServiceProxy(ns+'/lidar_pipeline/reset',Trigger)
            assert reset().success
            require(lambda: latest(local).observation.generation>old.generation,'explicit reset changes generation')
            require(lambda: output and latest(output).header.stamp>=latest(local).observation.objects.header.stamp-rospy.Duration(.5),'reset resumes direct consumption')
            old_session=latest(local).observation.producer_session_id
            stop(node);node=pipeline()
            require(lambda: latest(local).observation.producer_session_id!=old_session,'producer restart changes session')
            stamp=latest(local).observation.objects.header.stamp
            require(lambda: latest(output).header.stamp>=stamp,'new session resumes planning')
            state['imu']=False
            require(lambda: latest(local).observation.objects.status==3,'IMU loss reports INPUT_INVALID')
            time.sleep(.5);before=len(output);time.sleep(.5);assert len(output)==before
            state['imu']=True
            require(lambda: len(output)>before,'IMU recovery resumes planning')
            require(lambda: latest(diagnostic) is not None,'bounded input diagnostics')
            publishers=dict(master.getSystemState()[0])
            assert len(publishers.get(ns+'/planning_observations_map',[]))==1
            assert not any(publishers.get(t) for t in ('/last_lidar','/YOLO/BBox','/ctrl_cmd_0'))
            assert not publishers.get('/planning/input/tracked_objects')
            print('PASS one producer, no legacy wire/JSON/control dependency',flush=True)
            for subscriber in subscribers:
                subscriber.unregister()
        except BaseException:
            for path in temporary.glob('*.log'):
                print('\n'+path.name+'\n'+'\n'.join(path.read_text(errors='replace').splitlines()[-25:]),flush=True)
            raise
        finally:
            running.clear()
            if thread is not None:
                thread.join(timeout=2)
            for process in reversed(processes):
                stop(process)
            for handle in handles:
                handle.close()


if __name__=='__main__':
    main()
