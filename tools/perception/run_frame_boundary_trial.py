#!/usr/bin/env python3
"""One isolated frame-boundary trial with frozen YOLO; never controls a vehicle."""
import argparse
import json
import math
from pathlib import Path
import re
import subprocess
import threading
import time

import rosbag
import rosgraph
import rospkg
import rospy
import yaml
from diagnostic_msgs.msg import DiagnosticArray
from morai_msgs.msg import CtrlCmd, GPSMessage, EgoVehicleStatus
from Msg.msg import TrackedObjectArray, Yolo_bbox_arr
from rosgraph_msgs.msg import Log
from sensor_msgs.msg import PointCloud2, Imu, CompressedImage

from asmc_msgs.msg import LidarObjectArray
from b1a_protocol import launch_values, test_launch, require_stopped_simulator
from b0_measurements import quantiles, resource_report
from b0_resources import ResourceSampler
from run_b0_matrix import stop
from frame_boundary_measurements import frame_totals, observation_latencies


def geometry(message):
    return [dict(id=o.id, classification=o.classification, yolo_class=o.yolo_class,
                 center=[o.obb_center.x,o.obb_center.y,o.obb_center.z],
                 dimensions=[o.length,o.width,o.height], age=o.age, miss_count=o.miss_count,
                 speed_kph=o.speed_kph, is_dynamic=o.is_dynamic) for o in message.objects]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--job',required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--fixtures',type=Path,required=True)
    parser.add_argument('--source',type=Path,default=Path('/root/ws'))
    parser.add_argument('--host-state',type=Path,required=True)
    args=parser.parse_args()
    job=json.loads(args.job)
    root=Path(__file__).resolve().parents[2]
    config=yaml.safe_load((root/'config/experiments/lidar_camera_frame_boundaries_v1.yaml').read_text())
    uri='http://127.0.0.1:%d'%config['conditions']['ros_master_port']
    import os
    if os.environ.get('ROS_MASTER_URI')!=uri: parser.error('isolated master required')
    def host_check():
        require_stopped_simulator(json.loads(args.host_state.read_text()),time.time())
    host_check()
    expected_main=(args.source/'src/lidar+sensorfusion/src/Main').resolve()
    if Path(rospkg.RosPack().get_path('Main')).resolve()!=expected_main:
        parser.error('Main source differs from the requested worktree; source outer then nested workspace')
    args.output.mkdir(parents=True,exist_ok=False)
    fixture=args.fixtures/(job['bag_id']+'-'+job['gps_mode']+'.bag')
    with rosbag.Bag(str(fixture)) as bag:
        bag_start=bag.get_start_time(); bag_duration=bag.get_end_time()-bag_start
        topics={k:v.message_count for k,v in bag.get_type_and_topic_info().topics.items()}
    rospy.set_param('/use_sim_time',True)
    rospy.init_node('frame_boundary_trial',anonymous=True,disable_signals=True)
    params=launch_values((root/'src/lidar+sensorfusion/src/Main/main.launch').read_text())
    profile_args=dict(config['profiles'][job['profile']])
    if job['suite']=='diagnostic': profile_args['fusion_trace_path']=str(args.output/'fusion.jsonl')
    params.update(profile_args); params['planner_udp_enabled']=False
    for key,value in list(params.items()):
        if isinstance(value,str):
            params[key]=re.sub(r'\$\(find ([^)]+)\)',lambda m:rospkg.RosPack().get_path(m.group(1)),value)
    node='/Main'
    for key,value in params.items(): rospy.set_param(node+'/'+key,value)
    (args.output/'parameters.json').write_text(json.dumps(params,indent=2)+'\n')
    inputs,outputs,diagnostics,logs,controls,graphs={}, {},[],[],[],[]
    subscription=job['subscribers']
    lock=threading.RLock(); subscribers=[]
    def basic(message):
        return dict(arrival=time.monotonic(),ros_ns=rospy.Time.now().to_nsec(),
                    stamp_ns=message.header.stamp.to_nsec(),frame_id=message.header.frame_id)
    def receive_input(name,message):
        row=basic(message)
        if name=='gps': row.update(status=message.status,latitude=message.latitude,longitude=message.longitude,altitude=message.altitude)
        if name=='lidar': row['points']=message.width*message.height
        with lock: inputs.setdefault(name,[]).append(row)
    def receive_objects(name,message):
        row=basic(message); row['objects']=geometry(message)
        with lock: outputs.setdefault(name,[]).append(row)
    def receive_typed(name,message):
        row=basic(message);row.update(status=message.status,reason=message.reason,
            stamp_provenance=message.stamp_provenance,objects=[dict(id=o.track_id,
                detection_index=o.detection_index,classification=o.classification,yolo_class=o.camera_class,
                center=[o.center.x,o.center.y,o.center.z],
                corners=[[p.x,p.y,p.z] for p in o.obb_corners],yaw=o.yaw,
                dimensions=[o.length,o.width,o.height],age=o.age,miss_count=o.miss_count,
                relative_velocity_valid=o.relative_velocity_valid,absolute_velocity_valid=o.absolute_velocity_valid,
                dynamic_valid=o.dynamic_valid,point_count=o.source_point_count) for o in message.objects])
        with lock: outputs.setdefault(name,[]).append(row)
    for name in ('local','map'):
        if subscription in (name,'both'):
            subscribers.append(rospy.Subscriber('/perception/lidar/objects_'+name,LidarObjectArray,
                lambda m,n=name:receive_typed('typed_'+n,m),queue_size=100))
    def receive_roi(message):
        row=basic(message); row['points']=message.width*message.height
        with lock: outputs.setdefault('processed_roi',[]).append(row)
    def receive_diagnostic(message):
        with lock:
            for status in message.status:
                diagnostics.append(dict(arrival=time.monotonic(),ros_ns=rospy.Time.now().to_nsec(),
                    stamp_ns=message.header.stamp.to_nsec(),name=status.name,level=status.level,
                    values={v.key:v.value for v in status.values}))
    for name,topic,typ in [('gps','/gps',GPSMessage),('imu','/imu',Imu),('ego','/Ego_topic',EgoVehicleStatus),
                           ('lidar','/velodyne_points',PointCloud2),('front','/camera/image/compressed',CompressedImage),
                           ('yolo','/YOLO/BBox',Yolo_bbox_arr)]:
        subscribers.append(rospy.Subscriber(topic,typ,lambda m,n=name:receive_input(n,m),queue_size=100,buff_size=16777216))
    for name,topic in ([] if job.get('suite')=='cost' else [('map','/lidar/tracked_objects_map'),('nogps','/lidar_nogps'),('last','/last_lidar'),('tunnel','/lidar/tunnel/objects')]):
        subscribers.append(rospy.Subscriber(topic,TrackedObjectArray,lambda m,n=name:receive_objects(n,m),queue_size=100))
    if job.get('suite')!='cost':
        subscribers.append(rospy.Subscriber('/lidar/debug_roi',PointCloud2,receive_roi,queue_size=50,buff_size=16777216))
    subscribers.append(rospy.Subscriber('/perception/lidar/diagnostics',DiagnosticArray,receive_diagnostic,queue_size=100))
    subscribers.append(rospy.Subscriber('/rosout_agg',Log,lambda m:logs.append(dict(arrival=time.monotonic(),level=m.level,name=m.name,msg=m.msg)),queue_size=1000))
    for topic in ['/ctrl_cmd','/ctrl_cmd_0']:
        subscribers.append(rospy.Subscriber(topic,CtrlCmd,lambda m,t=topic:controls.append(dict(arrival=time.monotonic(),topic=t)),queue_size=100))
    sampler=ResourceSampler(uri,[node],config['conditions']['resource_sample_sec']); sampler.start()
    processes=[]; streams=[]; error=None; command=[]; child_code=None; started=time.monotonic()
    report=dict(job=job,fixture=str(fixture),fixture_topics=topics,bag_duration_sec=bag_duration,
                frozen_yolo=True,raw_bag_unchanged=True,diagnostic_observer_cost_included=True,
                debug_cloud_observer=job.get('suite')!='cost',typed_subscription=subscription,
                full_camera_stack_performance_claim=False,operational_adoption='none')
    try:
        stdout=(args.output/'node.log').open('w'); streams.append(stdout)
        if job['phase']!='realtime': raise ValueError('realtime phase required')
        launch_path=args.output/'trial.launch'
        launch_path.write_text(test_launch((root/'src/lidar+sensorfusion/src/Main/main.launch').read_text()))
        launch=["roslaunch",str(launch_path)]+[k+':='+('true' if v is True else 'false' if v is False else str(v)) for k,v in profile_args.items()]
        command=launch
        child=subprocess.Popen(launch,stdout=stdout,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(child)
        deadline=time.monotonic()+20
        while True:
            host_check()
            state=rosgraph.Master('/frame_boundary_trial').getSystemState()
            subscribed={topic for topic,names in state[1] if '/Main' in names}
            if {'/gps','/imu','/velodyne_points','/camera/image/compressed','/Ego_topic'} <= subscribed: break
            if child.poll() is not None or time.monotonic()>deadline: raise RuntimeError('Main startup incomplete')
            time.sleep(.05)
        baglog=(args.output/'bag.log').open('w'); streams.append(baglog)
        host_check()
        player=subprocess.Popen(['rosbag','play','--clock','--rate','1.0','--delay','1.0',str(fixture)],
                     stdout=baglog,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(player)
        deadline=time.monotonic()+bag_duration+20
        while player.poll() is None:
            host_check()
            state=rosgraph.Master('/frame_boundary_trial').getSystemState()
            graphs.append(dict(arrival=time.monotonic(),publishers=state[0],subscribers=state[1]))
            if child.poll() is not None: raise RuntimeError('Main exited during playback')
            if time.monotonic()>deadline: raise RuntimeError('bag playback timeout')
            time.sleep(.5)
        report['player_return_code']=player.returncode
        if player.returncode != 0: raise RuntimeError('bag player exited %d'%player.returncode)
        time.sleep(1.2)
        child_code=child.poll()
    except Exception as exc:
        error=str(exc)
    except KeyboardInterrupt:
        error='trial interrupted; incomplete attempt retained'
    finally:
        for process in reversed(processes): stop(process)
        for stream in streams: stream.close()
        time.sleep(.1)
        resources=sampler.close()
        for subscriber in subscribers: subscriber.unregister()
    observed=dict(inputs=inputs,outputs=outputs,diagnostics=diagnostics,rosout=logs,public_controls=controls,graph_samples=graphs)
    (args.output/'observations.json').write_text(json.dumps(observed,indent=2)+'\n')
    (args.output/'resources.json').write_text(json.dumps(resources,indent=2)+'\n')
    if job['phase']=='realtime':
        raw={r['stamp_ns'] if job['profile']!='legacy' else r['stamp_ns']//1000000*1000000
             for r in inputs.get('lidar',[])}
        published=[r['stamp_ns'] for r in outputs.get('processed_roi',[])]
        report.update(raw_input_frames=len(inputs.get('lidar',[])),processed_unique_stamps_ns=sorted(set(published)),
            unprocessed_input_stamps_ns=sorted(raw-set(published)),duplicate_processed_stamps=len(published)-len(set(published)),
            processed_source='debug_roi: published only when LidarProcess accepts a new raw scan',
            observer_input_complete=len(inputs.get('lidar',[]))==topics['/velodyne_points'])
        yolo_publishers=[names for g in graphs for topic,names in g['publishers'] if topic=='/YOLO/BBox']
        report['yolo_publisher_counts']=sorted(set(len(names) for names in yolo_publishers))
        report['single_frozen_yolo_publisher']=bool(yolo_publishers) and all(len(names)==1 and names[0].startswith('/play_') for names in yolo_publishers)
        if not report['single_frozen_yolo_publisher']: error=error or 'frozen YOLO publisher missing or duplicated'
    for name in ('typed_local','typed_map'):
        rows=outputs.get(name,[])
        stamps=[row['stamp_ns'] for row in rows]
        report[name]=dict(frames=len(rows),duplicate_stamps=len(stamps)-len(set(stamps)),
            regressions=sum(b<=a for a,b in zip(stamps,stamps[1:])),
            statuses={str(code):sum(row['status']==code for row in rows) for code in (1,2,3,4)},
            latency_ms=quantiles(observation_latencies(rows,inputs.get('lidar',[]),
                legacy_ms=job['profile']=='legacy')['wall_ms']))
    if job.get('suite')=='cost':
        report['processed_unique_stamps_ns']=None
        report['unprocessed_input_stamps_ns']=None
        report['processed_source']='diagnostic cumulative count; no debug cloud subscriber'
    report['trial_wall_duration_sec']=time.monotonic()-started
    report.update(command=command,child_return_code=child_code,error=error,resources=resource_report(resources),
                  public_control_messages=len(controls),resource_interval_sec=quantiles([b['arrival']-a['arrival'] for a,b in zip(resources,resources[1:])]),
                  complete=error is None and report.get('observer_input_complete',False) and not controls)
    report['processed_cumulative']=max([int(d['values'].get('processed','0')) for d in diagnostics]+[0])
    report['input_invalid_cumulative']=max([int(d['values'].get('invalid','0')) for d in diagnostics]+[0])
    report['timer_diagnostics_are_partial']=True
    if job['profile']!='legacy':
        try:
            report['final_frame_totals']=frame_totals((args.output/'node.log').read_text(errors='replace'))
        except ValueError as exc:
            report['complete']=False;report['error']=report['error'] or str(exc)
    (args.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(job=job,complete=report['complete'],error=error)),flush=True)
    return 0 if report['complete'] else 1


if __name__=='__main__': raise SystemExit(main())
