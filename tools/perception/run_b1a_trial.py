#!/usr/bin/env python3
"""One CPU filter verification trial with frozen YOLO input; no vehicle command."""
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

from b0_measurements import quantiles, resource_report
from b0_resources import ResourceSampler
from run_b0_matrix import stop


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
    parser.add_argument('--source',type=Path,default=Path('/root/filter_ws'))
    args=parser.parse_args()
    job=json.loads(args.job)
    root=Path(__file__).resolve().parents[2]
    config=yaml.safe_load((root/'config/experiments/camera_lidar_b1a_v1.yaml').read_text())
    uri='http://127.0.0.1:%d'%config['conditions']['ros_master_port']
    import os
    if os.environ.get('ROS_MASTER_URI')!=uri: parser.error('isolated master required')
    args.output.mkdir(parents=True,exist_ok=False)
    fixture=args.fixtures/(job['bag_id']+'-'+job['gps_mode']+'.bag')
    with rosbag.Bag(str(fixture)) as bag:
        bag_start=bag.get_start_time(); bag_duration=bag.get_end_time()-bag_start
        topics={k:v.message_count for k,v in bag.get_type_and_topic_info().topics.items()}
    rospy.set_param('/use_sim_time',True)
    rospy.init_node('b1a_trial',anonymous=True,disable_signals=True)
    params=json.loads((args.source/'main_parameters.json').read_text())
    params.update(config['profiles'][job['profile']]); params['planner_udp_enabled']=False
    for key,value in list(params.items()):
        if isinstance(value,str):
            params[key]=re.sub(r'\$\(find ([^)]+)\)',lambda m:rospkg.RosPack().get_path(m.group(1)),value)
    node='/tunnel_filter_offline_eval' if job['phase']=='offline' else '/Main'
    for key,value in params.items(): rospy.set_param(node+'/'+key,value)
    (args.output/'parameters.json').write_text(json.dumps(params,indent=2)+'\n')
    inputs,outputs,diagnostics,logs,controls,graphs={}, {},[],[],[],[]
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
    for name,topic in [('map','/lidar/tracked_objects_map'),('nogps','/lidar_nogps'),('last','/last_lidar'),('tunnel','/lidar/tunnel/objects')]:
        subscribers.append(rospy.Subscriber(topic,TrackedObjectArray,lambda m,n=name:receive_objects(n,m),queue_size=100))
    subscribers.append(rospy.Subscriber('/lidar/debug_roi',PointCloud2,receive_roi,queue_size=50,buff_size=16777216))
    subscribers.append(rospy.Subscriber('/lidar/tunnel/diagnostics',DiagnosticArray,receive_diagnostic,queue_size=100))
    subscribers.append(rospy.Subscriber('/rosout_agg',Log,lambda m:logs.append(dict(arrival=time.monotonic(),level=m.level,name=m.name,msg=m.msg)),queue_size=1000))
    for topic in ['/ctrl_cmd','/ctrl_cmd_0']:
        subscribers.append(rospy.Subscriber(topic,CtrlCmd,lambda m,t=topic:controls.append(dict(arrival=time.monotonic(),topic=t)),queue_size=100))
    sampler=ResourceSampler(uri,[node],config['conditions']['resource_sample_sec']); sampler.start()
    processes=[]; streams=[]; error=None; command=[]; child_code=None; started=time.monotonic()
    report=dict(job=job,fixture=str(fixture),fixture_topics=topics,bag_duration_sec=bag_duration,
                frozen_yolo=True,raw_bag_unchanged=True,diagnostic_observer_cost_included=True,
                full_camera_stack_performance_claim=False,operational_adoption='none')
    try:
        stdout=(args.output/'node.log').open('w'); streams.append(stdout)
        if job['phase']=='offline':
            command=[str(args.source/'devel/lib/Main/tunnel_filter_offline_eval'),'--bag',str(fixture),
                     '--out',str(args.output),'--processing-delay',str(config['conditions']['offline_processing_delay_sec'])]
            if job['profile'] in ('T1','T3','T4'):
                command.append('--dump-clouds')
            child=subprocess.Popen(command,stdout=stdout,stderr=subprocess.STDOUT,start_new_session=True)
            processes.append(child); child_code=child.wait(timeout=600)
            if child_code != 0: raise RuntimeError('offline evaluator exited %d'%child_code)
            summary_path=args.output/'summary.json'
            native=json.loads(summary_path.read_text()) if summary_path.exists() else None
            rows=[json.loads(s) for s in (args.output/'frames.jsonl').read_text().splitlines()] if (args.output/'frames.jsonl').exists() else []
            report.update(native=native,processed_frames=len(rows),all_scans_accounted=bool(native and native['frames']==topics['/velodyne_points']),
                          active_frames=sum(r['B']['active'] for r in rows),
                          pipeline_ms=quantiles([r['A']['ms'][0] for r in rows]),
                          snapshot_fields_rounded_by_existing_evaluator=True)
        else:
            launch=["roslaunch",str(args.source/'b1a_main.launch')]+[k+':='+('true' if v is True else 'false' if v is False else str(v)) for k,v in config['profiles'][job['profile']].items()]
            command=launch
            child=subprocess.Popen(launch,stdout=stdout,stderr=subprocess.STDOUT,start_new_session=True)
            processes.append(child)
            deadline=time.monotonic()+20
            while True:
                state=rosgraph.Master('/b1a_trial').getSystemState()
                subscribed={topic for topic,names in state[1] if '/Main' in names}
                if {'/gps','/imu','/velodyne_points','/camera/image/compressed','/Ego_topic'} <= subscribed: break
                if child.poll() is not None or time.monotonic()>deadline: raise RuntimeError('Main startup incomplete')
                time.sleep(.05)
            baglog=(args.output/'bag.log').open('w'); streams.append(baglog)
            player=subprocess.Popen(['rosbag','play','--clock','--rate','1.0','--delay','1.0',str(fixture)],
                         stdout=baglog,stderr=subprocess.STDOUT,start_new_session=True)
            processes.append(player)
            deadline=time.monotonic()+bag_duration+20
            while player.poll() is None:
                state=rosgraph.Master('/b1a_trial').getSystemState()
                graphs.append(dict(arrival=time.monotonic(),publishers=state[0],subscribers=state[1]))
                if child.poll() is not None: raise RuntimeError('Main exited during playback')
                if time.monotonic()>deadline: raise RuntimeError('bag playback timeout')
                time.sleep(.5)
            report['player_return_code']=player.returncode
            if player.returncode != 0: raise RuntimeError('bag player exited %d'%player.returncode)
            time.sleep(.5)
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
        raw={r['stamp_ns']//1000000*1000000 for r in inputs.get('lidar',[])}
        published=[r['stamp_ns'] for r in outputs.get('processed_roi',[])]
        report.update(raw_input_frames=len(inputs.get('lidar',[])),processed_unique_stamps_ns=sorted(set(published)),
            unprocessed_input_stamps_ns=sorted(raw-set(published)),duplicate_processed_stamps=len(published)-len(set(published)),
            processed_source='debug_roi: published only when LidarProcess accepts a new raw scan',
            observer_input_complete=len(inputs.get('lidar',[]))==topics['/velodyne_points'],
            active_diagnostics=sum(d['values'].get('tunnel_active')=='true' for d in diagnostics))
        yolo_publishers=[names for g in graphs for topic,names in g['publishers'] if topic=='/YOLO/BBox']
        report['yolo_publisher_counts']=sorted(set(len(names) for names in yolo_publishers))
        report['single_frozen_yolo_publisher']=bool(yolo_publishers) and all(len(names)==1 and names[0].startswith('/play_') for names in yolo_publishers)
        if not report['single_frozen_yolo_publisher']: error=error or 'frozen YOLO publisher missing or duplicated'
    report['trial_wall_duration_sec']=time.monotonic()-started
    report.update(command=command,child_return_code=child_code,error=error,resources=resource_report(resources),
                  public_control_messages=len(controls),resource_interval_sec=quantiles([b['arrival']-a['arrival'] for a,b in zip(resources,resources[1:])]),
                  complete=error is None and (report.get('all_scans_accounted',False) if job['phase']=='offline' else report.get('observer_input_complete',False)) and not controls)
    (args.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(job=job,complete=report['complete'],error=error)),flush=True)
    return 0 if report['complete'] else 1


if __name__=='__main__': raise SystemExit(main())
