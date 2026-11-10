#!/usr/bin/env python3
"""Replay raw recorded motion/OBBs with GPS outage+recovery, without a bridge.

Own master, no physical lease or simulator connection, no control publishers.
The parent registered experiment owns append-only replay subdirectories.
"""
import argparse
from collections import Counter
import copy
import hashlib
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
from localization_assessment import completed_gps_recovery


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('root',type=Path);p.add_argument('bag',type=Path);p.add_argument('--trial',required=True)
    p.add_argument('--rate',type=float,default=1.)
    p.add_argument('--yaw-source',choices=['orientation','rate'],default='orientation')
    p.add_argument('--motion-reference-offset',type=float,default=1.4)
    p.add_argument('--motion-turn-gain',type=float,default=1.54)
    p.add_argument('--static-avoidance',action='store_true',
                   help='replay recorded observed OBBs through the M3 preview; no control output')
    p.add_argument('--speed-cap-kph',type=float,default=5.)
    p.add_argument('--dynamic-obstacles',action='store_true')
    p.add_argument('--shadow-lane-rules',action='store_true')
    p.add_argument('--path-continuity',choices=['on','off'],default='off')
    p.add_argument('--existing-master',default='',help='explicit private master with no active input/control producers')
    p.add_argument('--dashboard-port',type=int,default=0)
    a=p.parse_args()
    if not .5<=a.rate<=4:raise ValueError('replay rate must be in [0.5,4]')
    if not 0<a.speed_cap_kph<=20:raise ValueError('replay speed cap must be in (0,20]')
    from localization_run import active_registration
    registration=active_registration(a.root)
    if registration['status']!='active' or Path(a.trial).name!=a.trial:raise ValueError('active registered experiment required')
    output=a.root/'replays'/a.trial;output.parent.mkdir(exist_ok=True);output.mkdir(exist_ok=False)
    checkout=Path(__file__).resolve().parents[2]
    patch=subprocess.check_output(['git','diff','HEAD','--binary'],cwd=checkout)
    (output/'source.patch').write_bytes(patch)
    (output/'input.json').write_text(json.dumps(dict(path=str(a.bag),sha256=hashlib.sha256(a.bag.read_bytes()).hexdigest(),
        revision=subprocess.check_output(['git','rev-parse','HEAD'],cwd=checkout,text=True).strip(),
        patch_sha256=hashlib.sha256(patch).hexdigest(),rate=a.rate,yaw_source=a.yaw_source,motion_reference_offset_m=a.motion_reference_offset,
        motion_turn_gain_m2ps=a.motion_turn_gain,
        static_avoidance=a.static_avoidance,
        dynamic_obstacles=a.dynamic_obstacles,shadow_lane_rules=a.shadow_lane_rules,path_continuity=a.path_continuity,
        speed_cap_kph=a.speed_cap_kph,
        library_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in
            (Path('/devel/lib/libasmc_shadow_localization_core.so'),
             Path('/devel/lib/libasmc_local_planner_core.so'),
             Path('/devel/lib/libasmc_planner_frenet_core.so'))},
        binary_sha256=hashlib.sha256(Path('/devel/lib/asmc_planner/asmc_shadow_localization_node').read_bytes()).hexdigest()),indent=2))
    from localization_observer import private_master_uri,ensure_dashboard,ExperimentReporter
    with socket.socket() as sock:sock.bind(('127.0.0.1',0));port=sock.getsockname()[1]
    uri=private_master_uri(a.existing_master) if a.existing_master else 'http://127.0.0.1:'+str(port)
    os.environ.update(ROS_MASTER_URI=uri,ROS_HOSTNAME='127.0.0.1')
    import rosbag,rosgraph,rospy
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String
    from pyproj import Transformer
    from scen_runner.runtime.validation_episode import EpisodeRoute
    route=EpisodeRoute().curve;transform=Transformer.from_crs(4326,32652,always_xy=True)
    children=[];logs=[];counts=Counter();sources=Counter();reasons=Counter();transitions=[]
    def start(command,name):
        f=(output/(name+'.log')).open('x');logs.append(f)
        child=subprocess.Popen(command,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);children.append(child)
        return child
    decisions=(output/'decisions.jsonl').open('x')
    geometry=(output/'geometry.jsonl').open('x')
    def trace(message):
        record=json.loads(message.data);sources[record['source']]+=1;reasons[record['reason']]+=1
        if not transitions or transitions[-1]['source']!=record['source']:
            transitions.append(dict(source=record['source'],stamp=record['stamp']))
        decisions.write(message.data+'\n');decisions.flush()
    try:
        if not a.existing_master:start(['roscore','-p',str(port)],'master')
        master=rosgraph.Master('/localization_bag_replay')
        deadline=time.monotonic()+10
        while True:
            try:master.getPid();break
            except OSError:
                if time.monotonic()>deadline:raise
                time.sleep(.1)
        occupied={t:n for t,n in master.getSystemState()[0] if n and t in
                  ('/ctrl_cmd','/ctrl_cmd/normal','/ctrl_cmd/jamming','/gps','/imu','/Ego_topic','/clock')}
        if occupied:raise RuntimeError('replay private graph already has input/control producers: '+str(occupied))
        restored_parameters={key:(master.hasParam(key),master.getParam(key) if master.hasParam(key) else None)
            for key in ('/use_sim_time','/localization_motion/speed_callerid','/localization/trial_id',
                        '/planning_mode/shadow_zone/path_file','/planning_mode/shadow_zone/station_start_m',
                        '/planning_mode/shadow_zone/station_end_m')}
        master.setParam('/use_sim_time',True)
        monitoring=ensure_dashboard(a.root,a.dashboard_port,uri)
        rospy.init_node('localization_bag_replay',disable_signals=True)
        reporter=ExperimentReporter(output,registration['run_id'],a.trial,'bag_replay_geometry_only',monitoring)
        master.setParam('/localization/trial_id',a.trial)
        clock=rospy.Publisher('/clock',Clock,queue_size=16,latch=True)
        topics=['/Ego_topic','/imu','/gps','/perception/lidar/tracks_local']
        if a.shadow_lane_rules:topics.append('/planning/mode')
        with rosbag.Bag(str(a.bag)) as bag:
            first=bag.get_start_time();clock.publish(Clock(rospy.Time.from_sec(first)))
            publishers={}
            for topic,message,_ in bag.read_messages(topics=topics):
                if topic not in publishers:publishers[topic]=rospy.Publisher(topic,type(message),queue_size=100)
                if len(publishers)==len(topics):break
            if len(publishers)!=len(topics):raise RuntimeError('missing recorded inputs')
            rospy.Subscriber('/planning/localization/trace',String,trace,queue_size=100)
            rospy.Subscriber('/planning/localization/debug',String,
                lambda m:(geometry.write(m.data+'\n'),geometry.flush()),queue_size=100)
            master.setParam('/localization_motion/speed_callerid','/localization_bag_replay')
            if a.shadow_lane_rules:
                master.setParam('/planning_mode/shadow_zone/path_file',str(checkout/'R_KR_PG_KATRI/2026_molit_comp_global_path.txt'))
                master.setParam('/planning_mode/shadow_zone/station_start_m',1847.744798966)
                master.setParam('/planning_mode/shadow_zone/station_end_m',1996.730791531)
            command=['roslaunch','asmc_planner','shadow_localization.launch','enable_control:=false',
                   'static_avoidance:='+str(a.static_avoidance).lower(),
                   'dynamic_obstacles:='+str(a.dynamic_obstacles).lower(),
                   'shadow_lane_rules:='+str(a.shadow_lane_rules).lower(),
                   'speed_cap_kph:='+str(a.speed_cap_kph),
                   'motion_yaw_source:='+a.yaw_source,'motion_reference_offset_m:='+str(a.motion_reference_offset),
                   'motion_turn_gain_m2ps:='+str(a.motion_turn_gain),
                   'run_id:='+registration['run_id']]
            from xml.sax.saxutils import escape
            wrapper=output/'replay.launch'
            wrapper.write_text('<launch><include file="$(find asmc_planner)/launch/shadow_localization.launch">'+
                ''.join('<arg name="'+arg.split(':=',1)[0]+'" value="'+escape(arg.split(':=',1)[1])+'"/>' for arg in command[3:])+
                '</include><param name="/localization/reuse_verified_path" value="'+str(a.path_continuity=='on').lower()+'"/></launch>')
            start(['roslaunch',str(wrapper)],'stack')
            deadline=time.monotonic()+10
            while not all(pub.get_num_connections() for pub in publishers.values()):
                if time.monotonic()>deadline:raise RuntimeError('replay input subscription timeout')
                time.sleep(.1)
            wall=time.monotonic()
            reporter.phase('running')
            for topic,message,stamp in bag.read_messages(topics=topics):
                remaining=(stamp.to_sec()-first)/a.rate-(time.monotonic()-wall)
                if remaining>0:time.sleep(remaining)
                clock.publish(Clock(stamp));counts[topic]+=1
                if topic=='/gps':
                    x,y=transform.transform(message.longitude,message.latitude)
                    station,_=route.project(x-message.eastOffset,y-message.northOffset)
                    if 1880<=station<=1985:
                        message=copy.deepcopy(message);message.status=0;message.latitude=message.longitude=0.
                        counts['masked_gps']+=1
                publishers[topic].publish(message)
            time.sleep(.2)
            command_publishers={t:n for t,n in master.getSystemState()[0] if t.startswith('/ctrl_cmd') and n}
            if command_publishers:raise RuntimeError('unexpected control publishers: '+str(command_publishers))
            reporter.finish('completed','recorded_inputs_exhausted')
    finally:
        if 'reporter' in locals() and not reporter.closed:reporter.finish('failed','replay_interrupted')
        for child in reversed(children):
            if child.poll() is None:
                os.killpg(child.pid,signal.SIGINT)
                try:child.wait(10)
                except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGTERM);child.wait(5)
        if a.existing_master and 'restored_parameters' in locals():
            for key,(existed,value) in restored_parameters.items():
                if existed:master.setParam(key,value)
                elif master.hasParam(key):master.deleteParam(key)
        decisions.close()
        geometry.close()
        for log in logs:log.close()
    verified=completed_gps_recovery([json.loads(line) for line in
        (output/'decisions.jsonl').read_text().splitlines()]) is not None
    summary=dict(scope='open-loop recorded motion/perception; no simulator or control commands; accelerated replay is not a timing benchmark',
                 static_avoidance=a.static_avoidance,
                 dynamic_obstacles=a.dynamic_obstacles,shadow_lane_rules=a.shadow_lane_rules,path_continuity=a.path_continuity,
                 speed_cap_kph=a.speed_cap_kph,
                 rate=a.rate,yaw_source=a.yaw_source,
                 input=str(a.bag),counts=counts,sources=sources,reasons=reasons,transitions=transitions,
                 gps_dr_recovery=verified)
    summary['monitoring']=monitoring
    summary['private_parameters_restored']=bool(a.existing_master)
    (output/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary),flush=True)


if __name__=='__main__':main()
