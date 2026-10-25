#!/usr/bin/env python3
"""Controlled yellow-policy trial; holds Ego on every exit, owns its launch.

Supports pre-roundabout trials and an explicit full official lap via CP14.
Simulator truth is evaluation only; no gRPC color/position becomes a ROS input.
"""
import argparse
import base64
import hashlib
import io
import json
import math
import os
import shutil
import threading
import time
from pathlib import Path

import rospy
import numpy as np
import yaml
from asmc_msgs.msg import VehicleState,BehaviorCommand,TrafficLightDetection,UrbanFeatureDebug
from planning_inputs.msg import DetectedObjectArray
from std_msgs.msg import String,UInt8MultiArray
from morai_msgs.msg import CtrlCmd
from scen_runner.config import REPO_ROOT,repo_path,load_config
from scen_runner.morai.clock_contract import ClockGuard
from scen_runner.morai.signal_trial_coordinates import Coordinates
from scen_runner.orchestration.process import OwnedProcess
from scen_runner.records.control_binding import decode
from scen_runner.records.collision_archive import json_scalar
from scen_runner.runtime.signal_control import SignalControl
from scen_runner.runtime.signal_trial_oracle import Oracle
from scen_runner.runtime.signal_trial_fixture import StaticFixture
from scen_runner.runtime.validation_episode import EpisodeLink,EpisodeRoute
from scen_runner.runtime.checkpoint_environment import CheckpointEnvironment
from scen_runner.runtime.scene_completion import SceneCompletion
from scen_runner.runtime.validation_lease import ValidationLease
from scen_runner.runtime.signal_trial_scenario import SignalTrialScenario


BAG_TOPICS = [
    '/image_jpeg/compressed','/cam_extra/image_jpeg/compressed',
    '/cam_left/image_jpeg/compressed','/cam_right/image_jpeg/compressed',
    '/morai_sensor_bridge/frame_timing','/gps','/imu','/velodyne_points',
    '/Competition_topic','/Ego_topic','/planning/vehicle/gps_imu_state',
    '/planning/vehicle/speed_diagnostics','/CollisionData','/tf','/tf_static','/last_lidar',
    '/planning/vehicle/state','/planning/objects','/perception/traffic_light/detection',
    '/behavior/command_v2','/planning/trajectory','/planning/frenet_feedback_v2',
    '/behavior/control_request','/planning/control_plan','/control/binding_status',
    '/control/binding_epoch','/planning/signal_policy','/planning/signal_policy_shadow',
    '/lidar/motion_debug','/lidar/tracked_objects_map','/planning/input/tracked_objects',
    '/behavior/feature_debug',
    '/planning/mode','/planning/control_selector/diagnostics',
    '/ctrl_cmd','/ctrl_cmd/normal','/ctrl_cmd/jamming','/rosout_agg',
]


def closed_bag_manifest(directory):
    import rosbag
    if list(directory.glob('drive*.bag.active')):
        raise RuntimeError('recording is not closed; raw bag retained')
    files=sorted(directory.glob('drive*.bag'))
    if not files:raise RuntimeError('no closed driving bag')
    manifest=[];counts={}
    for path in files:
        digest=hashlib.sha256()
        with path.open('rb') as stream:
            for block in iter(lambda:stream.read(1024*1024),b''):digest.update(block)
        with rosbag.Bag(str(path)) as bag:
            info=bag.get_type_and_topic_info().topics
            topics={name:topic.message_count for name,topic in info.items()}
            for name,count in topics.items():counts[name]=counts.get(name,0)+count
            manifest.append(dict(file=path.name,bytes=path.stat().st_size,sha256=digest.hexdigest(),
                start=bag.get_start_time(),end=bag.get_end_time(),topics=topics))
    required=('/planning/vehicle/state','/planning/objects','/behavior/control_request',
        '/planning/control_plan','/control/binding_status','/ctrl_cmd',
        '/perception/traffic_light/detection','/cam_extra/image_jpeg/compressed',
        '/morai_sensor_bridge/frame_timing','/Ego_topic','/planning/vehicle/gps_imu_state',
        '/planning/vehicle/speed_diagnostics','/planning/mode','/planning/control_selector/diagnostics')
    required+=('/lidar/motion_debug','/lidar/tracked_objects_map',
               '/planning/input/tracked_objects','/behavior/feature_debug')
    missing=[name for name in required if not counts.get(name)]
    if missing:raise RuntimeError('required bag evidence missing: '+str(missing))
    return dict(schema_version=4,files=manifest,topic_counts=counts)


def fingerprint():
    sources={};extensions={'.py','.cpp','.hpp','.h','.yaml','.launch','.msg','.json'}
    roots=['src/asmc_planner','src/asmc_controller','src/asmc_msgs','src/lidar+sensorfusion/src/Main',
           'tools/scen_runner/scen_runner','tools/scen_runner/launch']
    for root in roots:
        for path in sorted((REPO_ROOT/root).rglob('*')):
            if path.is_file() and (path.suffix in extensions or path.name in ('CMakeLists.txt','package.xml')) and '__pycache__' not in path.parts:
                sources[path.relative_to(REPO_ROOT).as_posix()]=hashlib.sha256(path.read_bytes()).hexdigest()
    for name in ('tools/scen_runner/run_yellow_signal_trial.py','tools/scen_runner/export_signal_trial_surface.py',
                 'tools/scen_runner/analyze_signal_bag.py','tools/scen_runner/analyze_yellow_signal_trial.py',
                 'tools/scen_runner/qualify_yellow_blockage.py',
                 'tools/scen_runner/requirements.txt','src/asmc_hd_map/config/kcity2025_compact_drivable_map.json'):
        sources[name]=hashlib.sha256((REPO_ROOT/name).read_bytes()).hexdigest()
    binaries={}
    for path in [REPO_ROOT/'devel/lib/asmc_planner/asmc_planner_node',
                 REPO_ROOT/'devel/lib/asmc_controller/mpc_controller_node',
                 REPO_ROOT/'src/lidar+sensorfusion/devel/lib/Main/main',
                 REPO_ROOT/'src/lidar+sensorfusion/devel/lib/libmyHeader.so',
                 *sorted((REPO_ROOT/'devel/lib').glob('libasmc*.so'))]:
        if path.is_file():binaries[path.relative_to(REPO_ROOT).as_posix()]=hashlib.sha256(path.read_bytes()).hexdigest()
    # The container's older Git refuses even command-scoped safe.directory
    # for this UID-1000 mount. Read this owned repository's ref without changing
    # global Git trust or executing a hook/configured external command.
    git_dir=REPO_ROOT/'.git'
    if git_dir.is_file():git_dir=(REPO_ROOT/git_dir.read_text().strip()[8:]).resolve()
    head=(git_dir/'HEAD').read_text().strip()
    if head.startswith('ref: '):
        ref=head[5:];target=(git_dir/ref).resolve()
        target.relative_to(git_dir.resolve())
        if target.is_file():head=target.read_text().strip()
        else:
            head=next(line.split()[0] for line in (git_dir/'packed-refs').read_text().splitlines()
                if line and not line.startswith(('#','^')) and line.split()[1]==ref)
    if len(head)!=40 or any(c not in '0123456789abcdef' for c in head):raise ValueError('invalid repository HEAD')
    return dict(head=head,sources=sources,binaries=binaries)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--host',required=True);p.add_argument('--port',type=int,default=7789)
    p.add_argument('--clock-profile',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--physical-road',type=Path,required=True,help='verified actual simulator pavement snapshot, kept outside Git')
    p.add_argument('--light',default='C1256W000077')
    p.add_argument('--case',choices=['green','red_stop','yellow_stop','yellow_close','yellow_inside','recovery','corridor'],default='green')
    p.add_argument('--start-distance',type=float,default=60.)
    p.add_argument('--trigger-distance',type=float)
    p.add_argument('--deadline',type=float,default=150.)
    p.add_argument('--full-route',action='store_true',help='official start, CP14, then START/END; no 800m finish')
    p.add_argument('--highway-merge-1',action='store_true',
        help='isolated high-speed actor section from s=985 through the first merge exit')
    p.add_argument('--natural-signals',action='store_true',help='corridor trials only: retain automatic signal cycles')
    p.add_argument('--record-bag',action='store_true',help='record raw operational inputs and bound command/control outputs')
    p.add_argument('--scenario-seed',type=int,help='canonical full-scene actors and checkpoint weather; requires natural signals')
    p.add_argument('--c091-bollard',action='store_true',
        help='local C091-only replay of the independently recorded trial-09 avoidance bollard')
    p.add_argument('--park-static',action='append',default=[],metavar='ID:DX:DY',
        help='explicit existing stationary prop relocation in metres, snapshot and restoration required')
    p.add_argument('--collect-road-failures',action='store_true',
        help='diagnostic run only: retain small road failures while collecting signal behavior; never passes those runs')
    args=p.parse_args()
    if args.full_route and args.highway_merge_1:
        p.error('--full-route and --highway-merge-1 are mutually exclusive')
    if args.highway_merge_1 and (args.case!='corridor' or args.scenario_seed is None):
        p.error('--highway-merge-1 requires --case corridor and --scenario-seed')
    if args.natural_signals and args.case!='corridor':
        p.error('--natural-signals requires a route corridor trial')
    if args.full_route and args.case in ('recovery','yellow_close'):
        p.error('recovery and unavoidable close-yellow remain isolated local trials')
    if args.scenario_seed is not None and not args.natural_signals:
        p.error('--scenario-seed requires --case corridor --natural-signals')
    if args.c091_bollard and (args.light!='C1256W000091' or args.full_route or args.scenario_seed is not None):
        p.error('--c091-bollard requires an isolated C1256W000091 trial')
    os.environ.setdefault('GRPC_POLL_STRATEGY','epoll1')
    clock=ClockGuard();clock.check();args.output.mkdir(parents=True,exist_ok=False)
    rospy.init_node('yellow_signal_trial',anonymous=True,disable_signals=True)
    lock=threading.RLock();latest={};seen=set();streams={}
    for name in ('ros','oracle','signals','fixture','scenario'):streams[name]=(args.output/(name+'.jsonl')).open('a',buffering=1)
    def emit(name,data):
        with lock:streams[name].write(json.dumps(dict(monotonic=time.monotonic(),ros_time=rospy.Time.now().to_sec(),**data),allow_nan=False,default=json_scalar)+'\n')
    def callback(name,msg):
        try:
            with lock:latest[name]=(msg,time.monotonic())
            if name in ('request','plan','status'):
                row=decode(msg)
                if name=='plan':
                    trajectory=row.pop('trajectory');feedback=row.pop('feedback');command=row.pop('command')
                    row.update(command_id=command.command_id,maneuver=command.maneuver_type,
                        accepted=feedback.command_accepted,valid=feedback.plan_valid,execution=feedback.execution_state,
                        source=feedback.trajectory_source,detail=feedback.result_detail,
                        trajectory_stamp=trajectory.header.stamp.to_sec(),points=len(trajectory.points),
                        endpoint=([trajectory.points[-1].x,trajectory.points[-1].y,trajectory.points[-1].velocity] if trajectory.points else None))
                    if tuple(row['token']) not in seen:
                        row['first_atomic_payload_b64']=base64.b64encode(bytes(msg.data)).decode();seen.add(tuple(row['token']))
                elif name=='request':
                    command=row.pop('command');row.update(command_id=command.command_id,maneuver=command.maneuver_type,
                        desired_speed=command.desired_speed_mps,reason=command.reason)
                with lock:latest[name+'_decoded']=(row,time.monotonic())
                emit('ros',dict(topic=name,**row))
            elif name=='policy':
                data=json.loads(msg.data)
                if data.get('motion')=='EXITED':
                    with lock:latest['exited:'+data['stop_line']]=(data,time.monotonic())
                emit('ros',dict(topic=name,data=data))
            elif name=='state':emit('ros',dict(topic=name,stamp=msg.header.stamp.to_sec(),valid=msg.valid,
                x=msg.x,y=msg.y,yaw=msg.yaw,speed=msg.speed,acceleration=msg.acceleration))
            elif name=='control':emit('ros',dict(topic=name,accel=msg.accel,brake=msg.brake,steering=msg.steering))
            elif name=='detection':emit('ros',dict(topic=name,stamp=msg.header.stamp.to_sec(),frame=msg.header.frame_id,
                state=msg.state,label=msg.label,valid=msg.valid,stale=msg.stale,model_id=msg.model_id,
                confidence=msg.confidence,inference_latency_ms=msg.inference_latency_ms))
            elif name=='features':emit('ros',dict(topic=name,stamp=msg.header.stamp.to_sec(),
                ego_s=msg.ego_s,selected=msg.selected_behavior,reason=msg.decision_reason,
                pedestrian_risk=msg.pedestrian_risk,emergency_risk=msg.emergency_risk,
                static_obstacle_ahead=msg.static_obstacle_ahead,avoidance_object_id=msg.avoidance_object_id))
        except Exception as error:
            with lock:latest['decode_error']=str(error)
    topics=[('state','/planning/vehicle/state',VehicleState),('objects','/planning/objects',DetectedObjectArray),
        ('request','/behavior/control_request',UInt8MultiArray),('plan','/planning/control_plan',UInt8MultiArray),
        ('status','/control/binding_status',UInt8MultiArray),('control','/ctrl_cmd',CtrlCmd),
        ('features','/behavior/feature_debug',UrbanFeatureDebug),
        ('policy','/planning/signal_policy',String),('detection','/perception/traffic_light/detection',TrafficLightDetection)]
    subs=[rospy.Subscriber(topic,cls,lambda msg,n=name:callback(n,msg),queue_size=128) for name,topic,cls in topics]
    route=EpisodeRoute();geometry=REPO_ROOT/'src/asmc_planner/config/behavior/signal_policy_geometry_kcity2025.yaml'
    bindings=yaml.safe_load(geometry.read_text())['reviewed_bindings'];target=next(b for b in bindings if b['traffic_light_id']==args.light)
    coordinates=Coordinates([302595.,4124145.])  # Actual map global_info origin, UTM52N.
    sline,_=route.curve.project(*target['line_point']);sexit,_=route.curve.project(*target['exit_point'])
    scene_start=args.full_route or (args.case=='corridor' and not args.highway_merge_1)
    start=(route.section('high_speed')['start'] if args.highway_merge_1 else
           0. if scene_start else sline+8. if args.case=='recovery' else max(0.,sline-args.start_distance))
    end=(1290. if args.highway_merge_1 else route.curve.length if args.full_route else
         800. if args.case=='corridor' else min(800.,sexit+15.))
    x,y,z,h=route.curve.pose(start)
    pose=dict(route.scene_start) if scene_start else dict(x=x,y=y,z=z+.35,roll=0.,pitch=0.,yaw=math.degrees(h))
    reset_pose=dict(pose)
    if scene_start:
        pose['yaw']=math.degrees(coordinates.grid_yaw(pose['x'],pose['y'],pose['yaw']))
    else:reset_pose['yaw']=coordinates.enu_yaw_degrees(x,y,h)
    if args.highway_merge_1:
        # The section episode performs its own reset. Give it the simulator
        # heading while readiness still compares the map-frame vehicle yaw.
        route.section('high_speed')['pose']['yaw']=reset_pose['yaw']
    cfg={'morai':dict(host=args.host,port=args.port,client_key='asmc_yellow_validation',ego_id='Ego',grpc_src='tools/grpc_inha_univ/src')}
    source_snapshot=fingerprint()
    (args.output/'source-manifest.json').write_text(json.dumps(source_snapshot,indent=2))
    (args.output/'trial-config.json').write_text(json.dumps(dict(arguments=vars(args),
        source_clock=yaml.safe_load(args.clock_profile.read_text()),
        clock_profile_sha256=hashlib.sha256(args.clock_profile.read_bytes()).hexdigest(),
        reviewed_bindings=bindings),indent=2,default=str))
    lease=ValidationLease(args.host,args.port);lease.acquire()
    link=EpisodeLink(cfg)
    try:link.connect()
    except BaseException:
        lease.release()
        raise
    owner=SignalControl(link,lambda e:emit('signals',e))
    fixture=StaticFixture(link,lambda e:emit('fixture',e))
    launch=None;log=None;recorder=None;record_log=None;scenario=None;local_obstacle_id=None
    result={'completed':False,'case':args.case,'start_s':start,'end_s':end,
            'full_route':args.full_route,'natural_signals':args.natural_signals,
            'scenario_seed':args.scenario_seed};oracle=None
    prior_binding=rospy.get_param('/control_binding/enabled',None)
    prior_policy=rospy.get_param('/behavior_planner/signal_policy',None)
    try:
        link.hold()
        for specification in args.park_static:
            aid,dx,dy=specification.split(':');fixture.park(aid,float(dx),float(dy))
        if not args.natural_signals:
            for light in bindings:
                owner.begin(light['traffic_light_id'],light['intersection_id'],light['green_phase'],light['green_color'],light['phase_duration_s'])
        if args.case in ('red_stop','recovery'):owner.set_aspect(args.light,1)
        if args.scenario_seed is not None:
            scenario=SignalTrialScenario(link,route,args.scenario_seed,lease,
                lambda e:emit('scenario',e),section='high_speed' if args.highway_merge_1 else None)
            scenario.prepare(clock)
        else:link.reset(reset_pose)
        if not link.held_at(reset_pose):raise RuntimeError('spawn hold/pose check failed')
        if args.c091_bollard:
            # Fixed obstacle and pose from full-scene-natural-09. Keep this
            # replay inside the local signal trial and destroy only our alias.
            location=(-58.16737836306327,-142.94354019022353,28.369107390873612)
            yaw_deg=100.07146618525695
            local_obstacle_id=link.spawn_obstacle('Bollard01',*location,yaw_deg,
                                                   label='c091_signal_recovery_bollard')
            if local_obstacle_id not in {state.actor_info.id.value for state in link.scoped_statics()}:
                raise RuntimeError('C091 local bollard spawn not independently visible')
            result['local_obstacle']=dict(alias=local_obstacle_id,model='Bollard01',
                                          location=location,yaw_deg=yaw_deg)
            emit('fixture',dict(event='c091_local_bollard_spawn',**result['local_obstacle']))
        # Production stack must already have been stopped explicitly. Never
        # steal a live ROS name or silently kill an unrelated launch.
        import rosnode
        if '/asmc_planner' in rosnode.get_node_names():raise RuntimeError('another planner is running')
        if args.record_bag:
            if shutil.disk_usage(args.output).free<20*1024**3:
                raise RuntimeError('less than 20GiB free; recording refused')
            record_log=(args.output/'recorder.log').open('w')
            recorder=OwnedProcess(['rosbag','record','--split','--size=4096','--buffsize=1024',
                '-O',str(args.output/'drive.bag'),*BAG_TOPICS],stdout=record_log,stderr=record_log)
            time.sleep(1.)
            if recorder.observe() is not None:raise RuntimeError('bag recorder failed before launch')
        log=(args.output/'planning.log').open('w')
        environment=dict(os.environ)
        # Same separately built legacy message overlay as run_planning.sh.
        messages=REPO_ROOT/'src/lidar+sensorfusion/devel/lib/python3/dist-packages'
        if messages.is_dir():
            environment['PYTHONPATH']=environment.get('PYTHONPATH','')+os.pathsep+str(messages)
        launch=OwnedProcess(['roslaunch',str(REPO_ROOT/'tools/scen_runner/launch/yellow_signal_trial.launch'),
            'clock_profile:='+str(args.clock_profile.resolve()),'mode:=active'],stdout=log,stderr=log,
            cwd=str(REPO_ROOT),env=environment)
        ready_deadline=time.monotonic()+35;stable=None
        while time.monotonic()<ready_deadline:
            owner.tick();clock.check()
            if launch.observe() is not None:raise RuntimeError('trial planner launch exited')
            with lock:
                rows=dict(latest)
            now=time.monotonic();state=rows.get('state');objects=rows.get('objects');status=rows.get('status_decoded')
            ready=state and objects and status and now-state[1]<.2 and now-objects[1]<.3 and now-status[1]<.25
            if ready:
                v=state[0];s=status[0]
                heading=abs(math.remainder(v.yaw-math.radians(pose['yaw']),2*math.pi))
                ready=v.valid and abs(v.speed)<.15 and math.hypot(v.x-pose['x'],v.y-pose['y'])<.75 and heading<math.radians(1) and s['linked'] and not s['execution_failed']
            if ready:
                stable=now if stable is None else stable
                if now-stable>.5:break
            else:stable=None
            time.sleep(.05)
        else:
            with lock:detail={k:v[0] for k,v in latest.items() if k.endswith('_decoded')}
            (args.output/'readiness-failure.json').write_text(json.dumps(detail,indent=2))
            raise RuntimeError('input/binding readiness timed out; ego remains held')
        drivable=json.loads(args.physical_road.read_text())
        if drivable.get('source')!='active MORAI road MeshCollider triangles, read-only' or \
           drivable.get('route_points_covered')!=drivable.get('route_point_count') or \
           not drivable.get('route_point_count'):
            raise RuntimeError('physical road oracle provenance/coverage not accepted')
        result['physical_road_sha256']=hashlib.sha256(args.physical_road.read_bytes()).hexdigest()
        if args.full_route:
            official_path=repo_path(load_config()['map']['global_path'])
            if drivable.get('route_end_station_m','missing') is not None or \
                    drivable['route_point_count']!=len(np.loadtxt(official_path)) or \
                    drivable.get('global_path_sha256')!=hashlib.sha256(official_path.read_bytes()).hexdigest():
                raise RuntimeError('full-route physical road coverage not proven')
        completion=None
        if args.full_route:
            checkpoints=CheckpointEnvironment(link)
            completion=SceneCompletion(checkpoints.checkpoints,checkpoints.start_end)
            completion.reset((reset_pose['x'],reset_pose['y']),time.monotonic())
        oracle=Oracle(drivable,bindings,coordinates,maximum_gap_s=.3)
        if scenario:scenario.start(clock)
        else:link.release()
        begun=time.monotonic();last_clock=begun;last_tick=0
        triggered=args.case in ('red_stop','recovery');yellow_at=None;stopped_at=None;released=False
        trigger=args.trigger_distance if args.trigger_distance is not None else {'yellow_stop':20.,'yellow_close':9.,'yellow_inside':-7.}.get(args.case)
        while time.monotonic()-begun<args.deadline:
            now=time.monotonic()
            if launch.observe() is not None:raise RuntimeError('planner terminated during driving')
            if recorder and recorder.observe() is not None:raise RuntimeError('bag recorder terminated during driving')
            if scenario:scenario.check()
            clock.check()
            if now-last_tick>.3:owner.tick();last_tick=now
            query_begin=time.monotonic();actor=link.ego();query_end=time.monotonic()
            station,error=route.curve.project(actor.transform.location.x,actor.transform.location.y)
            aspects={b['traffic_light_id']:link.traffic_light_color(b['traffic_light_id']) for b in bindings
                if math.hypot(actor.transform.location.x-b['line_point'][0],
                              actor.transform.location.y-b['line_point'][1])<40}
            row=oracle.observe(actor,query_end,aspects);row.update(station=station,route_error=error,aspects=aspects,
                state_query_interval=[query_begin,query_end])
            emit('oracle',row)
            if query_end-query_begin>.1 or not row['observation_complete']:
                raise RuntimeError('independent oracle observation interval unavailable or expired')
            # Apply deliberate transitions AFTER evaluating this captured pose.
            # Its position must not be paired with a later requested color.
            if trigger is not None and not triggered and sline-station<=trigger:
                owner.set_aspect(args.light,4);triggered=True;yellow_at=time.monotonic()
            if yellow_at and time.monotonic()-yellow_at>2.5 and owner.active[args.light]['color']==4:
                owner.set_aspect(args.light,1)
            if not row['road_ok'] and (not args.collect_road_failures or row['outside_area_m2']>.25 or error>.5):
                raise RuntimeError('independent oracle: actual vehicle left the reviewed road')
            if row['collisions']:raise RuntimeError('independent oracle: actual collision')
            if args.case in ('recovery','yellow_close') and not released:
                from scen_runner.runtime.signal_trial_oracle import red_recovery_exit_proof
                with lock:exited=latest.get('exited:'+target['stop_node_id'])
                proof=red_recovery_exit_proof(target,row,exited[0]) if exited else None
                if proof:
                    result['red_recovery_exit']=proof
                    emit('signals',dict(event='recovery_red_exit_confirmed',**proof))
                    # Red has covered the entire recovery. The extra 15m is
                    # ordinary post-exit driving; refresh the reviewed green
                    # phase without creating any green flash inside the zone.
                    owner.green(args.light);released=True
            if triggered and args.case in ('red_stop','yellow_stop','yellow_close') and not released:
                if row['speed']<=.15:
                    stopped_at=now if stopped_at is None else stopped_at
                    if now-stopped_at>=2.:
                        result['stop_station']=station;owner.green(args.light);released=True
                else:stopped_at=None
            if completion:
                completion.update(actor,query_end)
                result['lap_completion']=completion.snapshot()
                finished=completion.completed
            else:finished=station>=end
            if finished:
                result['completed']=True;break
            with lock:
                if 'decode_error' in latest:raise RuntimeError('control contract decode failed: '+latest['decode_error'])
            if int(now-begun)//5!=int(last_clock-begun)//5:
                print(json.dumps(dict(elapsed=round(now-begun,1),station=round(station,2),speed=round(row['speed'],2),
                    target_remaining=round(sline-station,2))),flush=True)
            last_clock=now;time.sleep(.02)
        if not result['completed']:raise RuntimeError('trial deadline without endpoint completion')
    except Exception as error:
        result['error']=str(error);print('TRIAL FAILURE: '+str(error),flush=True)
    finally:
        if scenario:
            try:scenario.stop_worker()
            except Exception as error:result['scenario_worker_error']=str(error)
        try:link.hold()
        except Exception as error:result['hold_error']=str(error)
        if launch:
            try:
                launch.stop()
                if prior_binding is None:
                    if rospy.has_param('/control_binding/enabled'):rospy.delete_param('/control_binding/enabled')
                else:rospy.set_param('/control_binding/enabled',prior_binding)
                if prior_policy is None:
                    if rospy.has_param('/behavior_planner/signal_policy'):rospy.delete_param('/behavior_planner/signal_policy')
                else:rospy.set_param('/behavior_planner/signal_policy',prior_policy)
            except Exception as error:result['shutdown_error']=str(error)
        if log:log.close()
        if recorder:
            try:
                recorder.stop(durations=(30,2,2))
                result['recording']=closed_bag_manifest(args.output)
            except Exception as error:result['recording_error']=str(error)
        if record_log:record_log.close()
        try:owner.restore()
        except Exception as error:result['restore_error']=str(error)
        if scenario:
            try:result['scenario']=scenario.snapshot()
            except Exception as error:result['scenario_snapshot_error']=str(error)
            try:scenario.cleanup()
            except Exception as error:result['scenario_restore_error']=str(error)
        try:fixture.restore()
        except Exception as error:result['fixture_restore_error']=str(error)
        if local_obstacle_id:
            try:
                if not link.destroy(local_obstacle_id,object_type=3):
                    raise RuntimeError('C091 local bollard deletion rejected')
                deadline=time.monotonic()+2.
                while local_obstacle_id in {state.actor_info.id.value for state in link.scoped_statics()}:
                    if time.monotonic()>=deadline:
                        raise RuntimeError('C091 local bollard still visible after deletion')
                    time.sleep(.05)
                emit('fixture',dict(event='c091_local_bollard_removed',alias=local_obstacle_id))
            except Exception as error:result['local_obstacle_restore_error']=str(error)
        link.disconnect()
        lease.release()
        if oracle:result['independent_evaluation']=oracle.result()
        result['clock_guard']=dict(maximum_observed_raw_rate_ppm=clock.maximum_observed_rate_ppm)
        result['passed']=result['completed'] and not any(k.endswith('error') for k in result) and oracle is not None and oracle.result()['violation_free']
        try:
            if fingerprint()!=source_snapshot:
                result['source_changed_during_trial']=True;result['passed']=False
        except Exception as error:
            result['fingerprint_error']=str(error);result['passed']=False
        (args.output/'result.json').write_text(json.dumps(result,indent=2,default=json_scalar));print(json.dumps(result,default=json_scalar),flush=True)
        for sub in subs:sub.unregister()
        for stream in streams.values():stream.close()
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
