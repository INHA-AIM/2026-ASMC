#!/usr/bin/env python3
"""Opt-in F1 MORAI-only 60s capture; bounded PP drive, owned actors, restoration.

gRPC pose is evaluation control/ground-truth only, never a production input.
Requires an already running isolated /migration/lidar plus camera and bridge.
"""
import argparse
import json
import math
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid

ROOT=Path(__file__).resolve().parents[2]


def main(args):
    if not args.execute:raise ValueError('--execute required')
    manifest=json.loads(args.manifest.read_text())
    if manifest['run_id']!=args.run_id:raise ValueError('run ID mismatch')
    if args.output.exists():raise ValueError('output already exists')
    sys.path[:0]=[str(ROOT/'tools/grpc_inha_univ/src/proto'),str(ROOT/'src/data_collection/src')]
    import grpc,rospy,rosgraph
    from google.protobuf.json_format import MessageToDict
    from morai.actor.actor_pb2_grpc import ActorStub
    from morai.actor.actor_get_pb2 import GetAllActorsFilter
    from morai.actor.actor_control_pb2 import VehicleCtrlCmd
    from morai.actor.actor_set_pb2 import VehicleControlModeParam,VehicleGear,SetTransformParam,EnableActor
    from morai.actor.actor_spawn_pb2 import ActorSpawnInfo,VehicleSpawnParam
    from morai.actor.actor_enum_pb2 import VEHICLE_CONTROL_AUTO_MODE,GEAR_MODE_D
    from morai.common.object_info_pb2 import ObjectInfo
    from morai.common.type_pb2 import Empty,Transform
    from morai.common.enum_pb2 import OBJECT_TYPE_VEHICLE
    from morai.environment.environment_pb2_grpc import EnvironmentStub
    from morai.environment.weather_pb2 import Weather
    from morai.environment.time_pb2 import SimulationTime
    from std_srvs.srv import Trigger
    from data_collection.capture_collection import LegacyPurePursuit,point_route_distance
    from drive_coco7_preview import restore_ego
    rospy.init_node('f1_owned_trial',anonymous=True,disable_signals=True)
    master=rosgraph.Master(rospy.get_name())
    topics=list(manifest['views'].values())+[manifest[k] for k in ('camera_observations_topic','camera_status_topic','timing_topic','trace_topic')]+['/velodyne_points','/imu','/gps','/Ego_topic','/CollisionData','/migration/lidar/diagnostics']
    publishers=dict(master.getSystemState()[0])
    def control_gate():
        if any(ns and ('ctrl_cmd' in t or 'actuator_cmd' in t) for t,ns in master.getSystemState()[0]):
            raise RuntimeError('another control publisher exists')
    control_gate()
    for t in topics:
        if len(publishers.get(t,[]))!=1:raise RuntimeError('need one publisher: '+t)
    channel=grpc.insecure_channel(args.address);actors=ActorStub(channel);environment=EnvironmentStub(channel)
    ego=ObjectInfo(object_type=OBJECT_TYPE_VEHICLE);ego.id.value='Ego'
    original=actors.GetActorState(ego,timeout=3);mode=actors.GetVehicleControlMode(ego,timeout=3).mode
    old_weather=environment.GetWeather(Empty(),timeout=3);old_time=environment.GetTime(Empty(),timeout=3)
    def speed(s):return math.sqrt(sum(getattr(s.global_velocity,k)**2 for k in 'xyz'))
    if speed(original)>.05:raise RuntimeError('Ego must be stationary')
    link='A2256W000126' if args.scenario=='tunnel' else 'A2256W000304'
    all_links=json.loads(args.link_set.read_text());points=next(v['points'] for v in all_links if v['idx']==link)
    # 20m past link start, using the existing route representation.
    station=0.;index=0
    for i in range(len(points)-1):
        station+=math.dist(points[i][:2],points[i+1][:2])
        if station>=20:index=i+1;break
    xyz=points[index];d=[points[min(index+2,len(points)-1)][i]-points[max(0,index-2)][i] for i in (0,1)]
    yaw=math.atan2(d[1],d[0]);target=Transform()
    target.location.x,target.location.y,target.location.z=xyz[0],xyz[1],xyz[2]+.36
    target.rotation.z=math.degrees(yaw)
    inventory=actors.GetAllActorsState(GetAllActorsFilter(vehicle=True,pedestrian=True,obstacle=True),timeout=3)
    for other in inventory.states:
        if other.actor_info.id.value=='Ego':continue
        q=other.transform.location
        if point_route_distance(points[index:],q.x,q.y)<5:
            raise RuntimeError('existing actor near planned trial route; not moving user actors')
    args.output.mkdir(parents=True)
    report=dict(run_id=args.run_id,scenario=args.scenario,link=link,ego_before=MessageToDict(original),mode_before=mode,
        weather_before=MessageToDict(old_weather),time_before=MessageToDict(old_time),
        publishers=publishers,actors_before=MessageToDict(inventory),owned=[],failures=[],restored=False)
    def save():(args.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    def mutate(method,request):
        result=method(request,timeout=3)
        if result.status!=1:raise RuntimeError('mutation rejected: '+str(result))
        return result
    def ctrl(brake=0.,throttle=0.,steer=0.):
        mutate(actors.ControlVehicle,VehicleCtrlCmd(actor_info=ego,long_cmd_type=1,brake=brake,throttle=throttle,steer=steer))
    def hold(seconds):
        until=time.monotonic()+seconds
        while time.monotonic()<until:ctrl(brake=1);time.sleep(.05)
    owned=[];record=None;changed=False
    def interrupt(*_):raise KeyboardInterrupt('F1 interrupted')
    signal.signal(signal.SIGTERM,interrupt);signal.signal(signal.SIGINT,interrupt)
    try:
        save();changed=True
        mutate(environment.SetWeather,Weather(type=2 if args.scenario=='foggy' else 0,instantly=True))
        new_time=SimulationTime();new_time.CopyFrom(old_time);new_time.hour=13;new_time.instantly=True
        mutate(environment.SetTime,new_time)
        mutate(actors.SetVehicleControlMode,VehicleControlModeParam(actor_info=ego,mode=VEHICLE_CONTROL_AUTO_MODE))
        hold(.5);mutate(actors.SetTransform,SetTransformParam(actor_info=ego,transform=target));hold(3)
        mutate(actors.SetVehicleControlMode,VehicleControlModeParam(actor_info=ego,mode=VEHICLE_CONTROL_AUTO_MODE))
        settle=actors.GetActorState(ego,timeout=3)
        if speed(settle)>.05:raise RuntimeError('Ego not settled')
        report['settled']=MessageToDict(settle)
        # Front and both sides: stationary assets only; never placed in a drive clip.
        if args.scenario in ('stationary','foggy'):
            for forward,lateral in ((18.,0.),(8.,7.),(8.,-7.)):
                info=ObjectInfo(object_type=OBJECT_TYPE_VEHICLE,client_key='asmc_f1');info.id.value='f1_'+uuid.uuid4().hex[:12]
                pose=Transform();pose.CopyFrom(settle.transform)
                pose.location.x+=forward*math.cos(yaw)-lateral*math.sin(yaw)
                pose.location.y+=forward*math.sin(yaw)+lateral*math.cos(yaw);pose.location.z+=5
                mutate(actors.SpawnVehicle,VehicleSpawnParam(spawn_info=ActorSpawnInfo(actor_info=info,model_name='2014_Kia_K7',label='F1 owned stationary',transform=pose),pause=True))
                owned.append(info);report['owned'].append(MessageToDict(info));save()
                state=actors.GetActorState(info,timeout=3)
                pose.location.z=state.transform.location.z+(settle.transform.location.z-.36)-(state.bounding_box.location.z-state.bounding_box.extent.z)
                mutate(actors.SetTransform,SetTransformParam(actor_info=info,transform=pose))
                mutate(actors.SetAI,EnableActor(actor_info=info,enable=False));mutate(actors.SetPause,EnableActor(actor_info=info,enable=True))
        hold(2)
        reply=rospy.ServiceProxy('/migration/lidar/lidar_pipeline/reset',Trigger)()
        if not reply.success:raise RuntimeError('owned pipeline reset failed')
        hold(2)
        record=subprocess.Popen(['rosbag','record','--lz4','--buffsize=256','--duration=60','-O',str(args.output/'signals.bag'),*topics],stdout=(args.output/'recorder.log').open('x'),stderr=subprocess.STDOUT)
        begin=time.monotonic();controller=LegacyPurePursuit(ROOT,points,10.)
        mutate(actors.SetVehicleGear,VehicleGear(actor_info=ego,gear=GEAR_MODE_D))
        with (args.output/'drive.jsonl').open('x') as journal:
            while time.monotonic()-begin<61:
                tick=time.monotonic();elapsed=tick-begin;s=actors.GetActorState(ego,timeout=1)
                loc=s.transform.location;v=speed(s);cte=point_route_distance(points,loc.x,loc.y)
                if v>4.5 or cte>1.0 or any(x!='Ego' for x in s.vehicle_state.collision_objects):
                    raise RuntimeError('drive guard speed/lateral/collision')
                control_gate()
                driving=args.scenario in ('straight','tunnel') and 10<=elapsed<35
                if driving:
                    cmd=controller.command(loc.x,loc.y,math.radians(s.transform.rotation.z),v)
                    if cmd['remaining_s']<12:driving=False
                if driving:ctrl(throttle=cmd['accel'],brake=cmd['brake'],steer=max(-1,min(1,math.degrees(cmd['steer'])/40.)))
                else:ctrl(brake=1.)
                journal.write(json.dumps(dict(ros_ns=rospy.Time.now().to_nsec(),elapsed=elapsed,speed_mps=v,cte_m=cte,link=s.vehicle_state.current_link_info.id.value,pose=MessageToDict(s.transform),driving=driving))+'\n')
                if record.poll() is not None and elapsed<58:raise RuntimeError('recorder stopped early')
                time.sleep(max(0,.1-(time.monotonic()-tick)))
        hold(1);record.wait(timeout=15);record=None
        report['weather_used']=MessageToDict(environment.GetWeather(Empty(),timeout=3));report['time_used']=MessageToDict(environment.GetTime(Empty(),timeout=3))
    except BaseException as e:
        report['failures'].append(str(e))
    finally:
        if record and record.poll() is None:
            record.send_signal(signal.SIGINT);record.wait(timeout=20)
        if changed:
            try:hold(1)
            except Exception as e:report['failures'].append('brake: '+str(e))
        for info in reversed(owned):
            try:mutate(actors.DestroyActor,info)
            except Exception as e:report['failures'].append('owned cleanup: '+str(e))
        if changed:
            try:report['restored']=restore_ego(actors,ego,original,mode,mutate,report)
            except Exception as e:report['failures'].append('restore: '+str(e))
            try:
                mutate(environment.SetWeather,old_weather);mutate(environment.SetTime,old_time)
                rospy.ServiceProxy('/migration/lidar/lidar_pipeline/reset',Trigger)()
                remaining=actors.GetAllActorsState(GetAllActorsFilter(vehicle=True,pedestrian=True,obstacle=True),timeout=3)
                before_ids={v.actor_info.id.value for v in inventory.states}
                after_ids={v.actor_info.id.value for v in remaining.states}
                report['actor_inventory_restored']=before_ids==after_ids
                report['weather_after']=MessageToDict(environment.GetWeather(Empty(),timeout=3))
                report['time_after']=MessageToDict(environment.GetTime(Empty(),timeout=3))
                if not report['actor_inventory_restored']:report['failures'].append('actor inventory not restored')
                if environment.GetWeather(Empty(),timeout=3).type!=old_weather.type:report['failures'].append('weather not restored')
                if environment.GetTime(Empty(),timeout=3).hour!=old_time.hour:report['failures'].append('time not restored')
            except Exception as e:report['failures'].append('environment restore: '+str(e))
        save()
    print(json.dumps(report,indent=2))
    if report['failures'] or not report['restored']:raise SystemExit(1)

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--execute',action='store_true');p.add_argument('--run-id',required=True)
    p.add_argument('--scenario',choices=['stationary','straight','tunnel','foggy'],required=True)
    p.add_argument('--address',default='host.docker.internal:7789')
    for name in ('manifest','output','link-set'):p.add_argument('--'+name,type=Path,required=True)
    main(p.parse_args())
