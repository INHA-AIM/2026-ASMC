#!/usr/bin/env python3
"""Finite Capture pilot and six-condition collection; preserve every failed run."""
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
sys.path.insert(0,str(ROOT/'tools/grpc_inha_univ/src/proto'))
sys.path.insert(0,str(ROOT/'src/data_collection/src'))
from data_collection.capture_mode import make_run_id
from data_collection.capture_collection import motion_summary, collection_trial_passed


def fallback_required(validation):
    return validation.get('requires_new_run_for_fallback',False) and validation.get('controller')=='mpc'


def reset_stopped_ego(actors, ego, saved):
    """Reset only stationary Ego between laps; preserve every other actor."""
    from google.protobuf.json_format import ParseDict, MessageToDict
    from morai.actor.actor_set_pb2 import EnableActor, SetTransformParam, SetVelocityParam, VehicleControlModeParam, VehicleGear
    from morai.actor.actor_enum_pb2 import VEHICLE_CONTROL_AUTO_MODE, GEAR_MODE_P
    state=actors.GetActorState(ego,timeout=3)
    if sum(getattr(state.global_velocity,k)**2 for k in 'xyz')>.01:
        raise RuntimeError('Ego reset requires verified stop')
    target=type(state)();ParseDict(saved,target)
    def mutate(method,request):
        if method(request,timeout=3).status!=1:raise RuntimeError('Ego reset rejected')
    report=dict(before=MessageToDict(state),temporary_ego_pause=True)
    try:
        mutate(actors.SetPause,EnableActor(actor_info=ego,enable=True))
        mutate(actors.SetVelocity,SetVelocityParam(actor_info=ego,velocity=0.))
        mutate(actors.SetTransform,SetTransformParam(actor_info=ego,transform=target.transform))
    finally:
        mutate(actors.SetPause,EnableActor(actor_info=ego,enable=False))
    mutate(actors.SetVehicleControlMode,VehicleControlModeParam(actor_info=ego,mode=VEHICLE_CONTROL_AUTO_MODE))
    mutate(actors.SetVehicleGear,VehicleGear(actor_info=ego,gear=GEAR_MODE_P))
    time.sleep(1)
    state=actors.GetActorState(ego,timeout=3)
    yaw_delta=(state.transform.rotation.z-target.transform.rotation.z+180)%360-180
    if 1 <= abs(yaw_delta) <= 5:
        corrected=SetTransformParam(actor_info=ego,transform=target.transform)
        corrected.transform.rotation.z-=yaw_delta
        try:
            mutate(actors.SetPause,EnableActor(actor_info=ego,enable=True))
            mutate(actors.SetTransform,corrected)
        finally:
            mutate(actors.SetPause,EnableActor(actor_info=ego,enable=False))
        mutate(actors.SetVehicleControlMode,VehicleControlModeParam(actor_info=ego,mode=VEHICLE_CONTROL_AUTO_MODE))
        mutate(actors.SetVehicleGear,VehicleGear(actor_info=ego,gear=GEAR_MODE_P))
        time.sleep(1)
        report['bounded_yaw_correction_deg']=-yaw_delta
        state=actors.GetActorState(ego,timeout=3)
    error=math.sqrt(sum((getattr(state.transform.location,k)-getattr(target.transform.location,k))**2 for k in 'xyz'))
    yaw=abs((state.transform.rotation.z-target.transform.rotation.z+180)%360-180)
    report.update(position_error_m=error,yaw_error_deg=yaw)
    if error>.1 or yaw>1:raise RuntimeError('Ego reset pose verification failed: '+str(report))
    return report


def trial(data,controller,speed,ledger,layout,captures=30,seconds=120,parent='',inspection_only=False):
    run=make_run_id('route_'+controller+'_'+str(speed))
    command=['roslaunch','data_collection','capture_route_collection.launch',
        'controller:='+controller,'target_kph:='+str(speed),'run_id:='+run,'data_root:='+str(data),
        'max_captures:='+str(captures),'max_seconds:='+str(seconds),
        'inspection_only:='+str(inspection_only).lower(),
        'parent_run:='+parent,'layout_id:='+layout,'spawn_ledger:='+str(ledger or '')]
    folder=data/'capture_runs'/run
    log=data/'capture_runs'/(run+'.log');log.parent.mkdir(parents=True,exist_ok=True)
    with log.open('w') as stream:
        process=subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        try:process.wait(timeout=seconds+30)
        except subprocess.TimeoutExpired:
            import os
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=10)
    validation=json.loads((folder/'validation.json').read_text()) if (folder/'validation.json').exists() else dict(
        controller=controller,tracking_passed=False,exit_reason='node_failed_before_validation')
    summary=json.loads((folder/'summary.json').read_text()) if (folder/'summary.json').exists() else {}
    validation.update(run_id=run,roslaunch_exit_code=process.returncode,capture_summary=summary)
    samples=[json.loads(line) for line in (folder/'control.jsonl').read_text().splitlines()] if (folder/'control.jsonl').exists() else []
    validation.update(motion_summary(samples,speed))
    restoration = json.loads((folder/'restoration.json').read_text()) if (folder/'restoration.json').exists() else {}
    validation['actuator_restoration'] = restoration
    validation['passed'] = collection_trial_passed(validation, summary, captures, restoration, process.returncode)
    return validation


def main():
    import grpc
    from google.protobuf.json_format import MessageToDict
    from morai.environment.environment_pb2_grpc import EnvironmentStub
    from morai.environment.weather_pb2 import Weather
    from morai.environment.time_pb2 import SimulationTime
    from morai.common.type_pb2 import Empty
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('operation',choices=('pilot','inspect','collect'))
    p.add_argument('--data-root',type=Path,default=Path('/data'))
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--ledger',type=Path)
    p.add_argument('--layout-id',default='objectless_pilot')
    p.add_argument('--grpc-host',default='host.docker.internal')
    p.add_argument('--pilot-captures',type=int,default=30)
    p.add_argument('--controller',choices=('mpc','pp'),default='mpc',help='PP is explicit after a preserved MPC diagnosis')
    p.add_argument('--pilot-results',type=Path)
    p.add_argument('--start-pose',type=Path,help='Saved stopped Ego state; reset before each trial/lap')
    a=p.parse_args()
    if a.output.exists():raise RuntimeError('choose a new immutable trial report')
    a.output.parent.mkdir(parents=True,exist_ok=True)
    report=dict(operation=a.operation,runs=[],layout_id=a.layout_id,completed=False)
    def save():a.output.write_text(json.dumps(report,indent=2)+'\n')
    save()
    from morai.actor.actor_pb2_grpc import ActorStub
    from morai.common.object_info_pb2 import ObjectInfo
    from morai.common.enum_pb2 import OBJECT_TYPE_VEHICLE
    channel=grpc.insecure_channel(a.grpc_host+':7789');actors=ActorStub(channel)
    ego=ObjectInfo(object_type=OBJECT_TYPE_VEHICLE);ego.id.value='Ego'
    start=json.loads(a.start_pose.read_text()) if a.start_pose else MessageToDict(actors.GetActorState(ego,timeout=3),preserving_proto_field_name=True)
    report['start_pose']=start;report['resets']=[]
    def reset():
        try:report['resets'].append(reset_stopped_ego(actors,ego,start))
        except Exception as exc:
            report['failure']='reset_failed: '+str(exc);save();raise
        save()
    if a.operation=='pilot':
        controller=a.controller;selected_speed=None
        for speed in (5,20,30):
            reset()
            result=trial(a.data_root,controller,speed,a.ledger,a.layout_id,a.pilot_captures,120)
            report['runs'].append(result);save()
            if not result['passed'] and fallback_required(result):
                controller='pp'
                # PP is independently checked at 5 km/h before returning to 20/30.
                for pp_speed in sorted(set((5,speed))):
                    reset()
                    retry=trial(a.data_root,controller,pp_speed,a.ledger,a.layout_id,a.pilot_captures,120,parent=result['run_id'])
                    report['runs'].append(retry);save()
                    if not retry['passed']:report['failure']='pp_pilot_failed';save();return 2
                result=retry
            if not result['passed']:
                if speed==30 and selected_speed==20:
                    report['30kph_rejected']=result['exit_reason'] or 'tracking_or_actual_speed_gate_failed';break
                report['failure']='pilot_failed';save();return 2
            selected_speed=speed
        report.update(completed=True,controller=controller,selected_speed_kph=selected_speed)
        save();return 0
    if not a.pilot_results or not a.ledger:raise RuntimeError('collection requires accepted pilot report and reviewed layout ledger')
    pilot=json.loads(a.pilot_results.read_text())
    if not pilot.get('completed') or pilot.get('selected_speed_kph') not in (20,30):
        raise RuntimeError('20/30km/h pilot has not passed')
    environment=EnvironmentStub(channel)
    start=pilot['start_pose']
    original_weather=environment.GetWeather(Empty(),timeout=5);original_time=environment.GetTime(Empty(),timeout=5)
    report.update(original_weather=MessageToDict(original_weather),original_time=MessageToDict(original_time));save()
    try:
        for weather_name,code in ((('sunny',0),) if a.operation=='inspect' else (('sunny',0),('foggy',2))):
            for hour in ((13,) if a.operation=='inspect' else (11,13,15)):
                for method,request in ((environment.SetWeather,Weather(type=code,instantly=True)),
                                       (environment.SetTime,SimulationTime(hour=hour,instantly=True))):
                    if method(request,timeout=5).status!=1:raise RuntimeError('environment change rejected')
                time.sleep(2)
                if environment.GetWeather(Empty(),timeout=5).type!=code or environment.GetTime(Empty(),timeout=5).hour!=hour:
                    raise RuntimeError('environment change not verified')
                reset()
                result=trial(a.data_root,pilot['controller'],pilot['selected_speed_kph'],a.ledger,a.layout_id,0,1800,
                             inspection_only=a.operation=='inspect')
                result.update(weather=weather_name,hour=hour)
                report['runs'].append(result);save()
                if not result['passed']:raise RuntimeError('condition failed; evidence retained')
        report['completed']=True;save()
    except Exception as exc:
        report['failure']=str(exc);save();raise
    finally:
        report['restoration']={}
        for name,method,request in (('weather',environment.SetWeather,original_weather),('time',environment.SetTime,original_time)):
            try:
                request.instantly=True
                response=method(request,timeout=5)
                report['restoration'][name]=MessageToDict(response)
                if response.status!=1:report['restoration'][name]['error']='restore rejected'
            except Exception as exc:report['restoration'][name]={'error':str(exc)}
        time.sleep(1)
        try:
            weather=environment.GetWeather(Empty(),timeout=5);clock=environment.GetTime(Empty(),timeout=5)
            report['restoration']['verified']=dict(weather_type=weather.type==original_weather.type,
                time_hour=clock.hour==original_time.hour)
        except Exception as exc:report['restoration']['verification_error']=str(exc)
        if any('error' in r for r in report['restoration'].values() if isinstance(r,dict)) or report['restoration'].get('verified')!={'weather_type':True,'time_hour':True}:
            report['completed']=False;report['restoration_failed']=True
        save();channel.close()
    return 0 if report['completed'] else 2


if __name__=='__main__':raise SystemExit(main())
