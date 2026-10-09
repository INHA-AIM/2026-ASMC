#!/usr/bin/env python3
"""Bounded 5 km/h MORAI rehearsal using an already-running preview stack.

This is an evaluation actuator policy, not an unchanged MPC validation. The
preview steering is retained; trajectory speed is capped and sent as type 2.
Only this gate publishes /ctrl_cmd. Existing actors/scenarios are preserved.
"""
import argparse
import json
import math
from pathlib import Path
import threading
import time

import numpy as np


def restoration_check(state, original, mode, original_mode):
    position = math.sqrt(sum((getattr(state.transform.location,k)-getattr(original.transform.location,k))**2 for k in 'xyz'))
    rotation = max(abs((getattr(state.transform.rotation,k)-getattr(original.transform.rotation,k)+180)%360-180) for k in 'xyz')
    speed = math.sqrt(sum(getattr(state.global_velocity,k)**2 for k in 'xyz'))
    return dict(position_error_m=position, rotation_error_deg=rotation, speed_mps=speed,
                gear_matches=state.vehicle_state.gear_mode==original.vehicle_state.gear_mode,
                mode_matches=mode==original_mode,
                passed=position<.1 and rotation<1 and speed<.05 and
                state.vehicle_state.gear_mode==original.vehicle_state.gear_mode and mode==original_mode)


def restore_ego(actors, ego, original, original_mode, mutate, report, sleep=time.sleep):
    from morai.actor.actor_set_pb2 import (EnableActor, SetVelocityParam,
        SetTransformParam, VehicleControlModeParam, VehicleGear)
    from morai.actor.actor_control_pb2 import VehicleCtrlCmd
    from capture_coco7_actors import MessageToDict
    report['restore_pause_scope']='Temporary Ego pause; prior pause state unavailable; explicitly resumed.'
    paused=False
    try:
        mutate(actors.SetPause,EnableActor(actor_info=ego,enable=True));paused=True
        mutate(actors.SetVehicleControlMode,VehicleControlModeParam(actor_info=ego,mode=3))
        mutate(actors.ControlVehicle,VehicleCtrlCmd(actor_info=ego,long_cmd_type=1,brake=1.,steer=0.))
        mutate(actors.SetVelocity,SetVelocityParam(actor_info=ego,velocity=0.))
        mutate(actors.SetTransform,SetTransformParam(actor_info=ego,transform=original.transform))
        mutate(actors.SetPause,EnableActor(actor_info=ego,enable=False));paused=False
        sleep(1.)
        state=actors.GetActorState(ego,timeout=3)
        yaw=(state.transform.rotation.z-original.transform.rotation.z+180)%360-180
        # SetTransform can yield a repeatable yaw offset in this MORAI build.
        # One bounded correction targets the saved pose; never relax tolerance.
        if 1 <= abs(yaw) <= 5:
            mutate(actors.SetPause,EnableActor(actor_info=ego,enable=True));paused=True
            request=SetTransformParam(actor_info=ego,transform=original.transform)
            request.transform.rotation.z-=yaw
            report['restore_yaw_correction_deg']=-yaw
            mutate(actors.SetTransform,request)
            mutate(actors.SetPause,EnableActor(actor_info=ego,enable=False));paused=False
        # SetTransform changes the control mode in this simulator. Restore mode
        # and gear last, after all transforms and after queued UDP commands drain.
        mutate(actors.SetVehicleControlMode,VehicleControlModeParam(actor_info=ego,mode=original_mode))
        mutate(actors.SetVehicleGear,VehicleGear(actor_info=ego,gear=original.vehicle_state.gear_mode))
        sleep(1.)
        report['restoration_checks']=[]
        for _ in range(3):
            sleep(.5)
            state=actors.GetActorState(ego,timeout=3)
            mode=actors.GetVehicleControlMode(ego,timeout=3).mode
            report['restoration_checks'].append(restoration_check(state,original,mode,original_mode))
        report['restored_ego']=MessageToDict(state)
        return all(c['passed'] for c in report['restoration_checks'])
    finally:
        if paused:
            mutate(actors.SetPause,EnableActor(actor_info=ego,enable=False))


def speed_command(preview, trajectory):
    """Never convert an explicit planner/controller stop into a drive request."""
    if not trajectory.valid or not trajectory.points or preview.brake > .5:
        return 0.
    velocity = trajectory.points[min(3,len(trajectory.points)-1)].velocity
    if not math.isfinite(velocity) or velocity <= 0: return 0.
    if preview.longlCmdType == 2:
        if not math.isfinite(preview.velocity) or preview.velocity <= 0: return 0.
        return min(5., max(0., velocity*3.6), preview.velocity)
    return min(5., max(0., velocity*3.6))


def grpc_speed_mps(command_kph):
    """Explicit SDK diagnostic conversion; ROS/UDP CtrlCmd remains km/h."""
    if not math.isfinite(command_kph) or not 0 <= command_kph <= 5:
        raise ValueError('bounded diagnostic speed must be0..5km/h')
    return command_kph / 3.6


def main():
    from capture_coco7_actors import grpc, ActorStub, ObjectInfo, OBJECT_TYPE_VEHICLE, MessageToDict
    from morai.actor.actor_set_pb2 import VehicleControlModeParam, VehicleGear, SetTransformParam, SetVelocityParam
    from morai.actor.actor_enum_pb2 import VEHICLE_CONTROL_AUTO_MODE, GEAR_MODE_D
    from morai.actor.actor_control_pb2 import VehicleCtrlCmd
    import rosgraph
    import rospy
    from asmc_camera.msg import InferenceStatus
    from asmc_msgs.msg import CameraObjectBoxArray, Trajectory, VehicleState
    from morai_msgs.msg import CtrlCmd, CollisionData
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--seconds',type=float,default=10.)
    p.add_argument('--grpc-host',default='host.docker.internal')
    p.add_argument('--actuator-transport',choices=('udp','grpc'),default='udp',
                   help='gRPC is a separate actuator diagnostic; never also send UDP commands')
    p.add_argument('--route',type=Path,default=Path('/root/ws/R_KR_PG_KATRI/2026_molit_comp_global_path.txt'))
    a=p.parse_args()
    if not 0 < a.seconds <= 30: p.error('trial must be at most 30 seconds')
    a.output.mkdir(parents=True,exist_ok=False)
    rospy.init_node('coco7_low_speed_gate',anonymous=True)
    master=rosgraph.Master(rospy.get_name())
    def publishers():
        return {topic:names for topic,names in master.getSystemState()[0]
                if topic in ('/ctrl_cmd','/ctrl_cmd_0') and names}
    if publishers(): raise RuntimeError('existing public control publisher; refusing gate')
    route=np.loadtxt(str(a.route))[:,:2]
    actors=ActorStub(grpc.insecure_channel(a.grpc_host+':7789'))
    ego=ObjectInfo(object_type=OBJECT_TYPE_VEHICLE);ego.id.value='Ego'
    original=actors.GetActorState(ego,timeout=3)
    original_mode=actors.GetVehicleControlMode(ego,timeout=3).mode
    if sum(getattr(original.global_velocity,k)**2 for k in 'xyz')>.01:
        raise RuntimeError('stationary Ego required before the bounded trial')
    report=dict(original_ego=MessageToDict(original),original_mode=original_mode,
                failures=[],rpc=[],samples=[],restored=False,
                bridge_control_destination=dict(ip=rospy.get_param('/morai_udp_bridge/cmd_ip',None),
                                                port=rospy.get_param('/morai_udp_bridge/cmd_port',None)),
                actuator_transport=a.actuator_transport,
                actuator_velocity_unit='m/s' if a.actuator_transport=='grpc' else 'km/h',
                actuator_policy=a.actuator_transport+' type2 speed capped at5km/h from valid trajectory; preview steering; MPC actuator conversion is explicit',
                actor_restore_scope='Ego pose/gear/control mode only; other actors untouched')
    def save(): (a.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    def mutate(method,request):
        response=method(request,timeout=3)
        report['rpc'].append(dict(request=MessageToDict(request),response=MessageToDict(response)))
        save()
        if response.status!=1: raise RuntimeError('gRPC setter rejected: '+str(response))
    latest={}; statuses={}; views={}; lock=threading.RLock(); collision=[False]; subs=[]
    def receive(key,msg):
        with lock: latest[key]=(time.monotonic(),msg)
    def receive_status(key,msg):
        with lock: statuses[key]=(time.monotonic(),msg)
    def receive_objects(msg):
        with lock: views[msg.header.frame_id]=(time.monotonic(),msg)
    def receive_collision(msg):
        if msg.collision_object or msg.collision_objecta: collision[0]=True
    for name,node in [('yolo','camera_object_detector'),('twinlite','camera_semantic_inference'),('resnet18','traffic_light_infer')]:
        subs.append(rospy.Subscriber('/'+node+'/inference_status',InferenceStatus,lambda m,n=name:receive_status(n,m),queue_size=5))
    subs += [rospy.Subscriber('/evaluation/ctrl_cmd',CtrlCmd,lambda m:receive('preview',m),queue_size=1),
             rospy.Subscriber('/planning/trajectory',Trajectory,lambda m:receive('trajectory',m),queue_size=1),
             rospy.Subscriber('/planning/vehicle/state',VehicleState,lambda m:receive('state',m),queue_size=1),
             rospy.Subscriber('/perception/camera/objects',CameraObjectBoxArray,receive_objects,queue_size=20),
             rospy.Subscriber('/CollisionData',CollisionData,receive_collision,queue_size=1)]
    publisher=None; changed=False
    def send(command):
        if a.actuator_transport=='udp':
            publisher.publish(command)
        else:
            # SDK proto omits the speed unit. The retained initial diagnostic
            # approached3.6x the km/h number; test an explicit m/s conversion.
            # Keep actual motion guards and the same normalized steering as UDP.
            steer=max(-1.,min(1.,math.degrees(command.steering)/40.))
            response=actors.ControlVehicle(VehicleCtrlCmd(actor_info=ego,
                long_cmd_type=command.longlCmdType,velocity=grpc_speed_mps(command.velocity),
                throttle=command.accel,brake=command.brake,steer=steer),timeout=.3)
            if response.status!=1:raise RuntimeError('gRPC actuator command rejected: '+str(response))
    try:
        deadline=time.monotonic()+10
        while time.monotonic()<deadline:
            with lock:
                ready=(all(k in latest for k in ('preview','trajectory','state')) and
                       len(statuses)==3 and all(k in views for k in ('camera_front','camera_left','camera_right')))
            if ready: break
            time.sleep(.05)
        if not ready: raise RuntimeError('preview/model subscription readiness timeout')
        if publishers(): raise RuntimeError('public publisher appeared during preparation')
        if a.actuator_transport=='udp':
            publisher=rospy.Publisher('/ctrl_cmd',CtrlCmd,queue_size=1)
            time.sleep(.3)
        changed=True
        mutate(actors.SetVehicleControlMode,VehicleControlModeParam(actor_info=ego,mode=VEHICLE_CONTROL_AUTO_MODE))
        mutate(actors.SetVehicleGear,VehicleGear(actor_info=ego,gear=GEAR_MODE_D))
        begin=time.monotonic()
        heading=np.array([math.cos(math.radians(original.transform.rotation.z)),
                          math.sin(math.radians(original.transform.rotation.z))])
        origin=np.array([original.transform.location.x,original.transform.location.y])
        while time.monotonic()-begin<a.seconds and not rospy.is_shutdown():
            now=time.monotonic();state=actors.GetActorState(ego,timeout=.3)
            point=np.array([state.transform.location.x,state.transform.location.y])
            distance=float(np.min(np.linalg.norm(route-point,axis=1)))
            reason=None; command=CtrlCmd(longlCmdType=1,brake=1.)
            with lock:
                if collision[0] or state.vehicle_state.collision_objects: reason='collision'
                elif float(np.dot(point-origin,heading)) < -.3: reason='reverse movement during forward-only rehearsal'
                elif distance>4.5: reason='route corridor exit'
                elif any(now-latest[k][0]>age for k,age in [('preview',.25),('trajectory',.25),('state',.2)]): reason='stale preview input'
                elif not latest['trajectory'][1].valid or not latest['state'][1].valid: reason='invalid planning input'
                elif any(now-t>.6 or m.mode!=2 or not m.prepared or not m.pipeline_ready for t,m in statuses.values()): reason='model not active/ready'
                elif any(now-t>.35 or not m.valid or not 0<=(rospy.Time.now()-m.header.stamp).to_sec()<=.35 for t,m in views.values()): reason='stale/invalid camera observation'
                elif math.sqrt(sum(getattr(state.global_velocity,k)**2 for k in 'xyz'))*3.6>5.5: reason='speed ceiling exceeded'
                else:
                    preview=latest['preview'][1]; trajectory=latest['trajectory'][1]
                    speed=speed_command(preview,trajectory)
                    if not math.isfinite(preview.steering): reason='invalid steering'
                    elif speed>0:
                        command=CtrlCmd(longlCmdType=2,velocity=speed,steering=preview.steering)
            actual=publishers()
            expected={'/ctrl_cmd':[rospy.get_name()]} if a.actuator_transport=='udp' else {}
            if actual!=expected: reason='duplicate public control publisher'
            if reason: command=CtrlCmd(longlCmdType=1,brake=1.)
            send(command)
            report['samples'].append(dict(elapsed_sec=now-begin,position=point.tolist(),
                speed_kph=math.sqrt(sum(getattr(state.global_velocity,k)**2 for k in 'xyz'))*3.6,
                route_distance_m=distance,command_kph=command.velocity,brake=command.brake,guard=reason))
            if reason: raise RuntimeError(reason)
            time.sleep(.05)
        if not report['samples']: raise RuntimeError('no driving samples')
        moved=np.linalg.norm(np.array(report['samples'][-1]['position'])-
                             np.array([original.transform.location.x,original.transform.location.y]))
        report['displacement_m']=float(moved)
        report['forward_progress_m']=float(np.dot(np.array(report['samples'][-1]['position'])-origin,heading))
        if moved<.5 or report['forward_progress_m']<.5:
            report['failures'].append('no verified forward drive: less than0.5m forward progress')
    except KeyboardInterrupt: report['failures'].append('trial interrupted; restoration attempted')
    except Exception as exc: report['failures'].append(str(exc))
    finally:
        if publisher:
            for _ in range(10): publisher.publish(CtrlCmd(longlCmdType=1,brake=1.));time.sleep(.05)
            publisher.unregister()
        elif changed and a.actuator_transport=='grpc':
            try:send(CtrlCmd(longlCmdType=1,brake=1.))
            except Exception as exc:report['failures'].append('gRPC final brake: '+str(exc))
        if changed:
            try:
                report['restored']=restore_ego(actors,ego,original,original_mode,mutate,report)
                if not report['restored']: report['failures'].append('Ego restoration tolerance not met')
            except Exception as exc: report['failures'].append('restore: '+str(exc))
        for sub in subs: sub.unregister()
        report['passed']=not report['failures'] and report['restored']
        save()
    print(json.dumps(dict(passed=report['passed'],failures=report['failures'],restored=report['restored'])))
    return 0 if report['passed'] else 1


if __name__=='__main__': raise SystemExit(main())
