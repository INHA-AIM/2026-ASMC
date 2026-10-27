#!/usr/bin/env python3
"""Opt-in, bounded MORAI-only motion diagnostic; restores original Ego state.

No spawning, training, weather changes, or ROS control publishers. Uses the same
gRPC throttle/brake and degrees/40 steering convention as Capture collection.
"""
import argparse
import json
import math
from pathlib import Path
import subprocess
import sys
import time

import numpy as np


def phase_command(seconds):
    if seconds<2:return 'accelerate',5.5,0.
    if seconds<6:return 'hold',5.5,0.
    if seconds<10:return 'brake',0.,0.
    if seconds<12:return 'rest',0.,0.
    if seconds<14:return 'accelerate2',4.5,0.
    if seconds<21:return 'steer',4.5,.22*math.sin(2*math.pi*(seconds-14)/3.5)
    return 'brake2',0.,0.


def project_route(points,position):
    a=points[:-1,:2];delta=np.diff(points[:,:2],axis=0)
    fraction=np.clip(np.sum((position-a)*delta,axis=1)/np.sum(delta*delta,axis=1),0,1)
    closest=a+fraction[:,None]*delta;at=int(np.argmin(np.linalg.norm(closest-position,axis=1)))
    length=np.linalg.norm(delta,axis=1);stations=np.r_[0,np.cumsum(length)]
    normal=np.array([-delta[at,1],delta[at,0]])/length[at]
    return stations[at]+fraction[at]*length[at],float((position-closest[at])@normal),stations


def main(args):
    if not args.execute:raise RuntimeError('explicit --execute required; MORAI only')
    args.output.mkdir(parents=True,exist_ok=False)
    sys.path[:0]=[str(args.grpc_root),str(args.grpc_root/'api'),str(args.grpc_root/'proto')]
    import grpc,rospy,rosgraph
    from google.protobuf.json_format import MessageToDict
    from morai.actor.actor_pb2_grpc import ActorStub
    from morai.actor.actor_get_pb2 import GetAllActorsFilter
    from morai.actor.actor_control_pb2 import VehicleCtrlCmd
    from morai.actor.actor_set_pb2 import SetTransformParam,VehicleGear,VehicleControlModeParam
    from morai.actor.actor_enum_pb2 import VEHICLE_CONTROL_AUTO_MODE,GEAR_MODE_D
    from morai.common.object_info_pb2 import ObjectInfo
    from morai.common.enum_pb2 import OBJECT_TYPE_VEHICLE
    rospy.init_node('tunnel_motion_diagnostic',anonymous=True,disable_signals=True)
    master=rosgraph.Master(rospy.get_name())
    writers=[(t,p) for t,p in master.getSystemState()[0] if 'ctrl_cmd' in t and p]
    if writers:raise RuntimeError('other control publishers exist: '+str(writers))
    actors=ActorStub(grpc.insecure_channel(args.address))
    ego=ObjectInfo(object_type=OBJECT_TYPE_VEHICLE);ego.id.value='Ego'
    filter=GetAllActorsFilter(vehicle=True,pedestrian=True,obstacle=True)
    before=actors.GetActorState(ego,timeout=2)
    original_mode=actors.GetVehicleControlMode(ego,timeout=2).mode
    if math.sqrt(sum(getattr(before.global_velocity,k)**2 for k in 'xyz'))>.05:raise RuntimeError('Ego must be stationary')
    all_before=actors.GetAllActorsState(filter,timeout=2)
    if len(all_before.states)!=1 or all_before.states[0].actor_info.id.value!='Ego':
        raise RuntimeError('bounded trial requires empty scene apart from Ego; no actors are removed')
    links=json.loads(args.link_set.read_text());points=np.array(next(r['points'] for r in links if r['idx']=='A2256W000126'))
    _,_,stations=project_route(points,points[0,:2]);start_s=20.
    at=int(np.searchsorted(stations,start_s)-1);f=(start_s-stations[at])/(stations[at+1]-stations[at])
    xyz=points[at]+f*(points[at+1]-points[at]);heading=math.atan2(*(points[at+1,:2]-points[at,:2])[::-1])
    transform=type(before.transform)();transform.CopyFrom(before.transform)
    transform.location.x,transform.location.y,transform.location.z=float(xyz[0]),float(xyz[1]),float(xyz[2]+.36)
    transform.rotation.x=0.;transform.rotation.y=0.;transform.rotation.z=math.degrees(heading)
    report=dict(ego_before=MessageToDict(before),original_mode=original_mode,
        start_transform=MessageToDict(transform),link='A2256W000126',speed_limit_mps=6.5,
        lateral_limit_m=.6,command_seconds=25.,actor_count_before=len(all_before.states))
    def checked(method,request,timeout=.5):
        result=getattr(actors,method)(request,timeout=timeout)
        if result.status!=1:raise RuntimeError(method+' rejected: '+str(result))
        return result
    def control(throttle=0.,brake=0.,steer=0.):
        checked('ControlVehicle',VehicleCtrlCmd(actor_info=ego,long_cmd_type=1,
            throttle=throttle,brake=brake,steer=steer))
    def hold_brake(seconds):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            control(brake=1.);time.sleep(.05)
    def reset_lidar():
        if not args.lidar_reset_service:return
        from std_srvs.srv import Trigger
        rospy.wait_for_service(args.lidar_reset_service,timeout=3.)
        reply=rospy.ServiceProxy(args.lidar_reset_service,Trigger)()
        if not reply.success:raise RuntimeError('LiDAR reset failed: '+reply.message)
        report.setdefault('lidar_resets',[]).append(dict(service=args.lidar_reset_service,
            ros_ns=rospy.Time.now().to_nsec(),message=reply.message))
    capture=None;changed=False;failure=None
    try:
        changed=True
        checked('SetVehicleControlMode',VehicleControlModeParam(actor_info=ego,mode=VEHICLE_CONTROL_AUTO_MODE))
        control(brake=1.)
        checked('SetTransform',SetTransformParam(actor_info=ego,transform=transform),timeout=2.)
        hold_brake(3.)
        rest=actors.GetActorState(ego,timeout=1)
        report['settled_before_drive']=MessageToDict(rest)
        reset_lidar()  # New sensor/track generation after an intentional teleport.
        if abs(rest.transform.location.z-transform.location.z)>.25:raise RuntimeError('unsettled road height')
        if sum(getattr(rest.global_velocity,k)**2 for k in 'xyz')>.0025:raise RuntimeError('not stopped after placement')
        capture=subprocess.Popen([sys.executable,str(Path(__file__).with_name('capture_imu_lane.py')),
            '--duration','35','--sample-hz','8','--output',str(args.output/'capture')],
            stdout=(args.output/'capture.log').open('w'),stderr=subprocess.STDOUT)
        hold_brake(3.)
        checked('SetVehicleGear',VehicleGear(actor_info=ego,gear=GEAR_MODE_D))
        begin=time.monotonic();maximum_error=0.;maximum_speed=0.
        with (args.output/'poses.jsonl').open('w') as log:
            while time.monotonic()-begin<25:
                sent_ros=rospy.Time.now().to_nsec();sent=time.monotonic()
                reply=actors.GetAllActorsState(filter,timeout=.5)
                received=time.monotonic();received_ros=rospy.Time.now().to_nsec()
                if len(reply.states)!=1:raise RuntimeError('another actor appeared during motion')
                state=reply.states[0]
                if state.actor_info.id.value!='Ego':raise RuntimeError('Ego state missing')
                speed=math.sqrt(sum(getattr(state.global_velocity,k)**2 for k in 'xyz'))
                station,error,stations=project_route(points,np.array([state.transform.location.x,state.transform.location.y]))
                if speed>6.5 or abs(error)>.6 or station>105:raise RuntimeError('motion envelope exceeded')
                if any(s!='Ego' for s in state.vehicle_state.collision_objects):raise RuntimeError('collision during trial')
                if received-sent>.2:raise RuntimeError('RPC state too slow for control')
                phase,target,offset=phase_command(time.monotonic()-begin)
                target_s=min(station+5.,stations[-1]);i=min(len(points)-2,int(np.searchsorted(stations,target_s)-1))
                f=(target_s-stations[i])/(stations[i+1]-stations[i]);aim=points[i,:2]+f*(points[i+1,:2]-points[i,:2])
                tangent=points[i+1,:2]-points[i,:2];normal=np.array([-tangent[1],tangent[0]])/np.linalg.norm(tangent)
                aim+=normal*offset
                yaw=math.radians(state.transform.rotation.z);difference=aim-np.array([state.transform.location.x,state.transform.location.y])
                lateral=-math.sin(yaw)*difference[0]+math.cos(yaw)*difference[1]
                steering=math.atan2(6.*lateral,float(difference@difference))
                normalized=float(np.clip(math.degrees(steering)/40.,-.45,.45))
                effort=.35*(target-speed)
                throttle=float(np.clip(effort,0,.65));brake=float(np.clip(-effort,0,.85))
                if target==0:throttle=0.;brake=.85
                control(throttle,brake,normalized)
                record=dict(monotonic=received,ros_request_ns=sent_ros,ros_receive_ns=received_ros,
                    rpc_timestamp_raw=int(reply.timestamp),rpc_roundtrip_ms=1000*(received-sent),
                    timestamp_note='API raw timestamp retained; microsecond Unix interpretation must be checked against sensor source clock',
                    phase=phase,target_mps=target,speed_mps=speed,station_m=station,lateral_error_m=error,
                    throttle=throttle,brake=brake,steer_normalized=normalized,state=MessageToDict(state))
                log.write(json.dumps(record)+'\n');log.flush()
                maximum_error=max(maximum_error,abs(error));maximum_speed=max(maximum_speed,speed)
                time.sleep(max(0,.05-(time.monotonic()-sent)))
        report.update(maximum_lateral_error_m=maximum_error,maximum_speed_mps=maximum_speed)
    except BaseException as error:
        failure=error;report['failure']=str(error)
    finally:
        if changed:
            try:
                deadline=time.monotonic()+6
                while time.monotonic()<deadline:
                    control(brake=1.)
                    stopped=actors.GetActorState(ego,timeout=.5)
                    if math.sqrt(sum(getattr(stopped.global_velocity,k)**2 for k in 'xyz'))<.05:break
                    time.sleep(.1)
                else:raise RuntimeError('cannot confirm stop; leaving brake and auto mode')
                if capture is not None:
                    deadline=time.monotonic()+(15 if failure is None else 1)
                    while capture.poll() is None and time.monotonic()<deadline:
                        control(brake=1.);time.sleep(.05)
                    if capture.poll() is None:
                        capture.terminate();capture.wait(timeout=3)
                        report['capture_incomplete']=True
                        if failure is None:failure=RuntimeError('capture did not finish')
                    elif capture.returncode and failure is None:failure=RuntimeError('capture failed')
                checked('SetTransform',SetTransformParam(actor_info=ego,transform=before.transform),timeout=2.)
                hold_brake(1.)
                reset_lidar()  # Also retire tunnel observations after restoring Ego.
                checked('SetVehicleControlMode',VehicleControlModeParam(actor_info=ego,mode=original_mode))
                # Native SetTransform/SetVehicleControlMode can reset gear to D.
                # Restore gear last, then verify actual state, not just RPC ACK.
                checked('SetVehicleGear',VehicleGear(actor_info=ego,gear=before.vehicle_state.gear_mode))
                time.sleep(2.)
                after=actors.GetActorState(ego,timeout=1.)
                mode=actors.GetVehicleControlMode(ego,timeout=1.).mode
                report['ego_after']=MessageToDict(after)
                report['restored']=(mode==original_mode and after.vehicle_state.gear_mode==before.vehicle_state.gear_mode and
                    sum((getattr(after.transform.location,k)-getattr(before.transform.location,k))**2 for k in 'xyz')<.01 and
                    sum(getattr(after.global_velocity,k)**2 for k in 'xyz')<.0025)
            except BaseException as error:
                report['restoration_error']=str(error);report['restored']=False
                if capture is not None and capture.poll() is None:capture.terminate()
        (args.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    if failure:raise failure
    if not report.get('restored'):raise RuntimeError('Ego restoration incomplete')
    print(json.dumps(report,indent=2),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--execute',action='store_true')
    p.add_argument('--lidar-reset-service',default='',help='explicit test LiDAR reset service after each teleport; never inferred')
    p.add_argument('--address',default='host.docker.internal:7789')
    p.add_argument('--grpc-root',type=Path,required=True)
    p.add_argument('--link-set',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    main(p.parse_args())
