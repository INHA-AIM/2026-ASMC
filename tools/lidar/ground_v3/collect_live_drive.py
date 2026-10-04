#!/usr/bin/env python3
"""Bounded MORAI path-following capture; preserve bridge and restore stationary Ego."""
import argparse
import json
import math
from pathlib import Path
import signal
import subprocess
import sys
import time


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('output', type=Path)
    p.add_argument('--route', nargs='+', required=True)
    p.add_argument('--duration', type=float, required=True)
    p.add_argument('--speed-kph', type=float, default=8)
    p.add_argument('--start-distance-m', type=float, default=3)
    p.add_argument('--method', choices=['all', 'jaeho', 'patchwork', 'linefit'], default='all')
    p.add_argument('--controller', choices=['pure_pursuit','native_cruise'], default='pure_pursuit')
    p.add_argument('--longitudinal', choices=['speed_mode','throttle_pi'], default='speed_mode')
    p.add_argument('--map', type=Path, default=Path('/data/R_KR_PG_K-city_2025/link_set.json'))
    p.add_argument('--host', default='host.docker.internal')
    p.add_argument('--confirm-simulator-move', action='store_true')
    a = p.parse_args()
    if not a.confirm_simulator_move or not 5 <= a.duration <= 300 or not 0 < a.speed_kph <= 30:
        p.error('explicit simulator movement and bounded duration/speed required')
    root = Path(__file__).resolve().parents[3]
    sys.path[:0] = [str(root/'tools/grpc_inha_univ/src/proto'), str(root/'src/data_collection/src')]
    import grpc
    import rospy
    import rosnode
    from google.protobuf.json_format import MessageToDict
    from morai.actor.actor_pb2_grpc import ActorStub
    from morai.actor.actor_get_pb2 import GetAllActorsFilter
    from morai.actor.actor_control_pb2 import VehicleCtrlCmd
    from morai.actor.actor_set_pb2 import (SetTransformParam, SetVelocityParam, VehicleGear,
        VehicleControlModeParam, VehicleRoute, EgoCruiseControl)
    from morai.actor.actor_enum_pb2 import (VEHICLE_CONTROL_KEYBOARD, VEHICLE_CONTROL_CRUISE_MODE, VEHICLE_CONTROL_AUTO_MODE,
        GEAR_MODE_P, GEAR_MODE_D, EGO_CRUISE_TYPE_CONSTANT)
    from morai.common.object_info_pb2 import ObjectInfo
    from morai.common.enum_pb2 import OBJECT_TYPE_VEHICLE, STATUS_CODE_SUCCESS
    from data_collection.bag_run import build_rosbag_command, sha256_file
    rospy.init_node('ground_v3_drive_capture', anonymous=True)
    if '/ground_compare_live' in rosnode.get_node_names():
        raise RuntimeError('existing ground comparison node; refusing duplicate ownership')
    api = ActorStub(grpc.insecure_channel(a.host+':7789'))
    ego = ObjectInfo(); ego.id.value = 'Ego'; ego.object_type = OBJECT_TYPE_VEHICLE
    original = api.GetActorState(ego, timeout=5)
    original_mode = api.GetVehicleControlMode(ego, timeout=5).mode
    if math.sqrt(sum(getattr(original.velocity,k)**2 for k in 'xyz')) > .15:
        raise RuntimeError('requires stationary Ego before capture')
    if original_mode != VEHICLE_CONTROL_KEYBOARD:
        raise RuntimeError('requires idle keyboard mode; unknown active controller is not replaced')
    links = {v['idx']:v for v in json.loads(a.map.read_text())}
    ids = [s if s.startswith('A2256W') else 'A2256W'+s.zfill(6) for s in a.route]
    for before, after in zip(ids, ids[1:]):
        if links[before]['to_node_idx'] != links[after]['from_node_idx']:
            raise ValueError('disconnected route '+before+' -> '+after)
    path=[];path_links=[];stations=[]
    for ident in ids:
        for point in links[ident]['points']:
            if path and math.dist(path[-1][:2],point[:2])<1e-6:continue
            stations.append((stations[-1]+math.dist(path[-1][:2],point[:2])) if path else 0.)
            path.append(point);path_links.append(ident)
    pts = links[ids[0]]['points']; distance = 0; index = 0
    for i in range(1,len(pts)-1):
        distance += math.dist(pts[i-1][:2],pts[i][:2])
        if distance >= a.start_distance_m:
            index = i; break
    nearest = min((point for row in links.values() for point in row['points']),
        key=lambda v:(v[0]-original.transform.location.x)**2+(v[1]-original.transform.location.y)**2)
    origin_ground_offset = original.transform.location.z-nearest[2]
    if not .1 < origin_ground_offset < .8:
        raise ValueError('unexpected Ego-origin ground offset '+str(origin_ground_offset))
    start = SetTransformParam(); start.transform.CopyFrom(original.transform)
    for k,v in zip('xyz',pts[index]): setattr(start.transform.location,k,v)
    start.transform.location.z += origin_ground_offset
    start.transform.rotation.x = 0; start.transform.rotation.y = 0
    start.transform.rotation.z = math.degrees(math.atan2(pts[index+1][1]-pts[index][1],pts[index+1][0]-pts[index][0]))
    a.output.mkdir(parents=True,exist_ok=False)
    cfg = root/'src/asmc_lidar/config/ground_v3/roll_pitch_all.yaml'
    manifest = {'schema':'ground-v3-live-drive-v1','method':a.method,'duration_request_s':a.duration,
        'controller':a.controller,'longitudinal':a.longitudinal,'cross_track_abort_m':1.0,'lookahead_m':5.0,
        'route':ids,'speed_request_kph':a.speed_kph,'original_state':MessageToDict(original,preserving_proto_field_name=True),
        'original_mode':original_mode,'map_sha256':sha256_file(str(a.map)),
        'comparison_config_sha256':sha256_file(str(cfg)),
        'sensor_config_sha256':sha256_file(str(root/'config/morai/sensors/ASMC_sensor_set_GT.json')),
        'original_route_query_available':False,'origin_ground_offset_m':origin_ground_offset,
        'bridge_params':{n:rospy.get_param(n,{}) for n in ['/morai_sensor_bridge','/morai_udp_bridge']},
        'complete':False,'restored':False,'samples':[]}
    def save():
        (a.output/'manifest.json').write_text(json.dumps(manifest,indent=2,sort_keys=True)+'\n')
    def call(method, request):
        request.actor_info.CopyFrom(ego)
        result = method(request,timeout=4)
        if result.status != STATUS_CODE_SUCCESS: raise RuntimeError(str(result))
    def stop():
        call(api.SetVehicleEgoCruise,EgoCruiseControl(cruise_on=False))
        call(api.SetVehicleControlMode,VehicleControlModeParam(mode=VEHICLE_CONTROL_KEYBOARD))
        call(api.ControlVehicle,VehicleCtrlCmd(long_cmd_type=1,brake=1,throttle=0,steer=0))
        call(api.SetVelocity,SetVelocityParam(velocity=0))
        call(api.SetVehicleGear,VehicleGear(gear=GEAR_MODE_P))
    processes=[]; logs=[]
    def launch(cmd,name):
        log=(a.output/(name+'.log')).open('x');logs.append(log)
        proc=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(proc);return proc
    def interrupt(_signum,_frame): raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM,interrupt);signal.signal(signal.SIGINT,interrupt)
    topics=['/velodyne_points','/velodyne_points_semantic','/velodyne_points_instance','/imu','/gps',
        '/Ego_topic','/Object_topic','/CollisionData','/image_jpeg/compressed','/tf_static',
        '/morai_sensor_bridge/diagnostics','/morai_udp_bridge/diagnostics',
        '/lidar_candidate/ground_live/status','/lidar_candidate/ground_live/fit_markers']
    for method in (['jaeho','patchwork','linefit'] if a.method=='all' else [a.method]):
        topics.extend('/lidar_candidate/ground_live/'+method+'/'+kind for kind in
            ['ground','nonground','uncertain','structural_removed','native_retained'])
    manifest['topics']=topics;save()
    try:
        stop(); call(api.SetTransform,start);time.sleep(3)
        review=launch(['roslaunch','asmc_lidar','ground_compare_live.launch','rviz:=false',
            'start_bridge:=false','method:='+a.method,'comparison_config:='+str(cfg)],'review')
        recorder=launch(build_rosbag_command({'compression':'lz4'},str(a.output/'drive'),topics),'rosbag')
        time.sleep(3)
        route=VehicleRoute(decision_range=10)
        for i,ident in enumerate(ids):
            link=route.links.add();link.id.value=ident;link.waypoint_idx=index if i==0 else 0
        call(api.SetVehicleGear,VehicleGear(gear=GEAR_MODE_D))
        if a.controller=='native_cruise':
            call(api.SetVehicleRoute,route)
            call(api.SetVehicleControlMode,VehicleControlModeParam(mode=VEHICLE_CONTROL_CRUISE_MODE))
            call(api.SetVehicleEgoCruise,EgoCruiseControl(cruise_on=True,
                cruise_type=EGO_CRUISE_TYPE_CONSTANT,constant_velocity=a.speed_kph))
        else:
            call(api.SetVehicleControlMode,VehicleControlModeParam(mode=VEHICLE_CONTROL_AUTO_MODE))
        wheelbase=api.GetVehicleSpec(ego,timeout=4).wheel_base
        if not 1< wheelbase < 5:raise ValueError('invalid simulator wheelbase')
        manifest['wheelbase_m']=wheelbase
        manifest['measurement_start_wall_s']=time.time();save()
        probe=launch([sys.executable,'-B',str(root/'tools/lidar/ground_v3/live_probe.py'),
            str(a.output/'live_metrics.json'),'--duration',str(a.duration),'--method',a.method],'probe')
        started=time.monotonic(); last_print=-5;nearest_index=index
        integral=0.;previous_time=started;reverse_started=None
        while time.monotonic()-started<a.duration:
            if review.poll() is not None or recorder.poll() is not None:
                raise RuntimeError('review or rosbag exited')
            now=time.monotonic()-started
            state=api.GetActorState(ego,timeout=3)
            pos=state.transform.location
            nearest_index=min(range(max(0,nearest_index-10),min(len(path),nearest_index+80)),
                key=lambda i:(path[i][0]-pos.x)**2+(path[i][1]-pos.y)**2)
            cte=math.hypot(path[nearest_index][0]-pos.x,path[nearest_index][1]-pos.y)
            if cte>1.:raise RuntimeError('cross-track abort at '+str(cte)+'m')
            target=nearest_index
            while target+1<len(path) and stations[target]-stations[nearest_index]<5.:target+=1
            if stations[-1]-stations[nearest_index]<5.:raise RuntimeError('route end before requested duration')
            yaw=math.radians(state.transform.rotation.z)
            dx=path[target][0]-pos.x;dy=path[target][1]-pos.y
            alpha=math.atan2(math.sin(math.atan2(dy,dx)-yaw),math.cos(math.atan2(dy,dx)-yaw))
            steer=max(-.55,min(.55,math.atan2(2*wheelbase*math.sin(alpha),max(1.,math.hypot(dx,dy)))))
            if a.controller=='pure_pursuit':
                if a.longitudinal=='throttle_pi':
                    # Simulator body-forward velocity is km/h; retain speed-mode as a separate control condition.
                    dt=max(.001,min(.3,time.monotonic()-previous_time));previous_time=time.monotonic()
                    error=a.speed_kph-state.velocity.x
                    integral=max(-10.,min(60.,integral+error*dt))
                    command=.10+.045*error+.012*integral
                    call(api.ControlVehicle,VehicleCtrlCmd(long_cmd_type=1,steer=steer,
                        throttle=max(0.,min(.65,command)),brake=max(0.,min(.35,-command))))
                else:
                    call(api.ControlVehicle,VehicleCtrlCmd(long_cmd_type=2,steer=steer,velocity=a.speed_kph))
                if state.velocity.x < -.5:
                    reverse_started=now if reverse_started is None else reverse_started
                    if now-reverse_started>1.:raise RuntimeError('reverse motion abort')
                else:reverse_started=None
            row={'elapsed_s':now,'wall_s':time.time(),'state':MessageToDict(state,preserving_proto_field_name=True)}
            row.update(map_link=path_links[nearest_index],route_station_m=stations[nearest_index],
                cross_track_point_distance_m=cte,command_steer_rad=steer)
            manifest['samples'].append(row)
            if now-last_print>=5:
                print(json.dumps({'elapsed_s':round(now,1),'link':path_links[nearest_index],
                    'station_m':round(stations[nearest_index],1),'cte_m':round(cte,3),
                    'position':row['state']['transform']['location'],'velocity':row['state'].get('velocity',{})}),flush=True)
                last_print=now;save()
            if now>15 and all(math.sqrt(sum(float(r['state'].get('velocity',{}).get(k,0))**2 for k in 'xyz'))<.2
                for r in manifest['samples'][-30:]):
                raise RuntimeError('cruise did not move or remained stopped; preserve evidence')
            time.sleep(.075)
        manifest['measurement_end_wall_s']=time.time()
        stop()
        probe.wait(timeout=15)
        manifest['complete']=probe.returncode==0
    except BaseException as exc:
        manifest['error']=repr(exc)
        raise
    finally:
        errors=[]
        for method,request in [(api.SetVehicleEgoCruise,EgoCruiseControl(cruise_on=False)),
            (api.SetVehicleControlMode,VehicleControlModeParam(mode=VEHICLE_CONTROL_KEYBOARD)),
            (api.ControlVehicle,VehicleCtrlCmd(long_cmd_type=1,brake=1,steer=0)),
            (api.SetVelocity,SetVelocityParam(velocity=0)),
            (api.SetVehicleGear,VehicleGear(gear=GEAR_MODE_P)),
            (api.SetTransform,SetTransformParam(transform=original.transform)),
            (api.SetVehicleGear,VehicleGear(gear=original.vehicle_state.gear_mode)),
            (api.SetVehicleControlMode,VehicleControlModeParam(mode=original_mode))]:
            try: call(method,request)
            except Exception as exc: errors.append(str(exc))
        # RPC success alone does not prove pose parity: this simulator adds about
        # 1.34 deg on a SetTransform/GetActorState yaw round trip. Bound correction
        # by readback instead of baking that simulator-specific offset into LiDAR.
        try:
            target_yaw=original.transform.rotation.z
            requested_yaw=target_yaw
            for attempt in range(3):
                time.sleep(.5)
                restored_state=api.GetActorState(ego,timeout=4)
                delta=(restored_state.transform.rotation.z-target_yaw+180)%360-180
                distance=math.sqrt(sum((getattr(restored_state.transform.location,k)-
                    getattr(original.transform.location,k))**2 for k in 'xyz'))
                if abs(delta)<=.2 and distance<=.1:break
                if attempt==2:raise RuntimeError('pose restore readback exceeds tolerance')
                requested_yaw-=delta
                correction=SetTransformParam(transform=original.transform)
                correction.transform.rotation.z=requested_yaw
                call(api.SetTransform,correction)
                # SetTransform can reset the simulator gear to D.
                call(api.SetVelocity,SetVelocityParam(velocity=0))
                call(api.SetVehicleGear,VehicleGear(gear=original.vehicle_state.gear_mode))
                call(api.SetVehicleControlMode,VehicleControlModeParam(mode=original_mode))
            manifest['restore_verification']={'distance_m':distance,'yaw_error_deg':delta,
                'state':MessageToDict(restored_state,preserving_proto_field_name=True)}
        except Exception as exc:errors.append(str(exc))
        manifest['restore_errors']=errors;manifest['restored']=not errors
        for proc in reversed(processes):
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
                try:proc.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    proc.terminate()
                    try:proc.wait(timeout=10)
                    except subprocess.TimeoutExpired:errors.append('process did not stop: '+str(proc.args))
        for log in logs:log.close()
        manifest['bags']=[{'path':f.name,'bytes':f.stat().st_size,'sha256':sha256_file(str(f))}
            for f in sorted(a.output.glob('*.bag'))]
        manifest['recording_complete']=bool(manifest['bags']) and not list(a.output.glob('*.active'))
        save();print(json.dumps({'complete':manifest['complete'],'restored':manifest['restored'],
            'recording_complete':manifest['recording_complete'],'errors':errors}),flush=True)


if __name__=='__main__':main()
