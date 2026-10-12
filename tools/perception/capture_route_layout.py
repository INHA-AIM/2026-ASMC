#!/usr/bin/env python3
"""Plan, create and remove only ledger-owned Capture collection actors."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import time
import uuid

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src/data_collection/src'))
from data_collection.capture_collection import allocation, route_sample, footprint_clear


def save(path, rows):
    temp=path.with_suffix('.part')
    temp.write_text(''.join(json.dumps(r,ensure_ascii=False)+'\n' for r in rows))
    temp.replace(path)


def plan(points, count, policy=None, phase=.5, side_phase=0):
    length=sum(math.hypot(b[0]-a[0],b[1]-a[1]) for a,b in zip(points,points[1:]))
    rows=[]
    policy=policy or {}
    for i,(label,asset) in enumerate(allocation(count,policy.get('presence'),policy.get('asset_pools'),policy.get('asset_offset',0),policy.get('balance_all_classes',False),policy.get('max_class_share'))):
        station=(i+phase)*length/count
        point,yaw=route_sample(points,station)
        offset=(1 if (i+side_phase)%2==0 else -1)*5.
        rows.append(dict(slot=i,station_m=station,target_class=label,api_name=asset,
            actor_type='pedestrian' if label in ('deer','dog') else 'obstacle',
            route_surface_z=point[2],yaw_deg=math.degrees(yaw),
            location=dict(x=point[0]-offset*math.sin(yaw),y=point[1]+offset*math.cos(yaw),z=point[2]),
            created=False,placement_approved=False,identity_approved=False,
            native_ids=[],height_basis='local_route_surface_estimate_requires_ground_review'))
    return rows


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('operation',choices=('plan','create','cleanup'))
    p.add_argument('--ledger',type=Path,required=True)
    p.add_argument('--route',type=Path,default=ROOT/'R_KR_PG_KATRI/2026_molit_comp_global_path.txt')
    p.add_argument('--count',type=int,default=120)
    p.add_argument('--grpc-host',default='host.docker.internal')
    p.add_argument('--near-ego',action='store_true',help='Six-object stationary GT/height pilot only')
    p.add_argument('--calibrate-native-pivot',action='store_true',help='Stationary six-object probes only; skip bbox-bottom correction and require visual calibration')
    p.add_argument('--ground-calibration',type=Path,default=ROOT/'config/morai/capture_asset_ground_v2.json')
    p.add_argument('--only-native-id',action='append',default=[],help='Cleanup only these owned native roots; repeat to select')
    p.add_argument('--pause-static',action='store_true',help='Pause owned collection actors after native physics settling; keep physics and GT bbox offsets')
    p.add_argument('--allocation-policy',type=Path,help='Frozen image-presence and known asset-pool policy for a new layout')
    p.add_argument('--station-phase',type=float,default=.5,help='Placement phase in [0,1) within each route interval')
    p.add_argument('--side-phase',type=int,choices=(0,1),default=0)
    a=p.parse_args()
    if a.only_native_id and a.operation!='cleanup':p.error('--only-native-id is restricted to cleanup')
    if not 0<=a.station_phase<1:p.error('--station-phase must be in [0,1)')
    if a.near_ego and a.count!=6:p.error('near-ego is restricted to the six-object pilot')
    if a.calibrate_native_pivot and (not a.near_ego or a.count!=6):p.error('pivot calibration requires stationary near-ego six-object probes')
    points=[tuple(map(float,line.split()[:3])) for line in a.route.read_text().splitlines() if line.strip()]
    if a.operation=='plan':
        if a.ledger.exists():raise RuntimeError('ledger exists; choose a new layout ID')
        policy=json.loads(a.allocation_policy.read_text()) if a.allocation_policy else None
        a.ledger.parent.mkdir(parents=True,exist_ok=True);rows=plan(points,a.count,policy,a.station_phase,a.side_phase)
        for r in rows:r['route_sha256']=hashlib.sha256(a.route.read_bytes()).hexdigest()
        for r in rows:
            r['station_phase']=a.station_phase;r['side_phase']=a.side_phase
            if a.allocation_policy:r['allocation_policy_sha256']=hashlib.sha256(a.allocation_policy.read_bytes()).hexdigest()
        save(a.ledger,rows);print(json.dumps(dict(planned=len(rows),ledger=str(a.ledger))))
        return
    sys.path.insert(0,str(ROOT/'tools/grpc_inha_univ/src/proto'))
    import grpc
    from google.protobuf.json_format import MessageToDict
    from morai.actor.actor_pb2_grpc import ActorStub
    from morai.simulator.simulator_pb2_grpc import SimulatorStub
    from morai.simulator.category_obstacles_pb2 import CategoryObstacles
    from morai.actor.actor_get_pb2 import GetAllActorsFilter
    from morai.common.object_info_pb2 import ObjectInfo
    from morai.common.enum_pb2 import OBJECT_TYPE_VEHICLE,OBJECT_TYPE_OBSTACLE,OBJECT_TYPE_PEDESTRIAN
    from morai.common.type_pb2 import Transform,Vector3
    from morai.actor.actor_spawn_pb2 import ActorSpawnInfo,ObstacleSpawnParam,PedestrianSpawnParam,VehicleSpawnParam
    from morai.actor.actor_set_pb2 import SetTransformParam, EnableActor
    channel=grpc.insecure_channel(a.grpc_host+':7789',options=[('grpc.max_receive_message_length',64<<20)])
    actors=ActorStub(channel);sim=SimulatorStub(channel)
    rows=[json.loads(line) for line in a.ledger.read_text().splitlines() if line.strip()]
    types={'pedestrian':OBJECT_TYPE_PEDESTRIAN,'obstacle':OBJECT_TYPE_OBSTACLE,'vehicle':OBJECT_TYPE_VEHICLE}
    ego=ObjectInfo(object_type=OBJECT_TYPE_VEHICLE);ego.id.value='Ego'
    current=actors.GetActorState(ego,timeout=5)
    if sum(getattr(current.global_velocity,k)**2 for k in 'xyz')>.01:
        raise RuntimeError('stationary Ego required')
    if a.operation=='cleanup':
        requested=set(a.only_native_id)
        roots=[str(r['native_ids'][0]) for r in rows if r.get('created') and not r.get('removed') and r.get('native_ids')]
        if requested and any(roots.count(n)!=1 for n in requested):raise RuntimeError('selected cleanup ID is absent or ambiguous')
        for row in rows:
            if not row.get('created') or row.get('removed'):continue
            if requested and str(row['native_ids'][0]) not in requested:continue
            if not row['id'].startswith('capture_'+row['generation']+'_') or not row['client_key'].startswith('capture_'):
                raise RuntimeError('unowned actor record')
            info=ObjectInfo(object_type=types[row['actor_type']],client_key=row['client_key']);info.id.value=row['id']
            actual=actors.GetActorState(info,timeout=5)
            if any(abs(getattr(actual.transform.location,k)-row['settled_location'][k])>.2 for k in 'xyz'):
                owned=actors.GetAllActorsState(GetAllActorsFilter(client_key=row['client_key'],vehicle=True,pedestrian=True,obstacle=True),timeout=5)
                matches=[s for s in owned.states if s.actor_info.id.value==row['id']]
                if actual.actor_info.id.value!=row['id'] or len(matches)!=1:
                    raise RuntimeError('moved actor ownership unresolved; refusing cleanup: '+row['id'])
                row['cleanup_after_physical_motion']=dict(verified_client_key=row['client_key'],
                    actor_id=row['id'],actual=MessageToDict(actual,preserving_proto_field_name=True))
            reply=actors.DestroyActor(info,timeout=5);row['cleanup']=MessageToDict(reply)
            row['removed']=reply.status==1;save(a.ledger,rows)
        return
    if any(r.get('created') for r in rows):raise RuntimeError('creation cannot overwrite a used ledger')
    calibration=json.loads(a.ground_calibration.read_text())['assets'] if a.ground_calibration.exists() else {}
    available=sim.GetAvailableObject(CategoryObstacles(vehicle=True,pedestrian=True,obstacle=True),timeout=5)
    pools=dict(pedestrian=set(available.pedestrian),obstacle=set(available.obstacle),
               vehicle=set(available.surround_vehicle))
    original=actors.GetAllActorsState(GetAllActorsFilter(vehicle=True,pedestrian=True,obstacle=True),timeout=5)
    occupied=[(s.bounding_box.location.x,s.bounding_box.location.y,
        math.hypot(s.bounding_box.extent.x,s.bounding_box.extent.y)) for s in original.states]
    generation=uuid.uuid4().hex[:12];client='capture_'+generation
    if a.near_ego:
        yaw=math.radians(current.transform.rotation.z)
        ground=current.bounding_box.location.z-current.bounding_box.extent.z
        for i,row in enumerate(rows):
            forward=10.+(i//2)*8.;lateral=5. if i%2==0 else -5.
            row['location']=dict(x=current.transform.location.x+forward*math.cos(yaw)-lateral*math.sin(yaw),
                y=current.transform.location.y+forward*math.sin(yaw)+lateral*math.cos(yaw),z=ground)
            row.update(route_surface_z=ground,yaw_deg=math.degrees(yaw),height_basis='stationary_local_ego_support_estimate')
    try:
        for row in rows:
            row.update(generation=generation,client_key=client,id='capture_'+generation+'_'+str(row['slot']))
            save(a.ledger,rows)
            if row['api_name'] not in pools[row['actor_type']]:
                row['reason']='api_asset_unavailable';save(a.ledger,rows);continue
            vehicle=row['actor_type']=='vehicle'
            if not vehicle and not a.calibrate_native_pivot and row['api_name'] not in calibration:
                row['reason']='asset_pivot_not_calibrated';save(a.ledger,rows);continue
            pivot_offset=0 if vehicle else (float(row.get('calibration_origin_above_ground_m',0)) if a.calibrate_native_pivot else calibration[row['api_name']]['origin_above_ground_m'])
            row['pivot_offset_m']=pivot_offset
            if not a.calibrate_native_pivot and not vehicle:
                row['ground_calibration_sha256']=hashlib.sha256(a.ground_calibration.read_bytes()).hexdigest()
            info=ObjectInfo(object_type=types[row['actor_type']],client_key=client);info.id.value=row['id']
            pose=Transform(location=Vector3(**row['location']),rotation=Vector3(z=row['yaw_deg']))
            pose.location.z+=pivot_offset
            spawn=ActorSpawnInfo(actor_info=info,model_name=row['api_name'],label=client,transform=pose)
            request=(VehicleSpawnParam(spawn_info=spawn,velocity=0,pause=True) if vehicle else
                PedestrianSpawnParam(spawn_info=spawn,velocity=0,start_action=False) if row['actor_type']=='pedestrian' else
                ObstacleSpawnParam(spawn_info=spawn,scale=Vector3(x=1,y=1,z=1)))
            reply=getattr(actors,'Spawn'+row['actor_type'].title())(request,timeout=5)
            row['spawn_response']=MessageToDict(reply);row['created']=reply.status==1;save(a.ledger,rows)
            if reply.status!=1:row['reason']='spawn_rejected';save(a.ledger,rows);continue
            time.sleep(.2);state=actors.GetActorState(info,timeout=5)
            time.sleep(.2);state=actors.GetActorState(info,timeout=5)
            if vehicle:
                if actors.SetAI(EnableActor(actor_info=info,enable=False),timeout=3).status!=1:
                    raise RuntimeError('owned collection vehicle AI disable rejected')
                # Vehicle SetTransform is world-relative. Derive each model's
                # anchor offset from its actual native bbox; RGB approval is separate.
                bottom=state.bounding_box.location.z-state.bounding_box.extent.z
                delta=row['route_surface_z']-bottom
                if not math.isfinite(delta) or abs(delta)>5:
                    raise RuntimeError('vehicle support alignment exceeds measured 5m bound')
                target=Transform();target.CopyFrom(state.transform);target.location.z+=delta
                row['vehicle_support_alignment']=dict(before=MessageToDict(state,preserving_proto_field_name=True),
                    requested=MessageToDict(target,preserving_proto_field_name=True),delta_z_m=delta,
                    basis='native bbox support aligned to local route estimate; RGB terrain review required')
                if actors.SetTransform(SetTransformParam(actor_info=info,transform=target),timeout=3).status!=1:
                    raise RuntimeError('owned vehicle support alignment rejected')
                time.sleep(.4);state=actors.GetActorState(info,timeout=5)
                pivot_offset=state.transform.location.z-row['route_surface_z']
                row['pivot_offset_m']=pivot_offset
            if a.pause_static:
                if actors.SetPause(EnableActor(actor_info=info,enable=True),timeout=3).status!=1:
                    raise RuntimeError('owned static pause rejected')
                row['static_pause_enabled']=True
            row['state']=MessageToDict(state,preserving_proto_field_name=True)
            row['settled_location']={k:getattr(state.transform.location,k) for k in 'xyz'}
            radius=math.hypot(state.bounding_box.extent.x,state.bounding_box.extent.y)
            center=(state.bounding_box.location.x,state.bounding_box.location.y)
            row['footprint_clear']=footprint_clear(points,center,radius,occupied)
            row['height_error_m']=state.bounding_box.location.z-state.bounding_box.extent.z-row['route_surface_z']
            row['bbox_bottom_error_m']=row['height_error_m']
            row['height_error_m']=state.transform.location.z-pivot_offset-row['route_surface_z']
            row['geometry_gate']=row['footprint_clear'] and abs(row['bbox_bottom_error_m'] if vehicle else row['height_error_m'])<=.2
            if a.calibrate_native_pivot:
                row.update(stationary_calibration_only=True,geometry_gate=False,
                           reason='native_origin_pivot_pending_visual_review')
            # Compare native and client-key state by exact settled pose. Native IDs are not guessed.
            matches=[]
            for attempt in range(5):
                state=actors.GetActorState(info,timeout=5)
                native=actors.GetAllActorsState(GetAllActorsFilter(vehicle=True,pedestrian=True,obstacle=True),timeout=5)
                matches=[s for s in native.states if s.actor_info.object_type==types[row['actor_type']] and
                    sum((getattr(s.transform.location,k)-getattr(state.transform.location,k))**2 for k in 'xyz')<1e-4]
                if len(matches)==1:break
                time.sleep(.2)
            row['settled_location']={k:getattr(state.transform.location,k) for k in 'xyz'}
            row['state_after_identity_settling']=MessageToDict(state,preserving_proto_field_name=True)
            radius=math.hypot(state.bounding_box.extent.x,state.bounding_box.extent.y)
            center=(state.bounding_box.location.x,state.bounding_box.location.y)
            row['footprint_clear']=footprint_clear(points,center,radius,occupied)
            row['bbox_bottom_error_m']=state.bounding_box.location.z-state.bounding_box.extent.z-row['route_surface_z']
            row['height_error_m']=state.transform.location.z-pivot_offset-row['route_surface_z']
            row['geometry_gate']=row['footprint_clear'] and abs(row['bbox_bottom_error_m'] if vehicle else row['height_error_m'])<=.2
            if len(matches)==1:
                row['native_ids']=[matches[0].actor_info.id.value]
                row['identity_approved']=True
            row['placement_approved']=False  # Numeric support estimates require RGB/ground review.
            if a.calibrate_native_pivot:row['geometry_gate']=False
            if not row['geometry_gate'] and not (a.calibrate_native_pivot and row['footprint_clear']):
                row['reason']='geometry_or_ground_gate_failed'
                reply=actors.DestroyActor(info,timeout=5)
                row['cleanup']=MessageToDict(reply);row['removed']=reply.status==1
            else:occupied.append((*center,radius))
            save(a.ledger,rows)
    finally:
        save(a.ledger,rows);channel.close()
    print(json.dumps(dict(created=sum(r['created'] and not r.get('removed',False) for r in rows),
        approved=sum(r['placement_approved'] for r in rows),ledger=str(a.ledger))))


if __name__=='__main__':main()
