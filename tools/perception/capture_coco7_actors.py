#!/usr/bin/env python3
"""Owned-actor MORAI inventory/capture; no scenario reset or global deletion.

Ground height is estimated from Ego's bounding box and attitude. Every case
remains pending visual placement review; actor boxes are NOT pixel-mask GT.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import time
import uuid
import yaml

ROOT = Path(__file__).resolve().parents[2]
GRPC = ROOT/'tools/grpc_inha_univ/src'
sys.path[:0] = [str(GRPC), str(GRPC/'api'), str(GRPC/'proto')]
import grpc
from google.protobuf.json_format import MessageToDict
from morai.simulator.simulator_pb2_grpc import SimulatorStub
from morai.simulator.category_obstacles_pb2 import CategoryObstacles
from morai.actor.actor_pb2_grpc import ActorStub
from morai.common.object_info_pb2 import ObjectInfo
from morai.common.enum_pb2 import OBJECT_TYPE_VEHICLE, OBJECT_TYPE_PEDESTRIAN, OBJECT_TYPE_OBSTACLE
from morai.common.type_pb2 import Transform, Vector3
from morai.actor.actor_spawn_pb2 import ActorSpawnInfo, VehicleSpawnParam, PedestrianSpawnParam, ObstacleSpawnParam
from morai.actor.actor_set_pb2 import EnableActor, SetTransformParam


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--catalogue', type=Path, default=ROOT/'config/morai/object_spawn_catalog_v1.yaml')
    p.add_argument('--extra-representatives', type=Path, help='Frozen experiment asset list; base catalogue preserved')
    p.add_argument('--inventory-only', action='store_true')
    p.add_argument('--limit', type=int, default=0)
    p.add_argument('--asset', nargs='+', help='Exact available catalogue API names')
    p.add_argument('--side-only', action='store_true', help='Only left/right 7m controlled placements')
    p.add_argument('--placements', action='store_true', help='Front 20/12/6m and left/right 7m')
    p.add_argument('--front-distances', type=float, nargs='+',
                   help='Explicit positive forward spawn-anchor distances in metres; replaces placements')
    p.add_argument('--capture-views', choices=['front', 'left', 'right'], nargs='+',
                   default=['front', 'left', 'right'])
    p.add_argument('--frames-per-view', type=int, default=2)
    p.add_argument('--grpc-host', default='host.docker.internal')
    a = p.parse_args()
    if a.side_only and (a.placements or a.front_distances):p.error('side-only cannot combine with front placements')
    if a.front_distances and (a.placements or any(not math.isfinite(d) or d <= 0 for d in a.front_distances)
                              or len(set(a.front_distances)) != len(a.front_distances)):
        p.error('front distances must be finite, positive, unique and used without --placements')
    if a.frames_per_view < 1 or len(set(a.capture_views)) != len(a.capture_views):
        p.error('positive frame count and unique capture views required')
    a.output.mkdir(parents=True, exist_ok=False)
    channel = grpc.insecure_channel(a.grpc_host+':7789')
    simulator, actors = SimulatorStub(channel), ActorStub(channel)
    available = simulator.GetAvailableObject(CategoryObstacles(vehicle=True,pedestrian=True,obstacle=True),timeout=10)
    policy = yaml.safe_load(a.catalogue.read_text())
    rows = policy['vehicles']+policy['coco7_representatives']
    if a.extra_representatives:
        rows += yaml.safe_load(a.extra_representatives.read_text())['representatives']
    pools = dict(vehicle=set(available.surround_vehicle),pedestrian=set(available.pedestrian),obstacle=set(available.obstacle))
    report = dict(available=MessageToDict(available),catalogue_sha256=hashlib.sha256(a.catalogue.read_bytes()).hexdigest(),
        catalogue_validation=[dict(r,available=r['api_name'] in pools[r['actor_type']]) for r in rows],
        cases=[],cleanup=[],pixel_ground_truth=False,extra_representatives_sha256=hashlib.sha256(a.extra_representatives.read_bytes()).hexdigest() if a.extra_representatives else None)
    def save(): (a.output/'capture.json').write_text(json.dumps(report,indent=2)+'\n')
    save()
    if a.inventory_only: return
    import cv2
    import numpy as np
    import rosgraph
    import rospy
    from sensor_msgs.msg import CompressedImage
    rospy.init_node('coco7_owned_actor_capture',anonymous=True)
    if any(t in ('/ctrl_cmd', '/ctrl_cmd_0') and ns for t,ns in rosgraph.Master('/coco7_capture').getSystemState()[0]):
        raise RuntimeError('stationary capture requires no public control publisher')
    ego_info=ObjectInfo(object_type=OBJECT_TYPE_VEHICLE);ego_info.id.value='Ego'
    initial=actors.GetActorState(ego_info,timeout=10)
    report['ego_before']=MessageToDict(initial)
    if sum(getattr(initial.global_velocity,k)**2 for k in 'xyz')>.01:
        raise RuntimeError('Ego must be stationary for controlled capture')
    yaw=math.radians(initial.transform.rotation.z)
    ground=initial.bounding_box.location.z-initial.bounding_box.extent.z
    topics=dict(front='/image_jpeg/compressed',left='/cam_left/image_jpeg/compressed',right='/cam_right/image_jpeg/compressed')
    types=dict(vehicle=OBJECT_TYPE_VEHICLE,pedestrian=OBJECT_TYPE_PEDESTRIAN,obstacle=OBJECT_TYPE_OBSTACLE)
    owned=[]
    try:
        valid=[r for r in report['catalogue_validation'] if r['available']]
        if a.asset:
            valid=[r for r in valid if r['api_name'] in a.asset]
            missing=set(a.asset)-{r['api_name'] for r in valid}
            if missing:raise RuntimeError('requested assets unavailable: '+str(sorted(missing)))
        if a.limit: valid=valid[:a.limit]
        placements=[('front20',20.,0.)] if not a.placements else [
            ('front20',20.,0.),('front12',12.,0.),('front6',6.,0.),('left7',0.,7.),('right7',0.,-7.)]
        if a.side_only:placements=[('left7',0.,7.),('right7',0.,-7.)]
        if a.front_distances:
            placements=[(f'front{distance:g}',distance,0.) for distance in a.front_distances]
        report['capture_arguments']=dict(front_distances=a.front_distances, views=a.capture_views,
                                         frames_per_view=a.frames_per_view)
        jobs=[(r,label,forward,lateral) for r in valid for label,forward,lateral in placements]
        for index,(row,placement,forward,lateral) in enumerate(jobs):
            info=ObjectInfo(object_type=types[row['actor_type']],client_key='coco7_evaluation')
            info.id.value='coco7_'+uuid.uuid4().hex[:12]
            pose=Transform();pose.CopyFrom(initial.transform)
            pose.location.x += forward*math.cos(yaw)-lateral*math.sin(yaw)
            pose.location.y += forward*math.sin(yaw)+lateral*math.cos(yaw)
            pose.location.z += 5
            spawn=ActorSpawnInfo(actor_info=info,model_name=row['api_name'],label='coco7 evaluation',transform=pose)
            case=dict(index=index,asset=row,placement=placement,actor_id=info.id.value,frames=[],placement_review='pending',pixel_gt=False,
                      requested_forward_anchor_m=forward,requested_lateral_anchor_m=lateral)
            report['cases'].append(case)
            try:
                request=(VehicleSpawnParam(spawn_info=spawn,pause=True) if row['actor_type']=='vehicle' else
                         PedestrianSpawnParam(spawn_info=spawn,velocity=0,start_action=False) if row['actor_type']=='pedestrian' else
                         ObstacleSpawnParam(spawn_info=spawn,scale=Vector3(x=1,y=1,z=1)))
                result=getattr(actors,'Spawn'+row['actor_type'].title())(request,timeout=10)
                case['spawn_response']=MessageToDict(result)
                if result.status!=1: raise RuntimeError(str(result))
                owned.append(info)
                if row['actor_type']=='vehicle': actors.SetAI(EnableActor(actor_info=info,enable=False),timeout=10)
                time.sleep(.25)
                state=actors.GetActorState(info,timeout=10)
                # Derive the anchor offset for EACH model; do not share a fixed z.
                case['initial_state']=MessageToDict(state)
                pose.location.z = state.transform.location.z + ground-(state.bounding_box.location.z-state.bounding_box.extent.z)
                case['requested_transform']=MessageToDict(pose)
                # Pedestrian Spawn snaps to the ground. In this MORAI build its
                # SetTransform z is ground-relative, unlike vehicle/obstacle.
                # Preserve that native placement rather than adding world z.
                native_ground=(row['actor_type']=='pedestrian' or row.get('height_policy')=='native_spawn_ground_snap')
                if native_ground:
                    case['height_policy']='native pedestrian spawn ground snap; no SetTransform'
                    if row['actor_type']!='pedestrian':case['height_policy']='explicit native obstacle ground snap; bbox height unreliable; visual review required'
                else:
                    result=actors.SetTransform(SetTransformParam(actor_info=info,transform=pose),timeout=10)
                    if result.status!=1: raise RuntimeError(str(result))
                    case['height_policy']='world-z correction from actual spawn state'
                if row['actor_type']!='obstacle': actors.SetPause(EnableActor(actor_info=info,enable=True),timeout=10)
                time.sleep(.4)
                settled=actors.GetActorState(info,timeout=10)
                case['state']=MessageToDict(settled)
                case['bbox_center_distance_from_ego_anchor_m']=math.sqrt(sum(
                    (getattr(settled.bounding_box.location,k)-getattr(initial.transform.location,k))**2 for k in 'xyz'))
                case['ground_bottom_error_m']=(settled.bounding_box.location.z-settled.bounding_box.extent.z)-ground
                case['placement_height_error_m']=(settled.transform.location.z-ground if
                    row.get('height_policy')=='native_spawn_ground_snap' else case['ground_bottom_error_m'])
                case['placement_height_gate_basis']='native_ground_anchor' if row.get('height_policy')=='native_spawn_ground_snap' else 'bbox_bottom'
                if abs(case['placement_height_error_m'])>.2:
                    raise RuntimeError('asset placement height not validated')
                if row['actor_type']=='vehicle':
                    case['vehicle_spec']=MessageToDict(actors.GetVehicleSpec(info,timeout=10))
                    case['resolved_option']=actors.GetOptionName(info,timeout=10).value
                for view,topic in topics.items():
                    if view not in a.capture_views: continue
                    for repeat in range(a.frames_per_view):
                        msg=rospy.wait_for_message(topic,CompressedImage,timeout=5)
                        image=cv2.imdecode(np.frombuffer(msg.data,np.uint8),1)
                        if image is None: raise ValueError('invalid source JPEG')
                        name=f'{index:03d}-{view}-{repeat}.jpg'
                        (a.output/name).write_bytes(bytes(msg.data))
                        case['frames'].append(dict(image=name,view=view,source_stamp_ns=msg.header.stamp.to_nsec(),
                            frame_id=msg.header.frame_id,height=image.shape[0],width=image.shape[1],
                            sha256=hashlib.sha256(bytes(msg.data)).hexdigest()))
            except Exception as exc:
                case['error']=str(exc)
            finally:
                if info in owned:
                    result=actors.DestroyActor(info,timeout=10)
                    report['cleanup'].append(dict(actor_id=info.id.value,response=MessageToDict(result)))
                    if result.status==1: owned.remove(info)
                save()
    finally:
        for info in owned:
            try: report['cleanup'].append(dict(actor_id=info.id.value,response=MessageToDict(actors.DestroyActor(info,timeout=10))))
            except Exception as exc: report['cleanup'].append(dict(actor_id=info.id.value,error=str(exc)))
        after=actors.GetActorState(ego_info,timeout=10)
        report['ego_after']=MessageToDict(after)
        report['ego_translation_delta_m']=math.sqrt(sum((getattr(after.transform.location,k)-getattr(initial.transform.location,k))**2 for k in 'xyz'))
        report['ego_gear_unchanged']=after.vehicle_state.gear_mode==initial.vehicle_state.gear_mode
        report['placement_policy']='actual spawn anchor + estimated ground minus actual bbox bottom; 0.2m gate; visual review still required'
        save();channel.close()


if __name__=='__main__': main()
