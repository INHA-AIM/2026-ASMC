#!/usr/bin/env python3
"""Rotate only owned actors between stopped laps; journal and roll back failed poses."""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'src/data_collection/src'))
sys.path.insert(0, str(ROOT/'tools/grpc_inha_univ/src/proto'))
from data_collection.capture_collection import footprint_clear


def yaw_error(actual, expected):
    return abs((actual-expected+180) % 360-180)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--ledger', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--degrees', type=float, default=60.)
    p.add_argument('--grpc-host', default='host.docker.internal')
    p.add_argument('--route', type=Path, default=ROOT/'R_KR_PG_KATRI/2026_molit_comp_global_path.txt')
    a = p.parse_args()
    if a.output.exists() or not math.isfinite(a.degrees) or abs(a.degrees)>60:
        raise ValueError('new ledger and finite rotation of at most 60 degrees required')
    import grpc
    from google.protobuf.json_format import MessageToDict
    from morai.actor.actor_pb2_grpc import ActorStub
    from morai.actor.actor_set_pb2 import SetTransformParam
    from morai.actor.actor_get_pb2 import GetAllActorsFilter
    from morai.common.object_info_pb2 import ObjectInfo
    from morai.common.enum_pb2 import OBJECT_TYPE_VEHICLE, OBJECT_TYPE_OBSTACLE, OBJECT_TYPE_PEDESTRIAN
    rows=[json.loads(l) for l in a.ledger.read_text().splitlines() if l.strip()]
    active=[r for r in rows if r.get('created') and not r.get('removed')]
    if not active or any(not r.get('placement_approved') or not r.get('geometry_gate') for r in active):
        raise ValueError('only a reviewed, numerically accepted layout can rotate')
    points=[tuple(map(float,l.split()[:3])) for l in a.route.read_text().splitlines() if l.strip()]
    channel=grpc.insecure_channel(a.grpc_host+':7789');actors=ActorStub(channel)
    ego=ObjectInfo(object_type=OBJECT_TYPE_VEHICLE);ego.id.value='Ego'
    if sum(getattr(actors.GetActorState(ego,timeout=3).global_velocity,k)**2 for k in 'xyz')>.01:
        raise RuntimeError('rotation requires stopped Ego')
    states={};infos={};types={'obstacle':OBJECT_TYPE_OBSTACLE,'pedestrian':OBJECT_TYPE_PEDESTRIAN}
    for r in active:
        if not r['id'].startswith('capture_'+r['generation']+'_') or not r['client_key'].startswith('capture_'):
            raise ValueError('unowned actor')
        info=ObjectInfo(object_type=types[r['actor_type']],client_key=r['client_key']);info.id.value=r['id']
        state=actors.GetActorState(info,timeout=3)
        if state.actor_info.id.value!=r['id'] or any(abs(getattr(state.transform.location,k)-r['settled_location'][k])>.2 for k in 'xyz'):
            raise ValueError('stale actor identity or position: '+r['id'])
        states[r['id']]=state;infos[r['id']]=info
    all_states=actors.GetAllActorsState(GetAllActorsFilter(vehicle=True,pedestrian=True,obstacle=True),timeout=5)
    roots={str(r['native_ids'][0]) for r in active}
    occupied=[(s.bounding_box.location.x,s.bounding_box.location.y,
               math.hypot(s.bounding_box.extent.x,s.bounding_box.extent.y))
              for s in all_states.states if s.actor_info.id.value not in roots]
    a.output.parent.mkdir(parents=True,exist_ok=True)
    journal=a.output.with_suffix('.mutations.jsonl')
    if journal.exists():raise ValueError('rotation journal already exists; inspect it before retry')
    changed=[]
    with journal.open('x',buffering=1) as stream:
        try:
            for r in active:
                before=states[r['id']];desired=copy.deepcopy(before.transform)
                desired.rotation.z=(desired.rotation.z+a.degrees)%360
                stream.write(json.dumps(dict(id=r['id'],before=MessageToDict(before,preserving_proto_field_name=True),
                    requested=MessageToDict(desired,preserving_proto_field_name=True)))+'\n')
                changed.append(r['id'])
                response=actors.SetTransform(SetTransformParam(actor_info=infos[r['id']],transform=desired),timeout=3)
                if response.status!=1:raise RuntimeError('actor rotation rejected')
                time.sleep(.08)
                actual=actors.GetActorState(infos[r['id']],timeout=3)
                radius=math.hypot(actual.bounding_box.extent.x,actual.bounding_box.extent.y)
                center=(actual.bounding_box.location.x,actual.bounding_box.location.y)
                if yaw_error(actual.transform.rotation.z,desired.rotation.z)>2 or any(
                    abs(getattr(actual.transform.location,k)-getattr(before.transform.location,k))>.2 for k in 'xyz'):
                    raise RuntimeError('rotation pose mismatch: '+r['id'])
                if not footprint_clear(points,center,radius,occupied):
                    raise RuntimeError('rotated footprint clearance failed: '+r['id'])
                if abs(actual.transform.location.z-r['pivot_offset_m']-r['route_surface_z'])>.2:
                    raise RuntimeError('rotated support height failed: '+r['id'])
                occupied.append((*center,radius))
                r['settled_location']={k:getattr(actual.transform.location,k) for k in 'xyz'}
                r['state_after_identity_settling']=MessageToDict(actual,preserving_proto_field_name=True)
                r['yaw_deg']=actual.transform.rotation.z
                r.setdefault('orientation_history',[]).append(dict(step_degrees=a.degrees,
                    actual_yaw_deg=r['yaw_deg'],parent_ledger_sha256=hashlib.sha256(a.ledger.read_bytes()).hexdigest(),
                    post_capture_rgb_review_pending=True))
                stream.write(json.dumps(dict(id=r['id'],verified=MessageToDict(actual,preserving_proto_field_name=True)))+'\n')
        except BaseException:
            errors=[]
            for key in reversed(changed):
                try:
                    response=actors.SetTransform(SetTransformParam(actor_info=infos[key],transform=states[key].transform),timeout=3)
                    if response.status!=1:raise RuntimeError('rollback rejected')
                    state=actors.GetActorState(infos[key],timeout=3)
                    if yaw_error(state.transform.rotation.z,states[key].transform.rotation.z)>2:raise RuntimeError('rollback yaw mismatch')
                except Exception as exc:errors.append(dict(id=key,error=str(exc)))
            stream.write(json.dumps(dict(rollback_attempted=changed,rollback_errors=errors))+'\n')
            raise
        finally:channel.close()
    a.output.write_text(''.join(json.dumps(r)+'\n' for r in rows))
    print(json.dumps(dict(rotated=len(active),degrees=a.degrees,output=str(a.output))))


if __name__=='__main__':main()
