#!/usr/bin/env python3
"""Create/clean only registered evaluation actors, preserving the existing scene."""
import argparse
import json
import math
from pathlib import Path
import time
import uuid

FIXTURE = [('vehicle','2014_Kia_K7',20.,0.),
    ('vehicle','2013_Hyundai_Xcient(P520)',35.,3.),
    ('vehicle','2016_Hyundai_Universe',45.,0.),
    ('pedestrian','Man1',12.,2.),('pedestrian','Dog1',10.,-2.),
    ('obstacle','Bicycle_city',3.,8.),('obstacle','SportBike',3.,-8.),
    ('vehicle','2023_Kia_EV6',0.,10.),('vehicle','2016_Hyundai_Ioniq',0.,-10.)]


def load_fixture(path):
    if path is None:
        return FIXTURE
    rows=json.loads(path.read_text())
    if len(rows)!=len(FIXTURE):raise ValueError('fixture must preserve all nine actor types/models')
    if [(r[0],r[1]) for r in rows]!=[(r[0],r[1]) for r in FIXTURE]:
        raise ValueError('fixture must preserve actor types/models and order')
    if any(len(r)!=4 or not all(math.isfinite(float(x)) and abs(float(x))<=100 for x in r[2:]) for r in rows):
        raise ValueError('invalid fixture offset')
    return rows


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('operation',choices=['create','cleanup'])
    p.add_argument('--registry',type=Path,required=True)
    p.add_argument('--grpc-host',default='host.docker.internal')
    p.add_argument('--fixture',type=Path,help='Explicit offsets for the same nine models; ground gate unchanged')
    a=p.parse_args()
    # Fixture validation and --help do not require the live simulator SDK.
    from capture_coco7_actors import (grpc, SimulatorStub, ActorStub, CategoryObstacles, ObjectInfo,
        OBJECT_TYPE_VEHICLE, OBJECT_TYPE_PEDESTRIAN, OBJECT_TYPE_OBSTACLE, Transform, Vector3,
        ActorSpawnInfo, VehicleSpawnParam, PedestrianSpawnParam, ObstacleSpawnParam,
        EnableActor, SetTransformParam, MessageToDict)
    TYPES = dict(vehicle=OBJECT_TYPE_VEHICLE,pedestrian=OBJECT_TYPE_PEDESTRIAN,obstacle=OBJECT_TYPE_OBSTACLE)
    channel=grpc.insecure_channel(a.grpc_host+':7789')
    actors=ActorStub(channel)
    if a.operation=='cleanup':
        report=json.loads(a.registry.read_text());report['cleanup']=[]
        for row in report['actors']:
            if not row.get('created'):continue
            if not row['id'].startswith('coco7_fixture_'):raise RuntimeError('refusing non-owned actor ID')
            info=ObjectInfo(object_type=TYPES[row['type']],client_key='coco7_fixture');info.id.value=row['id']
            reply=actors.DestroyActor(info,timeout=10)
            report['cleanup'].append(dict(id=row['id'],response=MessageToDict(reply)))
            a.registry.write_text(json.dumps(report,indent=2)+'\n')
        return
    if a.registry.exists():raise RuntimeError('registry already exists; refusing overwrite')
    ego_info=ObjectInfo(object_type=OBJECT_TYPE_VEHICLE);ego_info.id.value='Ego'
    ego=actors.GetActorState(ego_info,timeout=10)
    if sum(getattr(ego.global_velocity,k)**2 for k in 'xyz')>.01:raise RuntimeError('stationary Ego required')
    yaw=math.radians(ego.transform.rotation.z)
    ground=ego.bounding_box.location.z-ego.bounding_box.extent.z
    report=dict(ego_before=MessageToDict(ego),actors=[],pixel_gt=False,height_gate_m=.2,
                scope='stationary compute load fixture; not automatic driving or pixel ground truth')
    fixture=load_fixture(a.fixture)
    report['fixture']=fixture
    a.registry.parent.mkdir(parents=True,exist_ok=True)
    def save():a.registry.write_text(json.dumps(report,indent=2)+'\n')
    for kind,model,forward,lateral in fixture:
        info=ObjectInfo(object_type=TYPES[kind],client_key='coco7_fixture');info.id.value='coco7_fixture_'+uuid.uuid4().hex[:12]
        row=dict(id=info.id.value,type=kind,model=model,forward_m=forward,lateral_m=lateral,created=False)
        report['actors'].append(row);save()
        pose=Transform();pose.CopyFrom(ego.transform)
        pose.location.x+=forward*math.cos(yaw)-lateral*math.sin(yaw)
        pose.location.y+=forward*math.sin(yaw)+lateral*math.cos(yaw)
        spawn=ActorSpawnInfo(actor_info=info,model_name=model,label='coco7 fixed load fixture',transform=pose)
        request=(VehicleSpawnParam(spawn_info=spawn,pause=True) if kind=='vehicle' else
            PedestrianSpawnParam(spawn_info=spawn,velocity=0,start_action=False) if kind=='pedestrian' else
            ObstacleSpawnParam(spawn_info=spawn,scale=Vector3(x=1,y=1,z=1)))
        try:
            reply=getattr(actors,'Spawn'+kind.title())(request,timeout=10);row['spawn']=MessageToDict(reply)
            if reply.status!=1:raise RuntimeError(str(reply))
            row['created']=True;save()
            if kind=='vehicle':actors.SetAI(EnableActor(actor_info=info,enable=False),timeout=10)
            time.sleep(.4);state=actors.GetActorState(info,timeout=10)
            if kind!='pedestrian':
                pose.location.z=state.transform.location.z+ground-(state.bounding_box.location.z-state.bounding_box.extent.z)
                reply=actors.SetTransform(SetTransformParam(actor_info=info,transform=pose),timeout=10)
                if reply.status!=1:raise RuntimeError(str(reply))
            if kind!='obstacle':actors.SetPause(EnableActor(actor_info=info,enable=True),timeout=10)
            time.sleep(.3);state=actors.GetActorState(info,timeout=10);row['state']=MessageToDict(state)
            row['height_error_m']=state.bounding_box.location.z-state.bounding_box.extent.z-ground
            if abs(row['height_error_m'])>.2:raise RuntimeError('ground placement not validated')
        except Exception as exc:
            row['error']=str(exc)
            if row['created']:
                row['cleanup_on_failure']=MessageToDict(actors.DestroyActor(info,timeout=10));row['created']=False
        save()
    report['ego_after']=MessageToDict(actors.GetActorState(ego_info,timeout=10));save()


if __name__=='__main__':main()
