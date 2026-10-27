#!/usr/bin/env python3
"""Stationary MORAI challenge capture; only temporary owned actors are modified.

No Ego teleport, control publisher, global actor deletion, scenario reload or sensor
configuration change. Existing scene and Ego pose/gear/mode are recorded and checked.
Lowered-vehicle cases are synthetic geometry stress fixtures, not road-driving evidence.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import time
import uuid

ROOT=Path(__file__).resolve().parents[2]
GRPC=ROOT/'tools/grpc_inha_univ/src'
sys.path[:0]=[str(GRPC),str(GRPC/'api'),str(GRPC/'proto')]
import cv2
import numpy as np
import rosgraph
import rospy
from google.protobuf.json_format import MessageToDict
from api.morai_sim_client import MoraiSimClient
from api.vehicle import Vehicle
from morai.common.object_info_pb2 import ObjectInfo
from morai.common.enum_pb2 import OBJECT_TYPE_VEHICLE,OBJECT_TYPE_PEDESTRIAN
from morai.common.type_pb2 import Transform
from morai.actor.actor_spawn_pb2 import ActorSpawnInfo,VehicleSpawnParam,PedestrianSpawnParam
from morai.actor.actor_set_pb2 import EnableActor,SetTransformParam
from sensor_msgs.msg import CompressedImage


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--grpc-host',default='host.docker.internal')
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    rospy.init_node('hood_challenge_capture',anonymous=True)
    if any(topic=='/ctrl_cmd' and nodes for topic,nodes in rosgraph.Master('/hood_capture').getSystemState()[0]):
        raise RuntimeError('stationary capture requires no /ctrl_cmd publisher')
    client=MoraiSimClient('hood_challenge_'+uuid.uuid4().hex[:10]);client.connect(args.grpc_host,7789)
    adapter=client._sim_adapter;stub=adapter._actor_stub;ego=Vehicle(adapter,client._client_key,'Ego');info=ego.get_object_info()
    initial=stub.GetActorState(info,timeout=10);mode=stub.GetVehicleControlMode(info,timeout=10)
    if sum(getattr(initial.global_velocity,a)**2 for a in 'xyz')>.01:raise RuntimeError('Ego is moving')
    report=dict(profile='stationary_owned_actor_geometry_challenges',ego_before=MessageToDict(initial),mode_before=MessageToDict(mode),
                official_scenario_sha256=hashlib.sha256(Path('/data/R_KR_PG_K-city_2025/2026_molit_comp_sample_scene.json').read_bytes()).hexdigest(),
                sensor_config_sha256=hashlib.sha256((ROOT/'tools/bridge/templates/sensor_v3.json').read_bytes()).hexdigest(),rpc=[],frames=[],cleanup=[])
    owned=[]
    yaw=math.radians(initial.transform.rotation.z);forward=np.array([math.cos(yaw),math.sin(yaw)]);left=np.array([-forward[1],forward[0]])
    def call(name,request):
        result=getattr(stub,name)(request,timeout=10)
        report['rpc'].append(dict(method=name,request=MessageToDict(request),response=MessageToDict(result)))
        if result.status!=1:raise RuntimeError(name+' rejected '+str(result))
        return result
    def transform(distance,lateral,zoffset,relative_yaw=0):
        t=Transform();t.CopyFrom(initial.transform)
        xy=np.array([initial.transform.location.x,initial.transform.location.y])+distance*forward+lateral*left
        t.location.x,t.location.y=map(float,xy);t.location.z+=zoffset;t.rotation.z+=relative_yaw
        return t
    try:
        for kind,model in [('vehicle','2016_Hyundai_Sonata'),('person','Man1')]:
            actor_info=ObjectInfo(object_type=OBJECT_TYPE_VEHICLE if kind=='vehicle' else OBJECT_TYPE_PEDESTRIAN,client_key=client._client_key)
            actor_info.id.value='hood_'+kind+'_'+uuid.uuid4().hex[:8]
            spawn=ActorSpawnInfo(actor_info=actor_info,model_name=model,label='hood challenge '+kind,transform=transform(12,8,0))
            if kind=='vehicle':call('SpawnVehicle',VehicleSpawnParam(spawn_info=spawn,pause=True))
            else:call('SpawnPedestrian',PedestrianSpawnParam(spawn_info=spawn,velocity=0,start_action=False))
            owned.append(actor_info)
            if kind=='vehicle':call('SetAI',EnableActor(actor_info=actor_info,enable=False))
            call('SetPause',EnableActor(actor_info=actor_info,enable=True))
            for distance,lateral,zoffset,relative_yaw in ([(8,0,0,180),(5,0,0,90),(4,-3,0,90),(4,3,0,90),
                (3,-3,0,90),(3,3,0,90),(6,0,-.5,90),(5,0,-1.,90),(5,0,-1.5,90)] if kind=='vehicle' else
                [(5,0,0,0),(3,-2,0,0),(3,2,0,0),(4,0,-.5,0)]):
                call('SetTransform',SetTransformParam(actor_info=actor_info,transform=transform(distance,lateral,zoffset,relative_yaw)))
                call('SetPause',EnableActor(actor_info=actor_info,enable=True))
                time.sleep(.4)
                state=stub.GetActorState(actor_info,timeout=10)
                for repeat in range(3):
                    message=rospy.wait_for_message('/image_jpeg/compressed',CompressedImage,timeout=5)
                    image=cv2.imdecode(np.frombuffer(message.data,np.uint8),1)
                    if image is None:raise ValueError('camera JPEG cannot decode')
                    name='%03d.png'%len(report['frames']);cv2.imwrite(str(args.output/name),image)
                    report['frames'].append(dict(image=name,source_stamp_ns=message.header.stamp.to_nsec(),frame_id=message.header.frame_id,
                        dimensions=list(image.shape[:2]),kind=kind,model=model,actor=MessageToDict(state),
                        pose=dict(distance=distance,lateral=lateral,zoffset=zoffset,yaw=relative_yaw),repeat=repeat,
                        synthetic_geometry_stress=zoffset!=0,image_sha256=hashlib.sha256((args.output/name).read_bytes()).hexdigest()))
                    time.sleep(.1)
            call('SetTransform',SetTransformParam(actor_info=actor_info,transform=transform(30,20,0)))
    finally:
        for actor_info in reversed(owned):
            try:report['cleanup'].append(dict(actor=actor_info.id.value,result=MessageToDict(stub.DestroyActor(actor_info,timeout=10))))
            except Exception as exc:report['cleanup'].append(dict(actor=actor_info.id.value,error=str(exc)))
        final=stub.GetActorState(info,timeout=10);finalmode=stub.GetVehicleControlMode(info,timeout=10)
        report['ego_after']=MessageToDict(final);report['mode_after']=MessageToDict(finalmode)
        report['ego_pose_error_m']=math.sqrt(sum((getattr(final.transform.location,a)-getattr(initial.transform.location,a))**2 for a in 'xyz'))
        report['ego_rotation_error_deg']=max(abs((getattr(final.transform.rotation,a)-getattr(initial.transform.rotation,a)+180)%360-180) for a in 'xyz')
        report['ego_gear_mode_unchanged']=final.vehicle_state.gear_mode==initial.vehicle_state.gear_mode and finalmode.mode==mode.mode
        report['ego_pose_gear_mode_preserved']=report['ego_pose_error_m']<.1 and report['ego_rotation_error_deg']<.1 and report['ego_gear_mode_unchanged']
        report['owned_actor_cleanup_succeeded']=all(c.get('result',{}).get('status')=='STATUS_CODE_SUCCESS' for c in report['cleanup'])
        report['reference_hashes_are_not_live_scene_or_sensor_attestation']=True
        (args.output/'capture.json').write_text(json.dumps(report,indent=2)+'\n');client.disconnect()
    passed=report['ego_pose_gear_mode_preserved'] and report['owned_actor_cleanup_succeeded']
    print(json.dumps(dict(frames=len(report['frames']),ego_preserved=report['ego_pose_gear_mode_preserved'],owned_actor_cleanup=report['owned_actor_cleanup_succeeded'])))
    return 0 if passed else 1


if __name__=='__main__':raise SystemExit(main())
