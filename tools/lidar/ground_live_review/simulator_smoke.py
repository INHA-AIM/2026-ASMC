#!/usr/bin/env python3
"""Opt-in bounded MORAI-only motion check. Restores Ego; not a driving benchmark."""
import argparse
import json
import math
from pathlib import Path
import sys
import time


def main():
    p=argparse.ArgumentParser()
    p.add_argument("output",type=Path)
    p.add_argument("--confirm-simulator-move",action="store_true")
    a=p.parse_args()
    if not a.confirm_simulator_move:
        p.error("explicit simulator movement confirmation required")
    if a.output.exists():
        raise FileExistsError(a.output)
    repo=Path(__file__).resolve().parents[3]
    sys.path.insert(0,str(repo/"tools/grpc_inha_univ/src/proto"))
    import grpc
    from google.protobuf.json_format import MessageToDict
    from morai.actor.actor_pb2_grpc import ActorStub
    from morai.actor.actor_get_pb2 import GetAllActorsFilter
    from morai.actor.actor_control_pb2 import VehicleCtrlCmd
    from morai.actor.actor_set_pb2 import VehicleControlModeParam,VehicleGear,SetVelocityParam,SetTransformParam
    from morai.actor.actor_enum_pb2 import VEHICLE_CONTROL_AUTO_MODE,VEHICLE_CONTROL_KEYBOARD,GEAR_MODE_D,GEAR_MODE_P
    from morai.common.object_info_pb2 import ObjectInfo
    from morai.common.enum_pb2 import OBJECT_TYPE_VEHICLE,STATUS_CODE_SUCCESS
    api=ActorStub(grpc.insecure_channel("host.docker.internal:7789"))
    ego=ObjectInfo();ego.id.value="Ego";ego.object_type=OBJECT_TYPE_VEHICLE
    original=api.GetActorState(ego,timeout=5)
    mode=api.GetVehicleControlMode(ego,timeout=5).mode
    norm=lambda v:math.sqrt(sum(getattr(v,k)**2 for k in "xyz"))
    if norm(original.velocity)>.1 or not original.HasField("transform"):
        raise RuntimeError("requires stationary Ego and valid initial transform")
    def distance(s):
        return math.sqrt(sum((getattr(s.transform.location,k)-getattr(original.transform.location,k))**2 for k in "xy"))
    others=api.GetAllActorsState(GetAllActorsFilter(vehicle=True,pedestrian=True,obstacle=True),timeout=5)
    # Conservative forward corridor plus a near-Ego exclusion. Static fixtures
    # behind Ego need not prevent a short forward-only simulator smoke test.
    heading=math.radians(original.transform.rotation.z)
    for field,value in others.ListFields():
        if field.label==field.LABEL_REPEATED:
            for actor in value:
                if not hasattr(actor,"actor_info") or actor.actor_info.id.value=="Ego":
                    continue
                dx=actor.transform.location.x-original.transform.location.x
                dy=actor.transform.location.y-original.transform.location.y
                forward=dx*math.cos(heading)+dy*math.sin(heading)
                if distance(actor)<4 or (distance(actor)<15 and (forward>=0 or norm(actor.velocity)>.1)):
                    raise RuntimeError("actor in motion-check corridor: stationary/replay only")
    def call(method,request):
        request.actor_info.CopyFrom(ego)
        result=method(request,timeout=3)
        if result.status!=STATUS_CODE_SUCCESS:
            raise RuntimeError(str(result))
    report={"original":MessageToDict(original,preserving_proto_field_name=True),"original_mode":mode,
            "limit_seconds":14,"limit_displacement_m":8,"soft_speed_cap_mps":1.2,
            "samples":[],"restored":False,"kind":"bounded_grpc_smoke_not_manual_full_route"}
    a.output.parent.mkdir(parents=True,exist_ok=True)
    def save():
        a.output.write_text(json.dumps(report,indent=2)+"\n")
    save()
    try:
        call(api.SetVehicleControlMode,VehicleControlModeParam(mode=VEHICLE_CONTROL_AUTO_MODE))
        call(api.SetVehicleGear,VehicleGear(gear=GEAR_MODE_D))
        start=time.monotonic()
        while time.monotonic()-start<14:
            state=api.GetActorState(ego,timeout=3)
            speed=norm(state.velocity)
            report["samples"].append({"elapsed":time.monotonic()-start,"displacement_m":distance(state),
                "speed_mps":speed,"state":MessageToDict(state,preserving_proto_field_name=True)})
            if distance(state)>=8 or speed>2:
                break
            # Tiny pedal input; observed speed/displacement bounds do not assume velocity-command units.
            call(api.ControlVehicle,VehicleCtrlCmd(long_cmd_type=1,throttle=.04 if speed<1.2 else 0,
                 brake=.2 if speed>=1.2 else 0,steer=0))
            time.sleep(.1)
        report["max_displacement_m"]=max(x["displacement_m"] for x in report["samples"])
        report["max_speed_mps"]=max(x["speed_mps"] for x in report["samples"])
    finally:
        errors=[]
        actions=[(api.ControlVehicle,VehicleCtrlCmd(long_cmd_type=1,throttle=0,brake=1,steer=0)),
                 (api.SetVehicleControlMode,VehicleControlModeParam(mode=VEHICLE_CONTROL_KEYBOARD)),
                 (api.SetVelocity,SetVelocityParam(velocity=0)),
                 (api.SetVehicleGear,VehicleGear(gear=GEAR_MODE_P))]
        restore=SetTransformParam();restore.transform.CopyFrom(original.transform)
        actions.extend([(api.SetTransform,restore),(api.SetVehicleGear,VehicleGear(gear=original.vehicle_state.gear_mode)),
                        (api.SetVehicleControlMode,VehicleControlModeParam(mode=mode))])
        for method,request in actions:
            try:call(method,request)
            except Exception as e:errors.append(str(e))
        report["restore_errors"]=errors;report["restored"]=not errors
        save()
        if errors:
            raise RuntimeError("Ego restoration incomplete: "+str(errors))
    print(json.dumps({k:v for k,v in report.items() if k not in ("samples","original")}))


if __name__=="__main__":
    main()
