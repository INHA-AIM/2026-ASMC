#!/usr/bin/env python3
"""Explicit MORAI-only A126 stationary fixture with an original-state ledger.

No actors are spawned/removed. No ROS controls are published. Native transform
and control-mode calls may reset gear: restore position, mode, then gear LAST.
"""
import argparse
import json
import math
from pathlib import Path
import sys
import time

import numpy as np


def main(args):
    sys.path.insert(0, str(args.grpc_root / 'proto'))
    import grpc
    from google.protobuf.json_format import MessageToDict, ParseDict
    from morai.actor.actor_pb2_grpc import ActorStub
    from morai.actor.actor_get_pb2 import GetAllActorsFilter
    from morai.actor.actor_set_pb2 import SetTransformParam, VehicleControlModeParam, VehicleGear
    from morai.actor.actor_control_pb2 import VehicleCtrlCmd
    from morai.common.object_info_pb2 import ObjectInfo
    from morai.common.enum_pb2 import OBJECT_TYPE_VEHICLE
    from morai.common.type_pb2 import Empty, Transform
    from morai.environment.environment_pb2_grpc import EnvironmentStub
    from morai.environment.weather_pb2 import Weather
    from morai.environment.time_pb2 import SimulationTime
    from xmlrpc.client import ServerProxy
    actor = ActorStub(grpc.insecure_channel(args.address))
    environment = EnvironmentStub(grpc.insecure_channel(args.address))
    ego = ObjectInfo(object_type=OBJECT_TYPE_VEHICLE); ego.id.value = 'Ego'
    def call(method, request):
        result = method(request, timeout=3)
        if hasattr(result, 'status') and result.status != 1: raise RuntimeError(str(result))
        return result
    def brake(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            call(actor.ControlVehicle, VehicleCtrlCmd(actor_info=ego, long_cmd_type=1, brake=1.))
            time.sleep(.05)
    def state(): return actor.GetActorState(ego, timeout=3)
    if args.action == 'place':
        writers = [(t, n) for t, n in ServerProxy('http://127.0.0.1:11311').getSystemState('/imu_fixture')[2][0]
                   if t.startswith('/ctrl_cmd') and n]
        if writers: raise RuntimeError('public vehicle control producers exist: ' + str(writers))
        before = state()
        if sum(getattr(before.global_velocity, k)**2 for k in 'xyz') > .0025: raise RuntimeError('Ego must be stationary')
        all_actors = actor.GetAllActorsState(GetAllActorsFilter(vehicle=True, pedestrian=True, obstacle=True), timeout=3)
        if len(all_actors.states) != 1: raise RuntimeError('fixture requires empty scene; no existing actors are removed')
        original = dict(transform=MessageToDict(before.transform), gear=before.vehicle_state.gear_mode,
            mode=actor.GetVehicleControlMode(ego, timeout=3).mode,
            weather=MessageToDict(environment.GetWeather(Empty(), timeout=3)),
            time=MessageToDict(environment.GetTime(Empty(), timeout=3)))
        args.ledger.parent.mkdir(parents=True, exist_ok=True)
        if args.ledger.exists(): raise RuntimeError('original-state ledger already exists')
        args.ledger.write_text(json.dumps(original, indent=2) + '\n')
        points = np.array(next(r['points'] for r in json.loads(args.link_set.read_text()) if r['idx'] == 'A2256W000126'))
        distances = np.r_[0., np.cumsum(np.linalg.norm(np.diff(points[:, :2], axis=0), axis=1))]
        station = 60.; i = int(np.searchsorted(distances, station) - 1)
        fraction = (station - distances[i]) / (distances[i + 1] - distances[i])
        xyz = points[i] + fraction * (points[i + 1] - points[i])
        requested = Transform(); requested.location.x, requested.location.y, requested.location.z = (*xyz[:2], xyz[2] + .36)
        delta = points[i + 1] - points[i]; requested.rotation.z = math.degrees(math.atan2(delta[1], delta[0]))
        call(actor.SetVehicleControlMode, VehicleControlModeParam(actor_info=ego, mode=3))
        brake(.5)
        call(actor.SetTransform, SetTransformParam(actor_info=ego, transform=requested)); brake(3.)
        call(actor.SetVehicleControlMode, VehicleControlModeParam(actor_info=ego, mode=original['mode']))
        call(actor.SetVehicleGear, VehicleGear(actor_info=ego, gear=1))
        call(environment.SetTime, SimulationTime(hour=13, data_only=False, instantly=True))
        time.sleep(1.)
        result = dict(requested=MessageToDict(requested), actual=MessageToDict(state()), original=original)
        args.ledger.with_name('fixture-position.json').write_text(json.dumps(result, indent=2) + '\n')
    else:
        original = json.loads(args.ledger.read_text())
        call(actor.SetVehicleControlMode, VehicleControlModeParam(actor_info=ego, mode=3)); brake(1.)
        requested = ParseDict(original['transform'], Transform())
        call(actor.SetTransform, SetTransformParam(actor_info=ego, transform=requested)); brake(1.)
        call(actor.SetVehicleControlMode, VehicleControlModeParam(actor_info=ego, mode=original['mode']))
        call(actor.SetVehicleGear, VehicleGear(actor_info=ego, gear=original['gear']))
        call(environment.SetWeather, ParseDict(original['weather'], Weather()))
        call(environment.SetTime, ParseDict(dict(original['time'], instantly=True), SimulationTime()))
        time.sleep(2.)
        after = state(); distance = math.sqrt(sum((getattr(after.transform.location, k) - getattr(requested.location, k))**2 for k in 'xyz'))
        yaw = abs((after.transform.rotation.z - requested.rotation.z + 180) % 360 - 180)
        result = dict(actual=MessageToDict(after), position_error_m=distance, yaw_error_deg=yaw,
            restored=distance < .1 and after.vehicle_state.gear_mode == original['gear'] and
            actor.GetVehicleControlMode(ego, timeout=3).mode == original['mode'] and
            sum(getattr(after.global_velocity, k)**2 for k in 'xyz') < .0025)
        args.ledger.with_name('fixture-restored.json').write_text(json.dumps(result, indent=2) + '\n')
        if not result['restored']: raise RuntimeError('fixture restoration incomplete: ' + str(result))
    print(json.dumps(result, indent=2), flush=True)


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('action', choices=('place', 'restore'))
    p.add_argument('--grpc-root', type=Path, required=True)
    p.add_argument('--link-set', type=Path)
    p.add_argument('--ledger', type=Path, required=True)
    p.add_argument('--address', default='host.docker.internal:7789')
    main(p.parse_args())
