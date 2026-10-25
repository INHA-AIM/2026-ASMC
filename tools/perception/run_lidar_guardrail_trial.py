#!/usr/bin/env python3
"""Bounded MORAI A304 drive using the existing Capture PP/P controller.

Development-only gRPC Ego pose controls the rehearsal, never the LiDAR map pose.
No Capture, weather change, actor creation/deletion, public ROS control or stack
restart. The caller supplies a registered run ID and a fresh output directory.
"""
import argparse
from collections import deque
import hashlib
import json
import math
from pathlib import Path
import signal
import sys
import threading
import time


ROOT = Path(__file__).resolve().parents[2]


def main(args):
    if not args.execute:
        raise RuntimeError('MORAI driving requires --execute')
    if not 0 < args.seconds <= 20 or any(not 0 < s <= 20 for s in args.speeds):
        raise ValueError('at most20s/pass and20km/h')
    sys.path[:0] = [str(ROOT/'tools/grpc_inha_univ/src/proto'),
                   str(ROOT/'src/data_collection/src')]
    import grpc
    import rosbag
    import rosgraph
    import rospy
    from google.protobuf.json_format import MessageToDict
    from morai.actor.actor_pb2_grpc import ActorStub
    from morai.actor.actor_control_pb2 import VehicleCtrlCmd
    from morai.actor.actor_set_pb2 import VehicleControlModeParam, VehicleGear, SetTransformParam
    from morai.actor.actor_enum_pb2 import VEHICLE_CONTROL_AUTO_MODE, GEAR_MODE_D
    from morai.common.object_info_pb2 import ObjectInfo
    from morai.common.enum_pb2 import OBJECT_TYPE_VEHICLE
    from sensor_msgs.msg import PointCloud2, Imu
    from morai_msgs.msg import GPSMessage, EgoVehicleStatus, CollisionData
    from asmc_msgs.msg import CameraObjectObservation, InferenceStatus
    from std_msgs.msg import String
    from data_collection.capture_collection import LegacyPurePursuit, point_route_distance
    from drive_coco7_preview import restore_ego

    rospy.init_node('lidar_guardrail_trial', anonymous=True, disable_signals=True)
    master = rosgraph.Master(rospy.get_name())

    def check_control():
        writers = [(t, ns) for t, ns in master.getSystemState()[0]
                   if ns and ('ctrl_cmd' in t or 'actuator_cmd' in t)]
        if writers:
            raise RuntimeError('existing vehicle control publisher: '+str(writers))

    check_control()
    actors = ActorStub(grpc.insecure_channel(args.address))
    ego = ObjectInfo(object_type=OBJECT_TYPE_VEHICLE)
    ego.id.value = 'Ego'
    original = actors.GetActorState(ego, timeout=2)
    original_mode = actors.GetVehicleControlMode(ego, timeout=2).mode

    def speed(state):
        return math.sqrt(sum(getattr(state.global_velocity, k)**2 for k in 'xyz'))

    if speed(original) > .05:
        raise RuntimeError('Ego must be stopped before the trial')
    points = next(x['points'] for x in json.loads(args.link_set.read_text())
                  if x['idx'] == args.link)
    position = original.transform.location
    nearest = min(range(len(points)), key=lambda i: math.hypot(points[i][0]-position.x, points[i][1]-position.y))
    if math.hypot(points[nearest][0]-position.x, points[nearest][1]-position.y) > 3:
        raise RuntimeError('stationary Ego must already be near the selected guardrail link')
    remaining = sum(math.dist(a[:2], b[:2]) for a, b in zip(points[nearest:], points[nearest+1:]))
    if remaining < 30:
        raise RuntimeError('insufficient forward test route')
    target = type(original.transform)()
    target.location.x, target.location.y, target.location.z = (*points[nearest][:2], points[nearest][2]+.36)
    delta = [points[min(nearest+3, len(points)-1)][i]-points[max(0, nearest-3)][i] for i in (0, 1)]
    target.rotation.z = math.degrees(math.atan2(delta[1], delta[0]))
    args.output.mkdir(parents=True, exist_ok=False)
    report = dict(run_id=args.run_id, link=args.link, controller='existing LegacyPurePursuit + P speed',
                  input_role='gRPC Ego is evaluation actuator only; production LiDAR uses original GPS/IMU',
                  source_revision=args.revision, link_sha256=hashlib.sha256(args.link_set.read_bytes()).hexdigest(),
                  original_ego=MessageToDict(original), original_mode=original_mode,
                  start_transform=MessageToDict(target), passes=[], rpc=[], restored=False, failures=[])
    bag = rosbag.Bag(str(args.output/'signals.bag'), 'w', compression='lz4')
    lock = threading.RLock()
    abort = threading.Event()
    errors = deque(maxlen=8)
    current = [None]
    byte_count = [0]
    latest_lidar = [0.]
    latest_trace = [0.]
    stream = (args.output/'drive.jsonl').open('x', buffering=1)
    subs = []
    changed = False

    def save():
        (args.output/'report.json').write_text(json.dumps(report, indent=2)+'\n')

    def record(topic, msg):
        with lock:
            if current[0] is None:
                return
            if byte_count[0] > 384*2**20:
                errors.append('384MiB record budget exceeded'); abort.set(); return
            bag.write(topic, msg, rospy.Time.now())
            if isinstance(msg, PointCloud2):
                byte_count[0] += len(msg.data)
                latest_lidar[0] = time.monotonic()
            if topic == '/migration/lidar/trace':
                latest_trace[0] = time.monotonic()
            if isinstance(msg, CollisionData) and (msg.collision_object or msg.collision_objecta):
                errors.append('collision'); abort.set()

    def mutate(method, request):
        response = method(request, timeout=.5)
        report['rpc'].append(dict(method=getattr(method, '_method', b'').decode(),
                                  response=MessageToDict(response)))
        if response.status != 1:
            raise RuntimeError('gRPC mutation rejected')

    def brake(seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            mutate(actors.ControlVehicle, VehicleCtrlCmd(actor_info=ego, long_cmd_type=1, brake=1.))
            time.sleep(.05)

    def interrupt(*_):
        errors.append('interrupted'); abort.set()

    signal.signal(signal.SIGTERM, interrupt)
    signal.signal(signal.SIGINT, interrupt)
    try:
        for topic, cls in [('/velodyne_points', PointCloud2), ('/imu', Imu), ('/gps', GPSMessage),
                           ('/Ego_topic', EgoVehicleStatus), ('/CollisionData', CollisionData),
                           ('/perception/camera/objects/observations', CameraObjectObservation),
                           ('/perception/camera/objects/status', InferenceStatus),
                           ('/migration/lidar/trace', String)]:
            subs.append(rospy.Subscriber(topic, cls, lambda m, t=topic: record(t, m), queue_size=2))
        save()
        for index, target_kph in enumerate(args.speeds):
            if abort.is_set():
                raise RuntimeError(str(list(errors)))
            check_control()
            changed = True
            mutate(actors.SetVehicleControlMode, VehicleControlModeParam(actor_info=ego, mode=VEHICLE_CONTROL_AUTO_MODE))
            brake(.5)
            mutate(actors.SetTransform, SetTransformParam(actor_info=ego, transform=target))
            brake(2.)
            state = actors.GetActorState(ego, timeout=.5)
            if speed(state) > .05 or math.hypot(state.transform.location.x-target.location.x,
                                                state.transform.location.y-target.location.y) > .3:
                raise RuntimeError('unsettled starting pose')
            mutate(actors.SetVehicleControlMode, VehicleControlModeParam(actor_info=ego, mode=VEHICLE_CONTROL_AUTO_MODE))
            mutate(actors.SetVehicleGear, VehicleGear(actor_info=ego, gear=GEAR_MODE_D))
            controller = LegacyPurePursuit(ROOT, points, target_kph)
            begin = time.monotonic(); current[0] = index
            latest_lidar[0] = latest_trace[0] = begin
            samples = 0; max_cte = max_speed = 0.; start_s = end_s = None
            while time.monotonic()-begin < args.seconds:
                tick = time.monotonic()
                if abort.is_set() or rospy.is_shutdown():
                    raise RuntimeError(str(list(errors)))
                if tick-latest_lidar[0] > .5 or tick-latest_trace[0] > 1.:
                    raise RuntimeError('LiDAR/trace input stopped')
                state = actors.GetActorState(ego, timeout=.3)
                loc = state.transform.location; actual_speed = speed(state)
                cte = point_route_distance(points, loc.x, loc.y)
                if cte > .8 or actual_speed*3.6 > target_kph+3 or state.vehicle_state.collision_objects:
                    raise RuntimeError('drive guard: cte/speed/collision')
                if samples % 10 == 0:
                    check_control()
                command = controller.command(loc.x, loc.y, math.radians(state.transform.rotation.z), actual_speed)
                start_s = command['current_s'] if start_s is None else start_s
                end_s = command['current_s']
                if command['remaining_s'] < 10:
                    break
                mutate(actors.ControlVehicle, VehicleCtrlCmd(actor_info=ego, long_cmd_type=1,
                    throttle=command['accel'], brake=command['brake'],
                    steer=max(-1., min(1., math.degrees(command['steer'])/40.))))
                stream.write(json.dumps(dict(pass_index=index, target_kph=target_kph, ros_ns=rospy.Time.now().to_nsec(),
                    elapsed_s=tick-begin, x=loc.x, y=loc.y, speed_kph=actual_speed*3.6, cte_m=cte,
                    station_m=end_s, steer=command['steer']))+'\n')
                samples += 1; max_cte = max(max_cte, cte); max_speed = max(max_speed, actual_speed*3.6)
                time.sleep(max(0., .05-(time.monotonic()-tick)))
            brake(2.)
            with lock:
                current[0] = None
            result = dict(index=index, target_kph=target_kph, samples=samples,
                          forward_m=end_s-start_s if start_s is not None else 0,
                          max_cte_m=max_cte, max_speed_kph=max_speed)
            report['passes'].append(result); save(); print(json.dumps(result), flush=True)
            if result['forward_m'] < 5:
                raise RuntimeError('no verified forward progress')
    except BaseException as error:
        report['failures'].append(str(error))
    finally:
        with lock:
            current[0] = None
        if changed:
            try:
                brake(2.)
                report['restored'] = restore_ego(actors, ego, original, original_mode, mutate, report)
            except BaseException as error:
                report['failures'].append('restore: '+str(error))
        for sub in subs:
            sub.unregister()
        with lock:
            bag.close(); stream.close()
        report['point_payload_bytes'] = byte_count[0]; save()
    print(json.dumps(dict(restored=report['restored'], failures=report['failures'])), flush=True)
    return int(bool(report['failures']) or not report['restored'])


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--execute', action='store_true')
    p.add_argument('--run-id', required=True)
    p.add_argument('--revision', required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--address', default='host.docker.internal:7789')
    p.add_argument('--link-set', type=Path, default=Path('/data/R_KR_PG_K-city_2025/link_set.json'))
    p.add_argument('--link', default='A2256W000304')
    p.add_argument('--speeds', type=float, nargs='+', default=[10., 20., 20.])
    p.add_argument('--seconds', type=float, default=15.)
    raise SystemExit(main(p.parse_args()))
