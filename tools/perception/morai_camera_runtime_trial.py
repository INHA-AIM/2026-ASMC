#!/usr/bin/env python3
"""Explicit MORAI camera rehearsal; ground truth is for fixtures/watchdogs only.

Never starts SimulationWorld (which deletes actors). Original GPS, scenarios,
adapter code and defaults are not edited. Driving forwards the existing preview
through a test gate, with a single /ctrl_cmd publisher and a 5 km/h ceiling.
"""
import argparse
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[2]
GRPC_ROOT = ROOT / 'tools/grpc_inha_univ/src'
sys.path[:0] = [str(GRPC_ROOT), str(GRPC_ROOT/'api'), str(GRPC_ROOT/'proto')]

import numpy as np
import rospy
from google.protobuf.json_format import MessageToDict
from api.morai_sim_client import MoraiSimClient
from api.vehicle import Vehicle
from morai.common.type_pb2 import StringValue, Empty, Transform
from morai.actor.actor_set_pb2 import VehicleControlModeParam, VehicleGear, SetTransformParam, SetVelocityParam
from morai.actor.actor_control_pb2 import VehicleCtrlCmd
from morai.actor.actor_enum_pb2 import VEHICLE_CONTROL_AUTO_MODE, GEAR_MODE_D
from morai_msgs.msg import CtrlCmd, GPSMessage, CollisionData
from asmc_camera.msg import InferenceStatus
from asmc_camera.srv import SetInferenceMode
from asmc_hd_map.msg import HdMapContext
from asmc_msgs.msg import LaneBoundaryFamily
from lane_boundary_adapter.msg import CameraBBoxLaneResult

SCENARIO_SHA = 'c9ab17cef9b07dd3a7d4a2f56c6cc39f89f07cadccce8edce3d579e891237c85'
FAMILY_SHA = '1657f282fe70c13bdb285bbf2536405768a07b2b6462590e71c75787ef6d0cf9'
SHADOW_START, SHADOW_END = 1877.744798966, 1996.730791531


class Trial:
    def __init__(self, args):
        self.args = args
        self.output = args.output
        self.output.mkdir(parents=True, exist_ok=True)
        self.report = dict(profile=('canonical_yolo_external_region_example' if args.canonical_example
            else 'cropped_yolo_jamming_preview'), scenario_sha256=SCENARIO_SHA,
            actuator_transport=args.actuator_transport, grpc_mode_value=args.grpc_mode_value, expected_family_sha256=FAMILY_SHA, speed_ceiling_kph=5., hood_crop_bottom_ratio=.21,
            runs=[], rpc=[], failures=[], restored=False, full_actor_restore_claimed=False)
        self.rows = open(self.output/'observations.jsonl', 'w')
        self.lock = threading.RLock()
        self.processes, self.handles, self.subscribers = [], [], []
        self.status = self.family = self.bbox = self.preview = self.context = None
        self.status_at = self.family_at = self.bbox_at = self.preview_at = 0.
        self.inject_loss = self.enabled = False
        self.collision = False
        self.guard_reason = 'gate_closed'
        self.stop_event = threading.Event()
        self.gate_thread = self.observer_thread = self.actuator_thread = None
        self.latest_control = CtrlCmd(longlCmdType=1,brake=1.)
        self.latest_control_at = 0.
        self.ground_truth = None
        self.ground_truth_at = 0.
        self.control_publishers = []
        self.publisher_check_at = 0.
        self.original = self.original_mode = None
        self.client = MoraiSimClient('asmc_camera_runtime_trial')
        self.client.connect(args.grpc_host, 7789)
        self.sim = self.client._sim_adapter
        self.ego = Vehicle(self.sim, self.client._client_key, 'Ego')
        self.info = self.ego.get_object_info()
        self.route = np.loadtxt('/data/R_KR_PG_K-city_2025/2026_molit_comp_global_path.txt')
        self.stations = np.r_[0, np.cumsum(np.linalg.norm(np.diff(self.route[:,:2],axis=0),axis=1))]
        self.gps_pub = rospy.Publisher('/camera_trial/gps', GPSMessage, queue_size=5)
        self.control_pub = None

    def log(self, kind, **values):
        with self.lock:
            self.rows.write(json.dumps(dict(kind=kind, wall=time.time(), mono=time.monotonic(), **values))+'\n')

    def call(self, stub, request, mutation=False):
        result = stub(request, timeout=15)
        if mutation:
            self.report['rpc'].append(dict(method=getattr(stub,'_method',b'unknown').decode(),
                request=MessageToDict(request),response=MessageToDict(result)))
            if result.status != 1:
                raise RuntimeError('gRPC mutation rejected: '+str(result))
        return result

    def state(self):
        return self.call(self.sim._actor_stub.GetActorState, self.info)

    def mode(self, value):
        self.call(self.sim._actor_stub.SetVehicleControlMode,
            VehicleControlModeParam(actor_info=self.info, mode=value), True)

    def gear(self, value):
        self.call(self.sim._actor_stub.SetVehicleGear,
            VehicleGear(actor_info=self.info, gear=value), True)

    def stop(self):
        self.enabled = False
        if self.control_pub:
            for _ in range(10):
                self.control_pub.publish(CtrlCmd(longlCmdType=1, brake=1.))
                time.sleep(.05)
        self.ego.control(1, 0, 1, 0, 0, 0, 0)

    def pose(self, station):
        i = min(len(self.route)-2, max(0, int(np.searchsorted(self.stations,station))-1))
        fraction = (station-self.stations[i])/(self.stations[i+1]-self.stations[i])
        xyz = self.route[i]+fraction*(self.route[i+1]-self.route[i])
        delta = self.route[min(i+4,len(self.route)-1)]-self.route[max(0,i-2)]
        result = Transform()
        result.location.x, result.location.y, result.location.z = (*xyz[:2],xyz[2]+.34)
        result.rotation.z = math.degrees(math.atan2(delta[1],delta[0]))
        return result

    def teleport(self, station):
        self.stop()
        self.call(self.sim._actor_stub.SetTransform,
            SetTransformParam(actor_info=self.info, transform=self.pose(station)), True)
        self.log('teleport', fixture_station_m=station, observed=MessageToDict(self.state()))

    def receive(self, name, value):
        with self.lock:
            setattr(self, name, value)
            setattr(self, name+'_at',time.monotonic())
        if name == 'status':
            self.log('inference_status', mode=value.mode, generation=value.generation,
                prepared=value.prepared,pipeline_ready=value.pipeline_ready,reason=value.reason)
        elif name == 'family':
            self.log('lane_family', stamp=value.header.stamp.to_nsec(), valid=value.valid,
                config_sha256=value.config_sha256, reason=value.status_reason)
        elif name == 'bbox':
            self.log('bbox_result', valid=value.valid, reason=value.reason)
        elif name == 'context':
            self.log('hd_context',valid=value.pose_valid,lane_valid=value.lane_valid,
                route_valid=value.route_valid,pose_status=value.pose_status,
                station_m=value.route_station_m,pose_stamp=value.pose_stamp.to_nsec())

    def gps(self, message):
        natural_valid = message.status > 0 and (message.latitude != 0 or message.longitude != 0)
        self.log('gps', natural_valid=natural_valid, original_stamp=message.header.stamp.to_nsec(),
            injection_drop=self.inject_loss)
        if not self.inject_loss:
            self.gps_pub.publish(message)

    def collision_callback(self, message):
        if message.collision_object or message.collision_objecta:
            self.collision = True
            self.log('collision', message=repr(message))

    def request(self, mode):
        result = rospy.ServiceProxy('/camera_semantic_inference/set_inference_mode',SetInferenceMode)(mode,10.)
        if not result.accepted:
            raise RuntimeError('mode rejected: '+result.reason)
        self.log('mode_request', mode=mode, generation=result.generation)

    def wait(self, predicate, timeout):
        deadline = time.monotonic()+timeout
        while not predicate() and time.monotonic()<deadline and not rospy.is_shutdown():
            time.sleep(.05)
        return bool(predicate())

    def run_process(self, name, command):
        handle = open(self.output/(name+'.log'),'w')
        self.handles.append(handle)
        process = subprocess.Popen(command, stdout=handle, stderr=subprocess.STDOUT,start_new_session=True)
        self.processes.append(process)
        return process

    def publishers(self):
        state = rospy.get_master().getSystemState()[2]
        return next((nodes for topic,nodes in state[0] if topic=='/ctrl_cmd'),[])

    def observe_trial(self):
        # gRPC/XMLRPC delays never block the actuator watchdog.
        while not self.stop_event.wait(.1):
            try:
                state = self.state()
                publishers = self.publishers()
                with self.lock:
                    self.ground_truth, self.ground_truth_at = state, time.monotonic()
                    self.control_publishers, self.publisher_check_at = publishers, time.monotonic()
            except Exception as exc:
                self.log('watchdog_observation_error', reason=str(exc))

    def grpc_actuator(self):
        # Serial RPC with a deadline; never replay a stale accepted preview.
        while not self.stop_event.wait(.05):
            with self.lock:
                command = (copy.deepcopy(self.latest_control) if
                    time.monotonic()-self.latest_control_at<.2 else CtrlCmd(longlCmdType=1,brake=1.))
            request = VehicleCtrlCmd(actor_info=self.info,long_cmd_type=command.longlCmdType,
                velocity=command.velocity,throttle=command.accel,brake=command.brake,
                acceleration=command.acceleration,steer=command.steering)
            try:
                response=self.sim._actor_stub.ControlVehicle(request,timeout=.15)
                self.log('grpc_control',velocity_kph=command.velocity,brake=command.brake,
                    response=MessageToDict(response))
                if response.status!=1:
                    self.enabled=False
            except Exception as exc:
                self.enabled=False
                self.log('grpc_control_failure',reason=str(exc))

    def gate(self):
        """No blind actuator forwarding: each iteration rechecks readiness/input age."""
        while not self.stop_event.wait(.05):
            now = time.monotonic()
            reason = 'gate_closed'
            command = CtrlCmd(longlCmdType=1,brake=1.)
            with self.lock:
                status, family, bbox, preview = self.status,self.family,self.bbox,self.preview
                if self.enabled:
                    if self.collision: reason='collision'
                    elif not status or status.mode!=2 or not status.pipeline_ready or now-self.status_at>.5:
                        reason='model_not_active_or_ready'
                    elif not family or not family.valid or family.config_sha256!=FAMILY_SHA or now-self.family_at>.35:
                        reason='lane_not_valid_or_fresh'
                    elif not -.05 <= (rospy.Time.now()-family.header.stamp).to_sec() <= .35:
                        reason='lane_source_stale'
                    elif not bbox or not bbox.valid or now-self.bbox_at>.5: reason='bbox_not_fresh'
                    elif not preview or now-self.preview_at>.25: reason='preview_stale'
                    elif now-self.publisher_check_at>.5: reason='publisher_check_stale'
                    elif len(self.control_publishers)!=1: reason='duplicate_control_publisher'
                    elif now-self.ground_truth_at>.35: reason='watchdog_ground_truth_stale'
                    else:
                        # Ground truth guards the trial; it never enters inference or the adapter.
                        state=self.ground_truth
                        point=np.array([state.transform.location.x,state.transform.location.y])
                        distance=float(np.min(np.linalg.norm(self.route[:,:2]-point,axis=1)))
                        if state.vehicle_state.collision_objects: reason='collision'
                        elif distance>4.5: reason='trial_route_corridor_exit'
                        elif preview.longlCmdType==2 and 0<preview.velocity<=5.:
                            command=copy.deepcopy(preview);reason='forward_preview'
                        else: reason='adapter_stop_or_speed_exceeds_trial_ceiling'
                self.guard_reason=reason
            with self.lock:
                self.latest_control, self.latest_control_at = command, time.monotonic()
            self.control_pub.publish(command)
            self.log('gate',reason=reason,velocity_kph=command.velocity,brake=command.brake)

    def start(self):
        self.original = self.state()
        self.original_mode = self.call(self.sim._actor_stub.GetVehicleControlMode,self.info).mode
        self.report['original_ego'] = MessageToDict(self.original)
        self.report['original_mode'] = self.original_mode
        scenario = Path('/data/R_KR_PG_K-city_2025/2026_molit_comp_sample_scene.json')
        if hashlib.sha256(scenario.read_bytes()).hexdigest()!=SCENARIO_SHA:
            raise RuntimeError('official scenario SHA differs')
        if self.args.load_scenario:
            response=self.call(self.sim._scenario_stub.LoadMoraiScenario,StringValue(value=self.args.scenario_name),True)
            self.report['scenario_load_response']=MessageToDict(response)
            loaded=self.state()
            self.log('scenario_loaded', ego=MessageToDict(loaded))
            expected=json.loads(scenario.read_text())['egoVehicle']['initPosition']['pos']
            self.report['scenario_spawn_verified']=math.hypot(loaded.transform.location.x-expected['x'],
                loaded.transform.location.y-expected['y'])<1.
            if not self.report['scenario_spawn_verified']:
                raise RuntimeError('scenario load success response did not restore the official Ego spawn')
        rospy.set_param('/use_sim_time',False)
        if self.publishers():
            raise RuntimeError('existing /ctrl_cmd publisher: '+repr(self.publishers()))
        self.control_pub=rospy.Publisher('/ctrl_cmd',CtrlCmd,queue_size=1)
        self.mode(self.args.grpc_mode_value)
        self.report['observed_mode_after_request']=self.call(
            self.sim._actor_stub.GetVehicleControlMode,self.info).mode
        self.gear(GEAR_MODE_D)
        self.stop()
        self.call(self.sim._simulation_stub.Resume,Empty(),True)
        for name, topic, kind in (
            ('status','/camera_semantic_inference/inference_status',InferenceStatus),
            ('family','/perception/lane/boundary_family',LaneBoundaryFamily),
            ('bbox','/planning/camera_bbox_lane/result',CameraBBoxLaneResult),
            ('preview','/planning/camera_bbox_lane_change/control_preview',CtrlCmd),
            ('context','/hd_map/context',HdMapContext)):
            self.subscribers.append(rospy.Subscriber(topic,kind,lambda m,n=name:self.receive(n,m),queue_size=10))
        self.subscribers.append(rospy.Subscriber('/gps',GPSMessage,self.gps,queue_size=5))
        self.subscribers.append(rospy.Subscriber('/CollisionData',CollisionData,self.collision_callback,queue_size=5))
        self.run_process('camera',['roslaunch','asmc_camera','perception.launch','managed:=true',
            'lane_enable_shared_family_debug:=false','camera_model_grid_max_hz:=15.0'] +
            (['start_camera_objects:=true','start_traffic_light:=true','traffic_light_wait_first:=0']
             if self.args.canonical_example else []))
        self.run_process('hd_map',['roslaunch','asmc_hd_map','competition_map.launch',
            'publish_local:=true','local_gps_topic:=/camera_trial/gps','rviz:=false'])
        if self.args.canonical_example:
            self.run_process('fusion',['roslaunch','Main','main.launch',
                'start_yolo:=false','rviz:=false'])
            self.run_process('region_example',['roslaunch','asmc_camera','camera_region_example.launch',
                'allow_example_control:=true'])
            for model,node_name in [('yolo','camera_object_detector'),('resnet18','traffic_light_infer')]:
                self.subscribers.append(rospy.Subscriber('/'+node_name+'/inference_status',InferenceStatus,
                    lambda message,model=model:self.log('other_model_status',model=model,mode=message.mode,
                        generation=message.generation,prepared=message.prepared,pipeline_ready=message.pipeline_ready),
                    queue_size=10))
        else:
            self.run_process('cropped_yolo',['roslaunch','lane_boundary_adapter','camera_bbox_detector.launch',
                'hood_crop_bottom_ratio:=0.21','python:=/usr/bin/python3','dependencies:=/usr/local/lib/python3.8/dist-packages'])
            self.run_process('preview',['roslaunch','lane_boundary_adapter','camera_bbox_lane_change.launch',
                'enable_control:=false','target_speed_kph:=5.0','inferred_speed_kph:=5.0',
                'lane_change_speed_kph:=5.0','far_obstacle_speed_kph:=5.0','lookahead_distance:=8.0',
                'hood_crop_bottom_ratio:=0.21','max_sensor_skew:=0.4','one_lane_change_only:=false',
                'ignore_bbox_during_lane_change:=true','lane_change_cooldown:=4.0',
                'expected_config_sha256:='+FAMILY_SHA,
                'calibration_file:=/root/ws/src/asmc_camera/config/cameras_v3.yaml','use_rviz:=false'])
        rospy.wait_for_service('/camera_semantic_inference/set_inference_mode',timeout=60)
        self.observer_thread=threading.Thread(target=self.observe_trial,daemon=True)
        self.observer_thread.start()
        self.gate_thread=threading.Thread(target=self.gate,daemon=True)
        self.gate_thread.start()
        if self.args.actuator_transport=='grpc':
            self.actuator_thread=threading.Thread(target=self.grpc_actuator,daemon=True)
            self.actuator_thread.start()

    def trials(self):
        for repetition in range(self.args.repeats):
            for case, station in [('approach',SHADOW_START-15),('inside_start',SHADOW_START+35),
                                  ('reentry',SHADOW_START+5)]:
                self.inject_loss=False
                if not self.args.canonical_example:self.request(0)
                self.teleport(station)
                if not self.args.canonical_example:self.request(1)
                prepared=self.wait(lambda:self.status is not None and self.status.pipeline_ready,20)
                natural_context=self.context
                if not self.args.canonical_example:self.request(2)
                if self.args.canonical_example:
                    ready=self.wait(lambda:self.status is not None and self.status.pipeline_ready,10)
                    self.inject_loss=True
                    time.sleep(1.)
                    loss_active=bool(self.status and self.status.mode==2)
                    self.inject_loss=False
                else:
                    ready=self.wait(lambda:self.status is not None and self.status.mode==2 and self.status.pipeline_ready
                        and self.family is not None and self.family.valid and self.family.config_sha256==FAMILY_SHA
                        and time.monotonic()-self.family_at<.35
                        and self.bbox is not None and self.bbox.valid and time.monotonic()-self.bbox_at<.5
                        and self.preview is not None and time.monotonic()-self.preview_at<.25
                        and self.preview.longlCmdType==2 and self.preview.velocity>0,10)
                item=dict(repetition=repetition+1,case=case,fixture_station_m=station,prepared=prepared,
                    preview_ready=ready, observed_family_sha256=self.family.config_sha256 if self.family else None,
                    natural_gps_context_valid=bool(natural_context and natural_context.pose_valid),
                    driven=False,full_shadow_traversal=False)
                if self.args.canonical_example:
                    item.update(gps_injection_lane_active=loss_active,model_mode=self.status.mode if self.status else None)
                self.report['runs'].append(item)
                if ready and self.args.allow_vehicle_control:
                    self.inject_loss=case!='approach'
                    self.enabled=True
                    started=time.monotonic();initial=self.state()
                    while time.monotonic()-started<self.args.drive_sec:
                        time.sleep(.2)
                        self.log('ego',state=MessageToDict(self.state()))
                        if self.guard_reason in ('collision','trial_route_corridor_exit','duplicate_control_publisher'):
                            break
                    self.stop();after=self.state()
                    item.update(drive_requested=True,driven=False,drive_sec=time.monotonic()-started,
                        moved_m=math.hypot(after.transform.location.x-initial.transform.location.x,
                                          after.transform.location.y-initial.transform.location.y),
                        stop_reason=self.guard_reason)
                    item['driven']=item['moved_m']>.5
                # Recovery fixture is explicit teleportation, not a claimed complete drive.
                self.inject_loss=False
                self.teleport(SHADOW_END+6)
                recovered=self.wait(lambda:self.context is not None and self.context.pose_valid
                    and self.context.lane_valid and self.context.route_valid,10)
                item['fresh_hd_recovery']=recovered
                if not self.args.canonical_example:self.request(0)
                self.log('run_summary', **item)

    def finish(self):
        self.stop_event.set()
        if self.actuator_thread:
            self.actuator_thread.join(timeout=1.)
        if self.observer_thread:
            self.observer_thread.join(timeout=17.)
        if self.gate_thread:
            self.gate_thread.join(timeout=17.)
        try:self.stop()
        except Exception as exc:self.report['failures'].append('stop: '+str(exc))
        for process in reversed(self.processes):
            if process.poll() is None:
                os.killpg(process.pid,signal.SIGINT)
                try:process.wait(timeout=15)
                except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
        for handle in self.handles:handle.close()
        if self.original is not None:
            try:
                # A successful setter or an immediate XY read is insufficient:
                # changing control mode/gear can subsequently move the vehicle.
                self.call(self.sim._simulation_stub.Pause, Empty(), True)
                self.report['simulation_left_paused'] = True
                self.mode(self.original_mode)
                self.gear(self.original.vehicle_state.gear_mode)
                self.call(self.sim._actor_stub.SetVelocity,SetVelocityParam(actor_info=self.info,velocity=0),True)
                self.call(self.sim._actor_stub.SetTransform,
                    SetTransformParam(actor_info=self.info,transform=self.original.transform),True)
                time.sleep(1.)
                # Some simulator builds change control mode while setting gear.
                self.mode(self.original_mode)
                time.sleep(.2)
                restored=self.state()
                self.report['restored_ego']=MessageToDict(restored)
                mode_response=self.call(self.sim._actor_stub.GetVehicleControlMode,self.info)
                metrics=dict(
                    position_error_m=math.sqrt(sum((getattr(restored.transform.location,a)-
                        getattr(self.original.transform.location,a))**2 for a in ('x','y','z'))),
                    rotation_error_deg=max(abs((getattr(restored.transform.rotation,a)-
                        getattr(self.original.transform.rotation,a)+180)%360-180) for a in ('x','y','z')),
                    gear_matches=restored.vehicle_state.gear_mode==self.original.vehicle_state.gear_mode,
                    mode_matches=mode_response.mode==self.original_mode)
                self.report['restoration_checks']=metrics
                self.report['restored']=(metrics['position_error_m']<.1 and
                    metrics['rotation_error_deg']<1. and metrics['gear_matches'] and metrics['mode_matches'])
                if not self.report['restored']:
                    self.report['failures'].append('settled Ego restoration did not meet pose/gear/mode tolerances')
            except Exception as exc:self.report['failures'].append('restore: '+str(exc))
        if self.control_pub:self.control_pub.unregister()
        self.client.disconnect()
        self.rows.close()
        self.report['accepted']=False  # Rehearsal evidence needs separate lifecycle/driving review.
        (self.output/'report.json').write_text(json.dumps(self.report,indent=2)+'\n')
        print(json.dumps(dict(runs=self.report['runs'],failures=self.report['failures'],restored=self.report['restored'])))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--grpc-host',default='host.docker.internal')
    parser.add_argument('--load-scenario',action='store_true')
    parser.add_argument('--canonical-example',action='store_true')
    parser.add_argument('--scenario-name',default='2026_molit_comp_sample_scene.json')
    parser.add_argument('--allow-vehicle-control',action='store_true')
    parser.add_argument('--actuator-transport',choices=['udp','grpc'],default='udp')
    parser.add_argument('--grpc-mode-value',type=int,default=VEHICLE_CONTROL_AUTO_MODE,
        help='Explicit simulator API ordinal; older vendor examples use 1 instead of protobuf 3.')
    parser.add_argument('--repeats',type=int,default=3)
    parser.add_argument('--drive-sec',type=float,default=5.)
    args=parser.parse_args()
    if args.canonical_example and args.allow_vehicle_control:
        parser.error('canonical example is a non-driving serial profile; cropped jamming handles driving trials')
    rospy.init_node('camera_runtime_trial',anonymous=True)
    trial=Trial(args)
    try:
        trial.start();trial.trials()
    except Exception as exc:
        trial.report['failures'].append(str(exc))
    finally:
        trial.finish()
    return 0 if not trial.report['failures'] else 1


if __name__=='__main__':
    raise SystemExit(main())
