#!/usr/bin/env python3
"""Capture coordinator. Owns one control gate; MPC is private, PP is a fallback."""
import json
import hashlib
import copy
import math
from pathlib import Path
import threading
import time
import sys

import rospy
import rospkg
import rosgraph
import roslaunch
import yaml
from std_msgs.msg import UInt8
from asmc_msgs.msg import VehicleState, Trajectory
from morai_msgs.msg import CtrlCmd, CollisionData

sys.path.insert(0, str(Path(rospkg.RosPack().get_path('data_collection'))/'scripts'))
from capture_mode_collector_node import CaptureModeCollector
from data_collection.capture_collection import LegacyPurePursuit, RecoveryMonitor, StableRestoration, kph_to_mps, point_route_distance
from data_collection.capture_mode import _percentile


class CaptureRouteCollector(CaptureModeCollector):
    def __init__(self):
        self.control_thread = None
        self.control_stop = threading.Event()
        self.active_capture = False
        self.fatal = None
        self.mpc = None
        self.mpc_launch = None
        self.selector = None
        self.command_pub = None
        self.actuator_prepared = False
        self.samples = []
        self.latest = {}
        self.sample_lock = threading.RLock()
        self.controller = str(rospy.get_param('~controller', 'mpc'))
        self.actuator_transport = rospy.get_param('~actuator_transport', 'grpc')
        if self.actuator_transport not in ('grpc', 'udp'):
            raise ValueError('actuator_transport must be grpc or udp')
        self.actuator_acks = 0
        if self.controller not in ('mpc', 'pp'):
            raise ValueError('controller must be mpc or pp')
        self.target_kph = float(rospy.get_param('~target_kph', 20.))
        kph_to_mps(self.target_kph)
        self.repo = Path(rospkg.RosPack().get_path('data_collection')).parents[1]
        self.route_file = Path(rospy.get_param('~route', str(self.repo/'R_KR_PG_KATRI/2026_molit_comp_global_path.txt')))
        points = [tuple(map(float, line.split()[:3])) for line in self.route_file.read_text().splitlines() if line.strip()]
        self.pp = LegacyPurePursuit(self.repo, points, self.target_kph)
        self.route_points = points
        self.driving_seen = False
        self.recovery = RecoveryMonitor(self.controller)
        self.started_at = time.monotonic()
        self.startup_timeout = float(rospy.get_param('~startup_timeout', 15.))
        self.max_seconds = float(rospy.get_param('~max_seconds', 1800.))
        self.parent_run = rospy.get_param('~parent_run', '')
        self.layout_id = rospy.get_param('~layout_id', '')
        self.inspection_only = bool(rospy.get_param('~inspection_only', False))
        if rospy.get_param('~backend', 'grpc') != 'grpc' or float(rospy.get_param('~capture_hz', 1.)) != 1.:
            raise ValueError('route collection requires grpc and 1Hz')
        publishers = dict(rosgraph.Master(rospy.get_name()).getSystemState()[0])
        if any(publishers.get(t) for t in ('/ctrl_cmd/normal', '/ctrl_cmd', '/ctrl_cmd_0', '/collection/mpc_ctrl')):
            raise RuntimeError('collection requires exclusive control ownership; stop the existing controller/selector first')
        # One gate owns /ctrl_cmd/normal; MPC commands never reach public control directly.
        self.command_pub = rospy.Publisher('/ctrl_cmd/normal', CtrlCmd, queue_size=1)
        self.selector_launch=roslaunch.scriptapi.ROSLaunch();self.selector_launch.start()
        selector_remaps=[('/planning/mode','/collection/mode')]
        if self.actuator_transport == 'grpc':
            selector_remaps.append(('/ctrl_cmd','/collection/actuator_cmd'))
        self.selector=self.selector_launch.launch(roslaunch.core.Node('asmc_planner','asmc_control_selector_node',
            name='capture_control_selector',remap_args=selector_remaps))
        self.mode_pub=rospy.Publisher('/collection/mode',UInt8,queue_size=1,latch=True)
        self.mode_pub.publish(UInt8(data=0))
        self.subscriptions = [
            rospy.Subscriber('/collection/actuator_cmd', CtrlCmd, self.grpc_command, queue_size=1),
            rospy.Subscriber(rospy.get_param('~state_topic','/collection/vehicle/state'), VehicleState, lambda m:self.receive('state',m), queue_size=1),
            rospy.Subscriber('/planning/trajectory', Trajectory, lambda m:self.receive('trajectory',m), queue_size=1),
            rospy.Subscriber('/collection/mpc_ctrl', CtrlCmd, lambda m:self.receive('mpc',m), queue_size=1),
            rospy.Subscriber('/CollisionData', CollisionData, self.collision, queue_size=1),
        ]
        self.live_ledger=[]
        super().__init__()
        self.samples_path = Path(self.writer.root)/'control.jsonl'
        self.samples_stream = self.samples_path.open('a', buffering=1)
        ledger = rospy.get_param('~spawn_ledger', '')
        if ledger:
            rows = [json.loads(line) for line in Path(ledger).read_text().splitlines() if line.strip()]
            live=[r for r in rows if r.get('created') and not r.get('removed')]
            self.live_ledger=live
            if any(not r.get('geometry_gate') or not r.get('identity_approved') or
                   (not self.inspection_only and not r.get('placement_approved')) for r in live):
                raise RuntimeError('spawn ledger contains unapproved placement/identity')
            (Path(self.writer.root)/'spawn-ledger.jsonl').write_text(Path(ledger).read_text())
        (Path(self.writer.root)/'collection.json').write_text(json.dumps(dict(controller=self.controller,
            target_kph=self.target_kph, route=str(self.route_file), layout_id=self.layout_id,
            parent_run=self.parent_run, split='train', split_group=self.layout_id,
            inspection_only=self.inspection_only,training_eligible=not self.inspection_only,
            route_sha256=hashlib.sha256(self.route_file.read_bytes()).hexdigest(),
            collection_policy_sha256=hashlib.sha256((self.repo/'src/data_collection/src/data_collection/capture_collection.py').read_bytes()).hexdigest(),
            control_gate='/ctrl_cmd/normal', actuator_transport=self.actuator_transport,
            grpc_velocity_unit='m/s', ros_velocity_unit='km/h',
            freshness=dict(state=.2,trajectory=.25)), indent=2))
        if self.controller == 'mpc':
            root=Path(rospkg.RosPack().get_path('asmc_controller'))
            params=yaml.safe_load((root/'config/runtime.yaml').read_text())
            rospy.set_param('/capture_collection_mpc', params)
            rospy.set_param('/capture_collection_mpc/mpc/config_path', str(root/'config/controller.yaml'))
            rospy.set_param('/capture_collection_mpc/topics/vehicle_state', rospy.get_param('~state_topic','/collection/vehicle/state'))
            self.mpc_launch=roslaunch.scriptapi.ROSLaunch();self.mpc_launch.start()
            self.mpc=self.mpc_launch.launch(roslaunch.core.Node('asmc_controller','mpc_controller_node',
                name='capture_collection_mpc', remap_args=[('/ctrl_cmd/normal','/collection/mpc_ctrl')]))
        self.control_thread=threading.Thread(target=self.control_loop, daemon=True)
        self.prepare_actuator()
        self.control_thread.start()

    def grpc_command(self, command):
        if self.actuator_transport != 'grpc' or not self.actuator_prepared or self.active_capture or self.control_stop.is_set():
            return
        from morai.actor.actor_control_pb2 import VehicleCtrlCmd
        try:
            response=self.actors.ControlVehicle(VehicleCtrlCmd(actor_info=self.ego_info,
                long_cmd_type=command.longlCmdType,velocity=command.velocity/3.6,
                throttle=command.accel,brake=command.brake,
                steer=max(-1.,min(1.,math.degrees(command.steering)/40.))),timeout=.3)
            if response.status != 1:raise RuntimeError('gRPC actuator rejected command')
            self.actuator_acks += 1
        except Exception as exc:
            self.fatal='actuator_failure: '+str(exc)
            rospy.signal_shutdown(self.fatal)

    def prepare_actuator(self):
        import sys
        sys.path.insert(0,str(self.repo/'tools/grpc_inha_univ/src/proto'))
        import grpc
        from morai.actor.actor_pb2_grpc import ActorStub
        from morai.common.object_info_pb2 import ObjectInfo
        from morai.common.enum_pb2 import OBJECT_TYPE_VEHICLE,OBJECT_TYPE_OBSTACLE,OBJECT_TYPE_PEDESTRIAN
        from morai.actor.actor_set_pb2 import VehicleControlModeParam, VehicleGear
        from morai.actor.actor_enum_pb2 import VEHICLE_CONTROL_AUTO_MODE, GEAR_MODE_D
        self.actor_channel=grpc.insecure_channel(rospy.get_param('~grpc_address','host.docker.internal')+':7789')
        self.actors=ActorStub(self.actor_channel)
        self.ego_info=ObjectInfo(object_type=OBJECT_TYPE_VEHICLE);self.ego_info.id.value='Ego'
        initial=self.actors.GetActorState(self.ego_info,timeout=3)
        from google.protobuf.json_format import MessageToDict
        self.ego_before=MessageToDict(initial,preserving_proto_field_name=True)
        if sum(getattr(initial.global_velocity,k)**2 for k in 'xyz')>.01:
            raise RuntimeError('Ego must be stationary before collection')
        for row in self.live_ledger:
            info=ObjectInfo(object_type=OBJECT_TYPE_PEDESTRIAN if row['actor_type']=='pedestrian' else OBJECT_TYPE_OBSTACLE,
                            client_key=row['client_key'])
            info.id.value=row['id']
            actual=self.actors.GetActorState(info,timeout=3)
            if actual.actor_info.id.value!=row['id'] or any(abs(getattr(actual.transform.location,k)-row['settled_location'][k])>.2 for k in 'xyz'):
                raise RuntimeError('reviewed actor moved or ownership changed: '+row['id'])
            expected_yaw=row.get('state_after_identity_settling',row.get('state',{})).get('transform',{}).get('rotation',{}).get('z',0.)
            if abs((actual.transform.rotation.z-expected_yaw+180)%360-180)>2:
                raise RuntimeError('reviewed actor orientation changed: '+row['id'])
        self.original_mode=self.actors.GetVehicleControlMode(self.ego_info,timeout=3).mode
        self.original_gear=initial.vehicle_state.gear_mode
        self.actuator_prepared=True
        for method,request in ((self.actors.SetVehicleControlMode,VehicleControlModeParam(actor_info=self.ego_info,mode=VEHICLE_CONTROL_AUTO_MODE)),
                               (self.actors.SetVehicleGear,VehicleGear(actor_info=self.ego_info,gear=GEAR_MODE_D))):
            if method(request,timeout=3).status!=1:raise RuntimeError('actuator preparation rejected')

    def receive(self, key, message):
        with self.sample_lock:
            self.latest[key]=(time.monotonic(),message)

    def collision(self, message):
        if message.collision_object or message.collision_objecta:
            self.fatal='collision'

    def before_capture(self, record):
        if self.fatal:
            raise RuntimeError(self.fatal)
        self.active_capture=True
        record['controller']=self.controller
        record['layout_id']=self.layout_id
        record['state']['control_source']='collection_'+self.controller+'_'+self.actuator_transport

    def after_capture(self, record):
        if not record.get('success'):
            self.active_capture=False
            self.fatal='capture_rpc_failure'
            rospy.signal_shutdown(self.fatal)
            return
        self.recovery.capture_completed(record['completed_monotonic_ns']/1e9)
        self.active_capture=False
        while not rospy.is_shutdown() and not self.fatal:
            with self.sample_lock:
                state=self.latest.get('state');traj=self.latest.get('trajectory')
            status=self.recovery.check(time.monotonic(),state[0] if state else None,
                bool(state and state[1].valid),abs(state[1].speed) if state else 0.,
                traj[0] if traj else None,bool(traj and traj[1].valid))
            if status=='ready':return
            if status in ('fallback','stop'):
                # End this run before switching. A new run owns the next controller.
                self.fatal='mpc_capture_fallback_required' if status=='fallback' else 'pp_capture_resume_failed'
                rospy.signal_shutdown(self.fatal)
                return
            time.sleep(.02)

    def _motion_allows_capture(self, snapshot):
        with self.sample_lock:
            state=self.latest.get('state');traj=self.latest.get('trajectory')
        now=time.monotonic()
        valid=state and state[1].valid and now-state[0]<=.2 and abs(state[1].speed)>=.2
        if self.controller=='mpc':valid=valid and traj and traj[1].valid and now-traj[0]<=.25
        return bool(valid) and not self.fatal

    def control_loop(self):
        try:
            while not self.control_stop.is_set() and not rospy.is_shutdown():
                now=time.monotonic();cmd=CtrlCmd(longlCmdType=1,brake=1.)
                with self.sample_lock:values=dict(self.latest)
                state=values.get('state');trajectory=values.get('trajectory');mpc=values.get('mpc')
                if now-self.started_at>self.max_seconds:self.fatal='run_deadline'
                if state and state[1].valid and now-state[0]<=.2:
                    m=state[1]; geometry=self.pp.command(m.x,m.y,m.yaw,m.speed)
                    cte=point_route_distance(self.route_points,m.x,m.y)
                    if cte>1.:self.fatal='cross_track_error_exceeds_1m'
                    if geometry['remaining_s']<4.:self.fatal='route_complete'
                    if not self.fatal and not self.active_capture:
                        if self.controller=='pp':
                            cmd=CtrlCmd(longlCmdType=1,steering=geometry['steer'],accel=geometry['accel'],brake=geometry['brake'])
                        elif trajectory and trajectory[1].valid and any(p.velocity>0 for p in trajectory[1].points) and now-trajectory[0]<=.25 and mpc and now-mpc[0]<=.2:
                            self.driving_seen=True
                            cmd=copy.deepcopy(mpc[1])
                            # Collection speed cap can only reduce the existing MPC request.
                            if cmd.longlCmdType==2:cmd.velocity=min(cmd.velocity,self.target_kph)
                            elif m.speed>kph_to_mps(self.target_kph):
                                cmd.accel=0.;cmd.brake=max(cmd.brake,min(1.,.5*(m.speed-kph_to_mps(self.target_kph))))
                    row=dict(monotonic=now, controller=self.controller, x=m.x,y=m.y, cte_m=cte, speed_mps=m.speed,
                             station_m=geometry['current_s'], capture_active=self.active_capture,
                             command=dict(steering=cmd.steering,accel=cmd.accel,brake=cmd.brake,velocity=cmd.velocity))
                    self.samples.append(row);self.samples_stream.write(json.dumps(row)+'\n')
                elif now-self.started_at>self.startup_timeout and not self.active_capture and self.recovery.completed is None:
                    self.fatal='vehicle_state_unavailable'
                if self.controller=='mpc' and now-self.started_at>self.startup_timeout and not trajectory:
                    self.fatal='mpc_trajectory_unavailable'
                elif self.controller=='mpc' and now-self.started_at>self.startup_timeout and not self.driving_seen:
                    self.fatal='mpc_initial_drive_unavailable'
                # Capture freezes simulation. Do not queue brake/control updates
                # during the pause; resume from a newly received vehicle state.
                if not self.active_capture:self.command_pub.publish(cmd)
                if self.fatal:
                    rospy.signal_shutdown(self.fatal);return
                time.sleep(.05)
        except Exception as exc:
            self.fatal=type(exc).__name__+': '+str(exc)
            self.command_pub.publish(CtrlCmd(longlCmdType=1,brake=1.))
            rospy.signal_shutdown(self.fatal)

    def close(self):
        self.control_stop.set()
        if self.active_capture:
            # The main loop owns RPC completion and manifest writing. A shutdown
            # callback must not close its client/writer mid-record. Main's finally
            # invokes close again after the completed/failed record is preserved.
            return
        if self.mpc:self.mpc.stop()
        if self.command_pub:
            for _ in range(3):
                self.command_pub.publish(CtrlCmd(longlCmdType=1,brake=1.));time.sleep(.05)
        if self.control_thread and self.control_thread is not threading.current_thread():self.control_thread.join(timeout=1.)
        if hasattr(self,'samples_stream') and not self.samples_stream.closed:
            self.samples_stream.close()
            ctes=sorted(r['cte_m'] for r in self.samples if not r['capture_active'])
            report=dict(controller=self.controller,target_kph=self.target_kph,exit_reason=self.fatal,
                actuator_transport=self.actuator_transport,actuator_acknowledgements=self.actuator_acks,
                samples=len(ctes),cte_p95_m=_percentile(ctes,95) if ctes else None,
                cte_max_m=max(ctes) if ctes else None,requires_new_run_for_fallback=self.fatal in
                ('mpc_capture_fallback_required','mpc_trajectory_unavailable','mpc_initial_drive_unavailable'),
                ego_before=getattr(self,'ego_before',None),
                tracking_passed=bool(ctes and _percentile(ctes,95)<=.5 and max(ctes)<=1. and
                                     self.fatal in (None,'route_complete')))
            (Path(self.writer.root)/'validation.json').write_text(json.dumps(report,indent=2))
        if hasattr(self,'writer'):super().close()
        try:
            if self.actuator_prepared:
                self.actuator_prepared=False
                from morai.actor.actor_set_pb2 import VehicleControlModeParam, VehicleGear
                from morai.actor.actor_control_pb2 import VehicleCtrlCmd
                self.actors.ControlVehicle(VehicleCtrlCmd(actor_info=self.ego_info,long_cmd_type=1,brake=1.),timeout=3)
                deadline=time.monotonic()+5
                while time.monotonic()<deadline:
                    state=self.actors.GetActorState(self.ego_info,timeout=3)
                    if sum(getattr(state.global_velocity,k)**2 for k in 'xyz')<.01:break
                    time.sleep(.1)
                else:raise RuntimeError('Ego did not stop; retaining brake instead of restoring drive mode')
                for method,request in ((self.actors.SetVehicleGear,VehicleGear(actor_info=self.ego_info,gear=self.original_gear)),
                    (self.actors.SetVehicleControlMode,VehicleControlModeParam(actor_info=self.ego_info,mode=self.original_mode))):
                    if method(request,timeout=3).status != 1:raise RuntimeError('Ego restoration rejected')
                gate=StableRestoration();deadline=time.monotonic()+5;samples=[];stable=False
                while time.monotonic()<deadline:
                    restored=self.actors.GetActorState(self.ego_info,timeout=3)
                    mode=self.actors.GetVehicleControlMode(self.ego_info,timeout=3).mode
                    restoration=dict(gear_matches=restored.vehicle_state.gear_mode==self.original_gear,
                        mode_matches=mode==self.original_mode,
                        stopped=sum(getattr(restored.global_velocity,k)**2 for k in 'xyz')<.01)
                    samples.append(dict(monotonic=time.monotonic(),speed_mps=math.sqrt(sum(
                        getattr(restored.global_velocity,k)**2 for k in 'xyz')),**restoration))
                    if gate.update(time.monotonic(),**restoration):stable=True;break
                    time.sleep(.1)
                restoration['stopped']=restoration['stopped'] and stable
                restoration['stable_interval_s']=.5
                restoration['samples']=samples
                (Path(self.writer.root)/'restoration.json').write_text(json.dumps(restoration,indent=2))
                if not stable:raise RuntimeError('Ego restoration verification failed')
        finally:
            if hasattr(self,'actor_channel'):self.actor_channel.close()
            if self.selector:self.selector.stop()


if __name__=='__main__':
    rospy.init_node('capture_route_collector')
    node=None
    try:
        node=CaptureRouteCollector.__new__(CaptureRouteCollector);node.__init__();node.run()
    finally:
        if node:node.close()
