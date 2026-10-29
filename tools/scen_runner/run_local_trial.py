#!/usr/bin/env python3
"""One bounded LOCAL-1 MORAI trial using scen_runner's owned replay fixture.

Explicit opt-in simulation control. A private master owns bridge/camera/LiDAR;
existing command or mode producers are a hard conflict. Only child processes and
fixture actors created here are stopped. Native collision evidence is evaluation
only and never published as a planner input. Every run uses a new registered root.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time


def collision_objects(message):
    # MORAI's repeated (-1,0) entry denotes no collision. Preserve actual
    # nonnegative object types independently of10Hz native polling so a brief
    # contact cannot fall between SDK samples. Evaluation only, never planning.
    return [dict(type=int(o.type),id=int(o.unique_id))
            for o in message.collision_object if o.type>=0]


def fresh_udp_contacts(events,after,quarantine):
    fresh=set()
    for received,objects in events:
        if received<=after:continue
        current={(o['type'],o['id']) for o in objects}
        quarantine.intersection_update(current)
        fresh.update(current-quarantine)
        after=received
    return fresh,after


def selected_command_ready(latest,now):
    wire=latest.get('wire')
    if latest.get('mode') not in (0,1) or not wire:return False
    received,message=wire
    return (0<=now-received<=.15 and message.longlCmdType==1 and
            math.isfinite(message.steering+message.accel+message.brake) and
            message.brake<.9)


def validate_trial_definition(raw,seconds):
    definition=raw['replay']['tunnel']
    env=raw.get('trial_environment',dict(weather='Sunny',hour=13))
    if env.get('weather') not in ('Sunny','Foggy') or env.get('hour') not in (11,13,15):
        raise ValueError('bounded competition weather/time required')
    records=[e['record'] for e in definition.get('baseline',[])]
    records += [e['record'] for e in definition.get('events',[]) if e['action']=='spawn']
    if any(r['definition']['type'] not in ('static','crossing') for r in records):
        raise ValueError('LOCAL campaign supports owned static/crossing replay only')
    keys={r['key'] for r in records}
    actions=list(raw.get('trial_actions',[]))
    for a in actions:
        if (a.get('action') not in ('pause','resume') or a.get('key') not in keys or
                not 0<=a.get('time_s',-1)<=seconds or not 0<=a.get('ego_travel_m',0)<=300):
            raise ValueError('bounded action must name an owned scenario actor')
    return actions


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("root", type=Path)
    p.add_argument("--scene", type=Path, required=True)
    p.add_argument("--seconds", type=float, default=40)
    p.add_argument("--cap", type=float, default=10)
    p.add_argument("--master-port", type=int, default=11348)
    p.add_argument("--lease-wait-seconds", type=float, default=0,
                   help="bounded wait before SDK/UDP startup; never stop another owner")
    p.add_argument("--no-dashboard", action="store_true", help="observer-OFF cost baseline; control is unchanged")
    p.add_argument("--dashboard-port", type=int, default=8088, help="preferred stable observer URL; never replace another listener")
    p.add_argument("--integrated", action="store_true", help="exercise real NORMAL/JAMMING mode transitions")
    p.add_argument("--bridge-invert-steer", choices=("true", "false"), default="false")
    p.add_argument("--distance", type=float, default=35)
    p.add_argument("--stationary-timeout", type=float, default=12,
                   help="bounded wait for scripted crossing/blocked traffic, max30s")
    p.add_argument("--near-blind-assist", action="store_true")
    p.add_argument("--empty", action="store_true")
    p.add_argument("--lidar-config", type=Path)
    p.add_argument("--planner-config", type=Path)
    p.add_argument("--family-gap-start-s",type=float,default=8.)
    p.add_argument("--family-gap-duration-s",type=float,default=0.,
                   help="standalone-only receive-drop fault; raw camera family is unchanged")
    p.add_argument(
        "--start-perception",
        action="store_true", default=True,
        help="owned camera/LiDAR start under the shared simulation lease",
    )
    p.add_argument(
        "--park-prop",
        action="append",
        default=[],
        help="explicit existing static actor to park +300m then restore",
    )
    args = p.parse_args()
    root = args.root
    if (
        not (root / "registration.json").is_file()
        or (root / "trial-result.json").exists()
    ):
        raise RuntimeError("new registered run required")
    if (
        not 0 < args.seconds <= 120
        or not 0 < args.cap <= 25
        or not 0 < args.distance <= 300
        or not 0 <= args.lease_wait_seconds <= 600
        or not 5 <= args.stationary_timeout <= 30
        or not 0 <= args.family_gap_start_s <= 90
        or not 0 <= args.family_gap_duration_s <= 3
        or (args.integrated and args.family_gap_duration_s>0)
    ):
        raise ValueError("bounded duration/cap required")
    if args.near_blind_assist and args.cap > 5:
        raise ValueError("unvalidated blind assist requires <=5km/h")
    if args.lidar_config and not args.start_perception:
        raise ValueError("a trial config requires owned perception startup")
    raw = json.loads(args.scene.read_text())
    definition = raw["replay"]["tunnel"]
    actions=validate_trial_definition(raw,args.seconds)
    import rospy
    import rosgraph
    from std_msgs.msg import Bool, UInt8, String
    from morai_msgs.msg import CtrlCmd, CollisionData
    from asmc_msgs.msg import LocalTrajectory, FrenetFeedback
    from google.protobuf.json_format import MessageToDict
    from scen_runner.config import load_config
    from scen_runner.runtime.validation_episode import EpisodeLink
    from scen_runner.runtime.validation_lease import ValidationBusy, ValidationLease
    from scen_runner.records.collision_archive import actor_state, load_collision
    from scen_runner.records.collision_replay import ReplaySection
    from scen_runner.runtime.signal_trial_fixture import StaticFixture

    rospy.init_node("local_trial_owner", disable_signals=True)
    master = rosgraph.Master(rospy.get_name())
    forbidden = {"/ctrl_cmd", "/ctrl_cmd/normal", "/ctrl_cmd/jamming", "/planning/mode"}
    conflicts = {
        name: nodes for name, nodes in master.getSystemState()[0] if name in forbidden
    }
    if conflicts:
        raise RuntimeError("existing control/mode owner: " + str(conflicts))
    from rosnode import get_node_names

    if {"/local_planner", "/local_controller"} & set(get_node_names()):
        raise RuntimeError("local preview already running")
    run_id = root.name
    workspace = Path(__file__).resolve().parents[2]
    files = [
        workspace / name
        for name in (
            "devel/lib/asmc_planner/asmc_local_planner_node",
            "devel/lib/asmc_controller/asmc_local_controller_node",
            "devel/lib/libasmc_local_planner_core.so",
            "src/asmc_planner/src/local/node.cpp",
            "src/asmc_planner/src/local/planner.cpp",
            "src/asmc_planner/src/local/planner.hpp",
            "src/asmc_planner/src/local/speed_profile.cpp",
            "src/asmc_planner/src/local/speed_profile.hpp",
            "src/asmc_planner/src/local/recovery.cpp",
            "src/asmc_planner/src/local/recovery.hpp",
            "src/asmc_planner/src/local/obstacle_memory.hpp",
            "src/asmc_planner/src/local/object_prediction.hpp",
            "src/asmc_planner/src/local/road_memory.hpp",
            "src/asmc_planner/src/local/motion.hpp",
            "src/asmc_planner/src/Frenet/Speed/SpeedProfiler.cpp",
            "src/asmc_controller/src/local/node.cpp",
            "src/asmc_planner/launch/local.launch",
            "src/asmc_planner/scripts/behavior_dashboard_server.py",
            "src/asmc_planner/web/dashboard_v2.js",
            "tools/scen_runner/scen_runner/runtime/trial_command_gate.py",
            "src/asmc_planner/web/index_v2.html",
            "tools/scen_runner/scen_runner/runtime/trial_monitor.py",
            "src/asmc_planner/config/local_planner.yaml",
            "src/asmc_controller/config/controller.yaml",
            "src/asmc_controller/config/local_controller.yaml",
        )
    ]
    if args.lidar_config:
        files.append(args.lidar_config)
    if args.planner_config:
        files.append(args.planner_config)
    files.append(Path(__file__).resolve())
    (root / "runtime-provenance.json").write_text(
        json.dumps(
            dict(
                options=dict(
                    vars(args),
                    bag_compression='lz4',
                    root=str(root),
                    scene=str(args.scene),
                    lidar_config=str(args.lidar_config) if args.lidar_config else None,
                    planner_config=str(args.planner_config) if args.planner_config else None,
                ),
                sha256={
                    (
                        str(f.relative_to(workspace))
                        if workspace in f.parents
                        else str(f)
                    ): hashlib.sha256(f.read_bytes()).hexdigest()
                    for f in files
                    if f.is_file()
                },
            ),
            indent=2,
        )
    )
    events = (root / "events.jsonl").open("x")

    def emit(data):
        events.write(
            json.dumps(
                dict(
                    monotonic=time.monotonic(),
                    ros_time=rospy.Time.now().to_sec(),
                    **data
                ),
                ensure_ascii=False,
            )
            + "\n"
        )
        events.flush()

    link = EpisodeLink(load_config())
    lease = ValidationLease(link.host, link.port)
    wait_started=time.monotonic()
    while True:
        try:
            lease.acquire()
            break
        except ValidationBusy:
            waited=time.monotonic()-wait_started
            if waited>=args.lease_wait_seconds: raise
            emit(dict(event='lease_wait',elapsed_s=waited,simulator_mutated=False))
            time.sleep(min(1.,args.lease_wait_seconds-waited))
    emit(dict(event='lease_acquired',waited_s=time.monotonic()-wait_started))
    link.client_key = "local1_" + run_id
    link.connect()
    from proto.morai.common.type_pb2 import Empty
    from proto.morai.actor.actor_set_pb2 import VehicleControlModeParam
    from proto.morai.actor.actor_enum_pb2 import VehicleControlMode

    ego = link.ego()
    original = actor_state(ego)
    states = link.states()
    original_control = link._require()._actor_stub.GetVehicleControlMode(
        link.ego_info(), timeout=3
    )
    weather = link._require()._environment_stub.GetWeather(Empty(), timeout=3)
    timestamp = link._require()._environment_stub.GetTime(Empty(), timeout=3)
    (root / "sim-before.json").write_text(
        json.dumps(
            dict(
                ego=original,
                ego_proto=MessageToDict(ego),
                control_mode=MessageToDict(original_control),
                actors=[actor_state(x) for x in states],
                weather=MessageToDict(weather),
                time=MessageToDict(timestamp),
            ),
            indent=2,
        )
    )
    # load_collision returns a normalized replay definition; use its nested
    # tunnel section, not the failure snapshot as an accidental start position.
    if args.empty:
        definition = dict(definition, baseline=[], events=[], allow_empty_scene=True)
    section = None
    children = []
    observer_pids = set()
    monitor = None
    child_names = {}
    handles = []
    fixture = StaticFixture(link, emit)
    mode = None
    latest = {}
    collision_events = []
    preexisting_contacts=set()
    udp_quarantine=set()
    native_evidence_limited=False
    control_rows = (root / "control.jsonl").open("x")
    handles.append(control_rows)

    def control(message):
        control_rows.write(message.data + "\n")
        control_rows.flush()

    transport_gate = None
    transport_timer = None
    family_fault_dropped = 0
    def commands(topic, name):
        stream = (root / name).open("x")
        handles.append(stream)
        def write(m):
            if topic=='/ctrl_cmd':
                latest['wire']=(time.monotonic(),m)
                if transport_gate is not None:transport_gate.command(m)
            stream.write(json.dumps(dict(received_steady=time.monotonic(), ros_time=rospy.Time.now().to_sec(),
                steering=m.steering, accel=m.accel, brake=m.brake)) + "\n")
        return rospy.Subscriber(topic, CtrlCmd, write, queue_size=100)

    def collision(message):
        objects=collision_objects(message)
        now=time.monotonic()
        collision_events.append((now,objects))
        del collision_events[:-100]
        if objects:
            emit(dict(event='udp_collision',objects=objects,
                      source_stamp=message.header.stamp.to_sec()))

    def on_mode(message):
        latest['mode']=int(message.data)
        emit(dict(event='mode',value=int(message.data)))

    subscriptions = [
        rospy.Subscriber('/CollisionData',CollisionData,collision,queue_size=100),
        rospy.Subscriber('/planning/frenet_feedback_v2', FrenetFeedback,
                         lambda m: latest.update(normal_feedback=m), queue_size=1),
        commands("/ctrl_cmd", "wire.jsonl"),
        commands("/ctrl_cmd/normal", "normal-commands.jsonl"),
        commands("/ctrl_cmd/jamming", "mpc-commands.jsonl"),
        rospy.Subscriber('/planning/mode', UInt8,on_mode,queue_size=10),
        rospy.Subscriber('/planning/mode/local_prepare', Bool,
            lambda m: emit(dict(event='prepare', value=bool(m.data))), queue_size=10),
        rospy.Subscriber('/planning/control_selector/diagnostics', String,
            lambda m: emit(dict(event='selector', value=m.data)), queue_size=10),
        rospy.Subscriber(
            "/planning/local/control_trace", String, control, queue_size=100
        ),
        rospy.Subscriber(
            "/planning/local/trajectory",
            LocalTrajectory,
            lambda m: latest.update(plan=m, receive=time.monotonic()),
            queue_size=1,
        ),
    ]
    from diagnostic_msgs.msg import DiagnosticArray
    from scen_runner.records.local_resources import LocalResources
    resources = LocalResources(root)
    delivery_file = (root / 'deliveries.jsonl').open('x')
    timing_file = (root / 'timings.jsonl').open('x')
    handles += [delivery_file, timing_file]
    timing_seen = set()
    def delivery(m):
        delivery_file.write(json.dumps(dict(sequence=m.plan_sequence, valid=m.valid,
            reason=m.reason, frame=m.header.frame_id, source=m.scene_stamp.to_sec(),
            stamp=m.header.stamp.to_sec(), received=rospy.Time.now().to_sec(),
            received_steady=time.monotonic()))+'\n')
    def diagnostics(m):
        for status in m.status:
            if status.name != 'planning/local': continue
            values = {v.key:v.value for v in status.values}
            for row in json.loads(values.get('timings','[]')):
                if row[0] not in timing_seen:
                    timing_seen.add(row[0]); timing_file.write(json.dumps(row)+'\n')
    subscriptions += [
        rospy.Subscriber('/planning/local/trajectory', LocalTrajectory, delivery, queue_size=100),
        rospy.Subscriber('/planning/local/diagnostics', DiagnosticArray, diagnostics, queue_size=10)]

    def start(cmd, name, env=None, observer=False):
        f = (root / (name + ".log")).open("x")
        handles.append(f)
        proc = subprocess.Popen(
            cmd, stdout=f, stderr=subprocess.STDOUT, start_new_session=True, env=env
        )
        children.append(proc)
        child_names[proc.pid]=name
        if observer: observer_pids.add(proc.pid)
        return proc

    reason = "preparation_failed"
    started = None
    contacts = []
    travel = 0.0
    last = None
    speedmax = 0.0
    cleanup_errors = []
    mutated = False
    child_exits=[]
    try:
        from scen_runner.runtime.private_master import require_udp_ports_free
        require_udp_ports_free([9099, 9798, 7505, 9092, 9291, 9293, 9295, 9297, 9299, 9301, 9303])
        if not args.no_dashboard:
            try:
                from scen_runner.runtime.trial_monitor import TrialMonitor
                import socket
                monitor = TrialMonitor(workspace, root, args.dashboard_port)
                # Backend reserves its own HTTP listener before ROS registration.
                # Identity verification also rejects a reused/raced socket.
                with socket.socket() as reservation:
                    reservation.bind(('127.0.0.1',0)); backend_port=reservation.getsockname()[1]
                observer = start(['rosrun','asmc_planner','behavior_dashboard_server.py',
                    '--port',str(backend_port),'__name:=local_trial_dashboard',
                    '_port:='+str(backend_port),'_package_path:='+str(workspace/'src/asmc_planner'),
                    '_interface_version:=v2','/vehicle/state:=/planning/vehicle/state'],
                    'dashboard',env=monitor.environment(),observer=True)
                monitor.update(backend_port=backend_port,backend_pid=observer.pid)
                resources.register_observer(observer.pid,'dashboard_backend')
                resources.register_observer(monitor.endpoint['pid'],'dashboard_frontend')
                emit(dict(event='monitor_started',url=monitor.endpoint['url'],
                          master_uri=os.environ['ROS_MASTER_URI'],backend_pid=observer.pid))
                print('Live trial dashboard: '+monitor.endpoint['url'],flush=True)
            except Exception as exc:
                # Observation startup cannot create a new control failure path.
                emit(dict(event='monitor_error',error=str(exc)))
                (root/'monitor-startup-error.json').write_text(json.dumps(dict(error=str(exc))))
        # A private transport topic prevents UDP's fixed D gear from driving
        # during SDK-P warmup. Operating bridge/selector contracts are intact.
        from scen_runner.runtime.trial_command_gate import TrialCommandGate
        def held_command():
            command=CtrlCmd();command.longlCmdType=1;command.brake=1.
            return command
        actuator=rospy.Publisher('/local_trial/actuator_cmd',CtrlCmd,queue_size=1)
        transport_gate=TrialCommandGate(actuator.publish,held_command)
        if monitor:monitor.update(actuation_hold=True)
        transport_timer=rospy.Timer(rospy.Duration(.05),lambda _:transport_gate.tick())
        subscriptions.append(commands('/local_trial/actuator_cmd','actuator.jsonl'))
        import xml.etree.ElementTree as ET
        bridge_xml=ET.Element('launch')
        ET.SubElement(bridge_xml,'remap',{'from':'/ctrl_cmd','to':'/local_trial/actuator_cmd'})
        include=ET.SubElement(bridge_xml,'include',{'file':'$(find asmc_bridge)/launch/bridge.launch'})
        for name,value in [('transport_profile','local'),('invert_steer',args.bridge_invert_steer)]:
            ET.SubElement(include,'arg',{'name':name,'value':value})
        bridge_path=root/'trial-bridge.launch'
        ET.ElementTree(bridge_xml).write(str(bridge_path),encoding='unicode')
        start(['roslaunch',str(bridge_path)],'bridge')
        rospy.wait_for_message("/Ego_topic", __import__("morai_msgs.msg", fromlist=["EgoVehicleStatus"]).EgoVehicleStatus, timeout=15)
        rospy.wait_for_message("/imu", __import__("sensor_msgs.msg", fromlist=["Imu"]).Imu, timeout=10)
        if args.family_gap_duration_s:
            from asmc_msgs.msg import LaneBoundaryFamily
            family_relay=rospy.Publisher('/local_trial/family',LaneBoundaryFamily,queue_size=2)
            def family_input(message):
                nonlocal family_fault_dropped
                released=latest.get('released_monotonic')
                elapsed=time.monotonic()-released if released is not None else -1.
                if args.family_gap_start_s<=elapsed<args.family_gap_start_s+args.family_gap_duration_s:
                    family_fault_dropped+=1
                    if family_fault_dropped==1:emit(dict(event='family_receive_fault_started',source_stamp=message.header.stamp.to_sec()))
                    return
                family_relay.publish(message) # Exact original stamp/geometry, no fake valid flag.
            subscriptions.append(rospy.Subscriber('/perception/lane/boundary_family',LaneBoundaryFamily,family_input,queue_size=4))
        if args.start_perception:
            # LOCAL-2C: acquire the shared simulator lease BEFORE any producer
            # startup. A concurrent worktree remains an exclusive owner.
            occupied = {
                "/perception/lidar/lidar_pipeline",
                "/yolo_node",
                "/twinlite_node",
                "/traffic_light_node",
            } & set(get_node_names())
            pubs = dict(master.getSystemState()[0])
            sensor_outputs = [
                "/perception/camera/objects/observations",
                "/perception/lane/boundary_family",
                "/perception/lidar/tracks_local",
            ]
            if occupied or any(pubs.get(t) for t in sensor_outputs):
                raise RuntimeError("perception already owned; omit --start-perception")
            start(["roslaunch", "asmc_camera", "camera.launch"], "camera")
            lidar_command = ["roslaunch", "asmc_lidar", "lidar.launch"]
            if args.lidar_config:
                lidar_command.append("pipeline_config:=" + str(args.lidar_config))
            start(lidar_command, "lidar")
            # Actual delivery rather than a ROS advertised name is required.
            rospy.wait_for_message(
                "/perception/lane/boundary_family",
                __import__(
                    "asmc_msgs.msg", fromlist=["LaneBoundaryFamily"]
                ).LaneBoundaryFamily,
                timeout=60.0,
            )
            # Warm all inference workers before moving the fixture. Cold-start
            # gaps remain in camera/lidar logs, not mixed with a driving trial.
            time.sleep(8.0)
            if any(child.poll() is not None for child in children if child.pid not in observer_pids):
                raise RuntimeError("owned perception exited during warmup")
        missing = set(args.park_prop) - {s.actor_info.id.value for s in states}
        if missing:
            raise RuntimeError("fixture IDs changed before trial: " + str(missing))
        mutated = True
        for aid in args.park_prop:
            found = next(s for s in states if s.actor_info.id.value == aid)
            link.actor_scopes[aid] = found.actor_info.client_key
            fixture.park(aid, 30, 0)
        link.reset(definition["start_pose"])
        link.gear_ego(False)
        link.zero_ego()
        link.pause_ego(False)
        time.sleep(0.5)
        link.hold()
        environment=raw.get("trial_environment",dict(weather="Foggy",hour=13))
        link.set_environment(environment["weather"],environment["hour"])
        from proto.morai.environment import environment_enum_pb2 as environment_enum
        def verify_environment():
            stub=link._require()._environment_stub
            current=stub.GetWeather(Empty(),timeout=3);clock=stub.GetTime(Empty(),timeout=3)
            emit(dict(event='environment_readback',requested=environment,
                      actual_weather=MessageToDict(current),actual_time=MessageToDict(clock)))
            expected={'Sunny':environment_enum.WEATHER_TYPE_SUNNY,'Foggy':environment_enum.WEATHER_TYPE_FOGGY}
            if current.type!=expected[environment['weather']] or clock.hour!=environment['hour']:
                raise RuntimeError('trial weather/time changed; refusing mixed-condition comparison')
        verify_environment()
        emit(dict(event="environment_applied",**environment))
        section = ReplaySection(
            "tunnel",
            42,
            link.client_key + "_objects",
            load_config(),
            lambda: False,
            print,
            definition=definition,
        )
        deadline = time.monotonic() + 15
        while not section.poll_prepare():
            if time.monotonic() > deadline:
                raise RuntimeError("fixture prepare timeout")
            time.sleep(0.1)
        emit(
            dict(
                event="fixture_ready",
                owned=section.owned(),
                start=actor_state(link.ego()),
                fixture_states=[
                    actor_state(x)
                    for x in link.states()
                    if x.actor_info.id.value in section.owned()
                ],
            )
        )
        for action in list(actions):
            if action['action']=='pause' and action['time_s']==0 and action.get('ego_travel_m',0)==0:
                actor=section.by_key.get(action['key'])
                if actor is None or actor.kind!=2:
                    raise ValueError('initial pause requires an owned prepared pedestrian')
                section.plans[actor.request_id].hold_at_start=True
                actions.remove(action)
                emit(dict(event='scripted_actor_action',definition=action,actor_id=actor.native_id,phase='pre_release'))
        prefix = "/experimental/shadow/semantic_space_trial/"
        topics = [
            "/Ego_topic",
            "/imu",
            "/gps",
            "/perception/lane/boundary_family",
            "/perception/lidar/tracks_local",
            "/perception/lidar/trace",
            "/velodyne_points",
            "/perception/camera/objects/observations",
            "/perception/camera/objects/status",
            "/perception/camera/lane/status",
            "/perception/lane/diagnostics",
            "/perception/lane/ground_projection",
            "/perception/bev/diagnostics",
            "/perception/lidar/diagnostics",
            "/morai_sensor_bridge/frame_timing",
            prefix + "context",
            prefix + "coverage",
            prefix + "drivable_probability",
            "/planning/local/odometry",
            "/planning/local/trajectory",
            "/planning/local/trace",
            "/planning/local/control_trace",
            "/planning/local/diagnostics",
            "/planning/mode",
            "/planning/mode/local_prepare",
            "/planning/control_selector/diagnostics",
            "/planning/frenet_feedback_v2",
            "/ctrl_cmd/normal",
            "/ctrl_cmd",
            "/ctrl_cmd/jamming",
            "/local_trial/actuator_cmd",
            "/local_trial/family",
            "/CollisionData",
            "/image_jpeg/compressed",
            "/cam_left/image_jpeg/compressed",
            "/cam_right/image_jpeg/compressed",
        ]
        start(
            [
                "rosbag",
                "record",
                "--lz4",
                "--split",
                "--duration=60",
                "--buffsize=128",
                "-O",
                str(root / "sensors"),
                *topics,
            ],
            "record",
        )
        if args.integrated:
            command=["roslaunch", "asmc_planner", "autonomy_no_bridges_v2.launch",
                   "jamming_planner:=local", "dashboard:=false",
                   "local_speed_cap_kph:=" + str(args.cap), "local_run_id:=" + run_id,
                   "local_journal_path:=" + str(root / "decisions.jsonl")]
            if args.planner_config: command.append('local_planner_config:='+str(args.planner_config))
            start(command,"autonomy")
        else:
            # Start only the final command router, never autonomy/map planning or
            # the old BBox lane follower. Local MPC owns its historical JAMMING slot.
            selector = start(
                [
                    "rosrun",
                    "asmc_planner",
                    "asmc_control_selector_node",
                    "__name:=local_trial_selector",
                ],
                "selector",
            )
            local_command = [
                    "roslaunch",
                    "asmc_planner",
                    "local.launch",
                    "enable_control:=true",
                    "semantic_input:=true",
                    "run_id:=" + run_id,
                    "journal_path:=" + str(root / "decisions.jsonl"),
                    "speed_cap_kph:=" + str(args.cap),
                    "near_blind_assist:=" + str(args.near_blind_assist).lower(),
                    "planner_config:=" + str(args.planner_config or workspace/'src/asmc_planner/config/local_planner.yaml'),
                ]
            if args.family_gap_duration_s:
                # Remap only this included standalone LOCAL stack. Camera
                # inference/raw family and every other master remain intact.
                trial_xml=ET.Element('launch')
                ET.SubElement(trial_xml,'remap',{'from':'/perception/lane/boundary_family','to':'/local_trial/family'})
                include=ET.SubElement(trial_xml,'include',{'file':'$(find asmc_planner)/launch/local.launch'})
                for argument in local_command[3:]:
                    key,value=argument.split(':=',1)
                    ET.SubElement(include,'arg',{'name':key,'value':value})
                trial_path=root/'trial-local.launch'
                ET.ElementTree(trial_xml).write(str(trial_path),encoding='unicode')
                local_command=['roslaunch',str(trial_path)]
            local=start(local_command,'local')
            mode = rospy.Publisher("/planning/mode", UInt8, queue_size=1, latch=True)
            mode.publish(UInt8(1))
        # Sensor orientation is stale while paused. Settle with P before D.
        link.gear_ego(False)
        link.pause_ego(False)
        time.sleep(3)
        ready_until = time.monotonic()+10
        while True:
            plan = latest.get('plan')
            normal_feedback = latest.get('normal_feedback')
            local_ready = plan and plan.valid and 0 <= (rospy.Time.now()-plan.scene_stamp).to_sec() <= .35
            normal_ready = args.integrated and normal_feedback and normal_feedback.plan_valid and normal_feedback.command_accepted
            # A valid NORMAL feedback can precede the selector's latched mode
            # (drive14). Never enable AUTO on that advertisement alone.
            if (local_ready or normal_ready) and selected_command_ready(latest,time.monotonic()): break
            if any(child.poll() is not None for child in children if child.pid not in observer_pids):
                raise RuntimeError('owned producer exited before control readiness')
            if time.monotonic() > ready_until:
                reason = 'input_or_plan_not_ready'
                raise RuntimeError('no fresh usable trajectory before enabling auto control')
            time.sleep(.05)
        # MORAI can retain the native unknown-contact marker after a removed
        # actor (observed after drive06). Preserve/quarantine startup records;
        # they are not a new contact. UDP events remain independently active.
        before_states=link.states()
        before_drive=actor_state(next(x for x in before_states if x.actor_info.id.value==link.ego_id))
        preexisting_contacts=set(before_drive['native_contacts'])-{link.ego_id,''}
        if preexisting_contacts:
            native_evidence_limited=True
            emit(dict(event='preexisting_native_contact',contacts=sorted(preexisting_contacts),
                      limitation='Persistent startup IDs cannot certify later contact release; raw samples and UDP evidence retained.'))
            from scen_runner.runtime.contact_evidence import inspect_contacts
            evidence=inspect_contacts(dict(contacts=preexisting_contacts,owned_ids=set(section.owned()),
                actor_states=before_states,query_s=0.),set(section.owned()),link.ego_id)
            emit(dict(event='startup_contact_geometry',evidence=evidence))
            # Native IDs can be reused after a deleted contact. Existing team
            # contact_evidence rejects only bodies that are provably far away;
            # never dismiss a physically nearby startup contact as stale.
            if any(e['verdict']!='rejected' for e in evidence):
                raise RuntimeError('near/unknown owned actor contact before trial release')
        if collision_events:
            udp_quarantine={(o['type'],o['id']) for o in collision_events[-1][1]}
        if udp_quarantine:
            from scen_runner.runtime.contact_evidence import inspect_contacts
            evidence=inspect_contacts(dict(contacts={str(i) for _,i in udp_quarantine},
                owned_ids=set(section.owned()),actor_states=before_states,query_s=0.),
                set(section.owned()),link.ego_id)
            emit(dict(event='startup_udp_contact_geometry',raw=sorted(udp_quarantine),evidence=evidence))
            if any(e['verdict']!='rejected' for e in evidence):
                raise RuntimeError('near/unknown owned UDP contact before trial release')
            native_evidence_limited=True
        # Independent native boxes are an evaluation guard for recycled,
        # continuously published collision IDs. They never enter the planner.
        from scen_runner.records.local_margin_audit import footprint
        snapshot=link.states()
        initial=actor_state(next(x for x in snapshot if x.actor_info.id.value==link.ego_id))
        verify_environment()
        if not link.held_at(definition['start_pose']):
            emit(dict(event='trial_start_pose_or_speed_invalid',actual=initial,expected=definition['start_pose']))
            raise RuntimeError('trial moved before release; fixed start condition not met')
        last=(initial['pose']['x'],initial['pose']['y'])
        started = time.monotonic()
        last_udp_evaluation=started
        emit(dict(event='driving_started'))
        emit(dict(event='native_sample',phase='pre_release',ego=initial,travel_m=0.,
            plan_valid=bool(latest.get('plan') and latest['plan'].valid),
            plan_reason=latest['plan'].reason if latest.get('plan') else 'missing',
            plan_sequence=latest['plan'].plan_sequence if latest.get('plan') else 0,
            owned=section.owned(),actor_states=[actor_state(x) for x in snapshot if x.actor_info.id.value in section.owned()]))
        r = VehicleControlModeParam()
        r.actor_info.CopyFrom(link.ego_info())
        r.mode = VehicleControlMode.VEHICLE_CONTROL_AUTO_MODE
        link.check(link._require()._actor_stub.SetVehicleControlMode(r, timeout=3),"enable native auto control")
        link.release()
        latest['released_monotonic']=time.monotonic()
        transport_gate.release()
        if monitor:monitor.update(phase='driving',actuation_hold=False)
        emit(dict(event='actuation_gate_released',held_commands=transport_gate.held,
                  actual_pre_release_speed_kph=initial['speed_kmh'],topic='/local_trial/actuator_cmd'))
        emit(dict(event='ego_released'))
        # The earlier LOCAL fixture runner only prepared saved actors. Without
        # resuming/ticking ReplaySection, pedestrians/events stayed frozen and
        # a nominal dynamic test was actually a static-placement test.
        section.resume_schedule(started)
        emit(dict(event='actors_resumed'))
        reason = "time_limit"
        stationary = started
        last_environment_check=time.monotonic()
        while time.monotonic() - started < args.seconds:
            if time.monotonic()-last_environment_check>=10.:
                try:verify_environment()
                except Exception as error:
                    reason='environment_changed'
                    emit(dict(event='environment_trial_rejected',error=str(error)))
                    break
                last_environment_check=time.monotonic()
            section.tick()
            elapsed=time.monotonic()-started
            for action in list(actions):
                if elapsed < action['time_s'] or travel < action.get('ego_travel_m',0):
                    continue
                actor=section.by_key.get(action['key'])
                if actor is None or actor.native_id not in section.owned():
                    raise RuntimeError('scripted action must name this trial owned actor')
                section.link.pause_actor(actor.request_id,actor.kind,action['action']=='pause')
                emit(dict(event='scripted_actor_action',definition=action,actor_id=actor.native_id))
                actions.remove(action)
            child_exits=[dict(name=child_names[x.pid],returncode=x.returncode)
                         for x in children if x.pid not in observer_pids and x.poll() is not None]
            if child_exits:
                emit(dict(event='child_exit',processes=child_exits))
                reason = "child_exit"
                break
            if os.statvfs(root).f_bavail * os.statvfs(root).f_frsize < 2 * 2**30:
                reason = "disk_floor"
                break
            if (root / "stop-request.json").exists():
                reason = "external_stop_request"
                break
            snapshot=link.states()
            state = actor_state(next(x for x in snapshot if x.actor_info.id.value==link.ego_id))
            resources.sample()
            current_contacts=set(state.get('native_contacts',[]))-{link.ego_id,''}
            preexisting_contacts.intersection_update(current_contacts)
            native=sorted(current_contacts-preexisting_contacts)
            new_udp,last_udp_evaluation=fresh_udp_contacts(list(collision_events),last_udp_evaluation,udp_quarantine)
            udp=[f'udp:{t}:{i}' for t,i in new_udp]
            own_box=footprint(state)
            boxes=[(x.actor_info.id.value,footprint(actor_state(x))) for x in snapshot
                   if x.actor_info.id.value in section.owned()]
            overlaps=[aid for aid,box in boxes if own_box.intersection(box).area>1e-5]
            point = (state["pose"]["x"], state["pose"]["y"])
            if last is not None:
                travel += math.dist(point, last)
            last = point
            speedmax = max(speedmax, state["speed_kmh"])
            plan = latest.get("plan")
            emit(
                dict(
                    event="native_sample",
                    ego=state,
                    travel_m=travel,
                    plan_valid=bool(plan and plan.valid),
                    plan_reason=plan.reason if plan else "missing",
                    plan_sequence=plan.plan_sequence if plan else 0,
                    owned=section.owned(),
                    actor_states=[actor_state(x) for x in snapshot
                                  if x.actor_info.id.value in section.owned()],
                    minimum_native_box_gap_m=min((own_box.distance(box) for _,box in boxes),default=None),
                )
            )
            if native or udp:
                contacts += native+udp
                reason = "native_contact" if native else "udp_contact"
                break
            if overlaps:
                emit(dict(event='native_box_overlap',actors=overlaps,
                          scope='conservative SDK hull reference, not a mesh collision claim'))
                reason='native_box_overlap'
                break
            if state["speed_kmh"] > 0.36:
                stationary = time.monotonic()
            if time.monotonic() - stationary > args.stationary_timeout:
                reason = "stationary_%gs" % args.stationary_timeout
                break
            if travel >= args.distance:
                reason = "distance_limit"
                break
            time.sleep(0.1)
    finally:
        driving_finished=time.monotonic()
        if transport_gate:transport_gate.close()
        if monitor: monitor.update(phase='finishing')
        if started is not None: emit(dict(event='driving_ended'))
        resources.close()
        if mutated:
            try:
                link.hold()
            except Exception as e:
                cleanup_errors.append("hold: " + str(e))
        if mode:
            mode.unregister()
        for proc in reversed(children):
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
                try:
                    proc.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGTERM)
                    proc.wait(timeout=10)
        if transport_timer:transport_timer.shutdown()
        if section:
            try:
                section.cleanup()
            except Exception as e:
                cleanup_errors.append("section: " + str(e))
        if mutated:
            try:
                fixture.restore()
            except Exception as e:
                cleanup_errors.append("fixture: " + str(e))
            try:
                link.reset(original["pose"])
                link.gear_ego(False)
                link.pause_ego(False)
                restore_control = VehicleControlModeParam()
                restore_control.actor_info.CopyFrom(link.ego_info())
                restore_control.mode = original_control.mode
                link.check(
                    link._require()._actor_stub.SetVehicleControlMode(
                        restore_control, timeout=3
                    ),
                    "restore control mode",
                )
                w = type(weather)()
                w.CopyFrom(weather)
                w.instantly = True
                t = type(timestamp)()
                t.CopyFrom(timestamp)
                t.instantly = True
                t.data_only = False
                link.check(
                    link._require()._environment_stub.SetWeather(w, timeout=3),
                    "restore weather",
                )
                link.check(
                    link._require()._environment_stub.SetTime(t, timeout=3), "restore time"
                )
                (root / "sim-after.json").write_text(
                    json.dumps(
                        dict(
                            ego=actor_state(link.ego()),
                            actors=[actor_state(x) for x in link.states()],
                        ),
                        indent=2,
                    )
                )
            except Exception as e:
                cleanup_errors.append("environment: " + str(e))
        link.disconnect()
        lease.release()
        if monitor: monitor.close(reason)
        emit(dict(event="finish", reason=reason, cleanup_errors=cleanup_errors))
        events.close()
        for f in handles:
            f.close()
        result = dict(
            run_id=run_id,
            reason=reason,
            native_contacts=contacts,
            native_evidence_limited=native_evidence_limited,
            travel_m=travel,
            maximum_speed_kph=speedmax,
            seconds=driving_finished - started if started else 0,
            cleanup_seconds=time.monotonic()-driving_finished,
            cap_kph=args.cap,
            near_blind_assist=args.near_blind_assist,
            cleanup_errors=cleanup_errors,
            closed_loop_acceptance=False,
            child_exits=child_exits,
            transport_gate=dict(held_commands=transport_gate.held,forwarded_commands=transport_gate.forwarded) if transport_gate else None,
            family_receive_fault=dict(start_s=args.family_gap_start_s,duration_s=args.family_gap_duration_s,dropped=family_fault_dropped),
        )
        (root / "trial-result.json").write_text(json.dumps(result, indent=2))
        print(json.dumps(result), flush=True)
    from scen_runner.records.analyze_local import analyze

    if (root / "decisions.jsonl").exists():
        print(json.dumps(analyze(root)), flush=True)
    if cleanup_errors:
        raise RuntimeError("incomplete restoration: " + str(cleanup_errors))


if __name__ == "__main__":
    owner = argparse.ArgumentParser(add_help=False)
    owner.add_argument("root", type=Path)
    owner.add_argument("--master-port", type=int, default=11348)
    selected, _ = owner.parse_known_args()
    if not (selected.root / "registration.json").is_file():
        raise RuntimeError("registered output required before master startup")
    from scen_runner.runtime.private_master import PrivateMaster
    with PrivateMaster(selected.root, selected.master_port):
        main()
