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


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("root", type=Path)
    p.add_argument("--scene", type=Path, required=True)
    p.add_argument("--seconds", type=float, default=40)
    p.add_argument("--cap", type=float, default=10)
    p.add_argument("--master-port", type=int, default=11348)
    p.add_argument("--lease-wait-seconds", type=float, default=0,
                   help="bounded wait before SDK/UDP startup; never stop another owner")
    p.add_argument("--integrated", action="store_true", help="exercise real NORMAL/JAMMING mode transitions")
    p.add_argument("--bridge-invert-steer", choices=("true", "false"), default="false")
    p.add_argument("--distance", type=float, default=35)
    p.add_argument("--near-blind-assist", action="store_true")
    p.add_argument("--empty", action="store_true")
    p.add_argument("--lidar-config", type=Path)
    p.add_argument("--planner-config", type=Path)
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
    ):
        raise ValueError("bounded duration/cap required")
    if args.near_blind_assist and args.cap > 5:
        raise ValueError("unvalidated blind assist requires <=5km/h")
    if args.lidar_config and not args.start_perception:
        raise ValueError("a trial config requires owned perception startup")
    raw = json.loads(args.scene.read_text())
    definition = raw["replay"]["tunnel"]
    import rospy
    import rosgraph
    from std_msgs.msg import Bool, UInt8, String
    from morai_msgs.msg import CtrlCmd
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
            "src/asmc_planner/src/local/road_memory.hpp",
            "src/asmc_planner/src/local/motion.hpp",
            "src/asmc_planner/src/Frenet/Speed/SpeedProfiler.cpp",
            "src/asmc_controller/src/local/node.cpp",
            "src/asmc_planner/launch/local.launch",
            "src/asmc_planner/config/local_planner.yaml",
            "src/asmc_controller/config/controller.yaml",
            "src/asmc_controller/config/local_controller.yaml",
        )
    ]
    if args.lidar_config:
        files.append(args.lidar_config)
    if args.planner_config:
        files.append(args.planner_config)
    (root / "runtime-provenance.json").write_text(
        json.dumps(
            dict(
                options=dict(
                    vars(args),
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
    for entry in definition.get("baseline", []):
        if entry["record"]["definition"]["type"] != "static":
            raise RuntimeError("LOCAL-1 first unit accepts static fixtures only")
    section = None
    children = []
    handles = []
    fixture = StaticFixture(link, emit)
    mode = None
    latest = {}
    control_rows = (root / "control.jsonl").open("x")
    handles.append(control_rows)

    def control(message):
        control_rows.write(message.data + "\n")
        control_rows.flush()

    def commands(topic, name):
        stream = (root / name).open("x")
        handles.append(stream)
        def write(m):
            stream.write(json.dumps(dict(received_steady=time.monotonic(), ros_time=rospy.Time.now().to_sec(),
                steering=m.steering, accel=m.accel, brake=m.brake)) + "\n")
        return rospy.Subscriber(topic, CtrlCmd, write, queue_size=100)

    subscriptions = [
        rospy.Subscriber('/planning/frenet_feedback_v2', FrenetFeedback,
                         lambda m: latest.update(normal_feedback=m), queue_size=1),
        commands("/ctrl_cmd", "wire.jsonl"),
        commands("/ctrl_cmd/normal", "normal-commands.jsonl"),
        commands("/ctrl_cmd/jamming", "mpc-commands.jsonl"),
        rospy.Subscriber('/planning/mode', UInt8,
            lambda m: emit(dict(event='mode', value=int(m.data))), queue_size=10),
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

    def start(cmd, name):
        f = (root / (name + ".log")).open("x")
        handles.append(f)
        proc = subprocess.Popen(
            cmd, stdout=f, stderr=subprocess.STDOUT, start_new_session=True
        )
        children.append(proc)
        return proc

    reason = "preparation_failed"
    started = None
    contacts = []
    travel = 0.0
    last = None
    speedmax = 0.0
    cleanup_errors = []
    mutated = False
    try:
        from scen_runner.runtime.private_master import require_udp_ports_free
        require_udp_ports_free([9099, 9798, 7505, 9092, 9291, 9293, 9295, 9297, 9299, 9301, 9303])
        start(["roslaunch", "asmc_bridge", "bridge.launch", "transport_profile:=local",
               "invert_steer:=" + args.bridge_invert_steer], "bridge")
        rospy.wait_for_message("/Ego_topic", __import__("morai_msgs.msg", fromlist=["EgoVehicleStatus"]).EgoVehicleStatus, timeout=15)
        rospy.wait_for_message("/imu", __import__("sensor_msgs.msg", fromlist=["Imu"]).Imu, timeout=10)
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
            if any(child.poll() is not None for child in children):
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
        link.set_environment("Sunny", 13)
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
            "/CollisionData",
            "/image_jpeg/compressed",
            "/cam_left/image_jpeg/compressed",
            "/cam_right/image_jpeg/compressed",
        ]
        start(
            [
                "rosbag",
                "record",
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
            local = start(
                [
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
                ],
                "local",
            )
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
            if local_ready or normal_ready: break
            if any(child.poll() is not None for child in children):
                raise RuntimeError('owned producer exited before control readiness')
            if time.monotonic() > ready_until:
                reason = 'input_or_plan_not_ready'
                raise RuntimeError('no fresh usable trajectory before enabling auto control')
            time.sleep(.05)
        r = VehicleControlModeParam()
        r.actor_info.CopyFrom(link.ego_info())
        r.mode = VehicleControlMode.VEHICLE_CONTROL_AUTO_MODE
        link.check(
            link._require()._actor_stub.SetVehicleControlMode(r, timeout=3),
            "enable native auto control",
        )
        link.release()
        started = time.monotonic()
        emit(dict(event='driving_started'))
        reason = "time_limit"
        stationary = started
        while time.monotonic() - started < args.seconds:
            if any(x.poll() is not None for x in children):
                reason = "child_exit"
                break
            if os.statvfs(root).f_bavail * os.statvfs(root).f_frsize < 2 * 2**30:
                reason = "disk_floor"
                break
            if (root / "stop-request.json").exists():
                reason = "external_stop_request"
                break
            state = actor_state(link.ego())
            resources.sample()
            native = [
                x for x in state.get("native_contacts", []) if x and x != link.ego_id
            ]
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
                )
            )
            if native:
                contacts += native
                reason = "native_contact"
                break
            if state["speed_kmh"] > 0.36:
                stationary = time.monotonic()
            if time.monotonic() - stationary > 12:
                reason = "stationary_12s"
                break
            if travel >= args.distance:
                reason = "distance_limit"
                break
            time.sleep(0.1)
    finally:
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
        emit(dict(event="finish", reason=reason, cleanup_errors=cleanup_errors))
        events.close()
        for f in handles:
            f.close()
        result = dict(
            run_id=run_id,
            reason=reason,
            native_contacts=contacts,
            travel_m=travel,
            maximum_speed_kph=speedmax,
            seconds=time.monotonic() - started if started else 0,
            cap_kph=args.cap,
            near_blind_assist=args.near_blind_assist,
            cleanup_errors=cleanup_errors,
            closed_loop_acceptance=False,
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
