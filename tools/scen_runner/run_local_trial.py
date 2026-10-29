#!/usr/bin/env python3
"""One bounded LOCAL-1 MORAI trial using scen_runner's owned replay fixture.

Explicit opt-in simulation control. Existing bridge/camera/LiDAR are reused;
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
    p.add_argument("--cap", type=float, default=5)
    p.add_argument("--near-blind-assist", action="store_true")
    p.add_argument("--empty", action="store_true")
    p.add_argument("--stack", choices=("local", "localization"), default="local")
    p.add_argument("--distance", type=float, default=35.)
    p.add_argument("--start-lateral-m", type=float, default=0.)
    p.add_argument("--start-heading-deg", type=float, default=0.)
    p.add_argument("--lease-directory", type=Path)
    p.add_argument("--runtime-root", type=Path, help="private executable bundle in the existing ROS network namespace")
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
    if not 0 < args.seconds <= (180 if args.stack=="localization" else 60) or not 0 < args.cap <= 25 or not 0 < args.distance <= 170:
        raise ValueError("bounded duration/cap required")
    if args.stack=="localization" and (not args.empty or args.cap>15 or args.near_blind_assist):
        raise ValueError("position baseline requires empty scene, cap<=15 and no blind assist")
    if args.runtime_root and args.stack!="localization":raise ValueError("runtime bundle is position-baseline only")
    if args.stack=="localization" and not args.lease_directory:
        raise ValueError("position trial requires the shared physical lease directory")
    if args.stack!="localization" and (args.start_lateral_m or args.start_heading_deg):
        raise ValueError("start offsets are position-baseline only")
    if args.near_blind_assist and args.cap > 5:
        raise ValueError("unvalidated blind assist requires <=5km/h")
    raw = json.loads(args.scene.read_text())
    definition = raw["replay"]["tunnel"]
    import rospy
    import rosgraph
    from std_msgs.msg import UInt8, String
    from asmc_msgs.msg import LocalTrajectory
    from morai_msgs.msg import CtrlCmd
    from google.protobuf.json_format import MessageToDict
    from scen_runner.config import load_config
    from scen_runner.runtime.validation_episode import EpisodeLink
    from scen_runner.runtime.validation_lease import ValidationLease
    from scen_runner.records.collision_archive import actor_state, load_collision
    from scen_runner.records.collision_replay import ReplaySection
    from scen_runner.runtime.signal_trial_fixture import StaticFixture

    rospy.init_node("localization_trial_"+root.name.replace("-","_") if args.stack=="localization" else "local_trial_owner", disable_signals=True)
    master = rosgraph.Master(rospy.get_name())
    forbidden = {"/ctrl_cmd", "/ctrl_cmd/normal", "/ctrl_cmd/jamming", "/planning/mode"}
    conflicts = {
        name: nodes for name, nodes in master.getSystemState()[0] if name in forbidden
    }
    if conflicts:
        raise RuntimeError("existing control/mode owner: " + str(conflicts))
    from rosnode import get_node_names

    owned_names=({"/local_planner", "/local_controller"} if args.stack=="local" else
                 {"/localization", "/localization_controller"})
    if owned_names & set(get_node_names()):
        raise RuntimeError("local preview already running")
    topic_prefix="/planning/local" if args.stack=="local" else "/planning/localization"
    run_id = root.name
    workspace = args.runtime_root or Path(__file__).resolve().parents[2]
    executables=["asmc_planner/asmc_control_selector_node","asmc_planner/asmc_local_planner_node",
                 "asmc_controller/asmc_local_controller_node"]
    if args.stack=="localization":executables.append("asmc_planner/asmc_shadow_localization_node")
    missing=[name for name in executables if not os.access(workspace/"devel/lib"/name,os.X_OK)]
    if missing:raise RuntimeError("missing runtime executables before simulator mutation: "+str(missing))
    files = [
        workspace / name
        for name in (
            "devel/lib/asmc_planner/asmc_local_planner_node",
            "devel/lib/asmc_controller/asmc_local_controller_node",
            "src/asmc_planner/src/local/node.cpp",
            "src/asmc_planner/src/local/planner.cpp",
            "src/asmc_controller/src/local/node.cpp",
            "src/asmc_planner/launch/local.launch",
            "src/asmc_planner/config/local_planner.yaml",
            "src/asmc_controller/config/controller.yaml",
            "src/asmc_controller/config/local_controller.yaml",
        )
    ]
    if args.stack=="localization":
        files += [workspace/name for name in (
            "devel/lib/asmc_planner/asmc_shadow_localization_node",
            "devel/lib/libasmc_shadow_localization_core.so",
            "src/asmc_planner/src/shadow_localization/position_node.cpp",
            "src/asmc_planner/src/shadow_localization/position_reference.cpp",
            "src/asmc_planner/src/shadow_localization/sampled_motion.hpp",
            "src/asmc_planner/src/shadow_localization/anchor.hpp",
            "src/asmc_planner/src/shadow_localization/input_alignment.hpp",
            "src/asmc_planner/src/shadow_localization/input_epoch.hpp",
            "tools/scen_runner/localization_trial_policy.py",
            "tools/scen_runner/run_local_trial.py",
            "src/asmc_planner/config/shadow_localization.yaml",
            "src/asmc_planner/launch/shadow_localization.launch")]
    (root / "runtime-provenance.json").write_text(
        json.dumps(
            dict(
                options=dict(vars(args), root=str(root), scene=str(args.scene), lease_directory=str(args.lease_directory) if args.lease_directory else None, runtime_root=str(args.runtime_root) if args.runtime_root else None),
                sha256={
                    str(f.relative_to(workspace)): hashlib.sha256(
                        f.read_bytes()
                    ).hexdigest()
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
    lease = ValidationLease(link.host, link.port, directory=args.lease_directory)
    lease.acquire()
    # Recheck under the common lease before any simulator mutation. Another
    # owner may have started between the initial ROS audit and lease acquisition.
    conflicts={name:nodes for name,nodes in master.getSystemState()[0] if name in forbidden and nodes}
    if conflicts or owned_names & set(get_node_names()):
        lease.release();events.close()
        raise RuntimeError("control owner appeared during preflight: "+str(conflicts))
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
    if args.stack=="localization":
        from localization_trial_policy import offset_start
        definition=dict(definition,start_pose=offset_start(definition['start_pose'],args.start_lateral_m,args.start_heading_deg))
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
    receipt_rows=(root/"command-receipts.jsonl").open("x")
    handles.append(receipt_rows)
    def receipt(message):
        receipt_rows.write(json.dumps(dict(monotonic=time.monotonic(),ros_time=rospy.Time.now().to_sec()))+"\n")
        receipt_rows.flush()

    def control(message):
        control_rows.write(message.data + "\n")
        control_rows.flush()

    def position_trace(message):
        latest.update(trace=json.loads(message.data),trace_received=time.monotonic())

    subscriptions = [
        rospy.Subscriber("/ctrl_cmd",CtrlCmd,receipt,queue_size=100,tcp_nodelay=True),
        rospy.Subscriber(
            topic_prefix+"/control_trace", String, control, queue_size=100
        ),
        rospy.Subscriber(
            topic_prefix+"/trajectory",
            LocalTrajectory,
            lambda m: latest.update(plan=m, receive=time.monotonic()),
            queue_size=1,
        ),
    ]
    if args.stack=="localization":
        subscriptions.append(rospy.Subscriber(topic_prefix+"/trace",String,position_trace,queue_size=1,tcp_nodelay=True))

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
    try:
        missing = set(args.park_prop) - {s.actor_info.id.value for s in states}
        if missing:
            raise RuntimeError("fixture IDs changed before trial: " + str(missing))
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
            "/ctrl_cmd",
            "/ctrl_cmd/jamming",
            "/CollisionData",
            "/image_jpeg/compressed",
            "/cam_left/image_jpeg/compressed",
            "/cam_right/image_jpeg/compressed",
        ]
        if args.stack!="local":
            topics+= [topic_prefix+name for name in ("/motion","/trajectory","/path","/trace","/control_trace")]
            topics+= ["/vehicle/state"]
        critical=({"/ctrl_cmd","/ctrl_cmd/jamming","/planning/mode","/Ego_topic","/imu","/gps",
                   topic_prefix+"/motion",topic_prefix+"/trace",topic_prefix+"/control_trace"}
                  if args.stack=="localization" else set())
        bulk_topics=[t for t in topics if t not in critical]
        start(
            [
                "rosbag",
                "record",
                "--split",
                "--duration=60",
                "--buffsize=128",
                *(["--tcpnodelay"] if args.stack=="localization" else []),
                "-O",
                str(root / "sensors"),
                *bulk_topics,
            ],
            "record",
        )
        if critical:
            # Each topic is recorded exactly once. Keep small timing-critical
            # messages out of the high-bandwidth recorder's callback queue.
            start(["rosbag","record","--split","--duration=60","--buffsize=16","--tcpnodelay",
                   "-O",str(root/"control"),*sorted(critical)],"record_control")
        # Start only the final command router, never autonomy/map planning or
        # the old BBox lane follower. Local MPC owns its historical JAMMING slot.
        selector_args=[
                "rosrun",
                "asmc_planner",
                "asmc_control_selector_node",
                "__name:=local_trial_selector",
            ]
        if args.runtime_root:selector_args=[str(workspace/"devel/lib/asmc_planner/asmc_control_selector_node"),"__name:=local_trial_selector"]
        selector = start(selector_args,"selector")
        launch_args=[
                "roslaunch",
                "asmc_planner",
                "local.launch" if args.stack=="local" else "shadow_localization.launch",
                "enable_control:=true",
                "run_id:=" + run_id,
                "journal_path:=" + str(root / "decisions.jsonl"),
                "speed_cap_kph:=" + str(args.cap),
            ]
        if args.stack=="local":launch_args.append("near_blind_assist:="+str(args.near_blind_assist).lower())
        elif any(t=="/planning/local/odometry" and nodes for t,nodes in master.getSystemState()[0]):
            launch_args += ["start_motion:=false","state_topic:=/planning/local/odometry"]
        if args.runtime_root:
            import yaml
            state_topic="/planning/localization/motion"
            if any(t=="/planning/local/odometry" and nodes for t,nodes in master.getSystemState()[0]):
                state_topic="/planning/local/odometry"
            else:
                start([str(workspace/"devel/lib/asmc_planner/asmc_local_planner_node"),
                       "__name:=localization_motion","_odometry_only:=true","_run_id:="+run_id,
                       "/planning/local/odometry:="+state_topic],"motion")
            params=yaml.safe_load((workspace/"src/planning_inputs/config/vehicle_state.yaml").read_text())
            params.update(yaml.safe_load((workspace/"src/asmc_planner/config/shadow_localization.yaml").read_text()))
            params.update(parameters=str(workspace/"src/asmc_planner/config/frenet/params.yaml"),run_id=run_id,
                          state_topic=state_topic,journal_path=str(root/"decisions.jsonl"),speed_cap_kph=args.cap)
            rospy.set_param("/localization",params)
            local=start([str(workspace/"devel/lib/asmc_planner/asmc_shadow_localization_node"),"__name:=localization"],"position")
            params=yaml.safe_load((workspace/"src/asmc_controller/config/local_controller.yaml").read_text())
            params.update(enable_control=True,controller_config=str(workspace/"src/asmc_controller/config/controller.yaml"))
            rospy.set_param("/localization_controller",params)
            start([str(workspace/"devel/lib/asmc_controller/asmc_local_controller_node"),"__name:=localization_controller",
                   "/planning/local/odometry:="+state_topic,"/planning/local/trajectory:=/planning/localization/trajectory",
                   "/planning/local/control_trace:=/planning/localization/control_trace",
                   "/planning/local/control_diagnostics:=/planning/localization/control_diagnostics"],"controller")
        else:local = start(launch_args,"local")
        mode = rospy.Publisher("/planning/mode", UInt8, queue_size=1, latch=True)
        mode.publish(UInt8(1))
        # Sensor orientation is stale while paused. Settle with P before D.
        link.gear_ego(False)
        link.pause_ego(False)
        if args.stack=="localization":
            from localization_trial_policy import ready
            deadline=time.monotonic()+10
            while True:
                plan=latest.get('plan');now=time.monotonic()
                if ready(now,bool(plan and plan.valid),latest.get('receive',-1e9),
                         latest.get('trace',{}),latest.get('trace_received',-1e9)):
                    emit(dict(event='position_ready',trace=latest['trace']));break
                if now>=deadline:
                    reason='position_readiness_timeout';raise RuntimeError(reason)
                if any(x.poll() is not None for x in children):
                    reason='child_exit_before_departure';raise RuntimeError(reason)
                time.sleep(.05)
        else:
            time.sleep(3)
        r = VehicleControlModeParam()
        r.actor_info.CopyFrom(link.ego_info())
        r.mode = VehicleControlMode.VEHICLE_CONTROL_AUTO_MODE
        link.check(
            link._require()._actor_stub.SetVehicleControlMode(r, timeout=3),
            "enable native auto control",
        )
        link.release()
        started = time.monotonic()
        reason = "time_limit"
        stationary = started
        while time.monotonic() - started < args.seconds:
            if any(x.poll() is not None for x in children):
                reason = "child_exit"
                break
            if os.statvfs(root).f_bavail * os.statvfs(root).f_frsize < (50 if args.stack=="localization" else 2) * 2**30:
                reason = "disk_floor"
                break
            if (root / "stop-request.json").exists():
                reason = "external_stop_request"
                break
            state = actor_state(link.ego())
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
            if args.stack=="localization" and state['speed_kmh']>args.cap+1:
                reason='speed_overshoot';break
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
        for subscription in subscriptions:subscription.unregister()
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

    if args.stack=="local" and (root / "decisions.jsonl").exists():
        print(json.dumps(analyze(root)), flush=True)
    if cleanup_errors:
        raise RuntimeError("incomplete restoration: " + str(cleanup_errors))


if __name__ == "__main__":
    main()
