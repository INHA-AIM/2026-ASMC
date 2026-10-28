#!/usr/bin/env python3
"""Real local MPC contract/ownership regression on a private ROS master only."""
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time


def main():
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        port = sock.getsockname()[1]
    os.environ["ROS_MASTER_URI"] = "http://127.0.0.1:%d" % port
    os.environ["ROS_HOSTNAME"] = "127.0.0.1"
    import rosgraph
    import rospy
    from nav_msgs.msg import Odometry
    from asmc_msgs.msg import LocalTrajectory, TrajectoryPoint
    from std_msgs.msg import String
    from morai_msgs.msg import CtrlCmd

    children = []
    with tempfile.TemporaryDirectory(prefix="local-boundary-") as folder:
        logs = []

        def start(cmd, name):
            f = open(Path(folder) / name, "w")
            logs.append(f)
            p = subprocess.Popen(
                cmd, stdout=f, stderr=subprocess.STDOUT, start_new_session=True
            )
            children.append(p)
            return p

        def stop(p):
            if p.poll() is None:
                os.killpg(p.pid, signal.SIGINT)
                try:
                    p.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(p.pid, signal.SIGTERM)
                    p.wait(timeout=5)

        try:
            start(["roscore", "-p", str(port)], "master.log")
            master = rosgraph.Master("/local_contract_test")
            for _ in range(50):
                try:
                    master.getPid()
                    break
                except Exception:
                    time.sleep(0.1)
            else:
                raise RuntimeError("private master unavailable")
            rospy.init_node("local_contract_test", disable_signals=True)
            import rospkg

            config = (
                rospkg.RosPack().get_path("asmc_controller") + "/config/controller.yaml"
            )
            preview = start(
                [
                    "rosrun",
                    "asmc_controller",
                    "asmc_local_controller_node",
                    "__name:=local_contract_controller",
                    "_enable_control:=false",
                    "_prediction_horizon:=49",
                    "_steering_change_cost:=25",
                    "_heading_cost:=6",
                    "_controller_config:=" + config,
                ],
                "controller.log",
            )
            states = rospy.Publisher("/planning/local/odometry", Odometry, queue_size=1)
            plans = rospy.Publisher(
                "/planning/local/trajectory", LocalTrajectory, queue_size=1
            )
            received = []
            sub = rospy.Subscriber(
                "/planning/local/control_trace",
                String,
                lambda m: received.append(json.loads(m.data)),
                queue_size=10,
            )
            time.sleep(1.0)
            seq = 0

            def case(name, mutate, expected):
                nonlocal seq
                begin = time.monotonic()
                seen = set()
                while time.monotonic() - begin < 0.8:
                    stamp = rospy.Time.now()
                    seq += 1
                    state = Odometry()
                    state.header.stamp = stamp
                    state.header.frame_id = "local/fixture/0"
                    state.child_frame_id = "base_link"
                    state.pose.pose.orientation.w = 1.0
                    state.twist.twist.linear.x = 1.0
                    plan = LocalTrajectory()
                    plan.header = state.header
                    plan.ego_at_plan = state
                    plan.scene_stamp = stamp
                    plan.run_id = "unit"
                    plan.valid = True
                    plan.reason = "ok"
                    plan.dt = 0.05
                    plan.plan_sequence = seq
                    for i in range(100):
                        p = TrajectoryPoint()
                        p.x = i * 0.05
                        p.velocity = 1.0
                        plan.points.append(p)
                    mutate(state, plan)
                    states.publish(state)
                    plans.publish(plan)
                    time.sleep(0.07)
                    seen.update(
                        r["reason"]
                        for r in received
                        if r.get("incoming_plan_sequence", r["plan_sequence"])
                        >= seq - 1
                    )
                    for row in received:
                        if row["reason"] in (
                            "motion_sample_pending",
                            "plan_replacement_pending",
                        ):
                            assert not row["command_published"]
                        if row["reason"] == "ok":
                            assert row["scene_age_s"] <= 0.35
                            assert row["motion_age_s"] <= 0.10
                    received.clear()
                for wanted in [expected] if isinstance(expected, str) else expected:
                    assert wanted in seen, (name, wanted, seen)
                print(name, expected, "PASS")

            case("exact epoch-scale stamp", lambda s, p: None, "ok")
            # This same solved path expires between planner ticks. It may wait
            # once without a command refresh, then must fail if no replacement.
            import copy

            state = Odometry()
            state.header.stamp = rospy.Time.now()
            state.header.frame_id = "local/fixture/0"
            state.child_frame_id = "base_link"
            state.pose.pose.orientation.w = 1.0
            state.twist.twist.linear.x = 2.0
            plan = LocalTrajectory()
            plan.header = copy.deepcopy(state.header)
            plan.ego_at_plan = copy.deepcopy(state)
            plan.scene_stamp = plan.header.stamp - rospy.Duration(0.17)
            plan.valid = True
            plan.reason = "ok"
            plan.dt = 0.05
            plan.plan_sequence = seq + 1
            for i in range(100):
                point = TrajectoryPoint()
                point.x = i * 0.1
                point.velocity = 2.0
                plan.points.append(point)
            received.clear()
            states.publish(state)
            plans.publish(plan)
            until = time.monotonic() + 0.45
            while time.monotonic() < until:
                state.header.stamp = rospy.Time.now()
                states.publish(state)
                time.sleep(0.015)
            reasons = [r["reason"] for r in received]
            assert (
                "ok" in reasons and 1 <= reasons.count("plan_replacement_pending") <= 3
            ), reasons
            assert "local_source_stale" in reasons, reasons
            assert any(
                r["reason"] == "plan_replacement_pending"
                and 0.20 < r["plan_age_s"] <= 0.25
                for r in received
            ), received
            assert not any(
                r["command_published"]
                for r in received
                if r["reason"] == "plan_replacement_pending"
            )
            print(
                "bounded source-replacement hold; sustained expiry fails",
                reasons,
                "PASS",
            )
            received.clear()
            seq = plan.plan_sequence
            case("recovery after source replacement timeout", lambda s, p: None, "ok")
            held = []

            def missing_new_state(s, p):
                import copy

                p.header = copy.deepcopy(p.header)
                p.ego_at_plan = copy.deepcopy(p.ego_at_plan)
                s.header = copy.deepcopy(s.header)
                if not held:
                    held.append(s.header.stamp)
                s.header.stamp = held[0]

            case(
                "one crawl cadence wait never refreshes command",
                missing_new_state,
                ["motion_sample_pending", "local_receive_stale"],
            )

            case(
                "old source cannot be renewed by fresh plan header",
                lambda s, p: setattr(
                    p, "scene_stamp", p.header.stamp - rospy.Duration(0.8)
                ),
                "local_source_stale",
            )

            def delayed_state(s, p, delay):
                import copy

                p.header = copy.deepcopy(p.header)
                p.ego_at_plan = copy.deepcopy(p.ego_at_plan)
                s.header.stamp -= rospy.Duration(delay)

            case(
                "80ms valid sensor cadence",
                lambda s, p: delayed_state(s, p, 0.08),
                "ok",
            )
            case(
                "single crawl tick waits; sustained120ms exceeds forecast",
                lambda s, p: delayed_state(s, p, 0.12),
                "control_motion_extrapolation_limit",
            )
            case(
                "reverse retains observations but no forward command",
                lambda s, p: setattr(s.twist.twist.linear, "x", -1.0),
                "reverse_control_not_supported",
            )

            def mismatch(s, p):
                # Separate header object: one nanosecond was enough to trigger
                # the former floating-point round-trip production bug.
                import copy

                p.ego_at_plan = copy.deepcopy(s)
                p.ego_at_plan.header.stamp = s.header.stamp + rospy.Duration(0, 1)

            case("one-nanosecond mismatch", mismatch, "local_contract_mismatch")
            case("recovery after invalid plan", lambda s, p: None, "ok")
            case(
                "invalid quaternion",
                lambda s, p: setattr(s.pose.pose.orientation, "w", 0.0),
                "nonfinite_motion",
            )
            case(
                "nonfinite path",
                lambda s, p: setattr(p.points[5], "x", float("nan")),
                "nonfinite_trajectory",
            )
            case(
                "unavailable scene",
                lambda s, p: (
                    setattr(p, "valid", False),
                    setattr(p, "reason", "scene_stale"),
                ),
                "scene_stale",
            )
            case(
                "solved crawl path",
                lambda s, p: setattr(p, "reason", "near_field_recovery"),
                "ok",
            )
            case(
                "brief gap then expiry",
                lambda s, p: (
                    setattr(p, "valid", False),
                    setattr(p, "reason", "scene_stale"),
                ),
                ["replan_grace_crawl", "scene_stale"],
            )
            case(
                "restore solved crawl",
                lambda s, p: setattr(p, "reason", "near_field_recovery"),
                "ok",
            )
            case(
                "no grace at faster speed",
                lambda s, p: (
                    setattr(s.twist.twist.linear, "x", 2.0),
                    setattr(p, "valid", False),
                    setattr(p, "reason", "scene_stale"),
                ),
                "scene_stale",
            )
            assert not any(
                n in ("/ctrl_cmd", "/ctrl_cmd/jamming")
                for n, _ in master.getSystemState()[0]
            )
            print("preview has no command publisher PASS")
            stop(preview)

            # A frozen private ROS clock reproduces the measured67us future
            # stamp deterministically; no probabilistic sleep-vs-callback race.
            from rosgraph_msgs.msg import Clock

            rospy.set_param("/use_sim_time", True)
            clock_pub = rospy.Publisher("/clock", Clock, queue_size=1, latch=True)
            clock_pub.publish(Clock(rospy.Time.from_sec(100.0)))
            clock_preview = start(
                [
                    "rosrun",
                    "asmc_controller",
                    "asmc_local_controller_node",
                    "__name:=local_clock_contract",
                    "_enable_control:=false",
                    "_controller_config:=" + config,
                ],
                "clock-contract.log",
            )
            time.sleep(0.8)

            # Wait for the new node's actual subscriptions, not a probabilistic
            # process-start sleep. The future-stamp phase duration stays fixed.
            deadline=time.monotonic()+5
            while True:
                subs=dict(master.getSystemState()[1])
                if all('/local_clock_contract' in subs.get(t,[]) for t in
                       ['/planning/local/odometry','/planning/local/trajectory','/clock']) and states.get_num_connections() and plans.get_num_connections() and clock_pub.get_num_connections():
                    break
                assert time.monotonic()<deadline,'clock contract node did not connect'
                time.sleep(.02)

            def clock_phase(t, lead, duration, mutate=lambda s, p: None):
                nonlocal seq
                seq += 1
                clock_pub.publish(Clock(rospy.Time.from_sec(t)))
                state = Odometry()
                state.header.stamp = rospy.Time.from_sec(t + lead)
                state.header.frame_id = "local/clock/0"
                state.child_frame_id = "base_link"
                state.pose.pose.orientation.w = 1.0
                state.twist.twist.linear.x = 1.0
                plan = LocalTrajectory()
                plan.header = copy.deepcopy(state.header)
                plan.header.stamp = rospy.Time.from_sec(t - 0.05)
                plan.ego_at_plan = copy.deepcopy(state)
                plan.ego_at_plan.header = copy.deepcopy(plan.header)
                plan.scene_stamp = rospy.Time.from_sec(t - 0.20)
                plan.valid = True
                plan.reason = "ok"
                plan.dt = 0.05
                plan.plan_sequence = seq
                for i in range(100):
                    point = TrajectoryPoint()
                    point.x, point.velocity = i * 0.05, 1.0
                    plan.points.append(point)
                mutate(state, plan)
                received.clear()
                states.publish(state)
                plans.publish(plan)
                time.sleep(duration)
                return [dict(r) for r in received]

            first_clock=clock_phase(100.0,-0.01,.10)
            assert any(r["reason"] == "ok" for r in first_clock), first_clock
            tiny_future = clock_phase(100.05, 0.000067, 0.08)
            assert any(
                r["reason"] == "motion_clock_pending" for r in tiny_future
            ), tiny_future
            assert not any(
                r["valid"] or r["command_published"] for r in tiny_future
            ), tiny_future
            recovered = clock_phase(100.10, -0.01, 0.08)
            assert any(r["reason"] == "ok" for r in recovered), recovered
            assert any(r.get("execution_seed_restarted") for r in recovered if r["valid"]), recovered
            assert all(r["execution_history_samples"] > 0 for r in recovered if r["valid"]), recovered
            prolonged = clock_phase(100.15, 0.000067, 0.30)
            assert any(
                r["reason"] in ("local_source_stale", "local_receive_stale")
                for r in prolonged
            ), prolonged
            clock_phase(100.20, -0.01, 0.10)
            large_future = clock_phase(100.25, 0.007, 0.08)
            assert any(
                r["reason"] == "local_source_stale" for r in large_future
            ), large_future
            assert not any(r["reason"] == "motion_clock_pending" for r in large_future)
            print(
                "67us clock handoff never computes future state; recovery/expiry/7ms rejection PASS"
            )
            clock_phase(100.30, -0.01, 0.10)
            bad_future = clock_phase(100.35, 0.000067, 0.08,
                                     lambda s, p: setattr(s.pose.pose.orientation, "w", 0.0))
            assert any(r["reason"] == "nonfinite_motion" for r in bad_future), bad_future
            assert not any(r["reason"] == "motion_clock_pending" for r in bad_future), bad_future
            print("pending handoff still rejects invalid geometry; resumed execution history updated PASS")
            stop(clock_preview)
            clock_pub.unregister()
            rospy.set_param("/use_sim_time", False)

            # Exercise the actual ownership check, including a NORMAL producer
            # whose commands do not share the local topic. These publishers are
            # isolated from the operating master and never send a command.
            for i, topic in enumerate(("/ctrl_cmd/normal", "/ctrl_cmd/jamming")):
                foreign = rospy.Publisher(topic, CtrlCmd, queue_size=1)
                for _ in range(30):
                    if any(t == topic for t, _ in master.getSystemState()[0]):
                        break
                    time.sleep(0.05)
                else:
                    raise RuntimeError("foreign publisher did not register")
                log_name = "conflict-%d.log" % i
                process = start(
                    [
                        "rosrun",
                        "asmc_controller",
                        "asmc_local_controller_node",
                        "__name:=local_conflict_%d" % i,
                        "_enable_control:=true",
                        "_controller_config:=" + config,
                    ],
                    log_name,
                )
                process.wait(timeout=8)
                log = (Path(folder) / log_name).read_text()
                assert "LOCAL control ownership conflict" in log, log
                assert topic + " owned by /local_contract_test" in log, log
                print("conflicting owner", topic, "PASS")
                foreign.unregister()

            # Our own JAMMING publisher and the single selector output are
            # allowed. No mode or valid input is sent, so this tests ownership
            # without requesting actuation even on the private master.
            router = rospy.Publisher("/ctrl_cmd", CtrlCmd, queue_size=1)
            exclusive = start(
                [
                    "rosrun",
                    "asmc_controller",
                    "asmc_local_controller_node",
                    "__name:=local_exclusive_contract",
                    "_enable_control:=true",
                    "_controller_config:=" + config,
                ],
                "exclusive.log",
            )
            deadline = time.monotonic() + 5
            registered = False
            while time.monotonic() < deadline:
                registered = any(
                    topic == "/ctrl_cmd/jamming"
                    and "/local_exclusive_contract" in owners
                    for topic, owners in master.getSystemState()[0]
                )
                if registered or exclusive.poll() is not None:
                    break
                time.sleep(0.05)
            assert registered and exclusive.poll() is None
            time.sleep(1.2)  # Cover the recurring check as well as startup.
            assert exclusive.poll() is None, (
                Path(folder) / "exclusive.log"
            ).read_text()
            late = rospy.Publisher("/ctrl_cmd/normal", CtrlCmd, queue_size=1)
            exclusive.wait(timeout=8)
            log = (Path(folder) / "exclusive.log").read_text()
            assert "/ctrl_cmd/normal owned by /local_contract_test" in log, log
            late.unregister()
            router.unregister()
            print("exclusive local owner and selector output PASS")
            print("late NORMAL owner rejected PASS")
        finally:
            for p in reversed(children):
                stop(p)
            for f in logs:
                f.close()


if __name__ == "__main__":
    main()
