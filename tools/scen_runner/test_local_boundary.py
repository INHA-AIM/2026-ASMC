#!/usr/bin/env python3
"""Real local MPC boundary regression on a private master; no control publisher."""
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time


def main():
    port = 11349
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", port))
    os.environ["ROS_MASTER_URI"] = "http://127.0.0.1:%d" % port
    os.environ["ROS_HOSTNAME"] = "127.0.0.1"
    import rosgraph
    import rospy
    from nav_msgs.msg import Odometry
    from asmc_msgs.msg import LocalTrajectory, TrajectoryPoint
    from std_msgs.msg import String

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
            start(
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
                    received.clear()
                for wanted in [expected] if isinstance(expected, str) else expected:
                    assert wanted in seen, (name, wanted, seen)
                print(name, expected, "PASS")

            case("exact epoch-scale stamp", lambda s, p: None, "ok")

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
        finally:
            for p in reversed(children):
                if p.poll() is None:
                    os.killpg(p.pid, signal.SIGINT)
                    try:
                        p.wait(timeout=8)
                    except subprocess.TimeoutExpired:
                        os.killpg(p.pid, signal.SIGTERM)
                        p.wait(timeout=5)
            for f in logs:
                f.close()


if __name__ == "__main__":
    main()
