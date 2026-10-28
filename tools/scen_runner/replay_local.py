#!/usr/bin/env python3
"""LOCAL-1 real ROS boundary replay on a private master; never replay control.

Run inside the built ROS workspace. Output must be a new child of a registered
run. Receive order is kept; no future IMU/GPS is preloaded into the estimator.
This tests the current C++ planner/MPC, not a Python planner approximation.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--seconds", type=float, default=0)
    parser.add_argument("--port", type=int, default=11348)
    parser.add_argument("--recovery", action="store_true")
    parser.add_argument("--near-blind-assist", action="store_true")
    args = parser.parse_args()
    args.output.mkdir(parents=False, exist_ok=False)
    probe = socket.socket()
    probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    probe.bind(("127.0.0.1", args.port))
    probe.close()
    os.environ["ROS_MASTER_URI"] = "http://127.0.0.1:%d" % args.port
    os.environ["ROS_HOSTNAME"] = "127.0.0.1"
    # Import only after selecting the isolated master.
    import rospy
    import rosbag
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String
    from asmc_msgs.msg import LocalTrajectory

    prefix = "/experimental/shadow/semantic_space_trial/"
    topics = [
        "/Ego_topic",
        "/imu",
        "/perception/lane/boundary_family",
        "/perception/lidar/tracks_local",
        prefix + "context",
        prefix + "coverage",
        prefix + "drivable_probability",
    ]
    children = []
    handles = []
    output = []
    counts = {}

    def start(command, name):
        log = (args.output / (name + ".log")).open("x")
        handles.append(log)
        p = subprocess.Popen(
            command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True
        )
        children.append(p)
        return p

    try:
        start(["roscore", "-p", str(args.port)], "master")
        import rosgraph

        master = rosgraph.Master("/local_replay_launcher")
        for attempt in range(80):
            try:
                master.getPid()
                break
            except Exception:
                time.sleep(0.1)
        else:
            raise RuntimeError("private master did not start")
        rospy.init_node("local_bag_source", disable_signals=True)
        rospy.set_param("/use_sim_time", True)
        rospy.set_param("/local_planner/speed_callerid", "/local_bag_source")
        clock = rospy.Publisher("/clock", Clock, queue_size=10, latch=True)
        pubs = {}
        last = 0.0
        begin = None
        with rosbag.Bag(str(args.bag)) as bag:
            for topic, m, receipt in bag.read_messages(topics=topics):
                if begin is None:
                    begin = receipt.to_sec()
                if topic not in pubs:
                    pubs[topic] = rospy.Publisher(
                        topic,
                        type(m),
                        queue_size=64 if topic in ("/imu", "/Ego_topic") else 8,
                    )
                if len(pubs) == len(topics):
                    break
        if len(pubs) != len(topics):
            raise RuntimeError("missing replay inputs: " + str(set(topics) - set(pubs)))
        clock.publish(Clock(rospy.Time.from_sec(begin)))
        start(
            [
                "roslaunch",
                "asmc_planner",
                "local.launch",
                "run_id:=" + args.run_id,
                "journal_path:=" + str(args.output / "decisions.jsonl"),
                "recovery:=" + str(args.recovery).lower(),
                "near_blind_assist:=" + str(args.near_blind_assist).lower(),
            ],
            "local",
        )
        control_file = (args.output / "control.jsonl").open("x")
        handles.append(control_file)

        def control(m):
            control_file.write(m.data + "\n")
            control_file.flush()

        sub = rospy.Subscriber(
            "/planning/local/control_trace", String, control, queue_size=100
        )
        time.sleep(2.0)
        # Publisher warmup cannot advance simulated time before the first input.
        started = time.monotonic()
        with rosbag.Bag(str(args.bag)) as bag:
            for topic, m, receipt in bag.read_messages(topics=topics):
                elapsed = receipt.to_sec() - begin
                if args.seconds and elapsed > args.seconds:
                    break
                wait = started + elapsed - time.monotonic()
                if wait > 0:
                    time.sleep(wait)
                clock.publish(Clock(receipt))
                pubs[topic].publish(m)
                last = receipt.to_sec()
                counts[topic] = counts.get(topic, 0) + 1
                if children[-1].poll() is not None:
                    raise RuntimeError("local launch exited")
        # Let callbacks drain. No old sensor is replayed as a fresh measurement.
        time.sleep(0.3)
        if any(
            name in ("/ctrl_cmd", "/ctrl_cmd/jamming")
            for name, _ in master.getSystemState()[0]
        ):
            raise RuntimeError("preview unexpectedly advertised a control command")
        (args.output / "input-summary.json").write_text(
            json.dumps(
                dict(
                    bag=str(args.bag),
                    counts=counts,
                    begin=begin,
                    end=last,
                    near_blind_assist=args.near_blind_assist,
                    command_publishers=0,
                    interpretation="recorded arrival order, real boundary replay; not closed-loop trajectory validation",
                ),
                indent=2,
            )
        )
    finally:
        for p in reversed(children):
            if p.poll() is None:
                os.killpg(p.pid, signal.SIGINT)
                try:
                    p.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(p.pid, signal.SIGTERM)
                    p.wait(timeout=10)
        for f in handles:
            f.close()
    from scen_runner.records.analyze_local import analyze

    print(json.dumps(analyze(args.output), ensure_ascii=False))


if __name__ == "__main__":
    main()
