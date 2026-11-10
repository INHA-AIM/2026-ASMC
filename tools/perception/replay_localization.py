#!/usr/bin/env python3
"""Replay recorded perception into an isolated, control-disabled ROS stack.

Use a dedicated network-none container. Never connects to MORAI or the existing
ROS master; only owned child process groups are stopped. A registered D run and
the identical run ID are mandatory. Original GPS becomes invalid after the
chosen duration; GT fields are not used as localization inputs.
"""
import argparse
from collections import Counter
import json
import hashlib
import os
from pathlib import Path
import signal
import socket
import subprocess
import time


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("bag", type=Path)
    p.add_argument("output", type=Path)
    p.add_argument("--run-id", required=True)
    p.add_argument("--run-manifest", type=Path, required=True)
    p.add_argument("--config", type=Path, required=True)
    p.add_argument("--storage-root", type=Path, default=Path("/storage"))
    p.add_argument("--gps-blackout-after", type=float, default=3.)
    p.add_argument("--near-blind-assist", action="store_true")
    a = p.parse_args()
    manifest = json.loads(a.run_manifest.read_text())
    configuration = json.loads(a.config.read_text())
    assert hashlib.sha256(a.config.read_bytes()).hexdigest() == manifest["config_sha256"]
    assert manifest["run_id"] == a.run_id and manifest["status"] == "active"
    assert a.output.resolve() == (a.storage_root / manifest["output"]).resolve()
    assert a.output.name == a.run_id and a.gps_blackout_after > 0
    assert a.near_blind_assist == configuration.get("near_blind_assist", False), "assist must match registered config"
    assert a.gps_blackout_after == configuration["gps_blackout_after_s"]
    assert any(x["root"] == "storage" and (a.storage_root / x["path"]).resolve() == a.bag.resolve()
               for x in manifest["inputs"]), "bag not registered as input"
    a.output.mkdir(exist_ok=False)
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        port = sock.getsockname()[1]
    os.environ.update(ROS_MASTER_URI=f"http://127.0.0.1:{port}",
                      ROS_HOSTNAME="127.0.0.1", ROS_LOG_DIR=str(a.output / "roslog"))
    import rosbag
    import rosgraph
    import rospy
    from rosgraph_msgs.msg import Clock
    from asmc_msgs.msg import LocalTrajectory
    from std_msgs.msg import String
    topics = ["/Ego_topic", "/imu", "/gps", "/perception/lidar/tracks_local",
              "/perception/lane/boundary_family", "/experimental/shadow/semantic_space_trial/context",
              "/experimental/shadow/semantic_space_trial/coverage",
              "/experimental/shadow/semantic_space_trial/drivable_probability"]
    counts, reasons, phases, sources = Counter(), Counter(), Counter(), Counter()
    children, logs = [], []
    trajectories = a.output / "trajectories.jsonl"
    decision_path = a.output / "decisions.jsonl"
    valid = 0
    def start(cmd, name):
        f = (a.output / name).open("w"); logs.append(f)
        child = subprocess.Popen(cmd, stdout=f, stderr=subprocess.STDOUT, start_new_session=True)
        children.append(child)
        return child
    def stop(child):
        if child.poll() is None:
            os.killpg(child.pid, signal.SIGINT)
            try: child.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGTERM); child.wait(timeout=5)
    with trajectories.open("w") as tf, decision_path.open("w") as df:
        def trajectory(m):
            nonlocal valid
            valid += int(m.valid); reasons[m.reason] += 1
            tf.write(json.dumps(dict(stamp_ns=m.header.stamp.to_nsec(), valid=m.valid,
                                    reason=m.reason, points=len(m.points))) + "\n")
        def decision(m):
            d = json.loads(m.data); join = d.get("input_join", {})
            phases[join.get("shadow_localization_phase", "unavailable")] += 1
            sources[join.get("shadow_localization_source", "unavailable")] += 1
            df.write(m.data + "\n")
        try:
            start(["roscore", "-p", str(port)], "master.log")
            master = rosgraph.Master("/shadow_replay")
            for _ in range(100):
                try: master.getPid(); break
                except Exception: time.sleep(.1)
            else: raise RuntimeError("private master unavailable")
            master.setParam("/use_sim_time", True)
            rospy.init_node("shadow_replay", disable_signals=True)
            clock = rospy.Publisher("/clock", Clock, queue_size=10, latch=True)
            with rosbag.Bag(str(a.bag)) as bag:
                first = bag.get_start_time()
                clock.publish(Clock(rospy.Time.from_sec(first)))
                pubs = {}
                for topic, msg, _ in bag.read_messages(topics=topics):
                    if topic not in pubs: pubs[topic] = rospy.Publisher(topic, type(msg), queue_size=100)
                    if len(pubs) == len(topics): break
                assert len(pubs) == len(topics), "recorded perception inputs incomplete"
                rospy.Subscriber("/planning/localization/trajectory", LocalTrajectory, trajectory, queue_size=200)
                rospy.Subscriber("/planning/localization/trace", String, decision, queue_size=200)
                stack = start(["roslaunch", "asmc_planner", "shadow_localization_static.launch",
                               "enable_control:=false", f"run_id:={a.run_id}",
                               f"near_blind_assist:={str(a.near_blind_assist).lower()}",
                               "speed_callerid:=/shadow_replay"], "stack.log")
                for _ in range(150):
                    if stack.poll() is not None: raise RuntimeError("stack exited before replay")
                    if all(pub.get_num_connections() for pub in pubs.values()): break
                    time.sleep(.1)
                else: raise RuntimeError("recorded topic subscribers unavailable: " +
                                         str([t for t,pub in pubs.items() if not pub.get_num_connections()]))
                wall = time.monotonic()
                for topic, msg, receipt in bag.read_messages(topics=topics):
                    remaining = receipt.to_sec() - first - (time.monotonic() - wall)
                    if remaining > 0: time.sleep(remaining)
                    clock.publish(Clock(receipt))
                    if topic == "/gps" and receipt.to_sec() - first >= a.gps_blackout_after:
                        msg.status = 0; msg.latitude = msg.longitude = 0.; counts["masked_gps"] += 1
                    pubs[topic].publish(msg); counts[topic] += 1
                time.sleep(1.)
                commands = [t for t, nodes in master.getSystemState()[0] if t.startswith("/ctrl_cmd") and nodes]
                assert not commands, f"unexpected command publisher: {commands}"
        finally:
            for child in reversed(children): stop(child)
            for f in logs: f.close()
    summary = dict(run_id=a.run_id, input=str(a.bag), input_counts=counts,
                   trajectory_reasons=reasons, valid_trajectories=valid,
                   phases=phases, pose_sources=sources,
                   gps_blackout_after_s=a.gps_blackout_after,
                   near_blind_assist=a.near_blind_assist,
                   verdict="paths_available" if valid else "held_by_validation",
                   scope="recorded open-loop perception; no live simulator or closed-loop vehicle motion",
                   command_publishers=0)
    (a.output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))
    if not counts["masked_gps"] or not sum(sources.values()):
        raise RuntimeError("GPS-masked planner decisions were not exercised")


if __name__ == "__main__":
    main()
