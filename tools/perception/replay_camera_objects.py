#!/usr/bin/env python3
"""Measure sequential old/new ROS bag replays in an isolated ROS master.

Start a dedicated roscore and source outer then nested workspaces first.
This tool starts actual launch files, waits for warmed subscriptions, and
stops only its own process groups. Each path runs twice by default.
"""
import argparse
import json
import os
import re
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

from analyze_fusion_identity import compare_stable_observations

import numpy as np
import rosgraph
import rospy
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import CompressedImage
from asmc_msgs.msg import CameraObjectBoxArray
from Msg.msg import Yolo_bbox_arr, TrackedObjectArray


def stop(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=12)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5)


def wait_for(predicate, description, timeout=45):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.05)
    raise RuntimeError("timed out waiting for " + description)


def percentile(values):
    return {"p50_ms": float(np.percentile(values, 50)), "p95_ms": float(np.percentile(values, 95))}


def replay(args, path, repeat):
    prefix = args.output / (path + "_" + str(repeat))
    processes, files, subscriptions = [], [], []
    lock = threading.Lock()
    inputs, outputs, diagnostics, candidates, tracks, graph_samples = [], [], [], [], [], []
    receiving = threading.Event()

    def launch(command, suffix):
        stream = prefix.with_suffix("." + suffix + ".log").open("w")
        files.append(stream)
        process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(process)
        return process

    def graph():
        return rosgraph.Master("/p2_replay_graph").getSystemState()

    def record_input(message):
        if receiving.is_set():
            with lock:
                inputs.append({"stamp_ns": message.header.stamp.to_nsec(), "arrival": time.monotonic()})

    def record_output(message):
        if receiving.is_set():
            with lock:
                outputs.append({"stamp_ns": message.header.stamp.to_nsec(), "frame_id": message.header.frame_id,
                                "arrival": time.monotonic(), "latency_ms": (rospy.Time.now() - message.header.stamp).to_sec() * 1000,
                                "objc": message.objc,
                                "boxes": [[b.classes, b.xMin, b.yMin, b.xMax, b.yMax] for b in message.yolo_bbox_arr]})

    def record_candidate(message):
        if receiving.is_set():
            with lock:
                candidates.append({"stamp_ns": message.header.stamp.to_nsec(), "valid": message.valid})

    def record_track(message):
        if receiving.is_set():
            with lock:
                tracks.append({"stamp_ns": message.header.stamp.to_nsec(),
                               "objects": [[o.id, o.classification, o.yolo_class] for o in message.objects],
                               "centers": {str(o.id): [o.cluster_center.x, o.cluster_center.y, o.cluster_center.z]
                                           for o in message.objects}})

    def record_diagnostic(message):
        if receiving.is_set():
            with lock:
                diagnostics.append([{ "level": s.level, "message": s.message,
                                      "values": {v.key: v.value for v in s.values}} for s in message.status])

    try:
        rospy.set_param("/use_sim_time", True)
        subscriptions = [rospy.Subscriber("/image_jpeg/compressed", CompressedImage, record_input, queue_size=20, buff_size=2**24),
                         rospy.Subscriber("/YOLO/BBox", Yolo_bbox_arr, record_output, queue_size=100),
                         rospy.Subscriber("/perception/camera/objects", CameraObjectBoxArray, record_candidate, queue_size=100),
                         rospy.Subscriber("/perception/camera/objects/diagnostics", DiagnosticArray, record_diagnostic, queue_size=100)]
        if args.fusion:
            subscriptions.append(rospy.Subscriber("/lidar/tracked_objects_map", TrackedObjectArray, record_track, queue_size=100))
            command = ["roslaunch", "Main", "main.launch", "start_yolo:=" + ("true" if path == "old" else "false"),
                       "rviz:=false", "start_velodyne_driver:=false"]
            if args.fusion_trace:
                command.append("fusion_trace_path:=" + str(prefix.with_suffix(".fusion.jsonl").resolve()))
            launch(command, "main")
        elif path == "old":
            launch(["rosrun", "Main", "yolo_detection_node", "__name:=yolo_detection_node",
                    "/camera/image/compressed:=/image_jpeg/compressed"], "legacy")
        else:
            launch(["rosrun", "Main", "camera_objects_yolo_bbox_adapter.py"], "adapter")
        if path == "new":
            launch(["roslaunch", "asmc_camera", "camera_objects.launch"], "producer")
        def ready():
            pubs, subs, _ = graph()
            producers = dict(pubs).get("/YOLO/BBox", [])
            image = "/camera/image/compressed" if args.fusion and path == "old" else "/image_jpeg/compressed"
            consumers = dict(subs).get(image, [])
            expected = "/yolo_detection_node" if path == "old" else "/camera_object_detector"
            return len(producers) == 1 and expected in consumers
        wait_for(ready, path + " warmed detector and adapter")
        time.sleep(0.5)
        receiving.set()
        command = ["rosbag", "play", "--clock", "--quiet", "--delay=1.0", "--rate=1.0", str(args.bag)]
        if args.play_duration:
            command += ["--duration=" + str(args.play_duration)]
        if not args.fusion:
            command += ["--topics", "/image_jpeg/compressed"]
        playback = launch(command, "bag")
        if args.acceptance and path == "new":
            wait_for(lambda: len(outputs) >= 20, "live object frames before runtime acceptance")
            with prefix.with_suffix(".acceptance.log").open("w") as log:
                accepted = subprocess.run([sys.executable, str(Path(__file__).resolve().parents[2] / "scripts/verify_perception_runtime.py"),
                                           "--camera-objects", "--no-camera-lane", "--duration", "5",
                                           "--yaml-out", str(prefix.with_suffix(".acceptance.yaml"))],
                                          stdout=log, stderr=subprocess.STDOUT, timeout=15)
            if accepted.returncode:
                raise RuntimeError("object-only runtime acceptance failed; see acceptance log")
        deadline = time.monotonic() + args.timeout
        while playback.poll() is None:
            if time.monotonic() > deadline:
                raise RuntimeError("bag replay timeout")
            if any(p.poll() is not None for p in processes[:-1]):
                raise RuntimeError("inference/launch process exited during replay")
            pubs, _, _ = graph()
            graph_samples.append(dict(pubs).get("/YOLO/BBox", []))
            time.sleep(0.5)
        if playback.returncode != 0:
            raise RuntimeError("rosbag play failed")
        time.sleep(0.5)
        receiving.clear()
    finally:
        for process in reversed(processes):
            stop(process)
        for subscription in subscriptions:
            subscription.unregister()
        for stream in files:
            stream.close()
    source = [row["stamp_ns"] for row in outputs]
    elapsed = (outputs[-1]["arrival"] - outputs[0]["arrival"]) if len(outputs) > 1 else 0
    rate = (len(outputs) - 1) / elapsed if elapsed else 0
    latency = [row["latency_ms"] for row in outputs]
    logs = "\n".join(p.read_text() for p in args.output.glob(prefix.name + ".*.log"))
    sync = re.findall(r"Fusion sync 5s: matched=(\d+) dropped=(\d+) offset median/p95/max=([\d.]+)/([\d.]+)/([\d.]+) ms", logs)
    totals = re.findall(r"Fusion sync total: matched=(\d+) dropped=(\d+) pending lidar/yolo=(\d+)/(\d+)", logs)
    result = {"path": path, "repeat": repeat, "bag": str(args.bag), "fusion": args.fusion,
              "input_frames": len(inputs), "output_frames": len(outputs), "rate_hz": rate,
              "coverage": len(set(source)) / len(inputs) if inputs else 0,
              "duplicate_stamps": len(source) - len(set(source)),
              "reverse_stamps": sum(b < a for a, b in zip(source, source[1:])),
              "zero_stamps": source.count(0), "frame_ids": sorted({o["frame_id"] for o in outputs}),
              "single_bbox_publisher": bool(graph_samples) and all(len(names) == 1 for names in graph_samples),
              "latency": percentile(latency) if latency else {},
              "candidate_invalid_frames": sum(not c["valid"] for c in candidates),
              "diagnostic_warnings": sum(s["level"] > 0 for d in diagnostics for s in d),
              "last_diagnostics": diagnostics[-1] if diagnostics else [],
              "fusion_intervals": [{"matched": int(m), "dropped": int(d), "median_ms": float(med),
                                    "p95_ms": float(p95), "max_ms": float(maximum)} for m, d, med, p95, maximum in sync],
              "fusion_matched_reported": sum(int(row[0]) for row in sync),
              "fusion_totals": {"matched": int(totals[-1][0]), "dropped": int(totals[-1][1]),
                                "pending_lidar": int(totals[-1][2]), "pending_yolo": int(totals[-1][3])}
                               if totals else None,
              "fusion_offset_p95_max_ms": max((float(row[3]) for row in sync), default=None),
              "track_frames": len(tracks)}
    result["pass"] = (bool(outputs) and result["single_bbox_publisher"] and not result["duplicate_stamps"]
                      and not result["reverse_stamps"] and not result["zero_stamps"]
                      and result["frame_ids"] == ["camera_front"] and not result["candidate_invalid_frames"])
    if path == "new" and not args.fusion:
        result["pass"] &= rate >= 18.0
    if args.fusion:
        result["pass"] &= (bool(sync) and bool(tracks) and len(totals) == 1
                           and result["fusion_offset_p95_max_ms"] <= 50.0)
    prefix.with_suffix(".json").write_text(json.dumps(result, indent=2) + "\n")
    prefix.with_suffix(".frames.json").write_text(json.dumps({"outputs": outputs, "tracks": tracks}))
    print(json.dumps(result, indent=2), flush=True)
    return result


def compare_tracks(reference, candidate):
    old = {(row["stamp_ns"], o[0]): tuple(o[1:]) for row in reference for o in row["objects"]}
    new = {(row["stamp_ns"], o[0]): tuple(o[1:]) for row in candidate for o in row["objects"]}
    common = old.keys() & new.keys()
    mismatches = [{"stamp_ns": key[0], "track": key[1], "old": old[key], "new": new[key]}
                  for key in sorted(common) if old[key] != new[key]]
    return {"common": len(common), "mismatches": mismatches}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--fusion", action="store_true")
    parser.add_argument("--fusion-trace", action="store_true", help="write complete Main JSONL observations")
    parser.add_argument("--repeats", type=int, default=2)
    parser.add_argument("--timeout", type=float, default=240)
    parser.add_argument("--path", choices=("both", "old", "new"), default="both")
    parser.add_argument("--play-duration", type=float, help="short replay for a runtime checker smoke test")
    parser.add_argument("--acceptance", action="store_true", help="also run object-only runtime acceptance during new-path replay")
    args = parser.parse_args()
    if args.fusion and args.path != "both":
        raise SystemExit("fusion comparison requires --path both")
    if args.fusion_trace and not args.fusion:
        raise SystemExit("--fusion-trace requires --fusion")
    if args.fusion and args.repeats != 2:
        raise SystemExit("strict fusion comparison requires two old and two new runs")
    args.output.mkdir(parents=True, exist_ok=True)
    # rospy chooses its clock subscription at initialization; setting this
    # afterwards would leave the measurement process on wall time.
    rosgraph.Master("/p2_replay_preflight").setParam("/use_sim_time", True)
    rospy.init_node("p2_replay_measurements", disable_signals=True)
    paths = ("old", "new") if args.path == "both" else (args.path,)
    result = [replay(args, path, repeat) for path in paths for repeat in range(1, args.repeats + 1)]
    summary = {"runs": result, "pass": all(row["pass"] for row in result)}
    if args.fusion:
        frames = [json.loads((args.output / (r["path"] + "_" + str(r["repeat"]) + ".frames.json")).read_text())["tracks"] for r in result]
        summary["track_comparisons"] = [dict(reference="old_1", candidate=r["path"] + "_" + str(r["repeat"]),
                                              **compare_tracks(frames[0], frame)) for r, frame in zip(result[1:], frames[1:])]
        minimum = min(r["fusion_matched_reported"] for r in result if r["path"] == "old")
        summary["matched_not_degraded"] = all(r["fusion_matched_reported"] >= minimum for r in result if r["path"] == "new")
        summary["pass"] &= summary["matched_not_degraded"]
        if all(r["fusion_totals"] for r in result):
            minimum_total = min(r["fusion_totals"]["matched"] for r in result if r["path"] == "old")
            summary["whole_bag_matched_not_degraded"] = all(
                r["fusion_totals"]["matched"] >= minimum_total for r in result if r["path"] == "new")
            summary["pass"] &= summary["whole_bag_matched_not_degraded"]
        tables = [{(row["stamp_ns"], o[0]): tuple(o[1:]) for row in frame for o in row["objects"]}
                  for frame in frames]
        common = set.intersection(*(set(table) for table in tables))
        stable = {key for key in common if tables[0][key] == tables[1][key]}
        regressions = [{"stamp_ns": key[0], "track": key[1], "old": tables[0][key],
                        "new": [table[key] for table in tables[2:]]}
                       for key in sorted(stable) if any(table[key] != tables[0][key] for table in tables[2:])]
        summary["stable_old_track_comparison"] = {"common_all_runs": len(common), "old_stable": len(stable),
                                                  "regressions": regressions}
        summary["pass"] &= bool(stable) and not regressions
        observations = compare_stable_observations(frames[:2], frames[2:])
        summary["stable_old_observation_comparison"] = observations
        summary["pass"] &= bool(observations["old_stable"]) and not observations["regressions"]
    (args.output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    return 0 if summary["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
