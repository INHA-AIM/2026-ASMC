#!/usr/bin/env python3
"""Inject faults into real GPU producer + adapter, alongside the lane launch.

Run after bag measurements, in the same isolated ROS master. The tool uses
wall time and stops only processes it starts. No simulator/bridge is needed.
"""
import argparse
import json
import os
import subprocess
import time
from pathlib import Path

import cv2
import numpy as np
import rosgraph
import rosnode
import rospy
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Header
from asmc_msgs.msg import CameraObjectBoxArray
from Msg.msg import Yolo_bbox_arr

from replay_camera_objects import stop, wait_for


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault("TWINLITE_ROOT", "/opt/baselines/TwinLiteNetPlus")
    rosgraph.Master("/p2_fault_preflight").setParam("/use_sim_time", False)
    rospy.init_node("p2_fault_checks", disable_signals=True)
    candidates, legacy_boxes, diagnostics, lane_outputs = [], [], [], []
    subscriptions = [rospy.Subscriber("/perception/camera/objects", CameraObjectBoxArray, candidates.append, queue_size=20),
                     rospy.Subscriber("/YOLO/BBox", Yolo_bbox_arr, legacy_boxes.append, queue_size=20),
                     rospy.Subscriber("/perception/camera/objects/diagnostics", DiagnosticArray, diagnostics.append, queue_size=20),
                     rospy.Subscriber("/perception/camera/front/drivable/probability", Image, lane_outputs.append, queue_size=20)]
    publisher = rospy.Publisher("/image_jpeg/compressed", CompressedImage, queue_size=5)
    jpeg = cv2.imencode(".jpg", np.zeros((720, 1280, 3), np.uint8))[1].tobytes()
    processes, streams, checks = [], [], {}

    def launch(command, suffix):
        stream = args.output.with_suffix("." + suffix + ".log").open("w")
        streams.append(stream)
        process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(process)
        return process

    def send(data):
        header = Header(seq=len(candidates) + 1, stamp=rospy.Time.now(), frame_id="camera_front")
        publisher.publish(CompressedImage(header=header, format="jpeg", data=data))
        return header

    def latest_values():
        return {value.key: value.value for value in diagnostics[-1].status[0].values} if diagnostics else {}

    try:
        graph = launch(["roslaunch", "asmc_camera", "perception.launch", "profile:=legacy", "start_camera_objects:=true"], "camera")
        launch(["rosrun", "Main", "camera_objects_yolo_bbox_adapter.py"], "adapter")
        wait_for(lambda: publisher.get_num_connections() >= 2, "warmed object and lane subscriptions", timeout=60)
        time.sleep(0.5)
        # Retain a stale pre-input diagnostic deliberately. Without source
        # stamp freshness the later decode assertion can read this snapshot.
        wait_for(lambda: int(latest_values().get("latest_result_stamp_ns", 0)) == 0
                 and float(latest_values().get("input_silence_sec", 0)) > 1.0,
                 "startup silence diagnostic before fault frames")
        checks["startup_silence_before_frames"] = True
        header = send(jpeg)
        wait_for(lambda: candidates and legacy_boxes and lane_outputs, "normal empty results and live lane output")
        checks["normal_empty"] = (candidates[-1].valid and not candidates[-1].boxes
                                 and legacy_boxes[-1].objc == 0 and candidates[-1].header == header)
        before = len(legacy_boxes)
        header = send(b"deliberately broken jpeg")
        wait_for(lambda: len(candidates) >= 2, "invalid decode result")
        time.sleep(0.3)
        checks["broken_jpeg_invalid_withheld"] = (not candidates[-1].valid and not candidates[-1].boxes
                                                  and candidates[-1].header == header and len(legacy_boxes) == before)
        count = len(candidates)
        # The monitor may still expose startup silence from before either
        # injected frame. Require the invalid frame's source stamp so this
        # assertion observes its decode error and subsequent input stop.
        wait_for(lambda: int(latest_values().get("latest_result_stamp_ns", 0)) == header.stamp.to_nsec()
                 and float(latest_values().get("input_silence_sec", 0)) > 1.0,
                 "fresh invalid-frame input silence diagnostic")
        checks["input_stop_no_repeated_output"] = len(candidates) == count
        checks["decode_error_diagnostic"] = int(latest_values().get("decode_errors", 0)) >= 1
        old = launch(["rosrun", "Main", "yolo_detection_node", "__name:=yolo_detection_node"], "legacy")
        wait_for(lambda: len(dict(rosgraph.Master("/p2_fault_graph").getSystemState()[0]).get("/YOLO/BBox", [])) == 2,
                 "two warmed bbox publishers")
        wait_for(lambda: "/yolo_detection_node" in latest_values().get("duplicate_nodes", ""), "duplicate inference diagnostic")
        checks["duplicate_warning"] = True
        stop(old)
        success, failures = rosnode.kill_nodes(["/camera_object_detector"])
        wait_for(lambda: "/camera_object_detector" not in rosnode.get_node_names(), "producer shutdown")
        lane_before = len(lane_outputs)
        for _ in range(3):
            send(jpeg)
            time.sleep(0.1)
        wait_for(lambda: len(lane_outputs) > lane_before, "lane output after detector termination")
        checks["producer_exit_lane_survives"] = (graph.poll() is None and not failures
                                                 and "/camera_semantic_inference" in rosnode.get_node_names()
                                                 and "/camera_objects_yolo_bbox_adapter" in rosnode.get_node_names())
        checks["all_pass"] = all(checks.values())
    finally:
        for process in reversed(processes):
            stop(process)
        for subscription in subscriptions:
            subscription.unregister()
        for stream in streams:
            stream.close()
        args.output.write_text(json.dumps(checks, indent=2) + "\n")
        args.output.with_suffix(".diagnostics.json").write_text(json.dumps([
            {"stamp_ns": message.header.stamp.to_nsec(), "status": [
                {"level": status.level, "message": status.message,
                 "values": {value.key: value.value for value in status.values}}
                for status in message.status]} for message in diagnostics], indent=2) + "\n")
    print(json.dumps(checks, indent=2))
    return 0 if checks.get("all_pass") else 1


if __name__ == "__main__":
    raise SystemExit(main())
