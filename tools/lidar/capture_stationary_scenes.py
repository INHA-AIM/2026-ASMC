#!/usr/bin/env python3
"""Opt-in MORAI stationary LiDAR fixtures; restore Ego, never drive or publish GT.

Run only on a development simulator. RPC snapshots are asynchronous evaluator
evidence, NOT synchronized labels. Uses data_collection's bag command builder.
"""
import argparse
import json
import math
from pathlib import Path
import signal
import subprocess
import sys
import time


def lane_pose(points, fraction, z):
    """Interpolate distance along a map centerline; map z=0 is not sim altitude."""
    if not 0 <= fraction <= 1 or len(points) < 2:
        raise ValueError("need a polyline and fraction in [0, 1]")
    segments = [(a, b, math.hypot(b[0]-a[0], b[1]-a[1]))
                for a, b in zip(points, points[1:])]
    segments = [(a, b, d) for a, b, d in segments if d > 1e-6]
    if not segments:
        raise ValueError("zero-length lane")
    remaining = sum(d for _, _, d in segments) * fraction
    for a, b, length in segments:
        if remaining <= length + 1e-6:
            t = min(1.0, remaining / length)
            return dict(x=a[0]+t*(b[0]-a[0]), y=a[1]+t*(b[1]-a[1]), z=z,
                        roll=0.0, pitch=0.0,
                        yaw=math.degrees(math.atan2(b[1]-a[1], b[0]-a[0])))
        remaining -= length
    raise ValueError("unreachable interpolation")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True,
                        help="new directory, refuses to overwrite")
    parser.add_argument("--host", default="host.docker.internal")
    parser.add_argument("--port", type=int, default=7789)
    parser.add_argument("--seconds", type=float, default=35)
    parser.add_argument("--confirm-simulator-move", action="store_true")
    args = parser.parse_args()
    if not args.confirm_simulator_move or not 5 <= args.seconds <= 120:
        parser.error("explicit move confirmation and seconds in [5,120] required")
    repo = Path(__file__).resolve().parents[2]
    sys.path[:0] = [str(repo / "tools/grpc_inha_univ/src/proto"),
                   str(repo / "src/data_collection/src")]
    import grpc
    from google.protobuf.json_format import MessageToDict, ParseDict
    from morai.actor.actor_pb2_grpc import ActorStub
    from morai.actor.actor_get_pb2 import GetAllActorsFilter
    from morai.actor.actor_set_pb2 import (SetTransformParam, SetVelocityParam,
                                         VehicleControlModeParam, VehicleGear)
    from morai.actor.actor_enum_pb2 import VEHICLE_CONTROL_KEYBOARD, GEAR_MODE_P
    from morai.common.object_info_pb2 import ObjectInfo
    from morai.common.enum_pb2 import OBJECT_TYPE_VEHICLE, STATUS_CODE_SUCCESS
    from data_collection.bag_run import build_rosbag_command, sha256_file

    channel = grpc.insecure_channel("{}:{}".format(args.host, args.port))
    api = ActorStub(channel)
    ego = ObjectInfo()
    ego.id.value = "Ego"
    ego.object_type = OBJECT_TYPE_VEHICLE
    original = api.GetActorState(ego, timeout=5)
    original_mode = api.GetVehicleControlMode(ego, timeout=5).mode
    if math.sqrt(sum(getattr(original.velocity, a)**2 for a in "xyz")) > 0.1:
        raise RuntimeError("Ego must be stationary before this experiment")
    if not original.HasField("transform"):
        raise RuntimeError("missing initial Ego transform")
    map_path = repo / "src/asmc_hd_map/config/kcity2025_compact_drivable_map.json"
    hdmap = json.loads(map_path.read_text())
    poses = [("initial", None)]
    for name, link, fraction in (("highway_A423", "A2256W000423", .5),
                                 ("shadow_A126_entry", "A2256W000126", .22),
                                 ("shadow_A126_middle", "A2256W000126", .55)):
        lane = next(l for l in hdmap["lanes"] if l["link_id"] == link)
        poses.append((name, lane_pose(lane["centerline"], fraction,
                                      original.transform.location.z)))
    poses.append(("recovered_initial", MessageToDict(original.transform,
                  preserving_proto_field_name=True)))
    args.output.mkdir(parents=True, exist_ok=False)
    def write(name, value):
        (args.output / name).write_text(json.dumps(value, indent=2, sort_keys=True))
    manifest = {"schema": "lidar-stationary-fixtures-v1",
                "simulator": "26R1 (operator-reported)", "original_control_mode": original_mode,
                "original_state": MessageToDict(original, preserving_proto_field_name=True),
                "gt_usage": "evaluation_only_asynchronous_not_synchronized_labels",
                "sensor_profile": "ASMC_sensor_set_v3 (operator-reported)",
                "timestamp_provenance": "bridge receive; no scan deskew timing claim",
                "map_sha256": sha256_file(str(map_path)), "scenes": [],
                "restore_complete": False, "recording_complete": False,
                "collection_complete": False}
    write("manifest.json", manifest)
    def call(method, cls, **kwargs):
        request = cls(**kwargs)
        request.actor_info.CopyFrom(ego)
        response = method(request, timeout=5)
        if response.status != STATUS_CODE_SUCCESS:
            raise RuntimeError("RPC failed: {}".format(response))
    topics = ["/velodyne_points", "/gps", "/imu", "/Ego_topic", "/CollisionData",
              "/image_jpeg/compressed", "/camera/image/compressed", "/YOLO/BBox",
              "/last_lidar", "/lidar_nogps", "/lidar/tracked_objects_map",
              "/lidar/tunnel/objects", "/lidar/tunnel/diagnostics",
              "/lidar/debug_roi", "/lidar/debug_ransac", "/lidar/debug_clustered",
              "/morai_sensor_bridge/diagnostics", "/morai_udp_bridge/diagnostics",
              "/tf_static"]
    manifest["topics"] = topics
    cmd = build_rosbag_command({"compression": "lz4"},
                              str(args.output / "stationary"), topics)
    recording = None
    errors = []
    try:
        call(api.SetVehicleControlMode, VehicleControlModeParam, mode=VEHICLE_CONTROL_KEYBOARD)
        call(api.SetVelocity, SetVelocityParam, velocity=0)
        call(api.SetVehicleGear, VehicleGear, gear=GEAR_MODE_P)
        with (args.output / "rosbag.log").open("w") as log, \
                (args.output / "actor_snapshots.jsonl").open("w") as snapshots:
            recording = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT,
                                         start_new_session=True)
            time.sleep(2)
            for name, pose in poses:
                scene = {"name": name, "transition_wall_s": time.time(), "requested_pose": pose}
                manifest["scenes"].append(scene)
                write("manifest.json", manifest)
                if pose:
                    req = SetTransformParam()
                    req.actor_info.CopyFrom(ego)
                    if "location" in pose:
                        ParseDict(pose, req.transform)
                    else:
                        for a in "xyz": setattr(req.transform.location, a, pose[a])
                        for axis, angle in zip("xyz", ("roll", "pitch", "yaw")):
                            setattr(req.transform.rotation, axis, pose[angle])
                    if api.SetTransform(req, timeout=5).status != STATUS_CODE_SUCCESS:
                        raise RuntimeError("teleport failed")
                    call(api.SetVelocity, SetVelocityParam, velocity=0)
                time.sleep(3)
                scene["measurement_start_wall_s"] = time.time()
                print("collecting", name, flush=True)
                until = time.monotonic()+args.seconds
                while time.monotonic() < until:
                    if recording.poll() is not None:
                        raise RuntimeError("rosbag exited unexpectedly")
                    before = time.time()
                    state = api.GetAllActorsState(GetAllActorsFilter(
                        vehicle=True, pedestrian=True, obstacle=True), timeout=5)
                    snapshots.write(json.dumps({"scene": name, "request_wall_s": before,
                        "response_wall_s": time.time(), "states": MessageToDict(
                            state, preserving_proto_field_name=True)})+"\n")
                    snapshots.flush()
                    time.sleep(.5)
                scene["measurement_end_wall_s"] = time.time()
                write("manifest.json", manifest)
            manifest["collection_complete"] = True
    finally:
        # Attempt each restoration independently; retain evidence if any RPC fails.
        for method, cls, kwargs in (
            (api.SetVelocity, SetVelocityParam, {"velocity": 0}),
            (api.SetTransform, SetTransformParam, {"transform": original.transform}),
            (api.SetVehicleGear, VehicleGear, {"gear": original.vehicle_state.gear_mode}),
            (api.SetVehicleControlMode, VehicleControlModeParam, {"mode": original_mode})):
            try: call(method, cls, **kwargs)
            except Exception as exc: errors.append(str(exc))
        if recording is not None and recording.poll() is None:
            recording.send_signal(signal.SIGINT)
            try: recording.wait(timeout=30)
            except subprocess.TimeoutExpired:
                errors.append("rosbag finalization timed out; do not use active bag")
        manifest["restore_complete"] = not errors
        manifest["restore_errors"] = errors
        manifest["recording_complete"] = (recording is not None and
            recording.returncode == 0 and not list(args.output.glob("*.active")))
        manifest["bags"] = [{"path": p.name, "bytes": p.stat().st_size,
                             "sha256": sha256_file(str(p))}
                            for p in args.output.glob("*.bag")]
        write("manifest.json", manifest)
        channel.close()
        print("restoration:", manifest["restore_complete"], flush=True)
    if errors or not manifest["recording_complete"]:
        raise RuntimeError("incomplete restoration/recording; inspect manifest")


if __name__ == "__main__":
    main()
