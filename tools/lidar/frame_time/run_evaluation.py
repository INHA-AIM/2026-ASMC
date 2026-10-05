#!/usr/bin/env python3
"""Offline L1-B1 evaluation. Writes only a NEW artifact directory; no ROS master.

Raw bag clouds are tested as raw sensor coordinates. This tool does NOT infer
the exact historic leveling/pose used by the legacy node from nearby IMU.
Synthetic snapshots explicitly carry that state for an honest adapter comparison.
"""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import platform
import subprocess
import sys

import numpy as np


def sha(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def write_json(path, value):
    with Path(path).open("x") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")


def rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @ np.array(
        [[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]) @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])


def pose(rotation, translation):
    return {"rotation": np.asarray(rotation).tolist(), "translation": np.asarray(translation).tolist()}


def snapshots():
    """Independent reference arithmetic and exact preprocessing snapshots."""
    result = []
    for name, angles, mount_angles, level_kind in [
        ("pitch_only", [0., -.07, .61], [0., 0., 0.], "pitch"),
        ("roll_pitch", [.12, -.07, .61], [0., 0., 0.], "full"),
        ("roll_pitch_mount", [.12, -.07, .61], [.08, -.11, .19], "full"),
        ("sensor_mount", [.12, -.07, .61], [.08, -.11, .19], "none"),
    ]:
        stamp = 1790415998913611401
        R, M = rpy(*angles), rpy(*mount_angles)
        L = rpy(0., angles[1], 0.) if level_kind == "pitch" else rpy(angles[0], angles[1], 0.)
        if level_kind == "none":
            L = np.eye(3)
        mount, antenna, base_map = np.array([.8, 0, 1.45]), np.array([.35, 0, 1.25]), np.array([12., -7, 1])
        center, axis = np.array([5., 1.2, -1.7]), np.array([1., 0, 0])
        corners = np.array([center + [1 if i & 1 else -1, .9 if i & 2 else -.9,
                                      .75 if i & 4 else -.75] for i in range(8)])
        center_base = M @ center + mount
        corners_base = corners @ M.T + mount
        velocity = np.array([3., 2, 1])
        sample = {"stamp_ns": stamp, "arrival_ns": stamp + 1000000, "clock": "ros_receive",
                  "valid": True, "reference": "gps_antenna", "gps_in_base": antenna.tolist(),
                  "pose": pose(R, base_map + R @ antenna)}
        row = {
            "input_id": name, "stream_id": name, "max_pose_gap_ns": 100000000,
            "frame": "sensor" if level_kind == "none" else "leveled_sensor",
            "calibration_source": "synthetic_fixture_v1_explicit_mount_not_runtime_config",
            "base_from_sensor": pose(M, mount), "applied_leveling": L.tolist(),
            "leveling_source": "synthetic_exact_applied_matrix",
            "time": {"stamp_ns": stamp, "receipt_ns": stamp + 2000000, "processing_ns": stamp + 120000000,
                     "clock": "ros_receive", "stamp_provenance": "synthetic_receive_clock"},
            "pose_samples": [sample],
            "objects": [{"id": 42, "classification": "unknown", "center": (L @ center).tolist(),
                         "corners": (corners @ L.T).tolist(), "long_axis": (L @ axis).tolist(),
                         "velocity_meaning": "absolute_input_vector", "velocity": (L @ velocity).tolist()}],
            "expected": {"center_base": center_base.tolist(), "center_map": (R @ center_base + base_map).tolist(),
                         "corners_base": corners_base.tolist(), "corners_map": (corners_base @ R.T + base_map).tolist(),
                         "velocity_base": (M @ velocity).tolist(), "velocity_map": (R @ M @ velocity).tolist()},
            "legacy_comparable": level_kind != "none",
            "legacy_snapshot": {"ego_rpy_rad": angles, "gps_map": (base_map + R @ antenna).tolist(),
                                "mount": mount.tolist(), "mount_rpy_deg": np.rad2deg(mount_angles).tolist(),
                                "gps_in_base": antenna.tolist()},
        }
        result.append(row)
    template = copy.deepcopy(result[0])
    for name in ["invalid_pose", "missing_pose", "future_pose_arrival", "over100ms_gap", "unknown_velocity",
                 "base_derivative", "base_derivative_no_twist"]:
        row = copy.deepcopy(template)
        row.update(input_id=name, stream_id=name, legacy_comparable=False)
        row.pop("legacy_snapshot")
        t = row["time"]["stamp_ns"]
        if name in {"invalid_pose", "missing_pose", "future_pose_arrival", "over100ms_gap"}:
            row["expected"]["center_map"] = row["expected"]["corners_map"] = row["expected"]["velocity_map"] = None
        if name == "invalid_pose":
            row["pose_samples"][0]["valid"] = False
        elif name == "missing_pose":
            row["pose_samples"] = []
        elif name == "future_pose_arrival":
            row["pose_samples"][0]["arrival_ns"] = row["time"]["processing_ns"] + 1
        elif name == "over100ms_gap":
            a, b = copy.deepcopy(row["pose_samples"][0]), copy.deepcopy(row["pose_samples"][0])
            a.update(stamp_ns=t-50000001, arrival_ns=t-50000000)
            b.update(stamp_ns=t+50000000, arrival_ns=t+50000001)
            row["pose_samples"] = [a, b]
        elif name == "unknown_velocity":
            row["objects"][0]["velocity_meaning"] = "unknown"
            row["expected"]["velocity_base"] = row["expected"]["velocity_map"] = None
        elif name.startswith("base_derivative"):
            row["frame"] = "base_link"
            row["objects"][0].update(center=row["expected"]["center_base"], corners=row["expected"]["corners_base"],
                                      long_axis=[1., 0., 0.], velocity=[0., 0., 0.], velocity_meaning="base_position_derivative")
            row["twist"] = {"stamp_ns": t, "arrival_ns": t+1000000, "clock": "ros_receive", "valid": name == "base_derivative",
                            "linear_base": [2., 0., 0.], "angular_base": [0., 0., .1]}
            vb = np.array([2., 0., 0.]) + np.cross([0., 0., .1], row["objects"][0]["center"])
            row["expected"]["velocity_base"] = vb.tolist() if name == "base_derivative" else None
            row["expected"]["velocity_map"] = (rpy(0., -.07, .61) @ vb).tolist() if name == "base_derivative" else None
        result.append(row)
    return result


def strip_timing(value):
    if isinstance(value, dict):
        return {k: strip_timing(v) for k, v in value.items() if "timing" not in k}
    if isinstance(value, list):
        return [strip_timing(v) for v in value]
    return value


def execute(binary, rows):
    completed = subprocess.run([str(binary)], input="".join(json.dumps(r, allow_nan=False)+"\n" for r in rows),
                               text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True)
    return [json.loads(line) for line in completed.stdout.splitlines()]


def check_snapshots(rows, outputs):
    if len(rows) != len(outputs):
        raise AssertionError("frame count changed")
    maximum = 0.
    for row, out in zip(rows, outputs):
        assert out["valid"], out
        assert out["input_id"] == row["input_id"] and out["stamp_ns"] == row["time"]["stamp_ns"]
        assert len(out["objects"]) == len(row["objects"])
        for arrival in out["pose_sample_arrivals"]:
            assert arrival <= row["time"]["processing_ns"]
        obj, expected = out["objects"][0], row["expected"]
        assert obj["id"] == 42 and obj["classification"] == "unknown" and obj["valid"]
        for field, actual in [("center_base", obj["base"]["center"]), ("corners_base", obj["base"]["corners"]),
                              ("center_map", obj["map"]["center"] if obj["map"] else None),
                              ("corners_map", obj["map"]["corners"] if obj["map"] else None),
                              ("velocity_base", obj["absolute_velocity_base"]), ("velocity_map", obj["absolute_velocity_map"])]:
            if expected[field] is None:
                assert actual is None, (row["input_id"], field)
            else:
                error = float(np.max(np.abs(np.array(actual)-expected[field])))
                maximum = max(maximum, error)
                assert error <= 1e-6, (row["input_id"], field, error)
    return maximum


def sensor_mount(repo):
    source = repo / "config/morai/sensors/ASMC_sensor_set_v3.json"
    sensors = json.loads(source.read_text())
    lidar = next(s for s in sensors["Lidar3DList"] if s["m_SensorUniqueID"] == 5)
    translation = [float(lidar["pos"][k]) for k in ("x", "y", "z")]
    rotation = rpy(*[math.radians(float(lidar["rot"][k])) for k in ("roll", "pitch", "yaw")])
    return pose(rotation, translation), {"repo_path": str(source.relative_to(repo)), "sha256": sha(source)}


def raw_record(identifier, mount, calibration, stamp, receipt, provenance):
    return {"input_id": identifier, "stream_id": identifier, "frame": "sensor",
            "calibration_source": calibration, "base_from_sensor": mount, "max_pose_gap_ns": 100000000,
            "time": {"stamp_ns": stamp, "receipt_ns": receipt, "processing_ns": receipt,
                     "clock": "ros_receive", "stamp_provenance": provenance}, "objects": [], "pose_samples": []}


def bag_records(path, scene_manifest, output, mount, calibration):
    import rosbag
    from sensor_msgs import point_cloud2
    scenes = json.loads(scene_manifest.read_text())["scenes"]
    selected = set()
    records = []
    imu_received = 0
    with rosbag.Bag(str(path)) as bag:
        for topic, msg, receipt in bag.read_messages(topics=["/velodyne_points", "/imu"]):
            if topic == "/imu":
                imu_received += 1
                continue
            ns, stamp = receipt.to_nsec(), msg.header.stamp.to_nsec()
            row = raw_record("bag_cloud_%05d" % len(records), mount, calibration, stamp, ns,
                             "bridge_receive_not_ray_time")
            row["stream_id"] = "stationary_bag"
            row["imu_samples_arrived"] = imu_received
            row["source_frame"] = msg.header.frame_id
            # No map pose or preprocessing rotation is invented from raw IMU.
            if msg.header.frame_id != "velodyne":
                raise ValueError("raw cloud frame differs from calibrated sensor")
            for scene in scenes:
                if (scene["name"] not in selected and scene["measurement_start_wall_s"] <= receipt.to_sec()
                        <= scene["measurement_end_wall_s"]):
                    points = np.asarray(list(point_cloud2.read_points(msg, field_names=("x", "y", "z", "intensity"),
                                                                      skip_nans=False)), dtype="<f4")
                    dest = output / (scene["name"] + "_raw_xyzi.bin")
                    with dest.open("xb") as stream:
                        stream.write(points.tobytes())
                    row["cloud_path"] = str(dest)
                    row["cloud_sha256"] = sha(dest)
                    row["scene"] = scene["name"]
                    selected.add(scene["name"])
                    break
            records.append(row)
    if len(selected) != len(scenes):
        raise ValueError("missing raw cloud for stationary scene")
    return records


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build", required=True, type=Path)
    parser.add_argument("--data-root", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    repo = Path(__file__).resolve().parents[3]
    data = args.data_root.resolve()
    out = args.output.resolve()
    out.relative_to(data)  # Results must stay in ASMC_DATA, never overwrite source/repo.
    out.mkdir(parents=True, exist_ok=False)
    source = data / "lidar_assessment/20260926"
    binary, legacy = args.build / "frame_time_eval", args.build / "legacy_adapter_probe"
    report = {"schema": "asmc-lidar-l1b1-report-v1", "status": "incomplete", "runtime_changed": False,
              "max_pose_gap_ns": 100000000, "no_extrapolation": True,
              "platform": platform.platform(), "python": platform.python_version(),
              "baseline_revisions": {"main": "1ce434d0082e3aec6cf79145c05484e1f66fac1c",
                                     "son": "6435af10671235e735e4a9ed1bd747bd8d8389ac",
                                     "seojun": "8b3f0b1fab5f8e8f10531b42fb63b2953c8df822"},
              "invocation": [sys.executable, *sys.argv],
              "compiler": subprocess.check_output(["c++", "--version"], text=True).splitlines()[0],
              "build_cache_sha256": sha(args.build / "CMakeCache.txt")}
    test = subprocess.run([str(args.build / "frame_time_tests")], text=True, capture_output=True, check=True)
    (out / "core-tests.log").write_text(test.stdout)
    rows = snapshots()
    results = execute(binary, rows)
    report["synthetic_max_error"] = check_snapshots(rows, results)
    report["synthetic_frames"] = len(rows)
    comparable = [r for r in rows if r["legacy_comparable"]]
    reference = execute(legacy, comparable)
    assert len(reference) == len(comparable)
    comparisons = []
    for row, old in zip(comparable, reference):
        expected = row["expected"]
        old_obj = old["objects"][0]
        comparisons.append({"case": row["input_id"],
                            "legacy_map_center_error_m": float(np.linalg.norm(np.array(old_obj["map_center"])-expected["center_map"])),
                            "legacy_local_vs_base_error_m": float(np.linalg.norm(np.array(old_obj["local_center_as_published"])-expected["center_base"])),
                            "legacy_corners_max_error_m": float(np.max(np.linalg.norm(np.array(old_obj["map_corners"])-expected["corners_map"], axis=1)))})
    assert comparisons[0]["legacy_map_center_error_m"] < 1e-6
    assert comparisons[1]["legacy_map_center_error_m"] > .1
    assert comparisons[0]["legacy_local_vs_base_error_m"] > .5
    report["legacy_adapter_comparison"] = comparisons
    write_json(out / "legacy_results.json", reference)
    mount, profile = sensor_mount(repo)
    report["sensor_profile"] = profile
    bag = source / "stationary_v2/stationary_0.bag"
    report["bag"] = {"path": str(bag.relative_to(data)), "sha256": sha(bag), "bytes": bag.stat().st_size}
    raw_rows = bag_records(bag, source / "stationary_v2/manifest.json", out, mount, profile["sha256"])
    report["scene_manifest"] = {"path": str((source / "stationary_v2/manifest.json").relative_to(data)),
                                "sha256": sha(source / "stationary_v2/manifest.json")}
    raw_results = execute(binary, raw_rows)
    assert len(raw_rows) == len(raw_results)
    for row, result in zip(raw_rows, raw_results):
        assert result["valid"] and result["stamp_ns"] == row["time"]["stamp_ns"], result
    report["bag_clouds_checked"] = len(raw_results)
    report["legacy_live_state_reconstructed"] = False
    stage = source / "ground_benchmark_v1/stage"
    stage_manifest = stage / "manifest.json"
    report["capture_stage_manifest"] = {"path": str(stage_manifest.relative_to(data)), "sha256": sha(stage_manifest)}
    staged = json.loads(stage_manifest.read_text())
    # Stage is the already admitted single-frame ROI set, NOT full raw Capture.
    admitted = {item["frame"] for item in staged["samples"]}
    capture_rows = []
    for path in sorted(stage.glob("*.bin")):
        if path.stem not in admitted:
            raise ValueError("unadmitted staged capture file")
        row = raw_record("capture_"+path.stem, mount, "capture_same_mount_checked_in_L1A; "+profile["sha256"],
                         1, 1, "synthetic_fixture_index_no_capture_measurement_time")
        row.update(cloud_path=str(path), cloud_sha256=sha(path), capture_time_unavailable=True)
        capture_rows.append(row)
    if len(capture_rows) != len(admitted):
        raise ValueError("missing or duplicated capture samples")
    capture_results = execute(binary, capture_rows)
    assert len(capture_results) == len(capture_rows)
    for row, result in zip(capture_rows, capture_results):
        assert result["valid"] and result["input_id"] == row["input_id"] and "cloud" in result
    cloud_results = [r for r in raw_results+capture_results if "cloud" in r]
    for result in cloud_results:
        cloud = result["cloud"]
        assert cloud["input_points"] == cloud["output_points"] and cloud["intensity_bit_mismatches"] == 0
        assert cloud["roundtrip_max_m"] <= 1e-6
    report["capture_samples"] = len(capture_results)
    report["cloud_roundtrip_max_m"] = max(r["cloud"]["roundtrip_max_m"] for r in cloud_results)
    report["cloud_metrics"] = [{"input_id": r["input_id"], **r["cloud"]} for r in cloud_results]
    report["object_batch_timing"] = [{"input_id": r["input_id"], "object_count": len(r["objects"]),
                                       "copy": r["object_batch_copy_timing"],
                                       "transform": r["object_batch_transform_timing"]} for r in results]
    all_rows = rows+raw_rows+capture_rows
    all_results = results+raw_results+capture_results
    with (out / "snapshots.jsonl").open("x") as stream:
        for row in all_rows:
            stream.write(json.dumps(row, sort_keys=True, allow_nan=False)+"\n")
    with (out / "results.jsonl").open("x") as stream:
        for result in all_results:
            stream.write(json.dumps(result, sort_keys=True, allow_nan=False)+"\n")
    rerun = execute(binary, all_rows)
    deterministic = json.dumps(strip_timing(all_results), sort_keys=True, allow_nan=False)
    assert deterministic == json.dumps(strip_timing(rerun), sort_keys=True, allow_nan=False)
    report["deterministic_geometry_status_sha256"] = hashlib.sha256(deterministic.encode()).hexdigest()
    report["artifact_sha256"] = {p.name: sha(p) for p in sorted(out.iterdir()) if p.is_file()}
    report["source_sha256"] = {p.name: sha(p) for p in Path(__file__).parent.iterdir()
                               if p.is_file() and p.suffix in {".py", ".cpp", ".hpp", ".cmake", ".txt"}}
    report["binary_sha256"] = {p.name: sha(p) for p in [binary, legacy, args.build / "frame_time_tests"]}
    report["limitations"] = [
        "Synthetic snapshots compare the actual pinned adapter transform functions, not detector/tracker accuracy.",
        "Live bag raw clouds keep receive timestamps. No exact legacy processing state reconstructed from nearby IMU.",
        "Capture stage was already ROI-filtered in L1-A, uses old 10Hz capture; dummy time=1 is fixture-only, never sensor time.",
        "No map pose invented for real bag/capture. Map and velocity accuracy need synchronized pose/motion evidence.",
        "Processing cutoff is replay receipt time; steady_clock timings are separate. No deskew or end-to-end latency claim.",
        "GPS transitions, continuous driving, detection recall, camera BEV correction and operational consumer integration remain unaccepted."]
    report["status"] = "passed_isolated_frame_time_only"
    write_json(out / "report.json", report)
    print(json.dumps({k: report[k] for k in ["status", "synthetic_frames", "synthetic_max_error", "bag_clouds_checked",
                                            "capture_samples", "cloud_roundtrip_max_m"]}, indent=2))


if __name__ == "__main__":
    main()
