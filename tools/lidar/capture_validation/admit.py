#!/usr/bin/env python3
"""Freeze paired Capture XYZI and evaluation-only GT without modifying sources.

Capture trigger time is NOT a beam measurement stamp. Repetition warms the
pinned wall-persistence gate on ONE stationary snapshot, not a moving sequence.
Only XYZI and paired attitude go to the C++ filter; GT is a separate artifact.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import yaml

SUFFIXES = ("Intensity.bin", "Semantic.bin", "Instance.bin", "Instance.txt", "Instance_8Points.txt")
SIGNS = np.array([[-1, 1, -1], [-1, 1, 1], [-1, -1, -1], [-1, -1, 1],
                  [1, 1, -1], [1, 1, 1], [1, -1, -1], [1, -1, 1]])


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)


def sha(path):
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def file_evidence(path, root):
    return {"path": str(path.relative_to(root)), "bytes": path.stat().st_size, "sha256": sha(path)}


def rotation(angles):
    r, p, y = angles
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr], [-sp, cp * sr, cp * cr]])


def read_native_state(path):
    result = {}
    for line in path.read_text().splitlines():
        if ":" in line:
            key, value = line.split(":", 1)
            result[key.strip()] = float(value.strip())
    if not result or not all(math.isfinite(v) for v in result.values()):
        raise ValueError("empty/nonfinite native state: " + str(path))
    return result


def attitude(imu):
    q = np.array([imu["Orientation " + a] for a in "XYZW"])
    norm = float(np.linalg.norm(q))
    if not .99 <= norm <= 1.01:
        raise ValueError("invalid quaternion norm")
    x, y, z, w = q / norm
    return (math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
            math.asin(float(np.clip(2 * (w * y - z * x), -1, 1))),
            math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def boxes(path, corner_path, xyz, ids):
    def parse(source, count):
        values = {}
        for line in source.read_text().splitlines():
            if not line.strip():
                continue
            fields = line.split()
            if len(fields) != count:
                raise ValueError("invalid box record length: " + str(source))
            v = np.array([float(x) for x in fields[1:]])
            if not np.isfinite(v).all() or v[-1] <= 0 or v[-1] != int(v[-1]):
                raise ValueError("invalid box fields/ID")
            key = int(v[-1])
            if key in values:
                raise ValueError("duplicate box ID")
            values[key] = (fields[0], v)
        return values
    basic, corners = parse(path, 15), parse(corner_path, 33)
    if set(basic) != set(corners) or not set(ids[ids > 0]).issubset(basic):
        raise ValueError("unpaired/unlabelled instance ID")
    result = []
    for key, (kind, b) in sorted(basic.items()):
        corner_kind, c = corners[key]
        center, angles, dims = b[:3], b[3:6], b[6:9]
        R = rotation(angles)
        vertices = c[:24].reshape(8, 3)
        predicted = center + (SIGNS * dims / 2) @ R.T
        err = float(np.max(np.linalg.norm(vertices - predicted, axis=1)))
        if kind != corner_kind or np.any(dims <= 0) or err > .001:
            raise ValueError("box/corner geometry disagreement")
        mask = ids == key
        inside = np.all(np.abs((xyz[mask] - center) @ R) <= dims / 2 + .03, axis=1)
        result.append({"id": key, "class": kind, "center": center.tolist(), "angles_rad": angles.tolist(),
                       "dimensions": dims.tolist(), "corners": vertices.tolist(), "distance_m": float(b[9]),
                       "visible_points": int(mask.sum()), "outside_box_3cm": int((~inside).sum()),
                       "corner_error_max_m": err,
                       "box_accuracy_admitted": bool(mask.any() and inside.all()),
                       "velocity_used": False})
    return result


def aligned_clouds(paths):
    matrices = []
    for path in paths:
        if path.stat().st_size % 16:
            raise ValueError("invalid float4 bytes")
        a = np.fromfile(path, dtype="<f4").reshape(-1, 4)
        if not len(a) or not np.isfinite(a).all():
            raise ValueError("empty/nonfinite cloud")
        matrices.append(a)
    if any(a.shape != matrices[0].shape or not np.array_equal(a[:, :3], matrices[0][:, :3]) for a in matrices[1:]):
        raise ValueError("GT XYZ differ from Intensity")
    for a in matrices[1:]:
        if np.any(a[:, 3] < 0) or np.any(a[:, 3] != np.floor(a[:, 3])):
            raise ValueError("nonintegral GT labels")
    return matrices


def camera_fixture(state, stamp):
    if state not in ("missing", "empty"):
        raise ValueError("unsupported controlled camera condition")
    selected = state == "empty"
    return {"selected_yolo_index": 0 if selected else None, "boxes": [],
            "selected_yolo_stamp_ns": stamp if selected else None,
            "selected_yolo_arrival_ns": stamp if selected else None,
            "scan_stamp_ns": stamp, "scan_effective_stamp_ns": stamp,
            "scan_arrival_ns": stamp, "decision_ns": stamp, "wait_ns": 0,
            "protection_enabled": True, "reason": "matched_empty" if selected else "missing_yolo",
            "unmatched_cause": None if selected else "not_yet_arrived",
            "fixture_only_not_recorded_yolo": True}


def projection_check(path, gt, setting):
    """Corroborate FLU handedness with matched front-camera boxes, not feed GT.

    Center projections and box midpoints need not coincide; these residuals
    compare declared and reflected y only, not approve physical calibration.
    """
    if not path.is_file():
        # Missing native BBox output is not proof of a valid empty YOLO result.
        return [{"file_available": False, "camera_detection_state": "unverified"}]
    camera, lidar = setting["Vehicle"]["Camera"], setting["Vehicle"]["LiDAR"]
    R = rotation(np.deg2rad([camera[k] for k in ("roll", "pitch", "yaw")])).T
    t = np.array([lidar[k] - camera[k] for k in ("x", "y", "z")])
    by_id = {b["id"]: b for b in gt}
    errors = []
    for line in path.read_text().splitlines():
        f = line.split()
        if not f:
            continue
        if len(f) < 6:
            raise ValueError("malformed front-camera box")
        # Camera visibility can include actors not observed by this LiDAR.
        if int(f[5]) not in by_id:
            errors.append({"id": int(f[5]), "lidar_box_paired": False})
            continue
        expected = (float(f[1]) + float(f[3])) / 2
        p = np.array(by_id[int(f[5])]["center"])
        residual = []
        for flip in (1, -1):
            q = R @ (p * [1, flip, 1] + t)
            residual.append(abs((camera["cx"] - camera["fx"] * q[1] / q[0]) - expected) if q[0] > 0 else None)
        errors.append({"id": int(f[5]), "lidar_box_paired": True,
                       "declared_y_error_px": residual[0], "reflected_y_error_px": residual[1]})
    return errors


def admit(args):
    if args.output.exists():
        raise ValueError("output exists; do not overwrite evidence")
    records = [json.loads(s) for s in (args.capture_run / "capture_manifest.jsonl").read_text().splitlines() if s]
    if not records or any(not r.get("success") for r in records):
        raise ValueError("only fully successful closed Capture runs are admitted")
    indices = [r["sequence"] for r in records]
    if indices != list(range(len(indices))) or len({r["custom_name"] for r in records}) != len(records):
        raise ValueError("duplicate/noncontiguous Capture sequence")
    closed = json.loads((args.capture_run / "summary.json").read_text())
    if not closed.get("closed_at_utc") or closed["succeeded"] != len(records):
        raise ValueError("Capture run is not closed/complete")
    setting = yaml.safe_load(args.setting.read_text())
    profile = json.loads(args.sensor_profile.read_text())
    lidar = [s for s in profile["Lidar3DList"] if s["m_SensorUniqueID"] == 5]
    imu_sensor = [s for s in profile["IMUList"] if s["m_SensorUniqueID"] == 7]
    if len(lidar) != 1 or len(imu_sensor) != 1:
        raise ValueError("expected declared LiDAR-5/IMU-7")
    for key in ("x", "y", "z"):
        if abs(float(lidar[0]["pos"][key]) - setting["Vehicle"]["LiDAR"][key]) > 1e-6:
            raise ValueError("profile/reference LiDAR mount mismatch")
    for s in (lidar[0], imu_sensor[0]):
        if any(float(s["rot"][k]) != 0 for k in ("roll", "pitch", "yaw")):
            raise ValueError("nonidentity sensor rotation requires explicit import implementation")
    args.output.mkdir(parents=True)
    (args.output / "clouds").mkdir()
    (args.output / "truth").mkdir()
    snapshots, truths, frames, source_entries = [], [], [], []
    counters = Counter()
    for record in records:
        name = record["custom_name"]
        # Reject path traversal even in a syntactically valid recorder manifest.
        if Path(name).name != name or not name.endswith("_{:06d}".format(record["sequence"])):
            raise ValueError("unsafe/inconsistent Capture key")
        files = [args.lidar_root / (name + "_" + suffix) for suffix in SUFFIXES]
        xyzi, semantic, instance = aligned_clouds(files[:3])
        ids = instance[:, 3].astype("<i4")
        semantics = semantic[:, 3].astype("<i4")
        gt = boxes(files[3], files[4], xyzi[:, :3], ids)
        imu_path = args.state_root / "IMU_7" / (name + ".txt")
        gps_path = args.state_root / "GPS_6" / (name + ".txt")
        camera_path = args.state_root / "CAMERA_1" / (name + "_BBox2D.txt")
        imu, gps = read_native_state(imu_path), read_native_state(gps_path)
        roll, pitch, yaw = attitude(imu)
        secs, nanos = imu["TimeStamp.Secs"], imu["TimeStamp.nSecs"]
        stamp_fields_valid = secs > 0 and secs == int(secs) and 0 <= nanos < 1e9 and nanos == int(nanos)
        counters["imu_invalid_native_stamp"] += not stamp_fields_valid
        cloud_path = Path("clouds") / (name + ".bin")
        # Derived evaluation copy of XYZI, bit-identical; no GT/intensity edits.
        xyzi.tofile(args.output / cloud_path)
        ids.tofile(args.output / "truth" / (name + ".instance.i32"))
        semantics.tofile(args.output / "truth" / (name + ".semantic.i32"))
        counts = dict(zip(*[a.tolist() for a in np.unique(ids, return_counts=True)]))
        evidence = {"sequence": record["sequence"], "capture_key": name, "raw_points": len(xyzi),
                    "instance_counts": counts, "boxes": gt,
                    "imu_native_fields": {"secs": int(secs), "nsecs": int(nanos)},
                    "imu_native_stamp_valid": bool(stamp_fields_valid), "attitude_rad": [roll, pitch, yaw],
                    "gps_valid_latlon": gps["Latitude"] != 0 and gps["Longitude"] != 0,
                    "ego_at_trigger": record.get("state", {}).get("ego"),
                    "projection_check": projection_check(camera_path, gt, setting)}
        frames.append(evidence)
        counters["front_bbox_file_missing"] += not camera_path.is_file()
        counters["raw_points"] += len(xyzi)
        counters["instance_labelled_points"] += int((ids > 0).sum())
        counters["box_observations"] += len(gt)
        counters["box_accuracy_excluded"] += sum(not b["box_accuracy_admitted"] for b in gt)
        for root_name, root, paths in (("lidar", args.lidar_root, files),
                                       ("state", args.state_root, [imu_path, gps_path, camera_path])):
            source_entries.extend({"root": root_name, **file_evidence(p, root)} for p in paths if p.is_file())
        cloud_sha = sha(args.output / cloud_path)
        if cloud_sha != sha(files[0]):
            raise ValueError("derived XYZI changed source bytes")
        for state in ("missing", "empty"):
            phase = name + "_" + state
            for warm in range(args.repetitions):
                identity = phase + "_warm" + str(warm)
                stamp = int(record["request_wall_time_ns"])
                d = camera_fixture(state, stamp)
                snapshots.append({"input_id": identity, "scene": state, "phase": phase,
                                  "capture_key": name, "sequence": record["sequence"],
                                  "controlled_camera_state": state, "measurement": warm == args.repetitions - 1,
                                  "geometry_valid": True, "already_preprocessed": False,
                                  "cloud_path": str(cloud_path), "cloud_sha256": cloud_sha,
                                  "roll": roll, "pitch": pitch, "yaw": yaw, "speed_mps": 0,
                                  "scan_stamp_ns": stamp, "scan_effective_stamp_ns": stamp, "scan_arrival_ns": stamp,
                                  "pose_evidence": {}, "baseline": d, "candidate": d,
                                  "time_role": "trigger_surrogate_not_measurement_time",
                                  "state_role": "paired_capture_attitude_no_live_arrival_evidence",
                                  "repetition_role": "stationary_snapshot_warmup_not_tracking"})
        truths.append({"capture_key": name, "instance_path": "truth/" + name + ".instance.i32",
                       "semantic_path": "truth/" + name + ".semantic.i32", **evidence})
    for filename, rows in (("snapshots.jsonl", snapshots), ("truth.jsonl", truths)):
        (args.output / filename).write_text("".join(canonical(r) + "\n" for r in rows))
    inputs = [("capture_manifest", args.capture_run / "capture_manifest.jsonl"),
              ("capture_meta", args.capture_run / "meta.json"), ("capture_summary", args.capture_run / "summary.json"),
              ("sensor_profile", args.sensor_profile), ("reference_setting", args.setting)]
    manifest = {"schema": "asmc-capture-admission-v1", "runtime_connected": False,
                "capture_run_id": args.capture_run.name, "frames": len(frames), "snapshots": len(snapshots),
                "repetitions": args.repetitions, "counts": dict(counters),
                "source_roots": {"lidar": str(args.lidar_root.resolve()), "state": str(args.state_root.resolve())},
                "sources": source_entries, "frames_evidence": frames,
                "inputs": {k: {"path": str(p.resolve()), "sha256": sha(p)} for k, p in inputs},
                "artifacts": {str(p.relative_to(args.output)): sha(p) for p in args.output.rglob("*") if p.is_file()},
                "tools_sha256": {p.name: sha(p) for p in Path(__file__).parent.glob("*.py")},
                "source_xyz_unchanged": True, "declared_axes": "sensor_origin_FLU_no_y_flip",
                "profile_provenance": "user-declared Autonav v3; no sensor JSON embedded in Capture",
                "limits": ["Native IMU timestamps are not repaired or used as scan measurement time",
                           "Controlled missing/empty camera fixtures are not recorded YOLO latency",
                           "Five identical-input warmups are not continuous vehicle/object motion",
                           "Structure semantic 109 is not wall or road-obstacle ground truth",
                           "Point preservation uses Instance IDs; questionable boxes do not approve OBB accuracy",
                           "Filter forced in isolated harness; valid GPS does not prove tunnel runtime routing"]}
    # Detect concurrent simulator overwrite/reset before admitting fixed evidence.
    for item in source_entries:
        if sha(Path(manifest["source_roots"][item["root"]]) / item["path"]) != item["sha256"]:
            raise ValueError("source changed during admission")
    (args.output / "manifest.json").write_text(json.dumps(manifest, sort_keys=True, indent=2) + "\n")
    print(canonical({k: manifest[k] for k in ("frames", "snapshots", "counts")}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("capture-run", "lidar-root", "state-root", "sensor-profile", "setting", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--repetitions", type=int, default=5)
    args = parser.parse_args()
    if args.repetitions < 3:
        parser.error("at least three stationary repetitions needed for pinned wall persistence")
    admit(args)
