#!/usr/bin/env python3
"""Read-only single-cloud AIM admission checks; unverified attitude blocks filter replay."""
import argparse
import hashlib
import json
import os
from pathlib import Path

import numpy as np


def sha(path):
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def rotation(q):
    q = np.asarray(q, dtype=np.float64)
    norm = np.linalg.norm(q)
    if not np.isfinite(norm) or norm < 1e-10:
        raise ValueError("invalid calibration quaternion")
    x, y, z, w = q / norm
    return np.array([[1 - 2 * (y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
                     [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
                     [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]])


def audit(args):
    if args.output.exists():
        raise ValueError("output exists")
    summary = json.loads((args.audit / "summary.json").read_text())
    sessions = [json.loads(line) for line in (args.audit / "sessions.jsonl").open()]
    result = []
    for campaign, evidence in sorted(summary["selected_payload_checks"].items()):
        if evidence["status"] != "sample_native_ros_geometry_verified":
            continue
        sample = evidence["samples"][len(evidence["samples"]) // 2]
        native = sample["native"]
        path = (args.aim_root / native["path"]).resolve()
        path.relative_to(args.aim_root.resolve())
        if sha(path) != native["sha256"]:
            raise ValueError("AIM native source changed")
        session = max((s for s in sessions if native["path"].startswith(s["session"] + "/")),
                      key=lambda s: len(s["session"]))
        profile = summary["sensor_profiles"][session["sensor_profile_sha256"]]
        if profile["raw_lidar_axes"] != "FLU":
            raise ValueError("unverified raw axes")
        mount = profile["lidar_to_base"]
        cloud = np.fromfile(path, dtype="<f4").reshape(-1, 4)
        finite = np.isfinite(cloud[:, :3]).all(axis=1)
        valid = finite & (np.linalg.norm(cloud[:, :3], axis=1) > 1)
        xyz = cloud[valid, :3].astype(np.float64)
        R, t = rotation(mount["rotation_xyzw"]), np.array(mount["translation_xyz"])
        base = xyz @ R.T + t  # Declared AIM base datum, rear-axle equivalence unverified.
        roundtrip = (base - t) @ R
        error = float(np.abs(roundtrip - xyz).max()) if len(xyz) else None
        result.append({"campaign": campaign, "session": session["session"], "native": native,
                       "sensor_profile_sha256": session["sensor_profile_sha256"],
                       "raw_axes": "FLU", "encoded_FRU_applied": False,
                       "source_stamp_ns": sample["header"]["stamp_ns"],
                       "source_frame": sample["header"]["frame_id"], "point_count": len(cloud),
                       "finite_nonzero_points": int(valid.sum()),
                       "declared_sensor_to_base": mount, "mount_application_count": 1,
                       "roundtrip_max_error_m": error,
                       "sensor_bbox": [xyz.min(axis=0).tolist(), xyz.max(axis=0).tolist()],
                       "declared_base_bbox": [base.min(axis=0).tolist(), base.max(axis=0).tolist()],
                       "filter_replay_admitted": False,
                       "blockers": ["selected-cloud attitude not admitted by the existing audit",
                                    "AIM base datum is not independently established as rear axle"],
                       "camera4_used": False, "accuracy_gt_used": False,
                       "map_route_sha": session["map_route_sha"]})
    args.output.mkdir(parents=True)
    report = {"schema": "asmc-l1b4-aim-support-v1", "runtime_connected": False,
              "samples": result, "verified_single_clouds": len(result),
              "filter_replay_admitted": 0, "source_root_variable": "ASMC_AIM_DATA",
              "limits": ["Native format/rigid transform verification is not attitude or timing admission",
                         "No raw copy, no camera projection, no tracking/accuracy claims"]}
    (args.output / "summary.json").write_text(json.dumps(report, sort_keys=True, indent=2) + "\n")
    manifest = {"schema": "asmc-l1b4-aim-support-manifest-v1", "runtime_connected": False,
                "audit_summary_sha256": sha(args.audit / "summary.json"),
                "audit_sessions_sha256": sha(args.audit / "sessions.jsonl"),
                "tool_sha256": sha(Path(__file__)), "summary_sha256": sha(args.output / "summary.json"),
                "native_sources": [r["native"] for r in result]}
    (args.output / "manifest.json").write_text(json.dumps(manifest, sort_keys=True, indent=2) + "\n")
    print(json.dumps({k: report[k] for k in ("verified_single_clouds", "filter_replay_admitted")}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--aim-root", type=Path, default=os.environ.get("ASMC_AIM_DATA", "/mnt/d/asmc_aim/dataset"))
    parser.add_argument("--audit", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    audit(parser.parse_args())
