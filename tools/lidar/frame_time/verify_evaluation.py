#!/usr/bin/env python3
"""Validate the immutable L1-B1 evidence and current evaluated source; read-only."""
import argparse
import hashlib
import json
from pathlib import Path

import yaml


def sha(path):
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(4*1024*1024), b""):
            h.update(chunk)
    return h.hexdigest()


def verify(data, repo, manifest):
    m = yaml.safe_load(manifest.read_text())
    path = data / m["evaluation"]["report_path"]
    assert sha(path) == m["evaluation"]["report_sha256"], "report changed"
    r = json.loads(path.read_text())
    assert r["status"] == "passed_isolated_frame_time_only" and not r["runtime_changed"]
    assert r["baseline_revisions"] == m["revisions"]
    assert r["bag_clouds_checked"] == 2562 and r["capture_samples"] == 100
    assert r["synthetic_frames"] == 11 and r["synthetic_max_error"] <= 1e-6
    assert r["cloud_roundtrip_max_m"] <= 1e-6 and r["no_extrapolation"]
    assert r["max_pose_gap_ns"] == 100000000
    for entry in (r["bag"], r["scene_manifest"], r["capture_stage_manifest"]):
        assert sha(data / entry["path"]) == entry["sha256"], entry["path"]
    profile = r["sensor_profile"]
    assert sha(repo / profile["repo_path"]) == profile["sha256"], "sensor profile changed"
    for name, expected in r["source_sha256"].items():
        assert sha(Path(__file__).parent / name) == expected, "evaluated source changed: " + name
    for name, expected in r["artifact_sha256"].items():
        assert sha(path.parent / name) == expected, "artifact changed: " + name
    rows = [json.loads(line) for line in (path.parent / "snapshots.jsonl").read_text().splitlines()]
    outputs = [json.loads(line) for line in (path.parent / "results.jsonl").read_text().splitlines()]
    assert len(rows) == len(outputs) == 2673
    for row, out in zip(rows, outputs):
        assert row["input_id"] == out["input_id"] and row["time"]["stamp_ns"] == out["stamp_ns"]
        assert all(a <= row["time"]["processing_ns"] for a in out["pose_sample_arrivals"])
        if "cloud_path" in row:
            p = data / Path(row["cloud_path"]).relative_to("/data")
            assert sha(p) == row["cloud_sha256"], "point cloud changed"
            assert out["cloud"]["input_points"] == out["cloud"]["output_points"]
            assert out["cloud"]["intensity_bit_mismatches"] == 0
    canonical = json.dumps(strip_timing(outputs), sort_keys=True, allow_nan=False)
    assert hashlib.sha256(canonical.encode()).hexdigest() == r["deterministic_geometry_status_sha256"]
    audit = m["references"]
    assert sha(data / audit["path"]) == audit["sha256"]
    refs = json.loads((data / audit["path"]).read_text())
    assert refs["candidate_count"] == 24 and refs["new_reference_count"] == 2
    assert len(refs["repositories"]) == 26
    print("L1-B1 verified: source/input/output SHA, 2673 observations, causal poses, XYZI and fixed revisions")


def strip_timing(value):
    if isinstance(value, dict):
        return {k: strip_timing(v) for k, v in value.items() if "timing" not in k}
    if isinstance(value, list):
        return [strip_timing(v) for v in value]
    return value


def main():
    repo = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-root", required=True, type=Path)
    parser.add_argument("--manifest", type=Path, default=repo / "config/baselines/lidar_l1b1_20260927.yaml")
    args = parser.parse_args()
    verify(args.data_root.resolve(), repo, args.manifest)


if __name__ == "__main__":
    main()
