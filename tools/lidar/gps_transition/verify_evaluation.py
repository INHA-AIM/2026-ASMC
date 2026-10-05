#!/usr/bin/env python3
"""Verify pinned L1-B2 source/evidence without replaying or rewriting the bag."""

import argparse
import hashlib
import json
from pathlib import Path

import yaml


REPO = Path(__file__).resolve().parents[3]
MANIFEST = REPO / "config/baselines/lidar_l1b2_20260927.yaml"


def sha(path):
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(4 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def verify(data_root=None):
    baseline = yaml.safe_load(MANIFEST.read_text())
    assert baseline["schema"] == "asmc-lidar-l1b2-baseline-v1"
    assert baseline["runtime_connected"] is False
    assert baseline["revisions"]["main"].startswith("1ce434d")
    assert baseline["revisions"]["son"].startswith("6435af1")
    assert baseline["revisions"]["seojun"].startswith("8b3f0b1")
    for rel, expected in baseline["source_sha256"].items():
        assert sha(REPO / rel) == expected, rel
    if data_root is None:
        return "source and pinned contract verified (data skipped)"
    audit_path = data_root / baseline["evaluation"]["audit_path"]
    snapshot_path = data_root / baseline["evaluation"]["snapshot_path"]
    assert sha(audit_path) == baseline["evaluation"]["audit_sha256"]
    assert sha(snapshot_path) == baseline["evaluation"]["snapshot_sha256"]
    audit, snapshot = json.loads(audit_path.read_text()), json.loads(snapshot_path.read_text())
    assert audit["bag"]["sha256"] == baseline["evaluation"]["bag_sha256"]
    assert audit["scene_manifest_sha256"] == baseline["evaluation"]["scene_manifest_sha256"]
    assert audit["topic_messages"] == baseline["acceptance"]["topic_messages"]
    assert len(snapshot["scenarios"]) == baseline["acceptance"]["synthetic_cases"]
    assert snapshot["source_result_sha256"] == baseline["evaluation"]["l1b1_result_sha256"]
    expected = baseline["acceptance"]["map_status_by_case"]
    assert {r["case"]: r["map"]["status"] for r in snapshot["scenarios"]} == expected
    for name in ("shadow_A126_entry", "shadow_A126_middle"):
        row = audit["scenes"][name]["gps"]
        assert row["status_1"] == row["zero_lat_lon"] == 176
        assert row["gate_invalid"] == 176
    assert sum(row["scan_gate"].get("map_pose_not_assessed", 0)
               for row in audit["scenes"].values()) == audit["topic_messages"]["/velodyne_points"]
    assert all(r["local"]["frame"] == "base_link" and r["map"]["frame"] == "map"
               for r in snapshot["scenarios"])
    assert all(r["stamp_ns"] is None or r["stamp_ns"] == snapshot["scenarios"][0]["stamp_ns"]
               for r in snapshot["scenarios"])
    return "source, evidence SHA, GPS status/zero, fixed frames and 11 cases verified"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", type=Path)
    args = parser.parse_args()
    print("L1-B2:", verify(args.data_root))


if __name__ == "__main__":
    main()
