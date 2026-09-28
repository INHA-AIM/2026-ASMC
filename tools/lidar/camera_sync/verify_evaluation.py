#!/usr/bin/env python3
"""Verify the fixed L1-B3 evidence and its provenance without modifying it."""
import argparse
import hashlib
import json
import subprocess
from pathlib import Path

import yaml


REPO = Path(__file__).resolve().parents[3]
BASELINE = REPO / "config/baselines/lidar_l1b3_20260928.yaml"


def sha(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def verify(data_root=None):
    fixed = yaml.safe_load(BASELINE.read_text())
    assert fixed["schema"] == "asmc-lidar-l1b3-isolated-v1"
    assert fixed["runtime_connected"] is False
    assert fixed["acceptance"]["actual_25ms_geometry_test_passed"] is False
    for rel, expected in fixed["source_sha256"].items():
        revision = fixed.get("historical_source_revision")
        if revision:
            content = subprocess.check_output(["git", "--git-dir=" + str(REPO / ".git"),
                                               "show", revision + ":" + rel])
            assert hashlib.sha256(content).hexdigest() == expected, rel
        else:
            assert sha(REPO / rel) == expected, rel
    if data_root is None:
        return "source and non-deployment contract verified (data skipped)"
    data = fixed["evaluation"]
    teammate = data_root / "lidar_assessment/20260926/jaeho_source/src/lidar+sensorfusion/src/Main"
    teammate_paths = {
        "TunnelFilterRos.cpp": teammate / "src/Lidar/TunnelFilter/TunnelFilterRos.cpp",
        "TunnelFilterCore.cpp": teammate / "src/Lidar/TunnelFilter/TunnelFilterCore.cpp",
        "tunnel_filter_offline_eval.cpp": teammate / "tools/tunnel_filter_offline_eval.cpp",
    }
    for name, expected_sha in fixed["pinned_teammate_source_sha256"].items():
        assert sha(teammate_paths[name]) == expected_sha, name
    for key, source in (("bag", "bag_path"),
                        ("scene_manifest", "scene_manifest_path"),
                        ("offline_130ms_reference", "offline_130ms_reference_path")):
        path = data_root / data[source]
        assert sha(path) == data[key + "_sha256"], str(path)
    root = data_root / data["run_path"]
    repeat = data_root / data["repeat_path"]
    for filename, expected in data["artifact_sha256"].items():
        assert sha(root / filename) == expected, filename
        assert sha(repeat / filename) == expected, filename + " repeat"
    manifest = json.loads((root / "manifest.json").read_text())
    assert manifest["bag_sha256"] == data["bag_sha256"]
    assert manifest["reference_130ms_sha256"] == data["offline_130ms_reference_sha256"]
    summary = json.loads((root / "summary.json").read_text())
    expected = fixed["acceptance"]
    all_rows = summary["all"]
    assert all_rows["scans"] == expected["scan_count"]
    assert all_rows["baseline_matches"] == expected["baseline_matched"]
    assert all_rows["candidate_matches"] == expected["candidate_matched"]
    assert all_rows["recovered_matches"] == expected["recovered_matches"]
    assert all_rows["candidate_reasons"]["matched_boxes"] == expected["candidate_matched_boxes"]
    assert all_rows["candidate_reasons"]["matched_empty"] == expected["candidate_matched_empty"]
    assert all_rows["candidate_reasons"]["timeout"] == expected["candidate_timeout"]
    assert summary["future_yolo_use"] == expected["candidate_future_use"] == 0
    assert len(summary["invalid_yolo"]) == fixed["matching"]["invalid_yolo_stamps"]
    count = 0
    with (root / "decisions.jsonl").open() as stream:
        for line in stream:
            row = json.loads(line)
            count += 1
            base, trial = row["baseline"], row["candidate"]
            assert base["source_index"] == trial["source_index"]
            assert base["scan_digest"] == trial["scan_digest"]
            assert base["scan_stamp_ns"] == trial["scan_stamp_ns"]
            assert 0 <= trial["wait_ns"] <= 25_000_000
            assert base["wait_ns"] == 0
            for decision in (base, trial):
                selected = decision["selected_yolo_arrival_ns"]
                assert selected is None or selected <= decision["decision_ns"]
                if selected is not None:
                    assert decision["yolo_offset_ns"] <= 50_000_000
            assert row["point_input"]["point_count"] > 0
    assert count == expected["scan_count"]
    return "source/input/artifact SHA, repeat determinism, 2562 causal decisions verified"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-root", type=Path)
    args = parser.parse_args()
    print("L1-B3:", verify(args.data_root))


if __name__ == "__main__":
    main()
