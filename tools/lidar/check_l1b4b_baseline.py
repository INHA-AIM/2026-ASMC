#!/usr/bin/env python3
"""Bind the checked-in L1-B4-B baseline to immutable local Capture artifacts."""
import argparse
import json
from pathlib import Path

import yaml
from capture_validation.admit import sha

REPO = Path(__file__).resolve().parents[2]


def check(data):
    fixed = yaml.safe_load((REPO / "config/baselines/lidar_l1b4b_capture_20260929.yaml").read_text())
    assert not fixed["runtime_connected"] and not fixed["result"]["deployment_accepted"]
    root = data / fixed["evaluation"]["data_relative_root"]
    paths = fixed["evaluation"]
    source = fixed["source_run"]
    assert sha(data / source["path"] / "capture_manifest.jsonl") == source["manifest_sha256"]
    admission = root / paths["admission"]
    for filename, key in (("manifest.json", "admission_manifest_sha256"),
                          ("snapshots.jsonl", "snapshots_sha256"), ("truth.jsonl", "truth_sha256")):
        assert sha(admission / filename) == paths[key], filename
    admitted = json.loads((admission / "manifest.json").read_text())
    assert admitted["frames"] == source["frames"] == 105
    assert admitted["counts"]["raw_points"] == source["raw_slots"]
    assert admitted["counts"]["instance_labelled_points"] == source["instance_labelled_points"]
    assert admitted["repetitions"] == fixed["comparison"]["stationary_repetitions_per_snapshot"] == 5
    result = fixed["result"]
    for run in (paths["run"], paths["repeat_run"]):
        s = json.loads((root / run / "summary.json").read_text())
        m = json.loads((root / run / "manifest.json").read_text())
        assert s["geometry_sha256"] == paths["geometry_status_sha256"]
        assert m["reference_revision"] == fixed["reference_revision"] and not m["runtime_connected"]
        assert s["counts"]["frames"] == 1050 and s["counts"]["valid"] == 1050
        for key in ("candidate_off_errors", "stamp_errors", "lineage_errors", "intensity_errors", "point_count_errors"):
            assert s["counts"].get(key, 0) == 0, key
    for scores in (paths["scores"], paths["repeat_scores"]):
        for filename, key in (("summary.json", "score_summary_sha256"),
                              ("objects.jsonl", "object_scores_sha256"), ("frames.jsonl", "frame_scores_sha256")):
            assert sha(root / scores / filename) == paths[key], filename
        s = json.loads((root / scores / "summary.json").read_text())
        assert not s["deploy_accepted"] and s["frames"] == 105 and s["controlled_conditions"] == 210
        assert s["counts"].get("baseline_success_losses", 0) == result["baseline_success_losses"] == 0
        assert s["counts"]["negative_camera_geometry_mask_differences"] == result["negative_camera_mask_differences"] == 0
        for r in s["by_class_camera_mode"]:
            kind = r["class"]
            assert r["observations"] == source["object_observations"][kind]
            assert r["accepted"] == result["accepted_visible_point_cluster_observations_all_policies"][kind]
            assert r["sufficient_voxel_observations"] == result["sufficient_voxel_observations"][kind]
            assert r["wall_stage_target_points"] == r["wall_stage_kept"]
        totals = s["frame_totals"]
        assert totals["empty"]["baseline"]["final_objects"] == result["final_objects_per_105_scenes"]["immediate_empty"]
        assert totals["missing"]["baseline"]["final_objects"] == result["final_objects_per_105_scenes"]["missing_or_policy_only"]
        for state in ("empty", "missing"):
            assert totals[state]["geometry"]["final_objects"] == result["final_objects_per_105_scenes"]["geometry"]
    print("L1-B4-B baseline: fixed GT/geometry/score SHA, two-camera-condition counts and non-deployment contract verified")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-root", type=Path, required=True)
    check(parser.parse_args().data_root)
