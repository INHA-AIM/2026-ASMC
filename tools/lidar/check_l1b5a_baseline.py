#!/usr/bin/env python3
"""Verify fixed L1-B5-A evidence, including non-adoption and Capture exclusion."""
import argparse
import json
from pathlib import Path

import yaml

from clustering_compare.projection import sha
from clustering_compare.verify import verify

REPO = Path(__file__).resolve().parents[2]


def check(data):
    fixed = yaml.safe_load((REPO / "config/baselines/lidar_l1b5a_result_20260929.yaml").read_text())
    assert not fixed["runtime_connected"] and not fixed["result"]["deployment_accepted"]
    assert sha(REPO / fixed["settings_path"]) == fixed["settings_sha256"]
    root = data / fixed["data_relative_root"]
    projection = root / fixed["projection"]["path"]
    for name in ("manifest", "summary"):
        assert sha(projection / (name+".json")) == fixed["projection"][name+"_sha256"]
    p = json.loads((projection / "summary.json").read_text())
    assert not p["admitted"] and p["frames"] == 105 and p["admitted_frames"] == 97
    with (projection / "frames.jsonl").open() as stream:
        assert sorted(r["sequence"] for r in map(json.loads,stream) if not r["admitted"]) == fixed["projection"]["rejected_sequences"]
    for name in ("capture", "synthetic", "extra", "stationary"):
        f = fixed[name]
        s = verify(root / f["run"], root / f["repeat"] if "repeat" in f else None)
        assert s["stable_geometry_sha256"] == f["stable_geometry_sha256"]
        for key in ("calls", "measured", "aos_evaluated"):
            assert s["counts"][key] == f[key]
        for key in ("off_parity_errors", "historical_parity_errors", "lineage_errors", "count_errors", "intensity_errors",
                    "future_pose_uses", "future_camera_uses"):
            assert s["counts"].get(key,0) == 0
        m = json.loads((root / f["run"] / "manifest.json").read_text())
        assert m["reference_provenance"]["revision"] == fixed["reference_revision"]
        assert m["reference_provenance"]["travel_revision"] == fixed["travel_revision"]
        if "run_manifest_sha256" in f:
            assert sha(root / f["run"] / "manifest.json") == f["run_manifest_sha256"]
        if "repeat_manifest_sha256" in f:
            assert sha(root / f["repeat"] / "manifest.json") == f["repeat_manifest_sha256"]
        if "scores" in f:
            for folder in (f["scores"],f["repeat_scores"]):
                assert sha(root / folder / "summary.json") == f["score_summary_sha256"]
                scores = json.loads((root / folder / "summary.json").read_text())
                assert not scores["deployment_accepted"]
                if name == "capture":
                    assert scores["aos_status"] == "blocked_projection_admission"
                    assert scores["transitions"]["raw_euclidean:baseline_success_losses"] == 0
                    assert scores["transitions"]["raw_euclidean:recovered_sufficient_failures"] == 0
                    for group in scores["groups"]:
                        assert group["accepted"] == f["accepted_per_condition"][group["class"]]
                else:
                    assert scores["transitions"]["travel_aos:baseline_success_losses"] == f["aos_baseline_success_losses"]
                    assert scores["transitions"]["travel_aos:recovered_sufficient_failures"] == f["aos_recovered_sufficient_failures"]
        else:
            p = json.loads((root / f["parity"]).read_text())
            assert p["historical_parity"]["differences"] == 0 and p["historical_parity"]["measured"] == 924
    print("L1-B5-A verified: fixed evidence and repeat geometry, preserved baselines, failed projection and non-adoption")


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--data-root",type=Path,required=True)
    check(p.parse_args().data_root)
