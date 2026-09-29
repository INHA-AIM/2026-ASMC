#!/usr/bin/env python3
"""Verify immutable artifacts and repeatability, not research/deployment acceptance."""
import argparse
import json
from pathlib import Path

if __package__:
    from .projection import sha
    from .run import old_shape, stable
else:
    from projection import sha
    from run import old_shape, stable

HISTORICAL = ("labels", "kept_indices", "objects", "leveling", "leveled_xyzi_fnv1a64",
              "clusters", "final_objects", "protected_indices", "protected_points",
              "ground_removed", "wall_removed", "ceiling_removed", "kept_points")


def historical_parity(run, previous):
    calls = measured = differences = 0
    with previous.open() as old, (run / "geometry.jsonl").open() as actual:
        prior = iter(map(json.loads, old))
        for r in map(json.loads, actual):
            p = next(prior, None)
            if p is None or r["input_id"] != p["input_id"] or r["measurement"] != p["measurement"]:
                raise ValueError("historical input/order differs")
            calls += 1
            if "skipped_reason" in r:
                differences += r.get("skipped_reason") != p.get("skipped_reason")
            elif r["measurement"]:
                measured += 1
                expected = stable({k: p["baseline"][k] for k in HISTORICAL})
                differences += old_shape(stable(r["baseline"]), expected) != expected
        if next(prior, None) is not None:
            raise ValueError("historical output has extra rows")
    if differences:
        raise ValueError("historical frozen geometry differs")
    return {"calls": calls, "measured": measured, "differences": differences,
            "historical_geometry_sha256": sha(previous), "run_manifest_sha256": sha(run / "manifest.json")}


def verify(run, repeat=None):
    manifest = json.loads((run / "manifest.json").read_text())
    if manifest["runtime_connected"]:
        raise ValueError("operational connection forbidden")
    for name, value in manifest["artifacts"].items():
        if sha(run / name) != value["sha256"]:
            raise ValueError("experiment artifact changed: " + name)
    s = json.loads((run / "summary.json").read_text())
    if not s["passed_preservation_and_parity"] or s["deployment_accepted"]:
        raise ValueError("invalid preservation/deployment state")
    if s["kind"] == "capture" and not s["projection_admitted_capture"] and s["counts"]["aos_evaluated"]:
        raise ValueError("AOS bypassed global Capture admission gate")
    if repeat:
        other = verify(repeat)
        if s["stable_geometry_sha256"] != other["stable_geometry_sha256"]:
            raise ValueError("repeat geometry/OBB/lineage differs")
    return s


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--run", type=Path, required=True)
    p.add_argument("--repeat", type=Path)
    p.add_argument("--historical", type=Path)
    p.add_argument("--report", type=Path)
    a = p.parse_args()
    s = verify(a.run, a.repeat)
    result = {"verified": True, "kind": s["kind"], "counts": s["counts"],
              "stable_geometry_sha256": s["stable_geometry_sha256"]}
    if a.historical:
        result["historical_parity"] = historical_parity(a.run, a.historical)
    if a.report:
        if a.report.exists():
            raise ValueError("report exists")
        a.report.write_text(json.dumps(result, sort_keys=True, indent=2)+"\n")
    print(json.dumps(result,sort_keys=True))
