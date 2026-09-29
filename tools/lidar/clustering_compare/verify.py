#!/usr/bin/env python3
"""Verify immutable artifacts and repeatability, not research/deployment acceptance."""
import argparse
import json
from pathlib import Path

if __package__:
    from .projection import sha
else:
    from projection import sha


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
    a = p.parse_args()
    s = verify(a.run, a.repeat)
    print(json.dumps({"verified": True, "kind": s["kind"], "counts": s["counts"],
                      "stable_geometry_sha256": s["stable_geometry_sha256"]},sort_keys=True))
