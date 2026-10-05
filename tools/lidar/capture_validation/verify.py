#!/usr/bin/env python3
"""Read-only evidence hashes, pinned run parity, repeated scoring and scope."""
import argparse
import json
from pathlib import Path
import subprocess

if __package__:
    from .admit import sha
else:
    from admit import sha

REPO = Path(__file__).resolve().parents[3]


def verify(args):
    manifest = json.loads((args.admission / "manifest.json").read_text())
    if manifest["runtime_connected"]:
        raise ValueError("runtime connected")
    for entry in manifest["sources"]:
        path = Path(manifest["source_roots"][entry["root"]]) / entry["path"]
        if path.stat().st_size != entry["bytes"] or sha(path) != entry["sha256"]:
            raise ValueError("original Capture changed: " + str(path))
    for key, entry in manifest["inputs"].items():
        if sha(Path(entry["path"])) != entry["sha256"]:
            raise ValueError("admission input changed: " + key)
    for name, expected in manifest["artifacts"].items():
        if sha(args.admission / name) != expected:
            raise ValueError("derived input changed: " + name)
    for name, expected in manifest["tools_sha256"].items():
        if sha(Path(__file__).parent / name) != expected:
            raise ValueError("admission tool changed: " + name)
    summary = json.loads((args.scores / "summary.json").read_text())
    score_manifest = json.loads((args.scores / "manifest.json").read_text())
    if summary["deploy_accepted"] or score_manifest["runtime_connected"]:
        raise ValueError("incomplete evidence cannot approve deployment")
    for name, expected in score_manifest["artifacts"].items():
        if sha(args.scores / name) != expected:
            raise ValueError("score artifact changed: " + name)
    for name, expected in score_manifest["tools_sha256"].items():
        if sha(Path(__file__).parent / name) != expected:
            raise ValueError("score tool changed: " + name)
    if sha(args.admission / "manifest.json") != score_manifest["admission_manifest_sha256"] or \
            sha(args.run / "manifest.json") != score_manifest["run_manifest_sha256"] or \
            sha(args.run / "geometry.jsonl") != score_manifest["geometry_jsonl_sha256"]:
        raise ValueError("score provenance changed")
    if summary["counts"].get("negative_camera_geometry_mask_differences", 0):
        raise ValueError("empty/missing changed geometry candidate mask")
    if args.repeat_run:
        a = json.loads((args.run / "summary.json").read_text())
        b = json.loads((args.repeat_run / "summary.json").read_text())
        if a["geometry_sha256"] != b["geometry_sha256"]:
            raise ValueError("non-deterministic geometry")
    if args.repeat_scores:
        for name in ("summary.json", "objects.jsonl", "frames.jsonl"):
            if sha(args.scores / name) != sha(args.repeat_scores / name):
                raise ValueError("non-deterministic score: " + name)
    # Environment copy; never mutate process env to query git.
    env = dict(__import__("os").environ, GIT_OPTIONAL_LOCKS="0")
    result = subprocess.run(["git", "diff", "c536c3c", "--", "src", "scripts", "config/morai"],
                            cwd=REPO, env=env, check=True, capture_output=True, text=True)
    if result.stdout:
        raise ValueError("operating/planning/sensor scope changed")
    print("Capture source/derived/score SHA, empty/missing mask parity, repeated geometry/score, operating scope verified")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("admission", "run", "scores"):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--repeat-run", type=Path)
    parser.add_argument("--repeat-scores", type=Path)
    verify(parser.parse_args())
