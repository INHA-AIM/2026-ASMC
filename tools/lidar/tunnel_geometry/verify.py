#!/usr/bin/env python3
"""Read-only baseline/artifact verification, including repeated geometry and scope."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

import yaml
from git_evidence import git_output

REPO = Path(__file__).resolve().parents[3]


def sha(path):
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def map_path(value, data):
    path = Path(value)
    if value.startswith("/data/"):
        return data / value[6:]
    if value.startswith("/root/ws/"):
        return REPO / value[9:]
    return path


def verify(args):
    config = yaml.safe_load((REPO / "config/baselines/lidar_l1b4_20260929.yaml").read_text())
    for name, item in config["data"].items():
        if sha(args.data_root / item["path"]) != item["sha256"]:
            raise ValueError("baseline input SHA changed: " + name)
    diffs = git_output(REPO, ["diff", "c536c3c", "--", "src", "scripts", "config/morai"])
    if diffs:
        raise ValueError("operational/planning/sensor scope changed")
    summaries = []
    for directory in args.run:
        manifest = json.loads((directory / "manifest.json").read_text())
        summary = json.loads((directory / "summary.json").read_text())
        if manifest["runtime_connected"] or manifest["reference_revision"] != config["reference_revision"]:
            raise ValueError("runtime or reference contract changed")
        if sha(directory / "summary.json") != manifest["summary_sha256"]:
            raise ValueError("summary SHA changed")
        if sha(directory / "geometry.jsonl") != manifest["geometry_file_sha256"]:
            raise ValueError("geometry JSONL SHA changed")
        for name, expected in manifest["derived_clouds_sha256"].items():
            if sha(directory / "clouds" / name) != expected:
                raise ValueError("derived point/lineage artifact changed")
        if sha(directory / "reference_provenance.json") != manifest["provenance_sha256"]:
            raise ValueError("reference extraction provenance changed")
        for name, entry in manifest["inputs"].items():
            path = map_path(entry["path"], args.data_root)
            if name in ("executable", "policy_library") and not path.exists():
                continue  # Build binary is transient; its frozen SHA remains recorded.
            if sha(path) != entry["sha256"]:
                raise ValueError("run input SHA changed: " + name)
        for name, expected in manifest["tools_sha256"].items():
            if sha(Path(__file__).parent / name) != expected:
                raise ValueError("evaluator source SHA changed: " + name)
        for value, expected in manifest.get("source_clouds", {}).items():
            if sha(map_path(value, args.data_root)) != expected["sha256"]:
                raise ValueError("original point-cloud SHA changed")
        for key in ("candidate_off_errors", "historical_parity_errors", "future_pose_uses", "stamp_errors",
                    "intensity_errors", "point_count_errors", "negative_camera_mask_differences", "lineage_errors"):
            if summary["counts"].get(key, 0):
                raise ValueError("regression: " + key)
        summaries.append(summary)
    if args.repeat:
        a = json.loads((args.run[0] / "summary.json").read_text())
        b = json.loads((args.repeat / "summary.json").read_text())
        if a["geometry_sha256"] != b["geometry_sha256"]:
            raise ValueError("repeated geometry differs")
    print("L1-B4: baseline/artifact SHA, OFF parity, causal states, scope and repeated geometry verified")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument("--run", type=Path, action="append", required=True)
    parser.add_argument("--repeat", type=Path)
    verify(parser.parse_args())
