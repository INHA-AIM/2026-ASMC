#!/usr/bin/env python3
"""Admit a geometric VLP16 projection without making up measured ring/time fields."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import yaml


def sha(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for part in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(part)
    return h.hexdigest()


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)


def project(points, config):
    """Match the bounded TRAVEL row/column conventions; retain rejection reasons."""
    xyz = np.asarray(points)[:, :3].astype(np.float64)
    finite = np.isfinite(xyz).all(axis=1)
    ranges = np.linalg.norm(xyz, axis=1)
    elevation = np.degrees(np.arctan2(xyz[:, 2], np.linalg.norm(xyz[:, :2], axis=1)))
    nominal = np.linspace(config["min_vertical_angle_deg"], config["max_vertical_angle_deg"],
                          config["vertical_channels"])
    errors = np.abs(elevation[:, None] - nominal)
    # Upper channel wins an exact tie, as in the pinned getRowIdx.
    rows = len(nominal) - 1 - np.argmin(errors[:, ::-1], axis=1)
    residual = errors[np.arange(len(xyz)), rows]
    angle = np.degrees(np.arctan2(xyz[:, 0], xyz[:, 1]))
    scaled = (angle - 90) / (360 / config["horizontal_columns"])
    # C++ round is half-away-from-zero, not NumPy's ties-to-even.
    rounded = np.copysign(np.floor(np.abs(scaled) + .5), scaled)
    columns = (-rounded + config["horizontal_columns"] // 2)
    columns = np.where(columns >= config["horizontal_columns"],
                       columns - config["horizontal_columns"], columns)
    columns = np.nan_to_num(columns, nan=-1).astype(np.int64)
    reasons = np.full(len(xyz), "eligible", dtype=object)
    reasons[~finite] = "nonfinite"
    reasons[finite & ((ranges < config["min_range_m"]) | (ranges > config["max_range_m"]))] = "range"
    valid = reasons == "eligible"
    reasons[valid & (residual > config["angle_uncertainty_deg"])] = "ambiguous_channel"
    valid = reasons == "eligible"
    reasons[valid & ((columns < 0) | (columns >= config["horizontal_columns"]))] = "column_bounds"
    cells = rows * config["horizontal_columns"] + columns
    return {"rows": rows, "columns": columns, "cells": cells, "reasons": reasons,
            "ranges": ranges, "elevation": elevation, "residual": residual}


def audit_cloud(points, aos, admission):
    projected = project(points, aos)
    valid = np.isfinite(points[:, :3]).all(axis=1) & (projected["ranges"] > .1)
    pattern = np.asarray(admission["expected_interleaved_elevations_deg"])
    slots = len(points) == admission["expected_capture_slots"]
    expected = pattern[np.arange(len(points)) % len(pattern)]
    nominal = np.sort(pattern)
    nearest = nominal[projected["rows"]]
    mismatch = valid & (nearest != expected)
    residual = np.abs(projected["elevation"][valid] - expected[valid])
    step_errors = np.empty(0)
    if slots:
        azimuth = np.degrees(np.arctan2(points[:, 1], points[:, 0])).reshape(-1, len(pattern))
        present = valid.reshape(-1, len(pattern))
        delta = (np.diff(azimuth, axis=0) + 180) % 360 - 180
        mask = present[:-1] & present[1:]
        step_errors = np.abs(np.abs(delta[mask]) - 360 / aos["horizontal_columns"])
    p99 = float(np.percentile(residual, 99)) if len(residual) else None
    p95_step = float(np.percentile(step_errors, 95)) if len(step_errors) else None
    mismatch_fraction = float(mismatch.sum() / max(1, valid.sum()))
    checks = {"slot_count": slots, "nonempty": bool(valid.any()),
              "channel_layout": mismatch_fraction <= admission["slot_channel_mismatch_fraction_max"],
              "elevation": p99 is not None and p99 <= admission["elevation_residual_p99_deg_max"],
              "azimuth_spacing": p95_step is not None and p95_step <=
              admission["adjacent_azimuth_step_residual_p95_deg_max"]}
    eligible = projected["reasons"] == "eligible"
    unique = len(np.unique(projected["cells"][eligible]))
    return {"slots": len(points), "valid_returns": int(valid.sum()),
            "channel_mismatches": int(mismatch.sum()), "channel_mismatch_fraction": mismatch_fraction,
            "elevation_error_p99_deg": p99, "azimuth_step_error_p95_deg": p95_step,
            "projection_eligible": int(eligible.sum()), "distinct_cells": unique,
            "cell_collision_points": int(eligible.sum()) - unique,
            "ambiguous_points": int(np.sum(projected["reasons"] == "ambiguous_channel")),
            "checks": checks, "admitted": all(checks.values())}


def run(args):
    if args.output.exists():
        raise ValueError("output exists; retain previous audit")
    fixed = yaml.safe_load(args.config.read_text())
    manifest = json.loads((args.admission / "manifest.json").read_text())
    if sha(args.admission / "manifest.json") != fixed["inputs"]["capture_admission_manifest_sha256"]:
        raise ValueError("Capture admission changed")
    frames = []
    snapshots = list(map(json.loads, (args.admission / "snapshots.jsonl").open()))
    clouds = {r["capture_key"]: r for r in snapshots}
    for key, row in sorted(clouds.items()):
        cloud = args.admission / row["cloud_path"]
        if sha(cloud) != row["cloud_sha256"]:
            raise ValueError("Capture cloud changed")
        points = np.fromfile(cloud, dtype="<f4").reshape(-1, 4)
        frames.append({"capture_key": key, "sequence": row["sequence"],
                       "cloud_sha256": sha(cloud), **audit_cloud(points, fixed["aos"], fixed["projection_admission"])})
    args.output.mkdir(parents=True)
    (args.output / "frames.jsonl").write_text("".join(canonical(r) + "\n" for r in frames))
    summary = {"schema": "asmc-l1b5a-projection-v1", "frames": len(frames),
               "admitted_frames": sum(r["admitted"] for r in frames),
               "admitted": len(frames) == manifest["frames"] and all(r["admitted"] for r in frames),
               "configuration_sha256": sha(args.config),
               "totals": {k: sum(r[k] for r in frames) for k in
                          ("slots", "valid_returns", "channel_mismatches", "ambiguous_points", "cell_collision_points")},
               "limits": ["Approximate angular projection; no measured ring or per-point time assertion",
                          "Cell collision exclusions and ambiguous points must remain scored",
                          "No labels or boxes used to choose projection or connection settings"]}
    (args.output / "summary.json").write_text(json.dumps(summary, sort_keys=True, indent=2) + "\n")
    (args.output / "manifest.json").write_text(json.dumps({
        "schema": "asmc-l1b5a-projection-manifest-v1", "runtime_connected": False,
        "admission_sha256": sha(args.admission / "manifest.json"), "config_sha256": sha(args.config),
        "tool_sha256": sha(Path(__file__)),
        "artifacts": {p.name: sha(p) for p in args.output.iterdir()}}, sort_keys=True, indent=2) + "\n")
    print(canonical(summary))
    if not summary["admitted"]:
        raise ValueError("projection preconditions unmet; do not run AOS on Capture")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("config", "admission", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    run(parser.parse_args())
