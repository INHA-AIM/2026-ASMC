#!/usr/bin/env python3
"""Postprocess frozen OBBs; wall proximity is a diagnostic, never accuracy GT."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path

import numpy as np


def digest(path):
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            value.update(chunk)
    return value.hexdigest()


def supported_distance(center, models):
    x, y, z = center
    distances = []
    for model in models:
        a, b, c, d = model["plane"]
        horizontal = float(np.hypot(a, b))
        norm = float(np.sqrt(a * a + b * b + c * c))
        if horizontal < 1e-8 or norm < 1e-8:
            continue
        # Matches pinned WallSegment tangent=(-b,a)/horizontal. No extrapolation.
        along = (-b * x + a * y) / horizontal
        if any(lo <= along <= hi for lo, hi in model["intervals"]):
            distances.append(abs(a * x + b * y + c * z + d) / norm)
    return min(distances) if distances else None


def summarize(rows):
    counts, scenes, flags = Counter(), Counter(), Counter()
    dimensions, distances, representatives = [], [], {}
    for row in rows:
        if "geometry" not in row or not row["measurement"] or "shadow" not in row["scene"]:
            continue
        scenes[row["scene"]] += 1
        objects = [o for o in row["geometry"]["objects"] if not o["removed"]]
        counts["final_objects"] += len(objects)
        if len(objects) >= 18 and row["scene"] not in representatives:
            representatives[row["scene"]] = {"input_id": row["input_id"], "objects": len(objects)}
        for obj in objects:
            dimensions.append(obj["dimensions"])
            for name, value in obj["flags"].items():
                flags[name] += bool(value)
            flags["camera_protected"] += obj["protected"]
            distance = supported_distance(obj["center"], row["wall_models"])
            counts["supported_centers" if distance is not None else "unsupported_centers"] += 1
            if distance is not None:
                distances.append(float(distance))
    for threshold, name in ((.3, "center_within_0p3m"), (.8, "center_within_0p8m")):
        counts[name] = sum(d <= threshold for d in distances)
    shape = dict(zip(("length", "width", "height"),
                     np.percentile(dimensions, [50, 95], axis=0).T.tolist())) if dimensions else {}
    return {"schema": "asmc-l1b4-fragments-v1", "scans_by_scene": dict(scenes),
            "counts": dict(counts), "flags": dict(flags), "dimension_p50_p95_m": shape,
            "representative_scans": representatives,
            "limits": ["OBB center proximity and filter flags are not wall/object ground truth",
                       "Measurements include only stationary shadow measurement-tag scans",
                       "Diagnostic thresholds do not modify deletion or promotion gates"]}


def main(args):
    if args.output.exists():
        raise ValueError("output exists; preserve previous reports")
    source = args.run / "geometry.jsonl"
    manifest = args.run / "manifest.json"
    frozen = json.loads(manifest.read_text())
    source_sha = digest(source)
    if source_sha != frozen["geometry_file_sha256"]:
        raise ValueError("geometry differs from frozen run")
    summary = summarize(map(json.loads, source.open()))
    args.output.mkdir(parents=True)
    report = args.output / "summary.json"
    report.write_text(json.dumps(summary, sort_keys=True, indent=2) + "\n")
    evidence = {"runtime_connected": False, "run_manifest_sha256": digest(manifest),
                "input_geometry_sha256": source_sha, "summary_sha256": digest(report),
                "analysis_tool_sha256": digest(Path(__file__)), "source_run": str(args.run),
                "evaluator_commit": frozen["evaluator_commit"]}
    (args.output / "manifest.json").write_text(json.dumps(evidence, sort_keys=True, indent=2) + "\n")
    print(json.dumps(summary, sort_keys=True))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    main(parser.parse_args())
