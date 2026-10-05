#!/usr/bin/env python3
"""Inventory existing MORAI capture LiDAR without importing/moving source files.

Samples are single-frame geometry/label evidence. Capture pauses are incompatible
with continuous-motion tracking evaluation. NumPy is needed only by this tool.
"""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path
import re

import numpy as np


SUFFIXES = ("Intensity.bin", "Semantic.bin", "Instance.bin", "Instance.txt", "Instance_8Points.txt")
FRAME_NAME = re.compile(r"^(.+)_(\d{6})_Intensity\.bin$")


def inspect_frame(path):
    prefix = path.name[:-len("_Intensity.bin")]
    files = {suffix: path.with_name(prefix + "_" + suffix) for suffix in SUFFIXES}
    missing = [suffix for suffix, source in files.items() if not source.is_file()]
    result = {"frame": prefix, "missing": missing}
    if missing:
        return result
    matrices = {}
    for suffix in SUFFIXES[:3]:
        source = files[suffix]
        if source.stat().st_size % 16:
            result["invalid_float4_file"] = suffix
            return result
        matrices[suffix] = np.fromfile(source, dtype="<f4").reshape(-1, 4)
    intensity, semantic, instance = (matrices[name] for name in SUFFIXES[:3])
    result["raw_points"] = len(intensity)
    result["xyz_exactly_equal"] = all(
        intensity.shape == other.shape and np.array_equal(intensity[:, :3], other[:, :3], equal_nan=True)
        for other in (semantic, instance))
    if not result["xyz_exactly_equal"]:
        return result
    valid = np.all(np.isfinite(intensity[:, :3]), axis=1) & (np.linalg.norm(intensity[:, :3], axis=1) > 1.)
    result["valid_points"] = int(valid.sum())
    result["intensity_min_max"] = [float(intensity[:, 3].min()), float(intensity[:, 3].max())] if len(intensity) else []
    values, counts = np.unique(semantic[valid, 3], return_counts=True)
    result["semantic_counts"] = {str(float(key)): int(count) for key, count in zip(values, counts)}
    values, counts = np.unique(instance[valid, 3], return_counts=True)
    per_instance = dict(zip(values, counts))
    boxes, malformed = [], []
    for line_number, line in enumerate(files["Instance.txt"].read_text().splitlines(), 1):
        tokens = line.split()
        if not tokens:
            continue
        if len(tokens) != 15:
            malformed.append(line_number)
            continue
        try:
            values = [float(value) for value in tokens[1:]]
        except ValueError:
            malformed.append(line_number)
            continue
        if not all(np.isfinite(values)):
            malformed.append(line_number)
            continue
        boxes.append({"class": tokens[0], "id": int(values[13]),
                      "points": int(per_instance.get(values[13], 0)),
                      "distance_m": values[9], "dimensions_m": values[6:9],
                      "dimensions_valid": all(value > 0 for value in values[6:9])})
    result["boxes"] = boxes
    result["malformed_box_lines"] = malformed
    result["source_intensity_sha256"] = hashlib.sha256(path.read_bytes()).hexdigest()
    result["source_box_sha256"] = hashlib.sha256(files["Instance.txt"].read_bytes()).hexdigest()
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--lidar-root", type=Path, required=True)
    parser.add_argument("--capture-root", type=Path, help="Existing capture_runs root (join by recorded custom_name)")
    parser.add_argument("--rename-history", type=Path, help="Explicit old_run_id/new_run_id JSONL; frame sets are checked")
    parser.add_argument("--run-prefix", action="append", help="Exact original filename prefix; repeatable")
    parser.add_argument("--max-samples", type=int, default=100, help="Evenly spread frame samples, default 100")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    if args.max_samples < 1:
        parser.error("max-samples must be positive")
    if args.output.exists():
        parser.error("output exists; choose a new report path")
    groups = defaultdict(list)
    ignored = 0
    for path in sorted(args.lidar_root.glob("*_Intensity.bin")):
        match = FRAME_NAME.match(path.name)
        if not match:
            ignored += 1
            continue
        if not args.run_prefix or match.group(1) in args.run_prefix:
            groups[match.group(1)].append(path)
    if not groups:
        parser.error("no six-digit capture frames matched")
    manifest_frames = {}
    manifest_sets = defaultdict(set)
    if args.capture_root:
        for source in sorted(args.capture_root.glob("*/capture_manifest.jsonl")):
            for line in source.read_text().splitlines():
                item = json.loads(line)
                if item.get("custom_name"):
                    manifest_frames[item["custom_name"]] = {"run": source.parent.name,
                                                            "ego": bool(item.get("state", {}).get("ego")),
                                                            "success": item.get("success", False)}
                    manifest_sets[source.parent.name].add(int(item["sequence"]))
    run_mapping = {}
    if args.rename_history:
        for line in args.rename_history.read_text().splitlines():
            item = json.loads(line)
            run_mapping[item["old_run_id"]] = item["new_run_id"]
    run_report = {}
    for prefix, paths in sorted(groups.items()):
        mapped = run_mapping.get(prefix, prefix)
        indices = {int(FRAME_NAME.match(path.name).group(2)) for path in paths}
        run_report[prefix] = {"frames": len(paths), "manifest_run": mapped,
                              "manifest_frame_set_equal": indices == manifest_sets[mapped] if args.capture_root else None,
                              "manifest_frames": len(manifest_sets[mapped]) if args.capture_root else None}
    all_paths = [path for paths in groups.values() for path in paths]
    indexes = np.linspace(0, len(all_paths) - 1, min(args.max_samples, len(all_paths)), dtype=int)
    samples = [inspect_frame(all_paths[int(index)]) for index in indexes]
    for sample in samples:
        if args.capture_root:
            prefix, sequence = sample["frame"].rsplit("_", 1)
            sample["manifest"] = manifest_frames.get(run_mapping.get(prefix, prefix) + "_" + sequence)
    classes, semantics = Counter(), Counter()
    boxes = []
    for sample in samples:
        classes.update(box["class"] for box in sample.get("boxes", []))
        semantics.update(sample.get("semantic_counts", {}))
        boxes.extend(sample.get("boxes", []))
    report = {"schema_version": "asmc-lidar-capture-inventory-1",
              "source_root": str(args.lidar_root), "source_untouched": True,
              "ignored_non_capture_intensity_files": ignored,
              "rename_history": str(args.rename_history) if args.rename_history else None,
              "runs": run_report,
              "total_capture_frames": len(all_paths), "sample_count": len(samples),
              "sample_class_counts": dict(sorted(classes.items())),
              "sample_semantic_point_counts": dict(sorted(semantics.items())),
              "sample_box_count": len(boxes),
              "sample_box_sparse_counts": {str(limit): sum(box["points"] < limit for box in boxes)
                                           for limit in (1, 5, 10, 20, 50)},
              "samples": samples,
              "scope": "Single-frame format and availability only; no detection accuracy or tracking score.",
              "cautions": ["Capture mode pauses motion; do not use adjacent capture frames for tracking/odometry.",
                           "Obstacle box dimensions need separate 8-corner consistency validation.",
                           "Ground/structure semantic values need explicit class policy before algorithm scoring.",
                           "Filename runs without manifest/approval (including rejected runs) are inventoried, not admitted as evaluation data."]}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x") as stream:
        json.dump(report, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
    print(json.dumps({key: report[key] for key in ("total_capture_frames", "sample_count", "sample_class_counts",
                                                  "sample_box_count", "sample_box_sparse_counts")}, indent=2))


if __name__ == "__main__":
    main()
