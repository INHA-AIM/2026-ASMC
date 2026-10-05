#!/usr/bin/env python3
"""Stage accepted capture samples and score offline ground masks. No ROS required."""
import argparse
from collections import Counter, defaultdict
import csv
import hashlib
import json
from pathlib import Path
import numpy as np

GROUND = frozenset((85, 127, 128, 136, 144, 170, 255))
OBSTACLE = frozenset((86, 109, 118, 125, 135, 145, 155, 164, 165, 175, 185))
STRUCTURE = frozenset((129, 132, 153, 178, 190))
POLICY = {"ground": sorted(GROUND), "obstacle": sorted(OBSTACLE),
          "structure_context_only": sorted(STRUCTURE), "unknown_excluded": [0, 10],
          "source": "workspace docs/competition/capture-lidar-data-spec.md section 7",
          "note": "Structure retention is descriptive, not wall/ceiling correctness. Ground includes painted pavement; asphalt is also reported separately."}


def write_json(path, value):
    with path.open("x") as stream:
        json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")


def roi_mask(points):
    xyz = points[:, :3]
    return (np.isfinite(points).all(axis=1) & (np.linalg.norm(xyz, axis=1) > 1)
            & (xyz[:, 0] >= -30) & (xyz[:, 0] <= 50) & (np.abs(xyz[:, 1]) <= 20)
            & (xyz[:, 2] >= -2) & (xyz[:, 2] <= 3))


def admitted(sample, inventory, rejected_names):
    manifest = sample.get("manifest") or {}
    prefix = sample["frame"].rsplit("_", 1)[0]
    run = inventory["runs"].get(prefix, {})
    return (not sample.get("missing") and sample.get("xyz_exactly_equal") is True
            and manifest.get("success") is True and manifest.get("ego") is True
            and run.get("manifest_frame_set_equal") is True
            and manifest.get("run") not in rejected_names and prefix not in rejected_names)


def stage(args):
    inventory = json.loads(args.inventory.read_text())
    source_root = Path(inventory["source_root"])
    rejected = {p.name for p in args.rejected_root.iterdir() if p.is_dir()}
    if args.output.exists():
        raise ValueError("stage output exists")
    samples = [s for s in inventory["samples"] if admitted(s, inventory, rejected)]
    if not samples:
        raise ValueError("no admitted samples")
    args.output.mkdir(parents=True)
    records = []
    for sample in samples:
        name = sample["frame"]
        if "/" in name or ".." in name:
            raise ValueError("invalid frame")
        arrays, sources = {}, {}
        for kind in ("Intensity", "Semantic", "Instance"):
            source = source_root / (name + "_" + kind + ".bin")
            content = source.read_bytes()
            if len(content) % 16:
                raise ValueError("not float4")
            arrays[kind] = np.frombuffer(content, dtype="<f4").reshape(-1, 4)
            sources[kind] = {"path": str(source), "sha256": hashlib.sha256(content).hexdigest(), "bytes": len(content)}
        if sources["Intensity"]["sha256"] != sample["source_intensity_sha256"]:
            raise ValueError("intensity changed since inventory")
        points = arrays["Intensity"]
        for kind in ("Semantic", "Instance"):
            if points.shape != arrays[kind].shape or not np.array_equal(points[:, :3], arrays[kind][:, :3], equal_nan=True):
                raise ValueError("point/label XYZ mismatch")
        keep = roi_mask(points)
        if not keep.any():
            raise ValueError("empty ROI")
        points[keep].tofile(args.output / (name + ".bin"))
        np.savez_compressed(args.output / (name + ".labels.npz"),
                            semantic=arrays["Semantic"][keep, 3], instance=arrays["Instance"][keep, 3])
        records.append({"frame": name, "run": sample["manifest"]["run"], "points": int(keep.sum()),
                        "sources": sources, "boxes": sample["boxes"]})
    (args.output / "frames.txt").write_text("".join(r["frame"] + "\n" for r in records))
    report = {"schema_version": "asmc-ground-staging-1", "inventory": str(args.inventory),
              "inventory_sha256": hashlib.sha256(args.inventory.read_bytes()).hexdigest(),
              "policy": POLICY, "roi": {"x_m": [-30, 50], "y_m": [-20, 20], "z_m": [-2, 3], "radius_min_m": 1},
              "frame": "sensor_mount x-forward y-left z-up; no IMU/voxel in this isolated benchmark",
              "admission": "successful capture + ego snapshot + exact run frame-set match + rejected-run exclusion",
              "rejected_names": sorted(rejected), "excluded_samples": len(inventory["samples"]) - len(records),
              "samples": records}
    write_json(args.output / "manifest.json", report)
    print(json.dumps({"staged_samples": len(records), "points": sum(r["points"] for r in records)}))


def counts(labels, status):
    if len(labels) != len(status) or not np.isin(status, [0, 1, 2, 3]).all():
        raise ValueError("invalid prediction mask")
    ground = np.isin(labels, list(GROUND)); obstacles = np.isin(labels, list(OBSTACLE))
    asphalt = labels == 127
    structure = np.isin(labels, list(STRUCTURE))
    return {"ground_points": int(ground.sum()), "ground_removed": int((ground & (status == 1)).sum()),
            "ground_retained": int((ground & (status == 2)).sum()),
            "asphalt_points": int(asphalt.sum()), "asphalt_removed": int((asphalt & (status == 1)).sum()),
            "obstacle_points": int(obstacles.sum()), "obstacle_retained": int((obstacles & (status == 2)).sum()),
            "obstacle_ground_removed": int((obstacles & (status == 1)).sum()),
            "obstacle_structure_removed": int((obstacles & (status == 3)).sum()),
            "structure_context_points": int(structure.sum()), "structure_retained": int((structure & (status == 2)).sum()),
            "unassigned_points": int((status == 0).sum()),
            "excluded_unknown_points": int((~(ground | obstacles | structure)).sum())}


def score(args):
    if args.output.exists():
        raise ValueError("score output exists")
    manifest = json.loads((args.stage / "manifest.json").read_text())
    timing = defaultdict(list)
    timing_frames = defaultdict(set)
    with (args.predictions / "timings.csv").open() as stream:
        for row in csv.DictReader(stream):
            milliseconds = float(row["milliseconds"])
            if not np.isfinite(milliseconds) or milliseconds < 0:
                raise ValueError("invalid timing")
            timing[row["algorithm"]].append(milliseconds)
            timing_frames[row["algorithm"]].add(row["frame"])
    expected_frames = {s["frame"] for s in manifest["samples"]}
    if any(frames != expected_frames for frames in timing_frames.values()):
        raise ValueError("timing frame set mismatch")
    near_ground_height = []
    for sample in manifest["samples"]:
        points = np.fromfile(args.stage / (sample["frame"] + ".bin"), dtype="<f4").reshape(-1, 4)
        with np.load(args.stage / (sample["frame"] + ".labels.npz")) as labels:
            radius = np.linalg.norm(points[:, :2], axis=1)
            selected = (labels["semantic"] == 127) & (radius >= 3) & (radius < 10)
        if selected.sum() >= 20:
            near_ground_height.append(float(-np.median(points[selected, 2])))
    results = {}
    for algorithm, values in timing.items():
        total, per_class, per_frame, boxes = Counter(), defaultdict(Counter), [], []
        for sample in manifest["samples"]:
            frame = sample["frame"]
            with np.load(args.stage / (frame + ".labels.npz")) as archive:
                semantic, instance = archive["semantic"], archive["instance"]
            mask = np.fromfile(args.predictions / (frame + "." + algorithm + ".mask"), dtype="u1")
            detail = counts(semantic, mask)
            total.update(detail); per_frame.append({"frame": frame, **detail})
            for value in sorted(OBSTACLE):
                selected = semantic == value
                per_class[str(value)].update({"points": int(selected.sum()), "retained": int((selected & (mask == 2)).sum())})
            for box in sample["boxes"]:
                selected = (instance == box["id"]) & np.isin(semantic, list(OBSTACLE))
                n = int(selected.sum())
                if n:
                    boxes.append({"frame": frame, "id": box["id"], "class": box["class"],
                                  "points_in_roi": n, "retained": int((selected & (mask == 2)).sum())})
        ratios = {}
        for name, numerator, denominator in (
                ("ground_removal_fraction", "ground_removed", "ground_points"),
                ("asphalt_removal_fraction", "asphalt_removed", "asphalt_points"),
                ("obstacle_retention_fraction", "obstacle_retained", "obstacle_points")):
            ratios[name] = total[numerator] / total[denominator] if total[denominator] else None
        results[algorithm] = {"counts": dict(total), "ratios": ratios, "obstacle_by_semantic": dict(per_class),
                              "timing_ms": {"p50": float(np.percentile(values, 50)), "p95": float(np.percentile(values, 95)),
                                            "p99": float(np.percentile(values, 99)), "maximum": max(values), "samples": len(values)},
                              "visible_boxes": len(boxes), "boxes_all_points_removed": sum(b["retained"] == 0 for b in boxes),
                              "sparse_boxes_under10": sum(b["points_in_roi"] < 10 for b in boxes),
                              "sparse_boxes_under10_all_removed": sum(b["points_in_roi"] < 10 and b["retained"] == 0 for b in boxes),
                              "per_frame": per_frame, "per_visible_box": boxes}
    if not results:
        raise ValueError("no timing rows")
    report = {"schema_version": "asmc-ground-benchmark-1", "policy": POLICY, "samples": len(manifest["samples"]),
              "patchworkpp_sha": "3e6903a1d5537a4cc2ace897b0bbb98a92d6014c", "results": results,
              "asphalt_height_diagnostic": {"samples": len(near_ground_height), "range_m": [3, 10],
                                            "per_frame_statistic": "negative median sensor-frame z, asphalt127, >=20points",
                                            "p05_m": float(np.percentile(near_ground_height, 5)) if near_ground_height else None,
                                            "p50_m": float(np.percentile(near_ground_height, 50)) if near_ground_height else None,
                                            "p95_m": float(np.percentile(near_ground_height, 95)) if near_ground_height else None},
              "limitations": ["Independent paused capture frames; no temporal/tracking scores.",
                              "Same raw ROI, no voxel or IMU leveling; PCL reference is not full teammate pipeline.",
                              "Patchwork++ reinitialized for every sample/repetition; RNR disabled, height=1.45 or 1.80m, range=[1,60]m.",
                              "Mount z=1.45m above base_link is not sensor height above road. Fixed1.80 trial informed by this sample's asphalt diagnostic: exploratory, not held-out validation.",
                              "Timing excludes file I/O, Patchwork construction, Python scoring and ROS latency; includes PCL conversion and output indices.",
                              "Structure labels do not distinguish tunnel walls/ceiling and must not be treated as structural-filter accuracy.",
                              "Visible box metrics condition on at least one labeled obstacle point inside ROI; not detector recall.",
                              "Capture LiDAR settings were 10Hz/28800 rays; results do not establish current v3 streaming 15Hz behavior."]}
    write_json(args.output, report)
    print(json.dumps({k: {"ratios": v["ratios"], "timing_ms": v["timing_ms"], "visible_boxes": v["visible_boxes"],
                           "boxes_all_points_removed": v["boxes_all_points_removed"]} for k, v in results.items()}, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    staging = sub.add_parser("stage")
    staging.add_argument("--inventory", type=Path, required=True)
    staging.add_argument("--rejected-root", type=Path, required=True)
    staging.add_argument("--output", type=Path, required=True)
    scoring = sub.add_parser("score")
    scoring.add_argument("--stage", type=Path, required=True)
    scoring.add_argument("--predictions", type=Path, required=True)
    scoring.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    (stage if args.command == "stage" else score)(args)


if __name__ == "__main__":
    main()
