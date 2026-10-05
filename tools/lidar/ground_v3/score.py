#!/usr/bin/env python3
"""Score frozen V1, linefit coverage, and V3 on the same GT and gravity frame."""
import argparse
from collections import Counter, defaultdict
import gzip
import itertools
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ground_live_review"))
from evaluate import digest
from metrics import frame_metrics


NAMES = ("jaeho_v1", "jaeho_rp", "patchwork_v1", "patchwork_rp",
         "linefit_v1", "linefit_covered", "linefit_rp")
ACTORS = ("vehicle", "pedestrian", "obstacle")


def summary(input_root, old_root, new_root, output):
    output.mkdir(parents=True, exist_ok=False)
    source_path = input_root / "snapshots.jsonl"
    paths = (old_root / "v1.jsonl", old_root / "lf_covered.jsonl",
             new_root / "roll_pitch_all.jsonl")
    totals = defaultdict(lambda: defaultdict(Counter))
    objects = defaultdict(lambda: defaultdict(Counter))
    graphs = defaultdict(lambda: defaultdict(lambda: defaultdict(Counter)))
    causes = defaultdict(lambda: defaultdict(Counter))
    attempts = defaultdict(Counter)
    timing = defaultdict(lambda: defaultdict(list))
    additional = defaultdict(Counter)
    cases = []
    count = 0
    with source_path.open() as src, paths[0].open() as old, paths[1].open() as lf, paths[2].open() as rp, \
         gzip.open(output / "frames.jsonl.gz", "wt") as out:
        for record in itertools.zip_longest(src, old, lf, rp):
            if None in record:
                raise ValueError("input or result length mismatch")
            source, baseline, coverage, corrected = map(json.loads, record)
            identifier, stamp = source["input_id"], source["scan_effective_stamp_ns"]
            mapping = baseline["raw_to_voxel"]
            for result in (baseline, coverage, corrected):
                if result["input_id"] != identifier or result["stamp_ns"] != stamp or result["raw_to_voxel"] != mapping:
                    raise ValueError("scan/stamp/ROI lineage mismatch at " + identifier)
            if corrected["config_id"] != "roll_pitch_all" or corrected.get("leveling_mode") != "roll_pitch":
                raise ValueError("V3 result is not roll_pitch_all")
            if coverage["config_id"] != "lf_covered":
                raise ValueError("linefit coverage reference mismatch")
            mask_rows = {
                "jaeho_v1": baseline["methods"]["jaeho"],
                "jaeho_rp": corrected["methods"]["jaeho"],
                "patchwork_v1": baseline["methods"]["patchwork"],
                "patchwork_rp": corrected["methods"]["patchwork"],
                "linefit_v1": baseline["methods"]["linefit"],
                "linefit_covered": coverage["methods"]["linefit"],
                "linefit_rp": corrected["methods"]["linefit"],
            }
            masks = {k: np.asarray(row["mask"], np.uint8) for k, row in mask_rows.items()}
            unique, aliases, identities = {}, {}, {}
            for name, mask in masks.items():
                key = mask.tobytes()
                if key not in identities:
                    identities[key] = name
                    unique[name] = mask
                aliases[name] = identities[key]
            rel = Path(source["cloud_path"])
            if rel.is_absolute() or ".." in rel.parts:
                raise ValueError("unsafe cloud path")
            cloud = np.fromfile(input_root / rel, dtype="<f4").reshape(-1, 4)
            with np.load(input_root / "labels" / (identifier + ".npz")) as labels:
                semantic, instance = labels["semantic"], labels["instance"]
            scored_unique = frame_metrics(cloud, semantic, instance, unique,
                                          source["pitch"], source["roll"])
            scored = {name: scored_unique[aliases[name]] for name in masks}
            roi = np.asarray(mapping) >= 0
            frame = {"input_id": identifier, "stamp_ns": stamp, "roll": source["roll"],
                     "pitch": source["pitch"], "methods": {}}
            for name, metrics in scored.items():
                frame["methods"][name] = {
                    "classes": {k: {"input": v["input_points"],
                                    "retained": v["retained_points"]}
                                for k, v in metrics["classes"].items()},
                    "actors": [{
                        "class": a["semantic"], "instance": a["instance_id"],
                        "input": a["input_points"], "retained": a["retained_points"],
                        "lower": a["observed_height_bands"]["lower"]["retained_points"],
                        "lost": a["complete_ground_loss"],
                    } for a in metrics["objects"] if not a["sparse"]]
                }
                for category, data in metrics["classes"].items():
                    totals[name][category].update({k: data[k] for k in (
                        "input_points", "retained_points", "ground_points", "unclassified_points")})
                    if "topology" in data:
                        for radius, graph in data["topology"].items():
                            graphs[name][category][radius].update({
                                k: v for k, v in graph.items() if isinstance(v, int)})
                for obj in metrics["objects"]:
                    group = obj["semantic"] + ("/sparse" if obj["sparse"] else "/observed10plus")
                    objects[name][group].update(
                        observations=1, input_points=obj["input_points"],
                        retained_points=obj["retained_points"],
                        complete_loss=int(obj["complete_ground_loss"]),
                        lower_input=obj["observed_height_bands"]["lower"]["input_points"],
                        lower_retained=obj["observed_height_bands"]["lower"]["retained_points"])
                raw = mask_rows[name]
                for key in ("copy_ms", "segment_ms", "guard_ms", "restore_ms"):
                    timing[name][key].append(raw[key])
                prep = corrected if name.endswith("_rp") else coverage if name == "linefit_covered" else baseline
                timing[name]["shared_prepare_ms"].append(prep["prepare_ms"])
                bits = np.asarray(raw["reasons"], dtype=np.uint32)
                if len(bits):
                    for bit in (1, 4, 8, 16, 1024, 2048):
                        causes[name]["all"][str(bit)] += int(np.count_nonzero(
                            ((bits[roi] & bit) != 0) & (masks[name][roi] == 2)))
            ref = {(a["semantic"], a["instance_id"]): a
                   for a in scored["jaeho_rp"]["objects"] if not a["sparse"]}
            for name in ("patchwork_rp", "linefit_rp"):
                for obj in scored[name]["objects"]:
                    if obj["sparse"]:
                        continue
                    before = ref[(obj["semantic"], obj["instance_id"])]
                    extra = bool(before["retained_points"] and not obj["retained_points"])
                    if extra:
                        additional[name][obj["semantic"]] += 1
                        cases.append({"input_id": identifier, "profile": name,
                                      "class": obj["semantic"], "instance": obj["instance_id"],
                                      "reason": "additional_complete_loss_vs_jaeho_rp"})
            for plane in mask_rows["jaeho_rp"].get("plane_attempts", []):
                key = "accepted" if plane["accepted_ground"] else "rejected"
                attempts[key]["attempts"] += 1
                attempts[key]["inlier_points"] += plane["inlier_points"]
                if plane["accepted_ground"] and plane["height_valid"] and plane["height_at_origin_m"] > 0:
                    attempts[key]["positive_origin_height_attempts"] += 1
                    cases.append({"input_id": identifier, "profile": "jaeho_rp",
                                  "reason": "positive_height_ground_plane",
                                  "height_m": plane["height_at_origin_m"],
                                  "inliers": plane["inlier_points"]})
            attempts["frames"]["total"] += 1
            attempts["frames"]["fallback"] += mask_rows["jaeho_rp"]["events"].get("ground_fallback", 0)
            out.write(json.dumps(frame, separators=(",", ":"), allow_nan=False) + "\n")
            count += 1
            if count % 250 == 0:
                print(json.dumps({"scored": count, "last": identifier}), flush=True)
    if not count:
        raise ValueError("empty result")
    result = {
        "frames": count, "classes": totals, "objects": objects, "topology": graphs,
        "additional_complete_loss_vs_jaeho_rp": additional,
        "plane_attempts": attempts, "causes": causes,
        "timing_ms": {
            name: {field: {f"p{q}": float(np.percentile(values, q)) for q in (50, 95, 99)}
                   for field, values in stages.items()} for name, stages in timing.items()},
        "sha256": {"snapshots": digest(source_path),
                   **{name: digest(path) for name, path in zip(("v1", "lf_covered", "roll_pitch_all"), paths)}},
        "notes": [
            "All seven masks use the same gravity-aligned GT coordinate; this differs from prior V2 pitch-only topology.",
            "Previously analysed input, not a held-out accuracy set.",
            "Positive origin height is a suspect ground plane, not proof of ceiling GT.",
            "Ground segmentation metrics do not measure detector or tracker performance.",
        ],
    }
    (output / "summary.json").write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    with (output / "cases.jsonl").open("x") as out:
        for case in cases:
            out.write(json.dumps(case, separators=(",", ":")) + "\n")
    print(json.dumps({"frames": count, "additional": additional, "plane_attempts": attempts},
                     default=dict))


def main():
    parser = argparse.ArgumentParser()
    for arg in ("input", "v2", "v3", "output"):
        parser.add_argument(arg, type=Path)
    args = parser.parse_args()
    summary(args.input, args.v2, args.v3, args.output)


if __name__ == "__main__":
    main()
