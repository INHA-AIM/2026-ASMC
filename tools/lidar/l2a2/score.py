#!/usr/bin/env python3
"""Score L2-A2 four-way ground masks against admitted point labels, without ROS."""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path
import statistics
import numpy as np

GROUND = frozenset((85, 127, 128, 136, 144, 170, 255))
TARGET = {86, 118, 125, 135, 145, 155, 164, 165, 175, 185}
MODES = ("ref_voxel", "patch_voxel", "ref_raw", "patch_raw")
CLASSES = (0, 1, 2, 3)

def sha(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()

def records(path):
    with Path(path).open() as stream:
        for line in stream:
            if line.strip():
                yield json.loads(line)

def percentiles(values):
    if not values:
        return None
    return {f"p{p}": float(np.percentile(values, p)) for p in (50, 95, 99)}

def score(comparison, admission, output):
    if output.exists():
        raise ValueError("output exists")
    manifest = json.loads((admission / "manifest.json").read_text())
    if manifest.get("frames") != 105:
        raise ValueError("unexpected Capture admission")
    snapshots = {r["input_id"]: r for r in records(admission / "snapshots.jsonl")}
    truth = {r["capture_key"]: r for r in records(admission / "truth.jsonl")}
    rows = []
    objects = []
    totals = {mode: defaultdict(Counter) for mode in MODES}
    timing = defaultdict(list)
    stage_timing = defaultdict(list)
    first_totals = {mode: Counter() for mode in MODES}
    first_changed = Counter()
    first_by_phase = {}
    paired = defaultdict(dict)
    measured = set()
    for result in records(comparison):
        input_id = result["input_id"]
        source = snapshots[input_id]
        if result["stamp_ns"] != source["scan_effective_stamp_ns"]:
            raise ValueError("stamp changed: " + input_id)
        camera = source["controlled_camera_state"]
        if camera != result["camera"]:
            raise ValueError("camera state changed")
        if camera == "missing":
            first_by_phase.setdefault(result["phase"], result)
        if not result["measurement"]:
            continue
        if input_id in measured:
            raise ValueError("duplicate measured input: " + input_id)
        measured.add(input_id)
        capture_key = source["capture_key"]
        paired[capture_key][camera] = {key: result[key] for key in MODES}
        if camera != "missing":
            continue  # empty is a paired invariance check, not an independent scene.
        stage_timing["preprocess"].append(float(result["timing_ms"]["preprocess"]))
        gt = truth[capture_key]
        semantic = np.fromfile(admission / gt["semantic_path"], dtype="<i4")
        instance = np.fromfile(admission / gt["instance_path"], dtype="<i4")
        if len(semantic) != result["raw_points"] or len(instance) != len(semantic):
            raise ValueError("point/GT length drift")
        masks = {mode: np.asarray(result[mode], dtype=np.uint8) for mode in MODES}
        if any(len(mask) != len(semantic) or not np.isin(mask, CLASSES).all()
               for mask in masks.values()):
            raise ValueError("invalid ground mask")
        roi = masks["ref_voxel"] != 0
        first = first_by_phase.pop(result["phase"])
        if first["stamp_ns"] != result["stamp_ns"] or first["raw_points"] != result["raw_points"]:
            raise ValueError("first/last stationary input drift")
        first_masks = {mode: np.asarray(first[mode], dtype=np.uint8) for mode in MODES}
        ground_truth = roi & np.isin(semantic, list(GROUND))
        target_truth = roi & np.isin(semantic, list(TARGET))
        for mode in MODES:
            mask = masks[mode]
            first_mask = first_masks[mode]
            if not np.array_equal(first_mask, mask):
                first_changed[mode] += 1
            first_totals[mode].update({
                "ground_gt": int(ground_truth.sum()),
                "ground_removed": int((ground_truth & (first_mask == 1)).sum()),
                "target_gt": int(target_truth.sum()),
                "target_retained": int((target_truth & (first_mask == 2)).sum()),
            })
            # A zero inside the common ROI is unassigned, not nonground.
            stage = {
                "roi_points": int(roi.sum()),
                "unassigned_in_roi": int((roi & (mask == 0)).sum()),
                "ceiling_excluded": int((roi & (mask == 3)).sum()),
                "ground_gt": int((roi & np.isin(semantic, list(GROUND))).sum()),
                "ground_removed": int((roi & np.isin(semantic, list(GROUND)) & (mask == 1)).sum()),
                "asphalt_gt": int((roi & (semantic == 127)).sum()),
                "asphalt_removed": int((roi & (semantic == 127) & (mask == 1)).sum()),
                "target_gt": int((roi & np.isin(semantic, list(TARGET))).sum()),
                "target_retained": int((roi & np.isin(semantic, list(TARGET)) & (mask == 2)).sum()),
                "structure109": int((roi & (semantic == 109)).sum()),
                "structure109_retained_context": int((roi & (semantic == 109) & (mask == 2)).sum()),
            }
            totals[mode]["all"].update(stage)
            for value in sorted(TARGET):
                chosen = roi & (semantic == value)
                if chosen.any():
                    totals[mode][str(value)].update({
                        "points": int(chosen.sum()),
                        "retained": int((chosen & (mask == 2)).sum()),
                        "removed_as_ground": int((chosen & (mask == 1)).sum()),
                    })
            elapsed = result["timing_ms"]
            value = (elapsed["ref_voxel_level"] + elapsed["ref_voxel_ground"]
                     if mode == "ref_voxel" else
                     elapsed["ref_raw_level"] + elapsed["ref_raw_ground"]
                     if mode == "ref_raw" else elapsed[mode]["total"])
            timing[mode].append(float(value))
            if mode.startswith("patch"):
                for component in ("level", "copy", "segment", "restore", "total"):
                    stage_timing[mode + "." + component].append(float(elapsed[mode][component]))
            else:
                stage_timing[mode + ".level"].append(float(elapsed[mode + "_level"]))
                stage_timing[mode + ".ground"].append(float(elapsed[mode + "_ground"]))
            rows.append({"capture_key": capture_key, "mode": mode, **stage})
        for box in gt["boxes"]:
            member = roi & (instance == box["id"])
            count = int(member.sum())
            if not count:
                continue
            entry = {"capture_key": capture_key, "instance_id": box["id"],
                     "class": box["class"], "distance_m": box["distance_m"],
                     "roi_points": count, "box_geometry_admitted": box["box_accuracy_admitted"],
                     "retained": {mode: int((member & (masks[mode] == 2)).sum()) for mode in MODES}}
            objects.append(entry)
    if first_by_phase:
        raise ValueError("unfinished first-pass phases")
    if len(measured) != 210 or len(paired) != 105 or any(set(v) != {"missing", "empty"} for v in paired.values()):
        raise ValueError("paired Capture frame count changed")
    differences = [key for key, v in paired.items() if v["missing"] != v["empty"]]
    summary = {
        "schema": "asmc-lidar-ground-comparison-v1",
        "profile": "tunnel", "scenes": 105, "measured_camera_conditions": 210,
        "camera_ground_mask_differences": len(differences),
        "camera_ground_mask_difference_examples": differences[:8],
        "modes": {},
        "interpretation": [
            "105 scenes contain correlated snapshots, not independent road situations.",
            "Masks score ground-stage decisions only. They do not score wall removal, clustering, OBB, tracking or planner acceptance.",
            "Class 109 StandingOBJ includes structures and is context, never actor recall or wall truth.",
            "Current v3 Capture contains vehicle and generic obstacle labels, no pedestrian acceptance.",
            "Zero inside ROI means Patchwork unassigned; ceiling exclusion is class 3.",
        ],
    }
    summary["stage_time_ms"] = {key: percentiles(value) for key, value in stage_timing.items()}
    summary["first_pass"] = {
        mode: {
            "ground_removal": first_totals[mode]["ground_removed"] / first_totals[mode]["ground_gt"],
            "target_retention": first_totals[mode]["target_retained"] / first_totals[mode]["target_gt"],
            "mask_changed_by_fifth_pass_scenes": first_changed[mode],
        }
        for mode in MODES
    }
    for mode in MODES:
        all_counts = totals[mode]["all"]
        ground_n = all_counts["ground_gt"]
        target_n = all_counts["target_gt"]
        summary["modes"][mode] = {
            "counts": dict(all_counts),
            "ground_removal": all_counts["ground_removed"] / ground_n if ground_n else None,
            "target_retention": all_counts["target_retained"] / target_n if target_n else None,
            "by_semantic": {key: dict(value) for key, value in totals[mode].items() if key != "all"},
            "time_ms": percentiles(timing[mode]),
        }
    strata = {mode: defaultdict(Counter) for mode in MODES}
    for obj in objects:
        if obj["roi_points"] < 10 or not obj["box_geometry_admitted"]:
            continue
        distance = obj["distance_m"]
        distance_bin = "<10" if distance < 10 else "10-20" if distance < 20 else (
            "20-40" if distance < 40 else ">=40")
        count = obj["roi_points"]
        point_bin = "10-49" if count < 50 else "50-199" if count < 200 else ">=200"
        for mode in MODES:
            for key in ("distance:" + distance_bin, "points:" + point_bin):
                group = strata[mode][key]
                group["observations"] += 1
                group["roi_points"] += count
                group["retained"] += obj["retained"][mode]
                group["complete_loss"] += obj["retained"][mode] == 0
    for mode in MODES:
        summary["modes"][mode]["admitted_box_strata"] = {
            key: dict(value) for key, value in sorted(strata[mode].items())}
    for new, old in (("patch_voxel", "ref_voxel"), ("patch_raw", "ref_raw")):
        new_loss = [o for o in objects if o["roi_points"] >= 10 and
                    o["retained"][old] > 0 and o["retained"][new] == 0]
        summary["modes"][new]["new_complete_loss_10raw_points"] = len(new_loss)
        summary["modes"][new]["new_complete_loss_admitted_10raw_points"] = sum(
            o["box_geometry_admitted"] for o in new_loss)
        summary["modes"][new]["new_complete_loss_unverified_10raw_points"] = sum(
            not o["box_geometry_admitted"] for o in new_loss)
        summary["modes"][new]["new_complete_loss_examples"] = [
            o["capture_key"] + ":" + str(o["instance_id"]) for o in new_loss[:15]]
    output.mkdir(parents=True)
    (output / "summary.json").write_text(json.dumps(summary, sort_keys=True, indent=2) + "\n")
    for name, values in (("frames.jsonl", rows), ("objects.jsonl", objects)):
        with (output / name).open("w") as dest:
            for value in values:
                dest.write(json.dumps(value, sort_keys=True, separators=(",", ":")) + "\n")
    source_sha = {name: sha(admission / name) for name in ("snapshots.jsonl", "truth.jsonl", "manifest.json")}
    report = {"schema": "asmc-lidar-ground-comparison-manifest-v1",
              "source_sha256": source_sha,
              "comparison_sha256": sha(comparison),
              "scorer_sha256": sha(Path(__file__)),
              "artifacts_sha256": {name: sha(output / name)
                                   for name in ("summary.json", "frames.jsonl", "objects.jsonl")}}
    (output / "manifest.json").write_text(json.dumps(report, sort_keys=True, indent=2) + "\n")
    print(json.dumps({"scenes": 105, "camera_differences": len(differences),
                      "modes": {key: {k: v[k] for k in ("ground_removal", "target_retention", "time_ms")}
                                for key, v in summary["modes"].items()}}, sort_keys=True))

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--comparison", required=True, type=Path)
    parser.add_argument("--admission", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    score(args.comparison, args.admission, args.output)

if __name__ == "__main__":
    main()
