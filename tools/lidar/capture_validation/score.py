#!/usr/bin/env python3
"""Score known Capture Instance points via raw->voxel->cluster lineage.

GT IDs are NEVER assigned to candidate voxels or fed back into the filter. A
mixed voxel contributes all of its raw members to purity; no majority voting
can hide a target/background merge. Structure 109 is only a residual proxy.
"""
import argparse
from collections import Counter, defaultdict
import json
from pathlib import Path

import numpy as np

if __package__:
    from .admit import canonical, sha
else:
    from admit import canonical, sha

MODES = ("baseline", "policy_only", "geometry", "candidate_off")


def object_metrics(key, ids, mapping, pre_wall, mode):
    ids, mapping, pre_wall = np.asarray(ids), np.asarray(mapping), np.asarray(pre_wall)
    n = len(pre_wall)
    labels, clusters = np.asarray(mode["labels"]), np.asarray(mode["cluster_id"])
    if len(ids) != len(mapping) or len(labels) != n or len(clusters) != n or \
            np.any(mapping < -1) or np.any(mapping >= n):
        raise ValueError("invalid raw/voxel/cluster lineage")
    target = ids == key
    roi = mapping >= 0
    raw_pre = np.full(len(mapping), -1, dtype=int)
    raw_labels, raw_clusters = raw_pre.copy(), raw_pre.copy()
    raw_pre[roi] = pre_wall[mapping[roi]]
    raw_labels[roi] = labels[mapping[roi]]
    raw_clusters[roi] = clusters[mapping[roi]]
    wall_target = target & roi & (raw_pre == 0)
    denominator = int(wall_target.sum())
    raw_count = int(target.sum())
    candidates = []
    for ci in sorted(set(raw_clusters[wall_target]) - {-1}):
        if ci < 0 or ci >= len(mode["objects"]):
            raise ValueError("unpaired cluster/OBB index")
        member = roi & (raw_clusters == ci)
        intersection = int((member & wall_target).sum())
        obj = mode["objects"][ci]
        candidates.append({"cluster_index": int(ci), "removed": obj["removed"],
                           "target_points": intersection, "cluster_raw_points": int(member.sum()),
                           "completeness": intersection / denominator if denominator else None,
                           "purity": intersection / int(member.sum()), "flags": obj.get("flags", {}),
                           "obb_center": obj["center"], "obb_dimensions": obj["dimensions"]})
    survivors = [c for c in candidates if not c["removed"]]
    best = max(survivors, key=lambda c: (c["completeness"], c["purity"], -c["cluster_index"]), default=None)
    accepted = bool(best and best["completeness"] >= .8 and best["purity"] >= .9)
    if not denominator:
        failure = "outside_roi" if not (target & roi).any() else "upstream_ground_or_ceiling"
    elif accepted:
        failure = None
    elif not (wall_target & (raw_labels == 0)).any():
        failure = "wall_stage_all_removed"
    elif not candidates:
        failure = "kept_but_no_cluster"
    elif not survivors:
        failure = "final_obb_rejected"
    elif best["purity"] < .9:
        failure = "merged_with_non_target_points"
    else:
        failure = "fragmented_or_partly_removed"
    return {"raw_target_points": raw_count, "roi_target_points": int((target & roi).sum()),
            "roi_excluded": int((target & ~roi).sum()),
            "voxel_target_points": len(np.unique(mapping[target & roi])),
            "wall_stage_target_points": denominator,
            "wall_stage_target_voxels": len(np.unique(mapping[wall_target])),
            "upstream_ground_removed": int((target & (raw_pre == 2)).sum()),
            "upstream_ceiling_removed": int((target & (raw_pre == 1)).sum()),
            "wall_stage_kept": int((wall_target & (raw_labels == 0)).sum()),
            "wall_stage_removed": int((wall_target & (raw_labels == 3)).sum()),
            "wall_stage_preservation": int((wall_target & (raw_labels == 0)).sum()) / denominator if denominator else None,
            "final_target": best, "target_clusters": candidates, "accepted_target": accepted,
            "failure_reason": failure,
            "original_visible_points_in_final_objects": int((target & (raw_clusters >= 0) &
                np.isin(raw_clusters, [c["cluster_index"] for c in survivors])).sum()),
            "sufficient_target_voxels_for_10_point_cluster": len(np.unique(mapping[wall_target])) >= 10}


def cluster_composition(mode, ids, mapping):
    roi = mapping >= 0
    clusters = np.full(len(ids), -1, dtype=int)
    clusters[roi] = np.asarray(mode["cluster_id"])[mapping[roi]]
    result = []
    for ci, obj in enumerate(mode["objects"]):
        if obj["removed"]:
            continue
        members = ids[clusters == ci]
        count = Counter(int(k) for k in members)
        known = sum(v for k, v in count.items() if k > 0)
        result.append({"cluster_index": ci, "raw_points": len(members), "known_instance_points": known,
                       "instance_point_counts": dict(count), "known_fraction": known / len(members) if len(members) else None,
                       "obb_center": obj["center"], "obb_dimensions": obj["dimensions"],
                       "flags": obj.get("flags", {})})
    return result


def mixed_voxels(ids, mapping):
    seen = defaultdict(set)
    for key, vi in zip(ids, mapping):
        if vi >= 0:
            seen[int(vi)].add(int(key))
    mixed = {v for v, values in seen.items() if len(values) > 1}
    return {"mixed_voxels": len(mixed),
            "target_raw_points_in_mixed_voxels": int(((ids > 0) & np.isin(mapping, list(mixed))).sum())}


def score_run(args):
    if args.output.exists():
        raise ValueError("output exists; choose a new report directory")
    admission = json.loads((args.admission / "manifest.json").read_text())
    if admission["runtime_connected"]:
        raise ValueError("not an isolated admission")
    for name, expected in admission["artifacts"].items():
        if sha(args.admission / name) != expected:
            raise ValueError("admitted artifact changed: " + name)
    truth = {r["capture_key"]: r for r in map(json.loads, (args.admission / "truth.jsonl").open())}
    snapshots = {r["input_id"]: r for r in map(json.loads, (args.admission / "snapshots.jsonl").open())}
    rows = []
    objects = []
    groups = defaultdict(lambda: Counter())
    paired = defaultdict(dict)
    global_counts = Counter()
    for line in (args.run / "geometry.jsonl").open():
        result = json.loads(line)
        if not result["measurement"]:
            continue
        if "skipped_reason" in result:
            raise ValueError("admitted frame was skipped")
        source = snapshots[result["input_id"]]
        gt = truth[source["capture_key"]]
        ids = np.fromfile(args.admission / gt["instance_path"], dtype="<i4")
        semantics = np.fromfile(args.admission / gt["semantic_path"], dtype="<i4")
        mapping = np.fromfile(args.run / "clouds" / (result["input_id"] + ".indices.bin"), dtype="<i4")
        pre = np.asarray(result["pre_wall_labels"])
        if len(mapping) != len(ids) or len(semantics) != len(ids):
            raise ValueError("GT lineage length changed")
        state = source["controlled_camera_state"]
        paired[source["capture_key"]][state] = result["geometry"]["labels"]
        roi = mapping >= 0
        pre_raw = np.full(len(ids), -1, dtype=int)
        pre_raw[roi] = pre[mapping[roi]]
        frame = {"input_id": result["input_id"], "capture_key": source["capture_key"],
                 "sequence": source["sequence"], "camera_state": state,
                 "wall_models": len(result["wall_models"]),
                 "lineage": mixed_voxels(ids, mapping), "modes": {}}
        for box in gt["boxes"]:
            scores = {m: object_metrics(box["id"], ids, mapping, pre, result[m]) for m in MODES}
            entry = {"input_id": result["input_id"], "capture_key": source["capture_key"],
                     "sequence": source["sequence"], "camera_state": state, "instance_id": box["id"],
                     "class": box["class"], "gt_box_admitted": box["box_accuracy_admitted"],
                     "outside_gt_box_3cm": box["outside_box_3cm"], "distance_m": box["distance_m"],
                     "scores": scores}
            objects.append(entry)
            if scores["baseline"]["accepted_target"] and not scores["geometry"]["accepted_target"]:
                global_counts["baseline_success_losses"] += 1
            for mode, s in scores.items():
                c = groups[(state, box["class"], mode)]
                c["observations"] += 1
                c["sufficient_voxel_observations"] += s["sufficient_target_voxels_for_10_point_cluster"]
                for key in ("raw_target_points", "roi_target_points", "roi_excluded", "wall_stage_target_points",
                            "upstream_ground_removed", "upstream_ceiling_removed", "wall_stage_kept", "wall_stage_removed"):
                    c[key] += s[key]
                c["accepted"] += s["accepted_target"]
                if s["failure_reason"]:
                    c["failure:" + s["failure_reason"]] += 1
        for mode in MODES:
            labels = np.asarray(result[mode]["labels"])
            raw_labels = np.full(len(ids), -1, dtype=int)
            raw_labels[roi] = labels[mapping[roi]]
            composition = cluster_composition(result[mode], ids, mapping)
            structure = (ids == 0) & (semantics == 109) & (pre_raw == 0)
            frame["modes"][mode] = {"final_objects": result[mode]["final_objects"],
                                   "known_instance_backed_objects": sum(c["known_instance_points"] > 0 for c in composition),
                                   "objects_without_labelled_instance_support": sum(c["known_instance_points"] == 0 for c in composition),
                                   "structure109_wall_stage_points": int(structure.sum()),
                                   "structure109_kept_proxy_not_wall_gt": int((structure & (raw_labels == 0)).sum()),
                                   "clusters": composition}
        rows.append(frame)
    if len(rows) != admission["frames"] * 2 or any(set(v) != {"missing", "empty"} for v in paired.values()):
        raise ValueError("missing paired camera conditions")
    global_counts["negative_camera_geometry_mask_differences"] = sum(v["missing"] != v["empty"] for v in paired.values())
    by_group = []
    for (state, kind, mode), c in sorted(groups.items()):
        by_group.append({"camera_state": state, "class": kind, "mode": mode, **dict(c),
                         "weighted_wall_stage_preservation": c["wall_stage_kept"] / c["wall_stage_target_points"]
                         if c["wall_stage_target_points"] else None})
    frame_totals = {}
    for state in ("missing", "empty"):
        frame_totals[state] = {}
        for mode in MODES:
            values = [r["modes"][mode] for r in rows if r["camera_state"] == state]
            keys = ("final_objects", "known_instance_backed_objects", "objects_without_labelled_instance_support",
                    "structure109_wall_stage_points", "structure109_kept_proxy_not_wall_gt")
            frame_totals[state][mode] = {k: sum(v[k] for v in values) for k in keys}
    summary = {"schema": "asmc-capture-scores-v1", "frames": admission["frames"], "controlled_conditions": len(rows),
               "counts": dict(global_counts), "by_class_camera_mode": by_group, "frame_totals": frame_totals,
               "deploy_accepted": False,
               "limits": ["Actual camera arrival and detector quality not evaluated",
                          "Single-frame stationary warmup does not certify continuous tracking or GPS transition",
                          "No pedestrian observations; all-mission acceptance incomplete",
                          "Structure109/background-only objects are not measured false positives or GT walls",
                          "Accepted target denotes >=80% visible wall-stage point coverage and >=90% raw-point purity",
                          "GT box geometry is not used in point scoring; questionable boxes remain excluded from OBB accuracy",
                          "No IoU/full-box geometry/velocity/track-ID accuracy claim"]}
    args.output.mkdir(parents=True)
    for filename, values in (("frames.jsonl", rows), ("objects.jsonl", objects)):
        (args.output / filename).write_text("".join(canonical(v) + "\n" for v in values))
    (args.output / "summary.json").write_text(json.dumps(summary, sort_keys=True, indent=2) + "\n")
    manifest = {"schema": "asmc-capture-score-manifest-v1", "runtime_connected": False,
                "admission_manifest_sha256": sha(args.admission / "manifest.json"),
                "run_manifest_sha256": sha(args.run / "manifest.json"),
                "geometry_jsonl_sha256": sha(args.run / "geometry.jsonl"),
                "tools_sha256": {p.name: sha(p) for p in Path(__file__).parent.glob("*.py")},
                "artifacts": {p.name: sha(p) for p in args.output.iterdir() if p.is_file()}}
    (args.output / "manifest.json").write_text(json.dumps(manifest, sort_keys=True, indent=2) + "\n")
    print(canonical({k: summary[k] for k in ("frames", "controlled_conditions", "counts", "deploy_accepted")}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("admission", "run", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    score_run(parser.parse_args())
