#!/usr/bin/env python3
"""Point-instance scores remain post-hoc; AOS BLOCKED is not an empty detection."""
import argparse
from collections import Counter, defaultdict
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from tools.lidar.capture_validation.score import object_metrics, cluster_composition
from tools.lidar.clustering_compare.projection import canonical, sha


def json_rows(path):
    with path.open() as stream:
        yield from map(json.loads, stream)


def raw_score(key, ids, mapping, pre, mode):
    roi = mapping >= 0
    expanded = np.full(len(mapping), 255, dtype=int)
    expanded[roi] = np.asarray(pre)[mapping[roi]]
    score = object_metrics(key, ids, np.arange(len(ids)), expanded, mode)
    target = ids == key
    wall_target = target & roi & (expanded == 0)
    score.update({"roi_target_points": int((target & roi).sum()), "roi_excluded": int((target & ~roi).sum()),
                  "voxel_target_points": len(np.unique(mapping[target & roi])),
                  "wall_stage_target_voxels": len(np.unique(mapping[wall_target])),
                  "sufficient_target_voxels_for_10_point_cluster": len(np.unique(mapping[wall_target])) >= 10})
    if not (target & roi).any():
        score["failure_reason"] = "outside_roi"
    return score


def run(args):
    if args.output.exists():
        raise ValueError("output exists")
    args.output.mkdir(parents=True)
    source = {r["input_id"]: r for r in json_rows(args.run / "snapshots.jsonl")}
    capture = args.kind == "capture"
    truth = {r["capture_key"] if capture else r["input_id"]: r for r in json_rows(args.truth)}
    groups, transitions, totals = defaultdict(Counter), Counter(), defaultdict(Counter)
    object_rows, frames = [], []
    for r in json_rows(args.run / "geometry.jsonl"):
        if not r["measurement"] or "skipped_reason" in r:
            continue
        s = source[r["input_id"]]
        t = truth[s["capture_key"]] if capture else truth[r["input_id"]]
        ids = np.fromfile(args.truth.parent / t["instance_path"], dtype="<i4") if capture else np.asarray(t["labels"])
        boxes = t["boxes"] if capture else [{"id": int(k), "class": t.get("kind") or "negative", "distance_m": 0}
                                            for k in sorted(set(ids) - {0, 1, 2, 4})]
        mapping = np.fromfile(args.run / "clouds" / (r["input_id"] + ".indices.bin"), dtype="<i4")
        pre = np.asarray(r["pre_wall_labels"])
        if len(ids) != len(mapping):
            raise ValueError("truth/lineage differs")
        state = s.get("controlled_camera_state", s.get("camera_state", t.get("camera_result", s.get("scene", "unknown"))))
        modes = ("baseline", "raw_euclidean") + (("travel_aos",) if r["travel_aos"]["status"] == "evaluated" else ())
        for b in boxes:
            scores = {m: object_metrics(b["id"], ids, mapping, pre, r[m]) if m == "baseline"
                      else raw_score(b["id"], ids, mapping, pre, r[m]) for m in modes}
            before = scores["baseline"]
            for m in modes:
                if m != "baseline":
                    reasons = np.asarray(r[m]["projection_reasons"])
                    roi = mapping >= 0
                    pre_raw = np.full(len(mapping), 255)
                    pre_raw[roi] = pre[mapping[roi]]
                    target = (ids == b["id"]) & (pre_raw == 0)
                    scores[m]["target_projection_reasons"] = {str(k): int((target & (reasons == k)).sum())
                                                               for k in sorted(set(reasons[target]))}
            for m, v in scores.items():
                c = groups[(state, b["class"], m)]
                c["observations"] += 1
                c["sufficient"] += v["sufficient_target_voxels_for_10_point_cluster"]
                c["accepted"] += v["accepted_target"]
                c["sufficient_accepted"] += v["sufficient_target_voxels_for_10_point_cluster"] and v["accepted_target"]
                if v["failure_reason"]:
                    c["failure:" + v["failure_reason"]] += 1
                if m != "baseline":
                    transitions[m + ":baseline_success_losses"] += before["accepted_target"] and not v["accepted_target"]
                    transitions[m + ":recovered_sufficient_failures"] += (before["sufficient_target_voxels_for_10_point_cluster"]
                        and not before["accepted_target"] and v["accepted_target"])
            object_rows.append({"input_id": r["input_id"], "sequence": s.get("sequence"), "camera_state": state,
                                "class": b["class"], "instance_id": b["id"], "distance_m": b.get("distance_m"),
                                "scores": scores, "aos_status": r["travel_aos"]["status"]})
        frame = {"input_id": r["input_id"], "camera_state": state, "sequence": s.get("sequence"), "modes": {}}
        for m in modes:
            scoring_ids = ids if capture else np.where(np.isin(ids, [0, 1, 2]), 0, ids)
            comp = cluster_composition(r[m], scoring_ids, mapping if m == "baseline" else np.arange(len(ids)))
            counts = {"final_objects": len(comp), "instance_supported": sum(c["known_instance_points"] > 0 for c in comp),
                      "without_instance_support": sum(c["known_instance_points"] == 0 for c in comp),
                      "multi_instance_clusters": sum(sum(int(k) > 0 and n >= .1*c["raw_points"]
                                                          for k, n in c["instance_point_counts"].items()) >= 2 for c in comp)}
            totals[(state, m)].update(counts)
            frame["modes"][m] = {**counts, "clusters": comp,
                "projection_exclusions": dict(Counter(r[m].get("projection_reasons", [])))}
        frames.append(frame)
    summary = {"schema": "asmc-l1b5a-point-score-v1", "kind": args.kind, "measured_frames": len(frames),
               "groups": [{"camera_state": s, "class": c, "mode": m, **dict(v)} for (s,c,m),v in sorted(groups.items())],
               "transitions": dict(transitions),
               "frame_totals": [{"camera_state": s, "mode": m, **dict(v)} for (s,m),v in sorted(totals.items())],
               "aos_status": "evaluated_subset_synthetic" if any("travel_aos" in r["modes"] for r in frames) else "blocked_projection_admission",
               "deployment_accepted": False,
               "limits": ["Raw count never replaces unique existing voxel admission",
                          "Unsupported outputs are diagnostics, not measured false positives",
                          "Capture development observations are correlated; no independent test claim",
                          "OBB IoU and continuous tracking are not accepted by these scores"]}
    for name, rows in (("objects.jsonl", object_rows), ("frames.jsonl", frames)):
        (args.output / name).write_text("".join(canonical(r) + "\n" for r in rows))
    (args.output / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
    (args.output / "manifest.json").write_text(json.dumps({"run_manifest_sha256": sha(args.run / "manifest.json"),
        "truth_sha256": sha(args.truth), "scorer_sha256": sha(Path(__file__)),
        "artifacts": {p.name: sha(p) for p in args.output.iterdir()}}, indent=2, sort_keys=True) + "\n")
    print(canonical(summary))


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    for name in ("run", "truth", "output"):
        p.add_argument("--" + name, type=Path, required=True)
    p.add_argument("--kind", choices=("capture", "synthetic"), required=True)
    run(p.parse_args())
