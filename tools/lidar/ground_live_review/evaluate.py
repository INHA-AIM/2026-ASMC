#!/usr/bin/env python3
"""Streaming shape/consistency diagnostics; no detector cluster score gate."""
import argparse
from collections import Counter, defaultdict
import hashlib
import itertools
import json
from pathlib import Path
import numpy as np
from metrics import frame_metrics


def digest(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(1024*1024), b""):
            h.update(block)
    return h.hexdigest()


def main():
    p = argparse.ArgumentParser()
    p.add_argument("input", type=Path)
    p.add_argument("result", type=Path)
    p.add_argument("output", type=Path)
    p.add_argument("--stride", type=int, default=1)
    a = p.parse_args()
    if a.stride < 1:
        p.error("stride must be positive")
    a.output.mkdir(parents=True, exist_ok=True)
    frames = a.output/"frames.jsonl"; summary = a.output/"summary.json"
    if frames.exists() or summary.exists():
        raise FileExistsError("choose a new output directory")
    totals = defaultdict(lambda: defaultdict(Counter))
    graph = defaultdict(lambda: defaultdict(lambda: defaultdict(Counter)))
    objs = defaultdict(lambda: defaultdict(Counter))
    times = defaultdict(lambda: defaultdict(list))
    histograms = defaultdict(lambda: defaultdict(lambda: np.zeros(5, dtype=int)))
    labels_audit = defaultdict(Counter)
    voxel_counts = defaultdict(Counter)
    count = 0; seen = 0
    with (a.input/"snapshots.jsonl").open() as snapshots, a.result.open() as results, frames.open("x") as out:
        for index, pair in enumerate(itertools.zip_longest(snapshots, results)):
            if None in pair:
                raise ValueError("stream lengths differ")
            source, current = map(json.loads, pair)
            if source["input_id"] != current["input_id"] or source["scan_effective_stamp_ns"] != current["stamp_ns"]:
                raise ValueError("identity or observation stamp mismatch")
            seen += 1
            if index % a.stride:
                continue
            path = Path(source["cloud_path"])
            if path.is_absolute() or ".." in path.parts:
                raise ValueError("invalid relative cloud path")
            cloud = np.fromfile(a.input/path, dtype="<f4").reshape(-1,4)
            with np.load(a.input/"labels"/(source["input_id"]+".npz")) as labels:
                semantic = labels["semantic"]; instance = labels["instance"]
            masks = {m: np.asarray(v["mask"], dtype=np.uint8) for m,v in current["methods"].items()}
            result = frame_metrics(cloud, semantic, instance, masks, source["pitch"])
            roi = np.asarray(current["raw_to_voxel"]) >= 0
            mapping = np.asarray(current["raw_to_voxel"])
            for method, mask in masks.items():
                _, first = np.unique(mapping[roi], return_index=True)
                voxel_mask = mask[roi][first]
                voxel_counts[method].update(input_voxels=len(first),
                    ground_voxels=int(np.sum(voxel_mask == 1)),
                    nonground_voxels=int(np.sum(voxel_mask == 2)),
                    uncertain_voxels=int(np.sum(voxel_mask == 3)))
            for code in np.unique(semantic[roi]):
                selected = roi & (semantic == code)
                labels_audit[str(int(code))].update(points=int(selected.sum()),
                    nonzero_instance_points=int(np.count_nonzero(instance[selected])))
            out.write(json.dumps({"input_id":source["input_id"], "stamp_ns":current["stamp_ns"],
                                  "methods":result}, separators=(",",":"), allow_nan=False)+"\n")
            for m, data in result.items():
                for name, row in data["classes"].items():
                    target = totals[m][name]
                    target.update({k:row[k] for k in ("input_points","retained_points","ground_points","unclassified_points")})
                    target["observations"] += 1
                    if "regions" in row:
                        target.update({k:v for k,v in row["regions"].items() if k != "retention_histogram"})
                        histograms[m][name] += row["regions"]["retention_histogram"]
                        for radius, values in row["topology"].items():
                            graph[m][name][radius].update({k:v for k,v in values.items() if isinstance(v,int)})
                for row in data["objects"]:
                    name = row["semantic"]+("/sparse" if row["sparse"] else "/observed10plus")
                    objs[m][name].update(observations=1, complete_ground_loss=int(row["complete_ground_loss"]),
                                         partial_retention=int(row["partial_retention"]),
                                         input_points=row["input_points"], retained_points=row["retained_points"])
                    for band, values in row["observed_height_bands"].items():
                        objs[m][name][band+"_input"] += values["input_points"]
                        objs[m][name][band+"_retained"] += values["retained_points"]
                for key, val in current["methods"][m].items():
                    if key.endswith("_ms"):
                        times[m][key].append(val)
                times[m]["shared_prepare_ms"].append(current["prepare_ms"])
            count += 1
            if count % 100 == 0:
                print(json.dumps({"scored":count,"seen":seen,"last":source["input_id"]}), flush=True)
    result = dict(frames=count, source_frames=seen, stride=a.stride,
                  classes=totals, objects=objs, topology=graph, semantic_instance_audit=labels_audit,
                  voxel_counts=voxel_counts,
                  region_retention_histograms={m:{c:v.tolist() for c,v in rows.items()} for m,rows in histograms.items()},
                  timing_ms={m:{k:{f"p{q}":float(np.percentile(v,q)) for q in (50,95,99)}
                                for k,v in rows.items()} for m,rows in times.items()},
                  sha256={"snapshots":digest(a.input/"snapshots.jsonl"),"results":digest(a.result)},
                  temporal_consistency="not_scored_without_verified_reobservation_correspondence",
                  limits=["GT groups are observed points, not full object shapes.",
                          "Background regions are sensor-frame cubes, not individual walls.",
                          "Topology is a diagnostic, not a clustering/detection acceptance gate.",
                          "Unknown/StandingOBJ are not wall truth or automatic false positives.",
                          "Ease of later filtering remains unvalidated.",
                          "Current recordings informed configuration selection; no held-out claim."])
    summary.write_text(json.dumps(result,indent=2,sort_keys=True,allow_nan=False)+"\n")
    print(json.dumps({"frames":count,"summary":str(summary)}), flush=True)


if __name__ == "__main__":
    main()
