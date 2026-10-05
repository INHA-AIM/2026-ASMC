#!/usr/bin/env python3
"""GT scoring for a fixed method sweep; no GT enters candidate inference."""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path

import numpy as np

from evaluate import CLASSES, ACTORS, clusters, voxelize


def digest(path):
    h = hashlib.sha256()
    with open(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def section(i):
    if i < 800:
        return "road_only_0000_0799"
    if i < 1200:
        return "development_0800_1199"
    return "temporal_check_1200_end"


def summary(raw):
    return {str(k): dict(v) if isinstance(v, Counter) else v for k, v in raw.items()}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument("result", type=Path)
    parser.add_argument("baseline", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--cluster-stride", type=int, default=10)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    buckets = defaultdict(lambda: defaultdict(lambda: {"classes": defaultdict(Counter),
                       "objects": defaultdict(Counter), "coupled": defaultdict(Counter),
                       "cluster": Counter(), "time_ms": []}))
    with open(args.input / "snapshots.jsonl") as snapshots, open(args.result) as stream, open(args.baseline) as old:
        count = 0
        for count, lines in enumerate(zip(snapshots, stream, old), 1):
            source, current, frozen = map(json.loads, lines)
            if source["input_id"] != current["input_id"] or current["input_id"] != frozen["input_id"]:
                raise ValueError("frame identity drift")
            if current["stamp_ns"] != source["scan_effective_stamp_ns"]:
                raise ValueError("observation stamp drift")
            cloud = np.fromfile(args.input / source["cloud_path"], dtype="<f4").reshape(-1, 4)
            with np.load(args.input / "labels" / (source["input_id"] + ".npz")) as labels:
                semantic, instance = labels["semantic"], labels["instance"]
                methods = {m: np.asarray(row["mask"], dtype=np.uint8)
                           for m, row in current["methods"].items()}
                old_mask = np.asarray(frozen["mask"], dtype=np.uint8)
                if any(len(a) != len(cloud) for a in (*methods.values(), old_mask, semantic, instance)):
                    raise ValueError("point count drift")
                if not np.array_equal(methods["pp_h180"], old_mask):
                    raise ValueError("Patchwork++ frozen mask parity drift at " + source["input_id"])
                roi = old_mask != 0
                if any(not np.array_equal(mask != 0, roi) for mask in methods.values()):
                    raise ValueError("ROI drift at " + source["input_id"])
                batch = buckets[section(count - 1)]
                members = {c: roi & np.isin(semantic, codes) for c, codes in CLASSES.items()}
                actor_observations = {}
                for c in ACTORS:
                    actor_observations[c] = [(instance == obj) & members[c]
                                             for obj in np.unique(instance[members[c]]) if obj]
                    actor_observations[c] = [a for a in actor_observations[c] if a.sum() >= 10]
                coupled = (count - 1) % args.cluster_stride == 0
                if coupled:
                    selected, inverse, centers = voxelize(cloud, roi)
                for method, mask in methods.items():
                    data = batch[method]
                    data["time_ms"].append(float(current["methods"][method]["segment_ms"]))
                    for c, member in members.items():
                        data["classes"][c].update(points=int(member.sum()),
                            removed=int(np.sum(member & (mask == 1))),
                            kept=int(np.sum(member & (mask == 2))))
                    for c, observations in actor_observations.items():
                        for actor in observations:
                            data["objects"][c].update(observations=1,
                                complete_loss=int(not np.any(actor & (mask == 2))),
                                retained_80=int(np.sum(actor & (mask == 2)) >= .8 * actor.sum()))
                    if not coupled:
                        continue
                    raw_mask = mask[selected].astype(np.int32)
                    minimum = np.full(len(centers), 255, dtype=np.int32)
                    maximum = np.zeros(len(centers), dtype=np.int32)
                    np.minimum.at(minimum, inverse, raw_mask)
                    np.maximum.at(maximum, inverse, raw_mask)
                    if not np.array_equal(minimum, maximum):
                        raise ValueError("voxel mask lineage drift")
                    cluster_id, groups = clusters(centers, minimum == 2, source["pitch"])
                    data["cluster"].update(frames=1, total=len(groups))
                    raw_cluster = cluster_id[inverse]
                    for c, observations in actor_observations.items():
                        for actor in observations:
                            actor_roi = actor[selected]
                            data["coupled"][c]["observations"] += 1
                            assigned = raw_cluster[actor_roi & (raw_cluster >= 0)]
                            if not len(assigned):
                                data["coupled"][c]["no_cluster"] += 1
                                continue
                            ids, counts = np.unique(assigned, return_counts=True)
                            cluster = int(ids[np.argmax(counts)])
                            got = int(counts.max())
                            completeness = got / int(actor.sum())
                            purity = got / int(np.sum(raw_cluster == cluster))
                            data["coupled"][c]["matched_80_90"] += int(completeness >= .8 and purity >= .9)
            if count % 200 == 0:
                print(json.dumps({"scored": count, "last": source["input_id"]}), flush=True)
        if snapshots.readline() or stream.readline() or old.readline():
            raise ValueError("unmatched stream length")
    def serialize(data):
        return {"classes": {c: {**dict(v), "removed_pct": 100 * v["removed"] / v["points"]
                                  if v["points"] else None} for c, v in data["classes"].items()},
                "objects": {c: dict(v) for c, v in data["objects"].items()},
                "coupled": {c: dict(v) for c, v in data["coupled"].items()},
                "clusters": dict(data["cluster"]),
                "segment_ms": {f"p{q}": float(np.percentile(data["time_ms"], q))
                               for q in (50, 95, 99)}}
    result = {"frames": count, "baseline_mask_exact": True,
              "method": "same source ROI/pitch/10cm voxel; PCL linefit C++ and generated fixed-height Patchwork++ source; downstream scipy Euclidean connectivity matched to PCL representative frames",
              "sections": {s: {m: serialize(d) for m, d in methods.items()}
                           for s, methods in buckets.items()},
              "sha256": {"snapshots": digest(args.input / "snapshots.jsonl"),
                         "comparison": digest(args.result), "baseline": digest(args.baseline)},
              "limits": ["The temporal check shares simulator/session conditions; not independent held-out competition driving",
                         "No verified complete-object 3D GT box; 80/90 scores cluster overlap, not OBB accuracy",
                         "Segmentation timing excludes common ROI/voxel, includes upstream linefit console writes to /dev/null; not full ROS latency",
                         "No operating LiDAR, ROS topic, tracking, map, or planner changes"]}
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"frames": count, "baseline_mask_exact": True,
                      "sections": list(result["sections"])}), flush=True)


if __name__ == "__main__":
    main()
