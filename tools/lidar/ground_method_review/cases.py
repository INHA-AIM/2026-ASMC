#!/usr/bin/env python3
"""Inspect the few temporal-check actor decisions without changing a candidate."""
import argparse
import json
from pathlib import Path

import numpy as np

from evaluate import CLASSES, clusters, voxelize


def main():
    p = argparse.ArgumentParser()
    p.add_argument("input", type=Path)
    p.add_argument("result", type=Path)
    p.add_argument("output", type=Path)
    a = p.parse_args()
    if a.output.exists():
        raise FileExistsError(a.output)
    methods = ("jaeho_reference", "pp_h180", "lf_90x60_gap2", "lf_90x40_gap3")
    records = []
    with (a.input / "snapshots.jsonl").open() as snaps, a.result.open() as results:
        for ordinal, (s, r) in enumerate(zip(snaps, results)):
            if ordinal < 1200 or ordinal % 10:
                continue
            source, result = json.loads(s), json.loads(r)
            if source["input_id"] != result["input_id"]:
                raise ValueError("identity drift")
            cloud = np.fromfile(a.input / source["cloud_path"], dtype="<f4").reshape(-1, 4)
            with np.load(a.input / "labels" / (source["input_id"] + ".npz")) as gt:
                semantic, instance = gt["semantic"], gt["instance"]
                raw_masks = {name: np.asarray(result["methods"][name]["mask"], dtype=np.uint8)
                             for name in methods}
                roi = raw_masks["jaeho_reference"] != 0
                selected, inverse, centers = voxelize(cloud, roi)
                actor_points = roi & np.isin(semantic, CLASSES["vehicle"])
                for obj in np.unique(instance[actor_points]):
                    if not obj:
                        continue
                    actor = actor_points & (instance == obj)
                    if actor.sum() < 10:
                        continue
                    record = {"frame": source["input_id"], "instance": int(obj),
                              "original_actor_points": int(actor.sum()), "methods": {}}
                    for name, mask in raw_masks.items():
                        voxel_mask = np.full(len(centers), -1, dtype=np.int32)
                        voxel_mask[inverse] = mask[selected]
                        cluster_id, groups = clusters(centers, voxel_mask == 2, source["pitch"])
                        assigned = cluster_id[inverse]
                        ids, counts = np.unique(assigned[actor[selected] & (assigned >= 0)], return_counts=True)
                        if len(ids):
                            best, best_count = int(ids[np.argmax(counts)]), int(counts.max())
                            purity = best_count / int(np.sum(assigned == best))
                            completeness = best_count / int(actor.sum())
                            cluster_size = len(groups[best])
                        else:
                            purity, completeness, cluster_size = 0., 0., 0
                        record["methods"][name] = {"ground_removed": int(np.sum(actor & (mask == 1))),
                            "completeness": round(completeness, 5), "purity": round(purity, 5),
                            "cluster_voxels": cluster_size,
                            "matched_80_90": completeness >= .8 and purity >= .9}
                    records.append(record)
    a.output.write_text(json.dumps(records, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"vehicle_observations": len(records), "output": str(a.output)}))


if __name__ == "__main__":
    main()
