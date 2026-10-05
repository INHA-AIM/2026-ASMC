#!/usr/bin/env python3
"""Independent raw-bag check without copying large baseline mask streams."""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path

import numpy as np

from evaluate import ACTORS, CLASSES, clusters, voxelize


def sha(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def main():
    p = argparse.ArgumentParser()
    p.add_argument("input", type=Path)
    p.add_argument("linefit", type=Path)
    p.add_argument("patchwork", type=Path)
    p.add_argument("jaeho", type=Path)
    p.add_argument("output", type=Path)
    p.add_argument("--cluster-stride", type=int, default=20)
    a = p.parse_args()
    if a.output.exists():
        raise FileExistsError(a.output)
    stats = defaultdict(lambda: {"classes": defaultdict(Counter), "objects": defaultdict(Counter),
                                 "coupled": defaultdict(Counter), "clusters": Counter()})
    with (a.input / "snapshots.jsonl").open() as snaps, a.linefit.open() as line, \
         a.patchwork.open() as pp, a.jaeho.open() as ja:
        count = 0
        for count, rows in enumerate(zip(snaps, line, pp, ja), 1):
            source, candidate, patch, reference = map(json.loads, rows)
            if len({source["input_id"], candidate["input_id"], patch["input_id"], reference["input_id"]}) != 1:
                raise ValueError("frame identity drift")
            if candidate["stamp_ns"] != source["scan_effective_stamp_ns"]:
                raise ValueError("stamp drift")
            cloud = np.fromfile(a.input / source["cloud_path"], dtype="<f4").reshape(-1, 4)
            with np.load(a.input / "labels" / (source["input_id"] + ".npz")) as gt:
                semantic, instance = gt["semantic"], gt["instance"]
                methods = {m: np.asarray(v["mask"], dtype=np.uint8)
                           for m, v in candidate["methods"].items()}
                methods.update(pp_h180=np.asarray(patch["mask"], dtype=np.uint8),
                               jaeho_reference=np.asarray(reference["mask"], dtype=np.uint8))
                if any(len(mask) != len(cloud) for mask in methods.values()):
                    raise ValueError("mask length drift")
                roi = methods["jaeho_reference"] != 0
                if any(not np.array_equal(mask != 0, roi) for mask in methods.values()):
                    raise ValueError("ROI drift")
                class_members = {c: roi & np.isin(semantic, codes) for c, codes in CLASSES.items()}
                actors = {c: [(instance == obj) & class_members[c]
                              for obj in np.unique(instance[class_members[c]]) if obj]
                          for c in ACTORS}
                actors = {c: [x for x in rows if x.sum() >= 10] for c, rows in actors.items()}
                coupled = (count - 1) % a.cluster_stride == 0
                if coupled:
                    selected, inverse, centers = voxelize(cloud, roi)
                for method, mask in methods.items():
                    stat = stats[method]
                    for c, member in class_members.items():
                        stat["classes"][c].update(points=int(member.sum()),
                           removed=int(np.sum(member & (mask == 1))), kept=int(np.sum(member & (mask == 2))))
                    for c, observations in actors.items():
                        for actor in observations:
                            stat["objects"][c].update(observations=1,
                                complete_loss=int(not np.any(actor & (mask == 2))))
                    if not coupled:
                        continue
                    raw_mask = mask[selected].astype(np.int32)
                    voxel_mask = np.full(len(centers), -1, dtype=np.int32)
                    voxel_mask[inverse] = raw_mask
                    if not np.array_equal(voxel_mask[inverse], raw_mask):
                        raise ValueError("voxel lineage drift")
                    ids, groups = clusters(centers, voxel_mask == 2, source["pitch"])
                    stat["clusters"].update(frames=1, total=len(groups))
                    assigned = ids[inverse]
                    for c, observations in actors.items():
                        for actor in observations:
                            stat["coupled"][c]["observations"] += 1
                            found, amounts = np.unique(assigned[actor[selected] & (assigned >= 0)],
                                                       return_counts=True)
                            if not len(found):
                                continue
                            cluster = int(found[np.argmax(amounts)])
                            got = int(amounts.max())
                            complete = got / int(actor.sum())
                            pure = got / int(np.sum(assigned == cluster))
                            stat["coupled"][c]["matched_80_90"] += int(complete >= .8 and pure >= .9)
            if count % 500 == 0:
                print(json.dumps({"scored": count}), flush=True)
        if snaps.readline() or line.readline() or pp.readline() or ja.readline():
            raise ValueError("stream length mismatch")
    def serialize(s):
        return {"classes": {c: {**dict(v), "removed_pct": 100 * v["removed"] / v["points"]
                                  if v["points"] else None} for c, v in s["classes"].items()},
                "objects": {c: dict(v) for c, v in s["objects"].items()},
                "coupled": {c: dict(v) for c, v in s["coupled"].items()},
                "clusters": dict(s["clusters"])}
    report = {"frames": count, "cluster_stride": a.cluster_stride,
              "methods": {m: serialize(s) for m, s in stats.items()},
              "sha256": {"linefit": sha(a.linefit), "patchwork": sha(a.patchwork),
                         "jaeho": sha(a.jaeho), "snapshots": sha(a.input / "snapshots.jsonl")},
              "limits": ["Same sensor and simulator environment, not an independent competition run",
                         "GT actor instances are repeated observations; class labels may be incomplete",
                         "Downstream is matched 1.5m/10-10000 connectivity, not operating tracker or planner"]}
    a.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"frames": count, "methods": sorted(stats)}))


if __name__ == "__main__":
    main()
