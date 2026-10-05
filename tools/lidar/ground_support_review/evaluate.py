#!/usr/bin/env python3
"""Isolated, label-blind ground-support probe and matched downstream replay.

This is research code, not an operating LiDAR backend. GT is used only to
score outputs. Source masks come from the pinned same-input voxel comparison.
"""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path
import time

import numpy as np
from scipy.spatial import cKDTree
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components


CLASSES = {"asphalt": (127,), "sidewalk": (129,),
           "vehicle": (86, 125, 135, 145, 155, 165, 175, 185),
           "pedestrian": (118,), "obstacle": (164,), "standing": (109,)}
ACTORS = ("vehicle", "pedestrian", "obstacle")
METHODS = ("jaeho", "patchwork", "height_veto", "local_support")


def sha(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def arrays(root):
    streams = {"jaeho": open(root / "results/jaeho_voxel_pitch.jsonl"),
               "patchwork": open(root / "results/patchwork_fixed_voxel_pitch.jsonl")}
    try:
        with open(root / "snapshots.jsonl") as snapshots:
            for row in snapshots:
                source = json.loads(row)
                results = {name: json.loads(next(stream)) for name, stream in streams.items()}
                for result in results.values():
                    if result["input_id"] != source["input_id"] or result["stamp_ns"] != source["scan_effective_stamp_ns"]:
                        raise ValueError("source/mask identity or stamp mismatch")
                cloud = np.fromfile(root / source["cloud_path"], dtype="<f4").reshape(-1, 4)
                with np.load(root / "labels" / (source["input_id"] + ".npz")) as labels:
                    sem, ins = labels["semantic"].copy(), labels["instance"].copy()
                masks = {k: np.asarray(r["mask"], dtype=np.uint8) for k, r in results.items()}
                if any(len(v) != len(cloud) for v in (sem, ins, *masks.values())):
                    raise ValueError("source/mask point count mismatch")
                if not np.array_equal(masks["jaeho"] != 0, masks["patchwork"] != 0):
                    raise ValueError("ROI mismatch")
                yield source, cloud, sem, ins, masks
        for stream in streams.values():
            if stream.readline():
                raise ValueError("extra result rows")
    finally:
        for stream in streams.values():
            stream.close()


def guarded(cloud, pitch, pp):
    # Same pitch leveling as pinned Main; constants are declared before scoring.
    level_z = (-np.sin(pitch) * cloud[:, 0] + np.cos(pitch) * cloud[:, 2])
    high_ground = (pp == 1) & (level_z > -1.45)
    simple = pp.copy()
    simple[high_ground] = 2
    floor = (pp == 1) & (level_z >= -2.2) & (level_z <= -1.55)
    supported = np.zeros(len(pp), dtype=bool)
    if floor.sum() >= 3 and high_ground.any():
        floor_points = np.column_stack((cloud[floor, 0], cloud[floor, 1]))
        high_points = np.column_stack((cloud[high_ground, 0], cloud[high_ground, 1]))
        distance, index = cKDTree(floor_points).query(high_points, k=3, distance_upper_bound=2.0)
        valid = np.all(np.isfinite(distance), axis=1)
        high_z = level_z[high_ground]
        floor_z = level_z[floor]
        valid[valid] &= high_z[valid] - np.median(floor_z[index[valid]], axis=1) >= .35
        supported[np.flatnonzero(high_ground)] = valid
    local = pp.copy()
    local[supported] = 2
    return simple, local, int(high_ground.sum()), int(supported.sum())


def voxelize(cloud, roi):
    # PCL VoxelGrid's 10 cm cells and centroid representation. Each source
    # point retains its centroid index for GT/cluster attribution.
    selected = np.flatnonzero(roi)
    xyz = cloud[selected, :3].astype(np.float64)
    cells, inverse = np.unique(np.floor(xyz / .1).astype(np.int32), axis=0, return_inverse=True)
    counts = np.bincount(inverse)
    centers = np.column_stack([np.bincount(inverse, weights=xyz[:, i]) / counts for i in range(3)])
    return selected, inverse, centers


def clusters(centers, keep, pitch):
    indices = np.flatnonzero(keep)
    label = np.full(len(centers), -1, dtype=np.int32)
    if len(indices) < 10:
        return label, []
    xyz = centers[indices].copy()
    cp, sp = np.cos(pitch), np.sin(pitch)
    x = xyz[:, 0] * cp + xyz[:, 2] * sp
    z = -xyz[:, 0] * sp + xyz[:, 2] * cp
    xyz[:, 0], xyz[:, 2] = x, z
    pairs = cKDTree(xyz).query_pairs(1.5, output_type="ndarray")
    if len(pairs):
        edge = coo_matrix((np.ones(2 * len(pairs), dtype=np.uint8),
                           (np.r_[pairs[:, 0], pairs[:, 1]], np.r_[pairs[:, 1], pairs[:, 0]])),
                          shape=(len(indices), len(indices))).tocsr()
        _, components = connected_components(edge, directed=False)
    else:
        components = np.arange(len(indices))
    groups = []
    for c in np.unique(components):
        member = np.flatnonzero(components == c)
        if 10 <= len(member) <= 10000:
            label[indices[member]] = len(groups)
            groups.append(xyz[member])
    return label, groups


def l_shape(points):
    # Same one-degree closeness search and XY/Z ranges as PinnedLShapeObb.
    xy = points[:, :2]
    if len(points) < 10:
        return None
    best_score = -np.inf
    best = None
    for degrees in range(90):
        angle = np.deg2rad(degrees)
        c, s = np.cos(angle), np.sin(angle)
        u = xy[:, 0] * c + xy[:, 1] * s
        v = -xy[:, 0] * s + xy[:, 1] * c
        ulo, uhi, vlo, vhi = u.min(), u.max(), v.min(), v.max()
        distance = np.minimum(np.minimum(u - ulo, uhi - u), np.minimum(v - vlo, vhi - v))
        score = float(np.sum(1.0 / np.maximum(distance, .05)))
        if score > best_score:
            best_score, best = score, (angle, uhi - ulo, vhi - vlo)
    angle, length, width = best
    if length < width:
        length, width = width, length
        angle += np.pi / 2
    return {"length_m": float(length), "width_m": float(width),
            "height_m": float(np.ptp(points[:, 2])), "yaw_rad": float(angle)}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("root", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--cluster-stride", type=int, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    total = {m: {c: Counter() for c in CLASSES} for m in METHODS}
    object_counts = {m: {c: Counter() for c in ACTORS} for m in METHODS}
    coupled = {m: {c: Counter() for c in ACTORS} for m in METHODS}
    cluster_totals = {m: Counter() for m in METHODS}
    obb_examples = []
    timing = {m: [] for m in METHODS}
    ground_timing = []
    frames = 0
    for ordinal, (source, cloud, sem, ins, base_masks) in enumerate(arrays(args.root)):
        frames += 1
        started = time.perf_counter()
        simple, local, high, support = guarded(cloud, source["pitch"], base_masks["patchwork"])
        ground_timing.append((time.perf_counter() - started) * 1000)
        masks = {**base_masks, "height_veto": simple, "local_support": local}
        roi = masks["jaeho"] != 0
        grouped = {c: roi & np.isin(sem, codes) for c, codes in CLASSES.items()}
        for method, mask in masks.items():
            for c, member in grouped.items():
                total[method][c].update(points=int(member.sum()), ground_removed=int(np.sum(member & (mask == 1))),
                                         retained=int(np.sum(member & (mask == 2))))
            for c in ACTORS:
                for instance in np.unique(ins[grouped[c]]):
                    if not instance:
                        continue
                    member = grouped[c] & (ins == instance)
                    n = int(member.sum())
                    if n < 10:
                        continue
                    object_counts[method][c].update(observations=1, complete_loss=int(np.sum(member & (mask == 2)) == 0))
        if ordinal % args.cluster_stride:
            continue
        selected, inverse, centers = voxelize(cloud, roi)
        for method, mask in masks.items():
            # All raw members of one voxel must inherit the same frozen mask.
            as_int = mask[selected].astype(np.int32)
            lo = np.full(len(centers), 255, dtype=np.int32)
            hi = np.zeros(len(centers), dtype=np.int32)
            np.minimum.at(lo, inverse, as_int)
            np.maximum.at(hi, inverse, as_int)
            if not np.array_equal(lo, hi):
                # A point-level guard can split a voxel; conservative voxel
                # decision keeps it only when all source points are retained.
                if method in ("jaeho", "patchwork"):
                    raise ValueError("frozen voxel mask inconsistent")
            voxel_mask = lo if method in ("jaeho", "patchwork") else np.where(lo == hi, lo, 1)
            started = time.perf_counter()
            group_id, group_points = clusters(centers, voxel_mask == 2, source["pitch"])
            timing[method].append((time.perf_counter() - started) * 1000)
            cluster_totals[method].update(frames=1, clusters=len(group_points), kept_voxels=int(np.sum(voxel_mask == 2)))
            raw_cluster = group_id[inverse]
            valid = raw_cluster >= 0
            for c in ACTORS:
                members = grouped[c][selected]
                for instance in np.unique(ins[selected][members]):
                    if not instance:
                        continue
                    actor = members & (ins[selected] == instance)
                    size = int(actor.sum())
                    if size < 10:
                        continue
                    stats = coupled[method][c]
                    stats["observations"] += 1
                    ids, counts = np.unique(raw_cluster[actor & valid], return_counts=True)
                    if not len(ids):
                        stats["no_cluster"] += 1
                        continue
                    k = int(ids[np.argmax(counts)])
                    got = int(counts.max())
                    purity = got / max(1, int(np.sum(raw_cluster == k)))
                    completeness = got / size
                    if completeness >= .8:
                        stats["complete_80"] += 1
                    if purity >= .9:
                        stats["pure_90"] += 1
                    if completeness >= .8 and purity >= .9:
                        stats["matched_80_90"] += 1
                    if got >= 10 and len(obb_examples) < 800:
                        box = l_shape(group_points[k])
                        obb_examples.append({"input_id": source["input_id"], "method": method,
                                             "class": c, "instance": int(instance),
                                             "completeness": completeness, "purity": purity,
                                             "obb": box})
        if ordinal % 100 == 0:
            print(json.dumps({"dataset": args.root.name, "processed": ordinal + 1}), flush=True)
    def rates(counters):
        return {m: {c: {**dict(v), "ground_removed_pct": 100*v["ground_removed"]/v["points"] if v["points"] else None}
                    for c, v in classes.items()} for m, classes in counters.items()}
    def pct(v):
        return {str(q): float(np.percentile(v, q)) for q in (50, 95, 99)} if v else {}
    result = {"dataset": args.root.name, "frames": frames, "cluster_stride": args.cluster_stride,
              "input_sha256": {p.name: sha(p) for p in [args.root / "snapshots.jsonl",
                  args.root / "results/jaeho_voxel_pitch.jsonl", args.root / "results/patchwork_fixed_voxel_pitch.jsonl"]},
              "method": "same frozen pitch-only/10cm voxel masks; point-level 1.8m height veto; local support: 3 lower PP-ground points within 2m, z [-2.2,-1.55], height difference >=0.35; 1.5m Euclidean connectivity, 10-10000 voxels; 1deg pinned L-shape port",
              "ground_classes": rates(total), "ground_objects": {m: {c: dict(v) for c,v in g.items()} for m,g in object_counts.items()},
              "coupled_objects": {m: {c: dict(v) for c,v in g.items()} for m,g in coupled.items()},
              "coupled_clusters": {m: dict(v) for m,v in cluster_totals.items()},
              "timing_ms_python": {"support_only": pct(ground_timing), "cluster_only": {m:pct(v) for m,v in timing.items()}},
              "obb_examples": obb_examples,
              "limits": ["GT scores reused analyzed scenes; not independent holdout", "Python scipy connectivity/L-shape replay, not bitwise Main/PCL parity or runtime benchmark", "GT instance is used only for scoring, not candidate inference", "Final detection OBB boxes are unverified; only cluster overlap and OBB dimensions are scored", "No tracking, wall/ceiling, ROS publish or planner output"]}
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"dataset": args.root.name, "frames": frames, "coupled": {m:dict(v) for m,v in cluster_totals.items()}}), flush=True)


if __name__ == "__main__":
    main()
