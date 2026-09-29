#!/usr/bin/env python3
"""First-hit analytic rays for clustering regressions; NOT a MORAI VLP16 model."""
import argparse
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from tools.lidar.clustering_compare.projection import canonical, sha

CASES = ("adjacent_separated", "adjacent_close", "wall_car", "beam_hole",
         "occluded", "far_sparse", "azimuth_seam", "wall_only")


def cloud(name):
    az, el = np.meshgrid(np.deg2rad(np.arange(-180., 180., .3)), np.deg2rad(np.arange(-15., 16., 2.)))
    rays = np.column_stack((np.cos(el.ravel())*np.cos(az.ravel()),
                            np.cos(el.ravel())*np.sin(az.ravel()), np.sin(el.ravel())))
    ranges = np.full(len(rays), np.inf)
    labels = np.full(len(rays), -1)

    def offer(distance, valid, key):
        mask = valid & np.isfinite(distance) & (distance > .5) & (distance < ranges)
        ranges[mask] = distance[mask]
        labels[mask] = key

    for z, key in ((-1.45, 0), (3., 2)):
        d = np.divide(z, rays[:,2], out=np.full(len(rays), np.inf), where=abs(rays[:,2]) > 1e-9)
        xyz = rays * np.where(np.isfinite(d), d, 0)[:,None]
        offer(d, (xyz[:,0] >= -30) & (xyz[:,0] <= 50) & (abs(xyz[:,1]) <= 3.6), key)
    for y in (-3.6, 3.6):
        d = np.divide(y, rays[:,1], out=np.full(len(rays), np.inf), where=abs(rays[:,1]) > 1e-9)
        xyz = rays * np.where(np.isfinite(d), d, 0)[:,None]
        offer(d, (xyz[:,0] >= -30) & (xyz[:,0] <= 50) & (xyz[:,2] >= -1.45) & (xyz[:,2] <= 3), 1)
    boxes = []
    if name.startswith("adjacent"):
        y = 1.9 if name == "adjacent_separated" else 1.05
        boxes = [(3, (12,-y,-.7), (4,1.8,1.1)), (5, (12,y,-.7), (4,1.8,1.1))]
    elif name != "wall_only":
        x = -12 if name == "azimuth_seam" else 40 if name == "far_sparse" else 12
        y = 2.3 if name == "wall_car" else 0
        boxes = [(3, (x,y,-.7), (4,1.8,1.1))]
        if name == "occluded":
            boxes.append((4, (7,-.55,-.7), (.2,.55,1.1)))
    for key, center, size in boxes:
        low, high = np.asarray(center)-np.asarray(size)/2, np.asarray(center)+np.asarray(size)/2
        near, far = [], []
        for axis in range(3):
            direction = rays[:,axis]
            a = np.divide(low[axis], direction, out=np.full(len(rays), np.inf), where=abs(direction)>1e-9)
            b = np.divide(high[axis], direction, out=np.full(len(rays), -np.inf), where=abs(direction)>1e-9)
            lo, hi = np.minimum(a,b), np.maximum(a,b)
            parallel = abs(direction)<=1e-9
            inside = low[axis] <= 0 <= high[axis]
            lo[parallel] = -np.inf if inside else np.inf
            hi[parallel] = np.inf if inside else -np.inf
            near.append(lo); far.append(hi)
        entry, leave = np.max(near,axis=0), np.min(far,axis=0)
        offer(np.where(entry>0,entry,leave), leave>=np.maximum(entry,0), key)
    valid = np.isfinite(ranges) & (ranges<80)
    if name == "beam_hole":
        # Deliberately unavailable azimuth columns, applied to every surface,
        # not a label-driven mask and not a real sensor timing assertion.
        valid &= abs(np.rad2deg(az.ravel())) > .6
    points = np.column_stack((rays[valid]*ranges[valid,None], np.full(valid.sum(), 10))).astype("<f4")
    return points, labels[valid]


def generate(output):
    if output.exists():
        raise ValueError("output exists")
    output.mkdir(parents=True)
    (output / "clouds").mkdir()
    rows, truth = [], []
    for index, name in enumerate(CASES):
        points, ids = cloud(name)
        path = output / "clouds" / (name + ".bin")
        points.tofile(path)
        for state in ("empty", "missing"):
            for warm in range(5):
                key = "l1b5_" + name + "_" + state + "_frame" + str(warm)
                stamp = 500_000_000_000 + (index*1000 + warm*100)*1_000_000
                decision = {"selected_yolo_index": 0 if state=="empty" else None, "boxes": [],
                            "protection_enabled": True, "decision_ns": stamp, "wait_ns": 0,
                            "selected_yolo_stamp_ns": stamp if state=="empty" else None,
                            "selected_yolo_arrival_ns": stamp-1 if state=="empty" else None}
                rows.append({"input_id": key, "scene": name, "phase": name+state, "measurement": warm==4,
                             "geometry_valid": True, "already_preprocessed": False, "roll": 0, "pitch": 0,
                             "yaw": 0, "speed_mps": 0, "scan_stamp_ns": stamp, "scan_effective_stamp_ns": stamp,
                             "scan_arrival_ns": stamp, "cloud_path": str(path.resolve()), "cloud_sha256": sha(path),
                             "controlled_camera_state": state, "pose_evidence": {},
                             "fixture_family": "l1b5_analytic_first_return", "baseline": decision})
                truth.append({"input_id": key, "kind": name, "labels": ids.tolist()})
    for filename, values in (("snapshots.jsonl",rows), ("truth.jsonl",truth)):
        (output / filename).write_text("".join(canonical(v)+"\n" for v in values))
    (output / "manifest.json").write_text(json.dumps({"schema": "asmc-l1b5a-fixtures-v1", "calls": len(rows),
        "cases": CASES, "generator_sha256": sha(Path(__file__)), "runtime_connected": False,
        "artifacts": {str(p.relative_to(output)): sha(p) for p in output.rglob("*") if p.is_file()},
        "limits": ["Analytic first-hit rays, not physical VLP16 simulation", "Camera empty/missing are controlled states",
                   "Only post-hoc truth sees object IDs", "All observations are stationary"]},indent=2,sort_keys=True)+"\n")


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", type=Path, required=True)
    generate(p.parse_args().output)
