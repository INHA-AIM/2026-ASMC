#!/usr/bin/env python3
"""Deterministic geometry probes. Labels are synthetic, not MORAI GT."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


def surface(grid_x, grid_y, z, label, identity=0):
    x, y = np.meshgrid(grid_x, grid_y, indexing="ij")
    zz = np.full_like(x, z, dtype=float)
    return np.column_stack((x.ravel(), y.ravel(), zz.ravel())), label, identity


def scene(with_tunnel):
    sections = [
        surface(np.arange(5., 26., .45), np.arange(-4., 4.1, .45), -1.8, 127),
        surface(np.arange(10., 12.6, .25), np.arange(-.9, .91, .25), -.8, 86, 1),
        surface(np.arange(7.5, 8.11, .13), np.arange(1.5, 2.11, .13), -.3, 118, 2),
        surface(np.arange(8.5, 9.41, .15), np.arange(-2.6, -1.7, .15), -1.55, 164, 3),
    ]
    if with_tunnel:
        sections.append(surface(np.arange(5., 26., .32), np.arange(-4., 4.1, .32), 2.3, 153))
        x, z = np.meshgrid(np.arange(5., 26., .45), np.arange(-1.8, 2.31, .25), indexing="ij")
        for y in (-4., 4.):
            sections.append((np.column_stack((x.ravel(), np.full(x.size, y), z.ravel())), 153, 0))
    coords = np.concatenate([s[0] for s in sections]).astype("<f4")
    semantic = np.concatenate([np.full(len(s[0]), s[1], np.uint16) for s in sections])
    instance = np.concatenate([np.full(len(s[0]), s[2], np.uint32) for s in sections])
    return coords, semantic, instance


def rotation(roll, pitch):
    cr, sr, cp, sp = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch)
    return np.array(((cp, sp * sr, sp * cr), (0., cr, -sr), (-sp, cp * sr, cp * cr)))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output / "clouds").mkdir()
    (args.output / "labels").mkdir()
    inventory = []
    with (args.output / "snapshots.jsonl").open("x") as snapshots:
        sequence = 0
        for tunnel in (False, True):
            coords, semantic, instance = scene(tunnel)
            for roll_degrees in (0, -2, 2, -5, 5):
                for pitch_degrees in (0, -3, 3):
                    sequence += 1
                    roll, pitch = np.radians((roll_degrees, pitch_degrees))
                    transformed = coords @ rotation(roll, pitch)
                    cloud = np.column_stack((transformed, np.arange(len(coords)) % 250 + 1)).astype("<f4")
                    name = f"synthetic_{sequence:03d}"
                    path = args.output / "clouds" / (name + ".bin")
                    path.write_bytes(cloud.tobytes())
                    np.savez_compressed(args.output / "labels" / (name + ".npz"),
                                        semantic=semantic, instance=instance)
                    row = {"input_id": name, "cloud_path": str(path.relative_to(args.output)),
                           "scan_effective_stamp_ns": sequence * 100000000,
                           "scan_arrival_ns": sequence * 100000000, "roll": float(roll),
                           "pitch": float(pitch), "geometry_valid": True,
                           "phase": name, "scenario": "tunnel" if tunnel else "open_road",
                           "gt_provenance": "synthetic_surfaces_not_morai"}
                    snapshots.write(json.dumps(row) + "\n")
                    inventory.append({"id": name, "scenario": row["scenario"],
                                      "roll_deg": roll_degrees, "pitch_deg": pitch_degrees,
                                      "cloud_sha256": hashlib.sha256(path.read_bytes()).hexdigest()})
    (args.output / "source.json").write_text(json.dumps({
        "basis": "fixed gravity-frame point surfaces transformed into sensor frame",
        "labels": {"127": "synthetic ground", "153": "synthetic ceiling/walls",
                   "86": "vehicle", "118": "pedestrian", "164": "low obstacle"},
        "warning": "Dense artificial surfaces; not a VLP16 or detection-accuracy simulation.",
        "frames": inventory}, indent=2) + "\n")
    print(json.dumps({"frames": len(inventory), "output": str(args.output)}))


if __name__ == "__main__":
    main()
