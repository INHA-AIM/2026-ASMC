#!/usr/bin/env python3
"""Unlabeled manual-drive proxy for the exact-voxel adaptive review candidate."""
import argparse
import json
from pathlib import Path

import numpy as np


METHODS = ("pp_h180", "pp_all_guard_25cm", "pp_adaptive_guard_25cm")


def summarize(values):
    return {"p50": float(np.percentile(values, 50)),
            "p95": float(np.percentile(values, 95)),
            "p99": float(np.percentile(values, 99))} if values else None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("snapshots", type=Path)
    parser.add_argument("results", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    rows = []
    with args.snapshots.open() as snapshots, args.results.open() as results:
        for source_line, result_line in zip(snapshots, results):
            source, result = json.loads(source_line), json.loads(result_line)
            if source["input_id"] != result["input_id"]:
                raise ValueError("frame identity drift")
            cloud = np.fromfile(args.snapshots.parent / source["cloud_path"],
                                dtype="<f4").reshape(-1, 4)
            masks = {name: np.asarray(result["methods"][name]["mask"], dtype=np.uint8)
                     for name in METHODS}
            if any(len(mask) != len(cloud) for mask in masks.values()):
                raise ValueError("mask length drift")
            z = -np.sin(source["pitch"]) * cloud[:, 0] + \
                np.cos(source["pitch"]) * cloud[:, 2]
            reference_height = source["road_patch_sensor_z_median"]
            proxy = (masks["pp_h180"] != 0) & (cloud[:, 0] >= 5) & \
                (cloud[:, 0] <= 15) & (np.abs(cloud[:, 1]) <= 3) & \
                (np.abs(z - reference_height) <= .12)
            fraction = {name: float(np.mean(mask[proxy] == 1)) if proxy.any() else None
                        for name, mask in masks.items()}
            candidate = result["methods"]["pp_adaptive_guard_25cm"]
            rows.append({"input_id": source["input_id"], "proxy_points": int(proxy.sum()),
                         "fractions": fraction,
                         "support_count": candidate["support_count"],
                         "estimated_floor_z": candidate["estimated_floor_z"],
                         "road_patch_sensor_z_median": reference_height})
        if snapshots.readline() or results.readline():
            raise ValueError("record count mismatch")
    usable = [r for r in rows if r["proxy_points"] >= 10]
    report = {"frames": len(rows), "proxy_frames_ge_10_points": len(usable),
              "ground_truth": False,
              "warning": "Near-lane road-height band is an unlabeled proxy; not semantic GT or a slope acceptance test.",
              "support_count": summarize([r["support_count"] for r in rows]),
              "support_missing_frames": sum(r["support_count"] < 30 for r in rows),
              "estimated_floor_z": summarize([r["estimated_floor_z"] for r in rows
                                               if r["estimated_floor_z"] is not None]),
              "road_patch_sensor_z_median": summarize([r["road_patch_sensor_z_median"] for r in rows]),
              "methods": {name: {
                  "proxy_ground_fraction": summarize([r["fractions"][name] for r in usable]),
                  "regression_gt_5pp_vs_baseline": sum(
                      r["fractions"]["pp_h180"] - r["fractions"][name] > .05
                      for r in usable),
              } for name in METHODS},
              "worst_adaptive_frames": sorted(usable,
                  key=lambda r: r["fractions"]["pp_h180"] -
                                r["fractions"]["pp_adaptive_guard_25cm"],
                  reverse=True)[:10]}
    args.output.write_text(json.dumps(report, sort_keys=True, indent=2) + "\n")
    print(json.dumps({"frames": len(rows), "proxy_frames": len(usable),
                      "adaptive_regressions": report["methods"]["pp_adaptive_guard_25cm"]["regression_gt_5pp_vs_baseline"]}))


if __name__ == "__main__":
    main()
