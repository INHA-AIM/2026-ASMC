#!/usr/bin/env python3
"""Unlabeled drive diagnostic; road-height band is a proxy, NEVER GT."""
import argparse
import json
from pathlib import Path

import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("snapshots", type=Path)
    parser.add_argument("results", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    records = []
    with args.snapshots.open() as snaps, args.results.open() as results:
        for source_line, result_line in zip(snaps, results):
            source, result = json.loads(source_line), json.loads(result_line)
            if source["input_id"] != result["input_id"]:
                raise ValueError("frame identity drift")
            cloud = np.fromfile(args.snapshots.parent / source["cloud_path"],
                                dtype="<f4").reshape(-1, 4)
            masks = {name: np.asarray(row["mask"], dtype=np.uint8)
                     for name, row in result["methods"].items()}
            if any(len(mask) != len(cloud) for mask in masks.values()):
                raise ValueError("mask length drift")
            valid = masks["pp_h180"] != 0
            height = float(source["road_patch_sensor_z_median"])
            # Same sensor-origin raw cloud and pitch-only leveling as the review.
            pitch = float(source["pitch"])
            z_level = -np.sin(pitch) * cloud[:, 0] + np.cos(pitch) * cloud[:, 2]
            proxy = valid & (cloud[:, 0] >= 5) & (cloud[:, 0] <= 15) & \
                (np.abs(cloud[:, 1]) <= 3) & (np.abs(z_level - height) <= .12)
            baseline = masks["pp_h180"]
            candidates = {}
            for name in ("pp_all_guard_25cm", "pp_all_guard_15cm", "pp_far_guard_25cm"):
                mask = masks[name]
                if not np.array_equal(mask != 0, valid):
                    raise ValueError("ROI drift")
                candidates[name] = {
                    "reclassified_ground_to_nonground": int(np.sum((baseline == 1) & (mask == 2))),
                    "proxy_ground_fraction": float(np.mean(mask[proxy] == 1)) if proxy.any() else None,
                }
            records.append({"input_id": source["input_id"],
                            "road_patch_sensor_z_median": height,
                            "pitch_rad": pitch,
                            "roll_rad": float(source["roll"]),
                            "proxy_points": int(proxy.sum()),
                            "baseline_proxy_ground_fraction":
                                float(np.mean(baseline[proxy] == 1)) if proxy.any() else None,
                            "candidates": candidates})
        if snaps.readline() or results.readline():
            raise ValueError("record count mismatch")
    def percentiles(values):
        return {"p50": float(np.percentile(values, 50)),
                "p95": float(np.percentile(values, 95)),
                "p99": float(np.percentile(values, 99))} if values else None
    populated = [row for row in records if row["proxy_points"] >= 10]
    report = {"frames": len(records), "proxy_frames_ge_10_points": len(populated),
              "ground_truth": False,
              "warning": "Road-height/lateral band is not semantic GT and may include non-road points; only a regression diagnostic.",
              "road_patch_z": percentiles([r["road_patch_sensor_z_median"] for r in records]),
              "pitch_abs_rad": percentiles([abs(r["pitch_rad"]) for r in records]),
              "baseline_proxy_ground_fraction":
                  percentiles([r["baseline_proxy_ground_fraction"] for r in populated]),
              "candidates": {name: {
                  "reclassified_per_frame": percentiles([r["candidates"][name]["reclassified_ground_to_nonground"] for r in records]),
                  "proxy_ground_fraction": percentiles([r["candidates"][name]["proxy_ground_fraction"] for r in populated]),
                  "proxy_fraction_drop_gt_5pp_frames": sum(
                      r["baseline_proxy_ground_fraction"] - r["candidates"][name]["proxy_ground_fraction"] > .05
                      for r in populated),
              } for name in ("pp_all_guard_25cm", "pp_all_guard_15cm", "pp_far_guard_25cm")},
              "worst_frames": sorted(populated, key=lambda r: r["baseline_proxy_ground_fraction"] -
                                     r["candidates"]["pp_all_guard_25cm"]["proxy_ground_fraction"],
                                     reverse=True)[:10]}
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"frames": report["frames"], "proxy_frames": len(populated),
                      "guard25_drop_gt_5pp": report["candidates"]["pp_all_guard_25cm"]["proxy_fraction_drop_gt_5pp_frames"]}))


if __name__ == "__main__":
    main()
