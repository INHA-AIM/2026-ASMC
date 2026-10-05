#!/usr/bin/env python3
"""Unlabelled IMU sequence: posture and ground-fit evidence, never GT accuracy."""
import argparse
import itertools
import json
from pathlib import Path

import numpy as np


def run(source, old, new, output):
    if output.exists():
        raise FileExistsError(output)
    observations = []
    with (source / "snapshots.jsonl").open() as snapshots, old.open() as v1, new.open() as rp:
        for lines in itertools.zip_longest(snapshots, v1, rp):
            if None in lines:
                raise ValueError("unequal sequence lengths")
            src, base, now = map(json.loads, lines)
            if src["input_id"] != base["input_id"] or src["input_id"] != now["input_id"] or \
                    src["scan_effective_stamp_ns"] != now["stamp_ns"] or \
                    base["raw_to_voxel"] != now["raw_to_voxel"]:
                raise ValueError("identity/stamp/ROI mismatch")
            row = {"input_id": src["input_id"], "roll_deg": float(np.degrees(src["roll"])),
                   "pitch_deg": float(np.degrees(src["pitch"])), "methods": {}}
            for name, item in (("jaeho_v1", base["methods"]["jaeho"]),
                               ("jaeho_rp", now["methods"]["jaeho"]),
                               ("patchwork_v1", base["methods"]["patchwork"]),
                               ("patchwork_rp", now["methods"]["patchwork"]),
                               ("linefit_v1", base["methods"]["linefit"]),
                               ("linefit_rp", now["methods"]["linefit"])):
                mask = np.asarray(item["mask"], np.uint8)
                row["methods"][name] = {"ground": int(np.count_nonzero(mask == 1)),
                                        "nonground": int(np.count_nonzero(mask == 2)),
                                        "unclassified": int(np.count_nonzero(mask == 3)),
                                        "segment_ms": item["segment_ms"]}
            planes = now["methods"]["jaeho"].get("plane_attempts", [])
            row["jaeho_ground_planes"] = [p for p in planes if p["accepted_ground"]]
            row["jaeho_fallback"] = now["methods"]["jaeho"]["events"].get("ground_fallback", 0)
            observations.append(row)
    output.parent.mkdir(parents=True, exist_ok=True)
    changes = {}
    for before, after in (("jaeho_v1", "jaeho_rp"), ("patchwork_v1", "patchwork_rp"),
                          ("linefit_v1", "linefit_rp")):
        difference = np.asarray([r["methods"][after]["nonground"] -
                                 r["methods"][before]["nonground"] for r in observations])
        changes[after] = {"frames_changed": int(np.count_nonzero(difference)),
                          "median_nonground_delta": float(np.median(difference)),
                          "p95_abs_nonground_delta": float(np.percentile(np.abs(difference), 95))}
    result = {"frames": len(observations), "gt_available": False,
              "roll_range_deg": [min(r["roll_deg"] for r in observations),
                                 max(r["roll_deg"] for r in observations)],
              "changes": changes,
              "positive_height_ground_frames": sum(any(p["height_valid"] and p["height_at_origin_m"] > 0
                  for p in r["jaeho_ground_planes"]) for r in observations),
              "fallback_frames": sum(r["jaeho_fallback"] for r in observations),
              "observations": observations}
    output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({k: v for k, v in result.items() if k != "observations"}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    for field in ("source", "old", "new", "output"):
        parser.add_argument(field, type=Path)
    args = parser.parse_args()
    run(args.source, args.old, args.new, args.output)
