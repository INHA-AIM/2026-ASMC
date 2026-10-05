#!/usr/bin/env python3
"""Diagnose selected-plane jumps; never call unlabeled masks ground truth."""
import argparse
import json
from pathlib import Path


def rows(path):
    with path.open() as stream:
        return [json.loads(line) for line in stream]


def selected(row):
    return next((plane for plane in row["methods"]["jaeho"].get("plane_attempts", [])
                 if plane["accepted_ground"]), None)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("pitch", type=Path)
    parser.add_argument("roll_pitch", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    source, old, new = map(rows, (args.source, args.pitch, args.roll_pitch))
    if not (len(source) == len(old) == len(new)):
        raise ValueError("replay count mismatch")
    frames = []
    for i, (scan, before, after) in enumerate(zip(source, old, new)):
        identity = scan["input_id"]
        if any(item["input_id"] != identity for item in (before, after)):
            raise ValueError("scan identity mismatch")
        plane = selected(after)
        frames.append({"index": i, "input_id": identity,
                       "stamp_ns": scan["scan_effective_stamp_ns"],
                       "roll_deg": scan["roll"] * 180 / 3.141592653589793,
                       "pitch_deg": scan["pitch"] * 180 / 3.141592653589793,
                       "selected_height_m": plane["height_at_origin_m"] if plane else None,
                       "support_points": plane["inlier_points"] if plane else 0,
                       "ground_attempt": plane["attempt"] if plane else None,
                       "fit_attempts": len(after["methods"]["jaeho"].get("plane_attempts", [])),
                       "nonground": {method: sum(label == 2 for label in after["methods"][method]["mask"])
                                     for method in ("jaeho", "patchwork", "linefit")},
                       "jaeho_pitch_nonground": sum(label == 2 for label in before["methods"]["jaeho"]["mask"])})
    heights = [f["selected_height_m"] for f in frames]
    abrupt = []
    for i in range(1, len(frames) - 1):
        left, here, right = heights[i - 1:i + 2]
        if None in (left, here, right):
            continue
        if abs(here - left) > 0.5 and abs(here - right) > 0.5 and abs(left - right) < 0.3:
            abrupt.append(frames[i])
    result = {"frames": len(frames), "accepted": sum(h is not None for h in heights),
              "positive_accepted_height": sum(h is not None and h > 0 for h in heights),
              "above_minus_one_meter": sum(h is not None and h > -1 for h in heights),
              "selected_height_min_max": [min(h for h in heights if h is not None),
                                          max(h for h in heights if h is not None)],
              "roll_deg_min_max": [min(f["roll_deg"] for f in frames),
                                   max(f["roll_deg"] for f in frames)],
              "isolated_height_jumps_over_half_meter": abrupt,
              "limit": "Historical bag has no ground/roof GT; moving vehicle changes observed surfaces. "
                       "This reports plane selection, not detection accuracy or same-surface temporal stability."}
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"frames": len(frames), "isolated": len(abrupt),
                      "above_minus_one_meter": result["above_minus_one_meter"]}))


if __name__ == "__main__":
    main()
