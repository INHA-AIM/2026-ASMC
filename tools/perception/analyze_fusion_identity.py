#!/usr/bin/env python3
"""Diagnose replay track IDs using identical source-stamped LiDAR observations.

This report supplements the strict replay gates; it does not relax them.
Only unambiguous clusters whose 3D centers differ by <=1 mm are compared.
Unmatched/ambiguous observations are counted, never treated as successes.
"""
import argparse
import json
import math
from pathlib import Path


def compare(reference, candidate, tolerance_m=0.001):
    old = {row["stamp_ns"]: row for row in reference}
    new = {row["stamp_ns"]: row for row in candidate}
    report = {"common_lidar_stamps": len(old.keys() & new.keys()), "matched_observations": 0,
              "unmatched_observations": 0, "reassigned_ids": 0, "classification_differences": []}
    for stamp in sorted(old.keys() & new.keys()):
        a, b = old[stamp], new[stamp]
        for obj in a["objects"]:
            center = a["centers"][str(obj[0])]
            nearby = [other for other in b["objects"]
                      if math.sqrt(sum((x-y)**2 for x, y in zip(center, b["centers"][str(other[0])]))) <= tolerance_m]
            if len(nearby) != 1:
                report["unmatched_observations"] += 1
                continue
            other = nearby[0]
            report["matched_observations"] += 1
            report["reassigned_ids"] += obj[0] != other[0]
            if obj[1:] != other[1:]:
                report["classification_differences"].append({"stamp_ns": stamp, "old": obj, "new": other,
                                                              "center": center})
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    args = parser.parse_args()
    files = [args.directory / (name + ".frames.json") for name in ("old_1", "old_2", "new_1", "new_2")]
    tracks = [json.loads(path.read_text())["tracks"] for path in files]
    result = {path.stem: compare(tracks[0], rows) for path, rows in zip(files[1:], tracks[1:])}
    (args.directory / "identity_analysis.json").write_text(json.dumps(result, indent=2) + "\n")
    for name, report in result.items():
        print(name, {k: len(v) if isinstance(v, list) else v for k, v in report.items()})


if __name__ == "__main__":
    main()
