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


def pair_frames(a, b, tolerance_m=0.001):
    """Mutually unique matches: two reference clusters cannot share one target."""
    def distance(x, y):
        return math.sqrt(sum((u-v)**2 for u, v in zip(x, y)))
    old, new = a["objects"], b["objects"]
    if len({o[0] for o in old}) != len(old) or len({o[0] for o in new}) != len(new):
        return []
    forward = [[j for j, target in enumerate(new)
                if distance(a["centers"][str(source[0])], b["centers"][str(target[0])]) <= tolerance_m]
               for source in old]
    reverse = [[i for i, neighbors in enumerate(forward) if j in neighbors]
               for j in range(len(new))]
    return [(old[i], new[neighbors[0]]) for i, neighbors in enumerate(forward)
            if len(neighbors) == 1 and len(reverse[neighbors[0]]) == 1]


def frames_by_stamp(rows):
    result = {row["stamp_ns"]: row for row in rows}
    if len(result) != len(rows):
        raise ValueError("duplicate LiDAR output stamps; identity comparison cannot overwrite observations")
    return result


def compare(reference, candidate, tolerance_m=0.001):
    old = frames_by_stamp(reference)
    new = frames_by_stamp(candidate)
    report = {"common_lidar_stamps": len(old.keys() & new.keys()), "matched_observations": 0,
              "unmatched_observations": 0, "unmatched_candidate_observations": 0,
              "old_only_lidar_stamps": sorted(old.keys() - new.keys()),
              "new_only_lidar_stamps": sorted(new.keys() - old.keys()),
              "reassigned_ids": 0, "classification_differences": []}
    for stamp in sorted(old.keys() & new.keys()):
        a, b = old[stamp], new[stamp]
        pairs = pair_frames(a, b, tolerance_m)
        report["unmatched_observations"] += len(a["objects"]) - len(pairs)
        report["unmatched_candidate_observations"] += len(b["objects"]) - len(pairs)
        for obj, other in pairs:
            center = a["centers"][str(obj[0])]
            report["matched_observations"] += 1
            report["reassigned_ids"] += obj[0] != other[0]
            if obj[1:] != other[1:]:
                report["classification_differences"].append({"stamp_ns": stamp, "old": obj, "new": other,
                                                              "center": center})
    return report


def compare_stable_observations(old_runs, new_runs):
    tables = [frames_by_stamp(rows) for rows in old_runs + new_runs]
    common = set.intersection(*(set(table) for table in tables))
    stable, regressions, unmatched = 0, [], 0
    for stamp in sorted(common):
        reference = tables[0][stamp]
        paired = [{a[0]: b for a, b in pair_frames(reference, table[stamp])}
                  for table in tables[1:]]
        for obj in reference["objects"]:
            if any(obj[0] not in mapping for mapping in paired):
                unmatched += 1
                continue
            candidates = [mapping[obj[0]] for mapping in paired]
            if any(candidate[1:] != obj[1:] for candidate in candidates[:len(old_runs)-1]):
                continue
            stable += 1
            if any(candidate[1:] != obj[1:] for candidate in candidates[len(old_runs)-1:]):
                regressions.append({"stamp_ns": stamp, "center": reference["centers"][str(obj[0])],
                                    "old": obj, "new": candidates[len(old_runs)-1:]})
    return {"common_lidar_stamps": len(common), "old_stable": stable,
            "unmatched_common_observations": unmatched, "regressions": regressions}


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
