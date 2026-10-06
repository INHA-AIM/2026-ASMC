#!/usr/bin/env python3
"""Account for complete fusion traces without changing acceptance thresholds."""
import argparse
from collections import Counter
import json
from pathlib import Path
import statistics


def summarize(rows):
    counts = Counter(row["event"] for row in rows)
    finals = [row for row in rows if row["event"] == "fusion_summary"]
    decisions = [row for row in rows if row["event"] == "match_decision"]
    resolved = [row for row in decisions if row["action"] != "wait"]
    selected = [row for row in resolved if row["action"] == "match"]
    classes = [row for row in rows if row["event"] == "class_after"]
    ends = [row for row in rows if row["event"] == "lidar_process_end"]
    durations = sorted(row["duration_ms"] for row in ends)
    consumed = [row["selected_yolo_stamp_ns"] for row in selected]
    return {
        "events": dict(counts), "final": finals[-1] if finals else None,
        "complete": len(finals) == 1,
        "accounting_consistent": len(finals) == 1 and finals[0]["matched"] == len(classes)
        and finals[0]["dropped"] == sum(row["action"] == "drop" for row in resolved)
        + counts["camera_decode_drop"],
        "lidar_input_frames": counts["lidar_input"],
        "lidar_processed_frames": sum(row["processed"] for row in ends),
        "lidar_pose_failures": sum(not row["pose_ready"] for row in ends),
        "skipped_generations": sum(row.get("skipped_generations", 0) for row in rows
                                   if row["event"] == "lidar_process_start"),
        "duplicate_consumed_yolo_stamps": len(consumed) - len(set(consumed)),
        "offset_max_ms": max((row["offset_ns"] / 1e6 for row in selected), default=None),
        "processing_ms": {"median": statistics.median(durations) if durations else None,
                          "p95": durations[min(len(durations)-1, int(.95*len(durations)))]
                          if durations else None},
    }


def compare(reference, candidate):
    def matches(rows):
        return {row["stamp_ns"]: row for row in rows if row["event"] == "match_decision"
                and row["action"] == "match"}
    a, b = matches(reference), matches(candidate)
    common = a.keys() & b.keys()
    changes = [{"lidar_stamp_ns": stamp,
                "old_yolo_stamp_ns": a[stamp]["selected_yolo_stamp_ns"],
                "new_yolo_stamp_ns": b[stamp]["selected_yolo_stamp_ns"],
                "old_waited_ms": a[stamp]["waited_ns"]/1e6,
                "new_waited_ms": b[stamp]["waited_ns"]/1e6}
               for stamp in sorted(common)
               if a[stamp]["selected_yolo_stamp_ns"] != b[stamp]["selected_yolo_stamp_ns"]]
    return {"common_matched_lidar_stamps": len(common),
            "old_only_matched_stamps": sorted(a.keys() - b.keys()),
            "new_only_matched_stamps": sorted(b.keys() - a.keys()),
            "different_yolo_selections": changes}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    args = parser.parse_args()
    paths = sorted(args.directory.glob("*.fusion.jsonl"))
    if not paths:
        raise SystemExit("no *.fusion.jsonl files; replay with --fusion --fusion-trace")
    traces = {path.name[:-len(".fusion.jsonl")]:
              [json.loads(line) for line in path.read_text().splitlines()] for path in paths}
    result = {"runs": {name: summarize(rows) for name, rows in traces.items()}}
    if "old_1" in traces:
        result["comparisons"] = {name: compare(traces["old_1"], rows)
                                 for name, rows in traces.items() if name != "old_1"}
    result["trace_integrity_pass"] = all(row["complete"] and row["accounting_consistent"]
                                        for row in result["runs"].values())
    (args.directory / "trace_analysis.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result["runs"], indent=2))
    return 0 if result["trace_integrity_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
