#!/usr/bin/env python3
"""Account for complete fusion traces without changing acceptance thresholds."""
import argparse
from collections import Counter
import json
from pathlib import Path
import statistics

try:
    from .analyze_fusion_identity import compare as compare_identity
except ImportError:
    from analyze_fusion_identity import compare as compare_identity


def compare_boxes(reference, candidate):
    """Compare identical source frames, keeping missing frames visible."""
    def table(rows):
        result = {row["stamp_ns"]: (row["frame_id"], row["objc"], row["boxes"]) for row in rows}
        if len(result) != len(rows):
            raise ValueError("duplicate YOLO output stamps")
        return result
    a, b = table(reference), table(candidate)
    common = a.keys() & b.keys()
    return {"common_source_frames": len(common),
            "old_only_frames": len(a.keys() - b.keys()),
            "new_only_frames": len(b.keys() - a.keys()),
            "different_boxes_or_headers": [stamp for stamp in sorted(common) if a[stamp] != b[stamp]]}


def object_state(rows, event, stamp, track_id):
    states = []
    # Every YOLO result is uniquely consumed, so its stamp identifies the
    # historical LiDAR snapshot whose state was copied to a current track.
    fused_stamps = {row["yolo_stamp_ns"]: row["stamp_ns"] for row in rows
                    if row["event"] == "class_after"}
    for row in rows:
        if row["event"] != event or row["stamp_ns"] != stamp:
            continue
        obj = next((obj for obj in row["objects"] if obj["id"] == track_id), None)
        if obj is not None:
            historical = fused_stamps.get(row["yolo_stamp_ns"])
            states.append({"yolo_stamp_ns": row["yolo_stamp_ns"], "state": obj,
                           "fused_lidar_stamp_ns": historical,
                           "lidar_state_lag_ms": (stamp-historical)/1e6 if historical is not None else None})
    return states


def publication_state(rows, stamp, track_id):
    """State handed to the publisher, not a claim about publisher filtering."""
    frames = [row for row in rows if row['event'] == 'class_publish_input'
              and row['stamp_ns'] == stamp]
    if len(frames) > 1:
        raise ValueError('duplicate publication input stamp')
    if not frames:
        return None
    row = frames[0]
    obj = next((obj for obj in row['objects'] if obj['id'] == track_id), None)
    origin = next((origin for origin in row['origins'] if origin['id'] == track_id), None)
    if obj is None or origin is None:
        return None
    source = origin['source_lidar_stamp_ns']
    return {'state': obj, 'generation': row['generation'], 'origin': origin,
            'lidar_state_lag_ms': (stamp - source) / 1e6 if source else None}


def explain_outputs(reference, candidate, reference_trace, candidate_trace):
    """Join real geometric differences with historical and published states.

    This explains differences; the strict replay gate remains unchanged.
    """
    differences = compare_identity(reference["tracks"], candidate["tracks"])["classification_differences"]
    result = []
    for difference in differences:
        stamp = difference["stamp_ns"]
        details = dict(difference)
        for name, track, rows in (("old", difference["old"], reference_trace),
                                  ("new", difference["new"], candidate_trace)):
            details[name + "_trace"] = {
                "before": object_state(rows, "class_before", stamp, track[0]),
                "after": object_state(rows, "class_after", stamp, track[0]),
                "current_updates": object_state(rows, "class_current", stamp, track[0]),
                "publication_input": publication_state(rows, stamp, track[0]),
            }
        result.append(details)
    return result


def summarize(rows):
    counts = Counter(row["event"] for row in rows)
    finals = [row for row in rows if row["event"] == "fusion_summary"]
    decisions = [row for row in rows if row["event"] == "match_decision"]
    resolved = [row for row in decisions if row["action"] != "wait"]
    selected = [row for row in resolved if row["action"] == "match"]
    classes = [row for row in rows if row["event"] == "class_after"]
    ends = [row for row in rows if row["event"] == "lidar_process_end"]
    durations = sorted(row["duration_ms"] for row in ends)
    processed = [row['stamp_ns'] for row in ends if row['processed']]
    publications = [row for row in rows if row['event'] == 'class_publish_input']
    published = [row['stamp_ns'] for row in publications]
    ready = [row for row in rows if row['event'] == 'class_lidar_ready']
    lag = [(row['stamp_ns'] - origin['source_lidar_stamp_ns']) / 1e6
           for row in publications for origin in row['origins']
           if origin['source_lidar_stamp_ns']]
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
        "processed_lidar_stamps": processed,
        "duplicate_processed_stamps": len(processed) - len(set(processed)),
        "continuous_observation_complete": bool(publications)
        and len(ready) == len(processed) == len(publications)
        and len(set(published)) == len(published) and set(published) == set(processed),
        "publication_frames": len(publications),
        "timeout_drops": sum(row['action'] == 'drop' and row.get('waited_ns', 0) >= 200000000
                             for row in resolved),
        "startup_discarded_lidar": counts['lidar_startup_discard'] + counts['lidar_startup_overflow'],
        "publication_state_lag_ms": {
            "median": statistics.median(lag) if lag else None,
            "p95": sorted(lag)[min(len(lag)-1, int(.95*len(lag)))] if lag else None,
            "max": max(lag) if lag else None},
        "objects_without_update_in_published_frame": sum(not origin['updated_in_frame']
            for row in publications for origin in row['origins']),
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
    processed_a = {row['stamp_ns'] for row in reference
                   if row['event'] == 'lidar_process_end' and row['processed']}
    processed_b = {row['stamp_ns'] for row in candidate
                   if row['event'] == 'lidar_process_end' and row['processed']}
    return {"common_matched_lidar_stamps": len(common),
            "old_only_matched_stamps": sorted(a.keys() - b.keys()),
            "new_only_matched_stamps": sorted(b.keys() - a.keys()),
            "different_yolo_selections": changes,
            "old_only_processed_lidar_stamps": sorted(processed_a - processed_b),
            "new_only_processed_lidar_stamps": sorted(processed_b - processed_a),
            "processed_lidar_count_delta": len(processed_b) - len(processed_a),
            "matched_count_delta": len(b) - len(a)}


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
        frame_files = {name: args.directory / (name + ".frames.json") for name in traces}
        if all(path.exists() for path in frame_files.values()):
            frames = {name: json.loads(path.read_text()) for name, path in frame_files.items()}
            result["source_box_comparisons"] = {
                name: compare_boxes(frames["old_1"]["outputs"], frame["outputs"])
                for name, frame in frames.items() if name != "old_1"}
            result["output_classification_explanations"] = {
                name: explain_outputs(frames["old_1"], frames[name], traces["old_1"], rows)
                for name, rows in traces.items() if name != "old_1"}
    result["trace_integrity_pass"] = all(row["complete"] and row["accounting_consistent"]
                                        for row in result["runs"].values())
    (args.directory / "trace_analysis.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result["runs"], indent=2))
    return 0 if result["trace_integrity_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
