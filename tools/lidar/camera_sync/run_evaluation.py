#!/usr/bin/env python3
"""Read an immutable ROS bag and compare immediate vs 25 ms YOLO matching.

This evaluates scheduler causality, not an operational LiDAR pipeline. The
optional 130 ms offline sweep is a conditional geometry reference, never an
actual before/after result for the selected 25 ms camera frame.
"""
import argparse
from collections import Counter, defaultdict
import hashlib
import io
import json
from pathlib import Path
import statistics
import sys

from core import Event, MAX_WAIT_NS, evaluate


def digest_message(message):
    stream = io.BytesIO()
    message.serialize(stream)
    return hashlib.sha256(stream.getvalue()).hexdigest()


def digest_file(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while True:
            chunk = stream.read(4 * 1024 * 1024)
            if not chunk:
                return digest.hexdigest()
            digest.update(chunk)


def percentile(values, percent):
    if not values:
        return None
    ordered = sorted(values)
    index = max(0, (len(ordered) * percent + 99) // 100 - 1)
    return ordered[index]


def read_events(bag_path):
    import rosbag
    events, details = [], {}
    with rosbag.Bag(str(bag_path), "r") as bag:
        for source_index, (topic, message, arrival) in enumerate(
                bag.read_messages(topics=["/velodyne_points", "/YOLO/BBox"])):
            if not hasattr(message, "header"):
                raise ValueError("message without header: " + topic)
            if topic == "/velodyne_points":
                kind = "scan"
                details[source_index] = {
                    "point_count": message.width * message.height,
                    "point_step": message.point_step,
                    "frame_id": message.header.frame_id,
                }
            else:
                kind = "yolo"
                details[source_index] = {
                    "box_count": len(message.yolo_bbox_arr),
                    "frame_id": message.header.frame_id,
                }
            events.append(Event(kind=kind, arrival_ns=arrival.to_nsec(),
                                stamp_ns=message.header.stamp.to_nsec(),
                                source_index=source_index,
                                box_count=details[source_index].get("box_count", 0),
                                payload_digest=digest_message(message),
                                effective_stamp_ns=(message.header.stamp.to_nsec() // 1_000_000)
                                * 1_000_000 if kind == "scan" else None))
    return events, details


def scene_at(arrival_ns, scenes):
    t = arrival_ns / 1e9
    for scene in scenes:
        if scene["measurement_start_wall_s"] <= t <= scene["measurement_end_wall_s"]:
            return scene["name"]
    return "transition_or_unassigned"


def load_reference(path, scan_events):
    if path is None:
        return {}
    rows = {}
    with path.open() as stream:
        for index, line in enumerate(stream):
            frame = json.loads(line)
            if index >= len(scan_events):
                raise ValueError("offline reference has more scans than bag")
            stamp_ns = int(round(frame["stamp"] * 1e9))
            if abs(stamp_ns - scan_events[index].stamp_ns) > 1_000_000:
                raise ValueError("offline reference scan alignment failed at {}".format(index))
            sweep = frame.get("sweep", {})
            ref = {}
            for name in ("baseline", "camera_missing"):
                data = sweep.get(name)
                if data:
                    ref[name] = {"reference_plane_band_remaining_points": data["q"][0],
                                 "clusters": data["c"][5],
                                 "kept_points": data["c"][4],
                                 "processing_ms": data["q"][2]}
            ref["130ms_selected"] = frame.get("B", {}).get("protect")
            rows[scan_events[index].source_index] = ref
    if len(rows) != len(scan_events):
        raise ValueError("offline reference scan count mismatch")
    return rows


def summarise(rows, invalid):
    by_scene = defaultdict(list)
    for row in rows:
        by_scene[row["scene"]].append(row)
    def one(group):
        counts = Counter(row["candidate"]["reason"] for row in group)
        causes = Counter(row["candidate"]["unmatched_cause"] for row in group
                         if row["candidate"]["unmatched_cause"])
        recovered = [row for row in group
                     if row["baseline"]["reason"] not in ("matched_boxes", "matched_empty")
                     and row["candidate"]["reason"] in ("matched_boxes", "matched_empty")]
        waited = [row["candidate"]["wait_ns"] / 1e6 for row in group]
        reference_processing = [
            row["offline_130ms_reference"]["baseline"]["processing_ms"]
            for row in group if "baseline" in row.get("offline_130ms_reference", {})]
        potential = []
        for row in recovered:
            ref = row.get("offline_130ms_reference", {})
            if "baseline" in ref and "camera_missing" in ref:
                potential.append(
                    ref["camera_missing"]["reference_plane_band_remaining_points"] -
                    ref["baseline"]["reference_plane_band_remaining_points"])
        return {"scans": len(group), "candidate_reasons": dict(sorted(counts.items())),
                "candidate_unmatched_causes": dict(sorted(causes.items())),
                "offline_130ms_filter_processing_reference_ms": {
                    str(p): percentile(reference_processing, p) for p in (50, 95, 99)},
                "baseline_matches": sum(row["baseline"]["reason"].startswith("matched")
                                        for row in group),
                "candidate_matches": sum(row["candidate"]["reason"].startswith("matched")
                                         for row in group),
                "recovered_matches": len(recovered),
                "wait_ms": {str(p): percentile(waited, p) for p in (50, 95, 99)},
                "recovered_conditional_130ms_reference": {
                    "available_scans": len(potential),
                    "missing_minus_protected_plane_band_remaining_p50": percentile(potential, 50),
                    "scope": "not an actual 25ms geometry replay"}}
    return {"schema": "asmc-camera-sync-fifo-v2", "all": one(rows), "by_scene": {key: one(value)
            for key, value in sorted(by_scene.items())},
            "invalid_yolo": invalid,
            "future_yolo_use": sum(row[mode]["selected_yolo_arrival_ns"] is not None and
                                   row[mode]["selected_yolo_arrival_ns"] > row[mode]["decision_ns"]
                                   for row in rows for mode in ("baseline", "candidate"))}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--scene-manifest", type=Path, required=True)
    parser.add_argument("--reference-130ms", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not args.bag.is_file() or not args.scene_manifest.is_file():
        parser.error("bag and scene manifest must exist")
    if args.reference_130ms is not None and not args.reference_130ms.is_file():
        parser.error("offline reference must exist")
    if args.output.exists():
        parser.error("output already exists; never overwrite an evaluation")
    events, details = read_events(args.bag)
    scans = [e for e in events if e.kind == "scan"]
    baseline, invalid_base = evaluate(events, 0)
    candidate, invalid_trial = evaluate(events, MAX_WAIT_NS)
    off, _ = evaluate(events, MAX_WAIT_NS, protection_enabled=False)
    wait_sweep = {}
    for delay_ms in (0, 10, 25, 50, 130, 200):
        decisions, _ = evaluate(events, delay_ms * 1_000_000)
        wait_sweep[str(delay_ms)] = {
            "matched": sum(row["reason"].startswith("matched") for row in decisions),
            "queue_overflow": sum(row["reason"] == "queue_overflow" for row in decisions),
        }
    if invalid_base != invalid_trial or len(scans) != len(candidate):
        raise AssertionError("input parity failed")
    refs = load_reference(args.reference_130ms, scans)
    scenes = json.loads(args.scene_manifest.read_text())["scenes"]
    base_by_id = {row["source_index"]: row for row in baseline}
    off_by_id = {row["source_index"]: row for row in off}
    rows = []
    for scan, trial in zip(scans, candidate):
        row = {"input_id": "bag_scan_{}".format(scan.source_index),
               "scene": scene_at(scan.arrival_ns, scenes),
               "point_input": details[scan.source_index],
               "baseline": base_by_id[scan.source_index],
               "candidate": trial,
               "protection_off": off_by_id[scan.source_index]}
        if refs:
            row["offline_130ms_reference"] = refs[scan.source_index]
        rows.append(row)
    summary = summarise(rows, invalid_trial)
    summary["wait_sweep_ms"] = wait_sweep
    summary["arrival_minus_source_stamp_ms"] = {}
    for kind in ("scan", "yolo"):
        latencies = [(event.arrival_ns - event.stamp_ns) / 1e6 for event in events
                     if event.kind == kind and event.stamp_ns > 0]
        summary["arrival_minus_source_stamp_ms"][kind] = {
            str(p): percentile(latencies, p) for p in (50, 95, 99)}
    if summary["future_yolo_use"]:
        raise AssertionError("future data used")
    args.output.mkdir(parents=True)
    with (args.output / "decisions.jsonl").open("w") as stream:
        for row in rows:
            stream.write(json.dumps(row, sort_keys=True, separators=(",", ":")) + "\n")
    (args.output / "summary.json").write_text(
        json.dumps(summary, sort_keys=True, indent=2) + "\n")
    manifest = {
        "bag": str(args.bag.resolve()),
        "bag_bytes": args.bag.stat().st_size,
        "bag_sha256": digest_file(args.bag),
        "scene_manifest": str(args.scene_manifest.resolve()),
        "scene_manifest_sha256": digest_file(args.scene_manifest),
        "reference_130ms": str(args.reference_130ms.resolve()) if args.reference_130ms else None,
        "reference_130ms_sha256": digest_file(args.reference_130ms)
                                  if args.reference_130ms else None,
        "comparison": {"baseline_wait_ms": 0, "candidate_wait_ms": 25,
                       "stamp_window_ms": 50, "queue_max_scans": 2,
                       "scan_match_stamp": "legacy_integer_millisecond_floor",
                       "same_time_policy": "bag_source_order_then_deadline",
                       "history_policy": "positive_stamps_sorted_keep_64_within_2s"},
        "limitations": [
            "ROS bag arrival order is replayed; actual live internal state was not captured",
            "LiDAR point cloud and leveling are not recalculated",
            "130 ms offline geometry sweep is conditional reference, not 25 ms result",
            "No actual 25 ms wall residual or cluster change can be claimed",
            "Stationary scenes do not establish object detection or tracking accuracy",
        ],
    }
    (args.output / "manifest.json").write_text(
        json.dumps(manifest, sort_keys=True, indent=2) + "\n")
    print(json.dumps(summary["all"], sort_keys=True))


if __name__ == "__main__":
    main()
