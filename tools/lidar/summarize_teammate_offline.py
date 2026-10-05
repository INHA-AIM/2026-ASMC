#!/usr/bin/env python3
"""Summarize teammate 6435af1 tunnel evaluator outputs without claiming GT scores.

Schema below is read from Main/tools/tunnel_filter_offline_eval.cpp. A is the
integrated runtime when tunnel filtering is enabled, not a legacy-only baseline.
"""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path

from audit_stationary_scenes import measurement_windows
from audit_tracking_bag import distribution


SCHEMA = {
    "B.counts": ["input_points", "ceiling_removed", "ground_removed", "wall_removed", "kept_points", "clusters", "protected_points"],
    "B.t": ["level_ms", "ceiling_ms", "ground_fit_ms", "ground_remove_ms", "wall_fit_ms", "wall_remove_ms",
            "cluster_ms", "obb_ms", "obb_filter_ms", "pipeline_ms", "protection_ms", "publish_ms", "frame_ms"],
    "sweep.c": ["input_points", "ceiling_removed", "ground_removed", "wall_removed", "kept_points", "clusters"],
    "sweep.g": ["ground_source", "ground_height_m", "ground_tilt_deg", "fallback"],
    "sweep.q": ["reference_plane_band_remaining_points", "above_ceiling_remaining_points", "process_call_ms"],
    "objects": ["cx", "cy", "cz", "length", "width", "z_min", "z_max", "z_min_road", "z_max_road", "yaw",
                "points", "protected_points", "protect_class", "is_protected", "ceiling_flag", "wall_flag",
                "ground_residual_flag", "removed_flag", "wall_attached_flag"],
}


def retained_objects(objects):
    if any(len(obj) != 19 for obj in objects):
        raise ValueError("objects schema differs from 6435af1; review the evaluator before summarizing")
    return sum(not obj[17] for obj in objects)


def summarize_rows(rows):
    active = [row for row in rows if row["B"]["active"]]
    result = {
        "frames": len(rows), "active_tunnel_frames": len(active),
        "gps_invalid_frames": sum(row["gps_invalid"] for row in rows),
        "gps_tunnel_latched_frames": sum(row["gps_tunnel_latched"] for row in rows),
        "input_bag_minus_header_ms": distribution([(row["rec"] - row["stamp"]) * 1000 for row in rows]),
        "integrated_runtime": {
            "lidar_process_ms": distribution([row["A"]["ms"][0] for row in rows]),
            "fusion_ms": distribution([row["A"]["ms"][1] for row in rows]),
            "retained_points": distribution([row["A"]["ransac"] for row in rows]),
            "map_output_objects": distribution([row["A"]["last_lidar"] for row in rows if row["A"]["last_lidar"] >= 0]),
            "local_output_objects": distribution([len(row["A"]["nogps"]) for row in rows]),
            "replica_state": dict(Counter(str(row["A"]["replica"]) for row in rows)),
        },
    }
    if not active:
        return result
    for row in active:
        if len(row["B"]["counts"]) != 7 or len(row["B"]["t"]) != 13:
            raise ValueError("B schema differs from 6435af1")
    result["tunnel"] = {
        "counts": {name: distribution([row["B"]["counts"][index] for row in active])
                   for index, name in enumerate(SCHEMA["B.counts"])},
        "timings": {name: distribution([row["B"]["t"][index] for row in active])
                    for index, name in enumerate(SCHEMA["B.t"])},
        "ground_height_m": distribution([row["B"]["ground"][6] for row in active if row["B"]["ground"][6] is not None]),
        "ground_sources": dict(Counter(str(row["B"]["ground"][0]) for row in active)),
        "protection_available": dict(Counter(str(row["B"]["protect"][0]).lower() for row in active)),
        "retained_objects": distribution([retained_objects(row["B"]["objs"]) for row in active]),
        "wall_ambiguous_skipped_points": distribution([sum(wall["amb_skip"] for wall in row["B"]["walls"]) for row in active]),
    }
    result["conditioned_on_protection_available"] = {}
    for available in (True, False):
        selected = [row for row in active if row["B"]["protect"][0] == available]
        result["conditioned_on_protection_available"][str(available).lower()] = {
            "frames": len(selected),
            "kept_points": distribution([row["B"]["counts"][4] for row in selected]),
            "retained_objects": distribution([retained_objects(row["B"]["objs"]) for row in selected]),
            "yolo_offset_ms": distribution([row["B"]["protect"][1] for row in selected if row["B"]["protect"][1] is not None]),
            "wall_ambiguous_skipped_points": distribution([sum(wall["amb_skip"] for wall in row["B"]["walls"]) for row in selected]),
        }
    result["sweeps"] = {}
    names = sorted({name for row in active for name in row.get("sweep", {})})
    for name in names:
        selected = [row["sweep"][name] for row in active if name in row.get("sweep", {})]
        if any(len(value["c"]) != 6 or len(value["g"]) != 4 or len(value["q"]) != 3 for value in selected):
            raise ValueError("sweep schema differs from 6435af1")
        metrics = {"frames": len(selected),
                   "retained_objects": distribution([retained_objects(value["objs"]) for value in selected])}
        for key, schema_key in (("c", "sweep.c"), ("g", "sweep.g"), ("q", "sweep.q")):
            for index, field in enumerate(SCHEMA[schema_key]):
                metrics[field] = distribution([value[key][index] for value in selected if value[key][index] is not None])
        paired = [(row["sweep"][name], row["sweep"]["baseline"]) for row in active
                  if name in row.get("sweep", {}) and "baseline" in row["sweep"]]
        metrics["paired_minus_baseline"] = {
            "frames": len(paired),
            "kept_points": distribution([left["c"][4] - right["c"][4] for left, right in paired]),
            "retained_objects": distribution([retained_objects(left["objs"]) - retained_objects(right["objs"])
                                                for left, right in paired]),
        }
        result["sweeps"][name] = metrics
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--frames", type=Path, required=True)
    parser.add_argument("--scene-manifest", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    if args.output.exists():
        parser.error("output exists; choose a new report path")
    completion = args.frames.parent / "summary.json"
    if not completion.is_file():
        parser.error("evaluator summary.json missing; wait for a completed run")
    summary = json.loads(completion.read_text())
    rows = [json.loads(line) for line in args.frames.read_text().splitlines() if line.strip()]
    if len(rows) != summary["frames"]:
        parser.error("frames.jsonl count disagrees with evaluator summary")
    manifest = json.loads(args.scene_manifest.read_text())
    windows = measurement_windows(manifest)
    scenes = {}
    for name, start_ns, end_ns in windows:
        selected = [row for row in rows if start_ns <= round(row["rec"] * 1e9) < end_ns]
        if not selected:
            parser.error("no frames in scene: " + name)
        scenes[name] = summarize_rows(selected)
    result = {
        "schema_version": "asmc-teammate-tunnel-offline-summary-1",
        "source_schema_commit": "6435af1",
        "frames_sha256": hashlib.sha256(args.frames.read_bytes()).hexdigest(),
        "frames_path": str(args.frames), "manifest_path": str(args.scene_manifest),
        "evaluator_summary": summary, "source_array_schema": SCHEMA,
        "scenes": scenes, "all_frames": summarize_rows(rows),
        "interpretation": [
            "A records the integrated runtime: with B active its final objects/points already use the tunnel filter.",
            "Legacy RANSAC replica mismatch during B-active frames is expected; it is not an A/B correctness failure.",
            "rec is original input bag receipt time, stamp is input header time. Scene windows use rec and exclude teleport/settling.",
            "The evaluator processes a scan after processing_delay_s of later bag sensor state; this differs from live callback scheduling.",
            "Sweep process_call_ms excludes ROS output, camera projection/protection, and the rest of tracking/fusion; compare only like-for-like.",
            "Sweeps have independent temporal state on the same input. Point count reduction alone does not demonstrate obstacle recall.",
            "Reference-plane-band residuals are geometric proxies, not annotated ground or wall precision/recall.",
            "Stationary actor-free fixtures reveal structural candidates, not ground-truth FP rates or tracking acceptance.",
        ],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x") as stream:
        json.dump(result, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
    print(json.dumps({"output": str(args.output), "frames": len(rows),
                      "scenes": {name: report["frames"] for name, report in scenes.items()}}, indent=2))


if __name__ == "__main__":
    main()
