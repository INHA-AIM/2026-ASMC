#!/usr/bin/env python3
"""Audit boundary availability and temporal IDs across shadow result bags."""

from __future__ import annotations

import argparse
from collections import defaultdict
from datetime import datetime, timezone
import json
from pathlib import Path
import statistics

import yaml

from drivable_bev.bag_batch import (
    diagnostic_values,
    load_bag_selection,
    result_quality_gates,
    sha256_file,
)
from drivable_bev.shadow_result_audit import (
    robustness_delta,
    summarize_boundary_frames,
    summarize_boundary_run_groups,
)


BOUNDARY_TOPIC = "/perception/lane/boundaries"
RATE_TOPICS = {
    "model_grid": "/perception/camera/front/road_marking/model_grid/class_id",
    "fused_bev": "/perception/bev/camera/road_marking/class_id",
    "boundaries": BOUNDARY_TOPIC,
}


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--selection", required=True)
    parser.add_argument("--model-id", required=True)
    parser.add_argument("--rosbag-root", default="/data/rosbags")
    parser.add_argument("--output")
    parser.add_argument("--only-run-id", action="append", default=[])
    return parser.parse_args()


def utc_now():
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def write_yaml_atomic(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(value, stream, allow_unicode=True, sort_keys=False)
    temporary.replace(path)


def decode_boundary_frames(bag_paths):
    import rosbag

    frames = []
    for bag_path in bag_paths:
        with rosbag.Bag(str(bag_path), "r") as bag:
            for _, message, _ in bag.read_messages(topics=[BOUNDARY_TOPIC]):
                decoded = []
                for marker in message.markers:
                    if not marker.text:
                        continue
                    try:
                        value = json.loads(marker.text)
                    except (TypeError, ValueError):
                        continue
                    if value.get("status") == "provisional_shadow_boundary":
                        decoded.append(value)
                frames.append(decoded)
    return frames


def result_bag_paths(result_root: Path, metadata: dict):
    result = []
    for row in metadata.get("bags") or []:
        name = row.get("file") if isinstance(row, dict) else row
        if name:
            path = result_root / "bags" / str(name)
            if path.is_file():
                result.append(path)
    if not result:
        result = sorted((result_root / "bags").glob("*.bag"))
    if not result:
        raise ValueError("no closed result bag: {}".format(result_root))
    return result


def runtime_rates(metadata):
    statistics_by_topic = metadata.get("topic_statistics") or {}
    result = {}
    for name, topic in RATE_TOPICS.items():
        value = statistics_by_topic.get(topic) or {}
        result[name] = {
            "messages": int(value.get("messages", 0) or 0),
            "average_hz": float(value.get("average_hz", 0.0) or 0.0),
            "end_window_hz": float(value.get("end_window_hz", 0.0) or 0.0),
        }
    return result


def rate_distribution(rows, scenario):
    selected = [row for row in rows if row["scenario"] == scenario]
    return {
        name: {
            "min_hz": round(min(row["runtime_rates"][name]["average_hz"] for row in selected), 6),
            "median_hz": round(
                statistics.median(
                    row["runtime_rates"][name]["average_hz"] for row in selected
                ),
                6,
            ),
            "max_hz": round(max(row["runtime_rates"][name]["average_hz"] for row in selected), 6),
        }
        for name in RATE_TOPICS
    }


def main():
    args = parse_args()
    selection_path = Path(args.selection).expanduser().resolve()
    rosbag_root = Path(args.rosbag_root).expanduser().resolve()
    _, runs = load_bag_selection(selection_path, rosbag_root)
    requested_ids = set(args.only_run_id)
    if requested_ids:
        runs = tuple(item for item in runs if item.run_id in requested_ids)
        missing = requested_ids.difference(item.run_id for item in runs)
        if missing:
            raise SystemExit("requested run IDs are not selected: {}".format(sorted(missing)))
    output = (
        Path(args.output).expanduser().resolve()
        if args.output
        else selection_path.parent / (args.model_id + "_boundary_audit.yaml")
    )
    document = {
        "schema_version": "asmc-shadow-boundary-audit-1.0.0",
        "generated_at_utc": utc_now(),
        "selection": {"path": str(selection_path), "sha256": sha256_file(selection_path)},
        "model_id": args.model_id,
        "runs": [],
    }
    grouped_frames = defaultdict(list)
    technical_passed = True
    for item in runs:
        result_root = rosbag_root / "results" / item.run_id / args.model_id
        metadata_path = result_root / "metadata.yaml"
        if not metadata_path.is_file():
            raise ValueError("missing result metadata: {}".format(metadata_path))
        with metadata_path.open("r", encoding="utf-8") as stream:
            metadata = yaml.safe_load(stream) or {}
        gates = result_quality_gates(metadata)
        technical_passed = technical_passed and all(gates.values())
        frames = decode_boundary_frames(result_bag_paths(result_root, metadata))
        summary = summarize_boundary_frames(frames)
        grouped_frames[item.scenario].append(frames)
        grouped_frames[(item.scenario, item.weather)].append(frames)
        document["runs"].append(
            {
                "run_id": item.run_id,
                "scenario": item.scenario,
                "weather": item.weather,
                "sim_hour": item.sim_hour,
                "duration_s": item.duration_s,
                "technical_quality_gates": gates,
                "technical_quality_passed": all(gates.values()),
                "lane_diagnostics_final": diagnostic_values(
                    metadata, "drivable_bev/lane_vectorizer"
                ),
                "runtime_rates": runtime_rates(metadata),
                "boundary_summary": summary,
            }
        )

    scenarios = sorted({item.scenario for item in runs})
    scenario_summary = {}
    for scenario in scenarios:
        scenario_summary[scenario] = summarize_boundary_run_groups(
            grouped_frames[scenario]
        )
    document["scenario_summary"] = scenario_summary
    document["weather_summary"] = {
        scenario: {
            weather: summarize_boundary_run_groups(
                grouped_frames[(scenario, weather)]
            )
            for weather in sorted({item.weather for item in runs if item.scenario == scenario})
        }
        for scenario in scenarios
    }
    document["runtime_rate_summary"] = {
        scenario: rate_distribution(document["runs"], scenario)
        for scenario in scenarios
    }
    if {"shadow_clear", "shadow_static_avoid_v1"}.issubset(scenario_summary):
        document["static_minus_clear"] = robustness_delta(
            scenario_summary["shadow_clear"],
            scenario_summary["shadow_static_avoid_v1"],
        )
    document["technical_quality_passed"] = technical_passed
    document["interpretation"] = (
        "Availability and temporal-consistency audit only. It is not lane-geometry "
        "ground truth accuracy and does not assign home/opposing semantics."
    )
    write_yaml_atomic(output, document)
    print("shadow boundary audit: {}".format(output))
    return 0 if technical_passed else 3


if __name__ == "__main__":
    raise SystemExit(main())
