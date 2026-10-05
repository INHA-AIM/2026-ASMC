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

from asmc_perception.offline.bev.bag_batch import (
    diagnostic_values,
    load_bag_selection,
    result_quality_gates,
    sha256_file,
)
from asmc_perception.offline.bev.shadow_result_audit import (
    robustness_delta,
    summarize_boundary_frames,
    summarize_boundary_run_groups,
    typed_boundary_frame,
)
from asmc_perception.offline.bev.shared_runtime_audit import (
    summarize_shared_runtime_frames,
    summarize_shared_runtime_run_groups,
)
from asmc_perception.bev.pipeline_audit import (
    summarize_pipeline_frames,
    summarize_pipeline_run_groups,
)


TYPED_BOUNDARY_TOPIC = "/perception/lane/boundary_observations"
LEGACY_BOUNDARY_TOPIC = "/perception/lane/boundaries"
PIPELINE_AUDIT_TOPIC = "/perception/lane/pipeline_audit"
SHARED_FAMILY_DIAGNOSTICS_TOPIC = (
    "/perception/lane/debug/shared_family/diagnostics"
)
RATE_TOPICS = {
    "model_grid": "/perception/camera/front/road_marking/model_grid/class_id",
    "fused_bev": "/perception/bev/camera/road_marking/class_id",
    "boundaries": TYPED_BOUNDARY_TOPIC,
    "pipeline_audit": PIPELINE_AUDIT_TOPIC,
    "shared_family": SHARED_FAMILY_DIAGNOSTICS_TOPIC,
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


def _legacy_boundary_frame(message):
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
    return decoded


def decode_boundary_frames(bag_paths):
    import rosbag

    frames = []
    for bag_path in bag_paths:
        with rosbag.Bag(str(bag_path), "r") as bag:
            available = set(bag.get_type_and_topic_info().topics)
            topic = (
                TYPED_BOUNDARY_TOPIC
                if TYPED_BOUNDARY_TOPIC in available
                else LEGACY_BOUNDARY_TOPIC
            )
            decoder = (
                typed_boundary_frame
                if topic == TYPED_BOUNDARY_TOPIC
                else _legacy_boundary_frame
            )
            for _, message, _ in bag.read_messages(topics=[topic]):
                decoded = decoder(message)
                if decoded is not None:
                    frames.append(decoded)
    return frames


def decode_pipeline_frames(bag_paths):
    """Decode the frame-local audit payload recorded with each lane output."""
    import rosbag

    frames = []
    topic_available = False
    malformed_messages = 0
    for bag_path in bag_paths:
        with rosbag.Bag(str(bag_path), "r") as bag:
            available = set(bag.get_type_and_topic_info().topics)
            if PIPELINE_AUDIT_TOPIC not in available:
                continue
            topic_available = True
            for _, message, _ in bag.read_messages(topics=[PIPELINE_AUDIT_TOPIC]):
                status = next(
                    (
                        item
                        for item in message.status
                        if item.name == "drivable_bev/lane_pipeline_audit"
                    ),
                    None,
                )
                if status is None:
                    malformed_messages += 1
                    continue
                values = {item.key: item.value for item in status.values}
                try:
                    frame = json.loads(values["frame"])
                except (KeyError, TypeError, ValueError):
                    malformed_messages += 1
                    continue
                if not isinstance(frame, dict):
                    malformed_messages += 1
                    continue
                frames.append(frame)
    summary = summarize_pipeline_frames(frames)
    summary["topic_available"] = topic_available
    summary["malformed_messages"] = malformed_messages
    return frames, summary


def _json_value(values, key, fallback):
    try:
        return json.loads(values.get(key, ""))
    except (TypeError, ValueError):
        return fallback


def _float_value(values, key):
    try:
        return float(values[key])
    except (KeyError, TypeError, ValueError):
        return None


def decode_shared_family_frames(bag_paths):
    """Decode the debug-only state contract from structured diagnostics."""
    import rosbag

    frames = []
    topic_available = False
    malformed_messages = 0
    for bag_path in bag_paths:
        with rosbag.Bag(str(bag_path), "r") as bag:
            available = set(bag.get_type_and_topic_info().topics)
            if SHARED_FAMILY_DIAGNOSTICS_TOPIC not in available:
                continue
            topic_available = True
            for _, message, _ in bag.read_messages(
                topics=[SHARED_FAMILY_DIAGNOSTICS_TOPIC]
            ):
                status = next(
                    (
                        item
                        for item in message.status
                        if item.name == "drivable_bev/shared_lane_family_debug"
                    ),
                    None,
                )
                if status is None:
                    malformed_messages += 1
                    continue
                values = {item.key: item.value for item in status.values}
                accepted = str(values.get("accepted", "false")).lower() == "true"
                deployment_allowed = (
                    str(values.get("deployment_allowed", "false")).lower()
                    == "true"
                )
                boundary_support = _json_value(values, "boundary_support", {})
                widths = _json_value(values, "widths_m", {})
                valid_x = _json_value(values, "valid_x_m", [])
                observed_roles = _json_value(values, "observed_roles", [])
                inferred_roles = _json_value(values, "inferred_roles", [])
                if not isinstance(boundary_support, dict) or not isinstance(widths, dict):
                    malformed_messages += 1
                    continue
                frames.append(
                    {
                        "accepted": accepted,
                        "reason": str(values.get("reason", status.message)),
                        "acceptance_mode": str(
                            values.get(
                                "acceptance_mode",
                                "legacy_unspecified" if accepted else "rejected",
                            )
                        ),
                        "deployment_allowed": deployment_allowed,
                        "fit_ms": _float_value(values, "fit_ms"),
                        "observed_roles": observed_roles,
                        "inferred_roles": inferred_roles,
                        "widths_m": widths,
                        "data_residual_p95_m": _float_value(
                            values, "data_residual_p95_m"
                        ),
                        "inlier_residual_p95_m": _float_value(
                            values, "inlier_residual_p95_m"
                        ),
                        "outlier_rate": _float_value(values, "outlier_rate"),
                        "valid_x_m": valid_x,
                        "boundary_support": boundary_support,
                    }
                )
    summary = summarize_shared_runtime_frames(frames)
    summary["topic_available"] = topic_available
    summary["malformed_messages"] = malformed_messages
    return frames, summary


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
        if name == "boundaries" and not value:
            value = statistics_by_topic.get(LEGACY_BOUNDARY_TOPIC) or {}
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
    grouped_shared_frames = defaultdict(list)
    all_pipeline_frame_groups = []
    all_shared_frame_groups = []
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
        bag_paths = result_bag_paths(result_root, metadata)
        frames = decode_boundary_frames(bag_paths)
        summary = summarize_boundary_frames(frames)
        pipeline_frames, pipeline_summary = decode_pipeline_frames(bag_paths)
        shared_frames, shared_summary = decode_shared_family_frames(bag_paths)
        grouped_frames[item.scenario].append(frames)
        grouped_frames[(item.scenario, item.weather)].append(frames)
        grouped_shared_frames[item.scenario].append(shared_frames)
        grouped_shared_frames[(item.scenario, item.weather)].append(shared_frames)
        all_shared_frame_groups.append(shared_frames)
        all_pipeline_frame_groups.append(pipeline_frames)
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
                "pipeline_summary": pipeline_summary,
                "shared_family_summary": shared_summary,
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
    document["shared_runtime_scenario_summary"] = {
        scenario: summarize_shared_runtime_run_groups(
            grouped_shared_frames[scenario]
        )
        for scenario in scenarios
    }
    document["shared_runtime_summary"] = summarize_shared_runtime_run_groups(
        all_shared_frame_groups
    )
    document["pipeline_runtime_summary"] = summarize_pipeline_run_groups(
        all_pipeline_frame_groups
    )
    document["shared_runtime_weather_summary"] = {
        scenario: {
            weather: summarize_shared_runtime_run_groups(
                grouped_shared_frames[(scenario, weather)]
            )
            for weather in sorted(
                {item.weather for item in runs if item.scenario == scenario}
            )
        }
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
        "ground truth accuracy and does not assign home/opposing semantics. "
        "Observed samples are primary, structure-inferred samples are degraded, "
        "and invalid samples are forbidden for consumers."
    )
    write_yaml_atomic(output, document)
    print("shadow boundary audit: {}".format(output))
    return 0 if technical_passed else 3


if __name__ == "__main__":
    raise SystemExit(main())
