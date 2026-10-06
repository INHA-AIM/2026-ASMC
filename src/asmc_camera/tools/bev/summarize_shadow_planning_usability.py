#!/usr/bin/env python3
"""Aggregate strict planning-consumer continuity from offline family replays."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

import numpy as np

from asmc_camera.bev.invalid_support import (
    DEFAULT_INVALID_SUPPORT_CONFIG,
    classify_invalid_support,
)


STRIPS = ("left_strip", "right_strip", "both_strips")
REPORT_COVERAGE_THRESHOLD = 0.80
INVALID_SUPPORT_FLAGS = (
    "vehicle_near_leading_tail",
    "preview_trailing_tail",
    "whole_support_shrink",
    "internal_support_gap",
    "preview_entry_missing",
)


def _percentile(values, percentile):
    finite = np.asarray(
        [value for value in values if value is not None and np.isfinite(value)],
        dtype=np.float64,
    )
    return float(np.percentile(finite, percentile)) if len(finite) else None


def _sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _longest_true_streak(values):
    current = longest = 0
    for value in values:
        current = current + 1 if bool(value) else 0
        longest = max(longest, current)
    return int(longest)


def _longest_failure_duration_sec(rows, failures):
    stamps = [int(row["stamp_ns"]) for row in rows]
    positive_steps = [
        (right - left) / 1e9
        for left, right in zip(stamps[:-1], stamps[1:])
        if right > left
    ]
    nominal_period = float(np.median(positive_steps)) if positive_steps else 0.0
    longest = 0.0
    start = None
    previous = None
    for stamp, failed in zip(stamps, failures):
        if failed:
            if start is None:
                start = stamp
            previous = stamp
        elif start is not None:
            longest = max(longest, (previous - start) / 1e9 + nominal_period)
            start = previous = None
    if start is not None:
        longest = max(longest, (previous - start) / 1e9 + nominal_period)
    return float(longest)


def _tags(run_id):
    scenarios = (
        "shadow_boundary_straddle",
        "shadow_lateral_sweep",
        "shadow_static_avoid_return",
        "shadow_static_avoid_v1",
        "shadow_clear",
    )
    scenario = next(
            (value for value in scenarios if "_{}_".format(value) in run_id),
            "unknown",
        )
    return {
        "cohort": (
            "baseline_matrix"
            if scenario in ("shadow_clear", "shadow_static_avoid_v1")
            else "maneuver"
            if scenario != "unknown"
            else "unknown"
        ),
        "scenario": scenario,
        "weather": next(
            (value for value in ("sunny", "foggy") if "_{}_".format(value) in run_id),
            "unknown",
        ),
        "sim_time": next(
            (value for value in ("11am", "1pm", "3pm") if "_{}_".format(value) in run_id),
            "unknown",
        ),
    }


def _load_run(directory):
    frames_path = directory / "frames.jsonl"
    metadata_path = directory / "metadata.json"
    rows = []
    for line in frames_path.open(encoding="utf-8"):
        row = json.loads(line)
        usability = (
            row.get("methods", {}).get("shared", {}).get("planning_usability")
        )
        if "error" not in row and usability is not None:
            rows.append(row)
    if not rows:
        raise ValueError("no planning-usability frames in {}".format(directory))
    return {
        "run_id": directory.name,
        "directory": directory,
        "frames_path": frames_path,
        "metadata_path": metadata_path,
        "rows": rows,
        "metadata": json.loads(metadata_path.read_text(encoding="utf-8")),
    }


def _frame_metric(row, zone, strip):
    return row["methods"]["shared"]["planning_usability"]["zones"][zone][strip]


def _frame_classification(row):
    shared = row["methods"]["shared"]
    stored = shared.get("invalid_support_classification")
    if stored is not None:
        return stored
    return classify_invalid_support(
        shared["planning_usability"], DEFAULT_INVALID_SUPPORT_CONFIG
    )


def _summarize_invalid_support(rows):
    values = [_frame_classification(row) for row in rows]
    denominator = max(len(values), 1)
    flags = {
        name: {
            "frames": int(sum(value["flags"][name] for value in values)),
            "rate": float(sum(value["flags"][name] for value in values))
            / denominator,
        }
        for name in INVALID_SUPPORT_FLAGS
    }
    primary = {}
    for value in values:
        label = value["primary_reason"]
        primary[label] = primary.get(label, 0) + 1
    measurement_names = (
        "control_leading_tail_m",
        "vehicle_near_leading_tail_m",
        "preview_trailing_tail_m",
        "control_internal_gap_m",
    )
    return {
        "frames": int(len(values)),
        "tail_metrics_explicit_rate": float(
            sum(value.get("tail_metrics_explicit", False) for value in values)
        )
        / denominator,
        "default_planning_use_allowed": False,
        "support_state_changed": False,
        "flags": flags,
        "primary_reason": {
            name: {"frames": int(count), "rate": float(count) / denominator}
            for name, count in sorted(primary.items())
        },
        "measurements_m": {
            name: {
                "p50": _percentile(
                    [value["measurements"][name] for value in values], 50
                ),
                "p95": _percentile(
                    [value["measurements"][name] for value in values], 95
                ),
                "max": max(
                    (value["measurements"][name] for value in values),
                    default=0.0,
                ),
            }
            for name in measurement_names
        },
    }


def _summarize_rows(rows, zone, strip):
    values = [_frame_metric(row, zone, strip) for row in rows]
    coverage = [value["coverage_ratio"] for value in values]
    observed = [value["observed_coverage_ratio"] for value in values]
    degraded = [value["degraded_coverage_ratio"] for value in values]
    horizon = [value["continuous_from_start_m"] for value in values]
    longest_gap = [value["longest_unusable_gap_m"] for value in values]
    internal_gap = [value["longest_internal_unusable_gap_m"] for value in values]
    below = [value < REPORT_COVERAGE_THRESHOLD for value in coverage]
    denominator = max(len(values), 1)
    return {
        "frames": int(len(values)),
        "family_available_rate": float(
            sum(
                row["methods"]["shared"]["planning_usability"]["available"]
                for row in rows
            )
        ) / denominator,
        "coverage_ratio": {
            "p05": _percentile(coverage, 5),
            "p50": _percentile(coverage, 50),
            "p95": _percentile(coverage, 95),
        },
        "observed_coverage_ratio": {
            "p05": _percentile(observed, 5),
            "p50": _percentile(observed, 50),
        },
        "degraded_coverage_ratio": {
            "p50": _percentile(degraded, 50),
            "p95": _percentile(degraded, 95),
        },
        "continuous_from_start_m": {
            "p05": _percentile(horizon, 5),
            "p50": _percentile(horizon, 50),
            "p95": _percentile(horizon, 95),
        },
        "longest_unusable_gap_m": {
            "p50": _percentile(longest_gap, 50),
            "p95": _percentile(longest_gap, 95),
        },
        "longest_internal_unusable_gap_m": {
            "p50": _percentile(internal_gap, 50),
            "p95": _percentile(internal_gap, 95),
        },
        "start_anchored_rate": float(
            sum(value["start_anchored"] for value in values)
        ) / denominator,
        "fully_continuous_rate": float(
            sum(value["fully_continuous"] for value in values)
        ) / denominator,
        "coverage_at_least_0_80_rate": float(
            sum(not value for value in below)
        ) / denominator,
        "coverage_at_least_0_50_rate": float(
            sum(value >= 0.50 for value in coverage)
        ) / denominator,
        "nonzero_coverage_rate": float(
            sum(value > 1e-9 for value in coverage)
        ) / denominator,
        "zero_coverage_rate": float(
            sum(value <= 1e-9 for value in coverage)
        ) / denominator,
        "longest_below_0_80_streak_frames": _longest_true_streak(below),
        "longest_below_0_80_streak_sec": _longest_failure_duration_sec(
            rows, below
        ),
    }


def _zones(run):
    first = run["rows"][0]["methods"]["shared"]["planning_usability"]["zones"]
    return tuple(first)


def _summarize_runs(runs, zone, strip):
    result = _summarize_rows(
        [row for run in runs for row in run["rows"]], zone, strip
    )
    per_run = [_summarize_rows(run["rows"], zone, strip) for run in runs]
    # Temporal streaks must reset at bag boundaries even if adjacent runs both
    # start/end below the reporting threshold.
    result["longest_below_0_80_streak_frames"] = max(
        value["longest_below_0_80_streak_frames"] for value in per_run
    )
    result["longest_below_0_80_streak_sec"] = max(
        value["longest_below_0_80_streak_sec"] for value in per_run
    )
    return result


def _aggregate(runs):
    zone_names = _zones(runs[0])
    for run in runs[1:]:
        if _zones(run) != zone_names:
            raise ValueError("planning-usability zones differ between runs")
    return {
        zone: {
            strip: _summarize_runs(runs, zone, strip) for strip in STRIPS
        }
        for zone in zone_names
    }


def _condition_breakdown(runs):
    result = {}
    for dimension in ("cohort", "scenario", "weather", "sim_time"):
        result[dimension] = {}
        labels = sorted({_tags(run["run_id"])[dimension] for run in runs})
        for label in labels:
            selected = [
                run
                for run in runs
                if _tags(run["run_id"])[dimension] == label
            ]
            result[dimension][label] = {
                "run_count": int(len(selected)),
                "frames": int(sum(len(run["rows"]) for run in selected)),
                "zones": _aggregate(selected),
                "invalid_support": _summarize_invalid_support(
                    [row for run in selected for row in run["rows"]]
                ),
            }
    return result


def _write_csv(runs, output_path):
    fields = [
        "run_id",
        "cohort",
        "scenario",
        "weather",
        "sim_time",
        "zone",
        "strip",
        "frames",
        "coverage_p05",
        "coverage_p50",
        "observed_coverage_p50",
        "degraded_coverage_p50",
        "continuous_from_start_p05_m",
        "continuous_from_start_p50_m",
        "longest_gap_p95_m",
        "longest_internal_gap_p95_m",
        "start_anchored_rate",
        "fully_continuous_rate",
        "coverage_at_least_0_80_rate",
        "coverage_at_least_0_50_rate",
        "nonzero_coverage_rate",
        "longest_below_0_80_streak_frames",
        "longest_below_0_80_streak_sec",
    ]
    with output_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for run in runs:
            tags = _tags(run["run_id"])
            for zone in _zones(run):
                for strip in STRIPS:
                    value = _summarize_rows(run["rows"], zone, strip)
                    writer.writerow(
                        {
                            "run_id": run["run_id"],
                            **tags,
                            "zone": zone,
                            "strip": strip,
                            "frames": value["frames"],
                            "coverage_p05": value["coverage_ratio"]["p05"],
                            "coverage_p50": value["coverage_ratio"]["p50"],
                            "observed_coverage_p50": value["observed_coverage_ratio"]["p50"],
                            "degraded_coverage_p50": value["degraded_coverage_ratio"]["p50"],
                            "continuous_from_start_p05_m": value["continuous_from_start_m"]["p05"],
                            "continuous_from_start_p50_m": value["continuous_from_start_m"]["p50"],
                            "longest_gap_p95_m": value["longest_unusable_gap_m"]["p95"],
                            "longest_internal_gap_p95_m": value["longest_internal_unusable_gap_m"]["p95"],
                            "start_anchored_rate": value["start_anchored_rate"],
                            "fully_continuous_rate": value["fully_continuous_rate"],
                            "coverage_at_least_0_80_rate": value["coverage_at_least_0_80_rate"],
                            "coverage_at_least_0_50_rate": value["coverage_at_least_0_50_rate"],
                            "nonzero_coverage_rate": value["nonzero_coverage_rate"],
                            "longest_below_0_80_streak_frames": value["longest_below_0_80_streak_frames"],
                            "longest_below_0_80_streak_sec": value["longest_below_0_80_streak_sec"],
                        }
                    )


def _write_invalid_support_csv(runs, output_path):
    fields = [
        "run_id",
        "cohort",
        "scenario",
        "weather",
        "sim_time",
        "frames",
        "tail_metrics_explicit_rate",
        *["{}_rate".format(name) for name in INVALID_SUPPORT_FLAGS],
        "control_leading_tail_p50_m",
        "control_leading_tail_p95_m",
        "vehicle_near_leading_tail_p95_m",
        "preview_trailing_tail_p50_m",
        "preview_trailing_tail_p95_m",
        "control_internal_gap_p95_m",
    ]
    with output_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for run in runs:
            value = _summarize_invalid_support(run["rows"])
            writer.writerow(
                {
                    "run_id": run["run_id"],
                    **_tags(run["run_id"]),
                    "frames": value["frames"],
                    "tail_metrics_explicit_rate": value[
                        "tail_metrics_explicit_rate"
                    ],
                    **{
                        "{}_rate".format(name): value["flags"][name]["rate"]
                        for name in INVALID_SUPPORT_FLAGS
                    },
                    "control_leading_tail_p50_m": value["measurements_m"][
                        "control_leading_tail_m"
                    ]["p50"],
                    "control_leading_tail_p95_m": value["measurements_m"][
                        "control_leading_tail_m"
                    ]["p95"],
                    "vehicle_near_leading_tail_p95_m": value["measurements_m"][
                        "vehicle_near_leading_tail_m"
                    ]["p95"],
                    "preview_trailing_tail_p50_m": value["measurements_m"][
                        "preview_trailing_tail_m"
                    ]["p50"],
                    "preview_trailing_tail_p95_m": value["measurements_m"][
                        "preview_trailing_tail_m"
                    ]["p95"],
                    "control_internal_gap_p95_m": value["measurements_m"][
                        "control_internal_gap_m"
                    ]["p95"],
                }
            )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-root", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--selection", action="append", default=[], type=Path)
    parser.add_argument("--checkpoint", type=Path)
    arguments = parser.parse_args()
    directories = sorted(
        path.parent for path in arguments.input_root.glob("*/frames.jsonl")
    )
    if not directories:
        raise SystemExit("no per-run frames found under {}".format(arguments.input_root))
    arguments.output_dir.mkdir(parents=True, exist_ok=False)
    runs = [_load_run(directory) for directory in directories]
    lineage = {"selections": []}
    for path in arguments.selection:
        resolved = path.expanduser().resolve()
        if not resolved.is_file():
            raise SystemExit("selection not found: {}".format(resolved))
        lineage["selections"].append(
            {"path": str(resolved), "sha256": _sha256(resolved)}
        )
    if arguments.checkpoint is not None:
        resolved = arguments.checkpoint.expanduser().resolve()
        if not resolved.is_file():
            raise SystemExit("checkpoint not found: {}".format(resolved))
        lineage["checkpoint"] = {
            "path": str(resolved),
            "sha256": _sha256(resolved),
        }
    summary = {
        "schema_version": "asmc-shadow-planning-usability-baseline-1.1.0",
        "created_at_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "status": "measurement_only_no_policy_change",
        "run_count": int(len(runs)),
        "valid_frames": int(sum(len(run["rows"]) for run in runs)),
        "lineage": lineage,
        "reporting_thresholds": {
            "coverage_ratio": REPORT_COVERAGE_THRESHOLD,
            "is_acceptance_gate": False,
        },
        "interpretation": {
            "usable_interval": "all pair-boundary endpoint states are non-invalid",
            "observed_interval": "all pair-boundary endpoint states are observed",
            "internal_gap": "unusable x-span bracketed by usable intervals in the same zone",
            "tail_gap": "unusable x-span touching either zone boundary",
            "geometry_repair_applied": False,
            "invalid_geometry_retained": (
                "coordinates remain available for explicit low-authority "
                "preview/fallback experiments, but are not perception evidence"
            ),
            "default_invalid_planning_use_allowed": False,
        },
        "overall": _aggregate(runs),
        "invalid_support": _summarize_invalid_support(
            [row for run in runs for row in run["rows"]]
        ),
        "condition_breakdown": _condition_breakdown(runs),
        "runs": {
            run["run_id"]: {
                "tags": _tags(run["run_id"]),
                "frames": len(run["rows"]),
                "zones": {
                    zone: {
                        strip: _summarize_rows(run["rows"], zone, strip)
                        for strip in STRIPS
                    }
                    for zone in _zones(run)
                },
                "invalid_support": _summarize_invalid_support(run["rows"]),
            }
            for run in runs
        },
    }
    summary_path = arguments.output_dir / "summary.json"
    summary_path.write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    csv_path = arguments.output_dir / "runs.csv"
    _write_csv(runs, csv_path)
    invalid_csv_path = arguments.output_dir / "invalid_support_runs.csv"
    _write_invalid_support_csv(runs, invalid_csv_path)
    metadata = {
        "schema_version": summary["schema_version"],
        "created_at_utc": summary["created_at_utc"],
        "source_root": str(arguments.input_root.resolve()),
        "lineage": lineage,
        "sources": [
            {
                "run_id": run["run_id"],
                "frames_path": str(run["frames_path"].resolve()),
                "frames_sha256": _sha256(run["frames_path"]),
                "metadata_path": str(run["metadata_path"].resolve()),
                "metadata_sha256": _sha256(run["metadata_path"]),
                "source_schema_version": run["metadata"].get("schema_version"),
            }
            for run in runs
        ],
        "artifacts": {
            "summary": summary_path.name,
            "runs": csv_path.name,
            "invalid_support_runs": invalid_csv_path.name,
        },
    }
    (arguments.output_dir / "metadata.json").write_text(
        json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(summary_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
