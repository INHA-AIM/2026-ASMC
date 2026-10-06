#!/usr/bin/env python3
"""Aggregate representative shared lane-family A/B replay artifacts."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np


METHODS = ("independent", "shared")
ROLES = (
    "left_outer_boundary",
    "yellow_centre_boundary",
    "right_outer_boundary",
)


def _percentile(values, percentile):
    finite = np.asarray([value for value in values if np.isfinite(value)], dtype=np.float64)
    return float(np.percentile(finite, percentile)) if len(finite) else None


def _sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_run(directory):
    rows = [json.loads(line) for line in (directory / "frames.jsonl").open()]
    return {
        "run_id": directory.name,
        "rows": [row for row in rows if "error" not in row],
        "metrics": json.loads((directory / "metrics.json").read_text()),
        "metadata": json.loads((directory / "metadata.json").read_text()),
    }


def _run_tags(run_id):
    """Return stable condition labels encoded by the canonical recorder run ID."""
    if "_shadow_clear_" in run_id:
        scenario = "shadow_clear"
        cohort = "baseline_matrix"
    elif "_shadow_static_avoid_v1_" in run_id:
        scenario = "shadow_static_avoid_v1"
        cohort = "baseline_matrix"
    elif "_shadow_boundary_straddle_" in run_id:
        scenario = "shadow_boundary_straddle"
        cohort = "maneuver"
    elif "_shadow_lateral_sweep_" in run_id:
        scenario = "shadow_lateral_sweep"
        cohort = "maneuver"
    elif "_shadow_static_avoid_return_" in run_id:
        scenario = "shadow_static_avoid_return"
        cohort = "maneuver"
    else:
        scenario = "unknown"
        cohort = "unknown"
    weather = next(
        (value for value in ("sunny", "foggy") if "_{}_".format(value) in run_id),
        "unknown",
    )
    sim_time = next(
        (value for value in ("11am", "1pm", "3pm") if "_{}_".format(value) in run_id),
        "unknown",
    )
    return {
        "cohort": cohort,
        "scenario": scenario,
        "weather": weather,
        "sim_time": sim_time,
    }


def _rates(rows, method):
    values = [row["methods"][method] for row in rows]
    denominator = max(len(values), 1)
    return {
        "frames": len(values),
        "availability_rate": sum(value["available"] for value in values) / denominator,
        "yellow_observed_rate": sum(value["yellow_observed"] for value in values) / denominator,
        "structure_complete_rate": sum(value["structure_complete"] for value in values) / denominator,
        "fully_observed_rate": sum(value["fully_observed"] for value in values) / denominator,
    }


def _aggregate_method(runs, method):
    all_rows = [row for run in runs for row in run["rows"]]
    odd_rows = [row for row in all_rows if row.get("within_map_odd", False)]
    residuals = [
        row["methods"][method].get("residual_p95_m")
        for row in all_rows
        if row["methods"][method].get("residual_p95_m") is not None
    ]
    map_p95 = []
    map_by_bin = {"0_10m": [], "10_15m": [], "15_20m": []}
    widths = {"left": [], "right": []}
    inlier_residuals = []
    outlier_rates = []
    inferred_fractions = []
    fully_observed_fractions = []
    valid_family_lengths = []
    pseudo_gt_disagreement_rates = []
    pseudo_gt_inferred_points = 0
    pseudo_gt_disagreement_points = 0
    jitter = {role: [] for role in ROLES}
    for run in runs:
        previous = {role: None for role in ROLES}
        for row in run["rows"]:
            value = row["methods"][method]
            if value.get("widths_m"):
                for side in widths:
                    widths[side].append(value["widths_m"][side])
            for source, target in (
                ("inlier_residual_p95_m", inlier_residuals),
                ("outlier_rate", outlier_rates),
                ("structure_inferred_fraction", inferred_fractions),
                ("locally_fully_observed_fraction", fully_observed_fractions),
                ("valid_family_length_m", valid_family_lengths),
            ):
                if value.get(source) is not None:
                    target.append(value[source])
            frame_roles = (value.get("map") or {}).get("roles", {})
            for role in ROLES:
                metric = frame_roles.get(role)
                if not metric or not metric.get("available", False):
                    previous[role] = None
                    continue
                if metric.get("map_distance_p95_m") is not None:
                    map_p95.append(metric["map_distance_p95_m"])
                disagreement = metric.get("inferred_pseudo_gt_disagreement") or {}
                if disagreement.get("above_threshold_rate") is not None:
                    pseudo_gt_disagreement_rates.append(
                        disagreement["above_threshold_rate"]
                    )
                    pseudo_gt_inferred_points += int(
                        disagreement.get("point_count", 0)
                    )
                    pseudo_gt_disagreement_points += int(
                        disagreement.get("above_threshold_count", 0)
                    )
                for label, item in metric.get("distance_bins", {}).items():
                    if label in map_by_bin and item.get("p95_m") is not None:
                        map_by_bin[label].append(item["p95_m"])
                error = metric.get("reference_y_error_m")
                if error is None:
                    previous[role] = None
                    continue
                if previous[role] is not None:
                    jitter[role].append(abs(float(error) - previous[role]))
                previous[role] = float(error)
    return {
        "all_frames": _rates(all_rows, method),
        "map_supported_odd": _rates(odd_rows, method),
        "residual_p95_frame_distribution_m": {
            "p50": _percentile(residuals, 50),
            "p95": _percentile(residuals, 95),
        },
        "map_p95_frame_distribution_m": {
            "p50": _percentile(map_p95, 50),
            "p95": _percentile(map_p95, 95),
        },
        "map_p95_by_distance_bin_m": {
            label: _percentile(values, 95) for label, values in map_by_bin.items()
        },
        "map_reference_y_jitter_p95_m": {
            role: _percentile(values, 95) for role, values in jitter.items()
        },
        "widths_m": {
            "{}_p50".format(side): _percentile(values, 50)
            for side, values in widths.items()
        },
        "support_audit": {
            "inlier_residual_p95_m": _percentile(inlier_residuals, 95),
            "outlier_rate_p50": _percentile(outlier_rates, 50),
            "outlier_rate_p95": _percentile(outlier_rates, 95),
            "structure_inferred_fraction_p50": _percentile(
                inferred_fractions, 50
            ),
            "structure_inferred_fraction_p95": _percentile(
                inferred_fractions, 95
            ),
            "locally_fully_observed_fraction_p50": _percentile(
                fully_observed_fractions, 50
            ),
            "valid_family_length_p50_m": _percentile(
                valid_family_lengths, 50
            ),
            "pseudo_gt_disagreement_rate_p95": _percentile(
                pseudo_gt_disagreement_rates, 95
            ),
            "pseudo_gt_inferred_point_count": pseudo_gt_inferred_points,
            "pseudo_gt_disagreement_point_count": pseudo_gt_disagreement_points,
            "pseudo_gt_disagreement_point_rate": (
                float(pseudo_gt_disagreement_points) / pseudo_gt_inferred_points
                if pseudo_gt_inferred_points
                else None
            ),
            "pseudo_gt_disagreement_is_review_signal_only": True,
        },
    }


def _paired_map_comparison(runs):
    paired = {method: [] for method in METHODS}
    paired_bins = {
        label: {method: [] for method in METHODS}
        for label in ("0_10m", "10_15m", "15_20m")
    }
    shared_only = []
    independent_only = []
    for run in runs:
        for row in run["rows"]:
            role_maps = {
                method: (row["methods"][method].get("map") or {}).get("roles", {})
                for method in METHODS
            }
            for role in set(role_maps["independent"]) | set(role_maps["shared"]):
                metrics = {method: role_maps[method].get(role) for method in METHODS}
                available = {
                    method: bool(
                        metrics[method]
                        and metrics[method].get("available", False)
                        and metrics[method].get("map_distance_p95_m") is not None
                    )
                    for method in METHODS
                }
                if all(available.values()):
                    for method in METHODS:
                        paired[method].append(
                            metrics[method]["map_distance_p95_m"]
                        )
                    for label in paired_bins:
                        bin_metrics = {
                            method: metrics[method].get("distance_bins", {}).get(label)
                            for method in METHODS
                        }
                        if all(
                            bin_metrics[method]
                            and bin_metrics[method].get("p95_m") is not None
                            for method in METHODS
                        ):
                            for method in METHODS:
                                paired_bins[label][method].append(
                                    bin_metrics[method]["p95_m"]
                                )
                elif available["shared"]:
                    shared_only.append(metrics["shared"]["map_distance_p95_m"])
                elif available["independent"]:
                    independent_only.append(
                        metrics["independent"]["map_distance_p95_m"]
                    )
    method_p95 = {
        method: _percentile(values, 95) for method, values in paired.items()
    }
    by_bin = {}
    for label, values in paired_bins.items():
        p95 = {
            method: _percentile(values[method], 95) for method in METHODS
        }
        by_bin[label] = {
            "paired_count": len(values["shared"]),
            "independent_p95_m": p95["independent"],
            "shared_p95_m": p95["shared"],
            "delta_m": (
                p95["shared"] - p95["independent"]
                if p95["shared"] is not None and p95["independent"] is not None
                else None
            ),
        }
    return {
        "paired_frame_role_count": len(paired["shared"]),
        "independent_p95_m": method_p95["independent"],
        "shared_p95_m": method_p95["shared"],
        "delta_m": (
            method_p95["shared"] - method_p95["independent"]
            if method_p95["shared"] is not None
            and method_p95["independent"] is not None
            else None
        ),
        "by_distance_bin": by_bin,
        "shared_only_frame_role_count": len(shared_only),
        "shared_only_p95_m": _percentile(shared_only, 95),
        "independent_only_frame_role_count": len(independent_only),
        "independent_only_p95_m": _percentile(independent_only, 95),
        "comparison_population": "same run/frame/role with map metrics from both methods",
    }


def _paired_map_gate(metric):
    deltas = [metric.get("delta_m")]
    deltas.extend(
        value.get("delta_m") for value in metric["by_distance_bin"].values()
    )
    finite = [value for value in deltas if value is not None]
    return bool(finite and max(finite) <= 0.05), (max(finite) if finite else None)


def _non_regression_comparison(methods, runs):
    independent = methods["independent"]
    shared = methods["shared"]
    paired_map = _paired_map_comparison(runs)
    run_comparisons = []
    for run in runs:
        metric = _paired_map_comparison([run])
        passed, worst_delta = _paired_map_gate(metric)
        run_comparisons.append(
            {
                "run_id": run["run_id"],
                "map_p95_delta_m": metric["delta_m"],
                "worst_map_or_bin_delta_m": worst_delta,
                "passed": passed,
            }
        )
    stratified_pass = all(value["passed"] for value in run_comparisons)
    run_deltas = [
        value["map_p95_delta_m"]
        for value in run_comparisons
        if value["map_p95_delta_m"] is not None
    ]
    passed = bool(
        shared["all_frames"]["availability_rate"]
        >= independent["all_frames"]["availability_rate"] - 0.01
        and shared["map_supported_odd"]["structure_complete_rate"]
        >= independent["map_supported_odd"]["structure_complete_rate"] - 0.01
        and stratified_pass
    )
    return {
        "frames": shared["all_frames"]["frames"],
        "structure_complete_delta": (
            shared["map_supported_odd"]["structure_complete_rate"]
            - independent["map_supported_odd"]["structure_complete_rate"]
        ),
        "map_p95_delta_m": paired_map["delta_m"],
        "median_run_map_p95_delta_m": _percentile(run_deltas, 50),
        "worst_run_map_or_bin_delta_m": max(
            value["worst_map_or_bin_delta_m"]
            for value in run_comparisons
            if value["worst_map_or_bin_delta_m"] is not None
        ),
        "stratified_run_map_gate_passed": stratified_pass,
        "per_run": run_comparisons,
        "paired_map": paired_map,
        "non_regression_passed": passed,
    }


def _condition_breakdown(runs):
    dimensions = {name: {} for name in ("cohort", "scenario", "weather", "sim_time")}
    for dimension in dimensions:
        labels = sorted({_run_tags(run["run_id"])[dimension] for run in runs})
        for label in labels:
            selected = [
                run
                for run in runs
                if _run_tags(run["run_id"])[dimension] == label
            ]
            methods = {
                method: _aggregate_method(selected, method) for method in METHODS
            }
            dimensions[dimension][label] = {
                "run_count": len(selected),
                "methods": methods,
                "comparison": _non_regression_comparison(methods, selected),
            }
    return dimensions


def _write_contact_sheet(runs, output_path):
    tiles = []
    for run in runs:
        images = sorted(run["directory"].glob("overlays/periodic/*.jpg"))
        if not images:
            continue
        selected = images[:: max(1, len(images) // 3)][:3]
        for path in selected:
            image = cv2.imread(str(path), cv2.IMREAD_COLOR)
            if image is None:
                continue
            image = cv2.resize(image, (240, 375), interpolation=cv2.INTER_AREA)
            cv2.putText(
                image,
                run["run_id"].split("_kcity2025_")[-1][:34],
                (5, 370),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.34,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
            tiles.append(image)
    if not tiles:
        return None
    columns = 3
    rows = []
    for start in range(0, len(tiles), columns):
        row = tiles[start : start + columns]
        row.extend([np.zeros_like(tiles[0])] * (columns - len(row)))
        rows.append(np.hstack(row))
    cv2.imwrite(str(output_path), np.vstack(rows))
    return output_path.name


def _write_high_residual_sheets(runs, output_dir):
    entries = []
    for run in runs:
        for path in sorted(
            run["directory"].glob("overlays/high_residual/*.jpg")
        ):
            entries.append((run["run_id"], path))
    if not entries:
        return []
    output_dir.mkdir(parents=True, exist_ok=True)
    names = []
    columns = 4
    page_size = 16
    for page_index, page_start in enumerate(range(0, len(entries), page_size), 1):
        tiles = []
        for run_id, path in entries[page_start : page_start + page_size]:
            image = cv2.imread(str(path), cv2.IMREAD_COLOR)
            if image is None:
                continue
            image = cv2.resize(image, (192, 300), interpolation=cv2.INTER_AREA)
            label = "{} {}".format(
                run_id.split("_kcity2025_")[-1][:19], path.stem
            )
            cv2.rectangle(image, (0, 279), (191, 299), (0, 0, 0), -1)
            cv2.putText(
                image,
                label,
                (3, 294),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.30,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
            tiles.append(image)
        if not tiles:
            continue
        rows = []
        for start in range(0, len(tiles), columns):
            row = tiles[start : start + columns]
            row.extend([np.zeros_like(tiles[0])] * (columns - len(row)))
            rows.append(np.hstack(row))
        name = "high_residual_contact_sheet_{:02d}.jpg".format(page_index)
        cv2.imwrite(str(output_dir / name), np.vstack(rows))
        names.append(name)
    return names


def _write_review_candidates(runs, output_path):
    candidates = []
    for run in runs:
        for row in run["rows"]:
            shared = row["methods"]["shared"]
            if not shared.get("available", False):
                continue
            disagreement = []
            for metric in (shared.get("map") or {}).get("roles", {}).values():
                review = metric.get("inferred_pseudo_gt_disagreement") or {}
                if review.get("above_threshold_rate") is not None:
                    disagreement.append(review["above_threshold_rate"])
            candidates.append(
                {
                    "run_id": run["run_id"],
                    "frame_index": row["frame_index"],
                    "stamp_ns": row["stamp_ns"],
                    "data_residual_p95_m": shared.get("residual_p95_m"),
                    "inlier_residual_p95_m": shared.get(
                        "inlier_residual_p95_m"
                    ),
                    "outlier_rate": shared.get("outlier_rate"),
                    "structure_inferred_fraction": shared.get(
                        "structure_inferred_fraction"
                    ),
                    "pseudo_gt_disagreement_rate_max": (
                        max(disagreement) if disagreement else None
                    ),
                    "pseudo_gt_is_review_only": True,
                }
            )
    candidates.sort(
        key=lambda item: (
            -(item["data_residual_p95_m"] or -1.0),
            item["run_id"],
            item["frame_index"],
        )
    )
    fields = [
        "run_id",
        "frame_index",
        "stamp_ns",
        "data_residual_p95_m",
        "inlier_residual_p95_m",
        "outlier_rate",
        "structure_inferred_fraction",
        "pseudo_gt_disagreement_rate_max",
        "pseudo_gt_is_review_only",
    ]
    with output_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(candidates[:100])
    return output_path.name


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-root", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    arguments = parser.parse_args()
    directories = sorted(
        path.parent for path in arguments.input_root.glob("*/metrics.json")
    )
    if not directories:
        raise SystemExit("no per-run metrics found under {}".format(arguments.input_root))
    arguments.output_dir.mkdir(parents=True, exist_ok=False)
    runs = []
    for directory in directories:
        run = _load_run(directory)
        run["directory"] = directory
        runs.append(run)
    methods = {method: _aggregate_method(runs, method) for method in METHODS}
    condition_breakdown = _condition_breakdown(runs)
    independent = methods["independent"]
    shared = methods["shared"]
    paired_map = _paired_map_comparison(runs)
    overall_comparison = _non_regression_comparison(methods, runs)
    processing_ms = [
        row["processing_ms"]
        for run in runs
        for row in run["rows"]
        if row.get("processing_ms") is not None
    ]
    independent_id_switches = {}
    for run in runs:
        for role, count in run["metrics"].get(
            "independent_role_id_switches", {}
        ).items():
            independent_id_switches[role] = independent_id_switches.get(role, 0) + int(count)
    completeness_gain = (
        shared["map_supported_odd"]["structure_complete_rate"]
        - independent["map_supported_odd"]["structure_complete_rate"]
    )
    unpaired_map_p95_delta = (
        shared["map_p95_frame_distribution_m"]["p95"]
        - independent["map_p95_frame_distribution_m"]["p95"]
    )
    map_p95_delta = overall_comparison["median_run_map_p95_delta_m"]
    quantitative_gate = bool(
        completeness_gain >= 0.05
        and shared["map_supported_odd"]["availability_rate"]
        >= independent["map_supported_odd"]["availability_rate"]
        and overall_comparison["stratified_run_map_gate_passed"]
    )
    condition_gate = all(
        item["comparison"]["non_regression_passed"]
        for groups in condition_breakdown.values()
        for item in groups.values()
    )
    failed_conditions = [
        "{}:{}".format(dimension, condition)
        for dimension, groups in condition_breakdown.items()
        for condition, item in groups.items()
        if not item["comparison"]["non_regression_passed"]
    ]
    decision_reasons = [
        "expanded quantitative comparison completed",
        "manual false-bridge review must be recorded separately before promotion",
        "temporal propagation and output-state contract are not implemented",
    ]
    if failed_conditions:
        decision_reasons.append(
            "condition non-regression failed: {}".format(", ".join(failed_conditions))
        )
    if (
        shared["residual_p95_frame_distribution_m"]["p95"]
        > independent["residual_p95_frame_distribution_m"]["p95"] * 1.5
    ):
        decision_reasons.append(
            "shared raw-evidence residual needs robust-support review before runtime use"
        )
    summary = {
        "schema_version": "asmc-shadow-lane-family-summary-1.3.0",
        "created_at_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "run_count": len(runs),
        "valid_frames": sum(len(run["rows"]) for run in runs),
        "methods": methods,
        "condition_breakdown": condition_breakdown,
        "paired_map_comparison": paired_map,
        "offline_processing_ms": {
            "p50": _percentile(processing_ms, 50),
            "p95": _percentile(processing_ms, 95),
            "includes_common_pipeline_and_both_methods": True,
        },
        "identity_audit": {
            "independent_temporal_role_id_switches": independent_id_switches,
            "shared_role_labels_are_structurally_fixed": True,
            "direct_id_switch_comparison_valid": False,
        },
        "comparison": {
            "map_odd_structure_complete_gain": completeness_gain,
            "map_p95_delta_m": map_p95_delta,
            "pooled_paired_map_p95_delta_m": paired_map["delta_m"],
            "unpaired_map_p95_delta_m": unpaired_map_p95_delta,
            "worst_run_map_or_bin_delta_m": overall_comparison[
                "worst_run_map_or_bin_delta_m"
            ],
            "map_gate_uses_stratified_paired_same_frame_role_population": True,
            "shared_residual_is_not_directly_comparable": True,
            "representative_quantitative_gate_passed": quantitative_gate,
            "all_condition_non_regression_passed": condition_gate,
            "expanded_16_run_gate_passed": bool(
                len(runs) >= 16 and quantitative_gate and condition_gate
            ),
        },
        "decision": {
            "status": (
                "promising_offline_candidate"
                if quantitative_gate and condition_gate
                else "offline_candidate_needs_rework"
            ),
            "runtime_promotion_allowed": False,
            "reasons": decision_reasons,
        },
    }
    (arguments.output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    with (arguments.output_dir / "runs.csv").open("w", newline="", encoding="utf-8") as stream:
        fields = [
            "run_id",
            "frames",
            "method",
            "availability_rate",
            "odd_structure_complete_rate",
            "yellow_observed_rate",
            "residual_p95_m",
            "map_p95_m",
        ]
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for run in runs:
            for method in METHODS:
                metric = run["metrics"]["methods"][method]
                writer.writerow(
                    {
                        "run_id": run["run_id"],
                        "frames": metric["frames"],
                        "method": method,
                        "availability_rate": metric["availability_rate"],
                        "odd_structure_complete_rate": metric["map_supported_odd"]["structure_complete_rate"],
                        "yellow_observed_rate": metric["yellow_observed_rate"],
                        "residual_p95_m": metric["residual_p95_frame_distribution_m"]["p95"],
                        "map_p95_m": metric["map_p95_frame_distribution_m"]["p95"],
                    }
                )
    with (arguments.output_dir / "conditions.csv").open(
        "w", newline="", encoding="utf-8"
    ) as stream:
        fields = [
            "dimension",
            "condition",
            "run_count",
            "frames",
            "structure_complete_delta",
            "map_p95_delta_m",
            "median_run_map_p95_delta_m",
            "worst_run_map_or_bin_delta_m",
            "stratified_run_map_gate_passed",
            "non_regression_passed",
        ]
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for dimension, groups in condition_breakdown.items():
            for condition, item in groups.items():
                writer.writerow(
                    {
                        "dimension": dimension,
                        "condition": condition,
                        "run_count": item["run_count"],
                        **{
                            field: item["comparison"].get(field)
                            for field in fields
                            if field not in ("dimension", "condition", "run_count")
                        },
                    }
                )
    contact_sheet = _write_contact_sheet(runs, arguments.output_dir / "contact_sheet.jpg")
    high_residual_sheets = _write_high_residual_sheets(
        runs, arguments.output_dir
    )
    review_candidates = _write_review_candidates(
        runs, arguments.output_dir / "review_candidates.csv"
    )
    metadata = {
        "input_root": str(arguments.input_root.resolve()),
        "inputs": [
            {
                "run_id": run["run_id"],
                "metrics_sha256": _sha256(run["directory"] / "metrics.json"),
                "metadata_sha256": _sha256(run["directory"] / "metadata.json"),
            }
            for run in runs
        ],
        "artifacts": [
            "summary.json",
            "runs.csv",
            "conditions.csv",
            review_candidates,
        ]
        + ([contact_sheet] if contact_sheet else []),
        "high_residual_contact_sheets": high_residual_sheets,
    }
    (arguments.output_dir / "metadata.json").write_text(
        json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary["comparison"], sort_keys=True))


if __name__ == "__main__":
    main()
