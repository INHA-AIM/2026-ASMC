#!/usr/bin/env python3
"""Aggregate relaxed, role-matched INVALID-to-map distance diagnostics."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np


def _sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _percentile(values, percentile):
    finite = np.asarray(
        [value for value in values if value is not None and np.isfinite(value)],
        dtype=np.float64,
    )
    return float(np.percentile(finite, percentile)) if len(finite) else None


def _load(input_root):
    runs = []
    overlays = []
    for frames_path in sorted(input_root.glob("*/frames.jsonl")):
        run_dir = frames_path.parent
        rows = [
            json.loads(line)
            for line in frames_path.read_text(encoding="utf-8").splitlines()
            if line.strip()
        ]
        metrics_path = run_dir / "metrics.json"
        metadata_path = run_dir / "metadata.json"
        runs.append(
            {
                "run_id": run_dir.name,
                "rows": rows,
                "frames_path": frames_path,
                "metrics": json.loads(metrics_path.read_text(encoding="utf-8")),
                "source": {
                    "frames": str(frames_path.resolve()),
                    "frames_sha256": _sha256(frames_path),
                    "metadata": str(metadata_path.resolve()),
                    "metadata_sha256": _sha256(metadata_path),
                },
            }
        )
        manifest_path = run_dir / "invalid_map_distance_review.json"
        if manifest_path.is_file():
            document = json.loads(manifest_path.read_text(encoding="utf-8"))
            for value in document.get("overlays", []):
                item = dict(value)
                item["source_overlay"] = str((run_dir / value["overlay"]).resolve())
                overlays.append(item)
    if not runs:
        raise ValueError("no offline audit frames found under {}".format(input_root))
    return runs, overlays


def _aggregate(rows, alignment):
    grouped = {}
    bands = {}
    for row in rows:
        value = ((row.get("invalid_map_distance") or {}).get(alignment) or {})
        for category, category_value in value.get("categories", {}).items():
            band = category_value.get("review_band", "unavailable")
            bands[(category, band)] = bands.get((category, band), 0) + 1
            for role, metric in category_value.get("roles", {}).items():
                key = (category, role)
                group = grouped.setdefault(
                    key,
                    {"p50": [], "p95": [], "signed": [], "points": 0},
                )
                group["p50"].append(metric.get("absolute_p50_m"))
                group["p95"].append(metric.get("absolute_p95_m"))
                group["signed"].append(metric.get("signed_normal_p50_m"))
                group["points"] += int(metric.get("point_count", 0))
    return grouped, bands


def _tile(entry, width=320, height=500):
    image = cv2.imread(entry["source_overlay"], cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError("cannot decode {}".format(entry["source_overlay"]))
    scale = min(float(width) / image.shape[1], float(height - 42) / image.shape[0])
    resized = cv2.resize(
        image,
        (max(1, round(image.shape[1] * scale)), max(1, round(image.shape[0] * scale))),
        interpolation=cv2.INTER_AREA,
    )
    tile = np.full((height, width, 3), 22, dtype=np.uint8)
    x0 = (width - resized.shape[1]) // 2
    tile[42 : 42 + resized.shape[0], x0 : x0 + resized.shape[1]] = resized
    label = "{} f{} {}".format(
        entry["run_id"].split("_kcity2025_")[-1],
        entry["frame_index"],
        entry["review_band"],
    )
    cv2.putText(tile, label[:52], (6, 17), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (240, 240, 240), 1, cv2.LINE_AA)
    cv2.putText(
        tile,
        "{} p95={:.3f}m".format(entry["selected_category"], entry["frame_absolute_p95_m"]),
        (6, 34),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.31,
        (190, 190, 190),
        1,
        cv2.LINE_AA,
    )
    return tile


def _contact_sheet(entries, path, columns):
    if not entries:
        return None
    tiles = [_tile(value) for value in entries]
    blank = np.full_like(tiles[0], 22)
    rows = []
    for index in range(0, len(tiles), columns):
        row = tiles[index : index + columns]
        row.extend([blank] * (columns - len(row)))
        rows.append(np.concatenate(row, axis=1))
    if not cv2.imwrite(str(path), np.concatenate(rows, axis=0), [cv2.IMWRITE_JPEG_QUALITY, 92]):
        raise OSError("failed to write {}".format(path))
    return path.name


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-root", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--max-overlays-per-band", type=int, default=8)
    parser.add_argument("--columns", type=int, default=4)
    arguments = parser.parse_args()
    arguments.output_dir.mkdir(parents=True, exist_ok=False)
    runs, overlays = _load(arguments.input_root.resolve())
    rows = [row for run in runs for row in run["rows"] if "error" not in row]

    alignment_modes = {}
    nearest_age = []
    bracket_span = []
    pose_translation = []
    pose_heading = []
    inside_odd = 0
    for row in rows:
        alignment = row.get("ego_pose_alignment") or {}
        mode = alignment.get("mode", "missing")
        alignment_modes[mode] = alignment_modes.get(mode, 0) + 1
        nearest_age.append(alignment.get("nearest_age_ms"))
        bracket_span.append(alignment.get("bracket_span_ms"))
        delta = alignment.get("delta_from_nearest") or {}
        pose_translation.append(delta.get("translation_m"))
        pose_heading.append(delta.get("heading_deg"))
        inside_odd += int(bool(row.get("within_map_odd", False)))

    aligned, aligned_bands = _aggregate(rows, "aligned")
    nearest, _ = _aggregate(rows, "nearest_reference")
    alignment_distance_deltas = []
    for row in rows:
        root = row.get("invalid_map_distance") or {}
        first = (root.get("aligned") or {}).get("categories", {})
        second = (root.get("nearest_reference") or {}).get("categories", {})
        for category in set(first) & set(second):
            aligned_p95 = first[category].get("frame_absolute_p95_m")
            nearest_p95 = second[category].get("frame_absolute_p95_m")
            if aligned_p95 is not None and nearest_p95 is not None:
                alignment_distance_deltas.append(float(aligned_p95) - float(nearest_p95))
    category_rows = []
    for key in sorted(set(aligned) | set(nearest)):
        category, role = key
        first = aligned.get(key, {})
        second = nearest.get(key, {})
        category_rows.append(
            {
                "category": category,
                "role": role,
                "aligned_frames": len(first.get("p95", [])),
                "aligned_points": first.get("points", 0),
                "aligned_abs_p50_frame_p50_m": _percentile(first.get("p50", []), 50),
                "aligned_abs_p95_frame_p50_m": _percentile(first.get("p95", []), 50),
                "aligned_abs_p95_frame_p95_m": _percentile(first.get("p95", []), 95),
                "aligned_signed_p50_frame_p50_m": _percentile(first.get("signed", []), 50),
                "nearest_abs_p95_frame_p50_m": _percentile(second.get("p95", []), 50),
                "nearest_abs_p95_frame_p95_m": _percentile(second.get("p95", []), 95),
            }
        )

    with (arguments.output_dir / "metrics_by_category_role.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(category_rows[0]) if category_rows else ["category"])
        writer.writeheader()
        writer.writerows(category_rows)

    run_rows = []
    for run in runs:
        metric = run["metrics"]
        align = metric.get("ego_pose_alignment") or {}
        invalid = metric.get("invalid_map_distance") or {}
        run_rows.append(
            {
                "run_id": run["run_id"],
                "frames": metric.get("frames"),
                "inside_pseudo_gt_odd_frames": sum(bool(row.get("within_map_odd")) for row in run["rows"]),
                "interpolated_frames": (align.get("mode_counts") or {}).get("interpolated", 0),
                "nearest_age_p95_ms": (align.get("nearest_age_ms") or {}).get("p95"),
                "bracket_span_p95_ms": (align.get("bracket_span_ms") or {}).get("p95"),
                "aligned_minus_nearest_abs_p95_m": ((invalid.get("aligned_minus_nearest_frame_p95_m") or {}).get("absolute_p95")),
            }
        )
    with (arguments.output_dir / "metrics_by_run.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(run_rows[0]))
        writer.writeheader()
        writer.writerows(run_rows)

    sheets = {}
    for category, band in sorted({(value["selected_category"], value["review_band"]) for value in overlays}):
        selected = sorted(
            [
            value for value in overlays
            if value["selected_category"] == category and value["review_band"] == band
            ],
            key=lambda value: -float(value["frame_absolute_p95_m"]),
        )[: arguments.max_overlays_per_band]
        name = "contact_sheet_{}_{}.jpg".format(category, band)
        sheets["{}/{}".format(category, band)] = _contact_sheet(
            selected, arguments.output_dir / name, arguments.columns
        )

    summary = {
        "schema_version": "asmc-invalid-map-distance-summary-1.0.0",
        "created_at_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "status": "representative_offline_measurement",
        "frames": len(rows),
        "inside_pseudo_gt_odd_frames": inside_odd,
        "ego_pose_alignment": {
            "mode_counts": alignment_modes,
            "nearest_age_ms": {"p50": _percentile(nearest_age, 50), "p95": _percentile(nearest_age, 95)},
            "bracket_span_ms": {"p50": _percentile(bracket_span, 50), "p95": _percentile(bracket_span, 95)},
            "delta_from_nearest": {
                "translation_m_p95": _percentile(pose_translation, 95),
                "heading_deg_p95": _percentile(pose_heading, 95),
            },
            "offline_map_evaluation_only": True,
            "runtime_ego_motion_compensation": False,
        },
        "review_band_frame_counts": {
            "{}/{}".format(category, band): count
            for (category, band), count in sorted(aligned_bands.items())
        },
        "aligned_minus_nearest_frame_p95_m": {
            "paired_category_frames": len(alignment_distance_deltas),
            "p50": _percentile(alignment_distance_deltas, 50),
            "p95": _percentile(alignment_distance_deltas, 95),
            "absolute_p95": _percentile(np.abs(alignment_distance_deltas), 95),
            "maximum_absolute": (
                float(np.max(np.abs(alignment_distance_deltas)))
                if alignment_distance_deltas else None
            ),
        },
        "error_bands_m": [0.35, 0.60],
        "bands_are_acceptance_gates": False,
        "interpretation": (
            "signed normal and absolute distances compare retained INVALID geometry "
            "with same-role map pseudo-GT; map, projection and asynchronous sensor "
            "uncertainty remain, so bands prioritize review and do not pass/fail frames"
        ),
        "invalid_geometry_planning_authority": False,
        "runs": [run["source"] for run in runs],
        "metrics_by_category_role": category_rows,
        "contact_sheets": sheets,
    }
    (arguments.output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )


if __name__ == "__main__":
    main()
