#!/usr/bin/env python3
"""Generate versioned MGeo-derived lane pseudo-GT and a top-down audit image."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import yaml

from asmc_perception.offline.bev.shadow_map_gt import build_shadow_lane_pseudo_gt, with_provenance


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--data-root", required=True)
    parser.add_argument("--output-dir", required=True)
    return parser.parse_args()


def _load(path):
    with Path(path).open("r", encoding="utf-8") as stream:
        return json.load(stream)


def _write_json_atomic(path: Path, value) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.write("\n")
    temporary.replace(path)


def _render(document, output_path: Path) -> None:
    curves = document["boundaries"]
    polygon = np.asarray(document["shadow_area"]["polygon_xy_m"], dtype=np.float64)
    values = [polygon]
    for points in curves.values():
        values.append(np.asarray(points, dtype=np.float64)[:, :2])
    all_points = np.vstack(values)
    lower = np.min(all_points, axis=0) - 3.0
    upper = np.max(all_points, axis=0) + 3.0
    canvas = np.full((520, 1120, 3), 24, dtype=np.uint8)

    def pixels(points):
        xy = np.asarray(points, dtype=np.float64).reshape(-1, 2)
        normalized = (xy - lower) / np.maximum(upper - lower, 1e-9)
        return np.column_stack(
            (
                30.0 + normalized[:, 0] * (canvas.shape[1] - 60.0),
                canvas.shape[0] - 30.0 - normalized[:, 1] * (canvas.shape[0] - 60.0),
            )
        ).round().astype(np.int32)

    cv2.polylines(canvas, [pixels(np.vstack((polygon, polygon[0])))], False, (100, 100, 100), 2)
    style = {
        "white_positive_normal": ((245, 245, 245), 3),
        "yellow_center": ((0, 220, 255), 4),
        "white_negative_normal": ((245, 245, 245), 3),
    }
    for name, points in curves.items():
        color, width = style[name]
        cv2.polylines(canvas, [pixels(np.asarray(points)[:, :2])], False, color, width, cv2.LINE_AA)
    text = "MAP PSEUDO-GT | NOT DEPLOYMENT GT | review: pending camera/LiDAR overlay"
    cv2.putText(canvas, text, (24, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 190, 80), 2, cv2.LINE_AA)
    cv2.imwrite(str(output_path), canvas)


def main():
    args = parse_args()
    config_path = Path(args.config).expanduser().resolve()
    data_root = Path(args.data_root).expanduser().resolve()
    output_dir = Path(args.output_dir).expanduser().resolve()
    with config_path.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream) or {}
    link_path = data_root / config["sources"]["link_set"]
    scenario_path = data_root / config["sources"]["scenario"]
    document = build_shadow_lane_pseudo_gt(
        _load(link_path), _load(scenario_path), config
    )
    document = with_provenance(document, config_path, link_path, scenario_path)
    output_dir.mkdir(parents=True, exist_ok=False)
    _write_json_atomic(output_dir / "shadow_lane_pseudo_gt.json", document)
    _render(document, output_dir / "shadow_lane_pseudo_gt_overlay.png")
    audit = {
        "schema_version": "asmc-shadow-lane-pseudo-gt-audit-1.0.0",
        "status": document["status"],
        "deployment_allowed": document["deployment_allowed"],
        "artifact_sha256": document["artifact_sha256"],
        "review_status": document["validation"]["review_status"],
        "sample_count": document["alignment"]["sample_count"],
        "nominal_lane_width_m": document["nominal_lane_width_m"],
        "center_separation_m": document["alignment"]["center_separation_m"],
        "lengths": document["lengths"],
    }
    _write_json_atomic(output_dir / "audit.json", audit)
    print("shadow pseudo-GT: {}".format(output_dir))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
