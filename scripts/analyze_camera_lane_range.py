#!/usr/bin/env python3
"""Select a defensible camera-BEV display range from lane-vector audit data."""

from __future__ import annotations

import argparse
import csv
import json
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np


CLASSES = {"white": 1, "yellow": 2}
WEATHERS = ("sunny", "foggy")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--vector-audit", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--minimum-lane-support", type=int, default=100)
    parser.add_argument("--overlay-count", type=int, default=12)
    return parser.parse_args()


def unpack_mask(packed, shape):
    size = int(np.prod(shape))
    value = np.unpackbits(packed, axis=1, count=size, bitorder="little")
    return value.reshape(len(value), *shape).astype(bool, copy=False)


def record_points(document, prefix, index):
    offsets = document[prefix + "_offsets"]
    lower, upper = int(offsets[index]), int(offsets[index + 1])
    return (
        document[prefix + "_xy"][lower:upper],
        document[prefix + "_class_id"][lower:upper],
    )


def nearest_distances(source, target):
    source = np.asarray(source, dtype=np.float64).reshape(-1, 2)
    target = np.asarray(target, dtype=np.float64).reshape(-1, 2)
    if not len(source):
        return np.empty(0, dtype=np.float64)
    if not len(target):
        return np.full(len(source), np.inf, dtype=np.float64)
    result = np.empty(len(source), dtype=np.float64)
    for start in range(0, len(source), 2048):
        delta = source[start : start + 2048, None, :] - target[None, :, :]
        result[start : start + 2048] = np.sqrt(np.min(np.sum(delta * delta, axis=2), axis=1))
    return result


class LaneStats:
    def __init__(self):
        self.prediction = 0
        self.target = 0
        self.matched_prediction = 0
        self.matched_target = 0
        self.distances = []

    def add(self, prediction, target, tolerance=0.2):
        pred_distance = nearest_distances(prediction, target)
        target_distance = nearest_distances(target, prediction)
        self.prediction += len(pred_distance)
        self.target += len(target_distance)
        self.matched_prediction += int(np.count_nonzero(pred_distance <= tolerance))
        self.matched_target += int(np.count_nonzero(target_distance <= tolerance))
        finite = np.concatenate(
            [pred_distance[np.isfinite(pred_distance)], target_distance[np.isfinite(target_distance)]]
        )
        if len(finite):
            step = max(1, int(np.ceil(len(finite) / 4000)))
            self.distances.extend(finite[::step].astype(np.float32).tolist())

    def result(self):
        precision = self.matched_prediction / max(1, self.prediction)
        recall = self.matched_target / max(1, self.target)
        distances = np.asarray(self.distances, dtype=np.float64)
        return {
            "prediction_count": self.prediction,
            "target_count": self.target,
            "precision_0.2m": precision,
            "recall_0.2m": recall,
            "f1_0.2m": 2.0 * precision * recall / max(precision + recall, np.finfo(float).eps),
            "chamfer_p50_m": float(np.percentile(distances, 50)) if len(distances) else None,
            "chamfer_p95_m": float(np.percentile(distances, 95)) if len(distances) else None,
        }


class BinaryStats:
    def __init__(self):
        self.intersection = 0
        self.union = 0
        self.target = 0
        self.valid = 0

    def add(self, prediction, target, valid):
        pred = np.asarray(prediction, dtype=bool) & valid
        truth = np.asarray(target, dtype=bool) & valid
        self.intersection += int(np.count_nonzero(pred & truth))
        self.union += int(np.count_nonzero(pred | truth))
        self.target += int(np.count_nonzero(truth))
        self.valid += int(np.count_nonzero(valid))

    def result(self):
        return {
            "intersection": self.intersection,
            "union": self.union,
            "target_positive": self.target,
            "valid_pixels": self.valid,
            "iou": self.intersection / max(1, self.union),
        }


def select_points(points, classes, class_id, bounds, extra=None):
    x_min, x_max, y_half = bounds
    keep = (
        (classes == class_id)
        & (points[:, 0] >= x_min)
        & (points[:, 0] < x_max)
        & (np.abs(points[:, 1]) < y_half)
    )
    if extra is not None:
        keep &= extra(points)
    return points[keep]


def manual_review(path):
    if not path.is_file():
        return {"complete": False, "reviewed": 0, "false_bridge_rate": None}
    with path.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    accepted = {"0", "false", "no", "n"}
    rejected = {"1", "true", "yes", "y"}
    bridge_values = [str(row.get("false_bridge", "")).strip().lower() for row in rows]
    marking_values = [
        str(row.get("non_lane_marking_false_positive", "")).strip().lower()
        for row in rows
    ]
    valid_values = accepted | rejected
    complete = bool(bridge_values) and all(
        value in valid_values for value in bridge_values + marking_values
    )
    bridge_failures = sum(value in rejected for value in bridge_values)
    marking_failures = sum(value in rejected for value in marking_values)
    return {
        "complete": complete,
        "reviewed": len(bridge_values) if complete else min(
            sum(value in valid_values for value in bridge_values),
            sum(value in valid_values for value in marking_values),
        ),
        "total": len(bridge_values),
        "false_bridge_count": bridge_failures,
        "false_bridge_rate": bridge_failures / len(bridge_values)
        if complete and bridge_values
        else None,
        "non_lane_marking_false_positive_count": marking_failures,
        "non_lane_marking_false_positive_rate": marking_failures / len(marking_values)
        if complete and marking_values
        else None,
    }


def candidate_metrics(document, prediction_masks, target_masks, valid_masks, ground_x, ground_y, bounds):
    store = defaultdict(lambda: {"drivable": BinaryStats(), "lane": defaultdict(LaneStats), "near": defaultdict(LaneStats), "far": defaultdict(LaneStats)})
    runs = document["run_id"]
    weathers = document["weather"]
    x_min, x_max, y_half = bounds
    raster_roi = (ground_x >= x_min) & (ground_x < x_max) & (np.abs(ground_y) < y_half)
    for index in range(len(runs)):
        condition = str(weathers[index])
        value = store[condition]
        value["drivable"].add(
            prediction_masks[index], target_masks[index], valid_masks[index] & raster_roi
        )
        pred_xy, pred_class = record_points(document, "predicted", index)
        target_xy, target_class = record_points(document, "target", index)
        for name, class_id in CLASSES.items():
            pred = select_points(pred_xy, pred_class, class_id, bounds)
            truth = select_points(target_xy, target_class, class_id, bounds)
            value["lane"][name].add(pred, truth)
            near_pred = pred[pred[:, 0] <= 15.0]
            near_truth = truth[truth[:, 0] <= 15.0]
            value["near"][name].add(near_pred, near_truth)
            far_pred = pred[pred[:, 0] > 15.0]
            far_truth = truth[truth[:, 0] > 15.0]
            value["far"][name].add(far_pred, far_truth)
    return {
        condition: {
            "drivable": values["drivable"].result(),
            "lane": {name: item.result() for name, item in values["lane"].items()},
            "near": {name: item.result() for name, item in values["near"].items()},
            "far": {name: item.result() for name, item in values["far"].items()},
        }
        for condition, values in store.items()
    }


def pass_candidate(bounds, metrics, review, minimum_support):
    reasons = []
    x_min, x_max, y_half = bounds
    if y_half < 6.0:
        reasons.append("does not include the required ego+adjacent-lane lateral extent")
    for weather in WEATHERS:
        if weather not in metrics:
            reasons.append("missing {} support".format(weather))
            continue
        values = metrics[weather]
        if values["drivable"]["iou"] < 0.95:
            reasons.append("{} drivable IoU {:.4f}<0.95".format(weather, values["drivable"]["iou"]))
        for class_name in CLASSES:
            lane = values["lane"].get(class_name, {})
            if lane.get("target_count", 0) < minimum_support:
                reasons.append("{} {} GT support {}<{}".format(weather, class_name, lane.get("target_count", 0), minimum_support))
                continue
            if lane["f1_0.2m"] < 0.80:
                reasons.append("{} {} F1@0.2 {:.4f}<0.80".format(weather, class_name, lane["f1_0.2m"]))
            if lane["recall_0.2m"] < 0.75:
                reasons.append("{} {} recall {:.4f}<0.75".format(weather, class_name, lane["recall_0.2m"]))
            near = values["near"].get(class_name, {})
            if near.get("chamfer_p95_m") is None or near["chamfer_p95_m"] > 0.35:
                reasons.append("{} {} x<=15m Chamfer p95 failed".format(weather, class_name))
            if x_max > 15.0:
                far = values["far"].get(class_name, {})
                if far.get("target_count", 0) >= minimum_support and (
                    far.get("chamfer_p95_m") is None or far["chamfer_p95_m"] > 0.60
                ):
                    reasons.append("{} {} x>15m Chamfer p95 failed".format(weather, class_name))
    if not review["complete"]:
        reasons.append("60-frame false-bridge/non-lane-marking review is incomplete")
    elif review["false_bridge_rate"] > 0.02:
        reasons.append("manual false-bridge rate {:.2%}>2%".format(review["false_bridge_rate"]))
    elif review["non_lane_marking_false_positive_rate"] > 0.02:
        reasons.append(
            "manual non-lane-marking false-positive rate {:.2%}>2%".format(
                review["non_lane_marking_false_positive_rate"]
            )
        )
    return not reasons, reasons


def bin_metrics(document, prediction_masks, target_masks, valid_masks, ground_x, ground_y):
    rows = []
    regions = []
    for lower in np.arange(-5.0, 30.0, 5.0):
        regions.append(("longitudinal", lower, lower + 5.0, lambda x, y, a=lower: (x >= a) & (x < a + 5.0)))
    for lower in np.arange(0.0, 12.0, 2.0):
        regions.append(("lateral_abs", lower, lower + 2.0, lambda x, y, a=lower: (np.abs(y) >= a) & (np.abs(y) < a + 2.0)))
    for kind, lower, upper, selector in regions:
        for weather in WEATHERS:
            drivable = BinaryStats()
            lanes = {name: LaneStats() for name in CLASSES}
            raster_roi = selector(ground_x, ground_y)
            for index, condition in enumerate(document["weather"]):
                if str(condition) != weather:
                    continue
                drivable.add(prediction_masks[index], target_masks[index], valid_masks[index] & raster_roi)
                pred_xy, pred_class = record_points(document, "predicted", index)
                target_xy, target_class = record_points(document, "target", index)
                for name, class_id in CLASSES.items():
                    pred = pred_xy[(pred_class == class_id) & selector(pred_xy[:, 0], pred_xy[:, 1])]
                    truth = target_xy[(target_class == class_id) & selector(target_xy[:, 0], target_xy[:, 1])]
                    lanes[name].add(pred, truth)
            rows.append({"region_type": kind, "lower_m": lower, "upper_m": upper, "weather": weather, "class": "drivable", **drivable.result()})
            for name, value in lanes.items():
                rows.append({"region_type": kind, "lower_m": lower, "upper_m": upper, "weather": weather, "class": name, **value.result()})
    return rows


def render_overlays(output, document, prediction_masks, target_masks, ground_x, ground_y, count):
    overlays = output / "overlays"
    overlays.mkdir()
    shape = prediction_masks.shape[1:]
    for index in np.linspace(0, len(prediction_masks) - 1, min(count, len(prediction_masks)), dtype=int):
        canvas = np.zeros((*shape, 3), dtype=np.uint8)
        canvas[target_masks[index]] = (80, 80, 0)
        canvas[prediction_masks[index]] = (0, 120, 0)
        pred_xy, pred_class = record_points(document, "predicted", index)
        for point, class_id in zip(pred_xy, pred_class):
            row = int(round((30.0 - point[0]) / 0.1))
            column = int(round((12.0 - point[1]) / 0.1))
            if 0 <= row < shape[0] and 0 <= column < shape[1]:
                canvas[row, column] = (255, 255, 255) if class_id == 1 else (0, 255, 255)
        image = cv2.resize(canvas, (720, 1050), interpolation=cv2.INTER_NEAREST)
        cv2.imwrite(str(overlays / "{:03d}_{}_f{:06d}.png".format(index, str(document["run_id"][index]), int(document["frame_id"][index]))), image)


def main():
    args = parse_args()
    source = Path(args.vector_audit).expanduser().resolve()
    output = Path(args.output_dir).expanduser().resolve()
    working = output.with_name(output.name + ".part")
    if output.exists() or working.exists():
        raise RuntimeError("refusing to overwrite output or partial output: {}".format(output))
    metadata_path = source / "metadata.json"
    inputs_path = source / "range_inputs.npz"
    if not metadata_path.is_file() or not inputs_path.is_file():
        raise RuntimeError("vector audit is incomplete: {}".format(source))
    with metadata_path.open("r", encoding="utf-8") as stream:
        parent_metadata = json.load(stream)
    document = np.load(str(inputs_path), allow_pickle=False)
    shape = tuple(int(value) for value in document["raster_shape"])
    bounds = document["raster_bounds"].astype(float).tolist()
    if bounds != [-5.0, 30.0, -12.0, 12.0, 0.1]:
        raise RuntimeError("range input uses an unsupported audit grid: {}".format(bounds))
    prediction_masks = unpack_mask(document["drivable_prediction"], shape)
    target_masks = unpack_mask(document["drivable_target"], shape)
    valid_masks = unpack_mask(document["drivable_valid"], shape)
    rows, columns = np.indices(shape, dtype=np.float64)
    ground_x = 30.0 - rows * 0.1
    ground_y = 12.0 - columns * 0.1
    review = manual_review(source / "manual_review.csv")
    working.mkdir(parents=True)

    candidates = [
        (float(x_min), float(x_max), float(y_half))
        for x_min in (-5, -3, 0)
        for x_max in (10, 15, 20, 25, 30)
        for y_half in (4, 6, 8, 10, 12)
        if x_max > x_min
    ]
    candidate_rows = []
    passing = []
    detailed = {}
    for candidate in candidates:
        metrics = candidate_metrics(
            document, prediction_masks, target_masks, valid_masks, ground_x, ground_y, candidate
        )
        passed, reasons = pass_candidate(candidate, metrics, review, args.minimum_lane_support)
        x_min, x_max, y_half = candidate
        key = "x{:+g}_{:+g}_yabs{:g}".format(x_min, x_max, y_half)
        detailed[key] = metrics
        candidate_rows.append({
            "candidate": key,
            "x_min_m": x_min,
            "x_max_m": x_max,
            "y_half_m": y_half,
            "area_m2": (x_max - x_min) * 2.0 * y_half,
            "passed": passed,
            "reasons": " | ".join(reasons),
            **{
                "{}_drivable_iou".format(weather): metrics.get(weather, {}).get("drivable", {}).get("iou")
                for weather in WEATHERS
            },
            **{
                "{}_{}_f1_0.2m".format(weather, name): metrics.get(weather, {}).get("lane", {}).get(name, {}).get("f1_0.2m")
                for weather in WEATHERS for name in CLASSES
            },
        })
        if passed:
            passing.append(candidate)
    selected = max(passing, key=lambda item: ((item[1] - item[0]) * 2.0 * item[2], item[1], item[2])) if passing else None
    minimum = next((row for row in candidate_rows if row["x_min_m"] == 0 and row["x_max_m"] == 15 and row["y_half_m"] == 6), None)
    status = "validated" if selected is not None else ("pending_manual_review" if not review["complete"] else "provisional_failed")
    selected_document = {
        "schema_version": "asmc-camera-lane-range-selection-1.0.0",
        "status": status,
        "selected_range": None if selected is None else {
            "x_min_m": selected[0], "x_max_m": selected[1],
            "y_min_m": -selected[2], "y_max_m": selected[2], "resolution_m": 0.10,
        },
        "minimum_required_candidate": minimum,
        "manual_review": review,
        "rule": "largest passing area, then longer forward extent, then wider lateral extent",
        "config_update_allowed": selected is not None,
        "note": "No range is silently shrunk to claim success.",
    }
    with (working / "candidate_ranges.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(candidate_rows[0]))
        writer.writeheader()
        writer.writerows(candidate_rows)
    bins = bin_metrics(document, prediction_masks, target_masks, valid_masks, ground_x, ground_y)
    with (working / "metrics_by_bin.csv").open("w", encoding="utf-8", newline="") as stream:
        fieldnames = sorted(set().union(*(row.keys() for row in bins)))
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(bins)
    with (working / "selected_range.json").open("w", encoding="utf-8") as stream:
        json.dump(selected_document, stream, indent=2, ensure_ascii=False)
    with (working / "metadata.json").open("w", encoding="utf-8") as stream:
        json.dump({
            "schema_version": "asmc-camera-lane-range-metadata-1.0.0",
            "created_at": datetime.now(timezone.utc).isoformat(),
            "parent_vector_audit": str(source),
            "parent_checkpoint_sha256": parent_metadata.get("checkpoint_sha256"),
            "parent_calibration_sha256": parent_metadata.get("calibration_sha256"),
            "parent_lane_config_sha256": parent_metadata.get("lane_config_sha256"),
            "thresholds": {
                "drivable_iou": 0.95, "lane_f1_0.2m": 0.80,
                "lane_recall_0.2m": 0.75, "near_chamfer_p95_m": 0.35,
                "far_chamfer_p95_m": 0.60, "false_bridge_rate": 0.02,
                "non_lane_marking_false_positive_rate": 0.02,
                "minimum_lane_support": args.minimum_lane_support,
            },
            "candidate_details": detailed,
        }, stream, indent=2, ensure_ascii=False)
    render_overlays(working, document, prediction_masks, target_masks, ground_x, ground_y, args.overlay_count)
    working.rename(output)
    print("range status={} selected={} output={}".format(status, selected_document["selected_range"], output))


if __name__ == "__main__":
    main()
