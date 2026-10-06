#!/usr/bin/env python3
"""Audit native-label, letterbox, hard/soft, and fusion losses in camera BEV."""

from __future__ import annotations

import argparse
import csv
import hashlib
import heapq
import json
import os
import sys
import time
from collections import defaultdict
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np
import torch
import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "asmc_perception" / "src"))
sys.path.insert(
    0, str(REPO_ROOT / "src" / "asmc_perception" / "src")
)
sys.path.insert(0, str(REPO_ROOT / "src" / "asmc_perception" / "src"))

from asmc_perception.camera.geometry import (  # noqa: E402
    LetterboxGeometry,
    restore_categorical_probabilities,
    restore_class_ids,
    restore_probability,
)
from asmc_perception.bev.calibration import load_calibration_snapshot  # noqa: E402
from asmc_perception.bev.fusion import CameraBevFusion, build_quality_maps  # noqa: E402
from asmc_perception.bev.grid import BevGridSpec  # noqa: E402
from asmc_perception.offline.bev.offline_evaluation import (  # noqa: E402
    fuse_probabilities,
    project_model_grid_hard,
    project_model_grid_probabilities,
)
from asmc_perception.bev.projector import CameraBevProjector, ProjectedSemantic  # noqa: E402
from asmc_perception.training.twinlite import (  # noqa: E402
    MoraiTwinLiteDataset,
    adapt_twinlite_outputs,
    configure_twinlite_task,
    load_external_twinlite,
    task_from_checkpoint,
)


VIEWS = ("front", "left", "right")
MODES = (
    "native_reference",
    "cache_roundtrip",
    "model_grid_hard",
    "prediction_native_hard",
    "prediction_model_hard",
    "prediction_model_soft",
)
MARKING_CLASSES = {"white": 1, "yellow": 2, "stopline": 3}
TOLERANCES_M = (0.1, 0.2, 0.3)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-root", default=os.environ.get("ASMC_DATA", "/data"))
    parser.add_argument(
        "--dataset-version",
        default="twinlite_morai_v6_road_marking_actor_negative",
    )
    parser.add_argument("--split", default="val", choices=("train", "val", "test"))
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument(
        "--native-target-name",
        default="perception_targets_native_v3_actor_negative",
    )
    parser.add_argument("--twinlite-root", default=os.environ.get("TWINLITE_ROOT"))
    parser.add_argument("--config", default="medium")
    parser.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    parser.add_argument("--weights", default="auto", choices=("auto", "ema", "model"))
    parser.add_argument("--max-groups", type=int)
    parser.add_argument("--log-interval", type=int, default=20)
    parser.add_argument("--overlay-count", type=int, default=6)
    return parser.parse_args()


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def geometry_from_meta(value):
    return LetterboxGeometry(
        original_hw=tuple(int(v) for v in value["original_hw"]),
        target_hw=tuple(int(v) for v in value["target_hw"]),
        resized_hw=tuple(int(v) for v in value["resized_hw"]),
        pad_ltrb=tuple(int(v) for v in value["pad_ltrb"]),
        scale_xy=tuple(float(v) for v in value["scale_xy"]),
    )


def weather_from_run(run_id):
    if "_foggy_" in run_id:
        return "foggy"
    if "_sunny_" in run_id:
        return "sunny"
    return "unknown"


def read_jsonl(path):
    rows = []
    with Path(path).open("r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            if line.strip():
                try:
                    rows.append(json.loads(line))
                except json.JSONDecodeError as exc:
                    raise RuntimeError(
                        "invalid JSONL {}:{}: {}".format(path, line_number, exc)
                    ) from exc
    return rows


def read_mask(path, shape, maximum, label):
    value = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    if value is None:
        raise RuntimeError("failed to read {} mask: {}".format(label, path))
    if value.ndim != 2 or tuple(value.shape) != tuple(shape):
        raise RuntimeError(
            "{} mask shape mismatch: expected={} actual={} path={}".format(
                label, tuple(shape), value.shape, path
            )
        )
    if value.dtype != np.uint8 or (value.size and int(value.max()) > int(maximum)):
        raise RuntimeError(
            "{} mask has invalid dtype/value range: {} max={} path={}".format(
                label, value.dtype, int(value.max()), path
            )
        )
    return value


def load_native_targets(data_root, run_ids, native_target_name):
    index = {}
    manifests = {}
    policy_ids = set()
    policy_hashes = set()
    for run_id in sorted(run_ids):
        root = (
            data_root
            / "datasets"
            / run_id
            / "derived"
            / native_target_name
        )
        success = root / "_SUCCESS"
        manifest = root / "manifest.jsonl"
        summary = root / "qa_summary.json"
        if not success.is_file() or not manifest.is_file() or not summary.is_file():
            raise RuntimeError("native target is incomplete: {}".format(root))
        manifests[run_id] = {"path": str(manifest), "sha256": sha256(manifest)}
        with summary.open("r", encoding="utf-8") as stream:
            qa = json.load(stream)
        policy_ids.add(str(qa.get("policy_id")))
        policy_hashes.add(str(qa.get("policy_sha256")))
        for row in read_jsonl(manifest):
            key = (str(row["run_id"]), int(row["frame_id"]), str(row["view"]))
            if key in index:
                raise RuntimeError("duplicate native target: {}".format(key))
            outputs = {
                name: (root / relative).resolve()
                for name, relative in row["outputs"].items()
            }
            index[key] = {
                "native_hw": tuple(int(value) for value in row["native_hw"]),
                "outputs": outputs,
                "task_valid": dict(row.get("task_valid") or {}),
            }
    if len(policy_ids) != 1 or len(policy_hashes) != 1:
        raise RuntimeError(
            "validation runs do not share one native target policy: ids={} hashes={}".format(
                sorted(policy_ids), sorted(policy_hashes)
            )
        )
    return index, manifests, next(iter(policy_ids)), next(iter(policy_hashes))


@dataclass(frozen=True)
class EvalRaster:
    drivable: np.ndarray
    marking: np.ndarray
    coverage: np.ndarray
    marking_probabilities: np.ndarray = None
    marking_confidence: np.ndarray = None


def from_hard(value):
    return EvalRaster(
        drivable=value.drivable_probability.astype(np.float32) / 255.0,
        marking=value.road_marking_class_id,
        coverage=value.coverage > 0,
        marking_confidence=value.road_marking_confidence.astype(np.float32) / 255.0,
    )


def from_soft(value):
    return EvalRaster(
        drivable=value.drivable_probability,
        marking=value.road_marking_class_id,
        coverage=value.coverage > 0,
        marking_probabilities=value.road_marking_probabilities,
        marking_confidence=value.road_marking_confidence,
    )


def warp_native_valid(projector, native_valid):
    result = cv2.warpPerspective(
        np.asarray(native_valid, dtype=np.uint8),
        projector.homography.bev_from_image,
        (projector.grid.width_px, projector.grid.height_px),
        flags=cv2.INTER_NEAREST,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=0,
    )
    result[projector.homography.coverage == 0] = 0
    return np.ascontiguousarray(result, dtype=np.uint8)


class BinaryStats:
    def __init__(self):
        self.intersection = 0
        self.union = 0
        self.prediction_positive = 0
        self.target_positive = 0

    def add(self, prediction, target, valid):
        prediction = np.asarray(prediction, dtype=bool) & valid
        target = np.asarray(target, dtype=bool) & valid
        self.intersection += int(np.count_nonzero(prediction & target))
        self.union += int(np.count_nonzero(prediction | target))
        self.prediction_positive += int(np.count_nonzero(prediction))
        self.target_positive += int(np.count_nonzero(target))

    def result(self):
        return {
            "intersection": self.intersection,
            "union": self.union,
            "prediction_positive": self.prediction_positive,
            "target_positive": self.target_positive,
            "iou": self.intersection / max(1, self.union),
            "precision": self.intersection / max(1, self.prediction_positive),
            "recall": self.intersection / max(1, self.target_positive),
        }


class ToleranceStats:
    def __init__(self, tolerance_m, resolution_m):
        self.tolerance_m = float(tolerance_m)
        self.resolution_m = float(resolution_m)
        self.matched_prediction = 0
        self.prediction_positive = 0
        self.matched_target = 0
        self.target_positive = 0

    def add(self, prediction, target, valid):
        prediction = np.asarray(prediction, dtype=bool) & valid
        target = np.asarray(target, dtype=bool) & valid
        distance_to_target = distance_to_mask(target)
        distance_to_prediction = distance_to_mask(prediction)
        self.add_distances(
            prediction, target, valid, distance_to_target, distance_to_prediction
        )

    def add_distances(
        self,
        prediction,
        target,
        valid,
        distance_to_target,
        distance_to_prediction,
    ):
        prediction = np.asarray(prediction, dtype=bool) & valid
        target = np.asarray(target, dtype=bool) & valid
        self.prediction_positive += int(np.count_nonzero(prediction))
        self.target_positive += int(np.count_nonzero(target))
        if np.any(target):
            self.matched_prediction += int(
                np.count_nonzero(
                    prediction
                    & (distance_to_target * self.resolution_m <= self.tolerance_m)
                )
            )
        if np.any(prediction):
            self.matched_target += int(
                np.count_nonzero(
                    target
                    & (distance_to_prediction * self.resolution_m <= self.tolerance_m)
                )
            )

    def result(self):
        precision = self.matched_prediction / max(1, self.prediction_positive)
        recall = self.matched_target / max(1, self.target_positive)
        f1 = 2.0 * precision * recall / max(np.finfo(float).eps, precision + recall)
        return {
            "tolerance_m": self.tolerance_m,
            "matched_prediction": self.matched_prediction,
            "prediction_positive": self.prediction_positive,
            "matched_target": self.matched_target,
            "target_positive": self.target_positive,
            "precision": precision,
            "recall": recall,
            "f1": f1,
        }


class DistanceHistogram:
    def __init__(self, bin_m=0.01, max_m=40.0):
        self.bin_m = float(bin_m)
        self.max_m = float(max_m)
        self.counts = np.zeros(int(round(max_m / bin_m)) + 1, dtype=np.int64)
        self.count = 0
        self.sum_m = 0.0
        self.clipped = 0

    def add(self, values):
        values = np.asarray(values, dtype=np.float64)
        values = values[np.isfinite(values) & (values >= 0.0)]
        if not values.size:
            return
        self.count += int(values.size)
        self.sum_m += float(values.sum())
        clipped = values > self.max_m
        self.clipped += int(np.count_nonzero(clipped))
        indices = np.floor(np.minimum(values, self.max_m) / self.bin_m).astype(np.int64)
        self.counts += np.bincount(indices, minlength=len(self.counts))[: len(self.counts)]

    def percentile(self, percentile):
        if self.count == 0:
            return None
        rank = float(percentile) / 100.0 * max(0, self.count - 1)
        index = int(np.searchsorted(np.cumsum(self.counts), rank + 1, side="left"))
        return min(index * self.bin_m, self.max_m)

    def result(self):
        return {
            "count": self.count,
            "mean_m": self.sum_m / self.count if self.count else None,
            "p50_m": self.percentile(50.0),
            "p95_m": self.percentile(95.0),
            "clipped_above_max": self.clipped,
            "histogram_bin_m": self.bin_m,
            "histogram_max_m": self.max_m,
        }


class FragmentationStats:
    def __init__(self, short_extent_m=1.0):
        self.short_extent_m = float(short_extent_m)
        self.positive_pixels = 0
        self.fragment_pixels = 0
        self.components = 0
        self.fragment_components = 0

    def add(self, mask, valid, resolution_m):
        mask = (np.asarray(mask, dtype=bool) & valid).astype(np.uint8)
        self.positive_pixels += int(np.count_nonzero(mask))
        count, _, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
        for component in range(1, count):
            area = int(stats[component, cv2.CC_STAT_AREA])
            extent_px = max(
                int(stats[component, cv2.CC_STAT_WIDTH]),
                int(stats[component, cv2.CC_STAT_HEIGHT]),
            )
            self.components += 1
            if extent_px * float(resolution_m) < self.short_extent_m:
                self.fragment_components += 1
                self.fragment_pixels += area

    def result(self):
        return {
            "positive_pixels": self.positive_pixels,
            "components": self.components,
            "fragment_components": self.fragment_components,
            "fragment_pixels": self.fragment_pixels,
            "fragment_pixel_ratio": self.fragment_pixels / max(1, self.positive_pixels),
            "short_extent_m": self.short_extent_m,
        }


class StoplineStats:
    def __init__(self):
        self.gt_frames = 0
        self.predicted_frames = 0
        self.matched_frames = 0
        self.false_positive_frames = 0
        self.distance_error = DistanceHistogram()

    def add(self, prediction, target, valid, x):
        prediction = np.asarray(prediction, dtype=bool) & valid
        target = np.asarray(target, dtype=bool) & valid
        has_prediction = bool(np.any(prediction))
        has_target = bool(np.any(target))
        self.predicted_frames += int(has_prediction)
        self.gt_frames += int(has_target)
        if has_prediction and has_target:
            self.matched_frames += 1
            self.distance_error.add(
                [abs(float(np.median(x[prediction])) - float(np.median(x[target])))]
            )
        elif has_prediction:
            self.false_positive_frames += 1

    def result(self):
        return {
            "gt_frames": self.gt_frames,
            "predicted_frames": self.predicted_frames,
            "matched_frames": self.matched_frames,
            "missed_frames": self.gt_frames - self.matched_frames,
            "false_positive_frames": self.false_positive_frames,
            "recall": self.matched_frames / max(1, self.gt_frames),
            "distance_error": self.distance_error.result(),
        }


def symmetric_distances(prediction, target, valid, resolution_m):
    prediction = np.asarray(prediction, dtype=bool) & valid
    target = np.asarray(target, dtype=bool) & valid
    if not np.any(prediction) or not np.any(target):
        return np.empty(0, dtype=np.float32)
    to_target = distance_to_mask(target)
    to_prediction = distance_to_mask(prediction)
    return np.concatenate((to_target[prediction], to_prediction[target])).astype(
        np.float32
    ) * float(resolution_m)


def distance_to_mask(mask):
    """Pixel distance to true pixels without treating invalid space as a match."""
    mask = np.asarray(mask, dtype=bool)
    source = np.ones(mask.shape, dtype=np.uint8)
    source[mask] = 0
    return cv2.distanceTransform(source, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)


def lateral_distances(prediction, target, valid, resolution_m):
    prediction = np.asarray(prediction, dtype=bool) & valid
    target = np.asarray(target, dtype=bool) & valid
    values = []
    for row in range(prediction.shape[0]):
        pred_columns = np.flatnonzero(prediction[row])
        target_columns = np.flatnonzero(target[row])
        if not len(pred_columns) or not len(target_columns):
            continue
        positions = np.searchsorted(target_columns, pred_columns)
        right = target_columns[np.minimum(positions, len(target_columns) - 1)]
        left = target_columns[np.maximum(positions - 1, 0)]
        values.append(np.minimum(np.abs(pred_columns - left), np.abs(pred_columns - right)))
    if not values:
        return np.empty(0, dtype=np.float32)
    return np.concatenate(values).astype(np.float32) * float(resolution_m)


class MetricAccumulator:
    def __init__(self, resolution_m):
        self.resolution_m = float(resolution_m)
        self.frames = 0
        self.valid_pixels = 0
        self.covered_pixels = 0
        self.binary = {
            "drivable": BinaryStats(),
            **{name: BinaryStats() for name in MARKING_CLASSES},
        }
        self.tolerance = {
            name: {
                tolerance: ToleranceStats(tolerance, resolution_m)
                for tolerance in TOLERANCES_M
            }
            for name in MARKING_CLASSES
        }
        self.chamfer = {
            name: DistanceHistogram() for name in MARKING_CLASSES
        }
        self.lateral = {
            name: DistanceHistogram() for name in MARKING_CLASSES
        }
        self.fragmentation = {
            name: FragmentationStats() for name in MARKING_CLASSES
        }
        self.stopline = StoplineStats()

    def add(
        self,
        prediction,
        target,
        valid,
        x,
        add_frame_metrics,
        distance_cache,
    ):
        valid = np.asarray(valid, dtype=bool)
        self.frames += 1
        self.valid_pixels += int(np.count_nonzero(valid))
        self.covered_pixels += int(np.count_nonzero(valid & prediction.coverage))
        self.binary["drivable"].add(
            prediction.drivable >= 0.5, target.drivable >= 0.5, valid
        )
        for name, class_id in MARKING_CLASSES.items():
            pred_class = prediction.marking == class_id
            target_class = target.marking == class_id
            self.binary[name].add(pred_class, target_class, valid)
            for stats in self.tolerance[name].values():
                stats.add_distances(
                    pred_class,
                    target_class,
                    valid,
                    distance_cache[name]["to_target"],
                    distance_cache[name]["to_prediction"],
                )
            if add_frame_metrics:
                self.chamfer[name].add(
                    np.concatenate(
                        (
                            distance_cache[name]["to_target"][pred_class & valid],
                            distance_cache[name]["to_prediction"][target_class & valid],
                        )
                    )
                    * self.resolution_m
                )
                self.lateral[name].add(
                    lateral_distances(
                        pred_class, target_class, valid, self.resolution_m
                    )
                )
                self.fragmentation[name].add(
                    pred_class, valid, self.resolution_m
                )
        if add_frame_metrics:
            self.stopline.add(
                prediction.marking == 3, target.marking == 3, valid, x
            )

    def result(self):
        return {
            "frames": self.frames,
            "valid_pixels": self.valid_pixels,
            "covered_pixels": self.covered_pixels,
            "coverage_recall": self.covered_pixels / max(1, self.valid_pixels),
            "binary": {name: value.result() for name, value in self.binary.items()},
            "tolerance": {
                name: {
                    "{:.1f}m".format(tolerance): value.result()
                    for tolerance, value in values.items()
                }
                for name, values in self.tolerance.items()
            },
            "chamfer": {name: value.result() for name, value in self.chamfer.items()},
            "lateral": {name: value.result() for name, value in self.lateral.items()},
            "fragmentation": {
                name: value.result() for name, value in self.fragmentation.items()
            },
            "stopline_frames": self.stopline.result(),
        }


def metric_regions(grid):
    rows, columns = np.indices(grid.shape, dtype=np.float64)
    ground = grid.pixel_to_metric(
        np.column_stack([columns.reshape(-1), rows.reshape(-1)])
    ).reshape((*grid.shape, 2))
    x = ground[:, :, 0]
    regions = {"all": np.ones(grid.shape, dtype=bool)}
    for lower in np.arange(grid.x_min_m, grid.x_max_m, 5.0):
        upper = min(lower + 5.0, grid.x_max_m)
        regions["x_{:+03.0f}_{:+03.0f}m".format(lower, upper)] = (
            (x >= lower) & (x < upper)
        )
    return x, regions


def add_to_store(store, mode, weather, scope, prediction, target, x, regions, resolution):
    common_valid = target.coverage
    distance_cache = {}
    for name, class_id in MARKING_CLASSES.items():
        prediction_class = (prediction.marking == class_id) & common_valid
        target_class = (target.marking == class_id) & common_valid
        distance_cache[name] = {
            "to_target": distance_to_mask(target_class),
            "to_prediction": distance_to_mask(prediction_class),
        }
    for condition in ("all", weather):
        for region_name, region_mask in regions.items():
            key = (mode, condition, scope, region_name)
            if key not in store:
                store[key] = MetricAccumulator(resolution)
            valid = target.coverage & region_mask
            store[key].add(
                prediction,
                target,
                valid,
                x,
                add_frame_metrics=region_name == "all",
                distance_cache=distance_cache,
            )


def serialize_store(store):
    result = {}
    for (mode, condition, scope, region), accumulator in sorted(store.items()):
        result.setdefault(mode, {}).setdefault(condition, {}).setdefault(scope, {})[
            region
        ] = accumulator.result()
    return result


def write_metrics_csv(path, metrics):
    columns = (
        "mode",
        "weather",
        "scope",
        "region",
        "class",
        "iou",
        "precision",
        "recall",
        "f1_0.1m",
        "f1_0.2m",
        "f1_0.3m",
        "chamfer_p50_m",
        "chamfer_p95_m",
        "lateral_p50_m",
        "lateral_p95_m",
        "fragment_pixel_ratio",
        "valid_pixels",
        "target_positive",
        "coverage_recall",
    )
    with Path(path).open("x", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        for mode, by_weather in metrics.items():
            for condition, by_scope in by_weather.items():
                for scope, by_region in by_scope.items():
                    for region, values in by_region.items():
                        for class_name, binary in values["binary"].items():
                            tolerance = values["tolerance"].get(class_name, {})
                            chamfer = values["chamfer"].get(class_name, {})
                            lateral = values["lateral"].get(class_name, {})
                            fragmentation = values["fragmentation"].get(class_name, {})
                            writer.writerow(
                                {
                                    "mode": mode,
                                    "weather": condition,
                                    "scope": scope,
                                    "region": region,
                                    "class": class_name,
                                    "iou": binary["iou"],
                                    "precision": binary["precision"],
                                    "recall": binary["recall"],
                                    "f1_0.1m": tolerance.get("0.1m", {}).get("f1"),
                                    "f1_0.2m": tolerance.get("0.2m", {}).get("f1"),
                                    "f1_0.3m": tolerance.get("0.3m", {}).get("f1"),
                                    "chamfer_p50_m": chamfer.get("p50_m"),
                                    "chamfer_p95_m": chamfer.get("p95_m"),
                                    "lateral_p50_m": lateral.get("p50_m"),
                                    "lateral_p95_m": lateral.get("p95_m"),
                                    "fragment_pixel_ratio": fragmentation.get(
                                        "fragment_pixel_ratio"
                                    ),
                                    "valid_pixels": values["valid_pixels"],
                                    "target_positive": binary["target_positive"],
                                    "coverage_recall": values["coverage_recall"],
                                }
                            )


def colorize(raster):
    drivable = np.clip(raster.drivable, 0.0, 1.0)
    image = np.zeros((*drivable.shape, 3), dtype=np.uint8)
    image[:, :, 1] = np.rint(drivable * 150.0).astype(np.uint8)
    colors = {
        1: (255, 255, 255),
        2: (0, 255, 255),
        3: (0, 0, 255),
    }
    for class_id, color in colors.items():
        image[raster.marking == class_id] = color
    image[~raster.coverage] = 0
    return image


def render_overlay(run_id, frame_id, fused_by_mode):
    tiles = []
    for mode in MODES:
        tile = colorize(fused_by_mode[mode])
        tile = cv2.resize(tile, None, fx=2.0, fy=2.0, interpolation=cv2.INTER_NEAREST)
        cv2.rectangle(tile, (0, 0), (tile.shape[1] - 1, 27), (0, 0, 0), -1)
        cv2.putText(
            tile,
            mode,
            (6, 19),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
        tiles.append(tile)
    mosaic = np.hstack(tiles)
    title = "{} frame={}".format(run_id, frame_id)
    output = np.zeros((mosaic.shape[0] + 30, mosaic.shape[1], 3), dtype=np.uint8)
    output[30:] = mosaic
    cv2.putText(
        output,
        title,
        (6, 21),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    return output


def frame_f1(prediction, target, tolerance_m, resolution_m):
    stats = ToleranceStats(tolerance_m, resolution_m)
    valid = target.coverage
    for class_id in (1, 2):
        stats.add(prediction.marking == class_id, target.marking == class_id, valid)
    return stats.result()["f1"]


def metric_value(metrics, mode, weather, key_path):
    value = metrics[mode][weather]["fused"]["all"]
    for key in key_path:
        value = value[key]
    return value


def evaluate_soft_gate(metrics):
    baseline = "prediction_native_hard"
    candidate = "prediction_model_soft"
    reasons = []
    comparisons = {}
    for condition in ("sunny", "foggy"):
        if (
            condition not in metrics.get(baseline, {})
            or condition not in metrics.get(candidate, {})
        ):
            comparisons[condition] = {"evaluated": False}
            reasons.append("missing {} samples; full Sunny/Foggy gate was not evaluated".format(condition))
            continue
        condition_result = {}
        for class_name in ("white", "yellow"):
            old = metric_value(
                metrics,
                baseline,
                condition,
                ("tolerance", class_name, "0.2m", "f1"),
            )
            new = metric_value(
                metrics,
                candidate,
                condition,
                ("tolerance", class_name, "0.2m", "f1"),
            )
            condition_result[class_name + "_f1_0.2m_delta"] = new - old
            if new < old - 0.01:
                reasons.append(
                    "{} {} F1@0.2m regressed {:.4f}".format(
                        condition, class_name, new - old
                    )
                )
        old_drivable = metric_value(
            metrics, baseline, condition, ("binary", "drivable", "iou")
        )
        new_drivable = metric_value(
            metrics, candidate, condition, ("binary", "drivable", "iou")
        )
        condition_result["drivable_iou_delta"] = new_drivable - old_drivable
        if new_drivable < old_drivable - 0.003:
            reasons.append(
                "{} drivable IoU regressed {:.4f}".format(
                    condition, new_drivable - old_drivable
                )
            )
        old_stop = metric_value(
            metrics, baseline, condition, ("stopline_frames", "recall")
        )
        new_stop = metric_value(
            metrics, candidate, condition, ("stopline_frames", "recall")
        )
        condition_result["stopline_recall_delta"] = new_stop - old_stop
        if new_stop < old_stop - 0.02:
            reasons.append(
                "{} stopline recall regressed {:.4f}".format(
                    condition, new_stop - old_stop
                )
            )
        condition_result["evaluated"] = True
        comparisons[condition] = condition_result

    old_f1 = np.mean(
        [
            metric_value(metrics, baseline, "all", ("tolerance", name, "0.2m", "f1"))
            for name in ("white", "yellow")
        ]
    )
    new_f1 = np.mean(
        [
            metric_value(metrics, candidate, "all", ("tolerance", name, "0.2m", "f1"))
            for name in ("white", "yellow")
        ]
    )
    old_chamfer = np.mean(
        [
            metric_value(metrics, baseline, "all", ("chamfer", name, "p95_m"))
            for name in ("white", "yellow")
        ]
    )
    new_chamfer = np.mean(
        [
            metric_value(metrics, candidate, "all", ("chamfer", name, "p95_m"))
            for name in ("white", "yellow")
        ]
    )
    old_fragment = np.mean(
        [
            metric_value(
                metrics,
                baseline,
                "all",
                ("fragmentation", name, "fragment_pixel_ratio"),
            )
            for name in ("white", "yellow")
        ]
    )
    new_fragment = np.mean(
        [
            metric_value(
                metrics,
                candidate,
                "all",
                ("fragmentation", name, "fragment_pixel_ratio"),
            )
            for name in ("white", "yellow")
        ]
    )
    improvements = {
        "mean_white_yellow_f1_0.2m_delta": float(new_f1 - old_f1),
        "mean_white_yellow_chamfer_p95_reduction_m": float(old_chamfer - new_chamfer),
        "mean_white_yellow_fragment_ratio_relative_reduction": float(
            (old_fragment - new_fragment) / max(np.finfo(float).eps, old_fragment)
        ),
    }
    meaningfully_improved = (
        improvements["mean_white_yellow_f1_0.2m_delta"] >= 0.02
        or improvements["mean_white_yellow_chamfer_p95_reduction_m"] >= 0.05
        or improvements["mean_white_yellow_fragment_ratio_relative_reduction"] >= 0.10
    )
    if not meaningfully_improved:
        reasons.append("no F1, Chamfer, or fragmentation improvement met the adoption gate")
    return {
        "adopt_for_runtime_pilot": not reasons,
        "baseline": baseline,
        "candidate": candidate,
        "comparisons": comparisons,
        "improvements": improvements,
        "reasons": reasons,
    }


def load_grid(path):
    with Path(path).open("r", encoding="utf-8") as stream:
        values = yaml.safe_load(stream)
    grid = BevGridSpec.from_mapping(values)
    return grid, float(values["max_ground_range_m"]), float(values["min_camera_depth_m"])


def main():
    args = parse_args()
    if args.max_groups is not None and args.max_groups <= 0:
        raise ValueError("max-groups must be positive")
    if args.overlay_count < 0:
        raise ValueError("overlay-count cannot be negative")

    output_dir = Path(args.output_dir).expanduser().resolve()
    working_dir = output_dir.with_name(output_dir.name + ".part")
    if output_dir.exists() or working_dir.exists():
        raise RuntimeError("refusing to overwrite output or partial output: {}".format(output_dir))
    working_dir.mkdir(parents=True)
    overlays_dir = working_dir / "overlays"
    overlays_dir.mkdir()

    data_root = Path(args.data_root).expanduser().resolve()
    dataset = MoraiTwinLiteDataset(
        data_root=data_root,
        dataset_version=args.dataset_version,
        split=args.split,
    )
    groups = defaultdict(dict)
    for index, row in enumerate(dataset.rows):
        groups[(str(row["run_id"]), int(row["frame_id"]))][str(row["view"])] = index
    complete = [
        (key, values)
        for key, values in sorted(groups.items())
        if set(values) == set(VIEWS)
    ]
    incomplete = len(groups) - len(complete)
    if args.max_groups is not None:
        complete = complete[: args.max_groups]
    if not complete:
        raise RuntimeError("validation manifest has no complete 3-view groups")
    weather_values = {weather_from_run(run_id) for (run_id, _), _ in complete}
    unexpected_weather = weather_values - {"sunny", "foggy"}
    if unexpected_weather:
        raise RuntimeError(
            "accuracy audit only accepts sunny/foggy validation runs: {}".format(
                sorted(unexpected_weather)
            )
        )

    run_ids = {run_id for (run_id, _), _ in complete}
    native_index, native_manifests, policy_id, policy_sha256 = load_native_targets(
        data_root, run_ids, args.native_target_name
    )

    checkpoint_path = Path(args.checkpoint).expanduser().resolve()
    checkpoint = torch.load(str(checkpoint_path), map_location="cpu")
    task = task_from_checkpoint(checkpoint)
    if task.secondary_head != "road_marking":
        raise RuntimeError("loss-budget analysis requires a 4-class road-marking checkpoint")
    model, upstream_commit = load_external_twinlite(
        config=args.config, root=args.twinlite_root
    )
    configure_twinlite_task(model, task)
    if checkpoint.get("upstream_commit") != upstream_commit:
        raise RuntimeError("checkpoint and TwinLite source commits differ")
    use_ema = args.weights == "ema" or (
        args.weights == "auto" and checkpoint.get("ema_state_dict") is not None
    )
    state_key = "ema_state_dict" if use_ema else "model_state_dict"
    if checkpoint.get(state_key) is None:
        raise RuntimeError("checkpoint does not contain {}".format(state_key))
    model.load_state_dict(checkpoint[state_key])
    device = torch.device(args.device)
    model = model.to(device).eval()

    calibration_path = REPO_ROOT / "src/asmc_perception/config/bev/cameras_v2.yaml"
    grid_path = REPO_ROOT / "src/asmc_perception/config/bev/bev_grid.yaml"
    _, cameras = load_calibration_snapshot(calibration_path)
    grid, max_ground_range, min_depth = load_grid(grid_path)
    projectors = {
        view: CameraBevProjector(
            cameras[view], grid, max_ground_range, min_depth
        )
        for view in VIEWS
    }
    quality = build_quality_maps(projectors, grid)
    hard_fusion = CameraBevFusion(quality, {"front": 1, "left": 2, "right": 3})
    x, regions = metric_regions(grid)
    store = {}
    representative_runs = set()
    worst_overlays = []
    started = time.perf_counter()

    with torch.no_grad():
        for group_index, ((run_id, frame_id), indices) in enumerate(complete, 1):
            samples = {view: dataset[indices[view]] for view in VIEWS}
            batch = torch.stack([samples[view]["image"] for view in VIEWS]).to(device)
            with torch.cuda.amp.autocast(enabled=device.type == "cuda"):
                logits = adapt_twinlite_outputs(model(batch), task.secondary_head)
                pred_drivable = torch.softmax(logits["drivable"].float(), dim=1)[:, 1]
                pred_marking = torch.softmax(logits["road_marking"].float(), dim=1)
            pred_drivable = pred_drivable.cpu().numpy()
            pred_marking = pred_marking.cpu().numpy()

            per_mode_view = {mode: {} for mode in MODES}
            hard_valid_by_mode = {
                mode: {} for mode in MODES if mode != "prediction_model_soft"
            }
            soft_projected = {}
            for batch_index, view in enumerate(VIEWS):
                sample = samples[view]
                row = dataset.rows[indices[view]]
                geometry = geometry_from_meta(sample["meta"]["geometry"])
                native_key = (run_id, frame_id, view)
                if native_key not in native_index:
                    raise RuntimeError("missing native target: {}".format(native_key))
                native = native_index[native_key]
                if tuple(native["native_hw"]) != tuple(geometry.original_hw):
                    raise RuntimeError("native and cache geometry differ: {}".format(native_key))
                native_drivable = read_mask(
                    native["outputs"]["drivable"],
                    geometry.original_hw,
                    1,
                    "native drivable",
                )
                native_marking = read_mask(
                    native["outputs"]["road_marking"],
                    geometry.original_hw,
                    3,
                    "native road marking",
                )
                native_ignore = read_mask(
                    native["outputs"]["drivable_ignore"],
                    geometry.original_hw,
                    1,
                    "native drivable ignore",
                )
                if not native["task_valid"].get("road_marking", False):
                    raise RuntimeError("road-marking task is invalid: {}".format(native_key))
                native_valid = (native_ignore == 0).astype(np.uint8) * 255
                native_projected = projectors[view].project(
                    native_drivable * 255,
                    native_marking,
                    np.full(native_marking.shape, 255, dtype=np.uint8),
                )
                native_bev_valid = warp_native_valid(projectors[view], native_valid)
                per_mode_view["native_reference"][view] = from_hard(native_projected)
                hard_valid_by_mode["native_reference"][view] = native_bev_valid

                cache_drivable = sample["targets"]["drivable"].numpy().astype(np.uint8)
                cache_marking = sample["targets"]["road_marking"].numpy().astype(np.uint8)
                cache_valid = (
                    sample["valid_masks"]["drivable"]
                    & sample["valid_masks"]["road_marking"]
                ).numpy().astype(np.uint8)
                roundtrip_drivable = restore_class_ids(cache_drivable, geometry)
                roundtrip_marking = restore_class_ids(cache_marking, geometry)
                roundtrip_valid = restore_class_ids(cache_valid, geometry)
                roundtrip_projected = projectors[view].project(
                    roundtrip_drivable * 255,
                    roundtrip_marking,
                    np.full(roundtrip_marking.shape, 255, dtype=np.uint8),
                )
                per_mode_view["cache_roundtrip"][view] = from_hard(roundtrip_projected)
                hard_valid_by_mode["cache_roundtrip"][view] = warp_native_valid(
                    projectors[view], roundtrip_valid * 255
                )

                cache_direct = project_model_grid_hard(
                    projectors[view],
                    geometry,
                    cache_drivable.astype(np.float32),
                    cache_marking,
                    np.ones(cache_marking.shape, dtype=np.float32),
                    cache_valid,
                )
                per_mode_view["model_grid_hard"][view] = from_hard(cache_direct)
                hard_valid_by_mode["model_grid_hard"][view] = cache_direct.coverage

                marking_probability = pred_marking[batch_index].astype(np.float32)
                drivable_probability = pred_drivable[batch_index].astype(np.float32)
                native_pred_drivable = restore_probability(drivable_probability, geometry)
                native_pred_marking_prob = restore_categorical_probabilities(
                    marking_probability, geometry
                )
                native_pred_marking = np.argmax(
                    native_pred_marking_prob, axis=0
                ).astype(np.uint8)
                native_pred_confidence = np.max(native_pred_marking_prob, axis=0)
                native_pred_projected = projectors[view].project(
                    np.rint(native_pred_drivable * 255.0).astype(np.uint8),
                    native_pred_marking,
                    np.rint(native_pred_confidence * 255.0).astype(np.uint8),
                )
                per_mode_view["prediction_native_hard"][view] = from_hard(
                    native_pred_projected
                )
                hard_valid_by_mode["prediction_native_hard"][view] = warp_native_valid(
                    projectors[view], roundtrip_valid * 255
                )

                model_pred_class = np.argmax(marking_probability, axis=0).astype(np.uint8)
                model_pred_confidence = np.max(marking_probability, axis=0)
                model_pred_hard = project_model_grid_hard(
                    projectors[view],
                    geometry,
                    drivable_probability,
                    model_pred_class,
                    model_pred_confidence,
                    cache_valid,
                )
                per_mode_view["prediction_model_hard"][view] = from_hard(model_pred_hard)
                hard_valid_by_mode["prediction_model_hard"][view] = model_pred_hard.coverage

                soft_projected[view] = project_model_grid_probabilities(
                    projectors[view],
                    geometry,
                    drivable_probability,
                    marking_probability,
                    cache_valid,
                )
                soft_view_class = np.argmax(
                    soft_projected[view].road_marking_probabilities, axis=0
                ).astype(np.uint8)
                soft_view_class[soft_projected[view].coverage == 0] = 0
                per_mode_view["prediction_model_soft"][view] = EvalRaster(
                    drivable=soft_projected[view].drivable_probability,
                    marking=soft_view_class,
                    coverage=soft_projected[view].coverage > 0,
                    marking_probabilities=soft_projected[view].road_marking_probabilities,
                    marking_confidence=np.max(
                        soft_projected[view].road_marking_probabilities, axis=0
                    ),
                )

                expected_image = (data_root / row["image"]).resolve()
                if not expected_image.is_file():
                    raise RuntimeError("dataset image disappeared during audit: {}".format(expected_image))

            fused_by_mode = {}
            for mode in MODES:
                if mode == "prediction_model_soft":
                    fused_by_mode[mode] = from_soft(
                        fuse_probabilities(soft_projected, quality)
                    )
                    continue
                # The rasters are already projected. Recreate the lightweight
                # ProjectedSemantic contract without applying a second warp.
                aligned = {
                    view: ProjectedSemantic(
                        np.rint(per_mode_view[mode][view].drivable * 255.0).astype(np.uint8),
                        per_mode_view[mode][view].marking,
                        np.rint(
                            np.clip(
                                per_mode_view[mode][view].marking_confidence,
                                0.0,
                                1.0,
                            )
                            * 255.0
                        ).astype(np.uint8),
                        np.where(
                            per_mode_view[mode][view].coverage, 255, 0
                        ).astype(np.uint8),
                    )
                    for view in VIEWS
                }
                fused_by_mode[mode] = from_hard(
                    hard_fusion.fuse(aligned, valid_by_view=hard_valid_by_mode[mode])
                )

            condition = weather_from_run(run_id)
            native_reference = per_mode_view["native_reference"]
            fused_reference = fused_by_mode["native_reference"]
            for mode in MODES:
                for view in VIEWS:
                    add_to_store(
                        store,
                        mode,
                        condition,
                        view,
                        per_mode_view[mode][view],
                        native_reference[view],
                        x,
                        regions,
                        grid.resolution_m,
                    )
                add_to_store(
                    store,
                    mode,
                    condition,
                    "fused",
                    fused_by_mode[mode],
                    fused_reference,
                    x,
                    regions,
                    grid.resolution_m,
                )

            overlay = None
            if run_id not in representative_runs and args.overlay_count:
                representative_runs.add(run_id)
                overlay = render_overlay(run_id, frame_id, fused_by_mode)
                cv2.imwrite(
                    str(overlays_dir / "representative_{}_f{:06d}.png".format(run_id, frame_id)),
                    overlay,
                )
            if args.overlay_count:
                score = frame_f1(
                    fused_by_mode["prediction_model_soft"],
                    fused_reference,
                    0.2,
                    grid.resolution_m,
                ) - frame_f1(
                    fused_by_mode["prediction_native_hard"],
                    fused_reference,
                    0.2,
                    grid.resolution_m,
                )
                if overlay is None:
                    overlay = render_overlay(run_id, frame_id, fused_by_mode)
                item = (-score, group_index, run_id, frame_id, overlay)
                if len(worst_overlays) < args.overlay_count:
                    heapq.heappush(worst_overlays, item)
                elif score < -worst_overlays[0][0]:
                    heapq.heapreplace(worst_overlays, item)

            if args.log_interval and (
                group_index % args.log_interval == 0 or group_index == len(complete)
            ):
                print(
                    "loss-budget group={}/{} run={} frame={}".format(
                        group_index, len(complete), run_id, frame_id
                    ),
                    flush=True,
                )

    for rank, item in enumerate(sorted(worst_overlays, key=lambda value: -value[0]), 1):
        negative_score, _, run_id, frame_id, overlay = item
        score = -negative_score
        cv2.imwrite(
            str(
                overlays_dir
                / "worst_soft_delta_{:02d}_{:+.4f}_{}_f{:06d}.png".format(
                    rank, score, run_id, frame_id
                )
            ),
            overlay,
        )

    metrics = serialize_store(store)
    soft_gate = evaluate_soft_gate(metrics)
    manifest_path = data_root / "dataset_versions" / args.dataset_version / (args.split + ".jsonl")
    metadata = {
        "schema_version": "asmc-camera-bev-loss-budget-1.0.0",
        "completed_at_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "dataset_version": args.dataset_version,
        "dataset_manifest": str(manifest_path),
        "dataset_manifest_sha256": sha256(manifest_path),
        "native_target_name": args.native_target_name,
        "native_policy_id": policy_id,
        "native_policy_sha256": policy_sha256,
        "native_manifests": native_manifests,
        "checkpoint": str(checkpoint_path),
        "checkpoint_sha256": sha256(checkpoint_path),
        "weights": state_key,
        "upstream_commit": upstream_commit,
        "groups": len(complete),
        "samples": len(complete) * len(VIEWS),
        "incomplete_groups": incomplete,
        "weather": sorted(weather_values),
        "cloudy_accuracy_data_used": False,
        "views": list(VIEWS),
        "model_input_hw": list(dataset.target_hw),
        "sensor_profile": {
            view: {
                "width": cameras[view].width,
                "height": cameras[view].height,
                "horizontal_fov_deg": cameras[view].horizontal_fov_deg,
            }
            for view in VIEWS
        },
        "runtime_sensor_rate_hz": {view: 20.0 for view in VIEWS},
        "evaluation_source": (
            "run-exclusive Sunny/Foggy Capture validation; frame sampling is not "
            "the 20Hz runtime sensor rate"
        ),
        "calibration": {"path": str(calibration_path), "sha256": sha256(calibration_path)},
        "ground_plane": cameras["front"].ground_plane.to_mapping(),
        "bev_config": {"path": str(grid_path), "sha256": sha256(grid_path)},
        "bev_grid": {
            "x_min_m": grid.x_min_m,
            "x_max_m": grid.x_max_m,
            "y_min_m": grid.y_min_m,
            "y_max_m": grid.y_max_m,
            "resolution_m": grid.resolution_m,
            "shape_hw": list(grid.shape),
        },
        "modes": list(MODES),
        "shared_representation_note": (
            "native_reference is the native-label projection into the fixed metric grid; "
            "homography and 0.10m rasterization are a shared representation floor, not "
            "claimed as independently observable ground-truth error"
        ),
        "elapsed_seconds": time.perf_counter() - started,
    }
    with (working_dir / "metadata.json").open("x", encoding="utf-8") as stream:
        json.dump(metadata, stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.write("\n")
    with (working_dir / "metrics.json").open("x", encoding="utf-8") as stream:
        json.dump(
            {"soft_bev_gate": soft_gate, "metrics": metrics},
            stream,
            ensure_ascii=False,
            indent=2,
            sort_keys=True,
        )
        stream.write("\n")
    write_metrics_csv(working_dir / "metrics_by_bin.csv", metrics)
    os.replace(working_dir, output_dir)
    print(json.dumps(soft_gate, ensure_ascii=False, indent=2, sort_keys=True))
    print(output_dir)


if __name__ == "__main__":
    main()
