#!/usr/bin/env python3
"""Evaluate direct metric lane evidence and robust splines against native GT.

The script deliberately bypasses a BEV lane raster: both model-grid predictions
and native labels are converted to continuous ``base_link`` points.  It also
writes the compact inputs consumed by ``analyze_camera_lane_range.py`` so range
selection cannot silently use a different checkpoint or geometry.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import sys
import time
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np
import torch
import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "perception"))
sys.path.insert(
    0, str(REPO_ROOT / "src/perception/camera_semantic_perception/src")
)
sys.path.insert(0, str(REPO_ROOT / "src/perception/drivable_bev/src"))

from camera_semantic_perception.geometry import LetterboxGeometry  # noqa: E402
from drivable_bev.calibration import load_calibration_snapshot  # noqa: E402
from drivable_bev.fusion import CameraBevFusion, build_quality_maps  # noqa: E402
from drivable_bev.grid import BevGridSpec  # noqa: E402
from drivable_bev.lane_evaluation import nearest_distances  # noqa: E402
from drivable_bev.lane_evidence import reduce_evidence_to_nodes  # noqa: E402
from drivable_bev.lane_tracking import associate_lane_nodes  # noqa: E402
from drivable_bev.model_grid_projection import (  # noqa: E402
    MetricEvidence,
    ModelGridGroundProjector,
)
from drivable_bev.offline_evaluation import project_model_grid_hard  # noqa: E402
from drivable_bev.projector import CameraBevProjector  # noqa: E402
from drivable_bev.spline_fitting import fit_lane_tracks  # noqa: E402
from drivable_bev.visualization import (  # noqa: E402
    draw_metric_evidence,
    draw_metric_grid,
    draw_metric_lanes,
)
from twinlite_morai import (  # noqa: E402
    MoraiTwinLiteDataset,
    adapt_twinlite_outputs,
    configure_twinlite_task,
    load_external_twinlite,
    task_from_checkpoint,
)


VIEWS = ("front", "left", "right")
SOURCE_MASKS = {"front": 1, "left": 2, "right": 4}
CLASSES = {"white": 1, "yellow": 2}
TOLERANCES = (0.1, 0.2, 0.3)
AUDIT_GRID = BevGridSpec(-5.0, 30.0, -12.0, 12.0, 0.10, "base_link")


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
    parser.add_argument("--config", default="medium")
    parser.add_argument("--twinlite-root", default=os.environ.get("TWINLITE_ROOT"))
    parser.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    parser.add_argument("--weights", default="auto", choices=("auto", "ema", "model"))
    parser.add_argument("--lane-config", default=str(REPO_ROOT / "src/perception/drivable_bev/config/lane_fitting.yaml"))
    parser.add_argument("--max-groups", type=int)
    parser.add_argument("--overlay-count", type=int, default=60)
    parser.add_argument("--log-interval", type=int, default=20)
    return parser.parse_args()


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def weather_from_run(run_id):
    if "_foggy_" in run_id:
        return "foggy"
    if "_sunny_" in run_id:
        return "sunny"
    return "unknown"


def geometry_from_meta(value):
    return LetterboxGeometry(
        original_hw=tuple(int(item) for item in value["original_hw"]),
        target_hw=tuple(int(item) for item in value["target_hw"]),
        resized_hw=tuple(int(item) for item in value["resized_hw"]),
        pad_ltrb=tuple(int(item) for item in value["pad_ltrb"]),
        scale_xy=tuple(float(item) for item in value["scale_xy"]),
    )


def read_jsonl(path):
    result = []
    with Path(path).open("r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            try:
                result.append(json.loads(line))
            except ValueError as exc:
                raise RuntimeError("invalid JSONL {}:{}: {}".format(path, line_number, exc)) from exc
    return result


def read_mask(path, shape, maximum, name):
    value = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    if value is None or value.ndim != 2 or tuple(value.shape) != tuple(shape):
        raise RuntimeError("{} mask shape/read failure: {}".format(name, path))
    if value.dtype != np.uint8 or (value.size and int(value.max()) > maximum):
        raise RuntimeError("{} mask has an invalid encoding: {}".format(name, path))
    return value


def load_native_index(data_root, run_ids, target_name):
    result = {}
    manifests = {}
    policies = set()
    policy_hashes = set()
    for run_id in sorted(run_ids):
        root = data_root / "datasets" / run_id / "derived" / target_name
        manifest = root / "manifest.jsonl"
        summary = root / "qa_summary.json"
        if not (root / "_SUCCESS").is_file() or not manifest.is_file() or not summary.is_file():
            raise RuntimeError("native target is incomplete: {}".format(root))
        manifests[run_id] = {"path": str(manifest), "sha256": sha256(manifest)}
        with summary.open("r", encoding="utf-8") as stream:
            qa = json.load(stream)
        policies.add(str(qa.get("policy_id")))
        policy_hashes.add(str(qa.get("policy_sha256")))
        for row in read_jsonl(manifest):
            key = (str(row["run_id"]), int(row["frame_id"]), str(row["view"]))
            if key in result:
                raise RuntimeError("duplicate native target {}".format(key))
            result[key] = {
                "native_hw": tuple(int(item) for item in row["native_hw"]),
                "outputs": {
                    name: (root / relative).resolve()
                    for name, relative in row["outputs"].items()
                },
                "task_valid": dict(row.get("task_valid") or {}),
            }
    if len(policies) != 1 or len(policy_hashes) != 1:
        raise RuntimeError("native targets do not share one policy")
    return result, manifests, next(iter(policies)), next(iter(policy_hashes))


def node_points(nodes, class_id):
    return np.asarray(
        [node.xy for node in nodes if int(node.class_id) == int(class_id)],
        dtype=np.float32,
    ).reshape(-1, 2)


def lane_points(lanes, class_id, observed_only=True):
    values = [
        lane.sample_points[lane.sample_observed]
        if observed_only else lane.sample_points
        for lane in lanes
        if int(lane.class_id) == int(class_id)
    ]
    return (
        np.ascontiguousarray(np.concatenate(values), dtype=np.float32)
        if values
        else np.empty((0, 2), dtype=np.float32)
    )


class MetricAccumulator:
    def __init__(self):
        self.prediction_count = 0
        self.target_count = 0
        self.matched_prediction = {value: 0 for value in TOLERANCES}
        self.matched_target = {value: 0 for value in TOLERANCES}
        self.distances = []

    def add(self, prediction, target):
        pred = np.asarray(prediction, dtype=np.float64).reshape(-1, 2)
        truth = np.asarray(target, dtype=np.float64).reshape(-1, 2)
        pred_distance = nearest_distances(pred, truth)
        target_distance = nearest_distances(truth, pred)
        self.prediction_count += len(pred)
        self.target_count += len(truth)
        for tolerance in TOLERANCES:
            self.matched_prediction[tolerance] += int(np.count_nonzero(pred_distance <= tolerance))
            self.matched_target[tolerance] += int(np.count_nonzero(target_distance <= tolerance))
        finite = np.concatenate(
            [pred_distance[np.isfinite(pred_distance)], target_distance[np.isfinite(target_distance)]]
        )
        if len(finite):
            step = max(1, int(np.ceil(len(finite) / 4000)))
            self.distances.extend(finite[::step].astype(np.float32).tolist())

    def result(self):
        tolerance = {}
        for value in TOLERANCES:
            precision = self.matched_prediction[value] / max(1, self.prediction_count)
            recall = self.matched_target[value] / max(1, self.target_count)
            tolerance["{:.1f}m".format(value)] = {
                "precision": precision,
                "recall": recall,
                "f1": 2.0 * precision * recall / max(precision + recall, np.finfo(float).eps),
                "matched_prediction": self.matched_prediction[value],
                "matched_target": self.matched_target[value],
            }
        distances = np.asarray(self.distances, dtype=np.float64)
        return {
            "prediction_count": self.prediction_count,
            "target_count": self.target_count,
            "tolerance": tolerance,
            "chamfer_p50_m": float(np.percentile(distances, 50)) if len(distances) else None,
            "chamfer_p95_m": float(np.percentile(distances, 95)) if len(distances) else None,
        }


def region_name(x, y):
    x_bin = int(np.floor(float(x) / 5.0) * 5)
    abs_y = abs(float(y))
    y_bin = int(np.floor(abs_y / 2.0) * 2)
    return "x_{:+03d}_{:+03d}m_yabs_{:02d}_{:02d}m".format(
        x_bin, x_bin + 5, y_bin, y_bin + 2
    )


def add_metrics(store, method, condition, class_name, prediction, target):
    store[(method, condition, class_name, "all")].add(prediction, target)
    regions = set()
    for points in (prediction, target):
        regions.update(region_name(x, y) for x, y in np.asarray(points).reshape(-1, 2))
    for name in sorted(regions):
        pieces = name.split("_")
        x_min, x_max = float(pieces[1]), float(pieces[2][:-1])
        y_min, y_max = float(pieces[4]), float(pieces[5][:-1])
        def select(points):
            points = np.asarray(points).reshape(-1, 2)
            keep = (
                (points[:, 0] >= x_min) & (points[:, 0] < x_max)
                & (np.abs(points[:, 1]) >= y_min) & (np.abs(points[:, 1]) < y_max)
            )
            return points[keep]
        store[(method, condition, class_name, name)].add(select(prediction), select(target))


def frame_distance_metrics(prediction, target, tolerance=0.2):
    """Return symmetric point metrics for one frame/class pair.

    Precision is measured from prediction to GT and recall from GT to
    prediction.  Calling a one-way match rate ``F1`` made the review CSV easy
    to misread, so the per-frame export uses the same symmetric definition as
    the aggregate evaluator.
    """
    pred_distance = nearest_distances(prediction, target)
    target_distance = nearest_distances(target, prediction)
    precision = float(np.mean(pred_distance <= tolerance)) if len(pred_distance) else 0.0
    recall = float(np.mean(target_distance <= tolerance)) if len(target_distance) else 0.0
    f1 = 2.0 * precision * recall / max(
        precision + recall, np.finfo(float).eps
    )
    finite = np.concatenate(
        [
            pred_distance[np.isfinite(pred_distance)],
            target_distance[np.isfinite(target_distance)],
        ]
    )
    return precision, recall, f1, (
        float(np.percentile(finite, 95)) if len(finite) else ""
    )


def duplicate_lane_count(lanes, threshold_m=0.30):
    count = 0
    for index, first in enumerate(lanes):
        for second in lanes[index + 1 :]:
            if first.class_id != second.class_id:
                continue
            lower = max(first.valid_x_min_m, second.valid_x_min_m)
            upper = min(first.valid_x_max_m, second.valid_x_max_m)
            if upper - lower < 2.0:
                continue
            first_points = first.sample_points
            second_points = second.sample_points
            first_points = first_points[(first_points[:, 0] >= lower) & (first_points[:, 0] <= upper)]
            second_points = second_points[(second_points[:, 0] >= lower) & (second_points[:, 0] <= upper)]
            distance = nearest_distances(first_points, second_points)
            if len(distance) and float(np.median(distance)) <= threshold_m:
                count += 1
    return count


def append_points(storage, points, class_id):
    value = np.asarray(points, dtype=np.float32).reshape(-1, 2)
    storage["xy"].append(value)
    storage["class_id"].append(np.full(len(value), class_id, dtype=np.uint8))


def pack_records(values, dtype, width=None):
    offsets = [0]
    flattened = []
    for value in values:
        array = np.asarray(value, dtype=dtype)
        if width is not None:
            array = array.reshape(-1, width)
        flattened.append(array)
        offsets.append(offsets[-1] + len(array))
    if flattened and offsets[-1]:
        data = np.concatenate(flattened, axis=0)
    else:
        data = np.empty((0, width), dtype=dtype) if width else np.empty(0, dtype=dtype)
    return data, np.asarray(offsets, dtype=np.int64)


def render_overlay(grid, direct_points, direct_classes, lanes, target_points, target_classes):
    canvas = np.zeros((*grid.shape, 3), dtype=np.uint8)
    draw_metric_grid(canvas, grid, 5.0, 2.0)
    draw_metric_evidence(
        canvas,
        target_points,
        target_classes,
        grid,
        radius=2,
        colors={1: (110, 110, 110), 2: (70, 120, 120)},
    )
    draw_metric_evidence(
        canvas,
        direct_points,
        direct_classes,
        grid,
        radius=1,
        colors={1: (255, 255, 0), 2: (0, 128, 255)},
    )
    draw_metric_lanes(canvas, [(lane.class_id, lane.sample_points) for lane in lanes], grid, thickness=2)
    image = cv2.resize(canvas, (672, 1050), interpolation=cv2.INTER_NEAREST)
    cv2.putText(
        image,
        "GT=gray direct=cyan/orange candidate-spline=white/yellow",
        (10, 24),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.52,
        (0, 0, 0),
        3,
        cv2.LINE_AA,
    )
    cv2.putText(
        image,
        "GT=gray direct=cyan/orange candidate-spline=white/yellow",
        (10, 24),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.52,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    return image


def main():
    args = parse_args()
    if args.max_groups is not None and args.max_groups <= 0:
        raise ValueError("max-groups must be positive")
    output = Path(args.output_dir).expanduser().resolve()
    working = output.with_name(output.name + ".part")
    if output.exists() or working.exists():
        raise RuntimeError("refusing to overwrite output or partial output: {}".format(output))
    working.mkdir(parents=True)
    overlays = working / "overlays"
    overlays.mkdir()

    data_root = Path(args.data_root).expanduser().resolve()
    dataset = MoraiTwinLiteDataset(
        data_root=data_root,
        dataset_version=args.dataset_version,
        split=args.split,
    )
    grouped = defaultdict(dict)
    for index, row in enumerate(dataset.rows):
        grouped[(str(row["run_id"]), int(row["frame_id"]))][str(row["view"])] = index
    complete = [(key, values) for key, values in sorted(grouped.items()) if set(values) == set(VIEWS)]
    incomplete = len(grouped) - len(complete)
    if args.max_groups is not None:
        complete = complete[: args.max_groups]
    if not complete:
        raise RuntimeError("dataset contains no complete 3-view groups")
    overlay_count = min(max(0, int(args.overlay_count)), len(complete))
    overlay_group_numbers = set(
        int(value)
        for value in np.linspace(1, len(complete), overlay_count, dtype=np.int64)
    ) if overlay_count else set()
    conditions = {weather_from_run(key[0]) for key, _ in complete}
    if conditions - {"sunny", "foggy"}:
        raise RuntimeError("accuracy audit accepts only Sunny/Foggy runs: {}".format(sorted(conditions)))

    native, manifests, policy_id, policy_hash = load_native_index(
        data_root, {key[0] for key, _ in complete}, args.native_target_name
    )
    calibration_path = REPO_ROOT / "src/perception/drivable_bev/config/cameras_v2.yaml"
    _, cameras = load_calibration_snapshot(calibration_path)
    lane_config_path = Path(args.lane_config).expanduser().resolve()
    with lane_config_path.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)

    model_projectors = {
        view: ModelGridGroundProjector(
            cameras[view],
            model_hw=tuple(config["model_input_hw"]),
            source_mask=SOURCE_MASKS[view],
            x_bounds_m=(config["evidence_x_min_m"], config["evidence_x_max_m"]),
            y_bounds_m=(config["evidence_y_min_m"], config["evidence_y_max_m"]),
            max_ground_range_m=config["max_ground_range_m"],
            min_camera_depth_m=config["min_camera_depth_m"],
            edge_taper_fraction=config["edge_taper_fraction"],
            forward_fade_start_m=config["forward_fade_start_m"],
        )
        for view in VIEWS
    }
    native_projectors = {
        view: ModelGridGroundProjector(
            cameras[view],
            model_hw=(cameras[view].height, cameras[view].width),
            source_mask=SOURCE_MASKS[view],
            x_bounds_m=(config["evidence_x_min_m"], config["evidence_x_max_m"]),
            y_bounds_m=(config["evidence_y_min_m"], config["evidence_y_max_m"]),
            max_ground_range_m=config["max_ground_range_m"],
            min_camera_depth_m=config["min_camera_depth_m"],
            edge_taper_fraction=config["edge_taper_fraction"],
            forward_fade_start_m=config["forward_fade_start_m"],
        )
        for view in VIEWS
    }
    raster_projectors = {
        view: CameraBevProjector(cameras[view], AUDIT_GRID, 35.0, config["min_camera_depth_m"])
        for view in VIEWS
    }
    fusion = CameraBevFusion(
        build_quality_maps(raster_projectors, AUDIT_GRID),
        {"front": 1, "left": 2, "right": 3},
    )

    checkpoint_path = Path(args.checkpoint).expanduser().resolve()
    checkpoint = torch.load(str(checkpoint_path), map_location="cpu")
    task = task_from_checkpoint(checkpoint)
    if task.secondary_head != "road_marking":
        raise RuntimeError("lane vector audit needs a 4-class road-marking checkpoint")
    model, upstream_commit = load_external_twinlite(config=args.config, root=args.twinlite_root)
    configure_twinlite_task(model, task)
    if checkpoint.get("upstream_commit") != upstream_commit:
        raise RuntimeError("checkpoint and TwinLite source commits differ")
    use_ema = args.weights == "ema" or (args.weights == "auto" and checkpoint.get("ema_state_dict") is not None)
    state_key = "ema_state_dict" if use_ema else "model_state_dict"
    if checkpoint.get(state_key) is None:
        raise RuntimeError("checkpoint does not contain {}".format(state_key))
    model.load_state_dict(checkpoint[state_key])
    device = torch.device(args.device)
    model = model.to(device).eval()

    threshold = {1: float(config["confidence_by_class"]["white"]), 2: float(config["confidence_by_class"]["yellow"])}
    node_parameters = {
        "x_bin_m": float(config["x_bin_m"]),
        "lateral_cluster_gap_m": float(config["lateral_cluster_gap_m"]),
        "x_origin_m": float(config["evidence_x_min_m"]),
    }
    track_parameters = {
        "max_track_gap_m": float(config["max_track_gap_m"]),
        "max_lateral_residual_m": float(config["max_lateral_residual_m"]),
        "max_heading_change_deg": float(config["max_heading_change_deg"]),
        "min_track_nodes": int(config["min_track_nodes"]),
        "min_observed_span_m": float(config["min_observed_span_m"]),
    }
    spline_parameters = {
        "smoothing": float(config["spline_smoothing"]),
        "sample_spacing_m": float(config["spline_sample_spacing_m"]),
        "robust_iterations": int(config["spline_robust_iterations"]),
        "outlier_scale": float(config["spline_outlier_scale"]),
        "control_spacing_m": float(config["spline_control_spacing_m"]),
    }

    metrics = defaultdict(MetricAccumulator)
    frame_rows = []
    manual_rows = []
    timings = []
    saved = {
        "predicted": {"xy": [], "class_id": []},
        "target": {"xy": [], "class_id": []},
        "drivable_prediction": [],
        "drivable_target": [],
        "drivable_valid": [],
    }
    record_run = []
    record_frame = []
    record_weather = []
    started = time.perf_counter()

    with torch.no_grad():
        for group_number, ((run_id, frame_id), indices) in enumerate(complete, 1):
            samples = {view: dataset[indices[view]] for view in VIEWS}
            batch = torch.stack([samples[view]["image"] for view in VIEWS]).to(device)
            with torch.cuda.amp.autocast(enabled=device.type == "cuda"):
                logits = adapt_twinlite_outputs(model(batch), task.secondary_head)
                drivable_probability = torch.softmax(logits["drivable"].float(), dim=1)[:, 1]
                marking_probability = torch.softmax(logits["road_marking"].float(), dim=1)
            drivable_probability = drivable_probability.cpu().numpy()
            marking_probability = marking_probability.cpu().numpy()

            predicted_evidence = []
            target_evidence = []
            pred_rasters = {}
            target_rasters = {}
            target_valid_by_view = {}
            for batch_index, view in enumerate(VIEWS):
                geometry = geometry_from_meta(samples[view]["meta"]["geometry"])
                pred_class = np.argmax(marking_probability[batch_index], axis=0).astype(np.uint8)
                pred_confidence = np.max(marking_probability[batch_index], axis=0).astype(np.float32)
                predicted_evidence.append(
                    model_projectors[view].extract(pred_class, pred_confidence, threshold)
                )

                key = (run_id, frame_id, view)
                if key not in native:
                    raise RuntimeError("native target is missing: {}".format(key))
                row = native[key]
                native_marking = read_mask(row["outputs"]["road_marking"], row["native_hw"], 3, "road_marking")
                native_drivable = read_mask(row["outputs"]["drivable"], row["native_hw"], 1, "drivable")
                native_ignore = read_mask(row["outputs"]["drivable_ignore"], row["native_hw"], 1, "drivable_ignore")
                target_evidence.append(
                    native_projectors[view].extract(
                        native_marking,
                        np.ones(native_marking.shape, dtype=np.float32),
                        {1: 1.0, 2: 1.0},
                    )
                )
                valid = samples[view]["valid_masks"]["drivable"].numpy().astype(np.uint8)
                pred_rasters[view] = project_model_grid_hard(
                    raster_projectors[view], geometry,
                    drivable_probability[batch_index], pred_class, pred_confidence, valid,
                )
                target_rasters[view] = raster_projectors[view].project(
                    native_drivable * 255, native_marking,
                    np.full(native_marking.shape, 255, dtype=np.uint8),
                )
                valid_projected = raster_projectors[view].project(
                    (native_ignore == 0).astype(np.uint8) * 255,
                    np.zeros(native_ignore.shape, dtype=np.uint8),
                ).drivable_probability
                target_valid_by_view[view] = valid_projected

            prediction = MetricEvidence.concatenate(predicted_evidence)
            target = MetricEvidence.concatenate(target_evidence)
            vector_started = time.perf_counter()
            predicted_nodes = reduce_evidence_to_nodes(prediction, **node_parameters)
            target_nodes = reduce_evidence_to_nodes(target, **node_parameters)
            tracks = associate_lane_nodes(predicted_nodes, **track_parameters)
            lanes = fit_lane_tracks(tracks, **spline_parameters)
            vector_ms = (time.perf_counter() - vector_started) * 1000.0
            timings.append(vector_ms)
            condition = weather_from_run(run_id)
            duplicate_count = duplicate_lane_count(lanes)
            maximum_bridged_ratio = max(
                [lane.bridged_length_ratio for lane in lanes], default=0.0
            )

            direct_xy_parts = []
            direct_class_parts = []
            target_xy_parts = []
            target_class_parts = []
            fitted_xy_parts = []
            fitted_class_parts = []
            for class_name, class_id in CLASSES.items():
                direct = node_points(predicted_nodes, class_id)
                truth = node_points(target_nodes, class_id)
                # Visible-only GT cannot score a deliberately interpolated
                # dash/gap as positive. Quantitative distance therefore uses
                # observed support; bridges remain in overlays/manual review.
                fitted = lane_points(lanes, class_id, observed_only=True)
                add_metrics(metrics, "direct", condition, class_name, direct, truth)
                add_metrics(metrics, "fitted", condition, class_name, fitted, truth)
                direct_xy_parts.append(direct)
                direct_class_parts.append(np.full(len(direct), class_id, dtype=np.uint8))
                target_xy_parts.append(truth)
                target_class_parts.append(np.full(len(truth), class_id, dtype=np.uint8))
                fitted_xy_parts.append(fitted)
                fitted_class_parts.append(np.full(len(fitted), class_id, dtype=np.uint8))
                precision, recall, f1, chamfer_p95 = frame_distance_metrics(
                    fitted, truth, tolerance=0.2
                )
                frame_rows.append({
                    "run_id": run_id,
                    "frame_id": frame_id,
                    "weather": condition,
                    "class": class_name,
                    "direct_points": len(direct),
                    "fitted_points": len(fitted),
                    "target_points": len(truth),
                    "fitted_precision_0.2m": precision,
                    "fitted_recall_0.2m": recall,
                    "fitted_f1_0.2m": f1,
                    "fitted_chamfer_p95_m": chamfer_p95,
                    "lane_count": len(lanes),
                    "duplicate_lane_count": duplicate_count,
                    "maximum_bridged_ratio": maximum_bridged_ratio,
                    "vector_ms": vector_ms,
                })
            fitted_xy = np.concatenate(fitted_xy_parts) if any(len(value) for value in fitted_xy_parts) else np.empty((0, 2), np.float32)
            fitted_class = np.concatenate(fitted_class_parts) if any(len(value) for value in fitted_class_parts) else np.empty(0, np.uint8)
            truth_xy = np.concatenate(target_xy_parts) if any(len(value) for value in target_xy_parts) else np.empty((0, 2), np.float32)
            truth_class = np.concatenate(target_class_parts) if any(len(value) for value in target_class_parts) else np.empty(0, np.uint8)
            direct_xy = np.concatenate(direct_xy_parts) if any(len(value) for value in direct_xy_parts) else np.empty((0, 2), np.float32)
            direct_class = np.concatenate(direct_class_parts) if any(len(value) for value in direct_class_parts) else np.empty(0, np.uint8)

            append_points(saved["predicted"], fitted_xy, 0)
            saved["predicted"]["class_id"][-1] = fitted_class
            append_points(saved["target"], truth_xy, 0)
            saved["target"]["class_id"][-1] = truth_class
            fused_prediction = fusion.fuse(pred_rasters)
            fused_target = fusion.fuse(target_rasters, valid_by_view=target_valid_by_view)
            valid = (fused_prediction.coverage > 0) & (fused_target.coverage > 0)
            saved["drivable_prediction"].append(fused_prediction.drivable_probability >= 128)
            saved["drivable_target"].append(fused_target.drivable_probability >= 128)
            saved["drivable_valid"].append(valid)
            record_run.append(run_id)
            record_frame.append(frame_id)
            record_weather.append(condition)

            if group_number in overlay_group_numbers:
                image = render_overlay(AUDIT_GRID, direct_xy, direct_class, lanes, truth_xy, truth_class)
                overlay_name = "{:03d}_{}_f{:06d}.png".format(group_number, run_id, frame_id)
                cv2.imwrite(str(overlays / overlay_name), image)
                manual_rows.append({
                    "overlay": "overlays/" + overlay_name,
                    "run_id": run_id,
                    "frame_id": frame_id,
                    "weather": condition,
                    "condition": "unclassified",
                    "false_bridge": "",
                    "duplicate_lane": "",
                    "non_lane_marking_false_positive": "",
                    "parking_or_safety_zone_context": "",
                    "calibration_issue": "",
                    "notes": "",
                })
            if args.log_interval and group_number % args.log_interval == 0:
                elapsed = time.perf_counter() - started
                print("groups={}/{} elapsed={:.1f}s vector_p95={:.2f}ms".format(
                    group_number, len(complete), elapsed, np.percentile(timings, 95)
                ), flush=True)

    serialized = defaultdict(dict)
    for (method, condition, class_name, region), accumulator in sorted(metrics.items()):
        serialized.setdefault(method, {}).setdefault(condition, {}).setdefault(class_name, {})[region] = accumulator.result()
    timing_array = np.asarray(timings, dtype=np.float64)
    metrics_document = {
        "schema_version": "asmc-camera-lane-vector-audit-1.0.0",
        "groups": len(complete),
        "incomplete_groups_excluded": incomplete,
        "metrics": serialized,
        "runtime_ms": {
            "p50": float(np.percentile(timing_array, 50)),
            "p95": float(np.percentile(timing_array, 95)),
            "maximum": float(np.max(timing_array)),
            "target_p95": 10.0,
            "target_passed": bool(np.percentile(timing_array, 95) <= 10.0),
        },
    }
    with (working / "metrics.json").open("w", encoding="utf-8") as stream:
        json.dump(metrics_document, stream, indent=2, ensure_ascii=False)

    bin_rows = []
    for method, by_condition in serialized.items():
        for condition, by_class in by_condition.items():
            for class_name, by_region in by_class.items():
                for region, value in by_region.items():
                    row = {"method": method, "weather": condition, "class": class_name, "region": region}
                    row.update({"f1_{}".format(key): item["f1"] for key, item in value["tolerance"].items()})
                    row.update({"recall_{}".format(key): item["recall"] for key, item in value["tolerance"].items()})
                    row["chamfer_p50_m"] = value["chamfer_p50_m"]
                    row["chamfer_p95_m"] = value["chamfer_p95_m"]
                    row["prediction_count"] = value["prediction_count"]
                    row["target_count"] = value["target_count"]
                    bin_rows.append(row)
    for name, rows in (("metrics_by_bin.csv", bin_rows), ("frame_metrics.csv", frame_rows), ("manual_review.csv", manual_rows)):
        with (working / name).open("w", encoding="utf-8", newline="") as stream:
            if rows:
                writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)

    pred_xy, pred_offsets = pack_records(saved["predicted"]["xy"], np.float32, 2)
    pred_class, _ = pack_records(saved["predicted"]["class_id"], np.uint8)
    target_xy, target_offsets = pack_records(saved["target"]["xy"], np.float32, 2)
    target_class, _ = pack_records(saved["target"]["class_id"], np.uint8)
    masks = [
        np.packbits(np.asarray(values, dtype=bool).reshape(len(complete), -1), axis=1, bitorder="little")
        for values in (
            saved["drivable_prediction"],
            saved["drivable_target"],
            saved["drivable_valid"],
        )
    ]
    np.savez_compressed(
        str(working / "range_inputs.npz"),
        run_id=np.asarray(record_run), frame_id=np.asarray(record_frame, dtype=np.int64),
        weather=np.asarray(record_weather),
        predicted_xy=pred_xy, predicted_class_id=pred_class, predicted_offsets=pred_offsets,
        target_xy=target_xy, target_class_id=target_class, target_offsets=target_offsets,
        drivable_prediction=masks[0], drivable_target=masks[1], drivable_valid=masks[2],
        raster_shape=np.asarray(AUDIT_GRID.shape, dtype=np.int32),
        raster_bounds=np.asarray([AUDIT_GRID.x_min_m, AUDIT_GRID.x_max_m, AUDIT_GRID.y_min_m, AUDIT_GRID.y_max_m, AUDIT_GRID.resolution_m], dtype=np.float64),
    )
    metadata = {
        "schema_version": "asmc-camera-lane-vector-metadata-1.0.0",
        "created_at": datetime.now(timezone.utc).isoformat(),
        "dataset_version": args.dataset_version,
        "split": args.split,
        "checkpoint": str(checkpoint_path),
        "checkpoint_sha256": sha256(checkpoint_path),
        "weights": state_key,
        "upstream_commit": upstream_commit,
        "native_target_name": args.native_target_name,
        "native_policy_id": policy_id,
        "native_policy_sha256": policy_hash,
        "native_manifests": manifests,
        "calibration": str(calibration_path),
        "calibration_sha256": sha256(calibration_path),
        "ground_plane": cameras["front"].ground_plane.to_mapping(),
        "lane_config": str(lane_config_path),
        "lane_config_sha256": sha256(lane_config_path),
        "parameters": config,
        "audit_grid": [AUDIT_GRID.x_min_m, AUDIT_GRID.x_max_m, AUDIT_GRID.y_min_m, AUDIT_GRID.y_max_m, AUDIT_GRID.resolution_m],
        "manual_review_complete": False,
        "evaluation_contract": {
            "ground_truth": "native visible-only road-marking masks projected to continuous base_link points",
            "direct": "clustered model-grid evidence nodes",
            "fitted": "only spline samples supported by observed evidence",
            "bridged_gaps": "excluded from quantitative point metrics and retained for manual false-bridge review",
            "manual_review_sampling": "evenly spaced across all evaluated 3-view groups",
            "lane_boundary_limit": "color-only Semantic GT does not distinguish a lane boundary from same-color parking or safety-zone markings; those false positives require manual/map-assisted review",
        },
        "notes": [
            "Native GT and predictions are evaluated as continuous ground points, not lane raster pixels.",
            "False-bridge acceptance remains pending until manual_review.csv is completed.",
            "White/yellow parking and safety-zone markings are not lane-boundary GT; review their false-positive status separately.",
        ],
    }
    with (working / "metadata.json").open("w", encoding="utf-8") as stream:
        json.dump(metadata, stream, indent=2, ensure_ascii=False)
    working.rename(output)
    print("wrote {} groups to {}".format(len(complete), output))


if __name__ == "__main__":
    try:
        main()
    except Exception:
        # Preserve partial output for diagnosis; no success marker is emitted.
        raise
