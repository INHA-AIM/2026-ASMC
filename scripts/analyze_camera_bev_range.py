#!/usr/bin/env python3
"""Select a conservative metric camera-BEV range from paired 3-view validation GT."""

from __future__ import annotations

import argparse
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


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "perception"))
sys.path.insert(
    0, str(REPO_ROOT / "src" / "perception" / "camera_semantic_perception" / "src")
)
sys.path.insert(0, str(REPO_ROOT / "src" / "perception" / "drivable_bev" / "src"))

from camera_semantic_perception.geometry import (  # noqa: E402
    LetterboxGeometry,
    restore_categorical_probabilities,
    restore_class_ids,
    restore_probability,
)
from drivable_bev.calibration import load_calibration_snapshot  # noqa: E402
from drivable_bev.fusion import CameraBevFusion, build_quality_maps  # noqa: E402
from drivable_bev.grid import BevGridSpec  # noqa: E402
from drivable_bev.projector import CameraBevProjector  # noqa: E402
from twinlite_morai import (  # noqa: E402
    MoraiTwinLiteDataset,
    adapt_twinlite_outputs,
    configure_twinlite_task,
    load_external_twinlite,
    task_from_checkpoint,
)


VIEWS = ("front", "left", "right")


class BinaryStats:
    def __init__(self):
        self.intersection = 0
        self.union = 0
        self.prediction_positive = 0
        self.target_positive = 0
        self.valid_pixels = 0

    def add(self, prediction, target, valid):
        prediction = np.asarray(prediction, dtype=bool) & valid
        target = np.asarray(target, dtype=bool) & valid
        self.intersection += int(np.count_nonzero(prediction & target))
        self.union += int(np.count_nonzero(prediction | target))
        self.prediction_positive += int(np.count_nonzero(prediction))
        self.target_positive += int(np.count_nonzero(target))
        self.valid_pixels += int(np.count_nonzero(valid))

    def result(self):
        return {
            "intersection": self.intersection,
            "union": self.union,
            "prediction_positive": self.prediction_positive,
            "target_positive": self.target_positive,
            "valid_pixels": self.valid_pixels,
            "iou": self.intersection / max(1, self.union),
            "precision": self.intersection / max(1, self.prediction_positive),
            "recall": self.intersection / max(1, self.target_positive),
        }


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-root", default=os.environ.get("ASMC_DATA", "/data"))
    parser.add_argument(
        "--dataset-version",
        default="twinlite_morai_v6_road_marking_actor_negative",
    )
    parser.add_argument("--split", default="val", choices=("train", "val", "test"))
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--twinlite-root", default=os.environ.get("TWINLITE_ROOT"))
    parser.add_argument("--config", default="medium")
    parser.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    parser.add_argument("--weights", default="auto", choices=("auto", "ema", "model"))
    parser.add_argument("--max-groups", type=int)
    parser.add_argument("--log-interval", type=int, default=20)
    parser.add_argument("--drivable-iou-threshold", type=float, default=0.95)
    parser.add_argument("--marking-iou-threshold", type=float, default=0.65)
    parser.add_argument("--marking-recall-threshold", type=float, default=0.75)
    parser.add_argument("--minimum-marking-support", type=int, default=100)
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


def weather(run_id):
    if "_foggy_" in run_id:
        return "foggy"
    if "_sunny_" in run_id:
        return "sunny"
    return "unknown"


def warp_valid(projector, native_mask):
    result = cv2.warpPerspective(
        np.asarray(native_mask, dtype=np.uint8),
        projector.homography.bev_from_image,
        (projector.grid.width_px, projector.grid.height_px),
        flags=cv2.INTER_NEAREST,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=0,
    )
    result[projector.homography.coverage == 0] = 0
    return np.ascontiguousarray(result, dtype=np.uint8)


def metric_masks(grid):
    rows, columns = np.indices(grid.shape, dtype=np.float64)
    ground = grid.pixel_to_metric(
        np.column_stack([columns.reshape(-1), rows.reshape(-1)])
    ).reshape((*grid.shape, 2))
    return ground[:, :, 0], ground[:, :, 1]


def new_metric_store(candidates):
    return {
        candidate: defaultdict(
            lambda: defaultdict(
                lambda: {
                    "drivable": BinaryStats(),
                    "marking": BinaryStats(),
                    "stopline": BinaryStats(),
                }
            )
        )
        for candidate in candidates
    }


def add_regions(store, candidate, condition, x, y, prediction, target, valid):
    x_max, y_half = candidate
    base = (
        (x >= -5.0)
        & (x < float(x_max))
        & (np.abs(y) < float(y_half))
        & valid
    )
    regions = {
        "all": base,
        "outer_forward": base & (x >= float(x_max) - 5.0),
        "outer_lateral": base & (np.abs(y) >= float(y_half) - 4.0),
    }
    for lower in np.arange(-5.0, float(x_max), 5.0):
        upper = min(lower + 5.0, float(x_max))
        regions["longitudinal_{:+03.0f}_{:+03.0f}m".format(lower, upper)] = (
            base & (x >= lower) & (x < upper)
        )
    for lower in np.arange(-float(y_half), float(y_half), 4.0):
        upper = min(lower + 4.0, float(y_half))
        regions["lateral_{:+03.0f}_{:+03.0f}m".format(lower, upper)] = (
            base & (y >= lower) & (y < upper)
        )
    pred_drivable, pred_marking = prediction
    gt_drivable, gt_marking = target
    for name, mask in regions.items():
        values = store[candidate][condition][name]
        values["drivable"].add(pred_drivable >= 128, gt_drivable >= 128, mask)
        values["marking"].add(
            np.isin(pred_marking, (1, 2)), np.isin(gt_marking, (1, 2)), mask
        )
        values["stopline"].add(pred_marking == 3, gt_marking == 3, mask)


def sample_chamfer(mask_a, mask_b, overlap, resolution, limit=5000):
    a = np.asarray(mask_a, dtype=bool) & overlap
    b = np.asarray(mask_b, dtype=bool) & overlap
    if not np.any(a) or not np.any(b):
        return []
    distance_to_b = cv2.distanceTransform((~b).astype(np.uint8), cv2.DIST_L2, 3)
    distance_to_a = cv2.distanceTransform((~a).astype(np.uint8), cv2.DIST_L2, 3)
    values = np.concatenate((distance_to_b[a], distance_to_a[b])) * float(resolution)
    if len(values) > limit:
        values = values[:: int(np.ceil(len(values) / limit))]
    return values.astype(np.float32).tolist()


def serialize_metrics(store):
    result = {}
    for candidate, by_condition in store.items():
        key = "xmax{}_y{}".format(candidate[0], candidate[1])
        result[key] = {}
        for condition, by_region in sorted(by_condition.items()):
            result[key][condition] = {
                region: {
                    name: stats.result() for name, stats in values.items()
                }
                for region, values in sorted(by_region.items())
            }
    return result


def candidate_passes(candidate_key, metrics, args):
    reasons = []
    by_condition = metrics[candidate_key]
    for condition in ("sunny", "foggy"):
        if condition not in by_condition:
            reasons.append("missing condition {}".format(condition))
            continue
        for region in ("all", "outer_forward", "outer_lateral"):
            values = by_condition[condition][region]
            drivable = values["drivable"]
            if drivable["valid_pixels"] <= 0 or drivable["iou"] < args.drivable_iou_threshold:
                reasons.append(
                    "{} {} drivable_iou={:.4f}".format(
                        condition, region, drivable["iou"]
                    )
                )
            marking = values["marking"]
            if marking["target_positive"] >= args.minimum_marking_support:
                if marking["iou"] < args.marking_iou_threshold:
                    reasons.append(
                        "{} {} marking_iou={:.4f}".format(
                            condition, region, marking["iou"]
                        )
                    )
                if marking["recall"] < args.marking_recall_threshold:
                    reasons.append(
                        "{} {} marking_recall={:.4f}".format(
                            condition, region, marking["recall"]
                        )
                    )
    return not reasons, reasons


def main():
    args = parse_args()
    if args.max_groups is not None and args.max_groups <= 0:
        raise ValueError("max-groups must be positive")
    output_path = Path(args.output).expanduser().resolve()
    if output_path.exists():
        raise RuntimeError("refusing to overwrite {}".format(output_path))

    dataset = MoraiTwinLiteDataset(
        data_root=args.data_root,
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

    checkpoint_path = Path(args.checkpoint).expanduser().resolve()
    checkpoint = torch.load(str(checkpoint_path), map_location="cpu")
    task = task_from_checkpoint(checkpoint)
    if task.secondary_head != "road_marking":
        raise RuntimeError("range analysis requires the 4-class road-marking checkpoint")
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

    calibration_path = (
        REPO_ROOT / "src/perception/drivable_bev/config/cameras_v2.yaml"
    )
    _, cameras = load_calibration_snapshot(calibration_path)
    grid = BevGridSpec(-5.0, 30.0, -12.0, 12.0, 0.1)
    projectors = {
        view: CameraBevProjector(cameras[view], grid, 32.5, 0.1)
        for view in VIEWS
    }
    quality = build_quality_maps(projectors, grid)
    fusion = CameraBevFusion(quality, {"front": 1, "left": 2, "right": 3})
    candidates = sorted(
        [
            (x_max, y_half)
            for x_max in (30, 25, 20)
            for y_half in (12, 10, 8)
        ],
        key=lambda value: ((value[0] + 5) * value[1] * 2, value[0], value[1]),
        reverse=True,
    )
    store = new_metric_store(candidates)
    x, y = metric_masks(grid)
    chamfer = {"near_0_20m": [], "far_20_30m": []}
    stopline_errors = []
    valid_pixels_by_weather_view = defaultdict(
        lambda: defaultdict(lambda: {"frames": 0, "valid_pixels": 0})
    )
    started = time.perf_counter()

    with torch.no_grad():
        for group_index, ((run_id, frame_id), indices) in enumerate(complete, 1):
            samples = {view: dataset[indices[view]] for view in VIEWS}
            batch = torch.stack([samples[view]["image"] for view in VIEWS]).to(device)
            with torch.cuda.amp.autocast(enabled=device.type == "cuda"):
                logits = adapt_twinlite_outputs(model(batch), task.secondary_head)
                drivable_probability = torch.softmax(logits["drivable"].float(), dim=1)[:, 1]
                marking_probability = torch.softmax(logits["road_marking"].float(), dim=1)
            drivable_probability = drivable_probability.cpu().numpy()
            marking_probability = marking_probability.cpu().numpy()

            predicted = {}
            target = {}
            valid_by_view = {}
            per_view_marking = {}
            for index, view in enumerate(VIEWS):
                sample = samples[view]
                geometry = geometry_from_meta(sample["meta"]["geometry"])
                pred_drivable_native = restore_probability(
                    drivable_probability[index].astype(np.float32), geometry
                )
                pred_marking_native = restore_categorical_probabilities(
                    marking_probability[index].astype(np.float32), geometry
                )
                pred_class_native = np.argmax(pred_marking_native, axis=0).astype(np.uint8)
                pred_conf_native = np.max(pred_marking_native, axis=0)
                predicted[view] = projectors[view].project(
                    np.rint(pred_drivable_native * 255.0).astype(np.uint8),
                    pred_class_native,
                    np.rint(pred_conf_native * 255.0).astype(np.uint8),
                )

                gt_drivable_native = restore_class_ids(
                    sample["targets"]["drivable"].numpy().astype(np.uint8), geometry
                )
                gt_marking_native = restore_class_ids(
                    sample["targets"]["road_marking"].numpy().astype(np.uint8), geometry
                )
                target[view] = projectors[view].project(
                    gt_drivable_native * 255,
                    gt_marking_native,
                    np.full(gt_marking_native.shape, 255, dtype=np.uint8),
                )
                valid_native = restore_class_ids(
                    (
                        sample["valid_masks"]["drivable"]
                        & sample["valid_masks"]["road_marking"]
                    ).numpy().astype(np.uint8),
                    geometry,
                )
                valid_by_view[view] = warp_valid(projectors[view], valid_native)
                per_view_marking[view] = predicted[view].road_marking_class_id

            fused_prediction = fusion.fuse(predicted, valid_by_view=valid_by_view)
            fused_target = fusion.fuse(target, valid_by_view=valid_by_view)
            valid = (fused_prediction.coverage > 0) & (fused_target.coverage > 0)
            condition = weather(run_id)
            for view in VIEWS:
                summary = valid_pixels_by_weather_view[condition][view]
                summary["frames"] += 1
                summary["valid_pixels"] += int(
                    np.count_nonzero(valid_by_view[view])
                )
            for candidate in candidates:
                add_regions(
                    store,
                    candidate,
                    condition,
                    x,
                    y,
                    (
                        fused_prediction.drivable_probability,
                        fused_prediction.road_marking_class_id,
                    ),
                    (
                        fused_target.drivable_probability,
                        fused_target.road_marking_class_id,
                    ),
                    valid,
                )

            stack_quality = np.stack([quality[view] for view in VIEWS])
            overlap = np.count_nonzero(stack_quality > 0.0, axis=0) >= 2
            for first, second in (("front", "left"), ("front", "right")):
                pair_overlap = overlap & (quality[first] > 0.0) & (quality[second] > 0.0)
                first_mask = np.isin(per_view_marking[first], (1, 2))
                second_mask = np.isin(per_view_marking[second], (1, 2))
                chamfer["near_0_20m"].extend(
                    sample_chamfer(
                        first_mask,
                        second_mask,
                        pair_overlap & (x >= 0.0) & (x < 20.0),
                        grid.resolution_m,
                    )
                )
                chamfer["far_20_30m"].extend(
                    sample_chamfer(
                        first_mask,
                        second_mask,
                        pair_overlap & (x >= 20.0) & (x < 30.0),
                        grid.resolution_m,
                    )
                )

            pred_stop = fused_prediction.road_marking_class_id == 3
            gt_stop = fused_target.road_marking_class_id == 3
            if np.any(pred_stop & valid) and np.any(gt_stop & valid):
                stopline_errors.append(
                    abs(float(np.median(x[pred_stop & valid])) - float(np.median(x[gt_stop & valid])))
                )
            if args.log_interval and (
                group_index % args.log_interval == 0 or group_index == len(complete)
            ):
                print(
                    "range-eval group={}/{} run={} frame={}".format(
                        group_index, len(complete), run_id, frame_id
                    ),
                    flush=True,
                )

    metrics = serialize_metrics(store)
    selection = None
    decisions = {}
    for candidate in candidates:
        key = "xmax{}_y{}".format(candidate[0], candidate[1])
        passed, reasons = candidate_passes(key, metrics, args)
        decisions[key] = {"passed": passed, "reasons": reasons}
        if selection is None and passed:
            selection = candidate
    fallback_used = selection is None
    if fallback_used:
        selection = (20, 8)

    def distribution(values):
        array = np.asarray(values, dtype=np.float32)
        return {
            "count": int(array.size),
            "p50_m": float(np.percentile(array, 50.0)) if array.size else None,
            "p95_m": float(np.percentile(array, 95.0)) if array.size else None,
        }

    report = {
        "schema_version": "asmc-camera-bev-range-audit-1.0.0",
        "completed_at_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "dataset_version": args.dataset_version,
        "dataset_manifest_sha256": sha256(
            Path(args.data_root) / "dataset_versions" / args.dataset_version / (args.split + ".jsonl")
        ),
        "checkpoint": str(checkpoint_path),
        "checkpoint_sha256": sha256(checkpoint_path),
        "weights": state_key,
        "groups": len(complete),
        "incomplete_groups": incomplete,
        "views": list(VIEWS),
        "calibration": {
            "path": str(calibration_path),
            "sha256": sha256(calibration_path),
        },
        "ground_plane": cameras["front"].ground_plane.to_mapping(),
        "largest_grid": {
            "x_min_m": -5.0,
            "x_max_m": 30.0,
            "y_min_m": -12.0,
            "y_max_m": 12.0,
            "resolution_m": 0.1,
        },
        "thresholds": {
            "drivable_iou": args.drivable_iou_threshold,
            "white_yellow_iou": args.marking_iou_threshold,
            "white_yellow_recall": args.marking_recall_threshold,
            "minimum_marking_support": args.minimum_marking_support,
        },
        "candidate_decisions": decisions,
        "selected_grid": {
            "x_min_m": -5.0,
            "x_max_m": float(selection[0]),
            "y_min_m": -float(selection[1]),
            "y_max_m": float(selection[1]),
            "resolution_m": 0.1,
            "fallback_used": fallback_used,
        },
        "metrics": metrics,
        "overlap_chamfer": {name: distribution(values) for name, values in chamfer.items()},
        "stopline_longitudinal_error": distribution(stopline_errors),
        "valid_pixels_by_weather_view": {
            condition: {
                view: dict(values) for view, values in sorted(by_view.items())
            }
            for condition, by_view in sorted(valid_pixels_by_weather_view.items())
        },
        "elapsed_seconds": time.perf_counter() - started,
    }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary = output_path.with_name(output_path.name + ".part")
    with temporary.open("x", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.write("\n")
    os.replace(temporary, output_path)
    print(json.dumps(report["selected_grid"], sort_keys=True))
    print(output_path)


if __name__ == "__main__":
    main()
