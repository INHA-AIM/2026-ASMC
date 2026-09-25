#!/usr/bin/env python3
"""Compare actor-ignore (A) and actor-negative (B) checkpoints on common GT."""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import os
import re
import sys
import time
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict

import cv2
import numpy as np
import torch
from torch.utils.data import DataLoader


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "asmc_perception" / "src"))

from asmc_perception.training.twinlite import (  # noqa: E402
    MoraiTwinLiteDataset,
    adapt_twinlite_outputs,
    load_external_twinlite,
    move_batch_to_device,
)


HEADS = ("drivable", "lane")
MODELS = ("a", "b")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-root", default=os.environ.get("ASMC_DATA", "/data"))
    parser.add_argument("--dataset-a", required=True)
    parser.add_argument("--dataset-b", required=True)
    parser.add_argument("--split", default="val", choices=("train", "val", "test"))
    parser.add_argument("--checkpoint-a", required=True)
    parser.add_argument("--checkpoint-b", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--twinlite-root", default=os.environ.get("TWINLITE_ROOT"))
    parser.add_argument(
        "--config", default="medium", choices=("nano", "small", "medium", "large")
    )
    parser.add_argument("--batch-size", type=int, default=12)
    parser.add_argument("--num-workers", type=int, default=2)
    parser.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    parser.add_argument("--weights", default="auto", choices=("auto", "ema", "model"))
    parser.add_argument("--no-amp", action="store_true")
    parser.add_argument("--log-interval", type=int, default=25)
    parser.add_argument(
        "--actor-road-margin-px",
        type=int,
        default=8,
        help="Include an actor component when it lies within this many pixels of drivable GT",
    )
    parser.add_argument(
        "--min-overlay-actor-pixels",
        type=int,
        default=20,
        help="Minimum road-context actor pixels for automatic overlay selection",
    )
    return parser.parse_args()


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _read_jsonl(path: Path) -> list[Dict[str, Any]]:
    rows = []
    with path.open("r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            if line.strip():
                row = json.loads(line)
                if not isinstance(row, dict):
                    raise RuntimeError(f"expected JSON object at {path}:{line_number}")
                rows.append(row)
    return rows


def _condition(run_id: str) -> Dict[str, str]:
    weather = (
        "foggy"
        if "_foggy_" in run_id
        else "sunny"
        if "_sunny_" in run_id
        else "legacy"
    )
    match = re.search(r"_sim([^_]+)_", run_id)
    return {"weather": weather, "sim_time": match.group(1) if match else "legacy"}


def _load_checkpoint(checkpoint_path: Path, args, upstream_commit):
    checkpoint = torch.load(checkpoint_path, map_location="cpu")
    use_ema = args.weights == "ema" or (
        args.weights == "auto" and checkpoint.get("ema_state_dict") is not None
    )
    state_key = "ema_state_dict" if use_ema else "model_state_dict"
    if checkpoint.get(state_key) is None:
        raise RuntimeError(f"{checkpoint_path} has no {state_key}")
    if checkpoint.get("upstream_commit") != upstream_commit:
        raise RuntimeError(f"checkpoint/source commit mismatch: {checkpoint_path}")
    return checkpoint, state_key


def road_context_actor(actor, drivable, margin_px):
    """Return complete actor components touching or near visible drivable GT."""
    actor_u8 = actor.astype(np.uint8)
    count, labels = cv2.connectedComponents(actor_u8, connectivity=8)
    if count <= 1:
        return np.zeros_like(actor, dtype=bool)
    diameter = 2 * margin_px + 1
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (diameter, diameter))
    near_road = cv2.dilate(drivable.astype(np.uint8), kernel).astype(bool)
    selected_labels = np.unique(labels[actor & near_road])
    selected_labels = selected_labels[selected_labels != 0]
    return np.isin(labels, selected_labels)


def _confusion(prediction, target, valid):
    prediction = prediction & valid
    target = target & valid
    return {
        "tp": int((prediction & target).sum()),
        "fp": int((prediction & ~target & valid).sum()),
        "fn": int((~prediction & target).sum()),
        "tn": int((~prediction & ~target & valid).sum()),
        "prediction_positive": int(prediction.sum()),
        "target_positive": int(target.sum()),
        "valid_pixels": int(valid.sum()),
    }


def _new_bucket():
    return {
        "samples": 0,
        "heads": {
            model: {head: defaultdict(int) for head in HEADS} for model in MODELS
        },
        "actor": {
            "pixels": 0,
            "road_context_pixels": 0,
            "positive_samples": 0,
            "road_context_positive_samples": 0,
            "leak_pixels": {model: 0 for model in MODELS},
            "road_context_leak_pixels": {model: 0 for model in MODELS},
            "b_better_samples": 0,
            "b_worse_samples": 0,
            "tie_samples": 0,
        },
    }


def _add_to_bucket(bucket, sample):
    bucket["samples"] += 1
    for model in MODELS:
        for head in HEADS:
            for name, value in sample["counts"][model][head].items():
                bucket["heads"][model][head][name] += int(value)
    actor = bucket["actor"]
    actor["pixels"] += sample["actor_pixels"]
    actor["road_context_pixels"] += sample["road_context_actor_pixels"]
    actor["positive_samples"] += int(sample["actor_pixels"] > 0)
    actor["road_context_positive_samples"] += int(
        sample["road_context_actor_pixels"] > 0
    )
    for model in MODELS:
        actor["leak_pixels"][model] += sample["actor_leak_pixels"][model]
        actor["road_context_leak_pixels"][model] += sample[
            "road_context_actor_leak_pixels"
        ][model]
    if sample["road_context_actor_pixels"]:
        a_leak = sample["road_context_actor_leak_pixels"]["a"]
        b_leak = sample["road_context_actor_leak_pixels"]["b"]
        if b_leak < a_leak:
            actor["b_better_samples"] += 1
        elif b_leak > a_leak:
            actor["b_worse_samples"] += 1
        else:
            actor["tie_samples"] += 1


def _safe_rate(numerator, denominator):
    return float(numerator) / denominator if denominator else None


def _finalize_bucket(bucket):
    result = {"samples": bucket["samples"], "models": {}}
    for model in MODELS:
        model_result = {}
        for head in HEADS:
            values = dict(bucket["heads"][model][head])
            tp, fp, fn = values.get("tp", 0), values.get("fp", 0), values.get("fn", 0)
            values["iou"] = _safe_rate(tp, tp + fp + fn)
            values["precision"] = _safe_rate(tp, tp + fp)
            values["recall"] = _safe_rate(tp, tp + fn)
            if head == "drivable":
                values["false_hole_rate"] = _safe_rate(fn, tp + fn)
            model_result[head] = values
        model_result["mean_head_iou"] = sum(
            model_result[head]["iou"] or 0.0 for head in HEADS
        ) / len(HEADS)
        result["models"][model] = model_result
    actor = bucket["actor"]
    actor_result = {
        key: actor[key]
        for key in (
            "pixels",
            "road_context_pixels",
            "positive_samples",
            "road_context_positive_samples",
            "b_better_samples",
            "b_worse_samples",
            "tie_samples",
        )
    }
    for model in MODELS:
        leak = actor["leak_pixels"][model]
        road_leak = actor["road_context_leak_pixels"][model]
        actor_result[model] = {
            "leak_pixels": leak,
            "leak_rate": _safe_rate(leak, actor["pixels"]),
            "rejection_rate": _safe_rate(actor["pixels"] - leak, actor["pixels"]),
            "road_context_leak_pixels": road_leak,
            "road_context_leak_rate": _safe_rate(
                road_leak, actor["road_context_pixels"]
            ),
            "road_context_rejection_rate": _safe_rate(
                actor["road_context_pixels"] - road_leak,
                actor["road_context_pixels"],
            ),
        }
    result["actor"] = actor_result
    result["delta_b_minus_a"] = {
        "drivable_iou": result["models"]["b"]["drivable"]["iou"]
        - result["models"]["a"]["drivable"]["iou"],
        "lane_iou": result["models"]["b"]["lane"]["iou"]
        - result["models"]["a"]["lane"]["iou"],
        "drivable_false_hole_rate": result["models"]["b"]["drivable"][
            "false_hole_rate"
        ]
        - result["models"]["a"]["drivable"]["false_hole_rate"],
        "actor_leak_rate": (
            None
            if actor_result["a"]["leak_rate"] is None
            else actor_result["b"]["leak_rate"] - actor_result["a"]["leak_rate"]
        ),
        "road_context_actor_leak_rate": (
            None
            if actor_result["a"]["road_context_leak_rate"] is None
            else actor_result["b"]["road_context_leak_rate"]
            - actor_result["a"]["road_context_leak_rate"]
        ),
    }
    return result


def _load_actor_mask(data_root, row, expected_hw):
    relative = row.get("auxiliary_targets", {}).get("drivable_ignore")
    if not relative:
        raise RuntimeError(f"A sample lacks drivable_ignore: {row.get('sample_id')}")
    mask = cv2.imread(str(data_root / relative), cv2.IMREAD_GRAYSCALE)
    if mask is None:
        raise RuntimeError(f"failed to read actor mask: {data_root / relative}")
    if mask.shape != expected_hw:
        raise RuntimeError(
            f"actor mask shape {mask.shape} != expected {expected_hw}: {relative}"
        )
    return mask > 0


def _sample_metrics(predictions, targets, valid_masks, actor, margin_px):
    road_actor = road_context_actor(actor, targets["drivable"], margin_px)
    counts = {
        model: {
            head: _confusion(predictions[model][head], targets[head], valid_masks[head])
            for head in HEADS
        }
        for model in MODELS
    }
    return {
        "counts": counts,
        "actor_pixels": int(actor.sum()),
        "road_context_actor_pixels": int(road_actor.sum()),
        "actor_leak_pixels": {
            model: int((predictions[model]["drivable"] & actor).sum())
            for model in MODELS
        },
        "road_context_actor_leak_pixels": {
            model: int((predictions[model]["drivable"] & road_actor).sum())
            for model in MODELS
        },
    }


def _compact_sample(row, sample):
    result = {key: row[key] for key in ("sample_id", "run_id", "frame_id", "view")}
    result.update(_condition(str(row["run_id"])))
    result.update(
        {
            "actor_pixels": sample["actor_pixels"],
            "road_context_actor_pixels": sample["road_context_actor_pixels"],
            "actor_leak_rate": {},
            "road_context_actor_leak_rate": {},
            "drivable_iou": {},
            "drivable_false_hole_rate": {},
            "lane_iou": {},
        }
    )
    for model in MODELS:
        actor_pixels = sample["actor_pixels"]
        road_pixels = sample["road_context_actor_pixels"]
        result["actor_leak_rate"][model] = _safe_rate(
            sample["actor_leak_pixels"][model], actor_pixels
        )
        result["road_context_actor_leak_rate"][model] = _safe_rate(
            sample["road_context_actor_leak_pixels"][model], road_pixels
        )
        for head in HEADS:
            c = sample["counts"][model][head]
            iou = _safe_rate(c["tp"], c["tp"] + c["fp"] + c["fn"])
            result[head + "_iou"][model] = iou
        c = sample["counts"][model]["drivable"]
        result["drivable_false_hole_rate"][model] = _safe_rate(
            c["fn"], c["tp"] + c["fn"]
        )
    a_road = result["road_context_actor_leak_rate"]["a"]
    b_road = result["road_context_actor_leak_rate"]["b"]
    result["road_context_actor_leak_delta_b_minus_a"] = (
        None if a_road is None else b_road - a_road
    )
    return result


def _title(image, text):
    result = image.copy()
    cv2.rectangle(result, (0, 0), (result.shape[1] - 1, 30), (0, 0, 0), -1)
    cv2.putText(
        result,
        text,
        (8, 21),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.48,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    return result


def _blend(base, mask, color, alpha):
    result = base.astype(np.float32)
    result[mask] = result[mask] * (1.0 - alpha) + np.asarray(color) * alpha
    return np.clip(result, 0, 255).astype(np.uint8)


def _actor_contours(image, actor, color=(255, 40, 40), thickness=2):
    contours, _ = cv2.findContours(
        actor.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    result = image.copy()
    cv2.drawContours(result, contours, -1, color, thickness, cv2.LINE_AA)
    return result


def _prediction_overlay(rgb, drivable, lane, actor, label, actor_leak, false_hole):
    result = _blend(rgb, drivable, (30, 200, 30), 0.42)
    result = _blend(result, lane, (255, 230, 0), 0.82)
    result = _actor_contours(result, actor)
    leak_text = "n/a" if actor_leak is None else f"{100.0 * actor_leak:.1f}%"
    hole_text = "n/a" if false_hole is None else f"{100.0 * false_hole:.2f}%"
    return _title(result, f"{label}: actor leak={leak_text}, false hole={hole_text}")


def _render_panel(rgb, targets, valid_masks, predictions, actor, compact):
    rgb_actor = _title(_actor_contours(rgb, actor), "RGB: red contour=actor GT")
    gt = _blend(rgb, targets["drivable"] & valid_masks["drivable"], (30, 200, 30), 0.42)
    gt = _blend(gt, targets["lane"] & valid_masks["lane"], (255, 230, 0), 0.82)
    gt = _blend(gt, actor, (255, 35, 35), 0.72)
    gt = _title(gt, "COMMON GT: green=road, red=actor, yellow=lane")
    a = _prediction_overlay(
        rgb,
        predictions["a"]["drivable"] & valid_masks["drivable"],
        predictions["a"]["lane"] & valid_masks["lane"],
        actor,
        "A ignore",
        compact["road_context_actor_leak_rate"]["a"],
        compact["drivable_false_hole_rate"]["a"],
    )
    b = _prediction_overlay(
        rgb,
        predictions["b"]["drivable"] & valid_masks["drivable"],
        predictions["b"]["lane"] & valid_masks["lane"],
        actor,
        "B negative",
        compact["road_context_actor_leak_rate"]["b"],
        compact["drivable_false_hole_rate"]["b"],
    )
    a_drive = predictions["a"]["drivable"] & valid_masks["drivable"]
    b_drive = predictions["b"]["drivable"] & valid_masks["drivable"]
    difference = (rgb.astype(np.float32) * 0.32).astype(np.uint8)
    difference[a_drive & b_drive] = (25, 105, 25)
    difference[a_drive & ~b_drive] = (0, 220, 255)
    difference[~a_drive & b_drive] = (255, 0, 200)
    difference = _actor_contours(difference, actor)
    difference = _title(difference, "CHANGE: cyan=A only/B removed, magenta=B only")
    target = targets["drivable"] & valid_masks["drivable"]
    b_pred = predictions["b"]["drivable"] & valid_masks["drivable"]
    errors = (rgb.astype(np.float32) * 0.25).astype(np.uint8)
    errors[target & b_pred] = (30, 170, 30)
    errors[target & ~b_pred] = (255, 40, 40)
    errors[~target & b_pred & valid_masks["drivable"]] = (40, 100, 255)
    errors = _actor_contours(errors, actor, color=(255, 230, 0))
    errors = _title(errors, "B ERROR: red=false hole, blue=false drivable")
    return np.concatenate(
        (np.concatenate((rgb_actor, gt, a), axis=1), np.concatenate((b, difference, errors), axis=1)),
        axis=0,
    )


def _select_overlays(per_sample, min_actor_pixels):
    grouped = defaultdict(list)
    for row in per_sample:
        if row["road_context_actor_pixels"] >= min_actor_pixels:
            grouped[(row["weather"], row["view"])].append(row)
    selected = []
    for (weather, view), rows in sorted(grouped.items()):
        used = set()
        choices = [
            (
                "best_b_actor_rejection",
                max(rows, key=lambda row: -row["road_context_actor_leak_delta_b_minus_a"]),
            ),
            ("actor_rich", max(rows, key=lambda row: row["road_context_actor_pixels"])),
        ]
        worst = max(rows, key=lambda row: row["road_context_actor_leak_delta_b_minus_a"])
        if worst["road_context_actor_leak_delta_b_minus_a"] > 0:
            choices.append(("largest_b_regression", worst))
        for reason, row in choices:
            if row["sample_id"] in used:
                continue
            used.add(row["sample_id"])
            selected.append({"reason": reason, **row})
    return selected


def _write_json(path, value):
    temporary = path.with_name(path.name + ".part")
    with temporary.open("w", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.write("\n")
    os.replace(temporary, path)


def main():
    args = parse_args()
    if args.batch_size <= 0 or args.num_workers < 0 or args.actor_road_margin_px < 0:
        raise ValueError("invalid batch/worker/margin argument")
    data_root = Path(args.data_root).expanduser().resolve()
    a_manifest = data_root / "dataset_versions" / args.dataset_a / f"{args.split}.jsonl"
    b_manifest = data_root / "dataset_versions" / args.dataset_b / f"{args.split}.jsonl"
    a_rows = _read_jsonl(a_manifest)
    b_rows = _read_jsonl(b_manifest)
    a_by_sample = {str(row["sample_id"]): row for row in a_rows}
    b_sample_ids = [str(row["sample_id"]) for row in b_rows]
    if set(a_by_sample) != set(b_sample_ids) or len(a_rows) != len(b_rows):
        raise RuntimeError("A/B split sample IDs do not match exactly")

    device = torch.device(args.device)
    if device.type == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("CUDA requested but unavailable")
    checkpoint_a_path = Path(args.checkpoint_a).expanduser().resolve()
    checkpoint_b_path = Path(args.checkpoint_b).expanduser().resolve()
    model_template, upstream_commit = load_external_twinlite(
        config=args.config, root=args.twinlite_root
    )
    checkpoint_a, state_key_a = _load_checkpoint(
        checkpoint_a_path, args, upstream_commit
    )
    checkpoint_b, state_key_b = _load_checkpoint(
        checkpoint_b_path, args, upstream_commit
    )
    model_a = copy.deepcopy(model_template)
    model_b = copy.deepcopy(model_template)
    model_a.load_state_dict(checkpoint_a[state_key_a])
    model_b.load_state_dict(checkpoint_b[state_key_b])
    model_a = model_a.to(device).eval()
    model_b = model_b.to(device).eval()
    del model_template
    amp_enabled = device.type == "cuda" and not args.no_amp
    dataset = MoraiTwinLiteDataset(
        data_root=str(data_root), dataset_version=args.dataset_b, split=args.split
    )
    loader = DataLoader(
        dataset,
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=args.num_workers,
        pin_memory=device.type == "cuda",
        persistent_workers=args.num_workers > 0,
    )
    buckets = {
        "overall": _new_bucket(),
        "by_weather": defaultdict(_new_bucket),
        "by_view": defaultdict(_new_bucket),
        "by_weather_view": defaultdict(_new_bucket),
        "by_sim_time": defaultdict(_new_bucket),
    }
    per_sample = []
    started = time.perf_counter()
    if device.type == "cuda":
        torch.cuda.reset_peak_memory_stats(device)
    with torch.no_grad():
        for batch_index, host_batch in enumerate(loader, 1):
            batch = move_batch_to_device(host_batch, device)
            with torch.cuda.amp.autocast(enabled=amp_enabled):
                raw_a = adapt_twinlite_outputs(model_a(batch["image"]))
                raw_b = adapt_twinlite_outputs(model_b(batch["image"]))
            predictions_batch = {
                "a": {
                    head: logits.argmax(dim=1).cpu().numpy().astype(bool)
                    for head, logits in raw_a.items()
                },
                "b": {
                    head: logits.argmax(dim=1).cpu().numpy().astype(bool)
                    for head, logits in raw_b.items()
                },
            }
            batch_size = int(batch["image"].shape[0])
            for index in range(batch_size):
                sample_id = str(host_batch["meta"]["sample_id"][index])
                row = a_by_sample[sample_id]
                targets = {
                    head: host_batch["targets"][head][index].numpy().astype(bool)
                    for head in HEADS
                }
                valid_masks = {
                    head: host_batch["valid_masks"][head][index].numpy().astype(bool)
                    for head in HEADS
                }
                actor = _load_actor_mask(data_root, row, targets["drivable"].shape)
                predictions = {
                    model: {
                        head: predictions_batch[model][head][index] for head in HEADS
                    }
                    for model in MODELS
                }
                sample = _sample_metrics(
                    predictions, targets, valid_masks, actor, args.actor_road_margin_px
                )
                compact = _compact_sample(row, sample)
                per_sample.append(compact)
                condition = _condition(str(row["run_id"]))
                for bucket in (
                    buckets["overall"],
                    buckets["by_weather"][condition["weather"]],
                    buckets["by_view"][str(row["view"])],
                    buckets["by_weather_view"][
                        condition["weather"] + "/" + str(row["view"])
                    ],
                    buckets["by_sim_time"][condition["sim_time"]],
                ):
                    _add_to_bucket(bucket, sample)
            if args.log_interval and (
                batch_index % args.log_interval == 0 or batch_index == len(loader)
            ):
                print(
                    f"compare batch={batch_index}/{len(loader)} samples={len(per_sample)}",
                    flush=True,
                )

    output_dir = Path(args.output_dir).expanduser().resolve()
    overlay_dir = output_dir / "overlays_selected"
    overlay_dir.mkdir(parents=True, exist_ok=True)
    with (output_dir / "per_sample.jsonl").open("w", encoding="utf-8") as stream:
        for row in per_sample:
            stream.write(json.dumps(row, ensure_ascii=False, sort_keys=True) + "\n")
    selections = _select_overlays(per_sample, args.min_overlay_actor_pixels)
    index_by_sample = {
        str(row["sample_id"]): index for index, row in enumerate(dataset.rows)
    }
    for selection_index, selected in enumerate(selections, 1):
        dataset_index = index_by_sample[selected["sample_id"]]
        sample = dataset[dataset_index]
        image = sample["image"].unsqueeze(0).to(device)
        with torch.no_grad(), torch.cuda.amp.autocast(enabled=amp_enabled):
            outputs_a = adapt_twinlite_outputs(model_a(image))
            outputs_b = adapt_twinlite_outputs(model_b(image))
        predictions = {
            "a": {
                head: logits.argmax(dim=1)[0].cpu().numpy().astype(bool)
                for head, logits in outputs_a.items()
            },
            "b": {
                head: logits.argmax(dim=1)[0].cpu().numpy().astype(bool)
                for head, logits in outputs_b.items()
            },
        }
        rgb = (
            sample["image"].permute(1, 2, 0).numpy() * 255.0
        ).round().clip(0, 255).astype(np.uint8)
        targets = {
            head: sample["targets"][head].numpy().astype(bool) for head in HEADS
        }
        valid_masks = {
            head: sample["valid_masks"][head].numpy().astype(bool) for head in HEADS
        }
        actor = _load_actor_mask(
            data_root, a_by_sample[selected["sample_id"]], targets["drivable"].shape
        )
        panel = _render_panel(rgb, targets, valid_masks, predictions, actor, selected)
        filename = "{:02d}_{}_{}_{}_{}.jpg".format(
            selection_index,
            selected["weather"],
            selected["view"],
            selected["reason"],
            selected["frame_id"],
        )
        if not cv2.imwrite(str(overlay_dir / filename), cv2.cvtColor(panel, cv2.COLOR_RGB2BGR)):
            raise RuntimeError(f"failed to write {overlay_dir / filename}")
        selected["overlay"] = str(overlay_dir / filename)

    finalized = {
        "overall": _finalize_bucket(buckets["overall"]),
        "by_weather": {
            key: _finalize_bucket(value)
            for key, value in sorted(buckets["by_weather"].items())
        },
        "by_view": {
            key: _finalize_bucket(value)
            for key, value in sorted(buckets["by_view"].items())
        },
        "by_weather_view": {
            key: _finalize_bucket(value)
            for key, value in sorted(buckets["by_weather_view"].items())
        },
        "by_sim_time": {
            key: _finalize_bucket(value)
            for key, value in sorted(buckets["by_sim_time"].items())
        },
    }
    report = {
        "schema_version": "twinlite-actor-policy-ab-1.0.0",
        "completed_at_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "training_performed": False,
        "common_ground_truth": args.dataset_b,
        "actor_mask_source": args.dataset_a + ":auxiliary_targets.drivable_ignore",
        "split": args.split,
        "sample_count": len(per_sample),
        "actor_road_margin_px": args.actor_road_margin_px,
        "model_a": {
            "policy": "actor-ignore",
            "checkpoint": str(checkpoint_a_path),
            "checkpoint_sha256": _sha256(checkpoint_a_path),
            "checkpoint_epoch": checkpoint_a.get("epoch"),
            "weights": state_key_a,
        },
        "model_b": {
            "policy": "actor-negative",
            "checkpoint": str(checkpoint_b_path),
            "checkpoint_sha256": _sha256(checkpoint_b_path),
            "checkpoint_epoch": checkpoint_b.get("epoch"),
            "weights": state_key_b,
        },
        "dataset_manifests": {
            "a": {"path": str(a_manifest), "sha256": _sha256(a_manifest)},
            "b": {"path": str(b_manifest), "sha256": _sha256(b_manifest)},
        },
        "device": str(device),
        "amp": amp_enabled,
        "elapsed_seconds": time.perf_counter() - started,
        "peak_cuda_memory_bytes": (
            torch.cuda.max_memory_allocated(device) if device.type == "cuda" else 0
        ),
        "metrics": finalized,
        "overlay_selection": selections,
    }
    _write_json(output_dir / "metrics.json", report)
    print(
        f"OK compared {len(per_sample)} samples and wrote {len(selections)} overlays to {output_dir}",
        flush=True,
    )


if __name__ == "__main__":
    main()
