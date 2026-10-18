#!/usr/bin/env python3
"""Evaluate a frozen 2-head TwinLite checkpoint without updating weights."""

from __future__ import annotations

import argparse
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

import torch
from torch.utils.data import DataLoader


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "asmc_camera" / "src"))

from asmc_camera.training.twinlite import (  # noqa: E402
    MaskedTwinLiteLoss,
    MoraiTwinLiteDataset,
    adapt_twinlite_outputs,
    load_external_twinlite,
    move_batch_to_device,
    configure_twinlite_task,
    task_from_checkpoint,
    task_from_training_heads,
    ROAD_MARKING_TASK,
)
from asmc_camera.training.twinlite.batching import ShapeBatchSampler


HEADS = ("drivable", "lane")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-root", default=os.environ.get("ASMC_DATA", "/data"))
    parser.add_argument("--dataset-version", required=True)
    parser.add_argument("--split", action="append", choices=("train", "val", "test"), required=True)
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--twinlite-root", default=os.environ.get("TWINLITE_ROOT"))
    parser.add_argument("--config", default="medium", choices=("nano", "small", "medium", "large"))
    parser.add_argument("--batch-size", type=int, default=12)
    parser.add_argument("--num-workers", type=int, default=6)
    parser.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    parser.add_argument("--weights", default="auto", choices=("auto", "ema", "model"))
    parser.add_argument("--no-amp", action="store_true")
    parser.add_argument("--channels-last", action="store_true")
    parser.add_argument("--log-interval", type=int, default=50)
    parser.add_argument("--max-samples", type=int, help="Smoke-test limit per split")
    return parser.parse_args()


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _condition(run_id: str) -> Dict[str, str]:
    weather = "foggy" if "_foggy_" in run_id else "sunny" if "_sunny_" in run_id else "legacy"
    match = re.search(r"_sim([^_]+)_", run_id)
    return {"weather": weather, "sim_time": match.group(1) if match else "legacy"}


def _new_stats():
    return {
        head: {key: 0 for key in ("intersection", "union", "prediction_positive", "target_positive", "valid_pixels")}
        for head in HEADS
    }


def _add(stats, head, prediction, target, valid):
    prediction = prediction.bool() & valid
    target = target.bool() & valid
    stats[head]["intersection"] += int((prediction & target).sum().item())
    stats[head]["union"] += int((prediction | target).sum().item())
    stats[head]["prediction_positive"] += int(prediction.sum().item())
    stats[head]["target_positive"] += int(target.sum().item())
    stats[head]["valid_pixels"] += int(valid.sum().item())


def _finalize(stats):
    result = {}
    for head, values in stats.items():
        values = dict(values)
        values["iou"] = values["intersection"] / max(1, values["union"])
        result[head] = values
    result["mean_head_iou"] = sum(result[head]["iou"] for head in HEADS) / len(HEADS)
    return result


def _write_json(path: Path, value: Any) -> None:
    temporary = path.with_name(path.name + ".part")
    with temporary.open("w", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.write("\n")
    os.replace(temporary, path)


def main():
    args = parse_args()
    if args.batch_size <= 0 or args.num_workers < 0:
        raise ValueError("batch-size must be positive and num-workers non-negative")
    device = torch.device(args.device)
    if device.type == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("CUDA was requested but is unavailable")
    amp_enabled = device.type == "cuda" and not args.no_amp
    checkpoint_path = Path(args.checkpoint).expanduser().resolve()
    data_root = Path(args.data_root).expanduser().resolve()
    dataset_root = data_root / "dataset_versions" / args.dataset_version
    dataset_metadata_path = dataset_root / "dataset.json"
    if not dataset_metadata_path.is_file():
        raise RuntimeError(f"dataset metadata not found: {dataset_metadata_path}")
    with dataset_metadata_path.open("r", encoding="utf-8") as stream:
        dataset_metadata = json.load(stream)
    split_manifest_sha256 = {
        split: _sha256(dataset_root / (split + ".jsonl"))
        for split in args.split
    }
    checkpoint = torch.load(checkpoint_path, map_location="cpu")
    use_ema = args.weights == "ema" or (
        args.weights == "auto" and checkpoint.get("ema_state_dict") is not None
    )
    if use_ema and checkpoint.get("ema_state_dict") is None:
        raise RuntimeError("checkpoint has no EMA weights")
    state_key = "ema_state_dict" if use_ema else "model_state_dict"

    model, upstream_commit = load_external_twinlite(
        config=args.config, root=args.twinlite_root
    )
    if checkpoint.get("upstream_commit") != upstream_commit:
        raise RuntimeError("checkpoint and loaded TwinLite source commits differ")
    task = task_from_training_heads(dataset_metadata['training_heads'])
    if task_from_checkpoint(checkpoint) != task:
        raise RuntimeError('checkpoint and evaluation target classes differ; binary BDD output is not a 4-class prediction')
    configure_twinlite_task(model,task)
    model.load_state_dict(checkpoint[state_key])
    model = model.to(device).eval()
    if args.channels_last:
        model = model.to(memory_format=torch.channels_last)
    criterion = MaskedTwinLiteLoss(task_spec=task).to(device)

    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    all_results = {}
    run_started = time.perf_counter()
    if device.type == "cuda":
        torch.cuda.reset_peak_memory_stats(device)

    if task == ROAD_MARKING_TASK:
        # Share the exact multiclass/validity/condition aggregation used during
        # training; the old binary baseline report remains unchanged below.
        sys.path.insert(0,str(Path(__file__).resolve().parent))
        from train_twinlite import validate
        for split in args.split:
            dataset = MoraiTwinLiteDataset(data_root=args.data_root,
                dataset_version=args.dataset_version,split=split,max_samples=args.max_samples)
            if not len(dataset):
                all_results[split] = dict(sample_count=0,reason='empty split')
                continue
            loader = DataLoader(dataset,batch_sampler=ShapeBatchSampler(
                dataset.rows,args.batch_size,shuffle=False),num_workers=args.num_workers,
                pin_memory=device.type=='cuda',persistent_workers=args.num_workers>0)
            all_results[split] = validate(model,loader,criterion,task,device,
                amp_enabled,channels_last=args.channels_last,log_interval=args.log_interval)
        report = dict(schema_version='twinlite-frozen-marking-eval-1.0.0',
            completed_at_utc=datetime.now(timezone.utc).isoformat(),
            dataset_version=args.dataset_version,dataset_metadata_sha256=_sha256(dataset_metadata_path),
            dataset_split_manifest_sha256=split_manifest_sha256,
            checkpoint=str(checkpoint_path),checkpoint_sha256=_sha256(checkpoint_path),
            checkpoint_epoch=checkpoint.get('epoch'),weights=state_key,
            upstream_commit=upstream_commit,task_spec=task.as_dict(),
            device=str(device),amp=amp_enabled,batch_size=args.batch_size,
            training_performed=False,results=all_results,
            elapsed_seconds=time.perf_counter()-run_started)
        _write_json(output_dir/'metrics.json',report)
        print(json.dumps(report,ensure_ascii=False,sort_keys=True),flush=True)
        return

    for split in args.split:
        dataset = MoraiTwinLiteDataset(
            data_root=args.data_root,
            dataset_version=args.dataset_version,
            split=split,
            max_samples=args.max_samples,
        )
        loader = DataLoader(
            dataset,
            batch_size=args.batch_size,
            shuffle=False,
            num_workers=args.num_workers,
            pin_memory=device.type == "cuda",
            persistent_workers=args.num_workers > 0,
        )
        aggregate = _new_stats()
        by_run = defaultdict(_new_stats)
        by_view = defaultdict(_new_stats)
        by_weather = defaultdict(_new_stats)
        by_sim_time = defaultdict(_new_stats)
        losses = defaultdict(float)
        per_sample = []
        split_started = time.perf_counter()
        sample_count = 0
        with torch.no_grad():
            for batch_index, host_batch in enumerate(loader, 1):
                batch = move_batch_to_device(host_batch, device)
                if args.channels_last:
                    batch["image"] = batch["image"].contiguous(memory_format=torch.channels_last)
                with torch.cuda.amp.autocast(enabled=amp_enabled):
                    outputs = adapt_twinlite_outputs(model(batch["image"]))
                    result_losses = criterion(outputs, batch["targets"], batch["valid_masks"])
                batch_size = int(batch["image"].shape[0])
                sample_count += batch_size
                for name, value in result_losses.items():
                    losses[name] += float(value.item()) * batch_size
                predictions = {
                    head: (
                        output.argmax(dim=1)
                        if output.shape[1] == 2
                        else (output[:, 0] > 0).long()
                    )
                    for head, output in outputs.items()
                }
                for index in range(batch_size):
                    run_id = str(batch["meta"]["run_id"][index])
                    view = str(batch["meta"]["view"][index])
                    frame_value = batch["meta"]["frame_id"][index]
                    frame_id = int(frame_value.item() if hasattr(frame_value, "item") else frame_value)
                    condition = _condition(run_id)
                    sample_metrics = {}
                    for head in HEADS:
                        valid = batch["valid_masks"][head][index].bool()
                        prediction = predictions[head][index]
                        target = batch["targets"][head][index]
                        for stats in (
                            aggregate,
                            by_run[run_id],
                            by_view[view],
                            by_weather[condition["weather"]],
                            by_sim_time[condition["sim_time"]],
                        ):
                            _add(stats, head, prediction, target, valid)
                        intersection = int(((prediction.bool() & target.bool()) & valid).sum().item())
                        union = int(((prediction.bool() | target.bool()) & valid).sum().item())
                        sample_metrics[head + "_iou"] = intersection / max(1, union)
                    per_sample.append(
                        {
                            "sample_id": host_batch["meta"]["sample_id"][index],
                            "run_id": run_id,
                            "frame_id": frame_id,
                            "view": view,
                            **condition,
                            **sample_metrics,
                        }
                    )
                if args.log_interval and (
                    batch_index % args.log_interval == 0 or batch_index == len(loader)
                ):
                    print(
                        "eval split={} batch={}/{} samples={}".format(
                            split, batch_index, len(loader), sample_count
                        ),
                        flush=True,
                    )
        elapsed = time.perf_counter() - split_started
        per_sample_path = output_dir / (split + "_per_sample.jsonl")
        with per_sample_path.open("w", encoding="utf-8") as stream:
            for row in per_sample:
                stream.write(json.dumps(row, ensure_ascii=False, sort_keys=True) + "\n")
        all_results[split] = {
            "sample_count": sample_count,
            "elapsed_seconds": elapsed,
            "samples_per_second": sample_count / max(elapsed, 1e-9),
            "losses": {name: value / max(1, sample_count) for name, value in losses.items()},
            "overall": _finalize(aggregate),
            "by_run": {name: _finalize(stats) for name, stats in sorted(by_run.items())},
            "by_view": {name: _finalize(stats) for name, stats in sorted(by_view.items())},
            "by_weather": {name: _finalize(stats) for name, stats in sorted(by_weather.items())},
            "by_sim_time": {name: _finalize(stats) for name, stats in sorted(by_sim_time.items())},
            "per_sample_manifest": str(per_sample_path),
        }

    report = {
        "schema_version": "twinlite-frozen-baseline-eval-1.0.0",
        "completed_at_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "dataset_version": args.dataset_version,
        "dataset_metadata_sha256": _sha256(dataset_metadata_path),
        "dataset_split_manifest_sha256": split_manifest_sha256,
        "dataset_target_policy_ids": dataset_metadata.get("target_policy_ids", []),
        "splits": list(args.split),
        "checkpoint": str(checkpoint_path),
        "checkpoint_sha256": _sha256(checkpoint_path),
        "checkpoint_epoch": checkpoint.get("epoch"),
        "weights": state_key,
        "upstream_commit": upstream_commit,
        "device": str(device),
        "gpu": torch.cuda.get_device_name(device) if device.type == "cuda" else None,
        "amp": amp_enabled,
        "batch_size": args.batch_size,
        "num_workers": args.num_workers,
        "elapsed_seconds": time.perf_counter() - run_started,
        "peak_cuda_memory_bytes": torch.cuda.max_memory_allocated(device) if device.type == "cuda" else 0,
        "training_performed": False,
        "results": all_results,
    }
    _write_json(output_dir / "metrics.json", report)
    print(json.dumps(report, ensure_ascii=False, sort_keys=True), flush=True)


if __name__ == "__main__":
    main()
