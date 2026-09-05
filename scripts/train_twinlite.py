#!/usr/bin/env python3
"""Train the pinned TwinLiteNet+ baseline on MORAI version manifests."""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import os
import random
import sys
import time
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict

import numpy as np
import torch
from torch.utils.data import DataLoader


REPO_ROOT = Path(__file__).resolve().parents[1]
PERCEPTION_SRC = REPO_ROOT / "src" / "perception"
sys.path.insert(0, str(PERCEPTION_SRC))

from twinlite_morai import (  # noqa: E402
    MaskedTwinLiteLoss,
    MoraiTwinLiteDataset,
    load_external_twinlite,
    move_batch_to_device,
    twinlite_training_step,
)


def _default_run_name() -> str:
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return f"twinlite_medium_morai_v1_{timestamp}"


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", default=os.environ.get("ASMC_DATA", "/data"))
    parser.add_argument("--dataset-version", default="twinlite_morai_v1")
    parser.add_argument("--twinlite-root", default=os.environ.get("TWINLITE_ROOT"))
    parser.add_argument("--config", default="medium", choices=("nano", "small", "medium", "large"))
    parser.add_argument("--epochs", type=int, default=30)
    parser.add_argument("--batch-size", type=int, default=12)
    parser.add_argument("--num-workers", type=int, default=6)
    parser.add_argument("--lr", type=float, default=5e-4)
    parser.add_argument("--weight-decay", type=float, default=5e-4)
    parser.add_argument("--poly-power", type=float, default=1.5)
    parser.add_argument("--seed", type=int, default=2026)
    parser.add_argument("--run-name", default=_default_run_name())
    parser.add_argument(
        "--output-root",
        default=str(REPO_ROOT / "artifacts/perception_eval/runs"),
    )
    parser.add_argument("--resume", help="Path to latest.pt from the same run")
    parser.add_argument(
        "--init-checkpoint",
        help="Load model weights only for a new run; optimizer/scheduler/EMA are reset",
    )
    parser.add_argument(
        "--init-weights",
        default="auto",
        choices=("auto", "ema", "model"),
        help="Weight state to load from --init-checkpoint",
    )
    parser.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    parser.add_argument("--no-amp", action="store_true")
    parser.add_argument("--no-ema", action="store_true")
    parser.add_argument("--ema-decay", type=float, default=0.9999)
    parser.add_argument("--max-train-samples", type=int)
    parser.add_argument("--max-val-samples", type=int)
    parser.add_argument("--skip-file-check", action="store_true")
    parser.add_argument("--channels-last", action="store_true")
    parser.add_argument("--log-interval", type=int, default=50)
    parser.add_argument(
        "--preflight-only",
        action="store_true",
        help="Load data/model/checkpoint and run one no-grad batch per split, then exit",
    )
    return parser.parse_args()


def seed_everything(seed: int) -> None:
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)


class ModelEMA:
    def __init__(self, model: torch.nn.Module, decay: float = 0.9999) -> None:
        self.model = copy.deepcopy(model).eval()
        self.decay = float(decay)
        self.updates = 0
        for parameter in self.model.parameters():
            parameter.requires_grad_(False)

    @torch.no_grad()
    def update(self, model: torch.nn.Module) -> None:
        self.updates += 1
        decay = self.decay * (1.0 - math.exp(-self.updates / 2000.0))
        source = model.state_dict()
        for name, value in self.model.state_dict().items():
            if value.dtype.is_floating_point:
                value.mul_(decay).add_(source[name].detach(), alpha=1.0 - decay)
            else:
                value.copy_(source[name])


def _atomic_torch_save(value: Dict[str, Any], path: Path) -> None:
    temporary = path.with_suffix(path.suffix + ".part")
    torch.save(value, temporary)
    os.replace(temporary, path)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _append_jsonl(path: Path, row: Dict[str, Any]) -> None:
    with path.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(row, ensure_ascii=False, sort_keys=True) + "\n")


def _new_confusion():
    return {head: defaultdict(lambda: [0, 0]) for head in ("drivable", "lane")}


def _update_confusion(confusion, outputs, batch):
    views = batch["meta"]["view"]
    for head in ("drivable", "lane"):
        valid = batch.get("valid_masks", {}).get(head, batch["valid_mask"]).bool()
        logits = outputs[head]
        prediction = (
            logits.argmax(dim=1)
            if logits.shape[1] == 2
            else (logits[:, 0] > 0).long()
        ).bool()
        target = batch["targets"][head].bool()
        for index, view in enumerate(views):
            selected_prediction = prediction[index] & valid[index]
            selected_target = target[index] & valid[index]
            intersection = int((selected_prediction & selected_target).sum().item())
            union = int((selected_prediction | selected_target).sum().item())
            confusion[head]["all"][0] += intersection
            confusion[head]["all"][1] += union
            confusion[head][str(view)][0] += intersection
            confusion[head][str(view)][1] += union


def _confusion_to_metrics(confusion):
    result = {}
    for head in ("drivable", "lane"):
        result[head] = {}
        for view in ("all", "front", "left", "right"):
            intersection, union = confusion[head][view]
            result[head][view] = {
                "intersection": intersection,
                "union": union,
                "iou": intersection / max(1, union),
            }
    return result


def _prepare_batch(host_batch, device, channels_last):
    batch = move_batch_to_device(host_batch, device)
    if channels_last:
        batch["image"] = batch["image"].contiguous(
            memory_format=torch.channels_last
        )
    return batch


@torch.no_grad()
def validate(
    model,
    loader,
    criterion,
    device,
    amp_enabled,
    channels_last=False,
    log_interval=0,
):
    model.eval()
    confusion = _new_confusion()
    totals = defaultdict(float)
    sample_count = 0
    started = time.perf_counter()
    for batch_index, host_batch in enumerate(loader, 1):
        batch = _prepare_batch(host_batch, device, channels_last)
        with torch.cuda.amp.autocast(enabled=amp_enabled):
            outputs, losses = twinlite_training_step(model, batch, criterion)
        batch_size = int(batch["image"].shape[0])
        sample_count += batch_size
        for name, value in losses.items():
            totals[name] += float(value.item()) * batch_size
        _update_confusion(confusion, outputs, batch)
        if log_interval > 0 and (
            batch_index % log_interval == 0 or batch_index == len(loader)
        ):
            elapsed = time.perf_counter() - started
            print(
                f"val batch={batch_index:04d}/{len(loader):04d} "
                f"samples={sample_count} sec={elapsed:.1f}",
                flush=True,
            )
    averaged_losses = {
        name: value / max(1, sample_count) for name, value in totals.items()
    }
    metrics = _confusion_to_metrics(confusion)
    mean_iou = 0.5 * (
        metrics["drivable"]["all"]["iou"] + metrics["lane"]["all"]["iou"]
    )
    return {
        "sample_count": sample_count,
        "losses": averaged_losses,
        "metrics": metrics,
        "mean_head_iou": mean_iou,
    }


def train_epoch(
    model,
    loader,
    criterion,
    optimizer,
    scaler,
    ema,
    device,
    amp_enabled,
    epoch,
    total_epochs,
    channels_last=False,
    log_interval=0,
):
    model.train()
    totals = defaultdict(float)
    sample_count = 0
    started = time.perf_counter()
    for batch_index, host_batch in enumerate(loader, 1):
        batch = _prepare_batch(host_batch, device, channels_last)
        optimizer.zero_grad(set_to_none=True)
        with torch.cuda.amp.autocast(enabled=amp_enabled):
            _, losses = twinlite_training_step(model, batch, criterion)
        scaler.scale(losses["loss"]).backward()
        scaler.step(optimizer)
        scaler.update()
        if ema is not None:
            ema.update(model)
        batch_size = int(batch["image"].shape[0])
        sample_count += batch_size
        for name, value in losses.items():
            totals[name] += float(value.detach().item()) * batch_size
        if log_interval > 0 and (
            batch_index % log_interval == 0 or batch_index == len(loader)
        ):
            elapsed = time.perf_counter() - started
            average_loss = totals["loss"] / max(1, sample_count)
            print(
                f"train epoch={epoch:03d}/{total_epochs} "
                f"batch={batch_index:04d}/{len(loader):04d} "
                f"samples={sample_count} loss={average_loss:.5f} sec={elapsed:.1f}",
                flush=True,
            )
    return {name: value / max(1, sample_count) for name, value in totals.items()}


def _checkpoint_state(
    args,
    epoch,
    model,
    ema,
    optimizer,
    scheduler,
    scaler,
    upstream_commit,
    best,
    validation,
    initialization,
    dataset_provenance,
):
    return {
        "schema_version": "twinlite-morai-checkpoint-1.0.0",
        "epoch": epoch,
        "args": vars(args),
        "upstream_commit": upstream_commit,
        "model_state_dict": model.state_dict(),
        "ema_state_dict": ema.model.state_dict() if ema is not None else None,
        "ema_updates": ema.updates if ema is not None else 0,
        "optimizer_state_dict": optimizer.state_dict(),
        "scheduler_state_dict": scheduler.state_dict(),
        "scaler_state_dict": scaler.state_dict(),
        "best": best,
        "validation": validation,
        "initialization": initialization,
        "dataset_provenance": dataset_provenance,
    }


@torch.no_grad()
def _preflight_split(model, loader, criterion, device, amp_enabled, channels_last):
    model.eval()
    host_batch = next(iter(loader))
    batch = _prepare_batch(host_batch, device, channels_last)
    with torch.cuda.amp.autocast(enabled=amp_enabled):
        outputs, losses = twinlite_training_step(model, batch, criterion)
    for name, value in losses.items():
        if not torch.isfinite(value):
            raise RuntimeError(f"non-finite preflight loss {name}: {value}")
    return {
        "sample_count": int(batch["image"].shape[0]),
        "image_shape": list(batch["image"].shape),
        "output_shapes": {
            head: list(output.shape) for head, output in outputs.items()
        },
        "losses": {name: float(value.item()) for name, value in losses.items()},
        "run_ids": sorted({str(value) for value in host_batch["meta"]["run_id"]}),
        "views": sorted({str(value) for value in host_batch["meta"]["view"]}),
    }


def main():
    args = parse_args()
    if (
        args.epochs <= 0
        or args.batch_size <= 0
        or args.num_workers < 0
        or args.log_interval < 0
    ):
        raise ValueError("epochs/batch-size must be positive and num-workers non-negative")
    if args.resume and args.init_checkpoint:
        raise ValueError("--resume and --init-checkpoint are mutually exclusive")
    seed_everything(args.seed)
    device = torch.device(args.device)
    if device.type == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("CUDA was requested but is not available")
    amp_enabled = device.type == "cuda" and not args.no_amp
    if device.type == "cuda":
        torch.backends.cudnn.benchmark = True

    output_dir = Path(args.output_root).expanduser().resolve() / args.run_name
    if output_dir.exists() and not args.resume:
        raise FileExistsError(
            f"output run already exists; choose a new --run-name: {output_dir}"
        )
    output_dir.mkdir(parents=True, exist_ok=True)
    history_path = output_dir / "history.jsonl"

    data_root = Path(args.data_root).expanduser().resolve()
    dataset_root = data_root / "dataset_versions" / args.dataset_version
    dataset_metadata_path = dataset_root / "dataset.json"
    dataset_provenance = {
        "dataset_version": args.dataset_version,
        "dataset_metadata_sha256": _sha256(dataset_metadata_path),
        "split_manifest_sha256": {
            split: _sha256(dataset_root / (split + ".jsonl"))
            for split in ("train", "val")
        },
    }

    train_dataset = MoraiTwinLiteDataset(
        data_root=args.data_root,
        dataset_version=args.dataset_version,
        split="train",
        verify_files=not args.skip_file_check,
        max_samples=args.max_train_samples,
    )
    val_dataset = MoraiTwinLiteDataset(
        data_root=args.data_root,
        dataset_version=args.dataset_version,
        split="val",
        verify_files=not args.skip_file_check,
        max_samples=args.max_val_samples,
    )
    generator = torch.Generator().manual_seed(args.seed)
    loader_options = {
        "batch_size": args.batch_size,
        "num_workers": args.num_workers,
        "pin_memory": device.type == "cuda",
        "persistent_workers": args.num_workers > 0,
    }
    train_loader = DataLoader(
        train_dataset, shuffle=True, generator=generator, **loader_options
    )
    val_loader = DataLoader(val_dataset, shuffle=False, **loader_options)

    model, upstream_commit = load_external_twinlite(
        config=args.config, root=args.twinlite_root
    )
    model = model.to(device)
    if args.channels_last:
        model = model.to(memory_format=torch.channels_last)
    criterion = MaskedTwinLiteLoss().to(device)
    optimizer = torch.optim.AdamW(
        model.parameters(), lr=args.lr, weight_decay=args.weight_decay
    )
    scheduler = torch.optim.lr_scheduler.LambdaLR(
        optimizer,
        lr_lambda=lambda epoch: max(0.0, 1.0 - epoch / args.epochs)
        ** args.poly_power,
    )
    scaler = torch.cuda.amp.GradScaler(enabled=amp_enabled)
    ema = None if args.no_ema else ModelEMA(model, decay=args.ema_decay)
    start_epoch = 1
    best = {"mean": -1.0, "drivable": -1.0, "lane": -1.0}
    initialization = {"mode": "random"}

    if args.resume:
        checkpoint = torch.load(args.resume, map_location=device)
        if checkpoint["upstream_commit"] != upstream_commit:
            raise RuntimeError("resume checkpoint uses a different upstream commit")
        model.load_state_dict(checkpoint["model_state_dict"])
        optimizer.load_state_dict(checkpoint["optimizer_state_dict"])
        scheduler.load_state_dict(checkpoint["scheduler_state_dict"])
        scaler.load_state_dict(checkpoint["scaler_state_dict"])
        best.update(checkpoint.get("best", {}))
        if ema is not None and checkpoint.get("ema_state_dict") is not None:
            ema.model.load_state_dict(checkpoint["ema_state_dict"])
            ema.updates = int(checkpoint.get("ema_updates", 0))
        start_epoch = int(checkpoint["epoch"]) + 1
        initialization = checkpoint.get(
            "initialization",
            {"mode": "resumed_legacy_checkpoint", "path": str(args.resume)},
        )
    elif args.init_checkpoint:
        initialization_path = Path(args.init_checkpoint).expanduser().resolve()
        checkpoint = torch.load(initialization_path, map_location="cpu")
        if checkpoint.get("upstream_commit") != upstream_commit:
            raise RuntimeError(
                "initialization checkpoint uses a different upstream commit"
            )
        use_ema = args.init_weights == "ema" or (
            args.init_weights == "auto"
            and checkpoint.get("ema_state_dict") is not None
        )
        if use_ema and checkpoint.get("ema_state_dict") is None:
            raise RuntimeError("initialization checkpoint has no EMA weights")
        state_key = "ema_state_dict" if use_ema else "model_state_dict"
        if checkpoint.get(state_key) is None:
            raise RuntimeError(
                f"initialization checkpoint has no {state_key}"
            )
        model.load_state_dict(checkpoint[state_key])
        if ema is not None:
            ema.model.load_state_dict(model.state_dict())
            ema.updates = 0
        source_args = checkpoint.get("args") or {}
        initialization = {
            "mode": "checkpoint_weights_only",
            "path": str(initialization_path),
            "sha256": _sha256(initialization_path),
            "checkpoint_epoch": checkpoint.get("epoch"),
            "weights": state_key,
            "source_dataset_version": source_args.get("dataset_version"),
            "optimizer_reset": True,
            "scheduler_reset": True,
            "ema_reset_from_loaded_model": ema is not None,
        }

    run_config = {
        **vars(args),
        "output_dir": str(output_dir),
        "device_resolved": str(device),
        "gpu": torch.cuda.get_device_name(device) if device.type == "cuda" else None,
        "torch_version": torch.__version__,
        "upstream_commit": upstream_commit,
        "model_parameters": sum(parameter.numel() for parameter in model.parameters()),
        "train_samples": len(train_dataset),
        "val_samples": len(val_dataset),
        "spatial_weighting": "uniform within valid_mask; padding weight zero",
        "augmentation": "none (first MORAI baseline)",
        "initialization": initialization,
        "dataset_provenance": dataset_provenance,
    }
    with (output_dir / "config.json").open("w", encoding="utf-8") as stream:
        json.dump(run_config, stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.write("\n")

    print(json.dumps(run_config, ensure_ascii=False, sort_keys=True), flush=True)
    if args.preflight_only:
        report = {
            "schema_version": "twinlite-training-preflight-1.0.0",
            "completed_at_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "training_performed": False,
            "initialization": initialization,
            "dataset_provenance": dataset_provenance,
            "train": _preflight_split(
                model,
                train_loader,
                criterion,
                device,
                amp_enabled,
                args.channels_last,
            ),
            "val": _preflight_split(
                model,
                val_loader,
                criterion,
                device,
                amp_enabled,
                args.channels_last,
            ),
        }
        with (output_dir / "preflight.json").open("w", encoding="utf-8") as stream:
            json.dump(report, stream, ensure_ascii=False, indent=2, sort_keys=True)
            stream.write("\n")
        print(json.dumps(report, ensure_ascii=False, sort_keys=True), flush=True)
        return
    if device.type == "cuda":
        torch.cuda.reset_peak_memory_stats(device)
    started = time.perf_counter()
    for epoch in range(start_epoch, args.epochs + 1):
        epoch_started = time.perf_counter()
        epoch_lr = optimizer.param_groups[0]["lr"]
        train_losses = train_epoch(
            model,
            train_loader,
            criterion,
            optimizer,
            scaler,
            ema,
            device,
            amp_enabled,
            epoch,
            args.epochs,
            channels_last=args.channels_last,
            log_interval=args.log_interval,
        )
        evaluation_model = ema.model if ema is not None else model
        validation = validate(
            evaluation_model,
            val_loader,
            criterion,
            device,
            amp_enabled,
            channels_last=args.channels_last,
            log_interval=args.log_interval,
        )
        scheduler.step()
        da_iou = validation["metrics"]["drivable"]["all"]["iou"]
        lane_iou = validation["metrics"]["lane"]["all"]["iou"]
        mean_iou = validation["mean_head_iou"]
        row = {
            "epoch": epoch,
            "lr": epoch_lr,
            "train_losses": train_losses,
            "validation": validation,
            "epoch_seconds": time.perf_counter() - epoch_started,
        }
        _append_jsonl(history_path, row)
        candidates = {
            "mean": mean_iou,
            "drivable": da_iou,
            "lane": lane_iou,
        }
        improved = []
        for name, score in candidates.items():
            if score > best[name]:
                best[name] = score
                improved.append(name)
        state = _checkpoint_state(
            args,
            epoch,
            model,
            ema,
            optimizer,
            scheduler,
            scaler,
            upstream_commit,
            best,
            validation,
            initialization,
            dataset_provenance,
        )
        _atomic_torch_save(state, output_dir / "latest.pt")
        for name in improved:
            _atomic_torch_save(state, output_dir / f"best_{name}.pt")
        print(
            f"epoch={epoch:03d}/{args.epochs} lr={row['lr']:.7f} "
            f"train={train_losses['loss']:.5f} val={validation['losses']['loss']:.5f} "
            f"DA={da_iou:.4f} lane={lane_iou:.4f} mean={mean_iou:.4f} "
            f"sec={row['epoch_seconds']:.1f}",
            flush=True,
        )

    result = {
        "completed_at_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "epochs_completed": args.epochs,
        "elapsed_seconds": time.perf_counter() - started,
        "best": best,
        "peak_cuda_memory_bytes": torch.cuda.max_memory_allocated(device)
        if device.type == "cuda"
        else 0,
        "output_dir": str(output_dir),
    }
    with (output_dir / "result.json").open("w", encoding="utf-8") as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.write("\n")
    print(json.dumps(result, ensure_ascii=False, sort_keys=True), flush=True)


if __name__ == "__main__":
    main()
