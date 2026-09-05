#!/usr/bin/env python3
"""Run a reproducible TwinLiteNet+ tiny-overfit on the MORAI manifest."""

from __future__ import annotations

import argparse
import json
import os
import random
import sys
import time
from pathlib import Path
from typing import Dict

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


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", default=os.environ.get("ASMC_DATA", "/data"))
    parser.add_argument("--dataset-version", default="twinlite_morai_v1")
    parser.add_argument("--split", default="train", choices=("train", "val"))
    parser.add_argument("--twinlite-root", default=os.environ.get("TWINLITE_ROOT"))
    parser.add_argument("--config", default="medium", choices=("nano", "small", "medium", "large"))
    parser.add_argument("--max-samples", type=int, default=12)
    parser.add_argument("--epochs", type=int, default=12)
    parser.add_argument("--batch-size", type=int, default=3)
    parser.add_argument("--num-workers", type=int, default=0)
    parser.add_argument("--lr", type=float, default=5e-4)
    parser.add_argument("--weight-decay", type=float, default=5e-4)
    parser.add_argument("--seed", type=int, default=2026)
    parser.add_argument(
        "--output-dir",
        default=str(REPO_ROOT / "artifacts/perception_eval/runs/twinlite_tiny_smoke"),
    )
    parser.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    parser.add_argument("--no-amp", action="store_true")
    return parser.parse_args()


def seed_everything(seed: int) -> None:
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)


def _head_iou(logits: torch.Tensor, target: torch.Tensor, valid: torch.Tensor) -> Dict[str, int]:
    prediction = logits.argmax(dim=1) if logits.shape[1] == 2 else (logits[:, 0] > 0).long()
    prediction = prediction.bool() & valid
    target = target.bool() & valid
    return {
        "intersection": int((prediction & target).sum().item()),
        "union": int((prediction | target).sum().item()),
    }


@torch.no_grad()
def evaluate(model, loader, criterion, device):
    model.eval()
    total_loss = 0.0
    sample_count = 0
    counts = {
        "drivable": {"intersection": 0, "union": 0},
        "lane": {"intersection": 0, "union": 0},
    }
    for host_batch in loader:
        batch = move_batch_to_device(host_batch, device)
        outputs, losses = twinlite_training_step(model, batch, criterion)
        batch_size = int(batch["image"].shape[0])
        total_loss += float(losses["loss"].item()) * batch_size
        sample_count += batch_size
        for head in ("drivable", "lane"):
            current = _head_iou(
                outputs[head], batch["targets"][head], batch["valid_mask"]
            )
            counts[head]["intersection"] += current["intersection"]
            counts[head]["union"] += current["union"]
    return {
        "loss": total_loss / max(1, sample_count),
        "drivable_iou": counts["drivable"]["intersection"]
        / max(1, counts["drivable"]["union"]),
        "lane_iou": counts["lane"]["intersection"]
        / max(1, counts["lane"]["union"]),
    }


def train_epoch(model, loader, criterion, optimizer, scaler, device, amp_enabled):
    model.train()
    total_loss = 0.0
    sample_count = 0
    for host_batch in loader:
        batch = move_batch_to_device(host_batch, device)
        optimizer.zero_grad(set_to_none=True)
        with torch.cuda.amp.autocast(enabled=amp_enabled):
            _, losses = twinlite_training_step(model, batch, criterion)
        scaler.scale(losses["loss"]).backward()
        scaler.step(optimizer)
        scaler.update()
        batch_size = int(batch["image"].shape[0])
        total_loss += float(losses["loss"].detach().item()) * batch_size
        sample_count += batch_size
    return total_loss / max(1, sample_count)


def main():
    args = parse_args()
    if args.max_samples <= 0 or args.epochs <= 0 or args.batch_size <= 0:
        raise ValueError("max-samples, epochs and batch-size must be positive")
    seed_everything(args.seed)
    device = torch.device(args.device)
    if device.type == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("CUDA was requested but is not available")

    dataset = MoraiTwinLiteDataset(
        data_root=args.data_root,
        dataset_version=args.dataset_version,
        split=args.split,
        max_samples=args.max_samples,
    )
    evaluation_loader = DataLoader(
        dataset,
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=args.num_workers,
        pin_memory=device.type == "cuda",
    )
    generator = torch.Generator().manual_seed(args.seed)
    training_loader = DataLoader(
        dataset,
        batch_size=args.batch_size,
        shuffle=True,
        generator=generator,
        num_workers=args.num_workers,
        pin_memory=device.type == "cuda",
    )

    model, upstream_commit = load_external_twinlite(
        config=args.config, root=args.twinlite_root
    )
    model = model.to(device)
    criterion = MaskedTwinLiteLoss().to(device)
    optimizer = torch.optim.AdamW(
        model.parameters(), lr=args.lr, weight_decay=args.weight_decay
    )
    amp_enabled = device.type == "cuda" and not args.no_amp
    scaler = torch.cuda.amp.GradScaler(enabled=amp_enabled)

    started = time.perf_counter()
    before = evaluate(model, evaluation_loader, criterion, device)
    history = []
    print(
        "before loss={loss:.6f} DA_IoU={drivable_iou:.4f} lane_IoU={lane_iou:.4f}".format(
            **before
        ),
        flush=True,
    )
    for epoch in range(1, args.epochs + 1):
        train_loss = train_epoch(
            model,
            training_loader,
            criterion,
            optimizer,
            scaler,
            device,
            amp_enabled,
        )
        metrics = evaluate(model, evaluation_loader, criterion, device)
        row = {"epoch": epoch, "train_loss": train_loss, **metrics}
        history.append(row)
        print(
            "epoch={epoch:03d} train={train_loss:.6f} eval={loss:.6f} "
            "DA_IoU={drivable_iou:.4f} lane_IoU={lane_iou:.4f}".format(**row),
            flush=True,
        )

    after = history[-1]
    elapsed = time.perf_counter() - started
    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    result = {
        "kind": "tiny-overfit-smoke",
        "dataset_version": args.dataset_version,
        "split": args.split,
        "sample_count": len(dataset),
        "config": args.config,
        "upstream_commit": upstream_commit,
        "device": str(device),
        "gpu": torch.cuda.get_device_name(device) if device.type == "cuda" else None,
        "amp": amp_enabled,
        "epochs": args.epochs,
        "batch_size": args.batch_size,
        "learning_rate": args.lr,
        "before": before,
        "after": after,
        "history": history,
        "elapsed_seconds": elapsed,
        "loss_decreased": after["loss"] < before["loss"],
    }
    with (output_dir / "metrics.json").open("w", encoding="utf-8") as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.write("\n")
    torch.save(
        {
            "model_state_dict": model.state_dict(),
            "optimizer_state_dict": optimizer.state_dict(),
            "result": result,
        },
        output_dir / "checkpoint.pt",
    )
    print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True), flush=True)
    if not result["loss_decreased"]:
        raise RuntimeError("tiny-overfit loss did not decrease")


if __name__ == "__main__":
    main()
