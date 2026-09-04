#!/usr/bin/env python3
"""Render RGB, ground truth and prediction panels from a TwinLite checkpoint."""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

import cv2
import numpy as np
import torch


REPO_ROOT = Path(__file__).resolve().parents[1]
PERCEPTION_SRC = REPO_ROOT / "src" / "perception"
sys.path.insert(0, str(PERCEPTION_SRC))

from twinlite_morai import (  # noqa: E402
    MoraiTwinLiteDataset,
    adapt_twinlite_outputs,
    load_external_twinlite,
)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", default=os.environ.get("ASMC_DATA", "/data"))
    parser.add_argument("--dataset-version", default="twinlite_morai_v1")
    parser.add_argument("--split", default="train", choices=("train", "val", "test"))
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--twinlite-root", default=os.environ.get("TWINLITE_ROOT"))
    parser.add_argument("--config", default="medium", choices=("nano", "small", "medium", "large"))
    parser.add_argument("--max-samples", type=int, default=12)
    parser.add_argument(
        "--run-id",
        action="append",
        help="Render only these run IDs; may be repeated.",
    )
    parser.add_argument(
        "--sample-id",
        action="append",
        help="Render exact sample IDs; may be repeated.",
    )
    parser.add_argument(
        "--weights",
        default="auto",
        choices=("auto", "ema", "model"),
        help="Use EMA weights when available (auto), require EMA, or use raw model weights.",
    )
    parser.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    return parser.parse_args()


def overlay(rgb, drivable, lane, valid_masks):
    output = rgb.copy().astype(np.float32)
    drivable = drivable.astype(bool) & valid_masks["drivable"].astype(bool)
    lane = lane.astype(bool) & valid_masks["lane"].astype(bool)
    output[drivable] = output[drivable] * 0.62 + np.array([30, 200, 30]) * 0.38
    output[lane] = output[lane] * 0.18 + np.array([255, 230, 0]) * 0.82
    return np.clip(output, 0, 255).astype(np.uint8)


def title(image, text):
    result = image.copy()
    cv2.rectangle(result, (0, 0), (result.shape[1] - 1, 30), (0, 0, 0), -1)
    cv2.putText(
        result,
        text,
        (8, 21),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    return result


def main():
    args = parse_args()
    device = torch.device(args.device)
    dataset = MoraiTwinLiteDataset(
        data_root=args.data_root,
        dataset_version=args.dataset_version,
        split=args.split,
    )
    if args.run_id:
        requested_runs = set(args.run_id)
        dataset.rows = [
            row for row in dataset.rows if str(row["run_id"]) in requested_runs
        ]
        missing_runs = requested_runs - {
            str(row["run_id"]) for row in dataset.rows
        }
        if missing_runs:
            raise RuntimeError(
                "requested run IDs are absent from split {}: {}".format(
                    args.split, sorted(missing_runs)
                )
            )
    if args.sample_id:
        requested_samples = set(args.sample_id)
        dataset.rows = [
            row for row in dataset.rows if str(row["sample_id"]) in requested_samples
        ]
        missing_samples = requested_samples - {
            str(row["sample_id"]) for row in dataset.rows
        }
        if missing_samples:
            raise RuntimeError(
                "requested sample IDs are absent from split {}: {}".format(
                    args.split, sorted(missing_samples)
                )
            )
    if args.max_samples is not None:
        if args.max_samples <= 0:
            raise ValueError("max-samples must be positive")
        dataset.rows = dataset.rows[: args.max_samples]
    model, _ = load_external_twinlite(config=args.config, root=args.twinlite_root)
    checkpoint = torch.load(args.checkpoint, map_location="cpu")
    use_ema = args.weights == "ema" or (
        args.weights == "auto" and checkpoint.get("ema_state_dict") is not None
    )
    if use_ema and checkpoint.get("ema_state_dict") is None:
        raise RuntimeError("checkpoint does not contain ema_state_dict")
    state_key = "ema_state_dict" if use_ema else "model_state_dict"
    model.load_state_dict(checkpoint[state_key])
    model = model.to(device).eval()
    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    with torch.no_grad():
        for sample in dataset:
            image = sample["image"].unsqueeze(0).to(device)
            outputs = adapt_twinlite_outputs(model(image))
            predictions = {
                head: logits.argmax(dim=1)[0].cpu().numpy().astype(np.uint8)
                for head, logits in outputs.items()
            }
            rgb = (
                sample["image"].permute(1, 2, 0).numpy() * 255.0
            ).round().clip(0, 255).astype(np.uint8)
            valid_masks = {
                head: sample["valid_masks"][head].numpy().astype(np.uint8)
                for head in ("drivable", "lane")
            }
            targets = {
                head: sample["targets"][head].numpy().astype(np.uint8)
                for head in ("drivable", "lane")
            }
            source = title(rgb, "RGB")
            ground_truth = title(
                overlay(rgb, targets["drivable"], targets["lane"], valid_masks),
                "GT: green=drivable yellow=lane",
            )
            prediction = title(
                overlay(
                    rgb,
                    predictions["drivable"],
                    predictions["lane"],
                    valid_masks,
                ),
                "PRED: green=drivable yellow=lane",
            )
            panel_rgb = np.concatenate((source, ground_truth, prediction), axis=1)
            panel_bgr = cv2.cvtColor(panel_rgb, cv2.COLOR_RGB2BGR)
            meta = sample["meta"]
            filename = "{}_{}_{:06d}.jpg".format(
                meta["run_id"], meta["view"], int(meta["frame_id"])
            )
            if not cv2.imwrite(str(output_dir / filename), panel_bgr):
                raise RuntimeError(f"failed to write preview {output_dir / filename}")
    print(
        f"OK wrote {len(dataset)} prediction panels to {output_dir} "
        f"using {state_key}"
    )


if __name__ == "__main__":
    main()
