#!/usr/bin/env python3
"""Load real MORAI samples and verify the two-head loss can backpropagate."""

import argparse
import os
import sys
from pathlib import Path

import torch
from torch.utils.data import DataLoader


REPO_ROOT = Path(__file__).resolve().parents[1]
PERCEPTION_SRC = REPO_ROOT / "src" / "perception"
sys.path.insert(0, str(PERCEPTION_SRC))

from twinlite_morai import (  # noqa: E402
    MaskedTwinLiteLoss,
    MoraiTwinLiteDataset,
    twinlite_training_step,
)


class SmokeTwin(torch.nn.Module):
    """Minimal two-head network used only to test the data/loss plumbing."""

    def __init__(self):
        super().__init__()
        self.stem = torch.nn.Conv2d(3, 8, kernel_size=3, padding=1)
        self.drivable = torch.nn.Conv2d(8, 2, kernel_size=1)
        self.lane = torch.nn.Conv2d(8, 2, kernel_size=1)

    def forward(self, image):
        features = torch.relu(self.stem(image))
        return self.drivable(features), self.lane(features)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", default=os.environ.get("ASMC_DATA", "/data"))
    parser.add_argument("--dataset-version", default="twinlite_morai_v1")
    parser.add_argument("--split", default="train", choices=("train", "val"))
    parser.add_argument("--batch-size", type=int, default=2)
    parser.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    return parser.parse_args()


def main():
    args = parse_args()
    dataset = MoraiTwinLiteDataset(
        data_root=args.data_root,
        dataset_version=args.dataset_version,
        split=args.split,
        max_samples=max(1, args.batch_size),
    )
    batch = next(iter(DataLoader(dataset, batch_size=args.batch_size, num_workers=0)))
    device = torch.device(args.device)
    model = SmokeTwin().to(device)
    batch["image"] = batch["image"].to(device)
    batch["targets"] = {
        name: value.to(device) for name, value in batch["targets"].items()
    }
    batch["valid_mask"] = batch["valid_mask"].to(device)
    outputs, losses = twinlite_training_step(model, batch, MaskedTwinLiteLoss())
    losses["loss"].backward()
    print(
        "OK device={} samples={} image={} drivable={} lane={} valid={} loss={:.6f}".format(
            device,
            len(dataset),
            tuple(batch["image"].shape),
            tuple(outputs["drivable"].shape),
            tuple(outputs["lane"].shape),
            int(batch["valid_mask"].sum().item()),
            float(losses["loss"].detach().cpu()),
        )
    )


if __name__ == "__main__":
    main()
