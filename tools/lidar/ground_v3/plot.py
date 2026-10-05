#!/usr/bin/env python3
"""BEV/side review for the identical V3 scan and its three ground masks."""
import argparse
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ground_live_review"))
from metrics import CLASSES, ACTORS


def find(path, identity):
    with path.open() as stream:
        for text in stream:
            item = json.loads(text)
            if item["input_id"] == identity:
                return item
    raise KeyError(identity)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument("result", type=Path)
    parser.add_argument("input_id")
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    source = find(args.input / "snapshots.jsonl", args.input_id)
    result = find(args.result, args.input_id)
    points = np.fromfile(args.input / source["cloud_path"], dtype="<f4").reshape(-1, 4)
    label_path = args.input / "labels" / (args.input_id + ".npz")
    actor = np.zeros(len(points), bool)
    if label_path.exists():
        with np.load(label_path) as labels:
            actor = np.isin(labels["semantic"],
                            [key for key, name in CLASSES.items() if name in ACTORS]) & (labels["instance"] != 0)
    fig, axes = plt.subplots(2, 3, figsize=(15, 8), constrained_layout=True)
    for column, method in enumerate(("jaeho", "patchwork", "linefit")):
        mask = np.asarray(result["methods"][method]["mask"], dtype=np.uint8)
        if len(mask) != len(points):
            raise ValueError("source-point lineage mismatch")
        selections = ((mask == 1, "#a6a6a6", "removed as ground"),
                      ((mask == 2) & ~actor, "#21834d", "retained background"),
                      ((mask == 2) & actor, "#1468c0", "retained actor"),
                      ((mask == 1) & actor, "#e13949", "removed actor"),
                      (mask == 3, "#a844ae", "unclassified"))
        for row, ordinate in enumerate((1, 2)):
            ax = axes[row, column]
            for selected, color, label in selections:
                ax.scatter(points[selected, 0], points[selected, ordinate],
                           s=1, c=color, label=label, rasterized=True)
            ax.set(xlabel="sensor x [m]", ylabel=("sensor y [m]" if row == 0 else "sensor z [m]"),
                   title=f"{method}: {'BEV' if row == 0 else 'side'}")
            ax.grid(alpha=.2)
            ax.set_xlim(-30, 50)
            ax.set_ylim((-20, 20) if row == 0 else (-3, 5))
    axes[0, 0].legend(fontsize=7, markerscale=3)
    fig.suptitle(args.input_id + (" / observed point semantic+instance GT" if label_path.exists()
                                  else " / unlabelled; color is algorithm output, not GT"))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=140)
    plt.close(fig)


if __name__ == "__main__":
    main()
