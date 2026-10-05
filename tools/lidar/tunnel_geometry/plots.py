#!/usr/bin/env python3
"""Standalone BEV/side figures. Filter categories are never presented as GT."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

MODES = ("baseline", "policy_only", "geometry")


def plot(run, row, truth, output):
    points = np.fromfile(run / "clouds" / row["voxel_path"], dtype="<f4").reshape(-1, 4)
    fig, axes = plt.subplots(2, 3, figsize=(15, 8), constrained_layout=True)
    labels = np.asarray(truth["labels"]) if truth else None
    for column, mode in enumerate(MODES):
        actual = np.asarray(row[mode]["labels"])
        upstream = np.asarray(row["pre_wall_labels"]) != 0
        kept = actual == 0
        for a, (x, y, xlabel, ylabel) in zip(axes[:, column], ((0, 1, "sensor x [m]", "sensor y [m]"),
                                                               (0, 2, "sensor x [m]", "leveled z [m]"))):
            background = np.flatnonzero(upstream)[::4]
            a.scatter(points[background, x], points[background, y], s=.4, c="#aaaaaa", alpha=.15)
            if labels is not None:
                wall_kept = (labels == 1) & kept
                target_kept = (labels == 3) & kept
                target_deleted = (labels == 3) & (actual == 3)
                removed = (labels == 1) & (actual == 3)
                for mask, color, label in ((removed, "#cccccc", "known wall removed"),
                                          (wall_kept, "#d28e00", "known wall kept"),
                                          (target_kept, "#00975c", "known target kept"),
                                          (target_deleted, "#e22828", "known target removed by wall stage")):
                    indices = np.flatnonzero(mask)
                    a.scatter(points[indices, x], points[indices, y], s=2, color=color, alpha=.6, label=label)
            else:
                a.scatter(points[kept, x], points[kept, y], s=2, color="#be8f00", label="kept (GT unavailable)")
                marked = actual == 3
                a.scatter(points[marked, x], points[marked, y], s=.6, color="#aaaaaa", label="filter wall label (not GT)")
            for obj in row[mode]["objects"]:
                corners = np.asarray(obj["corners"])
                color = "#c940cf" if obj["removed"] else "#0068d4"
                if y == 1:
                    center = np.asarray(obj["center"])
                    angle = obj["yaw"]
                    half = np.asarray(obj["dimensions"][:2]) / 2
                    local = np.array([[-1,-1],[1,-1],[1,1],[-1,1],[-1,-1]]) * half
                    R = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
                    vertices = local @ R.T + center[:2]
                    a.plot(vertices[:, 0], vertices[:, 1], c=color, lw=1)
                else:
                    a.plot([corners[:, x].min(), corners[:, x].max(), corners[:, x].max(),
                            corners[:, x].min(), corners[:, x].min()],
                           [corners[:, y].min(), corners[:, y].min(), corners[:, y].max(),
                            corners[:, y].max(), corners[:, y].min()], c=color, lw=1)
            a.set_xlabel(xlabel); a.set_ylabel(ylabel); a.grid(alpha=.2)
            a.set_xlim(-20, 45)
            a.set_ylim((-5, 5) if y == 1 else (-2, 3))
        axes[0, column].set_title(mode + " | final objects=" + str(row[mode]["final_objects"]))
    axes[0, 0].legend(loc="lower left", fontsize=6)
    fig.suptitle(row["input_id"] + " | blue: retained OBB, magenta: rejected OBB")
    for suffix in ("png", "svg"):
        fig.savefig(output / (row["input_id"] + "." + suffix), dpi=160)
    plt.close(fig)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--truth", type=Path)
    parser.add_argument("--case", action="append", default=[])
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("output exists")
    args.output.mkdir(parents=True)
    truth = {r["input_id"]: r for r in map(json.loads, args.truth.open())} if args.truth else {}
    wanted = set(args.case) or {identity for identity, gt in truth.items()
                                if gt.get("key_gate") and identity.endswith("frame4")}
    for line in (args.run / "geometry.jsonl").open():
        # Avoid parsing hundreds of unselected full point-mask records.
        if not any('"input_id":"' + identity + '"' in line for identity in wanted):
            continue
        row = json.loads(line)
        plot(args.run, row, truth.get(row["input_id"]), args.output)
