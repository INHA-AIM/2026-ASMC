#!/usr/bin/env python3
"""Standalone Capture BEV/side GT-point overlays; no RViz/runtime topics."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

MODES = ("baseline", "policy_only", "geometry")


def plot(args):
    if args.output.exists():
        raise ValueError("output exists")
    args.output.mkdir(parents=True)
    truth = {r["capture_key"]: r for r in map(json.loads, (args.admission / "truth.jsonl").open())}
    source = {r["input_id"]: r for r in map(json.loads, (args.admission / "snapshots.jsonl").open())}
    wanted = set(args.case)
    found = set()
    for line in (args.run / "geometry.jsonl").open():
        if not any('"input_id":"' + key + '"' in line for key in wanted):
            continue
        r = json.loads(line)
        found.add(r["input_id"])
        s = source[r["input_id"]]
        t = truth[s["capture_key"]]
        raw = np.fromfile(args.admission / s["cloud_path"], dtype="<f4").reshape(-1, 4)
        xyz = raw[:, :3] @ np.asarray(r["baseline"]["leveling"]).reshape(3, 3).T
        ids = np.fromfile(args.admission / t["instance_path"], dtype="<i4")
        mapping = np.fromfile(args.run / "clouds" / (r["input_id"] + ".indices.bin"), dtype="<i4")
        roi = mapping >= 0
        pre = np.full(len(raw), -1, dtype=int)
        pre[roi] = np.asarray(r["pre_wall_labels"])[mapping[roi]]
        fig, axes = plt.subplots(2, 3, figsize=(16, 9), constrained_layout=True)
        for col, mode in enumerate(MODES):
            actual = np.full(len(raw), -1, dtype=int)
            actual[roi] = np.asarray(r[mode]["labels"])[mapping[roi]]
            for ax, (x, y, label) in zip(axes[:, col], ((0, 1, "sensor y [m]"), (0, 2, "leveled sensor z [m]"))):
                background = (ids == 0) & roi & (actual == 0)
                ax.scatter(xyz[background, x][::2], xyz[background, y][::2], s=1, c="#999999", alpha=.45,
                           label="non-instance kept (NOT GT wall)")
                upstream = (ids > 0) & roi & (pre != 0)
                deleted = (ids > 0) & roi & (pre == 0) & (actual == 3)
                ax.scatter(xyz[upstream, x], xyz[upstream, y], s=8, c="#ee9900", label="GT object lost upstream")
                ax.scatter(xyz[deleted, x], xyz[deleted, y], s=10, c="#d62728", label="GT object lost at wall stage")
                for key in sorted(set(ids) - {0}):
                    mask = (ids == key) & roi & (actual == 0)
                    ax.scatter(xyz[mask, x], xyz[mask, y], s=8, label="GT instance {} kept".format(key))
                for obj in r[mode]["objects"]:
                    c = np.asarray(obj["corners"])
                    color = "#ba4fb8" if obj["removed"] else "#1976d2"
                    if y == 1:
                        a = obj["yaw"]
                        R = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
                        v = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1], [-1, -1]]) * np.array(obj["dimensions"][:2]) / 2
                        v = v @ R.T + np.array(obj["center"][:2])
                    else:
                        v = np.array([[c[:, x].min(), c[:, y].min()], [c[:, x].max(), c[:, y].min()],
                                      [c[:, x].max(), c[:, y].max()], [c[:, x].min(), c[:, y].max()],
                                      [c[:, x].min(), c[:, y].min()]])
                    ax.plot(v[:, 0], v[:, 1], color=color, lw=.9, alpha=.7)
                ax.set_xlim(-32, 42)
                ax.set_ylim((-11, 11) if y == 1 else (-2.5, 3.5))
                ax.set_xlabel("sensor x [m]")
                ax.set_ylabel(label)
                ax.grid(alpha=.2)
            axes[0, col].set_title(mode + " | final objects=" + str(r[mode]["final_objects"]))
        axes[0, 0].legend(fontsize=6, loc="lower left")
        fig.suptitle("Capture {:03d}, controlled camera {} | blue: retained OBB, purple: rejected OBB\n"
                     "Instance point GT; background / filter wall labels are NOT wall ground truth".format(s["sequence"], s["controlled_camera_state"]))
        for suffix in ("png", "svg"):
            fig.savefig(args.output / (r["input_id"] + "." + suffix), dpi=135)
        plt.close(fig)
    if found != wanted:
        raise ValueError("missing requested figure case")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("admission", "run", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    parser.add_argument("--case", action="append", required=True)
    plot(parser.parse_args())
