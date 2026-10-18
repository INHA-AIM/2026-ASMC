#!/usr/bin/env python3
"""Render the stratified split/over-merge/boundary review sample (CLUSTER_EVENTS_*.json review_sample) to PNG.
Each image: left = BEV of all cluster points within 12 m of the GT (coloured per track id), GT boxes (green; the
event GT thick), V2 predicted boxes (dashed); right = side view along the event GT long axis (u vs z).
A per-event JSON with the point counts per (track, GT) is written next to the image.
usage: render_events.py --contract C --inputs INPUT_DIR --run RUN_DIR --events CLUSTER_EVENTS.json --out-dir OUT
"""
import sys

sys.dont_write_bytecode = True

import argparse, json, math, os  # noqa: E402
import matplotlib  # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import eval_lib as E  # noqa: E402


def draw_box(ax, cx, cy, l, w, yaw, **kw):
    cs = E.box_corners(cx, cy, l, w, yaw)
    xs, ys = [p[0] for p in cs] + [cs[0][0]], [p[1] for p in cs] + [cs[0][1]]
    ax.plot(xs, ys, **kw)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--contract", required=True)
    ap.add_argument("--inputs", required=True)
    ap.add_argument("--run", required=True)
    ap.add_argument("--events", required=True)
    ap.add_argument("--out-dir", required=True)
    a = ap.parse_args()
    c = json.load(open(a.contract))
    ev = json.load(open(a.events))["review_sample"]
    os.makedirs(a.out_dir, exist_ok=True)
    cache = {}
    for e in ev:
        sc = e["scene"]
        if sc not in cache:
            cache.clear()
            frames = pd.read_csv(os.path.join(a.inputs, sc, "frames.csv"))
            gt_all = pd.read_csv(os.path.join(a.inputs, sc, "gt.csv"))
            gt = E.segment_gt(gt_all[gt_all.kind == "gt"], frames, c)
            cache[sc] = (gt, E.load_pred(os.path.join(a.run, "out", sc + ".jsonl")),
                         E.load_points(os.path.join(a.run, "out", sc + ".pts.bin")))
        gt, pred, pts = cache[sc]
        k = e["first_idx"] + (e["n_frames"] - 1) // 2      # middle frame of the event
        g_all = gt[gt.idx == k].to_dict("records")
        g0 = next((g for g in g_all if g["seg"] == e["seg"]), None)
        if g0 is None or k not in pts:
            continue
        valid, T, trk = pts[k]
        R, t = T[:, :3], T[:, 3]
        L, S, ay = E.canon(g0["l"], g0["w"], g0["yaw"])
        fig, (ax, bx) = plt.subplots(1, 2, figsize=(13, 6))
        cmap = plt.get_cmap("tab20")
        boxes = E._gt_box_arrays(g_all)
        counts = {}
        for n, (tid, P) in enumerate(sorted(trk.items())):
            Pb = P @ R.T + t
            near = np.hypot(Pb[:, 0] - g0["x"], Pb[:, 1] - g0["y"]) < 12.0
            if not near.any():
                continue
            col = cmap(n % 20)
            ax.scatter(Pb[near, 0], Pb[near, 1], s=4, color=col, label="trk %d (%d)" % (tid, len(Pb)))
            cc, ss = math.cos(ay), math.sin(ay)
            u = (Pb[near, 0] - g0["x"]) * cc + (Pb[near, 1] - g0["y"]) * ss
            v = -(Pb[near, 0] - g0["x"]) * ss + (Pb[near, 1] - g0["y"]) * cc
            side = np.abs(v) < S / 2 + 1.0
            bx.scatter(u[side], Pb[near, 2][side], s=4, color=col)
            asg = E.assign_points(Pb, boxes, np.ones(len(boxes), int))
            counts[int(tid)] = {str(int(g_all[int(gi)]["track_id"])): int((asg == gi).sum()) for gi in np.unique(asg[asg >= 0])}
            counts[int(tid)]["_total"] = int(len(Pb))
        for g in g_all:
            if math.hypot(g["x"] - g0["x"], g["y"] - g0["y"]) < 14.0:
                draw_box(ax, g["x"], g["y"], g["l"], g["w"], g["yaw"], color="green",
                         lw=3 if g["seg"] == e["seg"] else 1)
        d, _ = pred.get(k, (None, {}))
        for o in (d or {}).get("local", []):
            if math.hypot(o["c"][0] - g0["x"], o["c"][1] - g0["y"]) < 14.0:
                draw_box(ax, o["c"][0], o["c"][1], o["l"], o["w"], o["yaw"], color="black", ls="--", lw=1)
                ax.text(o["c"][0], o["c"][1], str(o["id"]), fontsize=7)
        bx.add_patch(plt.Rectangle((-L / 2, g0["z"] - g0["h"] / 2), L, g0["h"], fill=False, color="green", lw=2))
        ax.set_aspect("equal"); ax.set_xlim(g0["x"] - 8, g0["x"] + 8); ax.set_ylim(g0["y"] - 8, g0["y"] + 8)
        ax.set_title("%s %s idx=%d kind=%s gt=%d (%s) r=%.1f m" % (e["review_id"], sc, k, e["kind"], e["gt_tid"], e["cls"], e["range"]))
        ax.legend(fontsize=6, loc="upper right")
        bx.set_title("side view along event GT long axis (|v|<S/2+1 m)"); bx.set_xlabel("u [m]"); bx.set_ylabel("z [m]")
        bx.set_aspect("equal")
        fig.tight_layout()
        fig.savefig(os.path.join(a.out_dir, e["review_id"] + ".png"), dpi=90)
        plt.close(fig)
        json.dump({"event": e, "frame": int(k), "points_per_track_per_gt": counts,
                   "gt_boxes": [{"tid": int(g["track_id"]), "cls": int(g["class_id"]), "x": g["x"], "y": g["y"], "z": g["z"],
                                 "l": g["l"], "w": g["w"], "h": g["h"]} for g in g_all
                                if math.hypot(g["x"] - g0["x"], g["y"] - g0["y"]) < 14.0]},
                  open(os.path.join(a.out_dir, e["review_id"] + ".json"), "w"), indent=1, default=float)
    print("rendered", len([f for f in os.listdir(a.out_dir) if f.endswith(".png")]))


if __name__ == "__main__":
    main()
