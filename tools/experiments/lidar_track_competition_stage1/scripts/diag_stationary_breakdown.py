#!/usr/bin/env python3
"""Breakdown of stationary-object false speed in one arm's frozen-evaluator samples (read-only).
usage: python3 -B diag_stationary_breakdown.py RUN_DIR INPUTS_DIR OUT.json
GT stationary = |GT ground velocity| < 0.2 m/s; false moving = |pred vb| > 0.5 m/s; TP rows with kfc >= 2.
"""
import json, math, os, sys
import numpy as np
import pandas as pd
sys.dont_write_bytecode = True

run, inputs, out = sys.argv[1:4]
rows = []
for f in sorted(os.listdir(os.path.join(run, "samples"))):
    if not f.endswith(".pkl"):
        continue
    s = pd.read_pickle(os.path.join(run, "samples", f))
    tp = s["tp"]
    if len(tp):
        rows.append(tp)
tp = pd.concat(rows, ignore_index=True)
tp["gs"] = np.hypot(tp.gvx, tp.gvy)
tp["ps"] = np.hypot(tp.pvx, tp.pvy)
v = tp[(tp.kfc >= 2) & np.isfinite(tp.ps)]
st = v[v.gs < 0.2].copy()
st["fm"] = st.ps > 0.5
res = {"n_stationary": int(len(st)), "false_moving_rate": float(st.fm.mean())}


def grp(col, bins=None, labels=None):
    d = st.copy()
    if bins is not None:
        d["_b"] = pd.cut(d[col], bins=bins, labels=labels, right=False)
        key = "_b"
    else:
        key = col
    o = {}
    for k, g in d.groupby(key):
        if len(g) == 0:
            continue
        o[str(k)] = {"n": int(len(g)), "fm_rate": round(float(g.fm.mean()), 4),
                     "speed_p50": round(float(g.ps.median()), 3), "speed_p95": round(float(np.percentile(g.ps, 95)), 3)}
    return o


res["by_class"] = grp("cls")
res["by_age"] = grp("age", [0, 3, 5, 10, 20, 50, 100000], ["2", "3-4", "5-9", "10-19", "20-49", "50+"])
res["by_range"] = grp("range", [0, 10, 20, 30, 50, 1000], ["0-10", "10-20", "20-30", "30-50", "50+"])
res["by_npts"] = grp("npts", [0, 10, 30, 100, 300, 100000], ["<10", "10-29", "30-99", "100-299", "300+"])
res["by_prev_miss"] = grp("prev_miss")
res["by_kfc"] = grp("kfc")
res["by_split"] = grp("split")
res["by_over_merge"] = grp("over_merge")
# per-class x age
o = {}
for cl, g0 in st.groupby("cls"):
    g0 = g0.copy()
    g0["_b"] = pd.cut(g0.age, [0, 3, 5, 10, 20, 50, 100000], labels=["2", "3-4", "5-9", "10-19", "20-49", "50+"], right=False)
    o[cl] = {str(k): {"n": int(len(g)), "fm_rate": round(float(g.fm.mean()), 4), "speed_p95": round(float(np.percentile(g.ps, 95)), 3)}
             for k, g in g0.groupby("_b") if len(g)}
res["by_class_age"] = o
# component signs in ego axes
res["signed_mean_x_y"] = [float(st.pvx.mean()), float(st.pvy.mean())]
res["pedestrian_center_temporal_hint"] = "see tpairs center_temporal for pedestrians"
json.dump(res, open(out, "w"), indent=1)
print(json.dumps(res, indent=1)[:6000])
