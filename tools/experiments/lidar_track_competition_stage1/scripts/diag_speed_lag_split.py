#!/usr/bin/env python3
"""Speed-tracking lag, split by what the EGO was doing during the same segment (read-only, evaluation pose only).

usage: python3 -B diag_speed_lag_split.py OUT.json INPUTS_DIR ARM=RUN_DIR [ARM=RUN_DIR ...] [--scenes a,b]

Same segment definition and lag estimator as stage1_metrics.py (vehicle, kfc >= 2, contiguous TP run >= 12 scans,
GT speed change >= 2 m/s; lag = integer scan shift s minimising mean |pred_speed[k] - gt_speed[k-s]|).
Each segment is then classified by the evaluation ego pose over the same scans:
  ego_dv      = max - min of ego speed over the run (m/s)
  ego_cat     = ego_steady (ego_dv < 1.0) | ego_changing (ego_dv >= 1.0)
  co_motion   = Pearson corr(gt_speed, ego_speed) over the run; co_moving if corr >= 0.5 (target speed follows ego speed)
Why: under P0/CB the CAN speed is added instantly to the relative state, so a target whose speed changes WITH the ego
appears to be followed instantly, while a target changing speed independently must be followed by the CA filter in
both arms. Reported per arm: lag mean/p50/p95/hist and |speed error| by category, with n.
"""
import json, math, os, sys
import numpy as np
import pandas as pd
sys.dont_write_bytecode = True
out, inputs = sys.argv[1:3]
arms = {}
scenes = None
args = sys.argv[3:]
i = 0
while i < len(args):
    a = args[i]
    if a == "--scenes":
        scenes = args[i + 1].split(","); i += 2; continue
    k, v = a.split("=", 1); arms[k] = v; i += 1


def ego_speed(frames):
    t = frames["stamp_ns"].to_numpy() * 1e-9
    x, y = frames["ego_x"].to_numpy(), frames["ego_y"].to_numpy()
    n = len(t); v = np.zeros(n)
    for k in range(n):
        a, b = max(0, k - 1), min(n - 1, k + 1)
        if a == b:
            continue
        dt = t[b] - t[a]
        v[k] = math.hypot(x[b] - x[a], y[b] - y[a]) / dt if dt > 0 else 0.0
    return pd.Series(v, index=frames["idx"].to_numpy())


def lag_for_segment(gs, ps):
    best, bs = None, None
    for s in range(-3, 9):
        if s >= 0:
            e = np.abs(ps[s:] - gs[:len(gs) - s])
        else:
            e = np.abs(ps[:s] - gs[-s:])
        if len(e) < 8:
            continue
        m = float(e.mean())
        if best is None or m < best:
            best, bs = m, s
    return bs, best


def q(a, p):
    a = np.asarray(a, float)
    return float(np.percentile(a, p)) if len(a) else float("nan")


res = {"doc": __doc__, "arms": {}, "segments": {}}
for arm, run in arms.items():
    sdir = os.path.join(run, "samples")
    scs = scenes or sorted(f[:-4] for f in os.listdir(sdir) if f.endswith(".pkl"))
    rows = []
    for sc in scs:
        fp = os.path.join(inputs, sc, "frames.csv")
        if not os.path.isfile(fp):
            continue
        ev = ego_speed(pd.read_csv(fp))
        tp = pd.read_pickle(os.path.join(sdir, sc + ".pkl"))["tp"]
        if not len(tp):
            continue
        v = tp[(tp.cls == "vehicle") & (tp.kfc >= 2)]
        for (sc_, seg), g in v.groupby(["scene", "seg"]):
            if g.gs.max() < 1.0:
                continue
            g = g.sort_values("idx")
            run_id = (g.idx.diff() != 1).cumsum()
            for _, r in g.groupby(run_id):
                if len(r) < 12:
                    continue
                idx = r["idx"].to_numpy()
                gs = np.hypot(r["gvx"], r["gvy"]).to_numpy()
                ps = np.hypot(r["pvx"], r["pvy"]).to_numpy()
                if not np.isfinite(ps).all() or gs.max() - gs.min() < 2.0:
                    continue
                s, err = lag_for_segment(gs, ps)
                if s is None:
                    continue
                es = ev.reindex(idx).to_numpy()
                ego_dv = float(np.nanmax(es) - np.nanmin(es))
                corr = float(np.corrcoef(gs, es)[0, 1]) if np.nanstd(es) > 1e-6 and np.std(gs) > 1e-6 else 0.0
                rows.append({"scene": sc, "seg": int(seg), "idx0": int(idx[0]), "idx1": int(idx[-1]), "n": int(len(r)),
                             "gt_dv": float(gs.max() - gs.min()), "ego_dv": ego_dv, "corr": corr, "lag": int(s),
                             "speed_mae_at_lag0": float(np.abs(ps - gs).mean()), "speed_mae_at_best": err,
                             "ego_cat": "ego_steady" if ego_dv < 1.0 else "ego_changing",
                             "co_motion": "co_moving" if corr >= 0.5 else "independent"})
    d = pd.DataFrame(rows)
    res["segments"][arm] = rows
    summ = {}
    def block(g):
        return {"n_segments": int(len(g)), "lag_mean": float(g.lag.mean()) if len(g) else float("nan"),
                "lag_p50": q(g.lag, 50), "lag_p95": q(g.lag, 95),
                "lag_hist": {str(k): int((g.lag == k).sum()) for k in range(-3, 9)},
                "speed_mae_lag0_mean": float(g.speed_mae_at_lag0.mean()) if len(g) else float("nan")}
    summ["all"] = block(d)
    for c, g in d.groupby("ego_cat"):
        summ[c] = block(g)
    for c, g in d.groupby("co_motion"):
        summ[c] = block(g)
    for (a, b), g in d.groupby(["ego_cat", "co_motion"]):
        summ[a + "/" + b] = block(g)
    res["arms"][arm] = summ
json.dump(res, open(out, "w"), indent=1)
# compact print
cats = ["all", "ego_steady", "ego_changing", "independent", "co_moving", "ego_steady/independent", "ego_changing/co_moving"]
print("category".ljust(26), " ".join(a.ljust(22) for a in arms))
for c in cats:
    cells = []
    for a in arms:
        b = res["arms"][a].get(c)
        cells.append((f"n={b['n_segments']} mean={b['lag_mean']:.2f} p50={b['lag_p50']:.0f}" if b else "-").ljust(22))
    print(c.ljust(26), " ".join(cells))
