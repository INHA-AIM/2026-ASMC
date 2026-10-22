#!/usr/bin/env python3
"""Stage-1 supplementary metrics over frozen-evaluator samples (eval_lib v2 scene pkl). Read-only.

usage: python3 -B stage1_metrics.py OUT.json INPUTS_DIR ARM=RUN_DIR [ARM=RUN_DIR ...] [--scenes a,b,c]

The frozen evaluator (eval_lib.py, contract obb_quality_temporal_eval_v2) stays the primary source.
This script only re-reads its TP/FN/FP rows and adds:
  * velocity error tails (p50/p95/p99/max) by class, GT motion and EGO motion category
  * stationary-object false speed by ego motion (stationary / straight / turning)
  * signed stationary-object bias in ego axes (x = forward, y = left)
  * moving-object speed tracking lag (best integer scan shift) on segments with GT speed change
Ego motion categories use the EVALUATION ego pose (frames.csv), never an inference input.
Units: m/s, scans, rad/s. Velocity is the tracker 'vb' (base_link) vs GT ground velocity in ego axes.
"""
import json, math, os, sys
import numpy as np
import pandas as pd

sys.dont_write_bytecode = True


def ego_motion(frames):
    t = frames["stamp_ns"].to_numpy() * 1e-9
    x, y = frames["ego_x"].to_numpy(), frames["ego_y"].to_numpy()
    yaw = np.unwrap(frames["ego_yaw"].to_numpy())
    n = len(t)
    v = np.zeros(n); w = np.zeros(n)
    for k in range(n):
        a, b = max(0, k - 1), min(n - 1, k + 1)
        if a == b:
            continue
        dt = t[b] - t[a]
        v[k] = math.hypot(x[b] - x[a], y[b] - y[a]) / dt
        w[k] = (yaw[b] - yaw[a]) / dt
    cat = np.where(v < 0.3, "ego_stationary",
                   np.where(np.abs(w) < 0.05, "ego_straight", np.where(np.abs(w) >= 0.10, "ego_turning", "ego_mild_turn")))
    acc = np.zeros(n)
    for k in range(n):
        a, b = max(0, k - 2), min(n - 1, k + 2)
        if b > a:
            acc[k] = (v[b] - v[a]) / (t[b] - t[a])
    acat = np.where(v < 0.3, "ego_stationary", np.where(np.abs(acc) >= 0.5, "ego_accel_or_brake", "ego_const_speed"))
    return pd.DataFrame({"idx": frames["idx"].to_numpy(), "ego_v": v, "ego_w": w, "ego_a": acc, "ego_cat": cat, "ego_acat": acat})


def q(a, p):
    a = np.asarray(a, float)
    a = a[np.isfinite(a)]
    return float(np.percentile(a, p)) if len(a) else float("nan")


def stats(err):
    err = np.asarray(err, float)
    err = err[np.isfinite(err)]
    if not len(err):
        return {"n": 0}
    return {"n": int(len(err)), "mae": float(err.mean()), "p50": q(err, 50), "p95": q(err, 95),
            "p99": q(err, 99), "max": float(err.max())}


def lag_for_segment(g):
    """Best integer shift s (pred[k] ~ gt[k-s]) minimising mean |speed error|; s>0 = prediction lags."""
    g = g.sort_values("idx")
    if len(g) < 12:
        return None
    idx = g["idx"].to_numpy()
    if (np.diff(idx) != 1).any():
        return None
    gs = np.hypot(g["gvx"], g["gvy"]).to_numpy()
    ps = np.hypot(g["pvx"], g["pvy"]).to_numpy()
    if not np.isfinite(ps).all() or gs.max() - gs.min() < 2.0:
        return None
    best, bs = None, None
    for s in range(-3, 9):
        if s >= 0:
            e = np.abs(ps[s:] - gs[:len(gs) - s])
        else:
            e = np.abs(ps[:s] - gs[-s:])
        if len(e) < 8:
            continue
        m = e.mean()
        if best is None or m < best:
            best, bs = m, s
    return bs


def arm_metrics(run, inputs, scenes):
    sdir = os.path.join(run, "samples")
    tps, fns, fps, idss = [], [], [], []
    for sc in scenes:
        p = os.path.join(sdir, sc + ".pkl")
        if not os.path.isfile(p):
            continue
        fpath = os.path.join(inputs, sc, "frames.csv")
        if not os.path.isfile(fpath):   # e.g. static_bag lives in another inputs dir: reported separately
            continue
        s = pd.read_pickle(p)
        fr = pd.read_csv(fpath)
        em = ego_motion(fr)
        tp = s["tp"]
        if len(tp):
            tp = tp.merge(em, on="idx", how="left")
            tps.append(tp)
        fns.append(s["fn"]); fps.append(s["fp"]); idss.append(s["ids"])
    tp = pd.concat(tps, ignore_index=True) if tps else pd.DataFrame()
    fn = pd.concat(fns, ignore_index=True) if fns else pd.DataFrame()
    fp = pd.concat(fps, ignore_index=True) if fps else pd.DataFrame()
    ids = pd.concat(idss, ignore_index=True) if idss else pd.DataFrame()
    out = {"tp": int(len(tp)), "fn": int(len(fn)), "fp": int(len(fp)),
           "recall": len(tp) / max(1, len(tp) + len(fn)),
           "idsw": int(ids.idsw.sum()) if len(ids) else 0, "fragmentations": int(ids.frag.sum()) if len(ids) else 0}
    if not len(tp):
        return out
    tp["gs"] = np.hypot(tp.gvx, tp.gvy)
    tp["ps"] = np.hypot(tp.pvx, tp.pvy)
    tp["verr"] = np.hypot(tp.pvx - tp.gvx, tp.pvy - tp.gvy)
    v = tp[(tp.kfc >= 2) & np.isfinite(tp.pvx) & np.isfinite(tp.pvy)]
    out["velocity_all"] = stats(v.verr)
    for cl in ("vehicle", "pedestrian"):
        out["velocity_" + cl] = stats(v[v.cls == cl].verr)
    st = v[v.gs < 0.2]
    mv = v[v.gs >= 1.0]
    out["velocity_gt_stationary"] = stats(st.verr)
    out["velocity_gt_moving"] = stats(mv.verr)
    out["velocity_gt_moving_vehicle"] = stats(mv[mv.cls == "vehicle"].verr)
    out["stationary"] = {"n": int(len(st)), "false_moving_rate_0p5": float((st.ps > 0.5).mean()) if len(st) else float("nan"),
                         "false_moving_rate_1p0": float((st.ps > 1.0).mean()) if len(st) else float("nan"),
                         "speed": stats(st.ps)}
    by = {}
    for c, g in st.groupby("ego_cat"):
        by[c] = {"n": int(len(g)), "speed": stats(g.ps), "false_moving_rate_0p5": float((g.ps > 0.5).mean()),
                 "bias_x_mean": float(g.pvx.mean()), "bias_y_mean": float(g.pvy.mean()),
                 "vehicle_n": int((g.cls == "vehicle").sum()),
                 "vehicle_speed": stats(g[g.cls == "vehicle"].ps)}
    out["stationary_by_ego_motion"] = by
    by_a = {}
    for c, g in st.groupby("ego_acat"):
        by_a[c] = {"n": int(len(g)), "speed": stats(g.ps), "false_moving_rate_0p5": float((g.ps > 0.5).mean()),
                   "bias_x_mean": float(g.pvx.mean()), "vehicle_n": int((g.cls == "vehicle").sum()),
                   "vehicle_false_moving_rate_0p5": float((g[g.cls == "vehicle"].ps > 0.5).mean()) if (g.cls == "vehicle").any() else float("nan")}
    out["stationary_by_ego_accel"] = by_a
    acc_rows = st[st.ego_acat != "ego_stationary"]
    if len(acc_rows) > 3:
        out["stationary_bias_x_vs_ego_accel_corr"] = float(np.corrcoef(acc_rows.pvx, acc_rows.ego_a)[0, 1])
    # stationary bias versus ego yaw rate sign (turn-induced lateral/longitudinal bias)
    tr = st[st.ego_cat == "ego_turning"]
    if len(tr):
        sgn = np.sign(tr.ego_w)
        out["stationary_turning_signed"] = {"n": int(len(tr)), "bias_x_times_sign_w": float((tr.pvx * sgn).mean()),
                                            "bias_y_times_sign_w": float((tr.pvy * sgn).mean()),
                                            "corr_pvy_egow": float(np.corrcoef(tr.pvy, tr.ego_w)[0, 1]) if len(tr) > 3 else float("nan")}
    byego = {}
    for c, g in v.groupby("ego_cat"):
        byego[c] = stats(g.verr)
    out["velocity_by_ego_motion"] = byego
    # speed-tracking lag on moving segments (vehicles) with a GT speed change >= 2 m/s
    lags = []
    for (sc, seg), g in tp[(tp.cls == "vehicle") & (tp.kfc >= 2)].groupby(["scene", "seg"]):
        if g.gs.max() < 1.0:
            continue
        # contiguous runs only
        g = g.sort_values("idx")
        run_id = (g.idx.diff() != 1).cumsum()
        for _, r in g.groupby(run_id):
            s = lag_for_segment(r)
            if s is not None:
                lags.append(s)
    out["speed_lag_scans"] = {"n_segments": len(lags), "mean": float(np.mean(lags)) if lags else float("nan"),
                              "p50": q(lags, 50), "p95": q(lags, 95),
                              "hist": {str(k): int(sum(1 for x in lags if x == k)) for k in range(-3, 9)}}
    for key, col in (("center", "cerr"), ("long", "dlong"), ("short", "dshort"), ("height", "dh")):
        out[key + "_abs_err_vehicle"] = stats(np.abs(tp[tp.cls == "vehicle"][col]))
    out["yaw_abs_err_vehicle_deg"] = stats(np.degrees(np.abs(tp[tp.cls == "vehicle"].yerr)))
    return out


def main(argv):
    out = argv[0]; inputs = argv[1]
    scenes = None
    arms = {}
    rest = argv[2:]
    i = 0
    while i < len(rest):
        if rest[i] == "--scenes":
            scenes = rest[i + 1].split(","); i += 2; continue
        k, v = rest[i].split("=", 1); arms[k] = v; i += 1
    res = {"doc": __doc__, "inputs": inputs, "arms": {}}
    for name, run in arms.items():
        sc = scenes or sorted(f[:-4] for f in os.listdir(os.path.join(run, "samples")) if f.endswith(".pkl"))
        res["arms"][name] = {"run": run, "scenes": sc, "metrics": arm_metrics(run, inputs, sc)}
    with open(out, "w") as f:
        json.dump(res, f, indent=1)
    for name, a in res["arms"].items():
        m = a["metrics"]
        print(name, "tp", m.get("tp"), "fn", m.get("fn"), "fp", m.get("fp"), "idsw", m.get("idsw"),
              "vel", json.dumps(m.get("velocity_all")), "stat_fm", m.get("stationary", {}).get("false_moving_rate_0p5"))


if __name__ == "__main__":
    main(sys.argv[1:])
