#!/usr/bin/env python3
"""Reference-point study: point mean (KF measurement) vs output OBB centre vs KF position vs GT box centre.
usage: python3 -B diag_reference_point.py RUN_DIR INPUTS_DIR OUT.json [--scenes a,b]
Per evaluator TP (track pid matched to GT gt_tid in a scan) it takes from the harness jsonl tracks[]:
  meas = cluster point mean (tracking input frame), obb = output OBB centre, kf = KF position;
  all converted to base_link (+ LiDAR mount 0.80 m; leveling identity for these inputs).
Error vs GT box centre (base_link) and, for GT-stationary objects (|v_gt| < 0.2 m/s), the frame-to-frame
displacement of each point in the WORLD (evaluation pose, frames.csv; evaluation use only) for consecutive
TP scans of the same (pid, gt_tid). Units m. Range bins by GT range.
"""
import json, math, os, sys
import numpy as np
import pandas as pd
sys.dont_write_bytecode = True
run, inputs, out = sys.argv[1:4]
scenes = None
if "--scenes" in sys.argv:
    scenes = sys.argv[sys.argv.index("--scenes") + 1].split(",")
if scenes is None:
    scenes = sorted(f[:-4] for f in os.listdir(os.path.join(run, "samples")) if f.endswith(".pkl"))
recs = []
for sc in scenes:
    fp = os.path.join(inputs, sc, "frames.csv")
    if not os.path.isfile(fp):
        continue
    fr = pd.read_csv(fp).set_index("idx")
    tp = pd.read_pickle(os.path.join(run, "samples", sc + ".pkl"))["tp"]
    if not len(tp):
        continue
    want = {}
    for r in tp.itertuples():
        want.setdefault(int(r.idx), []).append(r)
    with open(os.path.join(run, "out", sc + ".jsonl")) as f:
        for line in f:
            d = json.loads(line)
            k = d["idx"]
            if k not in want:
                continue
            tr = {t["id"]: t for t in d.get("tracks", [])}
            ex, ey, eyaw = fr.loc[k, "ego_x"], fr.loc[k, "ego_y"], fr.loc[k, "ego_yaw"]
            c, s = math.cos(eyaw), math.sin(eyaw)
            for r in want[k]:
                t = tr.get(int(r.pid))
                if not t or t["meas"] is None or t["meas"][0] is None:
                    continue
                rec = {"scene": sc, "idx": k, "pid": int(r.pid), "gt_tid": r.gt_tid, "cls": r.cls, "range": r.range, "gs": math.hypot(r.gvx, r.gvy)}
                # evaluator: dx = pred_c - gt ; pred_c = V2 centre (base_link) -> gt = pred_c - dx
                rec["meas_x"], rec["meas_y"] = t["meas"][0] + 0.8, t["meas"][1]
                rec["obb_x"], rec["obb_y"] = t["obb"][0] + 0.8, t["obb"][1]
                rec["kf_x"], rec["kf_y"] = t["kf"][0] + 0.8, t["kf"][1]
                rec["cerr_v2"] = r.cerr
                rec["dx"], rec["dy"] = r.dx, r.dy
                rec["ex"], rec["ey"], rec["c"], rec["s"] = ex, ey, c, s
                recs.append(rec)
df = pd.DataFrame(recs)
# GT centre (base) from the V2 centre and evaluator residual: V2 centre == obb centre transformed (identity leveling)
df["gt_x"] = df.obb_x - df.dx; df["gt_y"] = df.obb_y - df.dy
res = {"run": run, "n_tp": int(len(df)), "doc": __doc__}
def st(a):
    a = np.asarray(a, float); a = a[np.isfinite(a)]
    return {"n": int(len(a)), "p50": float(np.percentile(a, 50)), "p95": float(np.percentile(a, 95)), "mean": float(a.mean())} if len(a) else {"n": 0}
for name in ("meas", "obb", "kf"):
    df[name + "_err"] = np.hypot(df[name + "_x"] - df.gt_x, df[name + "_y"] - df.gt_y)
out_err = {}
for cl in ("vehicle", "pedestrian"):
    g = df[df.cls == cl]
    out_err[cl] = {n: st(g[n + "_err"]) for n in ("meas", "obb", "kf")}
    rb = {}
    for lo_, hi_ in ((0, 10), (10, 20), (20, 35), (35, 100)):
        gg = g[(g.range >= lo_) & (g.range < hi_)]
        rb[f"{lo_}-{hi_}m"] = {n: st(gg[n + "_err"]) for n in ("meas", "obb", "kf")}
    out_err[cl + "_by_range"] = rb
res["error_vs_gt_center_m"] = out_err
# world-frame frame-to-frame displacement for GT-stationary objects
for name in ("meas", "obb", "kf", "gt"):
    df[name + "_wx"] = df.ex + df.c * df[name + "_x"] - df.s * df[name + "_y"]
    df[name + "_wy"] = df.ey + df.s * df[name + "_x"] + df.c * df[name + "_y"]
st_df = df[df.gs < 0.2].sort_values(["scene", "gt_tid", "pid", "idx"])
jit = {cl: {n: [] for n in ("meas", "obb", "kf", "gt")} for cl in ("vehicle", "pedestrian")}
for (sc, gtid, pid), g in st_df.groupby(["scene", "gt_tid", "pid"]):
    g = g.reset_index(drop=True)
    for i in range(1, len(g)):
        if g.idx[i] != g.idx[i - 1] + 1:
            continue
        for n in ("meas", "obb", "kf", "gt"):
            jit[g.cls[i]][n].append(math.hypot(g[n + "_wx"][i] - g[n + "_wx"][i - 1], g[n + "_wy"][i] - g[n + "_wy"][i - 1]))
res["stationary_world_step_m"] = {cl: {n: st(v) for n, v in d.items()} for cl, d in jit.items()}
json.dump(res, open(out, "w"), indent=1)
print(json.dumps({"error": {cl: {n: out_err[cl][n].get("p50") for n in ("meas", "obb", "kf")} for cl in ("vehicle", "pedestrian")},
                  "stationary_step_p95": {cl: {n: res["stationary_world_step_m"][cl][n].get("p95") for n in ("meas", "obb", "kf", "gt")} for cl in jit}}, indent=1))
