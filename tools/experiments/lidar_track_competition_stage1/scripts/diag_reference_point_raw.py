#!/usr/bin/env python3
"""Reference-point study v2 (adds the RAW detection L-shape OBB centre from the track trace).

usage: python3 -B diag_reference_point_raw.py RUN_DIR INPUTS_DIR OUT.json [--scenes a,b] [--table scene:gt_tid:out.csv ...]

Compares, per evaluator TP (track pid matched to GT gt_tid in a scan), four candidate reference points of the
same object against the GT full-box centre, and the frame-to-frame WORLD displacement of each for GT-stationary
objects (|v_gt| < 0.2 m/s):
  meas     = cluster point mean (= the KF measurement today; jsonl tracks[].meas)
  obb_raw  = raw L-shape OBB centre of the matched detection BEFORE stabilisation (trace 'match'.obb_raw)
  obb_stab = OBB centre after stabilizeMatchedOBB = V2 output centre (jsonl tracks[].obb; == trace obb_stab)
  kf       = KF posterior position (jsonl tracks[].kf)
  gt       = GT full-box centre (evaluation only: V2 centre - evaluator residual)
Frames: tracking input frame -> base_link by + LiDAR mount 0.80 m (identity leveling for these inputs);
world via the evaluation ego pose (frames.csv) - evaluation use only, never an inference input.
Requires RUN_DIR/trace/<scene>.trace.jsonl (run with TRACE_SCENES). Units m. Read-only; DIAGNOSTIC.
"""
import json, math, os, sys
import numpy as np
import pandas as pd
sys.dont_write_bytecode = True
MOUNT_X = 0.80
run, inputs, out = sys.argv[1:4]
opt = sys.argv[4:]
scenes = None
tables = []
for i, a in enumerate(opt):
    if a == "--scenes":
        scenes = opt[i + 1].split(",")
    if a == "--table":
        tables.append(opt[i + 1])
if scenes is None:
    scenes = sorted(f[:-12] for f in os.listdir(os.path.join(run, "trace")) if f.endswith(".trace.jsonl"))

recs = []
n_no_trace = 0
for sc in scenes:
    fp = os.path.join(inputs, sc, "frames.csv")
    tp_path = os.path.join(run, "samples", sc + ".pkl")
    tr_path = os.path.join(run, "trace", sc + ".trace.jsonl")
    if not (os.path.isfile(fp) and os.path.isfile(tp_path) and os.path.isfile(tr_path)):
        continue
    fr = pd.read_csv(fp).set_index("idx")
    tp = pd.read_pickle(tp_path)["tp"]
    if not len(tp):
        continue
    # trace: raw OBB per (scan, track id) from 'match' events
    raw = {}
    with open(tr_path) as f:
        for line in f:
            if '"ev":"match"' not in line:
                continue
            d = json.loads(line)
            raw[(int(d["scan"]), int(d["id"]))] = d
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
                m = raw.get((k, int(r.pid)))
                if m is None:
                    n_no_trace += 1
                    continue
                rec = {"scene": sc, "idx": k, "pid": int(r.pid), "gt_tid": r.gt_tid, "cls": r.cls, "range": r.range,
                       "gs": math.hypot(r.gvx, r.gvy), "prev_miss": int(m.get("prev_miss", 0)), "stab_rule": m.get("stab_rule", ""),
                       "npts": int(m.get("npts", 0))}
                rec["meas_x"], rec["meas_y"] = t["meas"][0] + MOUNT_X, t["meas"][1]
                rec["obb_stab_x"], rec["obb_stab_y"] = t["obb"][0] + MOUNT_X, t["obb"][1]
                rec["obb_raw_x"], rec["obb_raw_y"] = m["obb_raw"][0] + MOUNT_X, m["obb_raw"][1]
                rec["kf_x"], rec["kf_y"] = t["kf"][0] + MOUNT_X, t["kf"][1]
                # consistency check between jsonl output OBB and trace stabilised OBB
                rec["stab_consist"] = math.hypot(m["obb_stab"][0] - t["obb"][0], m["obb_stab"][1] - t["obb"][1])
                rec["raw_w"], rec["raw_h"], rec["raw_yaw"] = m["obb_raw"][2], m["obb_raw"][3], m["obb_raw"][4]
                rec["dx"], rec["dy"] = r.dx, r.dy
                rec["ex"], rec["ey"], rec["c"], rec["s"] = ex, ey, c, s
                recs.append(rec)
df = pd.DataFrame(recs)
res = {"run": run, "n_tp": int(len(df)), "n_tp_without_trace_match": n_no_trace, "scenes": scenes, "doc": __doc__}
if not len(df):
    json.dump(res, open(out, "w"), indent=1)
    print("no rows"); sys.exit(0)
df["gt_x"] = df.obb_stab_x - df.dx; df["gt_y"] = df.obb_stab_y - df.dy
NAMES = ("meas", "obb_raw", "obb_stab", "kf")

def st(a):
    a = np.asarray(a, float); a = a[np.isfinite(a)]
    return {"n": int(len(a)), "p50": float(np.percentile(a, 50)), "p95": float(np.percentile(a, 95)), "mean": float(a.mean())} if len(a) else {"n": 0}

res["stab_consistency_m"] = st(df.stab_consist)
for n in NAMES:
    df[n + "_err"] = np.hypot(df[n + "_x"] - df.gt_x, df[n + "_y"] - df.gt_y)
    # signed error along the ego->object line of sight (positive = nearer to ego than the GT centre)
    los = np.arctan2(df.gt_y, df.gt_x)
    df[n + "_err_los"] = -((df[n + "_x"] - df.gt_x) * np.cos(los) + (df[n + "_y"] - df.gt_y) * np.sin(los))
out_err = {}
for cl in ("vehicle", "pedestrian"):
    g = df[df.cls == cl]
    out_err[cl] = {n: st(g[n + "_err"]) for n in NAMES}
    out_err[cl + "_signed_toward_ego"] = {n: st(g[n + "_err_los"]) for n in NAMES}
    rb = {}
    for lo_, hi_ in ((0, 10), (10, 20), (20, 35), (35, 100)):
        gg = g[(g.range >= lo_) & (g.range < hi_)]
        rb[f"{lo_}-{hi_}m"] = {n: st(gg[n + "_err"]) for n in NAMES}
    out_err[cl + "_by_range"] = rb
res["error_vs_gt_center_m"] = out_err
# world-frame frame-to-frame displacement for GT-stationary objects (consecutive TP scans, same pid & gt)
for n in NAMES + ("gt",):
    df[n + "_wx"] = df.ex + df.c * df[n + "_x"] - df.s * df[n + "_y"]
    df[n + "_wy"] = df.ey + df.s * df[n + "_x"] + df.c * df[n + "_y"]
st_df = df[df.gs < 0.2].sort_values(["scene", "gt_tid", "pid", "idx"])
jit = {cl: {n: [] for n in NAMES + ("gt",)} for cl in ("vehicle", "pedestrian")}
jit_nomiss = {cl: {n: [] for n in NAMES + ("gt",)} for cl in ("vehicle", "pedestrian")}
for (sc, gtid, pid), g in st_df.groupby(["scene", "gt_tid", "pid"]):
    g = g.reset_index(drop=True)
    for i in range(1, len(g)):
        if g.idx[i] != g.idx[i - 1] + 1:
            continue
        for n in NAMES + ("gt",):
            d_ = math.hypot(g[n + "_wx"][i] - g[n + "_wx"][i - 1], g[n + "_wy"][i] - g[n + "_wy"][i - 1])
            jit[g.cls[i]][n].append(d_)
            if g.prev_miss[i] == 0 and g.prev_miss[i - 1] == 0:
                jit_nomiss[g.cls[i]][n].append(d_)
res["stationary_world_step_m"] = {cl: {n: st(v) for n, v in d.items()} for cl, d in jit.items()}
res["stationary_world_step_m_no_miss"] = {cl: {n: st(v) for n, v in d.items()} for cl, d in jit_nomiss.items()}
# raw vs stabilised OBB centre difference and stabilisation rule mix
res["obb_raw_minus_stab_m"] = st(np.hypot(df.obb_raw_x - df.obb_stab_x, df.obb_raw_y - df.obb_stab_y))
res["stab_rule_counts"] = {k: int(v) for k, v in df.stab_rule.value_counts().items()}
json.dump(res, open(out, "w"), indent=1)
# per-object consecutive tables
for spec in tables:
    sc, gtid, path = spec.split(":")
    g = df[(df.scene == sc) & (df.gt_tid.astype(str) == gtid)].sort_values("idx")
    cols = ["idx", "pid", "prev_miss", "stab_rule", "npts", "range", "gs"]
    for n in ("gt",) + NAMES:
        cols += [n + "_wx", n + "_wy"]
    for n in NAMES:
        cols += [n + "_err", n + "_err_los"]
    t = g[cols].copy()
    for n in ("gt",) + NAMES:
        t[n + "_step"] = np.hypot(t[n + "_wx"].diff(), t[n + "_wy"].diff())
    t.to_csv(path, index=False, float_format="%.3f")
print(json.dumps({"n_tp": res["n_tp"], "no_trace": n_no_trace, "stab_consistency_p95": res["stab_consistency_m"].get("p95"),
                  "error_p50": {cl: {n: round(out_err[cl][n].get("p50", float("nan")), 3) for n in NAMES} for cl in ("vehicle", "pedestrian")},
                  "signed_toward_ego_p50_vehicle": {n: round(out_err["vehicle_signed_toward_ego"][n].get("p50", float("nan")), 3) for n in NAMES},
                  "stationary_step_p95": {cl: {n: round(res["stationary_world_step_m"][cl][n].get("p95", float("nan")), 3) for n in NAMES + ("gt",)} for cl in jit},
                  "stationary_step_p95_no_miss": {cl: {n: round(res["stationary_world_step_m_no_miss"][cl][n].get("p95", float("nan")), 3) for n in NAMES + ("gt",)} for cl in jit}}, indent=1))
