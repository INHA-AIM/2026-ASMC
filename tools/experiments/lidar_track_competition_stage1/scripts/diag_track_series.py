#!/usr/bin/env python3
"""Per-object time series joining GT, evaluator match and harness track state (read-only).
usage: python3 -B diag_track_series.py RUN_DIR INPUTS_DIR SCENE OUT.csv [--gt-tid N] [--auto stationary_vehicle|stationary_pedestrian|moving_vehicle]
Columns: idx, t, ego speed proxy (eval pose), GT x/y/vx/vy (ego axes), matched pid, track miss/age/kfc,
point-mean measurement, KF position/velocity/acceleration (tracking input frame), vb (base_link), OBB center,
pred (association prediction). The KF state is in the tracking input frame (leveled sensor); vb and GT are
in base_link: they differ by the mount translation only (rotation = identity for this sensor set).
"""
import json, math, os, sys
import numpy as np
import pandas as pd
sys.dont_write_bytecode = True

run, inputs, scene, out = sys.argv[1:5]
opts = sys.argv[5:]
gt_tid = None; auto = None
for i, a in enumerate(opts):
    if a == "--gt-tid": gt_tid = int(opts[i + 1])
    if a == "--auto": auto = opts[i + 1]
s = pd.read_pickle(os.path.join(run, "samples", scene + ".pkl"))
tp = s["tp"].copy()
tp["gs"] = np.hypot(tp.gvx, tp.gvy); tp["ps"] = np.hypot(tp.pvx, tp.pvy)
if gt_tid is None:
    if auto == "stationary_vehicle":
        cand = tp[(tp.cls == "vehicle") & (tp.gs < 0.2) & (tp.kfc >= 2)]
    elif auto == "stationary_pedestrian":
        cand = tp[(tp.cls == "pedestrian") & (tp.gs < 0.2) & (tp.kfc >= 2)]
    else:
        cand = tp[(tp.cls == "vehicle") & (tp.gs > 2.0) & (tp.kfc >= 2)]
    agg = cand.groupby("gt_tid").agg(n=("ps", "size"), fm=("ps", lambda x: float((x > 0.5).mean())), p95=("ps", lambda x: float(np.percentile(x, 95))))
    agg = agg[agg.n >= 30].sort_values(["fm", "n"], ascending=False)
    print(agg.head(10).to_string())
    gt_tid = int(agg.index[0])
frames = pd.read_csv(os.path.join(inputs, scene, "frames.csv"))
gt = pd.read_csv(os.path.join(inputs, scene, "gt.csv"))
gt = gt[(gt.kind == "gt") & (gt.track_id == gt_tid)].set_index("idx")
match = tp[tp.gt_tid == gt_tid].set_index("idx")
tracks = {}
with open(os.path.join(run, "out", scene + ".jsonl")) as f:
    for line in f:
        d = json.loads(line)
        tracks[d["idx"]] = {t["id"]: t for t in d.get("tracks", [])}
rows = []
for fr in frames.itertuples(index=False):
    k = fr.idx
    if k not in gt.index:
        continue
    g = gt.loc[k]
    r = {"idx": k, "t": fr.stamp_ns * 1e-9, "ego_speed_proxy": fr.ego_speed_mps, "gt_x": g.x, "gt_y": g.y, "gt_vx": g.vx, "gt_vy": g.vy,
         "gt_l": g.l, "gt_w": g.w}
    if k in match.index:
        m = match.loc[k]
        pid = int(m.pid)
        r.update({"pid": pid, "cerr": m.cerr, "pvx": m.pvx, "pvy": m.pvy, "npts": m.npts, "age": m.age})
        t = tracks.get(k, {}).get(pid)
        if t:
            kf = t["kf"]
            r.update({"miss": t["miss"], "kfc": t["kfc"], "meas_x": (t["meas"] or [None])[0], "meas_y": (t["meas"] or [None, None])[1],
                      "kf_x": kf[0], "kf_y": kf[1], "kf_vx": kf[2], "kf_vy": kf[3], "kf_ax": kf[4], "kf_ay": kf[5],
                      "vb_x": t["vb"][0], "vb_y": t["vb"][1], "obb_cx": t["obb"][0], "obb_cy": t["obb"][1],
                      "obb_w": t["obb"][3], "obb_h": t["obb"][4],
                      "pred_x": (t["pred"] or [None, None])[0], "pred_y": (t["pred"] or [None, None])[1]})
    rows.append(r)
df = pd.DataFrame(rows)
df.to_csv(out, index=False)
pd.set_option("display.width", 250); pd.set_option("display.max_columns", 40)
cols = [c for c in ["idx", "ego_speed_proxy", "gt_x", "gt_y", "pid", "age", "npts", "meas_x", "meas_y", "kf_x", "kf_y", "kf_vx", "kf_vy", "kf_ax", "vb_x", "vb_y", "obb_cx", "obb_cy"] if c in df]
print("gt_tid", gt_tid, "rows", len(df))
print(df[cols].round(3).head(60).to_string())
