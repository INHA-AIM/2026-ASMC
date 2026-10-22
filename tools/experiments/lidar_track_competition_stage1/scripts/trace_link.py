#!/usr/bin/env python3
"""Offline GT <-> pipeline linking of one object over consecutive scans (read-only).

usage: python3 -B trace_link.py RUN_DIR INPUTS_DIR SCENE OUT_PREFIX [--gt-tid N] [--from IDX] [--to IDX]

Chain per scan (GT is joined ONLY here, offline; the tracker never sees it):
  GT box (base_link)  ->  raw detection(s) (point mean + raw L-shape OBB, tracking input frame)
  -> merge lineage    ->  association (prediction, candidate costs, gate, assignment)
  -> KF (prior, innovation, gain, posterior)  ->  OBB stabilisation (raw -> stabilised)
  -> absolute velocity (relative + CAN, or ground state)  ->  V2 local output  ->  planner object (if map valid)
Inputs: RUN_DIR/out/SCENE.jsonl, RUN_DIR/trace/SCENE.trace.jsonl, RUN_DIR/samples/SCENE.pkl (evaluator TP pairs),
INPUTS_DIR/SCENE/gt.csv. Tracking-input -> base_link: + LiDAR mount (0.80, 0, 1.45) (identity leveling when
roll = pitch = 0, as in the competition-mode and historical inputs).
Outputs: OUT_PREFIX.csv (one row per scan) and OUT_PREFIX.txt (readable chain).
"""
import json, math, os, sys
import numpy as np
import pandas as pd

sys.dont_write_bytecode = True
MOUNT = (0.80, 0.0)
run, inputs, scene, prefix = sys.argv[1:5]
opt = sys.argv[5:]
gt_tid = None; lo = -1; hi = 10 ** 9
for i, a in enumerate(opt):
    if a == "--gt-tid": gt_tid = int(opt[i + 1])
    if a == "--from": lo = int(opt[i + 1])
    if a == "--to": hi = int(opt[i + 1])

s = pd.read_pickle(os.path.join(run, "samples", scene + ".pkl"))
tp = s["tp"]
if gt_tid is None:
    gt_tid = int(tp.groupby("gt_tid").size().idxmax())
pid_of = {int(r.idx): int(r.pid) for r in tp[tp.gt_tid == gt_tid].itertuples()}
gt = pd.read_csv(os.path.join(inputs, scene, "gt.csv"))
gt = gt[(gt.kind == "gt") & (gt.track_id == gt_tid)].set_index("idx")

ev = {}   # scan -> list of events
tpath = os.path.join(run, "trace", scene + ".trace.jsonl")
for line in open(tpath):
    e = json.loads(line)
    ev.setdefault(e["scan"], []).append(e)
out_lines = {}
for line in open(os.path.join(run, "out", scene + ".jsonl")):
    d = json.loads(line)
    out_lines[d["idx"]] = d


def b(x, y):
    return (x + MOUNT[0], y + MOUNT[1])


rows = []
txt = []
prev_pid = None
for k in sorted(gt.index):
    if k < lo or k > hi:
        continue
    g = gt.loc[k]
    pid = pid_of.get(k)
    E = ev.get(k, [])
    r = {"idx": k, "gt_x": g.x, "gt_y": g.y, "gt_l": g.l, "gt_w": g.w, "gt_yaw": g.yaw, "gt_vx": g.vx, "gt_vy": g.vy, "pid": pid}
    scan = next((e for e in E if e["ev"] == "scan"), None)
    if scan:
        r.update({"dt": scan.get("dt_s"), "can_kph": scan.get("can_kph"), "imu_yaw_deg": scan.get("imu_yaw_deg")})
    line = f"[{k}] GT c=({g.x:.2f},{g.y:.2f}) l/w={g.l:.2f}/{g.w:.2f} yaw={math.degrees(g.yaw):.1f} v=({g.vx:.2f},{g.vy:.2f})"
    if pid is None:
        dets = next((e for e in E if e["ev"] == "detections"), None)
        near = None
        if dets:
            for dd in dets["det"]:
                bx, by = b(*dd["mean"][:2])
                dist = math.hypot(bx - g.x, by - g.y)
                if near is None or dist < near[0]:
                    near = (dist, dd)
        if near:
            r.update({"nearest_det": near[1]["det"], "nearest_det_dist": near[0], "nearest_det_npts": near[1]["npts"]})
        out_lines_k = out_lines.get(k, {})
        r["status"] = "FN"
        line += f"  -> FN (no matched output); nearest raw det {near[1]['det'] if near else None} at {near[0]:.2f} m" if near else "  -> FN, no detection"
        txt.append(line); rows.append(r); continue
    m = next((e for e in E if e["ev"] == "match" and e["id"] == pid), None)
    bth = next((e for e in E if e["ev"] == "birth" and e["id"] == pid), None)
    asc = next((e for e in E if e["ev"] == "assoc" and e["id"] == pid), None)
    kf = next((e for e in E if e["ev"] == "kf_update" and e["id"] == pid), None)
    pub = next((e for e in E if e["ev"] == "publish"), None)
    det_id = m["det"] if m else (bth["det"] if bth else None)
    dets = next((e for e in E if e["ev"] == "detections"), None)
    dd = next((x for x in (dets["det"] if dets else []) if x["det"] == det_id), None)
    merges = [e for e in E if e["ev"] == "merge" and det_id is not None and (e["owner_det"] == det_id or e["fragment_det"] == det_id)]
    r["status"] = "TP" + ("" if prev_pid in (None, pid) else "_IDSW")
    if dd:
        mx, my = b(*dd["mean"][:2]); ox, oy = b(dd["obb"][0], dd["obb"][1])
        r.update({"det": det_id, "det_npts": dd["npts"], "det_mean_x": mx, "det_mean_y": my, "raw_obb_x": ox, "raw_obb_y": oy,
                  "raw_obb_l": dd["obb"][2], "raw_obb_w": dd["obb"][3], "raw_obb_yaw": dd["obb"][4]})
        line += f"\n    det {det_id}: npts={dd['npts']} mean=({mx:.2f},{my:.2f}) rawOBB c=({ox:.2f},{oy:.2f}) {dd['obb'][2]:.2f}x{dd['obb'][3]:.2f} yaw={math.degrees(dd['obb'][4]):.1f}"
    for mg in merges:
        line += f"\n    merge owner={mg['owner_det']} frag={mg['fragment_det']} pts {mg['owner_pts']}+{mg['fragment_pts']} dist={mg['obb_center_dist']:.2f}"
    r["merges"] = len(merges)
    if bth:
        line += f"\n    BIRTH id={pid} reason={bth['reason']}"
    if asc:
        px, py = b(*asc["pred"]); r.update({"pred_x": px, "pred_y": py, "assoc_cost": None})
        cands = sorted(asc["cands"], key=lambda c: c["cost"])[:3]
        line += f"\n    assoc id={pid} age={asc['age']} miss={asc['miss']} kfc={asc['kfc']} pred=({px:.2f},{py:.2f}) gate={asc['gate']} cands=" + \
                ", ".join(f"det{c['det']}:d={c['d']:.2f}/c={c['cost']:.2f}{'*' if c['gated'] else ''}" for c in cands) + f" -> j={asc['assigned_j']}"
    if kf:
        r.update({"innov_x": kf["innov"][0], "innov_y": kf["innov"][1], "K_vx": kf["K"][4], "K_vy": kf["K"][7],
                  "kf_vx": kf["x_post"][2], "kf_vy": kf["x_post"][3], "kfc_in": kf["kfc_in"], "init_elapsed": kf["init_elapsed"]})
        line += f"\n    KF kfc_in={kf['kfc_in']} dt={kf['dt']:.3f} innov=({kf['innov'][0]:.3f},{kf['innov'][1]:.3f}) K_vel=({kf['K'][4]:.2f},{kf['K'][7]:.2f}) x_post v=({kf['x_post'][2]:.2f},{kf['x_post'][3]:.2f}) a=({kf['x_post'][4]:.2f},{kf['x_post'][5]:.2f})"
    if m:
        sx, sy = b(m["obb_stab"][0], m["obb_stab"][1])
        r.update({"stab_obb_x": sx, "stab_obb_y": sy, "stab_l": m["obb_stab"][2], "stab_w": m["obb_stab"][3], "stab_yaw": m["obb_stab"][4],
                  "stab_rule": m["stab_rule"], "abs_vx": m["abs_v"][0], "abs_vy": m["abs_v"][1], "can_added": m["can_added"], "prev_miss": m["prev_miss"]})
        line += f"\n    OBB {m['stab_rule']}: stab c=({sx:.2f},{sy:.2f}) {m['obb_stab'][2]:.2f}x{m['obb_stab'][3]:.2f} yaw={math.degrees(m['obb_stab'][4]):.1f}; absV=({m['abs_v'][0]:.2f},{m['abs_v'][1]:.2f}) can_added={m['can_added']:.2f}"
    if pub:
        lo_ = next((o for o in pub["local"] if o["id"] == pid), None)
        if lo_:
            r.update({"v2_x": lo_["c"][0], "v2_y": lo_["c"][1], "v2_l": lo_["l"], "v2_w": lo_["w"], "v2_yaw": lo_["yaw"], "v2_vel_valid": lo_["vel_valid"]})
            line += f"\n    V2 local c=({lo_['c'][0]:.2f},{lo_['c'][1]:.2f}) {lo_['l']:.2f}x{lo_['w']:.2f} yaw={math.degrees(lo_['yaw']):.1f} vel_valid={lo_['vel_valid']}"
        pl = pub.get("planner")
        if pl:
            p = next((o for o in pl if o["id"] == pid), None)
            if p:
                r.update({"planner_vx_map": p["v_map"][0], "planner_vy_map": p["v_map"][1], "planner_ms": p["ms"]})
                line += f"\n    planner v_map=({p['v_map'][0]:.2f},{p['v_map'][1]:.2f}) raw=({p['raw_v_map'][0]:.2f},{p['raw_v_map'][1]:.2f}) ms={p['ms']}"
        elif pub.get("planner_reason"):
            line += f"\n    planner: {pub['planner_reason']}"
    vb = None
    tk = next((t for t in out_lines.get(k, {}).get("tracks", []) if t["id"] == pid), None)
    if tk:
        r.update({"vb_x": tk["vb"][0], "vb_y": tk["vb"][1]})
    prev_pid = pid
    txt.append(line); rows.append(r)
pd.DataFrame(rows).to_csv(prefix + ".csv", index=False)
open(prefix + ".txt", "w").write(f"scene={scene} gt_tid={gt_tid} run={run}\n" + "\n".join(txt) + "\n")
print(f"scene={scene} gt_tid={gt_tid} rows={len(rows)} -> {prefix}.csv / .txt")
