#!/usr/bin/env python3
"""DIAGNOSTIC_ONLY synthetic-scene report (ideal geometry, exact ego inputs). Read-only.
usage: python3 -B syn_report.py OUT.json SYN_INPUTS ARM=RUN_DIR [ARM=RUN_DIR ...]
Per synthetic scene and arm: TP/FN, ID switches, fragmentations, velocity error (all / GT-stationary /
GT-moving vehicles), stationary false-moving rate, centre error. Never mixed with real-data metrics.
"""
import json, os, sys
sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import stage1_metrics as sm  # noqa: E402

out, syn = sys.argv[1], sys.argv[2]
arms = dict(a.split("=", 1) for a in sys.argv[3:])
scenes = sorted(d for d in os.listdir(syn) if d.startswith("syn"))
res = {"label": "DIAGNOSTIC_ONLY", "inputs": syn, "scenes": {}}
for sc in scenes:
    res["scenes"][sc] = {}
    for name, run in arms.items():
        m = sm.arm_metrics(run, syn, [sc])
        res["scenes"][sc][name] = {
            "tp": m.get("tp"), "fn": m.get("fn"), "fp": m.get("fp"), "idsw": m.get("idsw"), "frag": m.get("fragmentations"),
            "vel_all": m.get("velocity_all"), "vel_gt_stationary": m.get("velocity_gt_stationary"),
            "vel_gt_moving_vehicle": m.get("velocity_gt_moving_vehicle"),
            "stationary_false_moving_0p5": m.get("stationary", {}).get("false_moving_rate_0p5"),
            "center_vehicle": m.get("center_abs_err_vehicle"), "speed_lag": m.get("speed_lag_scans")}
json.dump(res, open(out, "w"), indent=1)
hdr = "%-28s %-8s %5s %5s %4s %4s %8s %8s %8s %8s %6s" % ("scene", "arm", "tp", "fn", "idsw", "frag", "vel_p50", "vel_p95", "stat_p95", "mov_p95", "fm0.5")
print(hdr)
for sc, d in res["scenes"].items():
    for name, m in d.items():
        f = lambda x, k: (x or {}).get(k, float("nan")) if x and x.get("n") else float("nan")
        print("%-28s %-8s %5s %5s %4s %4s %8.3f %8.3f %8.3f %8.3f %6.3f" % (
            sc, name, m["tp"], m["fn"], m["idsw"], m["frag"], f(m["vel_all"], "p50"), f(m["vel_all"], "p95"),
            f(m["vel_gt_stationary"], "p95"), f(m["vel_gt_moving_vehicle"], "p95"),
            m["stationary_false_moving_0p5"] if m["stationary_false_moving_0p5"] is not None else float("nan")))
