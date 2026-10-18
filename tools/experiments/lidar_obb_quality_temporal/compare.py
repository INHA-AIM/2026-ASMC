#!/usr/bin/env python3
"""Baseline vs candidate comparison and verdict, implementing EVAL_CONTRACT.json `decision` literally.

usage: compare.py --contract C --manifest M --base BASE_SAMPLES --cand CAND_SAMPLES --out-dir OUT
                  [--external EXTERNAL_GATES.json] [--cd CD_STATUS.json] [--bootstrap 10000]
EXTERNAL_GATES.json: {"existing_tests_new_failures": int, "sanitizer_new_signatures": int, "default_false_identical": bool,
  "contract_changes": int, "runtime_p95_rel_increase": float, "peak_rss_rel_increase": float, "udp_packets_sent": int}
CD_STATUS.json: {"C": {"status": ..., "evaluable_events": int, "selected": bool, "targeted_event_reduction": float|null}, "D": {...}}
Missing external gates are NOT_EVALUATED and block PASS. Development-only configuration selection is tune_eval.py.
"""
import sys

sys.dont_write_bytecode = True

import argparse, csv, json, os  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import eval_lib as E  # noqa: E402

SCENE_KEYS = ["recall", "precision", "recall_vehicle", "recall_pedestrian", "center_mae", "center_mae_vehicle",
              "long_mae_vehicle", "short_mae_vehicle", "yaw_mae_vehicle_deg", "corner_mae_vehicle",
              "long_temporal_p95_vehicle", "short_temporal_p95_vehicle", "yaw_temporal_p95_vehicle_deg",
              "size_yaw_jump_rate_vehicle", "axis_flip_rate_vehicle", "center_temporal_p95", "idsw_rate",
              "fragmentation_rate", "split_rate", "over_merge_rate", "scans_dropped", "tp", "fp", "fn",
              "temporal_pairs_vehicle", "track_ids_in_local"]
BUCKET_KEYS = ["center_mae_vehicle", "long_mae_vehicle", "short_mae_vehicle", "yaw_mae_vehicle_deg",
               "long_temporal_p95_vehicle", "short_temporal_p95_vehicle", "yaw_temporal_p95_vehicle_deg",
               "size_yaw_jump_rate_vehicle"]


def rel(b, c):
    if b is None or c is None or not np.isfinite(b) or not np.isfinite(c) or b == 0:
        return float("nan")
    return (c - b) / abs(b)


def worse_rel(metric, b, c):
    d = E.KEY.get(metric, -1)
    if b is not None and c is not None and np.isfinite(b) and np.isfinite(c) and b == 0:
        return 0.0 if (c <= 0 if d < 0 else c >= 0) else float("inf")
    r = rel(b, c)
    return -r if d > 0 else r


def metric_gates(b, cc, g, prefix, gates):
    def gate(name, ok, detail):
        gates[prefix + name] = {"pass": bool(ok), "detail": detail}
    pp = lambda k: 100.0 * (b[k] - cc[k])
    for k in ("recall", "recall_roi"):
        gate(k + "_drop_pp", pp(k) <= g["recall_overall_drop_pp_max"], {"drop_pp": pp(k), "max": g["recall_overall_drop_pp_max"]})
    for k in ("recall_vehicle", "recall_pedestrian", "recall_roi_vehicle", "recall_roi_pedestrian"):
        if np.isfinite(b.get(k, np.nan)):
            gate(k + "_drop_pp", pp(k) <= g["recall_class_drop_pp_max"], {"drop_pp": pp(k), "max": g["recall_class_drop_pp_max"]})
    wr = worse_rel("precision", b["precision"], cc["precision"])
    gate("precision_worsen_rel", wr <= g["precision_worsen_rel_max"], {"worsen_rel": wr, "base": b["precision"], "cand": cc["precision"]})
    for k in g["vehicle_accuracy_metrics"] + g["extra_accuracy_metrics_same_limit"]:
        wr = worse_rel(k, b.get(k), cc.get(k))
        gate(k + "_worsen_rel", wr <= g["vehicle_accuracy_worsen_rel_max"],
             {"worsen_rel": wr, "max": g["vehicle_accuracy_worsen_rel_max"], "base": b.get(k), "cand": cc.get(k)})
    for k in ("idsw_rate", "fragmentation_rate"):
        wr = worse_rel(k, b.get(k), cc.get(k))
        gate(k + "_worsen_rel", wr <= g["idsw_fragmentation_rate_worsen_rel_max"], {"worsen_rel": wr, "base": b.get(k), "cand": cc.get(k)})
    for k in ("split_rate", "over_merge_rate"):
        if k in b:
            wr = worse_rel(k, b.get(k), cc.get(k))
            gate(k + "_worsen_rel", wr <= g["split_overmerge_rate_worsen_rel_max"], {"worsen_rel": wr, "base": b.get(k), "cand": cc.get(k)})
    gate("new_scan_drops", cc["scans_dropped"] <= b["scans_dropped"], {"base": b["scans_dropped"], "cand": cc["scans_dropped"]})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--contract", required=True)
    ap.add_argument("--manifest", required=True)
    ap.add_argument("--base", required=True)
    ap.add_argument("--cand", required=True)
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--external")
    ap.add_argument("--cd")
    ap.add_argument("--bootstrap", type=int, default=10000)
    a = ap.parse_args()
    c = json.load(open(a.contract))
    man = json.load(open(a.manifest))
    sets = E.scene_sets(man)
    dec = c["decision"]
    g = dec["regression_gates"]
    os.makedirs(a.out_dir, exist_ok=True)
    res = {"contract_version": c["version"], "sets": {}, "gates": {}, "improvement": {}}
    samples, bd = {}, {}
    for name in ("primary", "development", "holdout", "highspeed", "all"):
        bs, cs = E.load_samples(a.base, sets[name]), E.load_samples(a.cand, sets[name])
        samples[name] = (bs, cs)
        mb, mc = E.compute_metrics(bs, c), E.compute_metrics(cs, c)
        res["sets"][name] = {"baseline": mb, "candidate": mc,
                             "rel_change": {k: rel(mb.get(k), mc.get(k)) for k in mb if isinstance(mb.get(k), (int, float))}}
    for name in ("primary", "all"):
        bd[name] = (E.breakdowns(samples[name][0], c), E.breakdowns(samples[name][1], c))
    sbp, scp = os.path.join(a.base, "static_bag.pkl"), os.path.join(a.cand, "static_bag.pkl")
    if os.path.exists(sbp) and os.path.exists(scp):
        mb, mc = E.compute_metrics([pd.read_pickle(sbp)], c), E.compute_metrics([pd.read_pickle(scp)], c)
        res["static_bag"] = {"baseline": mb, "candidate": mc}
    # ---------------- per-scene and per range/age tables
    rows = []
    per_scene = {}
    for s in sets["all"] + (["static_bag"] if "static_bag" in res else []):
        mb = E.compute_metrics(E.load_samples(a.base, [s]), c)
        mc = E.compute_metrics(E.load_samples(a.cand, [s]), c)
        per_scene[s] = (mb, mc)
        r = {"scene": s, "role": man["scenes"][s]["role"] if s in man["scenes"] else "static_bag"}
        for k in SCENE_KEYS:
            r["base_" + k], r["cand_" + k], r["rel_" + k] = mb.get(k), mc.get(k), rel(mb.get(k), mc.get(k))
        rows.append(r)
    with open(os.path.join(a.out_dir, "COMPARISON_BY_SCENE.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader(); w.writerows(rows)
    crow = []
    for setname in ("primary", "all"):
        bb, cb = bd[setname]
        for group in ("by_class", "by_range", "by_age", "by_speed"):
            for key in bb[group]:
                mb, mc = bb[group][key], cb[group][key]
                r = {"set": setname, "group": group, "bucket": key}
                for k in ["tp", "fn", "recall", "recall_vehicle"] + BUCKET_KEYS + ["center_temporal_p95_vehicle", "axis_flip_rate_vehicle",
                                                                                  "temporal_pairs_vehicle", "split_rate", "over_merge_rate"]:
                    if group == "by_age" and k.startswith("recall"):
                        continue
                    r["base_" + k], r["cand_" + k], r["rel_" + k] = mb.get(k), mc.get(k), rel(mb.get(k), mc.get(k))
                crow.append(r)
        for key in ("pooled_obs_1_5", "pooled_range_15_35"):
            mb, mc = bb[key], cb[key]
            r = {"set": setname, "group": "pooled", "bucket": key}
            for k in ["tp", "recall_vehicle"] + BUCKET_KEYS:
                r["base_" + k], r["cand_" + k], r["rel_" + k] = mb.get(k), mc.get(k), rel(mb.get(k), mc.get(k))
            crow.append(r)
    keys = sorted({k for r in crow for k in r}, key=lambda k: (k not in ("set", "group", "bucket"), k))
    with open(os.path.join(a.out_dir, "COMPARISON_BY_RANGE_AGE.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader(); w.writerows(crow)
    # ---------------- regression gates
    gates = {}
    for setname in g["evaluated_on"]:
        S = res["sets"][setname]
        metric_gates(S["baseline"], S["candidate"], g, setname + ":", gates)
    H = res["sets"]["highspeed"]
    hs = {}
    if H["baseline"]["n_scenes"]:
        metric_gates(H["baseline"], H["candidate"], dict(g, recall_overall_drop_pp_max=2.0, recall_class_drop_pp_max=2.0,
                                                         precision_worsen_rel_max=0.10, vehicle_accuracy_worsen_rel_max=0.10,
                                                         idsw_fragmentation_rate_worsen_rel_max=0.10,
                                                         split_overmerge_rate_worsen_rel_max=0.10), "highspeed_stress:", hs)
    else:
        hs["highspeed_stress:not_evaluated"] = {"pass": False, "detail": "highspeed set empty"}
    gates.update(hs)
    hold = []
    for s in sets["holdout"]:
        mb, mc = per_scene[s]
        for k in g["holdout_core_metrics"]:
            wr = worse_rel(k, mb.get(k), mc.get(k))
            hold.append({"scene": s, "metric": k, "base": mb.get(k), "cand": mc.get(k), "worsen_rel": wr,
                         "pass": (not np.isfinite(wr)) or wr <= g["holdout_scene_core_metric_worsen_rel_max"]})
    gates["holdout:per_scene_core_metrics"] = {"pass": all(h["pass"] for h in hold), "detail": hold}
    res["gates"]["metric"] = gates
    # ---------------- external gates
    ext = json.load(open(a.external)) if a.external and os.path.exists(a.external) else {}
    eg = {}
    def ext_gate(key, ok_fn, limit):
        if key not in ext:
            eg[key] = {"pass": False, "status": "NOT_EVALUATED", "limit": limit}
        else:
            eg[key] = {"pass": bool(ok_fn(ext[key])), "value": ext[key], "limit": limit}
    ext_gate("existing_tests_new_failures", lambda v: v == 0, 0)
    ext_gate("sanitizer_new_signatures", lambda v: v == 0, 0)
    ext_gate("default_false_identical", lambda v: v is True, True)
    ext_gate("contract_changes", lambda v: v == 0, 0)
    ext_gate("runtime_p95_rel_increase", lambda v: v <= g["runtime_p95_increase_rel_max"], g["runtime_p95_increase_rel_max"])
    ext_gate("peak_rss_rel_increase", lambda v: v <= g["peak_rss_increase_rel_max"], g["peak_rss_increase_rel_max"])
    ext_gate("udp_packets_sent", lambda v: v == 0, 0)
    res["gates"]["external"] = eg
    res["gates"]["external_evidence"] = ext.get("evidence")
    # ---------------- improvement criteria
    P = res["sets"]["primary"]
    boot = E.bootstrap(*samples["primary"], a.bootstrap, c["bootstrap"]["seed"]) if a.bootstrap else {}
    res["bootstrap_primary"] = boot
    crit = {"size": (["long_temporal_p95_vehicle", "short_temporal_p95_vehicle"], 0.15),
            "yaw": (["yaw_temporal_p95_vehicle_deg"], 0.15),
            "jump": (["size_yaw_jump_rate_vehicle"], 0.25)}
    imp = {}
    for name, (ks, thr) in crit.items():
        reds = {k: -rel(P["baseline"].get(k), P["candidate"].get(k)) for k in ks}
        meets = all(np.isfinite(v) and v >= thr for v in reds.values())
        dirs = []
        for s in sets["holdout"]:
            mb, mc = per_scene[s]
            ok = all(np.isfinite(mb.get(k, np.nan)) and np.isfinite(mc.get(k, np.nan)) and mc[k] < mb[k] for k in ks)
            dirs.append({"scene": s, **{"base_" + k: mb.get(k) for k in ks}, **{"cand_" + k: mc.get(k) for k in ks}, "improved": bool(ok)})
        cis = {k: boot.get(k, {}).get("ci95") for k in ks}
        ci_ok = all(ci is not None and ci[1] < 0.0 for ci in cis.values())
        n_imp = sum(d["improved"] for d in dirs)
        imp[name] = {"metrics": ks, "threshold": thr, "reduction_rel": reds,
                     "base": {k: P["baseline"].get(k) for k in ks}, "cand": {k: P["candidate"].get(k) for k in ks},
                     "meets_threshold": bool(meets), "bootstrap_ci95_rel_change": cis, "ci_below_zero": bool(ci_ok),
                     "holdout_improved_scenes": n_imp, "holdout": dirs,
                     "holdout_consistent": n_imp >= dec["improvement_criteria_on_primary"]["holdout_direction_consistency_min"]}
        imp[name]["fully_satisfied"] = bool(meets and ci_ok and imp[name]["holdout_consistent"])
    res["improvement"] = imp
    # ---------------- additional conditions
    add = {}
    bb, cb = bd["primary"]
    for key in ("pooled_obs_1_5", "pooled_range_15_35"):
        mb, mc = bb[key], cb[key]
        fails = {}
        for k in BUCKET_KEYS:
            wr = worse_rel(k, mb.get(k), mc.get(k))
            if np.isfinite(wr) and wr > 0.10:
                fails[k] = wr
        if key == "pooled_range_15_35":
            d = 100.0 * (mb.get("recall_vehicle", np.nan) - mc.get("recall_vehicle", np.nan))
            if np.isfinite(d) and d > 2.0:
                fails["recall_vehicle_drop_pp"] = d
        add[key] = {"pass": not fails, "large_regressions": fails}
    if "static_bag" in res:
        sbm, scm = res["static_bag"]["baseline"], res["static_bag"]["candidate"]
        fails = {}
        for k in ("long_temporal_p95", "short_temporal_p95", "yaw_temporal_p95_deg", "stationary_false_moving_rate"):
            b_, c_ = sbm.get(k), scm.get(k)
            if b_ is not None and c_ is not None and np.isfinite(b_) and np.isfinite(c_) and c_ > b_ + 1e-12:
                fails[k] = {"base": b_, "cand": c_}
        add["static_bag"] = {"pass": not fails, "worsened": fails}
    else:
        add["static_bag"] = {"pass": False, "status": "NOT_EVALUATED"}
    cd = json.load(open(a.cd)) if a.cd and os.path.exists(a.cd) else {}
    cd_insufficient = []
    for key in ("C", "D"):
        st = cd.get(key, {})
        if st.get("selected"):
            red = st.get("targeted_event_reduction")
            add["targeted_events_" + key] = {"pass": red is not None and red >= 0.20, "reduction": red}
        if st.get("evaluable_events", 0) < 100 and not st.get("selected"):
            cd_insufficient.append(key)
    res["additional_conditions"] = add
    res["cd_status"] = cd
    # ---------------- verdict
    reg_fail = [k for k, v in gates.items() if not v["pass"]] + [k for k, v in eg.items() if not v["pass"]]
    add_fail = [k for k, v in add.items() if not v["pass"]]
    thr_met = [k for k, v in imp.items() if v["meets_threshold"]]
    full = [k for k, v in imp.items() if v["fully_satisfied"]]
    if reg_fail:
        verdict, why = "REJECTED_EXPERIMENT", "regression gate failed: %s" % reg_fail
    elif add_fail:
        verdict, why = "REJECTED_EXPERIMENT", "additional condition failed (bucket/static bag/targeted events): %s" % add_fail
    elif len(thr_met) < dec["improvement_criteria_on_primary"]["required_count"]:
        verdict, why = "REJECTED_EXPERIMENT", "criteria reaching the primary threshold: %s (< 2)" % thr_met
    elif len(full) < dec["improvement_criteria_on_primary"]["required_count"]:
        verdict, why = "INCONCLUSIVE", "thresholds met %s but CI/holdout fully satisfied only for %s" % (thr_met, full)
    elif cd_insufficient:
        verdict, why = "PARTIAL_PASS_FOLLOWUP_REQUIRED", "A/B satisfied %s; %s could not be judged (evaluable events < 100)" % (full, cd_insufficient)
    else:
        verdict, why = "EXPERIMENT_PASS_CANDIDATE_ONLY", "all gates pass; fully satisfied criteria %s" % full
    res.update({"thresholds_met": thr_met, "fully_satisfied": full, "regression_failures": reg_fail,
                "additional_failures": add_fail, "verdict": verdict, "verdict_reason": why})
    json.dump(res, open(os.path.join(a.out_dir, "COMPARISON.json"), "w"), indent=1, default=float)
    print(json.dumps({"verdict": verdict, "reason": why,
                      "improvement": {k: {kk: v[kk] for kk in ("reduction_rel", "meets_threshold", "ci_below_zero",
                                                               "holdout_improved_scenes", "bootstrap_ci95_rel_change")} for k, v in imp.items()},
                      "failed_gates": reg_fail, "additional_failures": add_fail}, indent=1, default=float))


if __name__ == "__main__":
    main()
