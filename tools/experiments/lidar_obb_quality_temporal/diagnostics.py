#!/usr/bin/env python3
"""Baseline diagnostics (EVAL_CONTRACT observability / split_overmerge / clustering_counterfactual / roi_boundary /
c_d_justification). Reads per-scene samples of ONE run; writes
  OUT/CLUSTER_OBSERVABILITY_BASELINE.json, OUT/CLUSTER_SPLIT_MERGE_ANALYSIS.json, OUT/ROI_BOUNDARY_ANALYSIS.json,
  OUT/CLUSTER_EVENTS.json (event list incl. the stratified review sample).
usage: diagnostics.py --contract C --manifest M --samples DIR --out-dir OUT [--review-n 60]
"""
import sys

sys.dont_write_bytecode = True

import argparse, json, os  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import eval_lib as E  # noqa: E402


def q(a, qs=(0, 10, 25, 50, 75, 90, 100)):
    a = np.asarray(a, dtype=float)
    a = a[np.isfinite(a)]
    return {str(x): (float(np.percentile(a, x)) if len(a) else None) for x in qs}


def bucket_metrics(tp, pairs):
    v, pv = tp[tp.cls == "vehicle"], pairs[pairs.cls == "vehicle"] if len(pairs) else pairs
    out = {"n_tp_vehicle": int(len(v)), "n_pairs_vehicle": int(len(pv))}
    if len(v):
        out.update({"center_mae_vehicle": E.mean(v.cerr), "long_mae_vehicle": E.mean(np.abs(v.dlong)),
                    "short_mae_vehicle": E.mean(np.abs(v.dshort)), "yaw_mae_vehicle_deg": E.mean(np.degrees(np.abs(v.yerr))),
                    "long_bias_vehicle": E.mean(v.dlong), "short_bias_vehicle": E.mean(v.dshort)})
    if len(pv):
        out.update({"long_temporal_p95_vehicle": E.pct(pv.dlong_res, 95), "short_temporal_p95_vehicle": E.pct(pv.dshort_res, 95),
                    "yaw_temporal_p95_vehicle_deg": E.pct(np.degrees(pv.ydelta), 95),
                    "size_yaw_jump_rate_vehicle": E.rate(pv.size_jump | pv.yaw_jump), "axis_flip_rate_vehicle": E.rate(pv.flip)})
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--contract", required=True)
    ap.add_argument("--manifest", required=True)
    ap.add_argument("--samples", required=True)
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--review-n", type=int, default=60)
    ap.add_argument("--tag", default="BASELINE")
    a = ap.parse_args()
    c = json.load(open(a.contract))
    man = json.load(open(a.manifest))
    sets = E.scene_sets(man)
    os.makedirs(a.out_dir, exist_ok=True)
    S = {name: E.load_samples(a.samples, sets[name]) for name in ("development", "holdout", "highspeed")}
    S["primary"] = S["development"] + S["holdout"]
    S["all"] = S["primary"] + S["highspeed"]
    # ------------------------------------------------------------------ observability (development = baseline distribution)
    obs = {"contract_version": c["version"], "set": "development (79 scenes)", "ring_note": c["inputs"]["lidar_ring"]}
    tp = E.cat(S["development"], "tp")
    pairs = E.cat(S["development"], "tpairs")
    v = tp[tp.cls == "vehicle"]
    obs["n_tp"], obs["n_tp_vehicle"] = int(len(tp)), int(len(v))
    obs["feature_distribution_vehicle_tp"] = {f: q(v["f_" + f]) for f in
                                              ("npts", "az_span_deg", "z_span_m", "az_bins", "el_bins", "fit_residual_m",
                                               "inlier_ratio", "aspect", "pca_ratio", "sensor_range_m") if "f_" + f in v}
    obs["feature_distribution_pedestrian_tp"] = {f: q(tp[tp.cls == "pedestrian"]["f_" + f]) for f in
                                                 ("npts", "az_span_deg", "z_span_m", "el_bins") if "f_" + f in tp}
    obs["visible_faces_vehicle_share"] = {str(int(k)): float(n) / max(1, len(v)) for k, n in
                                          v["f_visible_faces"].value_counts().sort_index().items()} if "f_visible_faces" in v else {}
    obs["prev_miss_vehicle_share"] = E.rate(v.prev_miss.dropna().astype(bool)) if "prev_miss" in v else None
    obs["ambiguity_vehicle_share"] = E.rate(v.ambiguity.dropna().astype(bool)) if "ambiguity" in v else None
    obs["pred_length_lt_width_rate_vehicle"] = E.rate(v.pred_w_lt_h)
    B = {}
    rng_b = c["observability"]["buckets"]["range_m"]
    age_b = c["observability"]["buckets"]["track_age_obs"]
    for lo, hi in rng_b:
        f = lambda df: (df["range"] >= lo) & (df["range"] < hi + (1e-9 if hi == 35 else 0))
        B["range_%d_%d" % (lo, hi)] = bucket_metrics(tp[f(tp)], pairs[f(pairs)] if len(pairs) else pairs)
    for name, (lo, hi) in age_b.items():
        f = lambda df: (df["obs_count"] >= lo) & (df["obs_count"] <= hi)
        B["age_" + name] = bucket_metrics(tp[f(tp)], pairs[f(pairs)] if len(pairs) else pairs)
    if "f_npts" in tp:
        qs = np.percentile(v.f_npts.dropna(), [0, 25, 50, 75, 100])
        obs["npts_quartile_edges_vehicle"] = [float(x) for x in qs]
        for i in range(4):
            lo, hi = qs[i], qs[i + 1]
            m = (tp.f_npts >= lo) & ((tp.f_npts < hi) if i < 3 else (tp.f_npts <= hi))
            keyset = set(zip(tp[m].scene, tp[m].idx, tp[m].seg))
            pm = pairs.apply(lambda r: (r.scene, r.idx + 1, r.seg) in keyset, axis=1) if len(pairs) else pairs
            B["npts_q%d" % (i + 1)] = bucket_metrics(tp[m], pairs[pm] if len(pairs) else pairs)
        for k in (0, 1, 2):
            m = tp.f_visible_faces == k
            keyset = set(zip(tp[m].scene, tp[m].idx, tp[m].seg))
            pm = pairs.apply(lambda r: (r.scene, r.idx + 1, r.seg) in keyset, axis=1) if len(pairs) else pairs
            B["visible_faces_%d" % k] = bucket_metrics(tp[m], pairs[pm] if len(pairs) else pairs)
        for name, m in (("aspect_lt_1p2", tp.f_aspect < 1.2), ("aspect_1p2_2", (tp.f_aspect >= 1.2) & (tp.f_aspect < 2.0)),
                        ("aspect_ge_2", tp.f_aspect >= 2.0), ("inlier_lt_0p6", tp.f_inlier_ratio < 0.6),
                        ("inlier_ge_0p6", tp.f_inlier_ratio >= 0.6), ("prev_miss", tp.prev_miss == True),  # noqa: E712
                        ("ambiguity", tp.ambiguity == 1)):
            keyset = set(zip(tp[m].scene, tp[m].idx, tp[m].seg))
            pm = pairs.apply(lambda r: (r.scene, r.idx + 1, r.seg) in keyset, axis=1) if len(pairs) else pairs
            B[name] = bucket_metrics(tp[m], pairs[pm] if len(pairs) else pairs)
    obs["buckets"] = B
    obs["note"] = ("bucket temporal metrics use the pair's second frame (k+1) for npts/visibility/aspect/inlier/prev_miss/"
                   "ambiguity buckets and the second frame's obs_count/range for age/range buckets; "
                   "long/short bias = mean(pred - gt): negative = under-sized (partial view)")
    json.dump(obs, open(os.path.join(a.out_dir, "CLUSTER_OBSERVABILITY_%s.json" % a.tag), "w"), indent=1, default=float)
    # ------------------------------------------------------------------ split / over-merge
    sm = {"contract_version": c["version"], "definitions": c["split_overmerge"], "sets": {}}
    for name in ("development", "primary", "holdout", "highspeed", "all"):
        g = E.cat(S[name], "gtobs")
        om = E.cat(S[name], "overmerge")
        if not len(g):
            continue
        res = {}
        for cl in ("all", "vehicle", "pedestrian"):
            t = g if cl == "all" else g[g.cls == cl]
            r = {"eligible_gt_obs": int(len(t))}
            for k in ("split", "vertical", "horizontal", "over_merge", "boundary"):
                ne, nf, nr = E.events(t, k)
                r[k] = {"frames": nf, "events": ne, "repeated_events": nr, "rate": E.rate(t[k])}
            by_r = {}
            for lo, hi in rng_b:
                tt = t[(t["range"] >= lo) & (t["range"] < hi + (1e-9 if hi == 35 else 0))]
                by_r["%d-%d" % (lo, hi)] = {"eligible": int(len(tt)), **{k: {"rate": E.rate(tt[k]), "events": E.events(tt, k)[0]}
                                                                        for k in ("split", "vertical", "horizontal", "over_merge")}}
            by_a = {}
            for an, (lo, hi) in age_b.items():
                tt = t[(t.main_obs >= lo) & (t.main_obs <= hi)]
                by_a[an] = {"eligible": int(len(tt)), **{k: {"rate": E.rate(tt[k]), "events": E.events(tt, k)[0]}
                                                         for k in ("split", "vertical", "horizontal", "over_merge")}}
            r["by_range"], r["by_age"] = by_r, by_a
            res[cl] = r
        res["over_merge_predictions"] = int(len(om))
        sm["sets"][name] = res
    # contamination of the baseline extreme jumps (development, vehicle)
    pv = pairs[pairs.cls == "vehicle"] if len(pairs) else pairs
    jj = (pv.size_jump | pv.yaw_jump) if len(pv) else pv
    cont = {"vehicle_jump_pairs": int(jj.sum()) if len(pv) else 0,
            "share_touching_split_or_overmerge": float((jj & pv.flagged).sum() / max(1, jj.sum())) if len(pv) else None,
            "share_touching_split": float((jj & pv.flag_split).sum() / max(1, jj.sum())) if len(pv) else None,
            "share_touching_overmerge": float((jj & pv.flag_om).sum() / max(1, jj.sum())) if len(pv) else None,
            "share_touching_boundary": float((jj & pv.flag_boundary).sum() / max(1, jj.sum())) if len(pv) else None}
    dev = sm["sets"]["development"]["vehicle"]
    ev_c = dev["split"]["events"] + dev["over_merge"]["events"]
    flag_obs = E.cat(S["development"], "gtobs")
    fv = flag_obs[flag_obs.cls == "vehicle"]
    rate_c = float((fv.split | fv.over_merge).mean()) if len(fv) else 0.0
    just_c = ev_c >= 100 and (rate_c >= 0.005 or (cont["share_touching_split_or_overmerge"] or 0) >= 0.10)
    ev_d = dev["boundary"]["events"]
    rate_d = float(fv.boundary.mean()) if len(fv) else 0.0
    just_d = ev_d >= 100 and (rate_d >= 0.005 or (cont["share_touching_boundary"] or 0) >= 0.10)
    sm["contamination_development"] = cont
    sm["c_justification"] = {"rule": c["c_d_justification"]["C_split_overmerge_recovery"], "vehicle_events_split_plus_overmerge": ev_c,
                             "flagged_vehicle_obs_rate": rate_c, "jump_share": cont["share_touching_split_or_overmerge"],
                             "justified": bool(just_c),
                             "status": "JUSTIFIED" if just_c else ("NOT_JUSTIFIED_BY_CURRENT_DATA" + (" (insufficient events)" if ev_c < 100 else " (events sufficient, rate/contamination below threshold)"))}
    # counterfactual (how much the adaptive clustering and the duplicate merge already prevent)
    cf = E.cat(S["development"], "cf")
    if len(cf):
        def gt_set(lst):
            return sorted(set(x for x in lst if x >= 0))
        merged = cf[cf.n_adapt >= 2]
        m_same = merged[merged.frag_dom_adapt.apply(lambda l: len(gt_set(l)) == 1 and all(x >= 0 for x in l))]
        m_diff = merged[merged.frag_dom_adapt.apply(lambda l: len(gt_set(l)) >= 2)]
        fixed_split = cf[(cf.n_adapt == 1) & (cf.n_fixed05 >= 2)]
        f_same = fixed_split[fixed_split.frag_dom_fixed05.apply(lambda l: len(gt_set(l)) == 1)]
        f_diff = fixed_split[fixed_split.frag_dom_fixed05.apply(lambda l: len(gt_set(l)) >= 2)]
        fm = E.cat(S["development"], "frames")
        val = fm[fm.processed & fm.clusters.notna()]
        sm["counterfactual_development"] = {
            "objects_analysed": int(len(cf)), "objects_on_gt": int((cf.dom_whole >= 0).sum()),
            "duplicate_merge": {"merged_objects": int(len(merged)), "same_gt_merges_prevented_split": int(len(m_same)),
                                "same_gt_vertical": int(m_same.merge_vertical.sum()), "different_gt_merges_caused_overmerge": int(len(m_diff)),
                                "unattributed": int(len(merged) - len(m_same) - len(m_diff)),
                                "scan_level_merges_reported_by_harness": int((val.clusters - val.det).sum()),
                                "note": "objects_analysed covers tracks within 40 m, x>=-1 m, <=6000 points; harness count covers all"},
            "adaptive_vs_fixed_eps_0p5": {"objects_fixed_would_split": int(len(fixed_split)),
                                          "of_which_one_gt_prevented_split": int(len(f_same)),
                                          "of_which_multi_gt_adaptive_merged_distinct_objects": int(len(f_diff)),
                                          "unattributed": int(len(fixed_split) - len(f_same) - len(f_diff))}}
    json.dump(sm, open(os.path.join(a.out_dir, "CLUSTER_SPLIT_MERGE_ANALYSIS%s.json" % ("" if a.tag == "BASELINE" else "_" + a.tag)), "w"),
              indent=1, default=float)
    # ------------------------------------------------------------------ ROI boundary
    rb = {"contract_version": c["version"], "definition": c["roi_boundary"],
          "geometry_note": "only the base_link y=+-20 m ROI edges intersect the +-90 deg / 35 m evaluation mask (x in [0, 28.7] m)", "sets": {}}
    for name in ("development", "primary", "all"):
        g = E.cat(S[name], "gtobs")
        t = E.cat(S[name], "tp")
        if not len(g):
            continue
        r = {}
        for cl in ("vehicle", "pedestrian"):
            gg = g[g.cls == cl]
            ne, nf, nr = E.events(gg, "boundary")
            r[cl] = {"eligible": int(len(gg)), "boundary_frames": nf, "boundary_events": ne, "repeated": nr,
                     "rate": E.rate(gg.boundary)}
            tt = t[t.cls == cl]
            if len(tt):
                eff = {}
                for lo, hi in rng_b:
                    w = tt[(tt["range"] >= lo) & (tt["range"] < hi + (1e-9 if hi == 35 else 0))]
                    for flag in (True, False):
                        x = w[w.boundary == flag]
                        eff["%d-%d_%s" % (lo, hi, "boundary" if flag else "inside")] = {
                            "n": int(len(x)), "long_mae": E.mean(np.abs(x.dlong)), "short_mae": E.mean(np.abs(x.dshort)),
                            "long_bias": E.mean(x.dlong), "center_mae": E.mean(x.cerr)}
                r[cl]["tp_error_effect"] = eff
        rb["sets"][name] = r
    rb["contamination_development"] = {"share_of_vehicle_jump_pairs_touching_boundary": cont["share_touching_boundary"]}
    rb["d_justification"] = {"rule": c["c_d_justification"]["D_roi_padding"], "vehicle_boundary_events": ev_d,
                             "boundary_vehicle_obs_rate": rate_d, "justified": bool(just_d),
                             "status": "JUSTIFIED" if just_d else ("NOT_JUSTIFIED_BY_CURRENT_DATA" + (" (insufficient events)" if ev_d < 100 else " (events sufficient, rate/contamination below threshold)"))}
    json.dump(rb, open(os.path.join(a.out_dir, "ROI_BOUNDARY_ANALYSIS%s.json" % ("" if a.tag == "BASELINE" else "_" + a.tag)), "w"),
              indent=1, default=float)
    # ------------------------------------------------------------------ event list + stratified review sample
    g = E.cat(S["all"], "gtobs")
    evs = []
    if len(g):
        for kind in ("vertical", "horizontal", "over_merge", "boundary"):
            x = g[g[kind]].sort_values(["scene", "seg", "idx"])
            for (sc, seg), gg in x.groupby(["scene", "seg"]):
                idx = gg.idx.to_numpy()
                starts = np.flatnonzero(np.r_[True, np.diff(idx) != 1])
                ends = np.r_[starts[1:], len(idx)] - 1
                for s0, s1 in zip(starts, ends):
                    r0 = gg.iloc[s0]
                    evs.append({"kind": kind, "scene": sc, "seg": seg, "gt_tid": int(r0.gt_tid), "cls": r0.cls,
                                "first_idx": int(idx[s0]), "last_idx": int(idx[s1]), "n_frames": int(s1 - s0 + 1),
                                "range": float(r0["range"]), "frag_pids": list(r0.frag_pids), "om_pid": int(r0.om_pid),
                                "role": man["scenes"][sc]["role"] if sc in man["scenes"] else "static"})
    rng = np.random.default_rng(20261018)
    review = []
    quota = {"vertical": 15, "horizontal": 15, "over_merge": 20, "boundary": 10}
    scale = a.review_n / sum(quota.values())
    for kind, qn in quota.items():
        cand = [e for e in evs if e["kind"] == kind and e["role"] != "holdout"]
        n = min(len(cand), int(round(qn * scale)))
        if n:
            for i in rng.choice(len(cand), n, replace=False):
                review.append(dict(cand[int(i)], review_id="R%03d" % len(review)))
    json.dump({"contract_version": c["version"], "n_events": len(evs), "events": evs, "review_sample": review,
               "review_note": "stratified sample (seed 20261018) from development+highspeed scenes; holdout excluded"},
              open(os.path.join(a.out_dir, "CLUSTER_EVENTS_%s.json" % a.tag), "w"), indent=1, default=float)
    print(json.dumps({"c": sm["c_justification"], "d": rb["d_justification"], "n_events": len(evs), "review": len(review),
                      "contamination": cont}, indent=1, default=float))


if __name__ == "__main__":
    main()
