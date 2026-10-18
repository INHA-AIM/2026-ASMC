#!/usr/bin/env python3
"""Development-only configuration selection (EVAL_CONTRACT tuning). Holdout, highspeed and the static bag are never read.

usage: tune_eval.py --contract C --manifest M --base BASE_SAMPLES --out OUT.json [--prior PRIOR.json]
                    NAME=CAND_SAMPLES [NAME=...]
  eligibility: every metric regression gate (recall/recall_roi overall <=0.5 pp, per class <=1.0 pp, precision <=+2%,
               vehicle center/long/short/yaw MAE and all-class center MAE <=+2%, idsw/fragmentation <=+2%,
               split/over-merge rate <=+5%, scans_dropped not increased) on the 79 development scenes
  rank:        (number of improvement criteria reaching their threshold, mean relative reduction of
                [vehicle long temporal p95, short temporal p95, yaw temporal p95, extreme size+yaw jump rate])
  --prior:     tune result of the ablation this one builds on (B on A): its selected config's dev metrics are the
               reference for B's extra gate (vehicle center MAE and idsw/fragmentation not worse than A by > 2%)
"""
import sys

sys.dont_write_bytecode = True

import argparse, json, os  # noqa: E402
import numpy as np  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import eval_lib as E  # noqa: E402
import compare as C  # noqa: E402

CRIT = {"size": (["long_temporal_p95_vehicle", "short_temporal_p95_vehicle"], 0.15),
        "yaw": (["yaw_temporal_p95_vehicle_deg"], 0.15),
        "jump": (["size_yaw_jump_rate_vehicle"], 0.25)}
SCORE_KEYS = ["long_temporal_p95_vehicle", "short_temporal_p95_vehicle", "yaw_temporal_p95_vehicle_deg", "size_yaw_jump_rate_vehicle"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--contract", required=True)
    ap.add_argument("--manifest", required=True)
    ap.add_argument("--base", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--prior")
    ap.add_argument("cands", nargs="+")
    a = ap.parse_args()
    c = json.load(open(a.contract))
    man = json.load(open(a.manifest))
    dev = E.scene_sets(man)["development"]
    g = c["decision"]["regression_gates"]
    mb = E.compute_metrics(E.load_samples(a.base, dev), c)
    prior = json.load(open(a.prior)) if a.prior else None
    ref = prior["configs"][prior["selected"]]["metrics"] if prior and prior.get("selected") else None
    out = {"set": "development", "n_scenes": len(dev), "baseline": mb, "configs": {},
           "prior": {"file": a.prior, "selected": prior.get("selected") if prior else None}}
    for spec in a.cands:
        name, d = spec.split("=", 1)
        mc = E.compute_metrics(E.load_samples(d, dev), c)
        gates = {}
        C.metric_gates(mb, mc, g, "", gates)
        fails = [k for k, v in gates.items() if not v["pass"]]
        if ref is not None:
            for k in ("center_mae_vehicle", "idsw_rate", "fragmentation_rate"):
                wr = C.worse_rel(k, ref.get(k), mc.get(k))
                if np.isfinite(wr) and wr > 0.02:
                    fails.append("vs_prior:" + k)
        red = {k: -C.rel(mb.get(k), mc.get(k)) for k in SCORE_KEYS}
        met = [n for n, (ks, thr) in CRIT.items() if all(np.isfinite(red[k]) and red[k] >= thr for k in ks)]
        score = float(np.mean([red[k] for k in SCORE_KEYS]))
        out["configs"][name] = {"samples": d, "metrics": mc, "gates": gates, "failed_gates": fails, "reductions": red,
                                "criteria_met": met, "eligible": not fails, "rank": [len(met), score]}
    elig = [n for n, v in out["configs"].items() if v["eligible"]]
    if elig:
        best = max(elig, key=lambda n: tuple(out["configs"][n]["rank"]))
        why = "eligible %s; best by (criteria met, mean reduction)" % elig
    else:
        best, why = None, "no eligible configuration on the development set"
    out["selected"], out["selection_reason"] = best, why
    json.dump(out, open(a.out, "w"), indent=1, default=float)
    for n, v in out["configs"].items():
        print(n, "eligible" if v["eligible"] else "FAIL%s" % v["failed_gates"], "met=%s" % v["criteria_met"],
              {k: round(x, 4) for k, x in v["reductions"].items()})
    print("SELECTED", best, "-", why)


if __name__ == "__main__":
    main()
