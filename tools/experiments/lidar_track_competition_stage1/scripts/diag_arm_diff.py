#!/usr/bin/env python3
"""Find the TP rows whose velocity error differs most between two arms (same scene) and print the
trace events of those tracks around the scan (read-only).
usage: diag_arm_diff.py RUN_A RUN_B SCENE [N]
"""
import json, os, sys
import numpy as np
import pandas as pd
ra, rb, sc = sys.argv[1:4]
n = int(sys.argv[4]) if len(sys.argv) > 4 else 5


def tp(run):
    t = pd.read_pickle(os.path.join(run, "samples", sc + ".pkl"))["tp"].copy()
    t["verr"] = np.hypot(t.pvx - t.gvx, t.pvy - t.gvy)
    return t.set_index(["idx", "gt_tid"])


a, b = tp(ra), tp(rb)
j = a[["verr", "pid", "kfc", "prev_miss", "gvx", "gvy", "pvx", "pvy"]].join(b[["verr", "pid", "pvx", "pvy"]], rsuffix="_b", how="inner")
j["d"] = j.verr_b - j.verr
top = j.sort_values("d", ascending=False).head(n)
print(top.round(3).to_string())


def events(run, scan, pid):
    out = []
    for line in open(os.path.join(run, "trace", sc + ".trace.jsonl")):
        e = json.loads(line)
        if abs(e["scan"] - scan) <= 3 and e.get("id") == pid:
            out.append(e)
    return out


for (idx, g), r in top.head(2).iterrows():
    for run, pid in ((ra, int(r.pid)), (rb, int(r.pid_b))):
        print(f"--- {os.path.basename(run)} idx={idx} gt={g} pid={pid}")
        for e in events(run, idx, pid):
            keep = {k: e[k] for k in ("ev", "scan", "kfc_in", "dt", "elapsed_before", "init_elapsed", "z", "pred", "x_post", "abs_v", "prev_miss", "miss") if k in e}
            print(json.dumps(keep)[:400])
