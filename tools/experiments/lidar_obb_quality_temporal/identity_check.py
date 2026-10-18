#!/usr/bin/env python3
"""Default-false identity: candidate (all new features off) harness output must equal the baseline output byte for
byte once the per-scan wall-clock "timing" object is removed (the only non-deterministic field), and the binary
point dumps (*.pts.bin) must be byte-identical.
usage: identity_check.py BASE_OUT_DIR CAND_OUT_DIR OUT.json
"""
import sys

sys.dont_write_bytecode = True

import hashlib, json, os, re  # noqa: E402

TIMING = re.compile(r',"timing":\{[^}]*\}')


def digest(path):
    h = hashlib.sha256(); n = 0
    with open(path, "rb") as f:
        for line in f:
            h.update(TIMING.sub("", line.decode("utf-8")).encode("utf-8")); n += 1
    return h.hexdigest(), n


def sha(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for b in iter(lambda: f.read(1 << 22), b""):
            h.update(b)
    return h.hexdigest()


def first_diff(a, b):
    with open(a) as fa, open(b) as fb:
        for i, (x, y) in enumerate(zip(fa, fb)):
            x, y = TIMING.sub("", x), TIMING.sub("", y)
            if x != y:
                k = next((j for j in range(min(len(x), len(y))) if x[j] != y[j]), min(len(x), len(y)))
                return {"line": i, "offset": k, "base": x[max(0, k - 120):k + 120], "cand": y[max(0, k - 120):k + 120]}
    return {"line": None, "note": "different line counts"}


def main(base, cand, out):
    names = sorted(f for f in os.listdir(base) if f.endswith(".jsonl"))
    res = {"compared_jsonl": 0, "identical_jsonl": 0, "compared_points": 0, "identical_points": 0, "different": [],
           "missing_in_candidate": [], "lines": 0}
    for f in names:
        p, q = os.path.join(base, f), os.path.join(cand, f)
        if not os.path.exists(q):
            res["missing_in_candidate"].append(f); continue
        (hb, nb), (hc, nc) = digest(p), digest(q)
        res["compared_jsonl"] += 1; res["lines"] += nb
        if hb == hc and nb == nc:
            res["identical_jsonl"] += 1
        else:
            res["different"].append({"file": f, "base_lines": nb, "cand_lines": nc, "first_diff": first_diff(p, q)})
        pb, pc = p[:-len(".jsonl")] + ".pts.bin", q[:-len(".jsonl")] + ".pts.bin"
        if os.path.exists(pb):
            res["compared_points"] += 1
            if not os.path.exists(pc):
                res["missing_in_candidate"].append(os.path.basename(pc))
            elif sha(pb) == sha(pc):
                res["identical_points"] += 1
            else:
                res["different"].append({"file": os.path.basename(pb), "note": "points dump differs"})
    res["all_identical"] = (res["compared_jsonl"] == len(names) and not res["different"] and not res["missing_in_candidate"])
    json.dump(res, open(out, "w"), indent=1)
    print(json.dumps({k: v for k, v in res.items() if k != "different"}), "different=%d" % len(res["different"]))


if __name__ == "__main__":
    main(*sys.argv[1:4])
