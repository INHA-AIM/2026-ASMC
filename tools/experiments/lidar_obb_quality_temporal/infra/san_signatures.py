#!/usr/bin/env python3
"""Sanitizer report signatures (kind + summary + first non-sanitizer frames) per binary, baseline vs candidate.
A candidate report is NEW when its signature does not occur in the baseline run (any binary).
usage: san_signatures.py BASE_SAN_ROOT CAND_SAN_ROOT OUT.json   (roots contain one directory per binary/run)
"""
import json, os, re, sys

FRAME = re.compile(r"^\s+#\d+ 0x[0-9a-f]+ in (\S+)")


def sigs(root):
    out = {}
    for b in sorted(os.listdir(root)):
        d = os.path.join(root, b)
        if not os.path.isdir(d):
            continue
        s = []
        for f in sorted(os.listdir(d)):
            t = open(os.path.join(d, f), errors="replace").read()
            for block in re.split(r"(?m)^(?==+\n==\d+==ERROR|\S*runtime error)", t):
                kind = re.search(r"ERROR: (\w+Sanitizer): ([\w-]+)|(runtime error: [^\n]{0,80})", block)
                if not kind:
                    continue
                frames = [m.group(1) for m in map(FRAME.match, block.splitlines()) if m and "sanitizer" not in m.group(1)
                          and not m.group(1).startswith(("operator", "__interceptor", "malloc", "calloc"))][:3]
                summ = re.search(r"SUMMARY: \w+: (.*)", block)
                s.append({"kind": kind.group(0)[:120], "frames": frames, "summary": summ.group(1)[:160] if summ else None})
        out[b] = s
    return out


def key(x):
    return (x["kind"].split(":")[0] if "runtime error" not in x["kind"] else x["kind"], tuple(x["frames"]), x["summary"])


def main(base, cand, out):
    sb, sc = sigs(base), sigs(cand)
    base_keys = {key(x) for v in sb.values() for x in v}
    new = {b: [x for x in v if key(x) not in base_keys] for b, v in sc.items()}
    new = {b: v for b, v in new.items() if v}
    rep = {"baseline_reports": sum(len(v) for v in sb.values()), "candidate_reports": sum(len(v) for v in sc.values()),
           "distinct_baseline_signatures": [list(k) for k in sorted(base_keys, key=str)],
           "candidate_signatures_by_binary": {b: [list(key(x)) for x in v] for b, v in sc.items() if v},
           "new_in_candidate": new, "new_count": sum(len(v) for v in new.values())}
    json.dump(rep, open(out, "w"), indent=1)
    print(json.dumps({k: rep[k] for k in ("baseline_reports", "candidate_reports", "new_count")}),
          "distinct_baseline_signatures=%d" % len(base_keys))


if __name__ == "__main__":
    main(*sys.argv[1:4])
