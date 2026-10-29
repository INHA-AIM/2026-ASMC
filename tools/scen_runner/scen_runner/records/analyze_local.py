"""Bounded LOCAL-1 trace metrics. No closed-loop success inferred from a bag."""

import argparse
from collections import Counter
import json
from pathlib import Path


def quantiles(values):
    x = sorted(values)
    return {
        k: x[min(len(x) - 1, round((len(x) - 1) * q))] if x else None
        for k, q in [("p50", 0.5), ("p95", 0.95), ("p99", 0.99), ("max", 1)]
    }


def analyze(root):
    root = Path(root)
    reasons = Counter()
    reject = Counter()
    speed = Counter()
    timing = []
    caps = []
    gaps = []
    count = valid = assisted = dropped = discontinuities = 0
    last = None
    last_frame = None
    examples = {}
    previous = None
    for line in (root / "decisions.jsonl").open():
        row = json.loads(line)
        count += 1
        valid += bool(row["valid"])
        assisted += bool(row.get("assisted"))
        dropped = max(dropped, row.get("journal_dropped", 0))
        reasons[row["reason"]] += 1
        reject.update(row.get("rejected", {}))
        speed[row.get("speed_reason", "unavailable")] += 1
        if row["processing_ms"] > 0:
            timing.append(row["processing_ms"])
        if row["valid"]:
            caps.append(row["selected_kph"])
        t = row["decision_stamp"]
        frame = row["frame"]
        if frame == last_frame and last is not None:
            if last > 0 and 0 <= t - last <= 10:
                gaps.append((t - last) * 1000)
            else:
                discontinuities += 1
        last = t
        last_frame = frame
        key = (row["reason"], next(iter(row.get("rejected", {})), ""))
        if key not in examples:
            examples[key] = row
    control = []
    if (root / "control.jsonl").exists():
        for line in (root / "control.jsonl").open():
            row = json.loads(line)
            if row.get("valid", True):
                control.append(row["processing_ms"])
    result = dict(
        plans=count,
        valid=valid,
        assisted=assisted,
        reasons=dict(reasons),
        rejections=dict(reject),
        speed_reasons=dict(speed),
        planning_ms=quantiles(timing),
        selected_kph=quantiles(caps),
        plan_gap_ms=quantiles(gaps),
        clock_discontinuities=discontinuities,
        mpc_ms=quantiles(control),
        journal_dropped=dropped,
        examples=list(examples.values()),
        acceptance="analysis only; closed-loop and physical contact require simulator evidence",
    )
    target = root / "summary.json"
    with target.open("x") as f:
        json.dump(result, f, ensure_ascii=False, indent=2)
    return {k: v for k, v in result.items() if k != "examples"}


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("root", type=Path)
    print(json.dumps(analyze(p.parse_args().root), ensure_ascii=False, indent=2))
