"""Separate home-lane error from the receding trajectory's near-zero start error.

Only native-sampled drive interval is scored; preparation stops remain recorded.
Family-relative error is internal consistency, not an independent lane GT.
"""

import argparse
from collections import Counter
import json
import math
from pathlib import Path
from scen_runner.records.analyze_local import quantiles


def analyze(root):
    root = Path(root)
    native = [json.loads(x) for x in (root / "events.jsonl").read_text().splitlines()]
    native = [r for r in native if r.get("event") == "native_sample"]
    if not native:
        raise ValueError("no native drive samples")
    begin, end = native[0]["ros_time"], native[-1]["ros_time"]
    plans = [json.loads(x) for x in (root / "decisions.jsonl").read_text().splitlines()]
    control = [json.loads(x) for x in (root / "control.jsonl").read_text().splitlines()]
    control = [r for r in control if begin <= r["decision_stamp"] <= end]
    errors, heading, samples = [], [], []
    for p in plans:
        if not p["valid"] or not begin <= p["decision_stamp"] <= end:
            continue
        h = p.get("scene", {}).get("home", [])
        if len(h) < 2:
            continue
        a, b = h[0], h[1]
        for l, r in zip(h, h[1:]):
            if l[0] <= 0 <= r[0]:
                a, b = l, r
                break
        if abs(a[0]) > 1.0 or b[0] - a[0] < 0.01:
            continue
        slope = (b[1] - a[1]) / (b[0] - a[0])
        e = a[1] - a[0] * slope
        errors.append(abs(e))
        heading.append(abs(math.atan(slope)) * 180 / math.pi)
        samples.append(
            dict(
                time_s=p["decision_stamp"] - begin,
                home_y_m=e,
                home_heading_deg=math.atan(slope) * 180 / math.pi,
            )
        )
    actuated = sum(
        abs(r["ego"].get("steer", 0)) > 0.001
        or r["ego"].get("throttle", 0) > 0.01
        or r["ego"].get("brake", 0) > 0.01
        for r in native
    )
    result = dict(
        drive_seconds=end - begin,
        native_actuated_samples=actuated,
        native_samples=len(native),
        home_error_abs_m=quantiles(errors),
        home_heading_abs_deg=quantiles(heading),
        control_reasons=dict(Counter(r["reason"] for r in control)),
        hard_brake_ticks=sum(r.get("brake", 0) > 0.9 for r in control),
        steering_abs_rad=quantiles([abs(r.get("steering", 0)) for r in control]),
        actual_speed_kph=quantiles([r["ego"]["speed_kmh"] for r in native]),
        native_contacts=sorted(
            set(
                str(c)
                for r in native
                for c in r["ego"].get("native_contacts", [])
                if str(c) != "Ego"
            )
        ),
        evidence="family-relative, not independent lane GT; native contacts and actuation retained",
        samples=samples,
    )
    with (root / "following-analysis.json").open("x") as f:
        json.dump(result, f, indent=2)
    return {k: v for k, v in result.items() if k != "samples"}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path)
    print(json.dumps(analyze(parser.parse_args().root), indent=2))
