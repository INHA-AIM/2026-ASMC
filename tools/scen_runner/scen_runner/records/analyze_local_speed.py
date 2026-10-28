"""LOCAL-2C longitudinal audit: fixed native interval, no success-only sampling.

Keep startup/invalid/short runs visible. The steady comparison starts exactly
10s after native actuation; never choose its start by whether tracking succeeded.
"""

import argparse
from collections import Counter
import json
from pathlib import Path
from .analyze_local import quantiles


def analyze(root, output=None):
    def rows(name):
        return [json.loads(x) for x in (root / name).read_text().splitlines()]

    native = [r for r in rows("events.jsonl") if r.get("event") == "native_sample"]
    if not native:
        return dict(run_id=root.name, status="no_native_drive")
    begin, end = native[0]["ros_time"], native[-1]["ros_time"]
    cap = json.loads((root / "trial-result.json").read_text())["cap_kph"]
    controls = [r for r in rows("control.jsonl") if begin <= r["decision_stamp"] <= end]
    steady = [r for r in native if r["ros_time"] >= begin + 10.0]
    commands = [r["decision_stamp"] for r in controls if r.get("command_published")]
    result = dict(
        run_id=root.name,
        native_interval=[begin, end],
        requested_cap_kph=cap,
        steady_start_s=10.0,
        steady_samples=len(steady),
        steady_speed_kph=quantiles([r["ego"]["speed_kmh"] for r in steady]),
        steady_cap_error_kph=quantiles(
            [abs(r["ego"]["speed_kmh"] - cap) for r in steady]
        ),
        hard_brake_ticks=sum(
            r.get("command_published") and r.get("brake", 0) > 0.9 for r in controls
        ),
        control_reasons=dict(Counter(r["reason"] for r in controls)),
        command_gap_ms=quantiles(
            [(b - a) * 1000 for a, b in zip(commands, commands[1:])]
        ),
        solve_ms=quantiles([r["processing_ms"] for r in controls if r.get("valid")]),
        series=[
            dict(
                t=r["decision_stamp"] - begin,
                speed_kph=r.get("speed_mps", 0) * 3.6,
                ref_kph=r.get("reference_speed_mps", 0) * 3.6,
                ref_a=r.get("reference_acceleration_mps2"),
                solve_a=r.get("solver_acceleration_mps2"),
                throttle=r.get("throttle"),
                brake=r.get("brake"),
                reason=r["reason"],
            )
            for r in controls
        ],
        limitation="Fixed elapsed window, not path/obstacle acceptance; expired inputs and holds retained.",
    )
    with Path(output or root / "speed-analysis.json").open("x") as f:
        json.dump(result, f, indent=2)
    return {k: v for k, v in result.items() if k != "series"}


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("root", type=Path)
    p.add_argument("--output", type=Path)
    args = p.parse_args()
    print(json.dumps(analyze(args.root, args.output), indent=2))
