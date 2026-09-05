#!/usr/bin/env python3
"""Eval drivable crop IoU under Loc-Aid / noisy / blackout poses (STUB)."""
import argparse
import json
import sys
from pathlib import Path

def main():
    p = argparse.ArgumentParser(description="Eval drivable IoU (stub)")
    p.add_argument("--bake", default="artifacts/perception_eval/bake")
    p.add_argument("--poses", default="")
    p.add_argument("--out", default="artifacts/perception_eval/runs/stub")
    args = p.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    metrics = {
        "run_id": "stub",
        "modules": ["mgeo_bake"],
        "scenarios": [{"name": "stub"}],
        "scores": {
            "drivable_iou_mean": None,
            "drivable_iou_p10": None,
            "oob_false_positive_rate": None,
            "crop_latency_ms_p95": None,
            "bev_iou_mean": None,
        },
        "gate": {
            "pass": False,
            "thresholds": {"drivable_iou_mean": 0.85, "drivable_iou_p10": 0.70},
            "note": "stub — not evaluated",
        },
    }
    (out / "metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")
    print(f"[run_eval_drivable] STUB wrote {out / 'metrics.json'}")
    return 0

if __name__ == "__main__":
    sys.exit(main())
