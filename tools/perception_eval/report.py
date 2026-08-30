#!/usr/bin/env python3
"""Aggregate metrics + failure gallery paths (STUB)."""
import argparse
import json
import sys
from pathlib import Path

def main():
    p = argparse.ArgumentParser(description="Report perception eval (stub)")
    p.add_argument("--run-dir", default="artifacts/perception_eval/runs/stub")
    args = p.parse_args()
    m = Path(args.run_dir) / "metrics.json"
    if m.is_file():
        print(json.dumps(json.loads(m.read_text()), indent=2))
    else:
        print(f"[report] no metrics at {m}")
    return 0

if __name__ == "__main__":
    sys.exit(main())
