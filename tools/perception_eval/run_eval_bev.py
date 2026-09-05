#!/usr/bin/env python3
"""Eval 3-cam BEV vs GT mask (STUB)."""
import argparse
import sys

def main():
    p = argparse.ArgumentParser(description="Eval cam BEV IoU (stub)")
    p.add_argument("--images", default="")
    p.add_argument("--gt-mask", default="")
    p.add_argument("--out", default="artifacts/perception_eval/runs/stub")
    args = p.parse_args()
    print(f"[run_eval_bev] STUB out={args.out}")
    return 0

if __name__ == "__main__":
    sys.exit(main())
