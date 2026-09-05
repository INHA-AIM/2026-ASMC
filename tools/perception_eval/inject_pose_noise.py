#!/usr/bin/env python3
"""Inject GPS noise / blackout flags into pose logs (STUB)."""
import argparse
import sys

def main():
    p = argparse.ArgumentParser(description="Inject pose noise (stub)")
    p.add_argument("--pose-log", default="", help="Input pose log")
    p.add_argument("--sigma", type=float, default=1.0)
    p.add_argument("--blackout", action="store_true")
    p.add_argument("--out", default="artifacts/perception_eval/poses_noisy")
    args = p.parse_args()
    print(f"[inject_pose_noise] STUB sigma={args.sigma} blackout={args.blackout}")
    return 0

if __name__ == "__main__":
    sys.exit(main())
