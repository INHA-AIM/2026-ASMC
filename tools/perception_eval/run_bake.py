#!/usr/bin/env python3
"""MGeo → bake artifact (STUB). See docs/records/archive/agentic-loop.md"""
import argparse
import sys

def main():
    p = argparse.ArgumentParser(description="Bake MGeo drivable layer (stub)")
    p.add_argument("--mgeo-root", default="", help="Path to MGeo / road_mesh JSON")
    p.add_argument("--out", default="artifacts/perception_eval/bake", help="Output dir")
    args = p.parse_args()
    print(f"[run_bake] STUB mgeo={args.mgeo_root!r} out={args.out!r}")
    print("[run_bake] Not implemented: write bake artifact")
    return 0

if __name__ == "__main__":
    sys.exit(main())
