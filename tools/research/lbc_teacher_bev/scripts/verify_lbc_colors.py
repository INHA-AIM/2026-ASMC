#!/usr/bin/env python3
"""Verify LBC BEV visualization colors match official LearningByCheating."""
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from lbc_bev.spec import BACKGROUND, COLORS

OFFICIAL_BACKGROUND = [0, 47, 0]
OFFICIAL_COLORS = [
    (102, 102, 102),
    (253, 253, 17),
    (204, 6, 5),
    (250, 210, 1),
    (39, 232, 51),
    (0, 0, 142),
    (220, 20, 60),
]
NAMES = ["road", "lane", "tl_red", "tl_yellow", "tl_green", "vehicle", "pedestrian"]


def main() -> int:
    ok = True
    if list(BACKGROUND) != OFFICIAL_BACKGROUND:
        print("FAIL BACKGROUND", list(BACKGROUND), "!=", OFFICIAL_BACKGROUND)
        ok = False
    else:
        print("OK BACKGROUND", list(BACKGROUND))

    for i, name in enumerate(NAMES):
        ours = tuple(int(c) for c in COLORS[i])
        ref = OFFICIAL_COLORS[i]
        if ours != ref:
            print(f"FAIL ch{i} {name}: {ours} != {ref}")
            ok = False
        else:
            print(f"OK ch{i} {name}: RGB{ref}")

    print()
    print("Reference: LearningByCheating/bird_view/utils/carla_utils.py")
    print("  road=gray, lane=yellow, vehicles=blue (0,0,142), pedestrians=crimson")
    print("  background=dark green (0,47,0)")
    print("  (No ego layer in official LBC visualization.)")
    print()
    print("ALL PASSED" if ok else "FAILED")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
