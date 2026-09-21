#!/usr/bin/env python3

"""Build the Won-tae-observation-anchored HD-map candidate and raster layers."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def main() -> int:
    root = _repo_root()
    package_src = root / "src/asmc_hd_map/src"
    if str(package_src) not in sys.path:
        sys.path.insert(0, str(package_src))
    from asmc_hd_map.operational_map import build_operational_map

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config", type=Path,
        default=root / "src/asmc_hd_map/config/kcity2025_operational_map.yaml",
    )
    parser.add_argument(
        "--output", type=Path,
        default=root / "src/asmc_hd_map/reports/kcity2025_operational_map_20260916",
    )
    args = parser.parse_args()
    summary = build_operational_map(args.config.resolve(), root, args.output.resolve())
    print(
        "operational candidate built: "
        f"lanes={summary['counts']['lane_area']} "
        f"boundaries={summary['counts']['logical_lane_boundary']} "
        f"markings={summary['counts']['lane_marking']} "
        f"operational={summary['operational']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
