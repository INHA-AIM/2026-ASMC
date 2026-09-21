#!/usr/bin/env python3
"""Build the current K-City 2025 competition-corridor nominal baseline."""

from __future__ import annotations

import argparse
from pathlib import Path

from asmc_hd_map.current_builder import build_current_corridor


def _default_repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def main() -> None:
    repo_root = _default_repo_root()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=repo_root)
    parser.add_argument(
        "--config", type=Path,
        default=Path("src/asmc_hd_map/config/kcity2025_competition_corridor.yaml"),
    )
    parser.add_argument(
        "--output-dir", type=Path,
        default=Path(
            "src/asmc_hd_map/reports/kcity2025_current_corridor_20260916"
        ),
    )
    args = parser.parse_args()
    root = args.repo_root.resolve()
    config = args.config if args.config.is_absolute() else root / args.config
    output = args.output_dir if args.output_dir.is_absolute() else root / args.output_dir
    summary = build_current_corridor(config.resolve(), root, output.resolve())
    print(f"[asmc_hd_map] wrote current-only candidate to {output}")
    print(
        "[asmc_hd_map] route_links={route} corridor_links={corridor} "
        "boundaries={boundaries} route_stop_nodes={stop_nodes}".format(
            route=summary["route"]["link_count"],
            corridor=summary["corridor"]["unique_link_count"],
            boundaries=summary["nominal_lane_boundaries"]["deduplicated_boundary_count"],
            stop_nodes=summary["stoplines"]["route_stop_node_count"],
        )
    )


if __name__ == "__main__":
    main()

