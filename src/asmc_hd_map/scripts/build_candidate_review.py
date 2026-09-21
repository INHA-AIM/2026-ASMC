#!/usr/bin/env python3
"""Build section overlays and inventory for the K-City HD-map candidate."""

from __future__ import annotations

import argparse
from pathlib import Path

from asmc_hd_map.review_bundle import build_candidate_review


def main() -> None:
    repo_root = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=repo_root)
    parser.add_argument(
        "--config", type=Path,
        default=Path("src/asmc_hd_map/config/kcity2025_candidate_review.yaml"),
    )
    parser.add_argument(
        "--output-dir", type=Path,
        default=Path("src/asmc_hd_map/reports/kcity2025_candidate_review_20260916"),
    )
    args = parser.parse_args()
    root = args.repo_root.resolve()
    config = args.config if args.config.is_absolute() else root / args.config
    output = args.output_dir if args.output_dir.is_absolute() else root / args.output_dir
    summary = build_candidate_review(config.resolve(), root, output.resolve())
    high_speed = summary["high_speed_inventory"]
    print(f"[asmc_hd_map] wrote review bundle to {output}")
    print(
        "[asmc_hd_map] high_speed parallel={parallel} lane_change_reachable={reachable} "
        "missing_nominal_boundary={missing}".format(
            parallel=high_speed["parallel_current_link_count"],
            reachable=high_speed["lane_change_reachable_link_count"],
            missing=high_speed["centerline_only_boundary_review_link_ids"],
        )
    )


if __name__ == "__main__":
    main()
