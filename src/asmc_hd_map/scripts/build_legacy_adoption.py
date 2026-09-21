#!/usr/bin/env python3
"""Build the driving-reviewed legacy lane adoption candidate."""

from pathlib import Path
import argparse

from asmc_hd_map.legacy_adoption import build_legacy_adoption


def main() -> None:
    script = Path(__file__).resolve()
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo-root", type=Path, default=script.parents[3])
    parser.add_argument(
        "--config", type=Path,
        default=script.parents[1] / "config" / "kcity2025_legacy_adoption.yaml",
    )
    parser.add_argument(
        "--output-dir", type=Path,
        default=script.parents[1] / "reports" / "kcity2025_legacy_adoption_20260916",
    )
    args = parser.parse_args()
    summary = build_legacy_adoption(
        args.config.resolve(), args.repo_root.resolve(), args.output_dir.resolve(),
    )
    print(
        "[asmc_hd_map] legacy adoption: "
        f"segments={summary['counts']['adopted_segment_count']} "
        f"source_lanes={summary['counts']['adopted_legacy_source_lanes']} "
        f"rescued_D={summary['counts']['rescued_tier_d_source_lanes']} "
        f"operational={summary['operational']}"
    )


if __name__ == "__main__":
    main()
