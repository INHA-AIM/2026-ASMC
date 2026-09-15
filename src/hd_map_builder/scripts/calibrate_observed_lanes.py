#!/usr/bin/env python3
"""Calibrate current K-City lane-boundary candidates with observed lines."""

from __future__ import annotations

import argparse
from pathlib import Path

from hd_map_builder.observed_builder import build_observed_calibration


def _default_repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def main() -> None:
    repo_root = _default_repo_root()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=repo_root)
    parser.add_argument(
        "--config", type=Path,
        default=Path("src/hd_map_builder/config/kcity2025_observed_calibration.yaml"),
    )
    parser.add_argument(
        "--output-dir", type=Path,
        default=Path("src/hd_map_builder/reports/kcity2025_observed_calibration_20260916"),
    )
    args = parser.parse_args()
    root = args.repo_root.resolve()
    config = args.config if args.config.is_absolute() else root / args.config
    output = args.output_dir if args.output_dir.is_absolute() else root / args.output_dir
    summary = build_observed_calibration(config.resolve(), root, output.resolve())
    print(f"[hd_map_builder] wrote observed candidate to {output}")
    print(
        "[hd_map_builder] sources={sources} direct_rows={direct} "
        "calibrated_parents={parents} calibrated_intervals={intervals}".format(
            sources=summary["input_contract"]["source_count"],
            direct=summary["input_contract"]["direct_rows"],
            parents=summary["candidate_map"]["calibrated_parent_boundary_count"],
            intervals=summary["candidate_map"]["calibrated_interval_count"],
        )
    )


if __name__ == "__main__":
    main()
