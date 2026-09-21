#!/usr/bin/env python3
"""Build the reviewed legacy/current K-City 2025 hybrid candidate."""

from pathlib import Path
import argparse

from asmc_hd_map.hybrid_completion import build_hybrid_completion


def main() -> None:
    script = Path(__file__).resolve()
    package = script.parents[1]
    repo_root = script.parents[3]
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config", type=Path,
        default=package / "config" / "kcity2025_hybrid_completion.yaml",
    )
    parser.add_argument(
        "--output-dir", type=Path,
        default=package / "reports" / "kcity2025_hybrid_completion_20260916",
    )
    args = parser.parse_args()
    summary = build_hybrid_completion(
        args.config.resolve(), repo_root.resolve(), args.output_dir.resolve(),
    )
    counts = summary["counts"]
    print(
        f"built {summary['map_id']} operational={summary['operational']} "
        f"legacy={counts['legacy_lane_boundary_count_after_clipping']} "
        f"route_rebuilt={counts['route_reconstructed_boundary_count']} "
        f"high_speed_rebuilt={counts['high_speed_missing_shared_boundary_count']} "
        f"stoplines={counts['stopline_kept_count']}"
    )


if __name__ == "__main__":
    main()
