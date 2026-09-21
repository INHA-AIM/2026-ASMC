#!/usr/bin/env python3
"""Build the current route-by-route HD-map curation stage."""

from pathlib import Path
import argparse

from asmc_hd_map.route_curation import build_route_curation


def main() -> None:
    script = Path(__file__).resolve()
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo-root", type=Path, default=script.parents[3])
    parser.add_argument(
        "--config", type=Path,
        default=script.parents[1] / "config" / "kcity2025_route_curation.yaml",
    )
    parser.add_argument(
        "--output-dir", type=Path,
        default=script.parents[1] / "reports" / "kcity2025_route_curation_20260917",
    )
    args = parser.parse_args()
    summary = build_route_curation(
        args.config.resolve(), args.repo_root.resolve(), args.output_dir.resolve(),
    )
    print(
        "[asmc_hd_map] route curation: "
        f"implemented={len(summary['implemented_segments'])} "
        f"markings={summary['selected_marking_count']} "
        f"planned={summary['planned_segment_count']} "
        f"operational={summary['operational']}"
    )


if __name__ == "__main__":
    main()
