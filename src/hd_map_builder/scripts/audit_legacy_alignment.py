#!/usr/bin/env python3
"""Audit reusable legacy map segments without modifying an operational map."""

import argparse
from pathlib import Path
import sys


def _repo_root(script: Path) -> Path:
    return script.resolve().parents[3]


def main() -> None:
    script = Path(__file__)
    repo = _repo_root(script)
    source_dir = script.resolve().parents[1] / "src"
    if str(source_dir) not in sys.path:
        sys.path.insert(0, str(source_dir))

    from hd_map_builder.audit import run_audit

    parser = argparse.ArgumentParser(
        description="K-City current map vs legacy KATRI map transfer audit",
    )
    parser.add_argument(
        "--config", type=Path,
        default=script.resolve().parents[1] / "config" / "kcity2025_legacy_transfer.yaml",
    )
    parser.add_argument(
        "--output", type=Path,
        default=script.resolve().parents[1] / "reports" / "kcity2025_legacy_alignment_20260916",
    )
    parser.add_argument("--repo-root", type=Path, default=repo)
    args = parser.parse_args()
    summary = run_audit(
        args.config.resolve(), args.repo_root.resolve(), args.output.resolve(),
    )
    print(f"[hd_map_builder] wrote {args.output.resolve()}")
    print(
        "  route reuse: "
        f"{summary['route']['reuse_length_ratio'] * 100.0:.2f}% / "
        f"{summary['route']['total_length_m']:.1f} m"
    )
    print(
        "  legacy lane candidates A/B/D: "
        f"{summary['lane_boundary_candidates']['tiers']}"
    )
    print("  operational map unchanged")


if __name__ == "__main__":
    main()
