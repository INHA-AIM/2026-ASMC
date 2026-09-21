#!/usr/bin/env python3
"""Build or verify the deterministic P1-C competition metadata bundle."""

from __future__ import annotations

import argparse
import difflib
import os
from pathlib import Path
import sys
from typing import Optional, Sequence


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def _data_root(repo_root: Path) -> Path:
    configured = os.environ.get("ASMC_DATA")
    if configured:
        return Path(configured).expanduser().resolve()
    sibling = (repo_root.parent / "data").resolve()
    return sibling if sibling.is_dir() else Path("/data")


def main(argv: Optional[Sequence[str]] = None) -> int:
    root = _repo_root()
    package_src = root / "src/asmc_hd_map/src"
    if str(package_src) not in sys.path:
        sys.path.insert(0, str(package_src))

    from asmc_hd_map.competition_metadata import (
        build_competition_metadata,
        dump_metadata_yaml,
    )

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=root)
    parser.add_argument("--data-root", type=Path)
    parser.add_argument(
        "--config",
        type=Path,
        default=root / "src/asmc_hd_map/config/kcity2025_competition_metadata_source.yaml",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=root / "src/asmc_hd_map/config/kcity2025_competition_metadata.yaml",
    )
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--write", action="store_true")
    mode.add_argument("--check", action="store_true")
    args = parser.parse_args(argv)

    repo_root = args.repo_root.resolve()
    data_root = (args.data_root or _data_root(repo_root)).resolve()
    document = build_competition_metadata(args.config.resolve(), repo_root, data_root)
    expected = dump_metadata_yaml(document)
    output = args.output.resolve()
    if args.write:
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(expected, encoding="utf-8")
        print(
            "wrote %s: checkpoints=%d missions=%d"
            % (output, len(document["checkpoints"]), len(document["mission_zones"]))
        )
        return 0

    if not output.is_file():
        print("competition metadata is missing: %s" % output, file=sys.stderr)
        return 1
    actual = output.read_text(encoding="utf-8")
    if actual != expected:
        print("competition metadata is stale: %s" % output, file=sys.stderr)
        for line in difflib.unified_diff(
            actual.splitlines(), expected.splitlines(), fromfile=str(output), tofile="regenerated", lineterm=""
        ):
            print(line, file=sys.stderr)
        return 1
    print(
        "competition metadata is current: checkpoints=%d missions=%d"
        % (len(document["checkpoints"]), len(document["mission_zones"]))
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
