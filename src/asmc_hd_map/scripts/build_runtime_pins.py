#!/usr/bin/env python3
"""Build or verify the HD-map runtime pins that the competition publisher loads."""

from __future__ import annotations

import argparse
import difflib
from pathlib import Path
import sys
from typing import Optional, Sequence


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def main(argv: Optional[Sequence[str]] = None) -> int:
    root = _repo_root()
    package_src = root / "src/asmc_hd_map/src"
    if str(package_src) not in sys.path:
        sys.path.insert(0, str(package_src))

    from asmc_hd_map.runtime_bundle import RUNTIME_PINS_REL, build_runtime_pins, dump_runtime_pins

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=root)
    parser.add_argument("--output", type=Path)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--write", action="store_true")
    mode.add_argument("--check", action="store_true")
    args = parser.parse_args(argv)

    repo_root = args.repo_root.resolve()
    document = build_runtime_pins(repo_root)
    expected = dump_runtime_pins(document)
    output = (args.output or repo_root / RUNTIME_PINS_REL).resolve()
    metadata = document["competition_inputs"]["metadata_bundle"]
    summary = "checkpoints=%d missions=%d" % (metadata["checkpoint_count"], metadata["mission_zone_count"])
    if args.write:
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(expected, encoding="utf-8")
        print("wrote %s: %s" % (output, summary))
        return 0

    if not output.is_file():
        print("HD-map runtime pins are missing: %s" % output, file=sys.stderr)
        return 1
    actual = output.read_text(encoding="utf-8")
    if actual != expected:
        print("HD-map runtime pins are stale: %s" % output, file=sys.stderr)
        for line in difflib.unified_diff(
            actual.splitlines(), expected.splitlines(), fromfile=str(output), tofile="regenerated", lineterm=""
        ):
            print(line, file=sys.stderr)
        return 1
    print("HD-map runtime pins are current: %s" % summary)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
