#!/usr/bin/env python3
"""Build/check the deterministic P2-A drivable-area input audit."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import tempfile

from asmc_hd_map.drivable_area_audit import build_drivable_area_input_audit, write_audit


REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_SOURCE = REPO_ROOT / "src/asmc_hd_map/config/kcity2025_drivable_area_audit_source.yaml"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    source = args.source.resolve()
    document = build_drivable_area_input_audit(REPO_ROOT, source)
    source_config = __import__("yaml").safe_load(source.read_text(encoding="utf-8"))
    outputs = source_config["outputs"]
    manifest = REPO_ROOT / outputs["manifest"]
    report = REPO_ROOT / outputs["report"]
    if args.check:
        with tempfile.TemporaryDirectory() as directory:
            generated_manifest = Path(directory) / manifest.name
            generated_report = Path(directory) / report.name
            write_audit(document, generated_manifest, generated_report)
            failures = []
            for expected, generated in ((manifest, generated_manifest), (report, generated_report)):
                if not expected.is_file() or expected.read_bytes() != generated.read_bytes():
                    failures.append(str(expected.relative_to(REPO_ROOT)))
            if failures:
                print("P2-A audit is stale: %s" % ", ".join(failures), file=sys.stderr)
                return 1
        print(
            "P2-A audit is current: links=%d guarded=%.3fm clip_candidate=%.3fm"
            % (
                document["route_summary"]["link_count"],
                document["route_summary"]["guarded_length_m"],
                document["route_summary"]["lane_clip_candidate_length_m"],
            )
        )
        return 0
    write_audit(document, manifest, report)
    print("wrote %s" % manifest.relative_to(REPO_ROOT))
    print("wrote %s" % report.relative_to(REPO_ROOT))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
