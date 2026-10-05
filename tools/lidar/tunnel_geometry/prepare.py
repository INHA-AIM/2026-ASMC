#!/usr/bin/env python3
"""Verify upstream SHA, expose an offline harness, and copy only to build artifacts."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "camera_sync"))
from prepare_reference import prepare


def generate(source, output):
    prepare(source, output)
    directory = output / "pinned" / "Lidar" / "TunnelFilter"
    directory.mkdir(parents=True, exist_ok=True)
    header = (source / "src/Lidar/TunnelFilter/TunnelFilterCore.hpp").read_text()
    marker = "private:\n    void FitGround();"
    if header.count(marker) != 1:
        raise ValueError("pinned harness marker changed")
    header = header.replace(marker, "private:\n    friend class L1B4Harness;\n    void FitGround();")
    (directory / "TunnelFilterCore.hpp").write_text(header)
    cpp = source / "src/Lidar/TunnelFilter/TunnelFilterCore.cpp"
    (directory / cpp.name).write_bytes(cpp.read_bytes())
    helper = ROOT / "camera_sync/geometry.cpp"
    prefix = helper.read_text().split("int main(int argc,char** argv) try {", 1)
    if len(prefix) != 2:
        raise ValueError("pinned evaluator helper marker changed")
    (output / "evaluation_helpers.hpp").write_text("#pragma once\n" + prefix[0])
    provenance = json.loads((output / "reference_provenance.json").read_text())
    provenance["offline_adaptations"] = [
        "One friend declaration in generated core header; no algorithm body changed",
        "L1B4Harness restores post-ground/pre-wall labels and reuses exact ClusterAndFilter",
        "All policies share reference ground/wall fit and temporal persistence",
        "Evaluation helper prefix extracted verbatim from historical evaluator"]
    provenance["helper_source_sha256"] = hashlib.sha256(helper.read_bytes()).hexdigest()
    provenance["generated_core_sha256"] = hashlib.sha256(header.encode()).hexdigest()
    (output / "reference_provenance.json").write_text(json.dumps(provenance, sort_keys=True, indent=2) + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    generate(args.source, args.output)
