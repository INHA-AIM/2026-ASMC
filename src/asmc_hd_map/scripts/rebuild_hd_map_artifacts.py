#!/usr/bin/env python3
"""Rebuild the asmc_hd_map artifact chain after a source edit, in one step.

Order: competition metadata -> P2-A audit source pin -> P2-A audit -> compact
drivable product -> runtime pins -> historical-record integrity check.
Zones, speeds and surfaces are expected to change while planning integrates
the map; review the result with ``git diff`` instead of editing the pins by
hand. Historical v1 reports stay frozen; new revisions select new report paths.
``--check`` writes nothing and lists every stale step.
"""

from __future__ import annotations

import argparse
import hashlib
import os
from pathlib import Path
import re
import subprocess
import sys
from typing import List, Optional, Sequence

METADATA = Path("src/asmc_hd_map/config/kcity2025_competition_metadata.yaml")
AUDIT_SOURCE = Path("src/asmc_hd_map/config/kcity2025_drivable_area_audit_source.yaml")
DOC_MANIFEST = Path("config/docs/documentation_migration_v1.json")
BUILDER_RECORDS = (
    "docs/records/hd-map/kcity2025-drivable-input-v1.md",
    "docs/records/hd-map/kcity2025-compact-drivable-v1.md",
)
AUDIT_PIN = re.compile(
    r"(\n  competition_metadata:\n    path: [^\n]+\n    sha256: )([0-9a-f]{64})"
)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _run(repo_root: Path, script: str, *args: str) -> bool:
    env = dict(os.environ)
    package_src = str(repo_root / "src/asmc_hd_map/src")
    env["PYTHONPATH"] = os.pathsep.join(filter(None, [package_src, env.get("PYTHONPATH", "")]))
    command = [sys.executable, str(repo_root / "src/asmc_hd_map/scripts" / script), *args]
    result = subprocess.run(command, cwd=repo_root, env=env, capture_output=True, text=True)
    output = (result.stdout + result.stderr).strip().splitlines()
    print("[%s] %s%s" % ("ok" if result.returncode == 0 else "STALE", script, " " + " ".join(args) if args else ""))
    for line in output[-3:] if result.returncode == 0 else output[-40:]:
        print("    " + line)
    return result.returncode == 0


def _audit_pin(repo_root: Path, write: bool) -> bool:
    path = repo_root / AUDIT_SOURCE
    text = path.read_text(encoding="utf-8")
    match = AUDIT_PIN.search(text)
    if match is None:
        print("[STALE] audit source has no competition_metadata sha256 pin")
        return False
    actual = _sha256(repo_root / METADATA)
    if match.group(2) == actual:
        print("[ok] audit source metadata pin")
        return True
    if not write:
        print("[STALE] audit source metadata pin %s != %s" % (match.group(2)[:12], actual[:12]))
        return False
    path.write_text(text[: match.start(2)] + actual + text[match.end(2) :], encoding="utf-8")
    print("[ok] audit source metadata pin -> %s" % actual[:12])
    return True


def _doc_manifest(repo_root: Path, write: bool) -> bool:
    # These v1 records are historical snapshots, not current builder outputs.
    # A new map revision must select new report paths instead of re-pinning them.
    path = repo_root / DOC_MANIFEST
    text = path.read_text(encoding="utf-8")
    current = True
    for record in BUILDER_RECORDS:
        pattern = re.compile(r'("path": "%s",\s*"sha256": ")([0-9a-f]{64})' % re.escape(record))
        match = pattern.search(text)
        if match is None:
            continue
        actual = _sha256(repo_root / record)
        if match.group(2) != actual:
            current = False
    print("[%s] documentation-migration record SHAs" % ("ok" if current else "STALE"))
    return current


def _consumer_bundle(repo_root: Path, check: bool) -> bool:
    # Optional monorepo deployment export. asmc_hd_map remains the geometry
    # authority and can be built alone; no LiDAR runtime or ROS is imported.
    script=repo_root/"src/asmc_lidar/tools/sync_default_hd_map.py"
    if not script.is_file():
        return True
    result=subprocess.run([sys.executable,str(script)]+(["--check"] if check else []),
                          cwd=repo_root,capture_output=True,text=True)
    print("[%s] LiDAR deployment derivative" % ("ok" if result.returncode==0 else "STALE"))
    print((result.stdout+result.stderr).strip())
    return result.returncode==0


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=Path(__file__).resolve().parents[3])
    parser.add_argument("--check", action="store_true", help="report stale steps without writing")
    args = parser.parse_args(argv)
    repo_root = args.repo_root.resolve()
    root = str(repo_root)

    if args.check:
        results: List[bool] = [
            _run(repo_root, "build_competition_metadata.py", "--check"),
            _audit_pin(repo_root, write=False),
            _run(repo_root, "build_drivable_area_input_audit.py", "--check"),
            _run(repo_root, "build_compact_drivable_map.py", "--repo-root", root, "--check"),
            _run(repo_root, "build_runtime_pins.py", "--check"),
            _consumer_bundle(repo_root, check=True),
            _doc_manifest(repo_root, write=False),
        ]
        return 0 if all(results) else 1

    steps = (
        lambda: _run(repo_root, "build_competition_metadata.py", "--write"),
        lambda: _audit_pin(repo_root, write=True),
        lambda: _run(repo_root, "build_drivable_area_input_audit.py"),
        lambda: _run(repo_root, "build_compact_drivable_map.py", "--repo-root", root),
        lambda: _run(repo_root, "build_runtime_pins.py", "--write"),
        lambda: _consumer_bundle(repo_root, check=False),
        lambda: _doc_manifest(repo_root, write=True),
    )
    for step in steps:
        if not step():
            print("rebuild stopped; fix the reported source error and rerun")
            return 1
    print("rebuilt; review with git diff and run the asmc_hd_map tests")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
