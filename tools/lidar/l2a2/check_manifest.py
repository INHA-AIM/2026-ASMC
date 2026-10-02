#!/usr/bin/env python3
"""Verify L2-A2 pinned data and result hashes without copying the bags or clouds."""
import argparse
import hashlib
from pathlib import Path
import yaml


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", required=True, type=Path)
    parser.add_argument("--data-root", required=True, type=Path)
    args = parser.parse_args()
    manifest_path = args.repo_root / "config/baselines/lidar_l2a2_ground_20261002.yaml"
    manifest = yaml.safe_load(manifest_path.read_text())
    if manifest["decision"]["patchwork_fixed_configuration"] != "not_adopted":
        raise ValueError("candidate decision changed")
    if manifest["operational_topics_or_ros_messages_changed"]:
        raise ValueError("operational contract drift")
    root = args.data_root.resolve()
    checked = 0
    for name, record in manifest["data"].items():
        if not isinstance(record, dict) or "path" not in record:
            continue
        relative = Path(record["path"])
        if relative.is_absolute() or ".." in relative.parts:
            raise ValueError(f"unsafe relative path: {name}")
        path = (root / relative).resolve()
        try:
            path.relative_to(root)
        except ValueError:
            raise ValueError(f"path outside data root: {name}")
        actual = sha256(path)
        if actual != record["sha256"]:
            raise ValueError(f"SHA mismatch: {name}: {actual}")
        checked += 1
    if checked != 14:
        raise ValueError(f"unexpected artifact count: {checked}")
    print(f"L2-A2 manifest: {checked} data/result SHA values verified")


if __name__ == "__main__":
    main()
