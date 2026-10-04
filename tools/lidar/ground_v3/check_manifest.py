#!/usr/bin/env python3
"""Check fixed V3 path/SHA pairs without modifying source data."""
import argparse
import hashlib
from pathlib import Path

import yaml


def digest(path):
    result = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            result.update(block)
    return result.hexdigest()


def under(root, relative):
    path = Path(relative)
    if path.is_absolute() or ".." in path.parts:
        raise ValueError("unsafe manifest path: " + str(path))
    return root / path


def verify(section, root):
    checked = 0
    for key, expected in section.items():
        if not key.endswith("_sha256"):
            continue
        source_key = key[:-7]
        if source_key not in section:
            raise ValueError("missing path for " + key)
        path = under(root, section[source_key])
        if not path.is_file():
            raise FileNotFoundError(path)
        actual = digest(path)
        if actual != expected:
            raise ValueError(f"SHA mismatch: {path}: {actual} != {expected}")
        checked += 1
    return checked


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", type=Path, default=Path(__file__).resolve().parents[3])
    parser.add_argument("--data", type=Path, required=True)
    args = parser.parse_args()
    repo, data = args.repo.resolve(), args.data.resolve()
    manifest = repo / "config/baselines/lidar_l2a2_v3_20261005.yaml"
    with manifest.open() as stream:
        record = yaml.safe_load(stream)
    if record["id"] != "lidar_l2a2_v3_20261005" or not record["datasets"]:
        raise ValueError("incomplete V3 manifest")
    checked = verify(record["comparison"], repo)
    for item in record["datasets"].values():
        checked += verify(item, data)
    checked += verify(record["runtime"], data)
    print(f"V3 manifest OK: {checked} path/SHA pairs; selected research method="
          f"{record['selected_for_next_research_stage']}; operating output unchanged="
          f"{not record['operating_output_changed']}")


if __name__ == "__main__":
    main()
