#!/usr/bin/env python3
"""Reject missing attitude and unsupported camera provenance in isolated replay."""
import argparse
import json
from pathlib import Path
import subprocess
import tempfile


def run(binary, snapshots, calibration, reference):
    original = json.loads(snapshots.open().readline())
    source_cloud = snapshots.parent / original["cloud_path"]
    with tempfile.TemporaryDirectory(prefix="asmc-l2a2-input-") as directory:
        root = Path(directory)
        (root / "clouds").mkdir()
        (root / original["cloud_path"]).symlink_to(source_cloud)
        cases = (
            ("valid", original, True, ""),
            ("missing_roll", {k: v for k, v in original.items() if k != "roll"}, False,
             "missing roll, pitch, or geometry_valid"),
            ("missing_pitch", {k: v for k, v in original.items() if k != "pitch"}, False,
             "missing roll, pitch, or geometry_valid"),
            ("missing_geometry", {k: v for k, v in original.items() if k != "geometry_valid"},
             False, "missing roll, pitch, or geometry_valid"),
            ("missing_camera", {k: v for k, v in original.items() if k != "controlled_camera_state"},
             False, "missing controlled_camera_state"),
            ("positive_camera", {**original, "controlled_camera_state": "positive"},
             False, "unsupported controlled_camera_state"),
        )
        for name, row, should_pass, reason in cases:
            fixture = root / f"{name}.jsonl"
            fixture.write_text(json.dumps(row) + "\n")
            process = subprocess.run(
                [str(binary), str(fixture), str(calibration), str(reference),
                 "reference", str(root / f"{name}_result.jsonl")],
                text=True, capture_output=True, check=False,
            )
            if (process.returncode == 0) != should_pass:
                raise AssertionError(f"{name}: unexpected status {process.returncode}: {process.stderr}")
            if not should_pass and reason not in process.stderr:
                raise AssertionError(f"{name}: missing reason: {process.stderr}")
    print(f"offline input contract: {len(cases)} cases passed")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("binary", "snapshots", "calibration", "reference"):
        parser.add_argument("--" + name, required=True, type=Path)
    args = parser.parse_args()
    run(args.binary, args.snapshots, args.calibration, args.reference)


if __name__ == "__main__":
    main()
