#!/usr/bin/env python3
"""Replay only the frozen V3 profile; existing V2 results remain immutable baselines."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess


def digest(path):
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--repo", type=Path, default=Path(__file__).resolve().parents[3])
    args = parser.parse_args()
    root = args.repo.resolve()
    source = args.input.resolve()
    args.output.mkdir(parents=True, exist_ok=False)
    exe = root / "devel/lib/asmc_lidar/asmc_lidar_ground_live_offline"
    config = root / "src/asmc_lidar/config/ground_v3/roll_pitch_all.yaml"
    calibration = root / "src/asmc_lidar/config/autonav_v3.yaml"
    result = args.output / "roll_pitch_all.jsonl"
    command = [str(exe), str(source / "snapshots.jsonl"), str(calibration),
               str(result), "guard25", "all", str(config)]
    manifest = {
        "source": str(source),
        "configuration": "roll_pitch_all",
        "command": command,
        "complete": False,
        "sha256": {key: digest(path) for key, path in (
            ("input", source / "snapshots.jsonl"), ("binary", exe),
            ("configuration", config), ("calibration", calibration))},
    }
    (args.output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    with (args.output / "replay.log").open("x") as log:
        subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True)
    manifest["sha256"]["result"] = digest(result)
    manifest["complete"] = True
    (args.output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps({"result": str(result), "sha256": manifest["sha256"]["result"]}))


if __name__ == "__main__":
    main()
