#!/usr/bin/env python3
"""Replay one raw bag through canonical and compatibility perception paths."""

from __future__ import annotations

import argparse
import hashlib
import io
import json
import signal
import subprocess
import sys
import time
from pathlib import Path

import rosbag
import rosgraph


TOPICS = (
    "/perception/camera/front/road_marking/class_id",
    "/perception/bev/camera/road_marking/class_id",
    "/perception/lane/boundary_observations",
)


class ParityError(RuntimeError):
    pass


def _start(command, log_path):
    stream = log_path.open("wb")
    process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT)
    process._asmc_log_stream = stream  # type: ignore[attr-defined]
    return process


def _stop(process, timeout=10.0):
    if process is None:
        return
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            process.terminate()
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=3.0)
    stream = getattr(process, "_asmc_log_stream", None)
    if stream is not None:
        stream.close()


def _wait_for_publishers(launch, timeout=30.0):
    master = rosgraph.Master("/p4c5_runtime_parity")
    deadline = time.monotonic() + timeout
    expected = set(TOPICS)
    while time.monotonic() < deadline:
        if launch.poll() is not None:
            raise ParityError(
                "perception launch exited before publishers became ready"
            )
        try:
            published = {name for name, _type in master.getPublishedTopics("")}
            if expected <= published:
                return
        except Exception:
            pass
        time.sleep(0.25)
    raise ParityError("perception publishers were not ready within {:.1f}s".format(timeout))


def _run_variant(variant, input_bag, output_root, rate, duration):
    launch = None
    recorder = None
    output_bag = output_root / (variant + ".bag")
    try:
        launch = _start(
            [
                "roslaunch",
                "asmc_perception",
                "runtime_parity_pipeline.launch",
                "variant:={}".format(variant),
            ],
            output_root / (variant + "_launch.log"),
        )
        _wait_for_publishers(launch)
        recorder = _start(
            ["rosbag", "record", "-O", str(output_bag)] + list(TOPICS),
            output_root / (variant + "_record.log"),
        )
        time.sleep(0.75)
        play = subprocess.run(
            [
                "rosbag",
                "play",
                "--quiet",
                "--prefix=/p4c5_input",
                "--rate={}".format(rate),
                "--duration={}".format(duration),
                str(input_bag),
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            check=False,
            universal_newlines=True,
        )
        if play.returncode != 0:
            raise ParityError(
                "{} rosbag play failed: {}".format(variant, play.stdout.strip())
            )
        time.sleep(1.5)
    finally:
        _stop(recorder)
        _stop(launch)
    if not output_bag.is_file():
        raise ParityError("{} output bag was not created".format(variant))
    return output_bag


def _bag_digest(path):
    result = {topic: {} for topic in TOPICS}
    with rosbag.Bag(str(path), "r") as bag:
        for topic, message, _stamp in bag.read_messages(topics=list(TOPICS)):
            if not hasattr(message, "header"):
                raise ParityError("{} has no source header".format(topic))
            stamp_ns = int(message.header.stamp.to_nsec())
            # ROS publisher sequence numbers are connection-local and can
            # differ between two sequential replays. Frame, source stamp and
            # the full perception payload remain authoritative.
            message.header.seq = 0
            stream = io.BytesIO()
            message.serialize(stream)
            digest = hashlib.sha256(stream.getvalue()).hexdigest()
            if stamp_ns in result[topic] and result[topic][stamp_ns] != digest:
                raise ParityError(
                    "{} emitted different payloads at source stamp {}".format(
                        topic, stamp_ns
                    )
                )
            result[topic][stamp_ns] = digest
    return result


def _sha256_file(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _compare(canonical, compatibility, minimum_common):
    summary = {}
    passed = True
    for topic in TOPICS:
        left = canonical[topic]
        right = compatibility[topic]
        common = sorted(set(left) & set(right))
        mismatches = [stamp for stamp in common if left[stamp] != right[stamp]]
        topic_passed = len(common) >= minimum_common and not mismatches
        passed = passed and topic_passed
        summary[topic] = {
            "canonical_messages": len(left),
            "compatibility_messages": len(right),
            "common_source_stamps": len(common),
            "payload_mismatches": len(mismatches),
            "passed": topic_passed,
        }
    return passed, summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, type=Path)
    parser.add_argument("--output-dir", type=Path, default=Path("/tmp/asmc_p4c5_parity"))
    parser.add_argument("--rate", type=float, default=0.5)
    parser.add_argument("--duration", type=float, default=4.0)
    parser.add_argument("--minimum-common", type=int, default=8)
    args = parser.parse_args()
    input_bag = args.bag.expanduser().resolve()
    if not input_bag.is_file():
        raise SystemExit("input bag not found: {}".format(input_bag))
    if args.rate <= 0 or args.duration <= 0:
        raise SystemExit("rate and duration must be positive")
    output_root = args.output_dir.expanduser().resolve()
    output_root.mkdir(parents=True, exist_ok=True)
    occupied = [
        output_root / (name + ".bag")
        for name in ("canonical", "compatibility")
    ]
    if any(path.exists() for path in occupied):
        raise SystemExit(
            "output directory already contains parity bags; choose a new path"
        )

    try:
        bags = {
            variant: _run_variant(
                variant, input_bag, output_root, args.rate, args.duration
            )
            for variant in ("canonical", "compatibility")
        }
        digests = {name: _bag_digest(path) for name, path in bags.items()}
        passed, topics = _compare(
            digests["canonical"], digests["compatibility"], args.minimum_common
        )
        report = {
            "schema_version": 1,
            "acceptance_id": "p4c5_representative_bag_output_parity_v1",
            "status": "passed" if passed else "failed",
            "input_bag": str(input_bag),
            "input_bag_sha256": _sha256_file(input_bag),
            "rate": args.rate,
            "duration_s": args.duration,
            "topics": topics,
        }
        report_path = output_root / "report.json"
        report_path.write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        print(json.dumps(report, indent=2, sort_keys=True))
        return 0 if passed else 1
    except (OSError, ParityError, rosbag.ROSBagException) as exc:
        print("perception runtime parity failed: {}".format(exc), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
