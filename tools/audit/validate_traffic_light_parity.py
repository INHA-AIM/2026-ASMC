#!/usr/bin/env python3
"""Validate the committed P4-C2 traffic-light parity report."""

from __future__ import annotations

import argparse
import hashlib
import os
import sys
from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_REPORT = REPO_ROOT / "config/baselines/perception_traffic_light_parity_v1.yaml"


class ValidationError(ValueError):
    pass


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _resolve_reference(value: str) -> Path:
    if value == "$ASMC_DATA":
        data_root = Path(
            os.environ.get("ASMC_DATA", str(REPO_ROOT.parent / "data"))
        )
        return data_root.resolve()
    if value.startswith("$ASMC_DATA/"):
        data_root = Path(
            os.environ.get("ASMC_DATA", str(REPO_ROOT.parent / "data"))
        )
        return (data_root / value[len("$ASMC_DATA/") :]).resolve()
    path = Path(value).expanduser()
    return path.resolve() if path.is_absolute() else (REPO_ROOT / path).resolve()


def validate(path: Path, check_artifacts: bool = False):
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValidationError("report root must be a mapping")
    if document.get("schema_version") != 1:
        raise ValidationError("unsupported schema_version")
    if document.get("audit_id") != "p4c2_traffic_light_parity_v1":
        raise ValidationError("unexpected audit_id")
    if document.get("status") != "pass":
        raise ValidationError("parity report status is not pass")
    acceptance = document.get("acceptance", {})
    requested = int(acceptance.get("requested_fresh_frames", 0))
    compared = int(acceptance.get("initial_fresh_frames_compared", 0))
    if requested <= 0 or compared < requested:
        raise ValidationError("fresh-frame comparison is incomplete")
    if int(acceptance.get("recovery_fresh_frames_compared", 0)) < 1:
        raise ValidationError("recovery frame was not compared")
    if not acceptance.get("stale_transition_passed"):
        raise ValidationError("stale transition did not pass")
    if not acceptance.get("recovery_transition_passed"):
        raise ValidationError("recovery transition did not pass")
    if acceptance.get("source_stamp_and_frame_mismatches") != 0:
        raise ValidationError("source stamp/frame mismatch was observed")
    tolerance = float(acceptance.get("probability_tolerance", 0.0))
    if tolerance <= 0.0:
        raise ValidationError("probability_tolerance must be positive")
    for field in (
        "maximum_probability_delta",
        "maximum_candidate_confidence_delta",
    ):
        if float(acceptance.get(field, tolerance + 1.0)) > tolerance:
            raise ValidationError(f"{field} exceeds tolerance")
    if document.get("errors"):
        raise ValidationError("report contains errors")
    source = document.get("source", {})
    if source.get("sampling") != "uniform_topic_extent":
        raise ValidationError("representative bag must be sampled across its full topic extent")
    if int(source.get("topic_message_count", 0)) < requested + 1:
        raise ValidationError("representative bag does not contain enough source frames")

    if check_artifacts:
        for section, field in (("source", "bag"), ("checkpoint", "path")):
            value = document.get(section, {})
            artifact = _resolve_reference(str(value.get(field, "")))
            if not artifact.is_file():
                raise ValidationError(f"missing {section} artifact: {artifact}")
            expected_size = int(value.get("size_bytes", artifact.stat().st_size))
            if artifact.stat().st_size != expected_size:
                raise ValidationError(f"{section} artifact size mismatch")
            if _sha256(artifact) != value.get("sha256"):
                raise ValidationError(f"{section} artifact SHA mismatch")
    return document


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("report", nargs="?", type=Path, default=DEFAULT_REPORT)
    parser.add_argument("--check-artifacts", action="store_true")
    args = parser.parse_args(argv)
    try:
        report = validate(args.report.resolve(), args.check_artifacts)
    except (OSError, TypeError, ValueError, yaml.YAMLError) as exc:
        print(f"P4-C2 traffic-light parity validation failed: {exc}", file=sys.stderr)
        return 1
    acceptance = report["acceptance"]
    print(
        "P4-C2 traffic-light parity valid: "
        f"fresh={acceptance['initial_fresh_frames_compared']} "
        f"stale={acceptance['stale_transition_passed']} "
        f"recovery={acceptance['recovery_transition_passed']}"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
