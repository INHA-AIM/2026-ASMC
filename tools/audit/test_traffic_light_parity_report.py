#!/usr/bin/env python3

import tempfile
import sys
import unittest
from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "tools" / "audit"))

from validate_traffic_light_parity import ValidationError, validate


def valid_report():
    return {
        "schema_version": 1,
        "audit_id": "p4c2_traffic_light_parity_v1",
        "status": "pass",
        "source": {
            "bag": "$ASMC_DATA/example.bag",
            "sha256": "x",
            "sampling": "uniform_topic_extent",
            "topic_message_count": 100,
        },
        "checkpoint": {"path": "checkpoint.pt", "sha256": "y"},
        "acceptance": {
            "requested_fresh_frames": 24,
            "initial_fresh_frames_compared": 24,
            "recovery_fresh_frames_compared": 1,
            "stale_transition_passed": True,
            "recovery_transition_passed": True,
            "probability_tolerance": 0.0001,
            "maximum_probability_delta": 0.00005,
            "maximum_candidate_confidence_delta": 0.00005,
            "source_stamp_and_frame_mismatches": 0,
        },
        "errors": [],
    }


class TrafficLightParityReportTest(unittest.TestCase):
    def write(self, value):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        path = Path(directory.name) / "report.yaml"
        path.write_text(yaml.safe_dump(value), encoding="utf-8")
        return path

    def test_valid_report_passes_without_artifact_lookup(self):
        validate(self.write(valid_report()))

    def test_stale_failure_is_rejected(self):
        report = valid_report()
        report["acceptance"]["stale_transition_passed"] = False
        with self.assertRaises(ValidationError):
            validate(self.write(report))

    def test_probability_delta_above_tolerance_is_rejected(self):
        report = valid_report()
        report["acceptance"]["maximum_probability_delta"] = 0.1
        with self.assertRaises(ValidationError):
            validate(self.write(report))


if __name__ == "__main__":
    unittest.main()
