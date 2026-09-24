#!/usr/bin/env python3

import sys
import unittest
from pathlib import Path
from types import SimpleNamespace


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT / "src"))

from asmc_perception.traffic_light_parity import (
    ParityError,
    compare_details,
    parse_detail,
    validate_typed_result,
)


class Stamp:
    def __init__(self, value):
        self.value = value

    def to_nsec(self):
        return self.value


def typed_message(**values):
    defaults = {
        "label": "red",
        "state": 2,
        "stale": False,
        "valid": True,
        "confidence": 0.8,
        "candidate_labels": ["red", "yellow"],
        "candidate_confidences": [0.8, 0.1],
        "header": SimpleNamespace(stamp=Stamp(123), frame_id="camera_extra"),
    }
    defaults.update(values)
    return SimpleNamespace(**defaults)


class TrafficLightParityTest(unittest.TestCase):
    def test_detail_parser_and_comparison(self):
        value = parse_detail(
            '{"label":"red","prob":0.8,"top":[["red",0.8],["yellow",0.1]]}'
        )
        self.assertFalse(value["stale"])
        metrics = compare_details(value, dict(value))
        self.assertEqual(0.0, metrics["probability_delta"])

    def test_detail_mismatch_is_rejected(self):
        reference = parse_detail('{"label":"red","prob":0.8,"top":[]}')
        canonical = parse_detail('{"label":"green","prob":0.8,"top":[]}')
        with self.assertRaises(ParityError):
            compare_details(reference, canonical)

    def test_typed_result_matches_detail_and_header(self):
        detail = parse_detail(
            '{"label":"red","prob":0.8,"top":[["red",0.8],["yellow",0.1]]}'
        )
        validate_typed_result(
            typed_message(),
            detail,
            expected_stamp_nsec=123,
            expected_frame_id="camera_extra",
        )

    def test_stale_result_must_be_invalid(self):
        detail = parse_detail(
            '{"label":"not_detected","prob":0.0,"top":[],"stale":true}'
        )
        validate_typed_result(
            typed_message(
                label="not_detected",
                state=1,
                stale=True,
                valid=False,
                confidence=0.0,
                candidate_labels=[],
                candidate_confidences=[],
            ),
            detail,
            expected_stamp_nsec=123,
            expected_frame_id="camera_extra",
        )

    def test_wrong_source_stamp_is_rejected(self):
        detail = parse_detail(
            '{"label":"red","prob":0.8,"top":[["red",0.8],["yellow",0.1]]}'
        )
        with self.assertRaises(ParityError):
            validate_typed_result(
                typed_message(),
                detail,
                expected_stamp_nsec=124,
                expected_frame_id="camera_extra",
            )


if __name__ == "__main__":
    unittest.main()
