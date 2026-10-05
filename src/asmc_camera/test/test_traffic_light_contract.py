#!/usr/bin/env python3

import sys
import unittest
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT / "src"))

from asmc_camera.traffic_light_contract import (
    ordered_candidates,
    populate_detection,
    state_for_label,
    valid_for_label,
)


class MessageStub:
    pass


class TrafficLightContractTest(unittest.TestCase):
    def test_frozen_state_mapping_and_validity(self):
        expected = {
            "unknown": 0,
            "not_detected": 1,
            "red": 2,
            "yellow": 3,
            "green": 4,
            "red_yellow": 5,
            "red_left": 6,
            "green_left": 7,
            "left": 8,
        }
        self.assertEqual(expected, {name: state_for_label(name) for name in expected})
        self.assertFalse(valid_for_label("unknown"))
        self.assertFalse(valid_for_label("not_detected"))
        self.assertTrue(valid_for_label("red"))

    def test_candidates_are_parallel_and_descending(self):
        labels, confidences = ordered_candidates(
            ["red", "green", "yellow"], [0.2, 0.7, 0.1], 2
        )
        self.assertEqual(["green", "red"], labels)
        self.assertEqual([0.7, 0.2], confidences)

    def test_stale_detection_is_invalid_and_keeps_source_header(self):
        header = object()
        message = populate_detection(
            MessageStub(),
            header=header,
            label="red",
            confidence=0.9,
            stale=True,
            inference_latency_ms=4.2,
            model_id="sha256:test",
            candidate_labels=["red"],
            candidate_confidences=[0.9],
        )
        self.assertIs(header, message.header)
        self.assertFalse(message.valid)
        self.assertTrue(message.stale)
        self.assertEqual(2, message.state)

    def test_candidate_length_mismatch_is_rejected(self):
        with self.assertRaises(ValueError):
            populate_detection(
                MessageStub(),
                header=object(),
                label="red",
                confidence=0.9,
                stale=False,
                inference_latency_ms=4.2,
                model_id="sha256:test",
                candidate_labels=["red"],
                candidate_confidences=[],
            )


if __name__ == "__main__":
    unittest.main()
