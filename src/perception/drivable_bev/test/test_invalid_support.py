#!/usr/bin/env python3

import unittest

from drivable_bev.invalid_support import (
    InvalidSupportClassificationConfig,
    classify_invalid_support,
    invalid_support_config_from_dict,
)


def strip(
    coverage,
    *,
    start_anchored=True,
    leading=0.0,
    trailing=0.0,
    internal=0.0,
    fully=False,
):
    return {
        "coverage_ratio": float(coverage),
        "start_anchored": bool(start_anchored),
        "leading_unusable_gap_m": float(leading),
        "trailing_unusable_gap_m": float(trailing),
        "longest_unusable_gap_m": float(max(leading, trailing, internal)),
        "longest_internal_unusable_gap_m": float(internal),
        "fully_continuous": bool(fully),
    }


def usability(control, vehicle, near, mid, preview, available=True):
    values = {
        "planning_consumer": (0.0, 15.0, control),
        "vehicle_envelope": (-0.790, 3.845, vehicle),
        "near_ahead": (3.845, 10.0, near),
        "mid_ahead": (10.0, 15.0, mid),
        "far_debug": (15.0, 20.0, preview),
    }
    return {
        "available": available,
        "zones": {
            name: {"x_min_m": lower, "x_max_m": upper, "both_strips": value}
            for name, (lower, upper, value) in values.items()
        },
    }


class InvalidSupportClassificationTest(unittest.TestCase):
    def setUp(self):
        self.full = strip(1.0, fully=True)

    def test_vehicle_leading_and_preview_trailing_are_independent_flags(self):
        result = classify_invalid_support(
            usability(
                strip(0.95, start_anchored=False, leading=0.75),
                strip(0.84, start_anchored=False, leading=0.75),
                self.full,
                self.full,
                strip(0.60, trailing=2.0),
            )
        )
        self.assertTrue(result["flags"]["vehicle_near_leading_tail"])
        self.assertTrue(result["flags"]["preview_trailing_tail"])
        self.assertFalse(result["flags"]["whole_support_shrink"])
        self.assertEqual("mixed_invalid_support", result["primary_reason"])
        self.assertAlmostEqual(
            0.75, result["measurements"]["vehicle_near_leading_tail_m"]
        )

    def test_whole_support_shrink_uses_control_core_not_far_preview(self):
        preview_only = classify_invalid_support(
            usability(
                self.full,
                self.full,
                self.full,
                self.full,
                strip(0.0, start_anchored=False, leading=5.0, trailing=5.0),
            )
        )
        self.assertFalse(preview_only["flags"]["whole_support_shrink"])
        self.assertTrue(preview_only["flags"]["preview_trailing_tail"])

        control_shrink = classify_invalid_support(
            usability(
                strip(0.60, trailing=6.0),
                self.full,
                strip(0.70, trailing=1.8),
                strip(0.50, trailing=2.5),
                self.full,
            )
        )
        self.assertTrue(control_shrink["flags"]["whole_support_shrink"])

    def test_internal_gap_remains_separate(self):
        result = classify_invalid_support(
            usability(
                strip(0.90, internal=0.5),
                self.full,
                self.full,
                self.full,
                self.full,
            )
        )
        self.assertTrue(result["flags"]["internal_support_gap"])
        self.assertFalse(result["flags"]["vehicle_near_leading_tail"])

    def test_low_confidence_geometry_is_retained_but_not_authorized(self):
        result = classify_invalid_support(
            usability(self.full, self.full, self.full, self.full, self.full)
        )
        self.assertTrue(result["low_confidence_geometry_retained"])
        self.assertFalse(result["default_planning_use_allowed"])
        self.assertFalse(result["support_state_changed"])

    def test_legacy_metrics_do_not_invent_unanchored_trailing_tail(self):
        legacy_preview = {
            "coverage_ratio": 0.5,
            "start_anchored": False,
            "fully_continuous": False,
            "longest_unusable_gap_m": 2.5,
            "longest_internal_unusable_gap_m": 0.0,
        }
        result = classify_invalid_support(
            usability(self.full, self.full, self.full, self.full, legacy_preview)
        )
        self.assertFalse(result["flags"]["preview_trailing_tail"])
        self.assertTrue(result["flags"]["preview_entry_missing"])
        self.assertFalse(result["tail_metrics_explicit"])

    def test_config_validation(self):
        self.assertEqual(
            0.75,
            invalid_support_config_from_dict(
                {"whole_support_min_coverage_ratio": 0.75}
            ).whole_support_min_coverage_ratio,
        )
        with self.assertRaises(ValueError):
            InvalidSupportClassificationConfig(gap_tolerance_m=-1.0)
        with self.assertRaises(ValueError):
            invalid_support_config_from_dict({"unknown": 1})


if __name__ == "__main__":
    unittest.main()
