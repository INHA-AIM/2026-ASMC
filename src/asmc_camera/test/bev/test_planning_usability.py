#!/usr/bin/env python3

import unittest

import numpy as np

from asmc_camera.bev.planning_usability import (
    LongitudinalZone,
    measure_planning_usability,
    planning_zones_from_config,
)
from asmc_camera.bev.shared_lane_family import (
    LEFT_ROLE,
    RIGHT_ROLE,
    SAMPLE_INVALID,
    SAMPLE_OBSERVED,
    SAMPLE_STRUCTURE_INFERRED,
    SharedFamilyBoundary,
    SharedLaneFamily,
    YELLOW_ROLE,
)


def family_with_states(left, yellow, right, x=None):
    states = {
        LEFT_ROLE: np.asarray(left, dtype=np.uint8),
        YELLOW_ROLE: np.asarray(yellow, dtype=np.uint8),
        RIGHT_ROLE: np.asarray(right, dtype=np.uint8),
    }
    x = np.arange(len(states[LEFT_ROLE]), dtype=np.float64) if x is None else np.asarray(x)
    centre = np.column_stack((x, np.zeros_like(x)))
    boundaries = []
    for role, offset in ((LEFT_ROLE, 3.5), (YELLOW_ROLE, 0.0), (RIGHT_ROLE, -3.5)):
        sample_state = states[role]
        boundaries.append(
            SharedFamilyBoundary(
                role=role,
                class_id=2 if role == YELLOW_ROLE else 1,
                state="observed",
                sample_points=centre + np.asarray([0.0, offset]),
                sample_observed=sample_state == SAMPLE_OBSERVED,
                sample_state=sample_state,
                confidence=0.9,
                source_mask=1,
                observed_length_m=0.0,
                inferred_length_m=0.0,
                invalid_length_m=0.0,
                max_inferred_gap_m=0.0,
                bridged_length_ratio=0.0,
                residual_p50_m=0.0,
                residual_p95_m=0.0,
            )
        )
    return SharedLaneFamily(
        control_points=centre,
        knots=np.zeros(1),
        degree=3,
        centre_points=centre,
        sample_valid=np.ones(len(centre), dtype=bool),
        local_support_count=np.full(len(centre), 3, dtype=np.uint8),
        left_width_m=3.5,
        right_width_m=3.5,
        boundaries=tuple(boundaries),
        valid_x_min_m=float(x.min()),
        valid_x_max_m=float(x.max()),
        data_residual_p50_m=0.0,
        data_residual_p95_m=0.0,
        inlier_residual_p50_m=0.0,
        inlier_residual_p95_m=0.0,
        outlier_rate=0.0,
        weighted_rms_m=0.0,
        iterations=1,
    )


class PlanningUsabilityTest(unittest.TestCase):
    def setUp(self):
        self.zone = (LongitudinalZone("test", 0.0, 5.0),)

    def test_fully_observed_family_is_fully_continuous(self):
        observed = [SAMPLE_OBSERVED] * 6
        metric = measure_planning_usability(
            family_with_states(observed, observed, observed), self.zone
        )["zones"]["test"]["both_strips"]
        self.assertEqual(1.0, metric["coverage_ratio"])
        self.assertEqual(1.0, metric["observed_coverage_ratio"])
        self.assertTrue(metric["fully_continuous"])
        self.assertEqual(5.0, metric["continuous_from_start_m"])
        self.assertEqual(0.0, metric["leading_unusable_gap_m"])
        self.assertEqual(0.0, metric["trailing_unusable_gap_m"])
        self.assertEqual(0.0, metric["first_usable_x_m"])
        self.assertEqual(5.0, metric["last_usable_x_m"])

    def test_inferred_support_is_usable_but_degraded(self):
        observed = [SAMPLE_OBSERVED] * 6
        inferred = [SAMPLE_STRUCTURE_INFERRED] * 6
        metric = measure_planning_usability(
            family_with_states(observed, inferred, observed), self.zone
        )["zones"]["test"]["both_strips"]
        self.assertEqual(1.0, metric["coverage_ratio"])
        self.assertEqual(0.0, metric["observed_coverage_ratio"])
        self.assertEqual(1.0, metric["degraded_coverage_ratio"])

    def test_one_invalid_outer_boundary_only_breaks_its_strip(self):
        observed = [SAMPLE_OBSERVED] * 6
        left = [SAMPLE_INVALID] * 6
        zones = measure_planning_usability(
            family_with_states(left, observed, observed), self.zone
        )["zones"]["test"]
        self.assertEqual(0.0, zones["left_strip"]["coverage_ratio"])
        self.assertEqual(1.0, zones["right_strip"]["coverage_ratio"])
        self.assertEqual(0.0, zones["both_strips"]["coverage_ratio"])

    def test_internal_gap_is_separate_from_unanchored_tail(self):
        state = [
            SAMPLE_OBSERVED,
            SAMPLE_OBSERVED,
            SAMPLE_INVALID,
            SAMPLE_OBSERVED,
            SAMPLE_OBSERVED,
            SAMPLE_INVALID,
        ]
        metric = measure_planning_usability(
            family_with_states(state, state, state), self.zone
        )["zones"]["test"]["both_strips"]
        self.assertAlmostEqual(2.0, metric["coverage_ratio"] * 5.0)
        self.assertAlmostEqual(2.0, metric["longest_internal_unusable_gap_m"])
        self.assertAlmostEqual(2.0, metric["longest_unusable_gap_m"])
        self.assertAlmostEqual(1.0, metric["continuous_from_start_m"])
        self.assertEqual(0.0, metric["leading_unusable_gap_m"])
        self.assertEqual(1.0, metric["trailing_unusable_gap_m"])

    def test_zone_clipping_and_missing_family(self):
        observed = [SAMPLE_OBSERVED] * 4
        metric = measure_planning_usability(
            family_with_states(observed, observed, observed, x=[1.0, 2.0, 3.0, 4.0]),
            self.zone,
        )["zones"]["test"]["both_strips"]
        self.assertAlmostEqual(3.0 / 5.0, metric["coverage_ratio"])
        self.assertFalse(metric["start_anchored"])
        self.assertEqual(0.0, metric["continuous_from_start_m"])
        self.assertEqual(1.0, metric["leading_unusable_gap_m"])
        self.assertEqual(1.0, metric["trailing_unusable_gap_m"])

        missing = measure_planning_usability(None, self.zone)["zones"]["test"][
            "both_strips"
        ]
        self.assertEqual(0.0, missing["coverage_ratio"])
        self.assertEqual(5.0, missing["longest_unusable_gap_m"])

    def test_invalid_zone_and_unknown_state_are_rejected(self):
        with self.assertRaises(ValueError):
            LongitudinalZone("bad", 1.0, 1.0)
        observed = [SAMPLE_OBSERVED] * 6
        broken = family_with_states(observed, observed, observed)
        broken.boundaries[0].sample_state[2] = 9
        with self.assertRaises(ValueError):
            measure_planning_usability(broken, self.zone)

    def test_zone_bounds_can_be_extended_by_configuration(self):
        zones = planning_zones_from_config(
            [
                {"name": "planning_consumer", "x_min_m": 0.0, "x_max_m": 18.0},
                {"name": "far_debug", "x_min_m": 18.0, "x_max_m": 25.0},
            ]
        )
        self.assertEqual(18.0, zones[0].x_max_m)
        self.assertEqual(18.0, zones[1].x_min_m)
        with self.assertRaises(ValueError):
            planning_zones_from_config(
                [{"name": "missing_max", "x_min_m": 0.0}]
            )


if __name__ == "__main__":
    unittest.main()
