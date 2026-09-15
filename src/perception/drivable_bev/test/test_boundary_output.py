#!/usr/bin/env python3

import unittest

import numpy as np

from drivable_bev.boundary_candidates import (
    BoundaryObservation,
    TemporalBoundaryTracker,
    TrackedBoundary,
    select_shadow_boundaries,
)
from drivable_bev.boundary_output import (
    COLOR_WHITE,
    COLOR_YELLOW,
    COMPLETENESS_COMPLETE,
    COMPLETENESS_EMPTY,
    COMPLETENESS_PARTIAL,
    build_shadow_boundary_output,
)
from drivable_bev.spline_fitting import FittedLane


def lane(lane_id, class_id, y, x_start=0.0, x_end=15.0):
    x = np.linspace(x_start, x_end, 61)
    points = np.column_stack((x, np.full_like(x, y)))
    return FittedLane(
        lane_id=lane_id,
        class_id=class_id,
        control_points=points[::15],
        knots=np.linspace(0.0, 1.0, 8),
        degree=3,
        sample_points=points,
        sample_observed=np.ones(len(points), dtype=bool),
        confidence=0.9,
        source_mask=7,
        valid_x_min_m=x_start,
        valid_x_max_m=x_end,
        observed_length_m=x_end - x_start,
        bridged_length_ratio=0.05,
        residual_p50_m=0.02,
        residual_p95_m=0.04,
    )


def tracked(lanes):
    observations = select_shadow_boundaries(lanes)
    return TemporalBoundaryTracker().update(observations, 1_000_000_000)


class BoundaryOutputTest(unittest.TestCase):
    def test_complete_frame_is_generic_and_left_to_right(self):
        value = build_shadow_boundary_output(
            tracked(
                [
                    lane(1, 2, 0.2),
                    lane(2, 1, 3.75),
                    lane(3, 1, -3.35),
                ]
            ),
            preferred_reference_x_m=5.0,
        )
        self.assertEqual(COMPLETENESS_COMPLETE, value.completeness)
        self.assertEqual(3, value.expected_boundary_count)
        self.assertEqual([0, 1, 2], [item.lateral_order for item in value.boundaries])
        self.assertEqual(
            [COLOR_WHITE, COLOR_YELLOW, COLOR_WHITE],
            [item.color for item in value.boundaries],
        )
        self.assertTrue(all(item.order_valid for item in value.boundaries))
        self.assertGreater(
            value.boundaries[0].reference_y_m,
            value.boundaries[1].reference_y_m,
        )
        self.assertGreater(
            value.boundaries[1].reference_y_m,
            value.boundaries[2].reference_y_m,
        )
        self.assertTrue(
            all(np.all(np.diff(item.polyline_xy[:, 0]) >= 0.0) for item in value.boundaries)
        )

    def test_missing_yellow_is_partial_without_public_role(self):
        value = build_shadow_boundary_output(
            tracked([lane(1, 1, 2.0), lane(2, 1, -2.5)]),
            preferred_reference_x_m=5.0,
        )
        self.assertEqual(COMPLETENESS_PARTIAL, value.completeness)
        self.assertEqual(2, len(value.boundaries))
        self.assertTrue(all(item.color == COLOR_WHITE for item in value.boundaries))
        self.assertFalse(hasattr(value.boundaries[0], "role"))

    def test_empty_frame_is_valid_empty_payload(self):
        value = build_shadow_boundary_output((), preferred_reference_x_m=5.0)
        self.assertEqual(COMPLETENESS_EMPTY, value.completeness)
        self.assertEqual((), value.boundaries)

    def test_reference_moves_to_shared_support_when_preferred_x_is_unavailable(self):
        value = build_shadow_boundary_output(
            tracked([lane(1, 2, 0.0, x_start=-4.0, x_end=2.0)]),
            preferred_reference_x_m=5.0,
        )
        self.assertEqual(COMPLETENESS_PARTIAL, value.completeness)
        self.assertTrue(value.boundaries[0].order_valid)
        self.assertAlmostEqual(2.0, value.boundaries[0].reference_x_m)

    def test_shared_reference_recovers_order_for_unequal_curve_ranges(self):
        value = build_shadow_boundary_output(
            tracked(
                [
                    lane(1, 2, 0.0, x_start=-3.0, x_end=3.0),
                    lane(2, 1, 3.5, x_start=-2.0, x_end=2.0),
                    lane(3, 1, -3.5, x_start=-1.0, x_end=4.0),
                ]
            ),
            preferred_reference_x_m=5.0,
        )
        self.assertTrue(all(item.order_valid for item in value.boundaries))
        self.assertEqual([2.0, 2.0, 2.0], [item.reference_x_m for item in value.boundaries])
        self.assertEqual([0, 1, 2], [item.lateral_order for item in value.boundaries])
        self.assertGreater(value.boundaries[0].reference_y_m, value.boundaries[1].reference_y_m)
        self.assertGreater(value.boundaries[1].reference_y_m, value.boundaries[2].reference_y_m)

    def test_disjoint_curve_ranges_keep_order_invalid(self):
        first = tracked([lane(1, 2, 0.0, x_start=-4.0, x_end=-1.0)])[0]
        second_value = tracked([lane(2, 1, 3.5, x_start=1.0, x_end=4.0)])[0]
        second = TrackedBoundary(
            stable_id=2,
            observation=BoundaryObservation(
                lane=second_value.observation.lane,
                role=second_value.observation.role,
                score=second_value.observation.score,
                reference_y_m=second_value.observation.reference_y_m,
                heading_deg=second_value.observation.heading_deg,
                lateral_order_left_to_right=1,
            ),
            age_frames=second_value.age_frames,
            reacquired_after_missed_frames=second_value.reacquired_after_missed_frames,
        )
        value = build_shadow_boundary_output(
            [first, second], preferred_reference_x_m=5.0
        )
        self.assertTrue(all(not item.order_valid for item in value.boundaries))

    def test_invalid_polyline_fails_instead_of_publishing_bad_geometry(self):
        bad_lane = lane(1, 2, 0.0)
        object.__setattr__(bad_lane, "sample_points", np.asarray([[0.0, np.nan], [1.0, 0.0]]))
        invalid = TrackedBoundary(
            stable_id=1,
            observation=BoundaryObservation(
                lane=bad_lane,
                role="yellow_centre_boundary",
                score=0.8,
                reference_y_m=0.0,
                heading_deg=0.0,
                lateral_order_left_to_right=0,
            ),
            age_frames=1,
            reacquired_after_missed_frames=0,
        )
        with self.assertRaises(ValueError):
            build_shadow_boundary_output(
                [invalid], preferred_reference_x_m=0.5
            )

    def test_public_polyline_is_clipped_to_declared_valid_x(self):
        fitted = lane(1, 2, 0.0, x_start=0.0, x_end=15.0)
        extended_x = np.linspace(-1.0, 16.0, 69)
        object.__setattr__(
            fitted,
            "sample_points",
            np.column_stack((extended_x, np.zeros_like(extended_x))),
        )
        value = build_shadow_boundary_output(
            tracked([fitted]), preferred_reference_x_m=5.0
        )
        self.assertAlmostEqual(0.0, value.boundaries[0].polyline_xy[0, 0])
        self.assertAlmostEqual(15.0, value.boundaries[0].polyline_xy[-1, 0])


if __name__ == "__main__":
    unittest.main()
