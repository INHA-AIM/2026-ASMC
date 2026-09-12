#!/usr/bin/env python3

import unittest

import numpy as np

from drivable_bev.boundary_candidates import (
    TemporalBoundaryTracker,
    select_shadow_lane_tracks,
    select_shadow_boundaries,
)
from drivable_bev.lane_evidence import LaneNode
from drivable_bev.lane_tracking import LaneTrack
from drivable_bev.spline_fitting import FittedLane


def lane(lane_id, class_id, y, x_start=0.0, x_end=15.0, confidence=0.9,
         bridged_ratio=0.05, residual=0.04, lateral_slope=0.0):
    x = np.linspace(x_start, x_end, 61)
    points = np.column_stack((x, y + lateral_slope * x))
    return FittedLane(
        lane_id=lane_id,
        class_id=class_id,
        control_points=points[::15],
        knots=np.linspace(0.0, 1.0, 8),
        degree=3,
        sample_points=points,
        sample_observed=np.ones(len(points), dtype=bool),
        confidence=confidence,
        source_mask=7,
        valid_x_min_m=x_start,
        valid_x_max_m=x_end,
        observed_length_m=float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum()),
        bridged_length_ratio=bridged_ratio,
        residual_p50_m=residual / 2.0,
        residual_p95_m=residual,
    )


def track(class_id, y, x_end=15.0, lateral_slope=0.0):
    x = np.linspace(0.0, x_end, 31)
    return LaneTrack(
        tuple(
            LaneNode(
                xy=np.asarray([value, y + lateral_slope * value]),
                class_id=class_id,
                confidence=0.9,
                geometric_quality=1.0,
                source_mask=7,
                support=8,
                x_bin=index,
            )
            for index, value in enumerate(x)
        )
    )


class ShadowBoundarySelectionTest(unittest.TestCase):
    def test_prefit_selection_bounds_cubic_work_to_three_tracks(self):
        tracks = [
            track(2, 0.0),
            track(1, 3.55),
            track(1, -3.55),
            track(1, 6.8),
            track(1, -6.7),
            track(1, 1.0, x_end=1.5),
        ]
        selected = select_shadow_lane_tracks(tracks)
        self.assertEqual(3, len(selected))
        self.assertEqual([1, 2, 1], [item.class_id for item in selected])

    def test_selects_yellow_and_one_outer_white_per_side(self):
        values = [
            lane(1, 2, 0.4),
            lane(2, 1, 3.95),
            lane(3, 1, -3.15),
            lane(4, 1, 6.8, confidence=0.7),
        ]
        selected = select_shadow_boundaries(values)
        self.assertEqual(3, len(selected))
        self.assertEqual(
            ["left_outer_boundary", "yellow_centre_boundary", "right_outer_boundary"],
            [item.role for item in selected],
        )
        self.assertEqual([0, 1, 2], [item.lateral_order_left_to_right for item in selected])

    def test_rejects_short_transverse_and_high_residual_markings(self):
        selected = select_shadow_boundaries(
            [
                lane(1, 2, 0.0),
                lane(2, 1, 3.5, x_end=2.0),
                lane(3, 1, -3.5, lateral_slope=2.0),
                lane(4, 1, -3.5, residual=0.8),
            ]
        )
        self.assertEqual(1, len(selected))
        self.assertEqual("yellow_centre_boundary", selected[0].role)

    def test_without_yellow_keeps_separated_whites_but_no_topology_role(self):
        selected = select_shadow_boundaries(
            [lane(1, 1, 2.0), lane(2, 1, 2.2), lane(3, 1, -2.5)]
        )
        self.assertEqual(2, len(selected))
        self.assertTrue(all(item.role == "unresolved" for item in selected))


class TemporalBoundaryTrackerTest(unittest.TestCase):
    def test_ids_survive_lateral_motion_and_one_missing_frame(self):
        tracker = TemporalBoundaryTracker(max_lateral_residual_m=0.7, max_missed_frames=2)
        first = select_shadow_boundaries(
            [lane(1, 2, 0.2), lane(2, 1, 3.75), lane(3, 1, -3.35)]
        )
        output1 = tracker.update(first, 1_000_000_000)
        ids_by_role = {item.observation.role: item.stable_id for item in output1}

        second = select_shadow_boundaries(
            [lane(4, 2, 0.0), lane(5, 1, 3.55), lane(6, 1, -3.55)]
        )
        output2 = tracker.update(second, 1_100_000_000)
        self.assertEqual(
            ids_by_role,
            {item.observation.role: item.stable_id for item in output2},
        )

        missing = tuple(item for item in second if item.role != "left_outer_boundary")
        tracker.update(missing, 1_200_000_000)
        output4 = tracker.update(second, 1_300_000_000)
        left = next(item for item in output4 if item.observation.role == "left_outer_boundary")
        self.assertEqual(ids_by_role["left_outer_boundary"], left.stable_id)
        self.assertEqual(1, left.reacquired_after_missed_frames)

    def test_large_timestamp_gap_does_not_reuse_old_id(self):
        tracker = TemporalBoundaryTracker(reset_gap_sec=0.5)
        observation = select_shadow_boundaries([lane(1, 2, 0.0)])
        first_id = tracker.update(observation, 1_000_000_000)[0].stable_id
        second_id = tracker.update(observation, 2_000_000_000)[0].stable_id
        self.assertNotEqual(first_id, second_id)


if __name__ == "__main__":
    unittest.main()
