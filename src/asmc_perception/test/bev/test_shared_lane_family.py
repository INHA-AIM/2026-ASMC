#!/usr/bin/env python3

import unittest

import numpy as np

from asmc_perception.bev.boundary_family_output import (
    ROLE_LEFT_OUTER_BOUNDARY,
    ROLE_RIGHT_OUTER_BOUNDARY,
    ROLE_ROAD_CENTRE_BOUNDARY,
    SUPPORT_INVALID,
    SUPPORT_OBSERVED,
    SUPPORT_STRUCTURE_INFERRED,
    build_boundary_family_output,
)
from asmc_perception.bev.lane_evidence import LaneNode
from asmc_perception.bev.lane_tracking import LaneTrack
from asmc_perception.bev.shared_lane_family import (
    LEFT_ROLE,
    OBSERVED,
    RIGHT_ROLE,
    SAMPLE_INVALID,
    SAMPLE_OBSERVED,
    SAMPLE_STRUCTURE_INFERRED,
    STRUCTURE_INFERRED,
    YELLOW_ROLE,
    contiguous_sample_state_segments,
    fit_shared_lane_family,
    fit_shared_lane_family_with_audit,
)


def track(class_id, offset, noise=0.0, outlier=False, x_min=0.0, x_max=18.0):
    x = np.arange(float(x_min), float(x_max) + 0.01, 0.5)
    centre_y = 0.0025 * (x - 8.0) ** 2 - 0.10
    slope = 0.005 * (x - 8.0)
    normal = np.column_stack((-slope, np.ones_like(slope)))
    normal /= np.linalg.norm(normal, axis=1)[:, None]
    xy = np.column_stack((x, centre_y)) + float(offset) * normal
    if noise:
        xy[:, 1] += noise * np.sin(np.arange(len(x)) * 1.7)
    if outlier:
        xy[len(x) // 2, 1] += 1.5
    return LaneTrack(
        tuple(
            LaneNode(
                xy=value,
                class_id=class_id,
                confidence=0.90,
                geometric_quality=0.85,
                source_mask=1 << (index % 3),
                support=4,
                x_bin=index,
            )
            for index, value in enumerate(xy)
        )
    )


def without_station_range(value, lower, upper):
    return LaneTrack(
        tuple(
            node
            for node in value.nodes
            if not float(lower) <= float(node.xy[0]) <= float(upper)
        )
    )


def add_lateral_slope(value, slope):
    nodes = []
    for node in value.nodes:
        xy = np.asarray(node.xy, dtype=np.float64).copy()
        xy[1] += float(slope) * float(xy[0])
        nodes.append(
            LaneNode(
                xy=xy,
                class_id=node.class_id,
                confidence=node.confidence,
                geometric_quality=node.geometric_quality,
                source_mask=node.source_mask,
                support=node.support,
                x_bin=node.x_bin,
            )
        )
    return LaneTrack(tuple(nodes))


class SharedLaneFamilyTest(unittest.TestCase):
    def test_candidate_output_preserves_inferred_and_invalid_samples(self):
        family = fit_shared_lane_family(
            [
                track(1, 3.50, x_max=10.0),
                track(1, -3.50, x_max=18.0),
            ]
        )
        output = build_boundary_family_output(family, "accepted")
        self.assertTrue(output.valid)
        self.assertEqual("accepted", output.status_reason)
        self.assertEqual(
            [
                ROLE_LEFT_OUTER_BOUNDARY,
                ROLE_ROAD_CENTRE_BOUNDARY,
                ROLE_RIGHT_OUTER_BOUNDARY,
            ],
            [value.role for value in output.boundaries],
        )
        yellow = output.boundaries[1]
        self.assertEqual(SUPPORT_STRUCTURE_INFERRED, yellow.overall_state)
        self.assertIn(SUPPORT_STRUCTURE_INFERRED, yellow.support_state)
        self.assertIn(SUPPORT_INVALID, yellow.support_state)
        self.assertNotIn(SUPPORT_OBSERVED, yellow.support_state)
        self.assertEqual(len(yellow.polyline_xy), len(yellow.support_state))

    def test_rejected_candidate_is_fresh_invalid_empty_frame(self):
        output = build_boundary_family_output(None, "too_few_observed_roles")
        self.assertFalse(output.valid)
        self.assertEqual("too_few_observed_roles", output.status_reason)
        self.assertEqual(3, output.expected_boundary_count)
        self.assertEqual((), output.boundaries)

    def test_state_segments_do_not_bridge_transitions(self):
        points = np.column_stack((np.arange(8, dtype=np.float64), np.zeros(8)))
        states = np.asarray(
            [
                SAMPLE_OBSERVED,
                SAMPLE_OBSERVED,
                SAMPLE_OBSERVED,
                SAMPLE_STRUCTURE_INFERRED,
                SAMPLE_STRUCTURE_INFERRED,
                SAMPLE_INVALID,
                SAMPLE_INVALID,
                SAMPLE_OBSERVED,
            ],
            dtype=np.uint8,
        )
        segments = contiguous_sample_state_segments(points, states)
        self.assertEqual(
            ["observed", "structure_inferred", "invalid"],
            [state for state, _ in segments],
        )
        self.assertEqual([[0.0, 1.0, 2.0], [3.0, 4.0], [5.0, 6.0]], [
            value[:, 0].tolist() for _, value in segments
        ])

    def test_state_segment_input_is_validated(self):
        with self.assertRaises(ValueError):
            contiguous_sample_state_segments([[0.0, 0.0]], [9])

    def test_three_tracks_share_one_curved_family(self):
        family = fit_shared_lane_family(
            [track(1, 3.42, 0.015), track(2, 0.0, 0.01), track(1, -3.58, 0.015)]
        )
        self.assertAlmostEqual(3.42, family.left_width_m, delta=0.12)
        self.assertAlmostEqual(3.58, family.right_width_m, delta=0.12)
        self.assertLess(family.data_residual_p95_m, 0.08)
        self.assertEqual(
            [LEFT_ROLE, YELLOW_ROLE, RIGHT_ROLE],
            [value.role for value in family.boundaries],
        )
        self.assertTrue(all(value.state == OBSERVED for value in family.boundaries))
        self.assertTrue(
            all(
                np.any(value.sample_state == SAMPLE_OBSERVED)
                for value in family.boundaries
            )
        )
        left = family.boundary(LEFT_ROLE).sample_points
        yellow = family.boundary(YELLOW_ROLE).sample_points
        right = family.boundary(RIGHT_ROLE).sample_points
        self.assertTrue(np.allclose(np.linalg.norm(left - yellow, axis=1), family.left_width_m))
        self.assertTrue(np.allclose(np.linalg.norm(right - yellow, axis=1), family.right_width_m))

    def test_two_whites_infer_missing_yellow(self):
        family, audit = fit_shared_lane_family_with_audit(
            [track(1, 3.50, 0.01), track(1, -3.50, 0.01)]
        )
        self.assertTrue(audit["accepted"])
        self.assertEqual(STRUCTURE_INFERRED, family.boundary(YELLOW_ROLE).state)
        self.assertEqual(OBSERVED, family.boundary(LEFT_ROLE).state)
        self.assertEqual(OBSERVED, family.boundary(RIGHT_ROLE).state)
        self.assertEqual(0, family.boundary(YELLOW_ROLE).source_mask)
        yellow = family.boundary(YELLOW_ROLE)
        self.assertTrue(
            np.all(
                yellow.sample_state[family.sample_valid]
                == SAMPLE_STRUCTURE_INFERRED
            )
        )

    def test_yellow_and_one_white_infer_other_outer_boundary(self):
        family = fit_shared_lane_family([track(2, 0.0), track(1, 3.50)])
        self.assertEqual(OBSERVED, family.boundary(YELLOW_ROLE).state)
        self.assertEqual(OBSERVED, family.boundary(LEFT_ROLE).state)
        self.assertEqual(STRUCTURE_INFERRED, family.boundary(RIGHT_ROLE).state)

    def test_single_track_is_not_promoted_to_family(self):
        family, audit = fit_shared_lane_family_with_audit([track(2, 0.0)])
        self.assertIsNone(family)
        self.assertEqual("too_few_observed_roles", audit["reason"])

    def test_robust_weighting_limits_isolated_outlier(self):
        family = fit_shared_lane_family(
            [track(1, 3.48), track(2, 0.0, outlier=True), track(1, -3.52)]
        )
        self.assertLess(family.data_residual_p50_m, 0.06)
        self.assertLess(family.weighted_rms_m, 0.30)

    def test_width_prior_is_soft_not_fixed(self):
        family = fit_shared_lane_family(
            [track(1, 3.20), track(2, 0.0), track(1, -3.80)],
            {"width_prior_strength": 0.10},
        )
        self.assertLess(family.left_width_m, 3.40)
        self.assertGreater(family.right_width_m, 3.65)
        self.assertNotAlmostEqual(3.48, family.left_width_m, places=3)
        self.assertNotAlmostEqual(3.52, family.right_width_m, places=3)

    def test_result_is_deterministic(self):
        tracks = [track(1, 3.48, 0.01), track(2, 0.0), track(1, -3.52, 0.01)]
        first = fit_shared_lane_family(tracks)
        second = fit_shared_lane_family(tracks)
        self.assertTrue(np.array_equal(first.centre_points, second.centre_points))
        self.assertEqual(first.left_width_m, second.left_width_m)
        self.assertEqual(first.right_width_m, second.right_width_m)
        self.assertTrue(np.array_equal(first.sample_valid, second.sample_valid))

    def test_single_role_tail_is_invalid_not_structurally_extrapolated(self):
        family = fit_shared_lane_family(
            [
                track(1, 3.48, x_max=10.0),
                track(2, 0.0, x_max=10.0),
                track(1, -3.52, x_max=18.0),
            ]
        )
        tail = family.centre_points[:, 0] > 11.0
        self.assertTrue(np.any(tail))
        self.assertTrue(np.all(~family.sample_valid[tail]))
        self.assertTrue(
            np.all(
                family.boundary(RIGHT_ROLE).sample_state[tail] == SAMPLE_INVALID
            )
        )

    def test_bounded_same_frame_completion_promotes_only_candidate_window(self):
        family, audit = fit_shared_lane_family_with_audit(
            [
                track(1, 3.48, x_max=10.0),
                track(2, 0.0, x_max=10.0),
                track(1, -3.52, x_max=18.0),
            ],
            {
                "same_frame_completion_enabled": True,
                "same_frame_completion_x_min_m": 0.0,
                "same_frame_completion_x_max_m": 15.0,
            },
        )
        promoted_tail = (family.centre_points[:, 0] > 11.0) & (
            family.centre_points[:, 0] <= 15.0
        )
        far_tail = family.centre_points[:, 0] > 15.0
        self.assertTrue(np.any(promoted_tail))
        self.assertTrue(np.any(far_tail))
        self.assertTrue(np.all(family.sample_valid[promoted_tail]))
        self.assertTrue(
            np.all(
                family.boundary(RIGHT_ROLE).sample_state[promoted_tail]
                == SAMPLE_STRUCTURE_INFERRED
            )
        )
        self.assertTrue(np.all(~family.sample_valid[far_tail]))
        self.assertTrue(
            np.all(
                family.boundary(RIGHT_ROLE).sample_state[far_tail]
                == SAMPLE_INVALID
            )
        )
        self.assertGreater(
            audit["same_frame_completion"]["promoted_sample_count"], 0
        )
        self.assertFalse(
            audit["same_frame_completion"]["temporal_prediction_used"]
        )

    def test_same_frame_completion_never_accepts_one_role_family(self):
        family, audit = fit_shared_lane_family_with_audit(
            [track(2, 0.0)],
            {
                "same_frame_completion_enabled": True,
                "distributed_support_enabled": True,
            },
        )
        self.assertIsNone(family)
        self.assertEqual("too_few_observed_roles", audit["reason"])

    def test_same_frame_completion_range_is_validated(self):
        with self.assertRaises(ValueError):
            fit_shared_lane_family(
                [track(1, 3.48), track(2, 0.0)],
                {
                    "same_frame_completion_enabled": True,
                    "same_frame_completion_x_min_m": 15.0,
                    "same_frame_completion_x_max_m": 15.0,
                },
            )

    def test_distributed_multi_role_support_recovers_staggered_fragments(self):
        tracks = [
            track(1, 3.48, x_min=0.0, x_max=5.0),
            track(2, 0.0, x_min=5.5, x_max=11.0),
            track(1, -3.52, x_min=11.5, x_max=18.0),
        ]
        strict, strict_audit = fit_shared_lane_family_with_audit(tracks)
        self.assertIsNone(strict)
        self.assertEqual(
            "insufficient_local_multi_role_support", strict_audit["reason"]
        )

        family, audit = fit_shared_lane_family_with_audit(
            tracks,
            {
                "distributed_support_enabled": True,
                "same_frame_completion_enabled": True,
            },
        )
        self.assertIsNotNone(family)
        self.assertEqual("distributed_multi_role", audit["acceptance_mode"])
        self.assertTrue(audit["distributed_support"]["accepted"])
        self.assertEqual(3, len(audit["distributed_support"]["qualified_roles"]))
        self.assertTrue(
            all(
                np.any(boundary.sample_state == SAMPLE_OBSERVED)
                and np.any(boundary.sample_state == SAMPLE_STRUCTURE_INFERRED)
                for boundary in family.boundaries
            )
        )

    def test_distributed_support_rejects_inconsistent_role_headings(self):
        family, audit = fit_shared_lane_family_with_audit(
            [
                track(1, 3.48, x_min=0.0, x_max=5.0),
                add_lateral_slope(
                    track(1, -3.52, x_min=5.5, x_max=11.0), 0.20
                ),
            ],
            {
                "distributed_support_enabled": True,
                "distributed_max_heading_delta_deg": 5.0,
            },
        )
        self.assertIsNone(family)
        self.assertEqual("insufficient_local_multi_role_support", audit["reason"])
        self.assertIn(
            "heading_consistency",
            audit["distributed_support"]["rejection_reasons"],
        )

    def test_distributed_flag_does_not_change_strict_family_geometry(self):
        tracks = [track(1, 3.48), track(2, 0.0), track(1, -3.52)]
        strict, strict_audit = fit_shared_lane_family_with_audit(tracks)
        enabled, enabled_audit = fit_shared_lane_family_with_audit(
            tracks, {"distributed_support_enabled": True}
        )
        self.assertEqual("strict_local_multi_role", strict_audit["acceptance_mode"])
        self.assertEqual("strict_local_multi_role", enabled_audit["acceptance_mode"])
        self.assertFalse(enabled_audit["distributed_support"]["attempted"])
        self.assertTrue(np.array_equal(strict.control_points, enabled.control_points))
        self.assertTrue(np.array_equal(strict.centre_points, enabled.centre_points))
        self.assertTrue(np.array_equal(strict.sample_valid, enabled.sample_valid))
        for first, second in zip(strict.boundaries, enabled.boundaries):
            self.assertTrue(np.array_equal(first.sample_points, second.sample_points))
            self.assertTrue(np.array_equal(first.sample_state, second.sample_state))

    def test_laterally_displaced_support_is_not_marked_observed(self):
        yellow = track(2, 0.0)
        shifted_nodes = []
        for node in yellow.nodes:
            xy = np.asarray(node.xy).copy()
            if 7.0 <= xy[0] <= 10.0:
                xy[1] += 1.0
            shifted_nodes.append(
                LaneNode(
                    xy=xy,
                    class_id=node.class_id,
                    confidence=node.confidence,
                    geometric_quality=node.geometric_quality,
                    source_mask=node.source_mask,
                    support=node.support,
                    x_bin=node.x_bin,
                )
            )
        family = fit_shared_lane_family(
            [track(1, 3.48), LaneTrack(tuple(shifted_nodes)), track(1, -3.52)]
        )
        boundary = family.boundary(YELLOW_ROLE)
        middle = (boundary.sample_points[:, 0] >= 7.5) & (
            boundary.sample_points[:, 0] <= 9.5
        )
        self.assertTrue(np.any(middle))
        self.assertFalse(np.any(boundary.sample_state[middle] == SAMPLE_OBSERVED))
        self.assertTrue(
            np.any(boundary.sample_state[middle] == SAMPLE_STRUCTURE_INFERRED)
        )
        self.assertGreater(family.outlier_rate, 0.0)

    def test_long_internal_single_role_gap_is_invalid_not_bridged(self):
        family = fit_shared_lane_family(
            [
                without_station_range(track(1, 3.48), 7.0, 9.0),
                without_station_range(track(2, 0.0), 7.0, 9.0),
                track(1, -3.52),
            ]
        )
        gap = (family.centre_points[:, 0] >= 7.25) & (
            family.centre_points[:, 0] <= 8.75
        )
        self.assertTrue(np.any(gap))
        self.assertTrue(np.all(~family.sample_valid[gap]))

    def test_short_internal_single_role_gap_is_conservatively_bridged(self):
        family = fit_shared_lane_family(
            [
                without_station_range(track(1, 3.48), 7.5, 8.0),
                without_station_range(track(2, 0.0), 7.5, 8.0),
                track(1, -3.52),
            ]
        )
        gap = (family.centre_points[:, 0] >= 7.5) & (
            family.centre_points[:, 0] <= 8.0
        )
        self.assertTrue(np.any(gap))
        self.assertTrue(np.all(family.sample_valid[gap]))
        self.assertTrue(
            np.all(
                family.boundary(LEFT_ROLE).sample_state[gap]
                == SAMPLE_STRUCTURE_INFERRED
            )
        )


if __name__ == "__main__":
    unittest.main()
