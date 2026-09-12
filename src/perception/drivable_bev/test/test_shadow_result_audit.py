#!/usr/bin/env python3

import unittest
from types import SimpleNamespace

from drivable_bev.shadow_result_audit import (
    robustness_delta,
    summarize_boundary_frames,
    summarize_boundary_run_groups,
    typed_boundary_frame,
)


def boundary(role, stable_id, order, reacquired=0):
    return {
        "status": "provisional_shadow_boundary",
        "role": role,
        "stable_id": stable_id,
        "lateral_order_left_to_right": order,
        "reacquired_after_missed_frames": reacquired,
    }


class ShadowResultAuditTest(unittest.TestCase):
    def test_typed_contract_roles_are_derived_only_for_shadow_audit(self):
        def value(order, color, tracking_id, y):
            return SimpleNamespace(
                lateral_order=order,
                order_valid=True,
                color=color,
                tracking_id=tracking_id,
                geometric_quality=0.8,
                semantic_score=0.9,
                reference_x_m=5.0,
                reference_y_m=y,
                valid_x_min_m=0.0,
                valid_x_max_m=15.0,
                source_mask=7,
                observed_length_m=14.0,
                bridged_length_ratio=0.1,
                age_frames=4,
                reacquired_after_missed_frames=0,
            )

        frame = typed_boundary_frame(
            SimpleNamespace(
                valid=True,
                boundaries=[
                    value(2, 1, 12, -3.5),
                    value(0, 1, 10, 3.5),
                    value(1, 2, 11, 0.0),
                ],
            )
        )
        self.assertEqual(
            [
                "left_outer_boundary",
                "yellow_centre_boundary",
                "right_outer_boundary",
            ],
            [item["role"] for item in frame],
        )
        self.assertEqual([10, 11, 12], [item["stable_id"] for item in frame])

    def test_invalid_typed_frame_is_not_treated_as_fresh_empty_observation(self):
        self.assertIsNone(
            typed_boundary_frame(SimpleNamespace(valid=False, boundaries=[]))
        )

    def test_order_invalid_typed_frame_does_not_reconstruct_shadow_roles(self):
        def value(order, color):
            return SimpleNamespace(
                lateral_order=order,
                order_valid=False,
                color=color,
                tracking_id=order + 1,
                geometric_quality=0.8,
                semantic_score=0.9,
                reference_x_m=5.0,
                reference_y_m=float(1 - order),
                valid_x_min_m=0.0,
                valid_x_max_m=15.0,
                source_mask=7,
                observed_length_m=14.0,
                bridged_length_ratio=0.1,
                age_frames=4,
                reacquired_after_missed_frames=0,
            )

        frame = typed_boundary_frame(
            SimpleNamespace(
                valid=True,
                boundaries=[value(0, 1), value(1, 2), value(2, 1)],
            )
        )
        self.assertEqual(["unresolved"] * 3, [item["role"] for item in frame])
        summary = summarize_boundary_frames([frame])
        self.assertEqual(1, summary["order_invalid_frames"])
        self.assertEqual(0, summary["corridor_complete_frames"])

    def test_summarizes_empty_partial_and_complete_frames(self):
        frames = [
            [],
            [boundary("yellow_centre_boundary", 1, 0)],
            [
                boundary("left_outer_boundary", 2, 0),
                boundary("yellow_centre_boundary", 1, 1),
                boundary("right_outer_boundary", 3, 2, reacquired=1),
            ],
        ]
        value = summarize_boundary_frames(frames)
        self.assertEqual(3, value["frames"])
        self.assertAlmostEqual(2.0 / 3.0, value["any_boundary_rate"], places=6)
        self.assertAlmostEqual(1.0 / 3.0, value["corridor_complete_rate"], places=6)
        self.assertEqual(1, value["reacquired_events"])
        self.assertEqual(0, value["lateral_order_error_frames"])

    def test_counts_role_id_switches_and_order_errors(self):
        frames = [
            [boundary("yellow_centre_boundary", 1, 0)],
            [boundary("yellow_centre_boundary", 7, 0)],
            [
                boundary("left_outer_boundary", 2, 1),
                boundary("yellow_centre_boundary", 7, 0),
            ],
        ]
        value = summarize_boundary_frames(frames)
        self.assertEqual(1, value["stable_id_switches_total"])
        self.assertEqual(1, value["lateral_order_error_frames"])

    def test_multiple_unresolved_boundaries_are_not_false_id_switches(self):
        frames = [
            [
                boundary("unresolved", 1, 0),
                boundary("unresolved", 2, 1),
            ],
            [
                boundary("unresolved", 1, 0),
                boundary("unresolved", 2, 1),
            ],
        ]
        value = summarize_boundary_frames(frames)
        self.assertEqual(0, value["stable_id_switches_total"])

    def test_run_group_does_not_count_cross_run_id_change_as_switch(self):
        first = [[boundary("yellow_centre_boundary", 1, 0)]]
        second = [[boundary("yellow_centre_boundary", 9, 0)]]
        value = summarize_boundary_run_groups([first, second])
        self.assertEqual(0, value["stable_id_switches_total"])
        self.assertEqual(2, value["unique_stable_ids"])

    def test_robustness_delta_is_static_minus_clear(self):
        value = robustness_delta(
            {"yellow_centre_rate": 0.9, "corridor_complete_rate": 0.7},
            {"yellow_centre_rate": 0.8, "corridor_complete_rate": 0.4},
        )
        self.assertAlmostEqual(-0.1, value["yellow_centre_rate"])
        self.assertAlmostEqual(-0.3, value["corridor_complete_rate"])


if __name__ == "__main__":
    unittest.main()
