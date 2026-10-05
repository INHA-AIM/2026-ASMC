#!/usr/bin/env python3

import unittest

from asmc_camera.offline.bev.shared_runtime_audit import (
    summarize_shared_runtime_frames,
    summarize_shared_runtime_run_groups,
)


def frame(accepted=True, inferred=False, acceptance_mode=None):
    return {
        "accepted": accepted,
        "reason": "accepted" if accepted else "too_few_observed_roles",
        "acceptance_mode": acceptance_mode or (
            "strict_local_multi_role" if accepted else "rejected"
        ),
        "deployment_allowed": False,
        "fit_ms": 12.0,
        "observed_roles": [
            "left_outer_boundary",
            "right_outer_boundary",
        ],
        "inferred_roles": ["yellow_centre_boundary"] if inferred else [],
        "widths_m": {"left": 3.5, "right": 3.6},
        "data_residual_p95_m": 0.4,
        "inlier_residual_p95_m": 0.2,
        "outlier_rate": 0.1,
        "valid_x_m": [0.0, 15.0],
        "boundary_support": {
            "left_outer_boundary": {
                "observed_length_m": 10.0,
                "inferred_length_m": 1.0,
                "invalid_length_m": 2.0,
            },
            "yellow_centre_boundary": {
                "observed_length_m": 0.0 if inferred else 10.0,
                "inferred_length_m": 10.0 if inferred else 1.0,
                "invalid_length_m": 2.0,
            },
            "right_outer_boundary": {
                "observed_length_m": 10.0,
                "inferred_length_m": 1.0,
                "invalid_length_m": 2.0,
            },
        },
    }


class SharedRuntimeAuditTest(unittest.TestCase):
    def test_invalid_length_is_never_counted_as_usable(self):
        value = summarize_shared_runtime_frames([frame(inferred=True)])
        lengths = value["sample_length_m"]
        self.assertEqual(6.0, lengths["total"]["invalid"])
        self.assertEqual(
            lengths["total"]["observed"]
            + lengths["total"]["structure_inferred"],
            lengths["usable"],
        )
        self.assertEqual("forbidden", value["consumption_policy"]["invalid"])

    def test_reports_rates_widths_and_latency(self):
        value = summarize_shared_runtime_frames(
            [frame(), frame(False), frame(acceptance_mode="distributed_multi_role")]
        )
        self.assertEqual(3, value["frames"])
        self.assertEqual(0.666667, value["accepted_rate"])
        self.assertEqual(
            1, value["acceptance_modes"]["distributed_multi_role"]
        )
        self.assertEqual(0.333333, value["distributed_multi_role_rate"])
        self.assertEqual(12.0, value["fit_ms"]["p95"])
        self.assertEqual(3.5, value["widths_m"]["left"]["p50"])
        self.assertEqual(1, value["longest_rejected_streak_frames"])

    def test_rejected_streak_does_not_cross_run_boundary(self):
        value = summarize_shared_runtime_run_groups(
            [[frame(False), frame(False)], [frame(False)]]
        )
        self.assertEqual(2, value["longest_rejected_streak_frames"])
        self.assertEqual(2, value["runs"])


if __name__ == "__main__":
    unittest.main()
