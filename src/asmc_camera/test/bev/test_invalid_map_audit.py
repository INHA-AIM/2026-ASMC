#!/usr/bin/env python3

import unittest

import numpy as np

from asmc_camera.offline.bev.invalid_map_audit import (
    InvalidMapReviewSampler,
    invalid_category_masks,
    invalid_distance_band,
)


class InvalidMapAuditTest(unittest.TestCase):
    def test_error_bands_are_non_binding_labels(self):
        self.assertEqual("supportive", invalid_distance_band(0.35))
        self.assertEqual("review", invalid_distance_band(0.36))
        self.assertEqual("priority_review", invalid_distance_band(0.61))

    def test_builds_leading_trailing_and_internal_masks(self):
        x = np.arange(0.0, 20.25, 0.25)
        state = np.ones(len(x), dtype=np.uint8)
        state[(x <= 1.0) | ((x >= 7.0) & (x <= 7.5)) | (x >= 18.0)] = 0
        classification = {
            "flags": {
                "vehicle_near_leading_tail": True,
                "preview_trailing_tail": True,
                "whole_support_shrink": False,
                "internal_support_gap": True,
                "preview_entry_missing": False,
            },
            "measurements": {
                "control_x_min_m": 0.0,
                "control_x_max_m": 15.0,
                "vehicle_near_leading_tail_m": 1.0,
                "preview_x_min_m": 15.0,
                "preview_x_max_m": 20.0,
                "preview_trailing_tail_m": 2.0,
            },
            "thresholds": {"gap_tolerance_m": 0.0},
        }
        masks = invalid_category_masks(x, state, {}, classification)
        np.testing.assert_array_equal(x[masks["vehicle_near_leading_tail"]], x[x <= 1.0])
        np.testing.assert_array_equal(x[masks["preview_trailing_tail"]], x[x >= 18.0])
        np.testing.assert_array_equal(x[masks["internal_support_gap"]], x[(x >= 7.0) & (x <= 7.5)])

    def test_sampler_is_sparse_per_category_and_band(self):
        sampler = InvalidMapReviewSampler(2, 10)
        self.assertEqual((("tail", "review"),), sampler.select(0, {"tail": "review"}))
        self.assertEqual((), sampler.select(5, {"tail": "review"}))
        self.assertEqual((("tail", "review"),), sampler.select(10, {"tail": "review"}))
        self.assertEqual((), sampler.select(20, {"tail": "review"}))


if __name__ == "__main__":
    unittest.main()
