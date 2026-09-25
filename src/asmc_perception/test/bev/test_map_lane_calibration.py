#!/usr/bin/env python3

import unittest

import numpy as np

from asmc_perception.offline.bev.map_lane_calibration import (
    BoundaryResidualEstimate,
    BoundaryResidualProjector,
    calibrated_lane_geometry,
    estimate_residual_peak,
)


def line(y):
    x = np.linspace(0.0, 20.0, 41)
    return np.column_stack((x, np.full_like(x, y), np.zeros_like(x)))


class MapLaneCalibrationTest(unittest.TestCase):
    def test_signed_residuals_follow_positive_family_normal(self):
        projector = BoundaryResidualProjector(
            {
                "white_negative_normal": line(-3.5),
                "yellow_center": line(0.0),
                "white_positive_normal": line(3.5),
            }
        )
        points = np.asarray([[5.0, -3.7], [5.0, 0.1], [5.0, 3.8]])
        result = projector.residuals(points)
        self.assertAlmostEqual(-0.2, result["white_negative_normal"][0], places=6)
        self.assertAlmostEqual(0.1, result["yellow_center"][0], places=6)
        self.assertAlmostEqual(0.3, result["white_positive_normal"][0], places=6)

    def test_histogram_peak_ignores_uniform_outliers(self):
        random = np.random.RandomState(4)
        values = np.concatenate(
            [random.normal(0.24, 0.025, 500), random.uniform(-0.9, 0.9, 120)]
        )
        estimate = estimate_residual_peak(values)
        self.assertAlmostEqual(0.24, estimate.offset_m, delta=0.015)
        self.assertGreater(estimate.peak_fraction, 0.70)

    def test_offsets_convert_to_asymmetric_lane_widths(self):
        def estimate(value):
            return BoundaryResidualEstimate(value, value, 0.01, 20, 20, 1.0)

        geometry = calibrated_lane_geometry(
            3.5,
            {
                "white_negative_normal": estimate(-0.20),
                "yellow_center": estimate(0.10),
                "white_positive_normal": estimate(0.30),
            },
        )
        self.assertAlmostEqual(0.10, geometry["center_lateral_offset_m"])
        self.assertAlmostEqual(3.70, geometry["positive_lane_width_m"])
        self.assertAlmostEqual(3.80, geometry["negative_lane_width_m"])


if __name__ == "__main__":
    unittest.main()
