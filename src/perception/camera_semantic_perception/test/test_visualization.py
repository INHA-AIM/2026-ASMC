#!/usr/bin/env python3

import unittest

import numpy as np

from camera_semantic_perception.visualization import (
    DRIVABLE_COLOR_BGR,
    LANE_COLOR_BGR,
    hard_mask,
    render_mosaic,
    render_overlay,
    render_panel,
)


class VisualizationTest(unittest.TestCase):
    def test_hard_mask_accepts_uint8_probability(self):
        probability = np.array([[0, 127, 128, 255]], dtype=np.uint8)
        np.testing.assert_array_equal(
            hard_mask(probability, 0.5), [[False, False, True, True]]
        )

    def test_overlay_gives_lane_priority(self):
        image = np.zeros((4, 6, 3), dtype=np.uint8)
        drivable = np.ones((4, 6), dtype=np.float32)
        lane = np.zeros((4, 6), dtype=np.float32)
        lane[1, 2] = 1.0
        overlay = render_overlay(image, drivable, lane, alpha=1.0)
        self.assertEqual(tuple(overlay[0, 0]), DRIVABLE_COLOR_BGR)
        self.assertEqual(tuple(overlay[1, 2]), LANE_COLOR_BGR)

    def test_panel_preserves_source_aspect_ratio(self):
        image = np.zeros((720, 1280, 3), dtype=np.uint8)
        probability = np.zeros((720, 1280), dtype=np.float32)
        panel = render_panel(image, probability, probability, "front", panel_width=1280)
        self.assertEqual(panel.shape, (720, 1280, 3))

    def test_three_view_mosaic_order_and_shape(self):
        panels = {
            "front": np.full((720, 1280, 3), 10, dtype=np.uint8),
            "left": np.full((720, 1280, 3), 20, dtype=np.uint8),
            "right": np.full((720, 1280, 3), 30, dtype=np.uint8),
        }
        mosaic = render_mosaic(
            panels, ("front", "left", "right"), per_view_width=640
        )
        self.assertEqual(mosaic.shape, (360, 1920, 3))
        self.assertEqual(int(mosaic[200, 100, 0]), 10)
        self.assertEqual(int(mosaic[200, 700, 0]), 20)
        self.assertEqual(int(mosaic[200, 1400, 0]), 30)

    def test_shape_mismatch_is_rejected(self):
        image = np.zeros((10, 20, 3), dtype=np.uint8)
        probability = np.zeros((9, 20), dtype=np.float32)
        with self.assertRaisesRegex(ValueError, "does not match"):
            render_overlay(image, probability, probability)


if __name__ == "__main__":
    unittest.main()
