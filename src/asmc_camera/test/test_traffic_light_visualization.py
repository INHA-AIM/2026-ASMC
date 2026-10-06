#!/usr/bin/env python3
"""Unit tests for the traffic-light debug overlay helpers."""

import unittest

import numpy as np

from asmc_camera.traffic_light_visualization import (
    INVALID_COLOR_BGR,
    STATE_COLORS_BGR,
    StampedCache,
    border_color,
    overlay_lines,
    render_traffic_light_overlay,
)


RED = 2
GREEN = 4


class StampedCacheTest(unittest.TestCase):
    def test_exact_lookup_latest_and_eviction(self):
        cache = StampedCache(2)
        cache.put(10, "a")
        cache.put(20, "b")
        cache.put(30, "c")
        self.assertIsNone(cache.get(10))
        self.assertEqual("b", cache.get(20))
        self.assertEqual("c", cache.latest())
        self.assertEqual(2, len(cache))

    def test_reinserted_stamp_becomes_latest(self):
        cache = StampedCache(3)
        cache.put(10, "a")
        cache.put(20, "b")
        cache.put(10, "a2")
        self.assertEqual("a2", cache.latest())

    def test_rejects_non_positive_capacity(self):
        with self.assertRaises(ValueError):
            StampedCache(0)


class OverlayTest(unittest.TestCase):
    def setUp(self):
        self.image = np.full((480, 640, 3), 40, dtype=np.uint8)

    def test_border_uses_state_colour_only_when_valid_and_fresh(self):
        self.assertEqual(STATE_COLORS_BGR[RED], border_color(RED, True, False))
        self.assertEqual(INVALID_COLOR_BGR, border_color(RED, False, False))
        self.assertEqual(INVALID_COLOR_BGR, border_color(GREEN, True, True))
        self.assertEqual(INVALID_COLOR_BGR, border_color(99, True, False))

    def test_render_keeps_shape_draws_border_and_leaves_input_untouched(self):
        original = self.image.copy()
        rendered = render_traffic_light_overlay(
            self.image, RED, "red", 0.93, True, False, 3.1,
            ["red", "yellow"], [0.93, 0.05],
        )
        self.assertEqual(self.image.shape, rendered.shape)
        np.testing.assert_array_equal(original, self.image)
        self.assertEqual(STATE_COLORS_BGR[RED], tuple(int(v) for v in rendered[-1, -1]))
        self.assertFalse(np.array_equal(rendered, self.image))

    def test_lines_report_status_candidates_and_unmatched_source(self):
        lines = overlay_lines(
            "green", 0.8, True, True, 2.0, ["green", "left", "red", "x"],
            [0.8, 0.1, 0.05, 0.01], False,
        )
        self.assertTrue(lines[0].startswith("green"))
        self.assertIn("STALE", lines[1])
        self.assertEqual(3, lines[2].count(" | ") + 1)
        self.assertIn("not cached", lines[3])

    def test_rejects_non_bgr_input(self):
        with self.assertRaises(ValueError):
            render_traffic_light_overlay(
                np.zeros((10, 10), dtype=np.uint8), RED, "red", 1.0, True, False, 1.0
            )


if __name__ == "__main__":
    unittest.main()
