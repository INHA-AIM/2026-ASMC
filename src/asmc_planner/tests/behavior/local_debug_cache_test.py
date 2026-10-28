#!/usr/bin/env python3
"""An observer must not draw a different frame as the current planner input."""
import base64
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
from local_debug_cache import LocalDebugCache


class CacheTest(unittest.TestCase):
    def test_exact_and_bounded(self):
        c = LocalDebugCache(capacity=2)

        def put(key, stamp):
            return c.grid(key, stamp, 160, 250, 160, "mono8", bytes([stamp]) * 40000)

        put("probability", 1)
        put("coverage", 2)
        self.assertFalse(c.snapshot(1)["raster_available"])
        put("coverage", 1)
        self.assertTrue(c.snapshot(1)["raster_available"])
        put("probability", 2)
        put("probability", 3)
        self.assertFalse(c.snapshot(1)["raster_available"])
        put("coverage", 3)
        self.assertEqual(
            base64.b64decode(c.snapshot(3)["probability"]), bytes([3]) * 40000
        )
        self.assertFalse(c.grid("probability", 4, 160, 250, 160, "rgb8", bytes(40000)))
        self.assertFalse(c.snapshot(4)["raster_available"])

    def test_camera_is_not_a_new_ground_projection(self):
        c = LocalDebugCache()
        c.camera(dict(view="front", stamp_ns="2", boxes=[dict(bbox=[1, 2, 3, 4])]))
        snap = c.snapshot(1)
        self.assertFalse(snap["raster_available"])
        self.assertEqual(snap["cameras"][0]["stamp_ns"], "2")
        self.assertIn("not a planner obstacle", snap["camera_alignment"])


if __name__ == "__main__":
    unittest.main()
