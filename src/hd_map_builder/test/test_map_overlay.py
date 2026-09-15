import unittest

import numpy as np

from hd_map_builder.map_overlay import (
    SegmentLayer,
    grouped_lane_layers,
    lane_color_group,
)


class MapOverlayTest(unittest.TestCase):
    def test_origin_translation_and_base_link_transform(self):
        layer = SegmentLayer.from_features(
            [{"points": [[0.0, 0.0, 0.0], [2.0, 0.0, 0.0]]}],
            translation_xy=(10.0, 20.0),
        )
        local = layer.around_ego((10.0, 20.0), heading_deg=90.0, radius_m=5.0)
        self.assertTrue(
            np.allclose(local, [[[0.0, 0.0], [0.0, -2.0]]], atol=1e-9)
        )

    def test_local_radius_keeps_crossing_segment(self):
        layer = SegmentLayer(
            np.asarray([[[-10.0, 0.0], [10.0, 0.0]]], dtype=np.float64)
        )
        local = layer.around_ego((0.0, 0.0), heading_deg=0.0, radius_m=1.0)
        self.assertEqual(local.shape, (1, 2, 2))

    def test_lane_colors_are_grouped_without_promoting_unknown(self):
        features = [
            {"lane_color": ["White"], "points": [[0, 0], [1, 0]]},
            {"lane_color": ["Yellow"], "points": [[0, 1], [1, 1]]},
            {"lane_color": ["Purple"], "points": [[0, 2], [1, 2]]},
        ]
        grouped = grouped_lane_layers(features, (0.0, 0.0))
        self.assertEqual(len(grouped["white"].segments_xy), 1)
        self.assertEqual(len(grouped["yellow"].segments_xy), 1)
        self.assertEqual(len(grouped["unknown"].segments_xy), 1)
        self.assertEqual(lane_color_group(features[2]), "unknown")


if __name__ == "__main__":
    unittest.main()
