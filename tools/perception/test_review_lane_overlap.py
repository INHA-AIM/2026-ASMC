"""Controlled overlap checks; candidate never changes production defaults."""
import unittest
import numpy as np
from asmc_camera.bev.model_grid_projection import MetricEvidence
from asmc_camera.bev.lane_evidence import reduce_evidence_to_nodes
from tools.perception.review_lane_overlap import balanced_nodes


def evidence(xy, sources, classes=None):
    xy = np.asarray(xy, dtype=np.float32); count = len(xy)
    return MetricEvidence(xy, np.asarray(classes or [1] * count, dtype=np.uint8),
                          np.full(count, .8, dtype=np.float32), np.ones(count, dtype=np.float32),
                          np.asarray(sources, dtype=np.uint8))


class OverlapTest(unittest.TestCase):
    def test_single_camera_matches_existing_reducer(self):
        data = evidence([[1, 2], [1.1, 2.1], [2, 2.2]], [1, 1, 1])
        actual, overlap = balanced_nodes(data)
        expected = reduce_evidence_to_nodes(data)
        self.assertEqual(overlap, [])
        for a, b in zip(actual, expected):
            np.testing.assert_allclose(a.xy, b.xy, atol=1e-12)
            self.assertEqual(a.source_mask, b.source_mask)
            self.assertEqual(a.support, b.support)

    def test_duplicate_pixels_do_not_give_one_camera_more_leverage(self):
        data = evidence([[1, 2]] * 100 + [[1, 2.1]], [1] * 100 + [2])
        actual, overlap = balanced_nodes(data)
        self.assertEqual(len(actual), 1)
        self.assertAlmostEqual(actual[0].xy[1], 2.05, places=6)
        self.assertEqual(actual[0].source_mask, 3)
        self.assertEqual(actual[0].support, 100)
        self.assertAlmostEqual(overlap[0]['dominant_pixel_fraction'], 100 / 101)

    def test_adjacent_lines_and_different_classes_stay_separate(self):
        data = evidence([[1, 2], [1, 2.8], [1, 2]], [1, 2, 4], [1, 1, 2])
        actual, overlap = balanced_nodes(data)
        self.assertEqual(len(actual), 3)
        self.assertEqual(overlap, [])


if __name__ == '__main__': unittest.main()
