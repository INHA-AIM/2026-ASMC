#!/usr/bin/env python3
"""Small geometry and candidate-policy invariants for the isolated replay."""
import unittest

import numpy as np

from evaluate import clusters, guarded, l_shape, voxelize


class CandidateTest(unittest.TestCase):
    def test_support_requires_lower_ground(self):
        cloud = np.asarray([[5., 0., -.9, 1.], [5.1, 0., -1.8, 2.],
                            [5.2, 0., -1.8, 3.], [5.3, 0., -1.8, 4.]], dtype=np.float32)
        pp = np.asarray([1, 1, 1, 1], dtype=np.uint8)
        simple, supported, high, count = guarded(cloud, 0., pp)
        self.assertEqual((high, count), (1, 1))
        self.assertEqual(simple.tolist(), [2, 1, 1, 1])
        self.assertEqual(supported.tolist(), [2, 1, 1, 1])

    def test_unobserved_floor_does_not_invent_support(self):
        cloud = np.asarray([[5., 0., -.9, 1.]], dtype=np.float32)
        simple, supported, high, count = guarded(cloud, 0., np.asarray([1], dtype=np.uint8))
        self.assertEqual((high, count), (1, 0))
        self.assertEqual(simple.tolist(), [2])
        self.assertEqual(supported.tolist(), [1])

    def test_voxel_lineage_and_euclidean_threshold(self):
        points = np.asarray([[1. + .01*i, 0., 0., i] for i in range(10)] +
                            [[4. + .01*i, 0., 0., i] for i in range(10)], dtype=np.float32)
        selected, inverse, centroids = voxelize(points, np.ones(len(points), dtype=bool))
        self.assertEqual(len(selected), 20)
        self.assertEqual(len(inverse), 20)
        self.assertEqual(len(centroids), 2)
        # With only two voxels the production minimum cluster size is not met.
        _, groups = clusters(centroids, np.ones(2, dtype=bool), 0.)
        self.assertFalse(groups)

    def test_lshape_finite(self):
        xyz = np.asarray([[x, y, z] for x in (0., 1., 2., 3.)
                          for y in (0., .5, 1.) for z in (0., 1.)])
        box = l_shape(xyz)
        self.assertTrue(all(np.isfinite(v) for v in box.values()))
        self.assertGreaterEqual(box["length_m"], box["width_m"])
        self.assertEqual(box["height_m"], 1.)


if __name__ == "__main__":
    unittest.main()
