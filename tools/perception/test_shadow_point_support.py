#!/usr/bin/env python3
"""Point lineage and time-matching regressions for the offline experiment."""
import unittest
import numpy as np
from benchmark_shadow_point_support import box_indices, nearest_index, split_support


class SupportTest(unittest.TestCase):
    def test_time_boundary_tie_and_unavailable(self):
        self.assertIsNone(nearest_index([], 100))
        self.assertEqual(nearest_index([100, 200], 150, 50), 0)
        self.assertIsNone(nearest_index([100, 200], 150, 49))

    def test_background_component_is_returned_not_deleted(self):
        points = np.array([[3,0,0],[3,.1,0],[3,.2,0],[8,0,0],[8,.1,0],[8,.2,0]], float)
        pixels = np.tile([50.,50.], (6,1))
        depth = points[:,0]
        components, chosen = split_support(points, pixels, depth, np.arange(6), [0,0,100,100])
        self.assertEqual(len(components), 2)
        np.testing.assert_array_equal(chosen, [0,1,2])
        np.testing.assert_array_equal(np.sort(np.concatenate(components)), np.arange(6))

    def test_sparse_support_is_unknown_and_index_not_reordered(self):
        pixels = np.array([[5,5],[99,99],[10,10],[100,50]], float)
        selected = box_indices(pixels, np.array([True,False,True,True]), [0,0,100,100])
        np.testing.assert_array_equal(selected, [0,2])
        components, chosen = split_support(np.zeros((4,3)), pixels, np.ones(4), selected, [0,0,100,100])
        self.assertEqual(len(chosen), 0)
        np.testing.assert_array_equal(components[0], [0,2])


if __name__ == '__main__':
    unittest.main()
