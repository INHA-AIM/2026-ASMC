import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from export_signal_trial_surface import surface


class FullRouteSurfaceTest(unittest.TestCase):
    def meshes(self, include_far=True):
        vertices, indices = [], []
        for x in ([0., 900.] if include_far else [0.]):
            begin = len(vertices)
            vertices.extend([[x-1, -1, 0], [x+1, -1, 0], [x+1, 1, 0], [x-1, 1, 0]])
            indices.extend([[begin, begin+1, begin+2], [begin, begin+2, begin+3]])
        return [('reviewed', np.array(vertices), np.array(indices))]

    def test_full_route_requires_coverage_after_old_860m_limit(self):
        path = np.array([[0., 0, 0], [900., 0, 0]])
        old = surface(self.meshes(False), path)
        self.assertEqual(old['route_point_count'], 1)
        with self.assertRaisesRegex(ValueError, 'uncovered'):
            surface(self.meshes(False), path, end_station_m=None)
        full = surface(self.meshes(), path, end_station_m=None)
        self.assertEqual(full['route_point_count'], 2)
        self.assertEqual(full['route_points_covered'], 2)
        self.assertIsNone(full['route_end_station_m'])

    def test_invalid_endpoint_cannot_silently_shrink_evaluation_scope(self):
        path = np.array([[0., 0, 0], [900., 0, 0]])
        for endpoint in (0, -1, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                surface(self.meshes(), path, end_station_m=endpoint)


if __name__ == '__main__':
    unittest.main()
