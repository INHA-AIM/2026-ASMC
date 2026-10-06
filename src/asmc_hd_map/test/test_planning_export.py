from io import StringIO
import unittest
import numpy as np
from asmc_hd_map.planning_export import write_free_grid_rle


class ExportTest(unittest.TestCase):
    def test_free_cells_roundtrip_including_edges_and_unknown(self):
        occupancy = np.array([[0, 0, 100, -1, 0], [100] * 5, [0] * 5], dtype=np.int8)
        stream = StringIO()
        write_free_grid_rle(stream, occupancy, dict(width=5, height=3, resolution_m=.2, origin_x=-1.3, origin_y=2.4))
        lines = stream.getvalue().splitlines()
        self.assertEqual(list(map(float, lines[0].split())), [5, 3, .2, -1.3, 2.4])
        restored = np.zeros_like(occupancy, dtype=bool)
        for line in lines[1:]:
            row, start, end = map(int, line.split())
            restored[row, start:end] = True
        np.testing.assert_array_equal(restored, occupancy == 0)
