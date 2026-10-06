"""Lossless free-cell RLE used by the current Frenet AvoidRoadMap reader."""
import numpy as np


def write_free_grid_rle(stream, occupancy, grid):
    cells = np.asarray(occupancy)
    if cells.shape != (grid['height'], grid['width']):
        raise ValueError('grid metadata and occupancy shape differ')
    stream.write('%d %d %.17g %.17g %.17g\n' % (
        grid['width'], grid['height'], grid['resolution_m'], grid['origin_x'], grid['origin_y']))
    for row, cells_row in enumerate(cells):
        free = np.pad(cells_row == 0, (1, 1), constant_values=False)
        edges = np.flatnonzero(free[1:] != free[:-1])
        for start, end in edges.reshape(-1, 2):
            stream.write('%d %d %d\n' % (row, start, end))
