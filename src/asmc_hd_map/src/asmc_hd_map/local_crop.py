"""Deterministic map-frame crops for planner-facing HD-map topics.

The helpers in this module do not import ROS.  Local products remain in the
global ``map`` frame: only their spatial extent changes.
"""

from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
import math
from typing import Iterable, Optional, Sequence, Tuple

import numpy as np


Bounds = Tuple[float, float, float, float]


@dataclass(frozen=True)
class OccupancyCrop:
    """A fixed-size, grid-aligned occupancy crop."""

    data: np.ndarray
    origin_x: float
    origin_y: float
    resolution: float

    @property
    def width(self) -> int:
        return int(self.data.shape[1])

    @property
    def height(self) -> int:
        return int(self.data.shape[0])


def geometry_bounds(point_sets: Iterable[Iterable[Iterable[float]]]) -> Optional[Bounds]:
    """Return the finite XY bounds of one or more point sequences."""

    finite_values = []
    for raw in point_sets:
        points = np.asarray(list(raw), dtype=np.float64)
        if points.size == 0:
            continue
        if points.ndim != 2 or points.shape[1] < 2:
            raise ValueError("geometry points must have shape N x >=2")
        xy = points[:, :2]
        finite_values.append(xy[np.isfinite(xy).all(axis=1)])
    finite_values = [value for value in finite_values if len(value)]
    if not finite_values:
        return None
    points = np.concatenate(finite_values, axis=0)
    minimum = np.min(points, axis=0)
    maximum = np.max(points, axis=0)
    return float(minimum[0]), float(minimum[1]), float(maximum[0]), float(maximum[1])


def crop_bounds(center_xy: Sequence[float], extent_m: float) -> Bounds:
    """Return an axis-aligned square centered on ``center_xy``."""

    center = np.asarray(tuple(center_xy), dtype=np.float64)
    extent = float(extent_m)
    if center.shape != (2,) or not np.isfinite(center).all():
        raise ValueError("crop center must contain two finite values")
    if not math.isfinite(extent) or extent <= 0.0:
        raise ValueError("crop extent must be positive and finite")
    half = 0.5 * extent
    return center[0] - half, center[1] - half, center[0] + half, center[1] + half


def bounds_intersect(first: Bounds, second: Bounds) -> bool:
    """Return true when two closed XY bounding boxes intersect."""

    return not (
        first[2] < second[0]
        or first[0] > second[2]
        or first[3] < second[1]
        or first[1] > second[3]
    )


def select_intersecting_bounds(
    item_bounds: Sequence[Optional[Bounds]], center_xy: Sequence[float], extent_m: float
) -> list[int]:
    """Select geometry indices whose bounds touch the requested local square."""

    window = crop_bounds(center_xy, extent_m)
    return [
        index
        for index, bounds in enumerate(item_bounds)
        if bounds is not None and bounds_intersect(bounds, window)
    ]


def crop_occupancy_grid(
    occupancy: np.ndarray,
    resolution: float,
    origin_xy: Sequence[float],
    center_xy: Sequence[float],
    extent_m: float,
    unknown_value: int = -1,
) -> OccupancyCrop:
    """Create a fixed-size, map-aligned crop and pad outside cells as unknown."""

    source = np.asarray(occupancy)
    if source.ndim != 2:
        raise ValueError("occupancy grid must be a two-dimensional array")
    resolution = float(resolution)
    if not math.isfinite(resolution) or resolution <= 0.0:
        raise ValueError("grid resolution must be positive and finite")
    origin = np.asarray(tuple(origin_xy), dtype=np.float64)
    center = np.asarray(tuple(center_xy), dtype=np.float64)
    if origin.shape != (2,) or center.shape != (2,):
        raise ValueError("grid origin and crop center must be XY pairs")
    if not np.isfinite(np.concatenate((origin, center))).all():
        raise ValueError("grid origin and crop center must be finite")
    extent = float(extent_m)
    if not math.isfinite(extent) or extent <= 0.0:
        raise ValueError("crop extent must be positive and finite")

    cells = max(1, int(math.ceil(extent / resolution - 1e-12)))
    lower_world = center - 0.5 * cells * resolution
    lower_cell = np.floor((lower_world - origin) / resolution + 1e-12).astype(np.int64)
    output_origin = origin + lower_cell.astype(np.float64) * resolution
    output = np.full((cells, cells), int(unknown_value), dtype=source.dtype)

    source_x0 = max(0, int(lower_cell[0]))
    source_y0 = max(0, int(lower_cell[1]))
    source_x1 = min(source.shape[1], int(lower_cell[0]) + cells)
    source_y1 = min(source.shape[0], int(lower_cell[1]) + cells)
    if source_x1 > source_x0 and source_y1 > source_y0:
        destination_x0 = source_x0 - int(lower_cell[0])
        destination_y0 = source_y0 - int(lower_cell[1])
        output[
            destination_y0:destination_y0 + (source_y1 - source_y0),
            destination_x0:destination_x0 + (source_x1 - source_x0),
        ] = source[source_y0:source_y1, source_x0:source_x1]

    return OccupancyCrop(
        data=output,
        origin_x=float(output_origin[0]),
        origin_y=float(output_origin[1]),
        resolution=resolution,
    )


@dataclass(frozen=True)
class EgoWindow:
    """Heading-aligned window: x forward, y left, referenced to rear axle."""

    front_m: float = 90.0
    rear_m: float = 30.0
    left_m: float = 30.0
    right_m: float = 30.0

    def __post_init__(self):
        if not all(math.isfinite(v) and v > 0 for v in (
            self.front_m, self.rear_m, self.left_m, self.right_m
        )):
            raise ValueError("window distances must be positive and finite")

    def corners(self, xy, yaw):
        xy = np.asarray(xy, dtype=float)
        if xy.shape != (2,) or not np.isfinite(xy).all() or not math.isfinite(yaw):
            raise ValueError("ego pose must be finite XY and yaw")
        c, s = math.cos(yaw), math.sin(yaw)
        rotation = np.array([[c, -s], [s, c]])
        local = np.array([
            [-self.rear_m, -self.right_m], [self.front_m, -self.right_m],
            [self.front_m, self.left_m], [-self.rear_m, self.left_m],
        ])
        return local @ rotation.T + xy


def select_ego_bounds(item_bounds, xy, yaw, window):
    """Conservative AABB/rotated-window intersection using separating axes.

    Geometry and IDs remain intact; bounding boxes may select extra features.
    """
    corners = window.corners(xy, yaw)
    c, s = math.cos(yaw), math.sin(yaw)
    axes = np.array([[1., 0.], [0., 1.], [c, s], [-s, c]])
    projected = corners @ axes.T
    low, high = projected.min(axis=0), projected.max(axis=0)
    indices = [i for i, bounds in enumerate(item_bounds) if bounds is not None]
    if not indices:
        return []
    boxes = np.asarray([item_bounds[i] for i in indices], dtype=float)
    centers = .5 * (boxes[:, :2] + boxes[:, 2:])
    half_sizes = .5 * (boxes[:, 2:] - boxes[:, :2])
    box_centers = centers @ axes.T
    radii = half_sizes @ np.abs(axes.T)
    intersects = np.all(box_centers + radii >= low - 1e-9, axis=1)
    intersects &= np.all(box_centers - radii <= high + 1e-9, axis=1)
    return [indices[i] for i in np.flatnonzero(intersects)]


def crop_ego_occupancy_grid(occupancy, resolution, origin_xy, ego_xy, yaw, window):
    """Nearest source-cell samples at rotated output cell centers (no smoothing).

    Output +x is ego forward and +y left. Its origin is in map; the ROS grid
    origin quaternion must contain yaw. Unknown source/outside cells stay -1.
    """
    source = np.asarray(occupancy)
    resolution = float(resolution)
    if source.ndim != 2 or not math.isfinite(resolution) or resolution <= 0:
        raise ValueError("grid must be 2D with positive finite resolution")
    origin = np.asarray(origin_xy, dtype=float)
    if origin.shape != (2,) or not np.isfinite(origin).all():
        raise ValueError("grid origin must be finite XY")
    corner = window.corners(ego_xy, yaw)[0]
    width = int(math.ceil((window.front_m + window.rear_m) / resolution - 1e-12))
    height = int(math.ceil((window.left_m + window.right_m) / resolution - 1e-12))
    # Cache the cell-center lattice; only the pose-dependent transform changes.
    local_x, local_y = _cell_centers(width, height, resolution)
    c, s = math.cos(yaw), math.sin(yaw)
    xs = np.floor((corner[0] + c * local_x - s * local_y - origin[0]) / resolution).astype(np.int64)
    ys = np.floor((corner[1] + s * local_x + c * local_y - origin[1]) / resolution).astype(np.int64)
    valid = (xs >= 0) & (ys >= 0) & (xs < source.shape[1]) & (ys < source.shape[0])
    output = np.full((height, width), -1, dtype=np.int8)
    output[valid] = source[ys[valid], xs[valid]]
    return OccupancyCrop(output, float(corner[0]), float(corner[1]), resolution)


@lru_cache(maxsize=8)
def _cell_centers(width, height, resolution):
    return np.meshgrid((np.arange(width) + .5) * resolution,
                       (np.arange(height) + .5) * resolution)
