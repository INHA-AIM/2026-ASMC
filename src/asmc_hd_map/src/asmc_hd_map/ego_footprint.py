"""Ego vehicle footprint around base_link and its overlap with the drivable grid.

ROS-free helpers for the HD-map debug view. base_link is the rear-axle center
used for current-lane association; the footprint extends rear_overhang behind
it and wheelbase + front_overhang ahead of it.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Dict, List, Sequence, Tuple

import numpy as np


@dataclass(frozen=True)
class VehicleFootprint:
    rear_overhang_m: float = 0.79
    wheelbase_m: float = 3.000
    front_overhang_m: float = 0.845
    width_m: float = 1.892

    def __post_init__(self) -> None:
        for name in ("rear_overhang_m", "wheelbase_m", "front_overhang_m", "width_m"):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value < 0.0:
                raise ValueError("%s must be finite and non-negative" % name)
        if self.length_m <= 0.0 or self.width_m <= 0.0:
            raise ValueError("vehicle footprint must have a positive area")

    @property
    def length_m(self) -> float:
        return self.rear_overhang_m + self.wheelbase_m + self.front_overhang_m

    @property
    def center_offset_m(self) -> float:
        """Longitudinal offset from base_link to the footprint center."""

        return 0.5 * (self.wheelbase_m + self.front_overhang_m - self.rear_overhang_m)


# 2023 Hyundai Ioniq 5, competition rule book 2-6-1.
IONIQ5_FOOTPRINT = VehicleFootprint()


def _to_map(x_m: float, y_m: float, yaw_rad: float, local: np.ndarray) -> np.ndarray:
    cos_yaw, sin_yaw = math.cos(yaw_rad), math.sin(yaw_rad)
    rotation = np.asarray([[cos_yaw, -sin_yaw], [sin_yaw, cos_yaw]])
    return local @ rotation.T + np.asarray([x_m, y_m])


def footprint_corners(
    x_m: float, y_m: float, yaw_rad: float, footprint: VehicleFootprint = IONIQ5_FOOTPRINT
) -> List[Tuple[float, float]]:
    """Return rear-left, front-left, front-right, rear-right corners in the map frame."""

    rear = -footprint.rear_overhang_m
    front = footprint.wheelbase_m + footprint.front_overhang_m
    half = 0.5 * footprint.width_m
    local = np.asarray([[rear, half], [front, half], [front, -half], [rear, -half]])
    return [tuple(map(float, point)) for point in _to_map(x_m, y_m, yaw_rad, local)]


def footprint_samples(
    x_m: float,
    y_m: float,
    yaw_rad: float,
    footprint: VehicleFootprint = IONIQ5_FOOTPRINT,
    step_m: float = 0.2,
) -> np.ndarray:
    """Return an evenly spaced grid of map-frame points covering the footprint."""

    rear = -footprint.rear_overhang_m
    front = footprint.wheelbase_m + footprint.front_overhang_m
    half = 0.5 * footprint.width_m
    longitudinal = np.linspace(rear, front, max(2, int(math.ceil((front - rear) / step_m)) + 1))
    lateral = np.linspace(-half, half, max(2, int(math.ceil(2.0 * half / step_m)) + 1))
    grid = np.stack(np.meshgrid(longitudinal, lateral, indexing="ij"), axis=-1).reshape(-1, 2)
    return _to_map(x_m, y_m, yaw_rad, grid)


def occupancy_values(
    occupancy: np.ndarray,
    resolution_m: float,
    origin_xy: Sequence[float],
    points_xy: np.ndarray,
) -> np.ndarray:
    """Look up OccupancyGrid values (row = y, column = x); -1 outside the grid.

    Uses the same nearest-cell rounding as the compact builder and its tests.
    """

    points = np.asarray(points_xy, dtype=np.float64).reshape(-1, 2)
    columns = np.rint((points[:, 0] - float(origin_xy[0])) / resolution_m).astype(int)
    rows = np.rint((points[:, 1] - float(origin_xy[1])) / resolution_m).astype(int)
    inside = (rows >= 0) & (rows < occupancy.shape[0]) & (columns >= 0) & (columns < occupancy.shape[1])
    values = np.full(len(points), -1, dtype=np.int16)
    values[inside] = occupancy[rows[inside], columns[inside]]
    return values


def footprint_drivable_summary(
    x_m: float,
    y_m: float,
    yaw_rad: float,
    occupancy: np.ndarray,
    resolution_m: float,
    origin_xy: Sequence[float],
    footprint: VehicleFootprint = IONIQ5_FOOTPRINT,
) -> Dict[str, object]:
    """Fraction of the footprint outside drivable cells (value != 0) and per-corner flags."""

    samples = footprint_samples(x_m, y_m, yaw_rad, footprint, step_m=min(0.2, resolution_m))
    values = occupancy_values(occupancy, resolution_m, origin_xy, samples)
    corners = footprint_corners(x_m, y_m, yaw_rad, footprint)
    corner_values = occupancy_values(occupancy, resolution_m, origin_xy, np.asarray(corners))
    return {
        "sample_count": int(len(values)),
        "outside_fraction": float(np.mean(values != 0)),
        "corner_drivable": [bool(value == 0) for value in corner_values],
    }
