"""Official-route station projection, independent of ROS and planner smoothing."""
from dataclasses import dataclass
import math
import numpy as np


@dataclass(frozen=True)
class RouteProjection:
    station_m: float
    distance_m: float
    point: tuple


class RouteReference:
    def __init__(self, points, stations):
        self.points = np.asarray(points, dtype=float)
        self.stations = np.asarray(stations, dtype=float)
        if (self.points.ndim != 2 or self.points.shape[1] != 3 or len(self.points) < 2
                or self.stations.shape != (len(self.points),)
                or not np.isfinite(self.points).all() or not np.isfinite(self.stations).all()):
            raise ValueError("route needs finite Nx3 points and N stations")
        self.segments = np.diff(self.points[:, :2], axis=0)
        self.lengths = np.linalg.norm(self.segments, axis=1)
        if (abs(self.stations[0]) > 1e-8 or self.stations[-1] <= 0
                or not np.allclose(np.diff(self.stations), self.lengths, atol=1e-6, rtol=0)):
            raise ValueError("stations must equal official XY arc length from point zero")
        self.length_m = float(self.stations[-1])
        self.nonzero = self.lengths > 1e-9

    def point_at(self, station_m):
        station = self._station(station_m)
        index = min(len(self.points) - 2, int(np.searchsorted(self.stations, station, side="right") - 1))
        width = self.stations[index + 1] - self.stations[index]
        ratio = (station - self.stations[index]) / width if width > 0 else 0.
        return tuple(self.points[index] + ratio * (self.points[index + 1] - self.points[index]))

    def _station(self, station):
        if not math.isfinite(station) or not 0 <= station <= self.length_m:
            raise ValueError("station must be within the published official route")
        return float(station)

    def project(self, xy, hint_m=None, search_m=40., max_distance_m=15.):
        xy = np.asarray(xy, dtype=float)
        if xy.shape != (2,) or not np.isfinite(xy).all():
            raise ValueError("projection requires finite map XY")
        if not math.isfinite(search_m) or search_m <= 0 or not math.isfinite(max_distance_m) or max_distance_m <= 0:
            raise ValueError("projection distances must be positive and finite")
        delta = xy - self.points[:-1, :2]
        ratio = np.clip(np.sum(delta * self.segments, axis=1) /
                        np.maximum(self.lengths ** 2, 1e-18), 0., 1.)
        projected = self.points[:-1, :2] + ratio[:, None] * self.segments
        distances = np.linalg.norm(projected - xy, axis=1)
        distances[~self.nonzero] = np.inf
        stations = self.stations[:-1] + ratio * self.lengths
        if hint_m is not None:
            self._station(hint_m)
            delta_s = np.abs(stations - hint_m)
            # The published competition route is closed; preserve local progress
            # near its seam without treating remote parallel roads as nearby.
            if np.linalg.norm(self.points[0, :2] - self.points[-1, :2]) < 1e-6:
                delta_s = np.minimum(delta_s, self.length_m - delta_s)
            nearby = np.where(delta_s <= search_m, distances, np.inf)
            if np.min(nearby) <= max_distance_m:
                distances = nearby
        index = int(np.argmin(distances))
        if distances[index] > max_distance_m:
            return None
        point = self.points[index] + ratio[index] * (self.points[index + 1] - self.points[index])
        return RouteProjection(float(stations[index]), float(distances[index]), tuple(point))
