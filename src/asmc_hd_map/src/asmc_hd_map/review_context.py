"""Pure geometry lookup used by the 1 Hz HD-map driving-review logger."""

from __future__ import annotations

import math

import numpy as np

from .geometry import cumulative_distance, points_xyz
from .observed import PolylineProjectionIndex


def _width_at(profile: list[dict], station_m: float) -> float | None:
    if not profile:
        return None
    stations = np.asarray([float(row["station_m"]) for row in profile], dtype=np.float64)
    widths = np.asarray([float(row["width_m"]) for row in profile], dtype=np.float64)
    return float(np.interp(station_m, stations, widths))


class LaneReviewIndex:
    """Nearest-lane context without ROS or a runtime-planning dependency."""

    def __init__(self, document: dict):
        self.boundaries = {
            feature["id"]: feature for feature in document.get("features", [])
            if feature.get("feature_type") == "lane_boundary"
        }
        self.areas = {
            str(feature["attributes"]["lane_id"]): feature
            for feature in document.get("features", []) if feature.get("feature_type") == "lane_area"
        }
        polylines = {
            lane_id: feature["attributes"]["centerline"]
            for lane_id, feature in self.areas.items()
        }
        self.index = PolylineProjectionIndex(polylines, cell_size_m=4.0)
        self.stoplines = [
            feature for feature in document.get("features", [])
            if feature.get("feature_type") == "stopline"
            and feature.get("attributes", {}).get("route_s_m") is not None
        ]

    def query(self, x_m: float, y_m: float, heading_deg: float) -> dict | None:
        heading = math.radians(float(heading_deg))
        tangent = np.asarray([math.cos(heading), math.sin(heading)], dtype=np.float64)
        candidates = self.index.query(
            [x_m, y_m, 0.0], tangent, max_distance_m=20.0, max_heading_deg=75.0,
        )
        if not candidates:
            return None
        projection = candidates[0]
        lane_id = projection.owner_id
        area = self.areas[lane_id]
        attributes = area["attributes"]
        route_min = attributes.get("route_s_min_m")
        route_max = attributes.get("route_s_max_m")
        route_s = None
        if route_min is not None and route_max is not None:
            centerline = points_xyz(attributes["centerline"])
            total = float(cumulative_distance(centerline)[-1]) if len(centerline) else 0.0
            ratio = np.clip(projection.station_m / max(total, 1e-9), 0.0, 1.0)
            route_s = float(route_min) + ratio * (float(route_max) - float(route_min))
        next_stopline = None
        if route_s is not None:
            ahead = [
                (float(feature["attributes"]["route_s_m"]) - route_s, feature)
                for feature in self.stoplines
                if float(feature["attributes"]["route_s_m"]) >= route_s
            ]
            if ahead:
                distance, feature = min(ahead, key=lambda value: value[0])
                next_stopline = {
                    "node_id": feature["attributes"]["node_ids"][0],
                    "route_distance_m": distance,
                }

        def boundary_summary(boundary_id: str | None):
            if boundary_id is None or boundary_id not in self.boundaries:
                return None
            boundary = self.boundaries[boundary_id]
            values = boundary["attributes"]
            return {
                "id": boundary_id,
                "color": values.get("lane_color", "unknown"),
                "pattern": values.get("pattern", "unknown"),
                "crossing_rule": values.get("crossing_rule", "unknown"),
                "observed_fraction": values.get("observed_fraction", 0.0),
            }

        return {
            "map_x_m": float(x_m),
            "map_y_m": float(y_m),
            "heading_deg": float(heading_deg),
            "lane_id": lane_id,
            "lane_station_m": float(projection.station_m),
            "route_s_m": route_s,
            "lateral_offset_m": float(projection.signed_offset_m),
            "lane_width_m": _width_at(attributes.get("width_profile", []), projection.station_m),
            "geometry_quality": attributes.get("geometry_quality"),
            "roles": list(attributes.get("route_maneuver_roles", [])),
            "left_boundary": boundary_summary(attributes.get("left_boundary_id")),
            "right_boundary": boundary_summary(attributes.get("right_boundary_id")),
            "next_stopline": next_stopline,
        }
