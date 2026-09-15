"""Robust signed-offset estimation for map-derived lane boundaries."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping

import numpy as np
from scipy.spatial import cKDTree

from .shadow_map_gt import resample_polyline


BOUNDARY_ORDER = (
    "white_negative_normal",
    "yellow_center",
    "white_positive_normal",
)


@dataclass(frozen=True)
class BoundaryResidualEstimate:
    offset_m: float
    median_m: float
    mad_m: float
    support: int
    total_associated: int
    peak_fraction: float

    def to_mapping(self):
        return {
            "offset_m": float(self.offset_m),
            "median_m": float(self.median_m),
            "mad_m": float(self.mad_m),
            "support": int(self.support),
            "total_associated": int(self.total_associated),
            "peak_fraction": float(self.peak_fraction),
        }


class BoundaryResidualProjector:
    """Associate map points to the closest boundary and retain signed normal error."""

    def __init__(self, boundaries: Mapping[str, np.ndarray], spacing_m=0.05):
        if set(boundaries) != set(BOUNDARY_ORDER):
            raise ValueError("exactly the three shadow boundaries are required")
        if float(spacing_m) <= 0.0:
            raise ValueError("sampling spacing must be positive")
        raw = {name: np.asarray(boundaries[name], dtype=np.float64) for name in BOUNDARY_ORDER}
        count = max(
            2,
            int(
                np.ceil(
                    max(
                        np.sum(np.linalg.norm(np.diff(value[:, :2], axis=0), axis=1))
                        for value in raw.values()
                    )
                    / float(spacing_m)
                )
            )
            + 1,
        )
        sampled = {
            name: resample_polyline(raw[name], count)[:, :2] for name in BOUNDARY_ORDER
        }
        family_normal = sampled["white_positive_normal"] - sampled["white_negative_normal"]
        norm = np.linalg.norm(family_normal, axis=1)
        if np.any(norm <= 1e-9):
            raise ValueError("lane-family normal is degenerate")
        family_normal /= norm[:, None]
        self.samples = sampled
        self.normals = family_normal
        self.trees = {name: cKDTree(sampled[name]) for name in BOUNDARY_ORDER}

    def residuals(self, points_xy, max_distance_m=1.0):
        points = np.asarray(points_xy, dtype=np.float64).reshape(-1, 2)
        if not np.isfinite(points).all():
            raise ValueError("calibration points must be finite")
        if float(max_distance_m) <= 0.0:
            raise ValueError("association distance must be positive")
        if not len(points):
            return {name: np.empty(0, dtype=np.float64) for name in BOUNDARY_ORDER}
        distances = []
        indices = []
        for name in BOUNDARY_ORDER:
            distance, index = self.trees[name].query(points, k=1)
            distances.append(distance)
            indices.append(index)
        distance_stack = np.stack(distances, axis=1)
        index_stack = np.stack(indices, axis=1)
        owner = np.argmin(distance_stack, axis=1)
        result = {}
        for owner_index, name in enumerate(BOUNDARY_ORDER):
            selected = (owner == owner_index) & (
                distance_stack[:, owner_index] <= float(max_distance_m)
            )
            nearest = index_stack[selected, owner_index]
            delta = points[selected] - self.samples[name][nearest]
            result[name] = np.sum(delta * self.normals[nearest], axis=1)
        return result


def estimate_residual_peak(
    residuals,
    max_abs_m=0.9,
    bin_width_m=0.02,
    local_half_width_m=0.12,
) -> BoundaryResidualEstimate:
    values = np.asarray(residuals, dtype=np.float64).reshape(-1)
    values = values[np.isfinite(values) & (np.abs(values) <= float(max_abs_m))]
    if len(values) < 10:
        raise ValueError("too few boundary returns for a robust offset estimate")
    edges = np.arange(
        -float(max_abs_m),
        float(max_abs_m) + float(bin_width_m) * 1.01,
        float(bin_width_m),
    )
    histogram, edges = np.histogram(values, bins=edges)
    smoothed = np.convolve(histogram, np.asarray([1, 2, 3, 2, 1]), mode="same")
    peak_index = int(np.argmax(smoothed))
    peak = 0.5 * (edges[peak_index] + edges[peak_index + 1])
    local = values[np.abs(values - peak) <= float(local_half_width_m)]
    if len(local) < 10:
        raise ValueError("boundary residual histogram has no supported local peak")
    median = float(np.median(local))
    mad = float(np.median(np.abs(local - median)))
    return BoundaryResidualEstimate(
        offset_m=median,
        median_m=float(np.median(values)),
        mad_m=mad,
        support=int(len(local)),
        total_associated=int(len(values)),
        peak_fraction=float(len(local) / len(values)),
    )


def calibrated_lane_geometry(
    nominal_width_m: float, estimates: Mapping[str, BoundaryResidualEstimate]
):
    if set(estimates) != set(BOUNDARY_ORDER):
        raise ValueError("an estimate is required for every boundary")
    center_offset = float(estimates["yellow_center"].offset_m)
    positive_width = (
        float(nominal_width_m)
        + float(estimates["white_positive_normal"].offset_m)
        - center_offset
    )
    negative_width = (
        float(nominal_width_m)
        + center_offset
        - float(estimates["white_negative_normal"].offset_m)
    )
    if min(positive_width, negative_width) <= 0.0:
        raise ValueError("calibrated lane widths must be positive")
    return {
        "center_lateral_offset_m": center_offset,
        "positive_lane_width_m": positive_width,
        "negative_lane_width_m": negative_width,
    }
