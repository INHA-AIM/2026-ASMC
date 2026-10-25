"""Pure geometry used by the review-only legacy-map RViz overlay."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable

import numpy as np


def feature_paint_segments(feature):
    """Physical paint only; shared by RViz and the LiDAR web overlay."""
    if feature.get('feature_type') not in ('lane_marking','lane_boundary','stopline'):
        return np.zeros((0,2,2),dtype=np.float64)
    points=np.asarray((feature.get('geometry') or {}).get('coordinates',[]),dtype=np.float64)
    if points.ndim!=2 or len(points)<2 or not np.isfinite(points).all():
        return np.zeros((0,2,2),dtype=np.float64)
    a=feature.get('attributes') or {};m=a.get('mgeo') or {}
    if lane_shape_group(feature)=='broken':
        return dashed_polyline_segments([points],float(a.get('dash_length_m',m.get('dash_interval_L1',3))),
            float(a.get('dash_gap_m',m.get('dash_interval_L2',3))),float(a.get('dash_phase_m',0)))
    return np.stack([points[:-1,:2],points[1:,:2]],axis=1)


def lane_shape_group(feature: dict) -> str:
    """Normalize canonical or preserved-MGeo lane shape for visualization."""

    attributes = feature.get("attributes") or {}
    raw = attributes.get("lane_shape")
    if raw is None:
        raw = attributes.get("pattern")
    if raw is None:
        raw = (attributes.get("mgeo") or {}).get("lane_shape")
    values = raw if isinstance(raw, (list, tuple, set)) else [raw]
    lowered = {str(value).strip().lower() for value in values if value is not None}
    return "broken" if lowered.intersection({"broken", "dashed", "dash"}) else "solid"


def dashed_polyline_segments(
    polylines: Iterable[Iterable[Iterable[float]]],
    dash_length_m: float,
    gap_length_m: float,
    phase_m: float = 0.0,
) -> np.ndarray:
    """Convert continuous metric polylines to exact dashed line segments."""

    dash = float(dash_length_m)
    gap = float(gap_length_m)
    phase = float(phase_m)
    if not np.isfinite([dash, gap, phase]).all() or dash <= 0.0 or gap < 0.0:
        raise ValueError("dash length must be positive and gap length non-negative")
    cycle = dash + gap
    phase %= cycle
    output: list[np.ndarray] = []
    for raw in polylines:
        points = np.asarray(raw, dtype=np.float64)
        if points.ndim != 2 or points.shape[1] < 2 or len(points) < 2:
            continue
        xy = points[:, :2]
        lengths = np.linalg.norm(np.diff(xy, axis=0), axis=1)
        cumulative = np.concatenate(([0.0], np.cumsum(lengths)))
        total = float(cumulative[-1])
        if total <= np.finfo(np.float64).eps:
            continue

        def interpolate(station: float) -> np.ndarray:
            if station >= total:
                return xy[-1].copy()
            index = min(int(np.searchsorted(cumulative, station, side="right") - 1), len(lengths) - 1)
            if lengths[index] <= np.finfo(np.float64).eps:
                return xy[index].copy()
            ratio = (station - cumulative[index]) / lengths[index]
            return xy[index] + ratio * (xy[index + 1] - xy[index])

        start = -phase
        while start + dash <= 0.0:
            start += cycle
        while start < total - np.finfo(np.float64).eps:
            visible_start = max(0.0, start)
            end = min(start + dash, total)
            if end <= visible_start + np.finfo(np.float64).eps:
                start += cycle
                continue
            cuts = [visible_start]
            cuts.extend(
                float(value) for value in cumulative[1:-1]
                if visible_start < value < end
            )
            cuts.append(end)
            vertices = [interpolate(value) for value in cuts]
            output.extend(
                np.stack((first, second))
                for first, second in zip(vertices[:-1], vertices[1:])
                if np.linalg.norm(second - first) > np.finfo(np.float64).eps
            )
            start += cycle
    if not output:
        return np.zeros((0, 2, 2), dtype=np.float64)
    return np.asarray(output, dtype=np.float64)


@dataclass(frozen=True)
class SegmentLayer:
    """A collection of independent XY line segments in the current map frame."""

    segments_xy: np.ndarray
    _midpoints_xy: np.ndarray = field(init=False, repr=False)
    _half_lengths_m: np.ndarray = field(init=False, repr=False)

    def __post_init__(self) -> None:
        value = np.asarray(self.segments_xy, dtype=np.float64)
        if value.ndim != 3 or value.shape[1:] != (2, 2):
            raise ValueError("segments_xy must have shape N x 2 x 2")
        if not np.isfinite(value).all():
            raise ValueError("segments_xy contains non-finite coordinates")
        object.__setattr__(self, "segments_xy", np.ascontiguousarray(value))
        if len(value) == 0:
            object.__setattr__(self, "_midpoints_xy", np.zeros((0, 2)))
            object.__setattr__(self, "_half_lengths_m", np.zeros(0))
        else:
            object.__setattr__(self, "_midpoints_xy", np.mean(value, axis=1))
            object.__setattr__(
                self,
                "_half_lengths_m",
                0.5 * np.linalg.norm(value[:, 1] - value[:, 0], axis=1),
            )

    def __len__(self) -> int:
        """Return the number of independent line segments in this layer."""

        return len(self.segments_xy)

    @classmethod
    def from_features(
        cls,
        features: Iterable[dict],
        translation_xy: Iterable[float] = (0.0, 0.0),
    ) -> "SegmentLayer":
        translation = np.asarray(tuple(translation_xy), dtype=np.float64)
        if translation.shape != (2,) or not np.isfinite(translation).all():
            raise ValueError("translation_xy must contain two finite values")
        segments = []
        for feature in features:
            points = np.asarray(feature.get("points", ()), dtype=np.float64)
            if points.ndim != 2 or points.shape[1] < 2 or len(points) < 2:
                continue
            xy = points[:, :2] + translation
            finite = np.isfinite(xy).all(axis=1)
            valid = finite[:-1] & finite[1:]
            if np.any(valid):
                segments.append(np.stack((xy[:-1][valid], xy[1:][valid]), axis=1))
        if not segments:
            return cls(np.zeros((0, 2, 2), dtype=np.float64))
        return cls(np.concatenate(segments, axis=0))

    def around_ego(
        self,
        ego_xy: Iterable[float],
        heading_deg: float,
        radius_m: float,
    ) -> np.ndarray:
        """Return nearby segments expressed in rear-axle ``base_link`` XY."""

        ego = np.asarray(tuple(ego_xy), dtype=np.float64)
        values = np.asarray([*ego, float(heading_deg), float(radius_m)])
        if ego.shape != (2,) or not np.isfinite(values).all() or radius_m <= 0.0:
            raise ValueError("ego pose and positive radius must be finite")
        if len(self.segments_xy) == 0:
            return self.segments_xy.copy()

        keep = (
            np.linalg.norm(self._midpoints_xy - ego, axis=1)
            <= radius_m + self._half_lengths_m
        )
        selected = self.segments_xy[keep]
        if not len(selected):
            return np.zeros((0, 2, 2), dtype=np.float64)

        heading = np.deg2rad(float(heading_deg))
        forward = np.asarray([np.cos(heading), np.sin(heading)])
        left = np.asarray([-np.sin(heading), np.cos(heading)])
        delta = selected - ego.reshape(1, 1, 2)
        return np.stack((delta @ forward, delta @ left), axis=2)

    def nearest_distance_m(self, point_xy: Iterable[float]) -> float:
        """Return the exact 2-D distance from a point to the closest segment."""

        point = np.asarray(tuple(point_xy), dtype=np.float64)
        if point.shape != (2,) or not np.isfinite(point).all():
            raise ValueError("point_xy must contain two finite values")
        if len(self.segments_xy) == 0:
            return float("inf")

        starts = self.segments_xy[:, 0]
        vectors = self.segments_xy[:, 1] - starts
        squared_lengths = np.einsum("ij,ij->i", vectors, vectors)
        factors = np.zeros(len(vectors), dtype=np.float64)
        nonzero = squared_lengths > np.finfo(np.float64).eps
        factors[nonzero] = (
            np.einsum("ij,ij->i", point - starts[nonzero], vectors[nonzero])
            / squared_lengths[nonzero]
        )
        factors = np.clip(factors, 0.0, 1.0)
        closest = starts + factors[:, None] * vectors
        return float(np.min(np.linalg.norm(closest - point, axis=1)))


def lane_color_group(feature: dict) -> str:
    """Return a stable visualization group for an MGeo lane boundary."""

    raw = feature.get("lane_color") or []
    values = {str(value).strip().lower() for value in raw}
    for known in ("yellow", "white", "blue"):
        if known in values:
            return known
    return "unknown"


def grouped_lane_layers(
    features: Iterable[dict], translation_xy: Iterable[float]
) -> dict[str, SegmentLayer]:
    grouped = {name: [] for name in ("white", "yellow", "blue", "unknown")}
    for feature in features:
        grouped[lane_color_group(feature)].append(feature)
    return {
        name: SegmentLayer.from_features(values, translation_xy)
        for name, values in grouped.items()
    }
