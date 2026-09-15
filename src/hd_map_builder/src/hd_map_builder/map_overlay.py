"""Pure geometry used by the review-only legacy-map RViz overlay."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterable

import numpy as np


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
