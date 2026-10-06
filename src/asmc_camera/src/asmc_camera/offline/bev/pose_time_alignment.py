"""Source-time pose alignment helpers for offline map evaluation.

This module does not compensate perception output or predict ego motion.  It
only evaluates map geometry at the sensor source timestamp so bag replay order
does not become an avoidable source of pseudo-GT error.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Optional, Sequence

import numpy as np

from .rgb_bev import StampedEgoPose


@dataclass(frozen=True)
class EgoPoseAlignment:
    pose: Optional[StampedEgoPose]
    mode: str
    target_stamp_ns: int
    nearest_age_ms: Optional[float]
    before_age_ms: Optional[float]
    after_age_ms: Optional[float]
    bracket_span_ms: Optional[float]
    interpolation_fraction: Optional[float]

    def to_dict(self):
        value = asdict(self)
        value["available"] = self.pose is not None
        value.pop("pose")
        return value


def _shortest_heading_delta_deg(start_deg: float, stop_deg: float) -> float:
    return (float(stop_deg) - float(start_deg) + 180.0) % 360.0 - 180.0


def _normalized_heading_deg(value: float) -> float:
    return (float(value) + 180.0) % 360.0 - 180.0


def align_ego_pose(
    poses: Sequence[StampedEgoPose],
    target_stamp_ns: int,
    max_nearest_age_ns: int,
    max_bracket_span_ns: int,
    mode: str = "interpolate",
) -> EgoPoseAlignment:
    """Align an Ego pose to ``target_stamp_ns`` without extrapolation.

    ``interpolate`` uses linear XY and shortest-angle heading interpolation
    when a sufficiently tight bracket exists.  Otherwise it falls back to the
    nearest sample subject to ``max_nearest_age_ns``.  ``nearest`` preserves
    the legacy evaluation behavior.
    """
    target = int(target_stamp_ns)
    max_age = int(max_nearest_age_ns)
    max_span = int(max_bracket_span_ns)
    requested = str(mode).strip().lower()
    if target <= 0 or max_age < 0 or max_span < 0:
        raise ValueError("pose-alignment timestamps and limits are invalid")
    if requested not in ("nearest", "interpolate"):
        raise ValueError("pose-alignment mode must be nearest or interpolate")
    if not poses:
        return EgoPoseAlignment(None, "unavailable", target, None, None, None, None, None)

    ordered = sorted(poses, key=lambda value: int(value.stamp_ns))
    stamps = np.asarray([value.stamp_ns for value in ordered], dtype=np.int64)
    nearest_index = int(np.argmin(np.abs(stamps - target)))
    nearest = ordered[nearest_index]
    nearest_age_ns = abs(int(nearest.stamp_ns) - target)
    nearest_age_ms = nearest_age_ns / 1e6

    exact = np.flatnonzero(stamps == target)
    if len(exact):
        pose = ordered[int(exact[-1])]
        return EgoPoseAlignment(pose, "exact", target, 0.0, 0.0, 0.0, 0.0, 0.0)

    if requested == "interpolate":
        after_index = int(np.searchsorted(stamps, target, side="right"))
        before_index = after_index - 1
        if before_index >= 0 and after_index < len(ordered):
            before = ordered[before_index]
            after = ordered[after_index]
            span_ns = int(after.stamp_ns) - int(before.stamp_ns)
            if 0 < span_ns <= max_span:
                fraction = (target - int(before.stamp_ns)) / float(span_ns)
                heading = float(before.heading_deg) + fraction * _shortest_heading_delta_deg(
                    before.heading_deg, after.heading_deg
                )
                pose = StampedEgoPose(
                    stamp_ns=target,
                    x_m=float(before.x_m) + fraction * (float(after.x_m) - float(before.x_m)),
                    y_m=float(before.y_m) + fraction * (float(after.y_m) - float(before.y_m)),
                    heading_deg=_normalized_heading_deg(heading),
                )
                return EgoPoseAlignment(
                    pose=pose,
                    mode="interpolated",
                    target_stamp_ns=target,
                    nearest_age_ms=nearest_age_ms,
                    before_age_ms=(target - int(before.stamp_ns)) / 1e6,
                    after_age_ms=(int(after.stamp_ns) - target) / 1e6,
                    bracket_span_ms=span_ns / 1e6,
                    interpolation_fraction=float(fraction),
                )

    if nearest_age_ns <= max_age:
        return EgoPoseAlignment(
            pose=nearest,
            mode="nearest" if requested == "nearest" else "nearest_fallback",
            target_stamp_ns=target,
            nearest_age_ms=nearest_age_ms,
            before_age_ms=None,
            after_age_ms=None,
            bracket_span_ms=None,
            interpolation_fraction=None,
        )
    return EgoPoseAlignment(
        pose=None,
        mode="unavailable",
        target_stamp_ns=target,
        nearest_age_ms=nearest_age_ms,
        before_age_ms=None,
        after_age_ms=None,
        bracket_span_ms=None,
        interpolation_fraction=None,
    )


def pose_delta(first: Optional[StampedEgoPose], second: Optional[StampedEgoPose]):
    """Return translation and heading difference for alignment sensitivity."""
    if first is None or second is None:
        return None
    translation = math.hypot(first.x_m - second.x_m, first.y_m - second.y_m)
    heading = abs(_shortest_heading_delta_deg(first.heading_deg, second.heading_deg))
    return {"translation_m": float(translation), "heading_deg": float(heading)}
