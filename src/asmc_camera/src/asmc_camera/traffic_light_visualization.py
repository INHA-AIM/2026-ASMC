"""Debug overlay for typed traffic-light results.

Pure helpers so the overlay can be unit tested without ROS. The runtime node
only decodes, calls :func:`render_traffic_light_overlay` and publishes.
"""

from __future__ import annotations

from collections import OrderedDict
from typing import Generic, Optional, Sequence, Tuple, TypeVar

import cv2
import numpy as np


# asmc_msgs/TrafficLightDetection state constants -> BGR border colour.
STATE_COLORS_BGR = {
    0: (128, 128, 128),  # UNKNOWN
    1: (160, 160, 160),  # NOT_DETECTED
    2: (0, 0, 255),  # RED
    3: (0, 215, 255),  # YELLOW
    4: (0, 200, 0),  # GREEN
    5: (0, 140, 255),  # RED_YELLOW
    6: (0, 0, 255),  # RED_LEFT
    7: (0, 200, 0),  # GREEN_LEFT
    8: (0, 200, 0),  # LEFT
}
INVALID_COLOR_BGR = (128, 128, 128)

T = TypeVar("T")


class StampedCache(Generic[T]):
    """Bounded insertion-ordered cache keyed by source stamp (nanoseconds)."""

    def __init__(self, capacity: int) -> None:
        if capacity <= 0:
            raise ValueError("capacity must be positive")
        self.capacity = int(capacity)
        self._items: "OrderedDict[int, T]" = OrderedDict()

    def put(self, stamp_ns: int, value: T) -> None:
        self._items.pop(stamp_ns, None)
        self._items[stamp_ns] = value
        while len(self._items) > self.capacity:
            self._items.popitem(last=False)

    def get(self, stamp_ns: int) -> Optional[T]:
        return self._items.get(stamp_ns)

    def latest(self) -> Optional[T]:
        if not self._items:
            return None
        return next(reversed(self._items.values()))

    def __len__(self) -> int:
        return len(self._items)


def border_color(state: int, valid: bool, stale: bool) -> Tuple[int, int, int]:
    if not valid or stale:
        return INVALID_COLOR_BGR
    return STATE_COLORS_BGR.get(int(state), INVALID_COLOR_BGR)


def overlay_lines(
    label: str,
    confidence: float,
    valid: bool,
    stale: bool,
    latency_ms: float,
    candidate_labels: Sequence[str],
    candidate_confidences: Sequence[float],
    source_matched: bool,
) -> list:
    lines = ["{}  {:.2f}".format(label or "unknown", float(confidence))]
    status = "VALID" if valid and not stale else ("STALE" if stale else "INVALID")
    lines.append("{}  infer {:.1f} ms".format(status, float(latency_ms)))
    pairs = list(zip(candidate_labels, candidate_confidences))[:3]
    if pairs:
        lines.append(" | ".join("{} {:.2f}".format(name, float(score)) for name, score in pairs))
    if not source_matched:
        lines.append("source frame not cached: drawn on latest frame")
    return lines


def render_traffic_light_overlay(
    image_bgr: np.ndarray,
    state: int,
    label: str,
    confidence: float,
    valid: bool,
    stale: bool,
    latency_ms: float,
    candidate_labels: Sequence[str] = (),
    candidate_confidences: Sequence[float] = (),
    source_matched: bool = True,
) -> np.ndarray:
    """Return a copy of ``image_bgr`` with a state border and result text."""
    if image_bgr.ndim != 3 or image_bgr.shape[2] != 3:
        raise ValueError("image must be HxWx3 BGR")
    output = image_bgr.copy()
    height, width = output.shape[:2]
    color = border_color(state, valid, stale)
    thickness = max(4, min(height, width) // 60)
    cv2.rectangle(output, (0, 0), (width - 1, height - 1), color, thickness)

    lines = overlay_lines(
        label,
        confidence,
        valid,
        stale,
        latency_ms,
        candidate_labels,
        candidate_confidences,
        source_matched,
    )
    scale = max(0.5, width / 1280.0)
    line_height = int(round(28 * scale))
    box_height = line_height * len(lines) + int(round(12 * scale))
    box_width = min(width - 2 * thickness, int(round(620 * scale)))
    top_left = (thickness, thickness)
    bottom_right = (thickness + box_width, thickness + box_height)
    shade = output.copy()
    cv2.rectangle(shade, top_left, bottom_right, (0, 0, 0), -1)
    cv2.addWeighted(shade, 0.55, output, 0.45, 0.0, dst=output)
    for index, text in enumerate(lines):
        origin = (thickness + int(round(8 * scale)), thickness + line_height * (index + 1))
        cv2.putText(
            output,
            text,
            origin,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7 * scale,
            color if index == 0 else (255, 255, 255),
            max(1, int(round(2 * scale))),
            cv2.LINE_AA,
        )
    return output
