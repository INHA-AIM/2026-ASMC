"""Measurement-only map comparison for unsupported lane-family geometry."""

from __future__ import annotations

from typing import Dict

import numpy as np


ERROR_BANDS_M = (0.35, 0.60)


def invalid_distance_band(distance_p95_m):
    """Return a non-binding visual-review band for pseudo-GT disagreement."""
    if distance_p95_m is None or not np.isfinite(float(distance_p95_m)):
        return "unavailable"
    value = float(distance_p95_m)
    if value <= ERROR_BANDS_M[0]:
        return "supportive"
    if value <= ERROR_BANDS_M[1]:
        return "review"
    return "priority_review"


def invalid_category_masks(sample_x, sample_state, usability: Dict, classification: Dict):
    """Locate unsupported samples associated with each frame-level diagnosis.

    Diagnoses are intentionally multi-label.  The returned masks explain
    retained mathematical geometry only and never promote INVALID support.
    """
    x = np.asarray(sample_x, dtype=np.float64).reshape(-1)
    state = np.asarray(sample_state).reshape(-1)
    if x.shape != state.shape or not np.isfinite(x).all():
        raise ValueError("sample x/state must be finite and have identical shape")
    invalid = state == 0
    measurements = classification.get("measurements") or {}
    flags = classification.get("flags") or {}
    tolerance = float((classification.get("thresholds") or {}).get("gap_tolerance_m", 0.0))
    masks = {}

    if flags.get("vehicle_near_leading_tail", False):
        start = float(measurements["control_x_min_m"])
        stop = start + float(measurements["vehicle_near_leading_tail_m"])
        masks["vehicle_near_leading_tail"] = invalid & (x >= start - 1e-9) & (x <= stop + tolerance)

    if flags.get("preview_trailing_tail", False):
        stop = float(measurements["preview_x_max_m"])
        start = stop - float(measurements["preview_trailing_tail_m"])
        masks["preview_trailing_tail"] = invalid & (x >= start - tolerance) & (x <= stop + 1e-9)

    if flags.get("whole_support_shrink", False):
        start = float(measurements["control_x_min_m"])
        stop = float(measurements["control_x_max_m"])
        masks["whole_support_shrink"] = invalid & (x >= start) & (x <= stop)

    if flags.get("internal_support_gap", False):
        start = float(measurements["control_x_min_m"])
        stop = float(measurements["control_x_max_m"])
        in_control = (x >= start) & (x <= stop)
        valid_indices = np.flatnonzero((~invalid) & in_control)
        mask = np.zeros_like(invalid)
        if len(valid_indices) >= 2:
            mask = invalid & in_control & (x > x[valid_indices[0]]) & (x < x[valid_indices[-1]])
        masks["internal_support_gap"] = mask

    if flags.get("preview_entry_missing", False):
        start = float(measurements["preview_x_min_m"])
        stop = float(measurements["preview_x_max_m"])
        in_preview = (x >= start) & (x <= stop)
        valid_x = x[(~invalid) & in_preview]
        end = stop if not len(valid_x) else float(np.min(valid_x))
        masks["preview_entry_missing"] = invalid & in_preview & (x <= end + tolerance)

    if not any(bool(value) for value in flags.values()):
        masks["no_material_invalid_support"] = invalid
    return {name: np.ascontiguousarray(mask) for name, mask in masks.items()}


class InvalidMapReviewSampler:
    """Select sparse overlays per diagnosis/error-band pair deterministically."""

    def __init__(self, limit_per_category_band=0, min_frame_gap=15):
        self.limit = int(limit_per_category_band)
        self.min_frame_gap = int(min_frame_gap)
        if self.limit < 0 or self.min_frame_gap < 0:
            raise ValueError("invalid map-review sampler limits")
        self.counts = {}
        self.last_frame = {}

    def select(self, frame_index, category_bands):
        selected = []
        for category, band in sorted(category_bands.items()):
            key = (str(category), str(band))
            count = self.counts.get(key, 0)
            previous = self.last_frame.get(key)
            if self.limit <= 0 or count >= self.limit:
                continue
            if previous is not None and int(frame_index) - previous < self.min_frame_gap:
                continue
            self.counts[key] = count + 1
            self.last_frame[key] = int(frame_index)
            selected.append(key)
        return tuple(selected)

    def summary(self):
        return {
            "{}/{}".format(category, band): int(count)
            for (category, band), count in sorted(self.counts.items())
        }
