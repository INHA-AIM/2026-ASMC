"""Deterministic sampling policy for invalid-support overlay review.

The review sampler is deliberately separate from the support classifier.  It
selects frames for human inspection but never changes a support state or grants
planning authority to unsupported geometry.
"""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass, field
from typing import Dict, Tuple


INVALID_REVIEW_CATEGORIES = (
    "vehicle_near_leading_tail",
    "preview_trailing_tail",
    "whole_support_shrink",
    "internal_support_gap",
    "preview_entry_missing",
)
HEALTHY_REVIEW_CATEGORY = "no_material_invalid_support"


def invalid_support_review_categories(classification: Dict) -> Tuple[str, ...]:
    """Return every active review category, preserving multi-label meaning."""
    flags = classification.get("flags") if isinstance(classification, dict) else None
    if not isinstance(flags, dict):
        raise ValueError("invalid-support classification must contain flags")
    selected = tuple(
        category
        for category in INVALID_REVIEW_CATEGORIES
        if bool(flags.get(category, False))
    )
    return selected or (HEALTHY_REVIEW_CATEGORY,)


@dataclass
class InvalidSupportReviewSampler:
    """Choose sparse, reproducible frames independently for each category."""

    limit_per_category: int = 0
    min_frame_gap: int = 15
    counts: Counter = field(default_factory=Counter, init=False)
    last_frame: Dict[str, int] = field(default_factory=dict, init=False)

    def __post_init__(self):
        if self.limit_per_category < 0:
            raise ValueError("invalid-support overlay limit must be non-negative")
        if self.min_frame_gap < 0:
            raise ValueError("invalid-support overlay frame gap must be non-negative")

    def select(self, frame_index: int, classification: Dict) -> Tuple[str, ...]:
        if frame_index < 0:
            raise ValueError("frame index must be non-negative")
        if self.limit_per_category == 0:
            return ()
        result = []
        for category in invalid_support_review_categories(classification):
            if self.counts[category] >= self.limit_per_category:
                continue
            previous = self.last_frame.get(category)
            if previous is not None and frame_index - previous < self.min_frame_gap:
                continue
            self.counts[category] += 1
            self.last_frame[category] = int(frame_index)
            result.append(category)
        return tuple(result)

    def summary(self) -> Dict[str, int]:
        categories = INVALID_REVIEW_CATEGORIES + (HEALTHY_REVIEW_CATEGORY,)
        return {category: int(self.counts[category]) for category in categories}
