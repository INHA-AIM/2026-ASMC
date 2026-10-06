"""Classify unsupported tails without promoting them to perception evidence.

The shared lane-family message intentionally retains its full mathematical
polyline even where ``support_state`` is invalid.  This module explains where
that unsupported geometry lies.  It does not repair support, alter a message,
or grant control authority to an invalid sample.

Categories are multi-label because a frame can simultaneously lose support
near the vehicle and at the far preview edge.  ``primary_reason`` is only a
compact reporting label; consumers must inspect the flags and measurements.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Dict

import numpy as np


@dataclass(frozen=True)
class InvalidSupportClassificationConfig:
    """Names and thresholds for the provisional shadow-road audit.

    Zone bounds themselves live in :mod:`planning_usability`.  Keeping the
    classifier tied to named, recorded zones lets a later checkpoint extend
    the control or preview range by configuration rather than code changes.
    ``whole_support_min_coverage_ratio`` is a reporting threshold, not a
    production acceptance gate.
    """

    control_zone: str = "planning_consumer"
    vehicle_zone: str = "vehicle_envelope"
    near_zone: str = "near_ahead"
    mid_zone: str = "mid_ahead"
    preview_zone: str = "far_debug"
    strip: str = "both_strips"
    gap_tolerance_m: float = 0.26
    whole_support_min_coverage_ratio: float = 0.80

    def __post_init__(self):
        names = (
            self.control_zone,
            self.vehicle_zone,
            self.near_zone,
            self.mid_zone,
            self.preview_zone,
            self.strip,
        )
        if any(not str(value).strip() for value in names):
            raise ValueError("invalid-support zone and strip names must be non-empty")
        if not np.isfinite(self.gap_tolerance_m) or self.gap_tolerance_m < 0.0:
            raise ValueError("invalid-support gap tolerance must be non-negative")
        threshold = self.whole_support_min_coverage_ratio
        if not np.isfinite(threshold) or not 0.0 <= threshold <= 1.0:
            raise ValueError("whole-support coverage threshold must lie in [0,1]")

    def to_dict(self) -> Dict:
        return asdict(self)


DEFAULT_INVALID_SUPPORT_CONFIG = InvalidSupportClassificationConfig()


def invalid_support_config_from_dict(value) -> InvalidSupportClassificationConfig:
    """Create a checked config from a ROS/YAML-compatible mapping."""
    if value is None:
        return DEFAULT_INVALID_SUPPORT_CONFIG
    if not isinstance(value, dict):
        raise ValueError("invalid-support classification config must be a mapping")
    unknown = sorted(
        set(value) - set(InvalidSupportClassificationConfig.__dataclass_fields__)
    )
    if unknown:
        raise ValueError("unknown invalid-support config keys: {}".format(unknown))
    return InvalidSupportClassificationConfig(**value)


def _zone(usability, name, strip):
    zones = usability.get("zones") if isinstance(usability, dict) else None
    if not isinstance(zones, dict) or name not in zones:
        raise ValueError("planning-usability zone is missing: {}".format(name))
    value = zones[name]
    if not isinstance(value, dict) or strip not in value:
        raise ValueError(
            "planning-usability strip {!r} is missing from zone {!r}".format(
                strip, name
            )
        )
    return value, value[strip]


def _finite_metric(value, name):
    result = float(value)
    if not np.isfinite(result):
        raise ValueError("invalid-support metric must be finite: {}".format(name))
    return result


def _leading_gap(value, zone_length):
    """Return the leading gap, marking legacy inference at the caller."""
    key = "leading_unusable_gap_m"
    if key in value:
        return _finite_metric(value[key], key)
    if bool(value.get("start_anchored", False)):
        return 0.0
    if _finite_metric(value["coverage_ratio"], "coverage_ratio") <= 1e-9:
        return float(zone_length)
    return _finite_metric(value["longest_unusable_gap_m"], "longest gap")


def _trailing_gap(value, zone_length):
    """Return the trailing gap; legacy unanchored records stay conservative."""
    key = "trailing_unusable_gap_m"
    if key in value:
        return _finite_metric(value[key], key)
    coverage = _finite_metric(value["coverage_ratio"], "coverage_ratio")
    if bool(value.get("fully_continuous", False)):
        return 0.0
    if coverage <= 1e-9:
        return float(zone_length)
    if bool(value.get("start_anchored", False)):
        return _finite_metric(value["longest_unusable_gap_m"], "longest gap")
    # The old schema did not retain enough information to separate two tails.
    # Returning zero avoids mislabelling a leading-only loss as a trailing tail.
    return 0.0


def classify_invalid_support(
    usability: Dict,
    config: InvalidSupportClassificationConfig = DEFAULT_INVALID_SUPPORT_CONFIG,
) -> Dict:
    """Classify unsupported geometry already measured by the adapter rule.

    Older stored audits do not contain explicit leading/trailing lengths.  For
    them the longest boundary-touching gap is used only when the corresponding
    ``start_anchored`` flag makes its side unambiguous.  New audits always carry
    the explicit metrics.
    """
    if not isinstance(config, InvalidSupportClassificationConfig):
        raise TypeError("config must be InvalidSupportClassificationConfig")

    control_meta, control = _zone(usability, config.control_zone, config.strip)
    vehicle_meta, vehicle = _zone(usability, config.vehicle_zone, config.strip)
    _, near = _zone(usability, config.near_zone, config.strip)
    _, mid = _zone(usability, config.mid_zone, config.strip)
    preview_meta, preview = _zone(usability, config.preview_zone, config.strip)

    tolerance = float(config.gap_tolerance_m)
    threshold = float(config.whole_support_min_coverage_ratio)
    family_available = bool(usability.get("available", False))

    control_coverage = _finite_metric(control["coverage_ratio"], "control coverage")
    near_coverage = _finite_metric(near["coverage_ratio"], "near coverage")
    mid_coverage = _finite_metric(mid["coverage_ratio"], "mid coverage")
    preview_coverage = _finite_metric(
        preview["coverage_ratio"], "preview coverage"
    )

    control_zone_length = _finite_metric(
        control_meta["x_max_m"], "control end x"
    ) - _finite_metric(control_meta["x_min_m"], "control start x")
    preview_zone_length = _finite_metric(
        preview_meta["x_max_m"], "preview end x"
    ) - _finite_metric(preview_meta["x_min_m"], "preview start x")
    control_leading = _leading_gap(control, control_zone_length)
    preview_trailing = _trailing_gap(preview, preview_zone_length)
    internal_gap = _finite_metric(
        control["longest_internal_unusable_gap_m"], "control internal gap"
    )

    vehicle_front_m = _finite_metric(vehicle_meta["x_max_m"], "vehicle front x")
    control_start_m = _finite_metric(control_meta["x_min_m"], "control start x")
    leading_near_length = min(
        max(0.0, control_leading), max(0.0, vehicle_front_m - control_start_m)
    )

    # A leading tail is named vehicle-near when it removes any part of the
    # [control start, vehicle front] interval.  If support collapses farther
    # ahead the whole-support flag is additionally set; no information is lost
    # because these diagnoses are deliberately not mutually exclusive.
    vehicle_near_leading_tail = bool(
        family_available and leading_near_length > tolerance
    )
    preview_trailing_tail = bool(
        family_available and preview_trailing > tolerance
    )
    whole_support_shrink = bool(
        not family_available
        or control_coverage <= 1e-9
        or near_coverage < threshold
        or mid_coverage < threshold
    )
    internal_support_gap = bool(internal_gap > tolerance)
    preview_entry_missing = bool(
        family_available and not bool(preview.get("start_anchored", False))
    )

    flags = {
        "vehicle_near_leading_tail": vehicle_near_leading_tail,
        "preview_trailing_tail": preview_trailing_tail,
        "whole_support_shrink": whole_support_shrink,
        "internal_support_gap": internal_support_gap,
        "preview_entry_missing": preview_entry_missing,
    }
    requested = (
        "whole_support_shrink",
        "vehicle_near_leading_tail",
        "preview_trailing_tail",
        "internal_support_gap",
        "preview_entry_missing",
    )
    active = [name for name in requested if flags[name]]
    if not family_available:
        primary = "whole_support_shrink"
    elif len(active) > 1:
        primary = "mixed_invalid_support"
    elif active:
        primary = active[0]
    else:
        primary = "no_material_invalid_support"

    return {
        "schema_version": "asmc-invalid-support-classification-1.0.0",
        "measurement_only": True,
        "support_state_changed": False,
        "default_planning_use_allowed": False,
        "low_confidence_geometry_retained": family_available,
        "primary_reason": primary,
        "flags": flags,
        "measurements": {
            "control_x_min_m": control_start_m,
            "control_x_max_m": _finite_metric(
                control_meta["x_max_m"], "control end x"
            ),
            "vehicle_front_x_m": vehicle_front_m,
            "preview_x_min_m": _finite_metric(
                preview_meta["x_min_m"], "preview start x"
            ),
            "preview_x_max_m": _finite_metric(
                preview_meta["x_max_m"], "preview end x"
            ),
            "control_coverage_ratio": control_coverage,
            "near_coverage_ratio": near_coverage,
            "mid_coverage_ratio": mid_coverage,
            "preview_coverage_ratio": preview_coverage,
            "control_leading_tail_m": control_leading,
            "vehicle_near_leading_tail_m": leading_near_length,
            "preview_trailing_tail_m": preview_trailing,
            "control_internal_gap_m": internal_gap,
        },
        "thresholds": config.to_dict(),
        "tail_metrics_explicit": bool(
            "leading_unusable_gap_m" in control
            and "trailing_unusable_gap_m" in preview
        ),
    }
