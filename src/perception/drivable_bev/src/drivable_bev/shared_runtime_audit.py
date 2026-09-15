"""Pure runtime-baseline aggregation for shared lane-family diagnostics."""

from __future__ import annotations

from collections import Counter
import math
from typing import Iterable, Mapping, Sequence


BOUNDARY_ROLES = (
    "left_outer_boundary",
    "yellow_centre_boundary",
    "right_outer_boundary",
)


def _rate(numerator, denominator):
    return 0.0 if denominator <= 0 else float(numerator) / float(denominator)


def _percentile(values, percentile):
    ordered = sorted(float(value) for value in values if math.isfinite(float(value)))
    if not ordered:
        return None
    position = (len(ordered) - 1) * float(percentile) / 100.0
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def _distribution(values):
    finite = [float(value) for value in values if math.isfinite(float(value))]
    return {
        "count": len(finite),
        "min": min(finite) if finite else None,
        "p50": _percentile(finite, 50),
        "p95": _percentile(finite, 95),
        "max": max(finite) if finite else None,
    }


def summarize_shared_runtime_frames(frames: Sequence[Mapping]) -> dict:
    """Summarize decoded per-frame shared-family diagnostics.

    Invalid sample length is deliberately excluded from usable support even
    when the extrapolated debug curve happens to align visually with the road.
    """
    frames = [dict(frame) for frame in frames]
    total = len(frames)
    accepted = [frame for frame in frames if bool(frame.get("accepted"))]
    reasons = Counter(str(frame.get("reason", "unknown")) for frame in frames)
    acceptance_modes = Counter(
        str(
            frame.get(
                "acceptance_mode",
                "legacy_unspecified" if frame.get("accepted") else "rejected",
            )
        )
        for frame in frames
    )
    observed_role_frames = Counter()
    inferred_role_frames = Counter()
    role_lengths = {
        role: {"observed": 0.0, "structure_inferred": 0.0, "invalid": 0.0}
        for role in BOUNDARY_ROLES
    }
    fit_ms = []
    data_residual = []
    inlier_residual = []
    outlier_rate = []
    valid_span = []
    widths = {"left": [], "right": []}
    frame_usable_ratio = []
    frame_inferred_of_usable = []
    current_rejected = longest_rejected = 0
    deployment_allowed_frames = 0

    for frame in frames:
        if bool(frame.get("deployment_allowed", False)):
            deployment_allowed_frames += 1
        if bool(frame.get("accepted")):
            current_rejected = 0
        else:
            current_rejected += 1
            longest_rejected = max(longest_rejected, current_rejected)
        for role in frame.get("observed_roles") or []:
            if role in BOUNDARY_ROLES:
                observed_role_frames[role] += 1
        for role in frame.get("inferred_roles") or []:
            if role in BOUNDARY_ROLES:
                inferred_role_frames[role] += 1
        if frame.get("fit_ms") is not None:
            fit_ms.append(frame["fit_ms"])
        if frame.get("data_residual_p95_m") is not None:
            data_residual.append(frame["data_residual_p95_m"])
        if frame.get("inlier_residual_p95_m") is not None:
            inlier_residual.append(frame["inlier_residual_p95_m"])
        if frame.get("outlier_rate") is not None:
            outlier_rate.append(frame["outlier_rate"])
        frame_widths = frame.get("widths_m") or {}
        for side in widths:
            if frame_widths.get(side) is not None:
                widths[side].append(frame_widths[side])
        valid_x = frame.get("valid_x_m") or []
        if len(valid_x) == 2 and float(valid_x[1]) >= float(valid_x[0]):
            valid_span.append(float(valid_x[1]) - float(valid_x[0]))

        frame_lengths = {"observed": 0.0, "structure_inferred": 0.0, "invalid": 0.0}
        for role, support in (frame.get("boundary_support") or {}).items():
            if role not in role_lengths or not isinstance(support, Mapping):
                continue
            values = {
                "observed": max(0.0, float(support.get("observed_length_m", 0.0))),
                "structure_inferred": max(
                    0.0, float(support.get("inferred_length_m", 0.0))
                ),
                "invalid": max(0.0, float(support.get("invalid_length_m", 0.0))),
            }
            for state, length in values.items():
                role_lengths[role][state] += length
                frame_lengths[state] += length
        usable = frame_lengths["observed"] + frame_lengths["structure_inferred"]
        total_length = usable + frame_lengths["invalid"]
        if total_length > 0.0:
            frame_usable_ratio.append(usable / total_length)
        if usable > 0.0:
            frame_inferred_of_usable.append(
                frame_lengths["structure_inferred"] / usable
            )

    total_lengths = {
        state: sum(role_lengths[role][state] for role in BOUNDARY_ROLES)
        for state in ("observed", "structure_inferred", "invalid")
    }
    usable_length = total_lengths["observed"] + total_lengths["structure_inferred"]
    curve_length = usable_length + total_lengths["invalid"]
    return {
        "frames": total,
        "accepted_frames": len(accepted),
        "accepted_rate": round(_rate(len(accepted), total), 6),
        "rejection_reasons": dict(sorted(reasons.items())),
        "acceptance_modes": dict(sorted(acceptance_modes.items())),
        "distributed_multi_role_rate": round(
            _rate(acceptance_modes["distributed_multi_role"], total), 6
        ),
        "longest_rejected_streak_frames": longest_rejected,
        "deployment_allowed_frames": deployment_allowed_frames,
        "observed_role_presence_rates": {
            role: round(_rate(observed_role_frames[role], total), 6)
            for role in BOUNDARY_ROLES
        },
        "inferred_role_presence_rates": {
            role: round(_rate(inferred_role_frames[role], total), 6)
            for role in BOUNDARY_ROLES
        },
        "fit_ms": _distribution(fit_ms),
        "data_residual_p95_m": _distribution(data_residual),
        "inlier_residual_p95_m": _distribution(inlier_residual),
        "outlier_rate": _distribution(outlier_rate),
        "valid_span_m": _distribution(valid_span),
        "widths_m": {side: _distribution(values) for side, values in widths.items()},
        "sample_length_m": {
            "by_role": role_lengths,
            "total": total_lengths,
            "usable": usable_length,
            "all": curve_length,
        },
        "sample_state_ratios": {
            "observed_of_all": round(_rate(total_lengths["observed"], curve_length), 6),
            "structure_inferred_of_all": round(
                _rate(total_lengths["structure_inferred"], curve_length), 6
            ),
            "invalid_of_all": round(_rate(total_lengths["invalid"], curve_length), 6),
            "usable_of_all": round(_rate(usable_length, curve_length), 6),
            "structure_inferred_of_usable": round(
                _rate(total_lengths["structure_inferred"], usable_length), 6
            ),
        },
        "frame_usable_ratio": _distribution(frame_usable_ratio),
        "frame_inferred_of_usable": _distribution(frame_inferred_of_usable),
        "consumption_policy": {
            "observed": "primary",
            "structure_inferred": "degraded_with_confidence",
            "invalid": "forbidden",
            "temporal_predicted": "not_implemented",
        },
    }


def summarize_shared_runtime_run_groups(groups: Iterable[Sequence[Mapping]]) -> dict:
    """Aggregate frames while preventing rejected streaks from crossing runs."""
    groups = [list(group) for group in groups]
    combined = [frame for group in groups for frame in group]
    result = summarize_shared_runtime_frames(combined)
    per_run = [summarize_shared_runtime_frames(group) for group in groups]
    result["longest_rejected_streak_frames"] = max(
        (value["longest_rejected_streak_frames"] for value in per_run), default=0
    )
    result["runs"] = len(groups)
    return result
