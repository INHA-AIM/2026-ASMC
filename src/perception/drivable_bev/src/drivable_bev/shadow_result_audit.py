"""Pure aggregation helpers for the GPS-shadow boundary result bags."""

from __future__ import annotations

from collections import Counter
from typing import Iterable, Mapping, Optional, Sequence


BOUNDARY_ROLES = (
    "left_outer_boundary",
    "yellow_centre_boundary",
    "right_outer_boundary",
)


def typed_boundary_frame(message) -> Optional[list]:
    """Convert the general typed contract into shadow-only audit roles.

    The inferred role exists only in this audit. It is not part of the public
    perception message and must not be used as a home/opposing label.
    """
    if not bool(message.valid):
        return None
    ordered = sorted(message.boundaries, key=lambda item: int(item.lateral_order))
    yellow_indices = [
        index for index, item in enumerate(ordered) if int(item.color) == 2
    ]
    yellow_index = yellow_indices[0] if len(yellow_indices) == 1 else None
    frame_order_valid = all(bool(item.order_valid) for item in ordered)
    result = []
    for index, item in enumerate(ordered):
        color = int(item.color)
        if frame_order_valid and color == 2 and yellow_index == index:
            role = "yellow_centre_boundary"
        elif (
            frame_order_valid
            and color == 1
            and yellow_index is not None
            and index < yellow_index
        ):
            role = "left_outer_boundary"
        elif (
            frame_order_valid
            and color == 1
            and yellow_index is not None
            and index > yellow_index
        ):
            role = "right_outer_boundary"
        else:
            role = "unresolved"
        result.append(
            {
                "status": "provisional_shadow_boundary",
                "stable_id": int(item.tracking_id),
                "lateral_order_left_to_right": int(item.lateral_order),
                "order_valid": bool(item.order_valid),
                "role": role,
                "class_id": color,
                "color": {1: "white", 2: "yellow"}.get(color, "unknown"),
                "score": float(item.geometric_quality),
                "confidence": float(item.semantic_score),
                "reference_x_m": float(item.reference_x_m),
                "reference_y_m": float(item.reference_y_m),
                "valid_x": [float(item.valid_x_min_m), float(item.valid_x_max_m)],
                "source_mask": int(item.source_mask),
                "observed_length_m": float(item.observed_length_m),
                "bridged_ratio": float(item.bridged_length_ratio),
                "track_age_frames": int(item.age_frames),
                "reacquired_after_missed_frames": int(
                    item.reacquired_after_missed_frames
                ),
            }
        )
    return result


def _rate(numerator: int, denominator: int) -> float:
    return 0.0 if denominator <= 0 else float(numerator) / float(denominator)


def summarize_boundary_frames(frames: Sequence[Sequence[Mapping]]) -> dict:
    """Summarize decoded boundary marker payloads without ROS dependencies."""
    total = len(frames)
    any_count = yellow_count = complete_count = unresolved_count = 0
    duplicate_role_frames = lateral_order_error_frames = order_invalid_frames = 0
    total_boundaries = reacquired_events = 0
    role_presence = Counter()
    stable_ids = set()
    role_id_switches = Counter()
    previous_id_by_role = {}

    for frame in frames:
        values = [
            dict(item)
            for item in frame
            if item.get("status") == "provisional_shadow_boundary"
        ]
        total_boundaries += len(values)
        any_count += bool(values)
        roles = [str(item.get("role", "unresolved")) for item in values]
        role_counts = Counter(roles)
        for role in set(roles):
            role_presence[role] += 1
        yellow_count += "yellow_centre_boundary" in role_counts
        complete_count += set(BOUNDARY_ROLES).issubset(role_counts)
        unresolved_count += "unresolved" in role_counts
        duplicate_role_frames += any(
            count > 1 for role, count in role_counts.items() if role != "unresolved"
        )

        orders = [
            int(item["lateral_order_left_to_right"])
            for item in values
            if item.get("lateral_order_left_to_right") is not None
        ]
        if orders and (orders != sorted(orders) or len(orders) != len(set(orders))):
            lateral_order_error_frames += 1
        order_invalid_frames += any(item.get("order_valid") is False for item in values)

        for item in values:
            stable_id = item.get("stable_id")
            if stable_id is not None:
                stable_id = int(stable_id)
                stable_ids.add(stable_id)
                role = str(item.get("role", "unresolved"))
                # Up to two fallback boundaries can legitimately share the
                # unresolved role in one frame, so a role-based switch metric
                # is meaningful only for the three unique resolved roles.
                if role in BOUNDARY_ROLES:
                    previous = previous_id_by_role.get(role)
                    if previous is not None and stable_id != previous:
                        role_id_switches[role] += 1
                    previous_id_by_role[role] = stable_id
            if int(item.get("reacquired_after_missed_frames", 0) or 0) > 0:
                reacquired_events += 1

    return {
        "frames": total,
        "frames_with_any_boundary": any_count,
        "any_boundary_rate": round(_rate(any_count, total), 6),
        "yellow_centre_frames": yellow_count,
        "yellow_centre_rate": round(_rate(yellow_count, total), 6),
        "corridor_complete_frames": complete_count,
        "corridor_complete_rate": round(_rate(complete_count, total), 6),
        "unresolved_frames": unresolved_count,
        "unresolved_rate": round(_rate(unresolved_count, total), 6),
        "role_presence_frames": dict(sorted(role_presence.items())),
        "role_presence_rates": {
            role: round(_rate(role_presence[role], total), 6)
            for role in sorted(role_presence)
        },
        "boundaries_total": total_boundaries,
        "boundaries_per_frame": round(_rate(total_boundaries, total), 6),
        "unique_stable_ids": len(stable_ids),
        "stable_id_switches_by_role": dict(sorted(role_id_switches.items())),
        "stable_id_switches_total": sum(role_id_switches.values()),
        "reacquired_events": reacquired_events,
        "duplicate_role_frames": duplicate_role_frames,
        "lateral_order_error_frames": lateral_order_error_frames,
        "order_invalid_frames": order_invalid_frames,
    }


def combine_frame_groups(groups: Iterable[Sequence[Sequence[Mapping]]]) -> list:
    result = []
    for frames in groups:
        result.extend(frames)
    return result


def summarize_boundary_run_groups(
    groups: Iterable[Sequence[Sequence[Mapping]]],
) -> dict:
    """Aggregate runs while keeping temporal-ID metrics inside run boundaries."""
    groups = [list(frames) for frames in groups]
    result = summarize_boundary_frames(combine_frame_groups(groups))
    per_run = [summarize_boundary_frames(frames) for frames in groups]
    additive = (
        "unique_stable_ids",
        "stable_id_switches_total",
        "reacquired_events",
        "duplicate_role_frames",
        "lateral_order_error_frames",
        "order_invalid_frames",
    )
    for key in additive:
        result[key] = sum(value[key] for value in per_run)
    role_switches = Counter()
    for value in per_run:
        role_switches.update(value["stable_id_switches_by_role"])
    result["stable_id_switches_by_role"] = dict(sorted(role_switches.items()))
    return result


def robustness_delta(clear_summary: Mapping, static_summary: Mapping) -> dict:
    """Return static-obstacle minus clear detection-rate deltas."""
    keys = (
        "any_boundary_rate",
        "yellow_centre_rate",
        "corridor_complete_rate",
        "unresolved_rate",
        "boundaries_per_frame",
    )
    return {
        key: round(float(static_summary.get(key, 0.0)) - float(clear_summary.get(key, 0.0)), 6)
        for key in keys
    }
