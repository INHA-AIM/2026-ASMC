"""
MORAI K-City / KATRI intersection phase tables.

IntscnTL ``state`` is an intersection **phase index** (0, 1, 2, …), NOT a LightColor bitmask.
Each phase row lists one symbolic state per *traffic number* (SSN group in synced JSON).
"""
from __future__ import annotations

import json
import math
from collections import defaultdict
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Set, Tuple

INTTL1_PHASES: List[List[str]] = [
    ["G_with_GLeft", "R", "R"],
    ["R", "G_with_GLeft", "R"],
    ["R", "R", "G_with_GLeft"],
]

INTTL2_PHASES: List[List[str]] = [
    ["SG", "R", "SG", "R"],
    ["G_with_GLeft", "R", "R", "R"],
    ["R", "R", "R", "G_with_GLeft"],
    ["R", "SG", "R", "SG"],
    ["R", "G_with_GLeft", "R", "R"],
    ["R", "R", "G_with_GLeft", "R"],
]

INTTL3_PHASES: List[List[str]] = [
    ["SG", "R"],
    ["R", "SG"],
]

INTTL4_PHASES: List[List[str]] = [
    ["R_with_GLeft", "R", "R"],
    ["R", "R", "G_with_GLeft"],
    ["R", "SG", "SG"],
]

_INTTL_4LEG_6PHASE = INTTL2_PHASES

INTTL8_PHASES: List[List[str]] = [
    ["R", "R", "G_with_GLeft"],
    ["R", "R_with_GLeft", "R"],
    ["SG", "R", "R"],
]

INTERSECTION_PHASE_TABLES: Dict[str, List[List[str]]] = {
    "IntTL1": INTTL1_PHASES,
    "IntTL2": INTTL2_PHASES,
    "IntTL3": INTTL3_PHASES,
    "IntTL4": INTTL4_PHASES,
    "IntTL5": _INTTL_4LEG_6PHASE,
    "IntTL6": _INTTL_4LEG_6PHASE,
    "IntTL7": _INTTL_4LEG_6PHASE,
    "IntTL8": INTTL8_PHASES,
}

LIGHT_NAME_TO_BITMASK: Dict[str, int] = {
    "R": 0b000000000001,
    "Y": 0b000000000100,
    "SG": 0b000000010000,
    "LG": 0b000000100000,
    "RG": 0b000001000000,
    "R_with_Y": 0b000000000001 | 0b000000000100,
    "Y_with_G": 0b000000000100 | 0b000000010000,
    "Y_with_GLeft": 0b000000000100 | 0b000000100000,
    "G_with_GLeft": 0b000000010000 | 0b000000100000,
    "R_with_GLeft": 0b000000000001 | 0b000000100000,
}


def light_name_to_bitmask(name: str) -> int:
    key = (name or "").strip()
    if not key:
        return 0
    return int(LIGHT_NAME_TO_BITMASK.get(key, 0))


def _normalize_deg(deg: float) -> float:
    return (float(deg) % 360.0 + 360.0) % 360.0


def _angle_diff_deg(a: float, b: float) -> float:
    d = abs(_normalize_deg(a) - _normalize_deg(b))
    return min(d, 360.0 - d)


def _group_centroid(
    group: Sequence[str],
    catalog: Dict[str, Tuple[float, float, float, Tuple[str, ...]]],
) -> Optional[Tuple[float, float]]:
    pts = []
    for sig_id in group:
        if sig_id in catalog:
            pts.append((catalog[sig_id][0], catalog[sig_id][1]))
    if not pts:
        return None
    return sum(p[0] for p in pts) / len(pts), sum(p[1] for p in pts) / len(pts)


def _group_face_heading_deg(
    group: Sequence[str],
    catalog: Dict[str, Tuple[float, float, float, Tuple[str, ...]]],
) -> Optional[float]:
    headings = [float(catalog[s][2]) for s in group if s in catalog]
    if not headings:
        return None
    sx = sum(math.cos(math.radians(h)) for h in headings)
    sy = sum(math.sin(math.radians(h)) for h in headings)
    if abs(sx) < 1e-9 and abs(sy) < 1e-9:
        return _normalize_deg(headings[0])
    return _normalize_deg(math.degrees(math.atan2(sy, sx)))


def pick_ego_traffic_group_indices(
    ego_x: float,
    ego_y: float,
    ego_yaw_deg: float,
    traffic_groups: List[List[str]],
    catalog: Dict[str, Tuple[float, float, float, Tuple[str, ...]]],
    *,
    max_ahead_angle_deg: float = 95.0,
    max_lateral_angle_deg: float = 60.0,
    max_align_diff_deg: float = 50.0,
) -> Tuple[List[int], bool]:
    """
  Pick the single traffic-number group ego is approaching.

  Returns (group_indices, confident). When confident is False, callers should not
  paint intersection phase (unknown approach — avoids opposite-leg stale colors).
    """
    yaw = math.radians(ego_yaw_deg)
    fwd = (math.cos(yaw), math.sin(yaw))
    max_ahead = math.cos(math.radians(max_ahead_angle_deg))
    max_lat = math.sin(math.radians(max_lateral_angle_deg))

    candidates: List[Tuple[float, float, int]] = []
    for gi, group in enumerate(traffic_groups):
        c = _group_centroid(group, catalog)
        if c is None:
            continue
        face = _group_face_heading_deg(group, catalog)
        # Catalog heading = direction the signal faces (oncoming traffic approach axis).
        align = _angle_diff_deg(ego_yaw_deg, face) if face is not None else 180.0

        dx, dy = c[0] - ego_x, c[1] - ego_y
        dist = math.hypot(dx, dy)
        if dist > 1.0:
            ux, uy = dx / dist, dy / dist
            ahead = ux * fwd[0] + uy * fwd[1]
            if ahead < max_ahead:
                continue
            lat = abs(-ux * fwd[1] + uy * fwd[0])
            if lat > max_lat:
                continue

        candidates.append((align, dist, gi))

    if not candidates:
        return [], False

    candidates.sort(key=lambda x: (x[0], x[1]))
    best_align, _best_dist, best_gi = candidates[0]
    confident = best_align <= max_align_diff_deg
    return [best_gi], confident


def intersection_centroid(
    traffic_groups: List[List[str]],
    catalog: Dict[str, Tuple[float, float, float, Tuple[str, ...]]],
) -> Optional[Tuple[float, float]]:
    pts: List[Tuple[float, float]] = []
    for group in traffic_groups:
        c = _group_centroid(group, catalog)
        if c is not None:
            pts.append(c)
    if not pts:
        return None
    return sum(p[0] for p in pts) / len(pts), sum(p[1] for p in pts) / len(pts)


def expand_intersection_phase(
    inttl: str,
    phase: int,
    traffic_groups: List[List[str]],
    group_indices: Optional[List[int]] = None,
) -> Dict[str, int]:
    table = INTERSECTION_PHASE_TABLES.get(str(inttl))
    if not table or not traffic_groups:
        return {}

    phase_idx = int(phase) % len(table)
    row = table[phase_idx]
    out: Dict[str, int] = {}
    indices = group_indices if group_indices is not None else list(range(len(traffic_groups)))

    for gi in indices:
        if gi < 0 or gi >= len(traffic_groups):
            continue
        group = traffic_groups[gi]
        if not group:
            continue
        light_name = row[gi] if gi < len(row) else "R"
        bm = light_name_to_bitmask(light_name)
        if bm <= 0:
            continue
        for sig_id in group:
            out[str(sig_id)] = bm
    return out


def load_intersection_traffic_groups(
    synced_json_path: Path,
    vehicle_signal_ids: Set[str],
) -> Dict[str, List[List[str]]]:
    if not synced_json_path.is_file():
        return {}

    with open(synced_json_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    allow = vehicle_signal_ids
    buckets: Dict[str, List[tuple]] = defaultdict(list)

    for entry in data:
        ctrl = entry.get("intersection_controller_id")
        if not ctrl or not str(ctrl).startswith("IntTL"):
            continue
        ssn = str(entry.get("idx", ""))
        signals = [str(s) for s in (entry.get("signal_id_list") or [])]
        if allow is not None:
            signals = [s for s in signals if s in allow]
        if not signals:
            continue
        buckets[str(ctrl)].append((ssn, signals))

    out: Dict[str, List[List[str]]] = {}
    for ctrl, items in buckets.items():
        items.sort(key=lambda x: x[0])
        out[ctrl] = [sigs for _, sigs in items]
    return out
