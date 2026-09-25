"""Dynamic BEV layers: vehicles, pedestrians, traffic lights (LBC 3ch)."""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np
from scipy.spatial import cKDTree

from .map_baker import BakedMaps
from .spec import (
    LBC_VEHICLE_TL_TYPES,
    MAP_SIZE,
    OBJECT_BEV_MARGIN_PX,
    TL_BEV_MARGIN_PX,
    TRAFFIC_LIGHT_RADIUS_M,
    lbc_world_to_pixel_width,
)
from .static_layers import poly_overlaps_bev, world_points_to_bev_pixels


@dataclass
class VehicleState:
    x: float
    y: float
    yaw_deg: float
    length: float = 4.5
    width: float = 2.0


@dataclass
class PedestrianState:
    x: float
    y: float
    yaw_deg: float = 0.0
    length: float = 0.8
    width: float = 0.8


@dataclass
class TrafficLightState:
  idx: str
  x: float
  y: float
  state: str  # red | yellow | green
  heading_deg: float = 0.0


class TrafficLightStoplineMapper:
    """Simplified mapper: link endpoints -> nearest lane segment as stopline."""

    _NEAREST_K = 48

    def __init__(
        self,
        tl_json_path: Path,
        lane_json_path: Path,
        link_json_path: Optional[Path] = None,
        max_match_distance: float = 80.0,
    ):
        self._max_match_distance = max_match_distance
        with open(tl_json_path, 'r', encoding='utf-8') as f:
            self._tl_data = json.load(f)
        with open(lane_json_path, 'r', encoding='utf-8') as f:
            lane_data = json.load(f)
        self._lane_segments: List[Tuple[np.ndarray, np.ndarray]] = []
        mids: List[np.ndarray] = []
        for lb in lane_data:
            pts = lb.get('points') or []
            if len(pts) < 2:
                continue
            arr = np.array([[float(p[0]), float(p[1])] for p in pts], dtype=np.float32)
            for i in range(len(arr) - 1):
                a, b = arr[i], arr[i + 1]
                self._lane_segments.append((a, b))
                mids.append(0.5 * (a + b))
        if mids:
            self._segment_tree = cKDTree(np.stack(mids, axis=0))
        else:
            self._segment_tree = None

        link_endpoints: Dict[str, Tuple[float, float]] = {}
        if link_json_path and link_json_path.is_file():
            with open(link_json_path, 'r', encoding='utf-8') as f:
                for link in json.load(f):
                    pts = link.get('points')
                    if pts and len(pts) >= 1:
                        link_endpoints[str(link['idx'])] = (float(pts[0][0]), float(pts[0][1]))

        self._tl_to_stoplines: Dict[str, List[Tuple[Tuple[float, float], Tuple[float, float]]]] = {}
        for tl in self._tl_data:
            if str(tl.get("type", "")) not in LBC_VEHICLE_TL_TYPES:
                continue
            tl_idx = tl['idx']
            tx, ty = float(tl['point'][0]), float(tl['point'][1])
            matched: List[Tuple[Tuple[float, float], Tuple[float, float]]] = []
            for link_id in tl.get('link_id_list') or []:
                ep = link_endpoints.get(str(link_id))
                if ep is None:
                    continue
                sl = self._nearest_lane_stopline(ep[0], ep[1])
                if sl is not None:
                    matched.append(sl)
            if not matched:
                sl = self._nearest_lane_stopline(tx, ty)
                if sl is not None:
                    matched.append(sl)
            if matched:
                self._tl_to_stoplines[tl_idx] = matched

    def _nearest_lane_stopline(
        self, x: float, y: float
    ) -> Optional[Tuple[Tuple[float, float], Tuple[float, float]]]:
        if not self._lane_segments:
            return None
        p = np.array([x, y], dtype=np.float32)
        if self._segment_tree is None:
            candidate_idxs = range(len(self._lane_segments))
        else:
            k = min(self._NEAREST_K, len(self._lane_segments))
            _, idxs = self._segment_tree.query(p, k=k)
            candidate_idxs = [int(idxs)] if np.isscalar(idxs) else [int(i) for i in idxs]
        best_d = float('inf')
        best: Optional[Tuple[Tuple[float, float], Tuple[float, float]]] = None
        for i in candidate_idxs:
            a, b = self._lane_segments[i]
            ab = b - a
            t = np.dot(p - a, ab) / (np.dot(ab, ab) + 1e-9)
            t = float(np.clip(t, 0.0, 1.0))
            proj = a + t * ab
            d = float(np.linalg.norm(p - proj))
            if d < best_d:
                best_d = d
                seg_len = float(np.linalg.norm(ab))
                if seg_len < 1e-3:
                    continue
                perp = np.array([-ab[1], ab[0]], dtype=np.float32) / seg_len
                half = 2.5
                c = proj
                best = (
                    (float(c[0] - perp[0] * half), float(c[1] - perp[1] * half)),
                    (float(c[0] + perp[0] * half), float(c[1] + perp[1] * half)),
                )
        if best is not None and best_d > self._max_match_distance:
            return None
        return best

    def stoplines_for(self, tl_idx: str) -> List[Tuple[Tuple[float, float], Tuple[float, float]]]:
        return self._tl_to_stoplines.get(tl_idx, [])


def _lbc_bbox_polygon_world(
    x: float,
    y: float,
    yaw_deg: float,
    length: float,
    width: float,
) -> np.ndarray:
    """
    CARLA/LBC axis-aligned bbox footprint (map_utils._render_vehicles / _render_walkers).
    extent = half-length / half-width; corner order matches pygame.draw.polygon.
    """
    hl = 0.5 * float(length)
    hw = 0.5 * float(width)
    yaw = math.radians(yaw_deg)
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)
    local = (
        (-hl, -hw),
        (-hl, hw),
        (hl, hw),
        (hl, -hw),
    )
    out = np.empty((4, 2), dtype=np.float32)
    for i, (lx, ly) in enumerate(local):
        out[i, 0] = x + lx * cos_y - ly * sin_y
        out[i, 1] = y + lx * sin_y + ly * cos_y
    return out


def _render_bbox_entities(
    canvas: np.ndarray,
    entities: Sequence,
    bev_affine: np.ndarray,
    baked: BakedMaps,
    *,
    margin_px: int = OBJECT_BEV_MARGIN_PX,
) -> None:
    for ent in entities:
        poly = _lbc_bbox_polygon_world(
            ent.x, ent.y, ent.yaw_deg, ent.length, ent.width
        )
        pts = world_points_to_bev_pixels(poly, bev_affine, baked)
        if not poly_overlaps_bev(pts, margin_px=margin_px):
            continue
        arr = pts.reshape(-1, 1, 2)
        cv2.fillPoly(canvas, [arr], 255)


def render_vehicles(
    canvas: np.ndarray,
    vehicles: Sequence[VehicleState],
    bev_affine: np.ndarray,
    baked: BakedMaps,
    *,
    margin_px: int = OBJECT_BEV_MARGIN_PX,
) -> None:
    _render_bbox_entities(
        canvas, vehicles, bev_affine, baked, margin_px=margin_px
    )


def render_pedestrians(
    canvas: np.ndarray,
    pedestrians: Sequence[PedestrianState],
    bev_affine: np.ndarray,
    baked: BakedMaps,
    *,
    margin_px: int = OBJECT_BEV_MARGIN_PX,
) -> None:
    _render_bbox_entities(
        canvas, pedestrians, bev_affine, baked, margin_px=margin_px
    )


def render_traffic_lights(
    red: np.ndarray,
    yellow: np.ndarray,
    green: np.ndarray,
    traffic_lights: Sequence[TrafficLightState],
    bev_affine: np.ndarray,
    baked: BakedMaps,
    *,
    radius_m: float = TRAFFIC_LIGHT_RADIUS_M,
    margin_px: int = TL_BEV_MARGIN_PX,
) -> None:
    """LBC map_utils._render_traffic_lights: pygame.draw.circle at TL actor position."""
    radius_px = max(1, lbc_world_to_pixel_width(float(radius_m)))
    for tl in traffic_lights:
        state = (tl.state or "").lower()
        if state not in ("red", "yellow", "green"):
            continue
        center = world_points_to_bev_pixels(
            np.array([[tl.x, tl.y]], dtype=np.float32), bev_affine, baked
        )[0]
        col, row = int(center[0]), int(center[1])
        if not (
            -margin_px <= col < MAP_SIZE + margin_px
            and -margin_px <= row < MAP_SIZE + margin_px
        ):
            continue
        target = red if state == "red" else yellow if state == "yellow" else green
        cv2.circle(target, (col, row), radius_px, 255, thickness=-1)


def render_dynamic_layers(
    bev_affine: np.ndarray,
    baked: BakedMaps,
    vehicles: Sequence[VehicleState],
    pedestrians: Sequence[PedestrianState],
    traffic_lights: Sequence[TrafficLightState],
    mapper: Optional[TrafficLightStoplineMapper] = None,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    veh = np.zeros((MAP_SIZE, MAP_SIZE), dtype=np.uint8)
    ped = np.zeros((MAP_SIZE, MAP_SIZE), dtype=np.uint8)
    tl_r = np.zeros((MAP_SIZE, MAP_SIZE), dtype=np.uint8)
    tl_y = np.zeros((MAP_SIZE, MAP_SIZE), dtype=np.uint8)
    tl_g = np.zeros((MAP_SIZE, MAP_SIZE), dtype=np.uint8)
    render_vehicles(veh, vehicles, bev_affine, baked)
    render_pedestrians(ped, pedestrians, bev_affine, baked)
    del mapper  # stopline mapper kept for diagnostics; LBC TL rendering uses pole positions only
    render_traffic_lights(tl_r, tl_y, tl_g, traffic_lights, bev_affine, baked)
    return veh, ped, tl_r, tl_y, tl_g
