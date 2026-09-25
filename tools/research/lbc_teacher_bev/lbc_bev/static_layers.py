"""Extract 320x320 ego-centric road/lane from baked global maps."""
from __future__ import annotations

from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np

from .map_baker import BakedMaps, world_to_map_pixel
from .spec import (
    MAP_SIZE,
    OBJECT_BEV_MARGIN_PX,
    PIXELS_AHEAD_VEHICLE,
    EGO_PIXEL_COL,
    EGO_PIXEL_ROW,
)

def _bev_affine_src_map(
    baked: BakedMaps,
    ego_x: float,
    ego_y: float,
    ego_yaw_deg: float,
    pixels_ahead: int,
) -> np.ndarray:
    """Map-pixel (col, row) of the 3 corners that define the ego-centric BEV warp."""
    ppm = baked.pixels_per_meter
    yaw = np.deg2rad(ego_yaw_deg)
    forward = np.array([np.cos(yaw), np.sin(yaw)], dtype=np.float32)
    right = np.array([np.cos(yaw - 0.5 * np.pi), np.sin(yaw - 0.5 * np.pi)], dtype=np.float32)
    w = float(MAP_SIZE)
    ev_to_bottom = MAP_SIZE - EGO_PIXEL_ROW
    mpp = 1.0 / ppm
    ego = np.array([ego_x, ego_y], dtype=np.float32)
    ego = ego + (pixels_ahead / ppm) * forward
    bottom_left = ego - ev_to_bottom * mpp * forward - 0.5 * w * mpp * right
    top_left = ego + (w - ev_to_bottom) * mpp * forward - 0.5 * w * mpp * right
    top_right = ego + (w - ev_to_bottom) * mpp * forward + 0.5 * w * mpp * right
    src_world = np.stack((bottom_left, top_left, top_right), axis=0)
    src_map = np.zeros((3, 2), dtype=np.float32)
    for i, (wx, wy) in enumerate(src_world):
        src_map[i] = world_to_map_pixel(float(wx), float(wy), ppm, baked.world_offset)
    return src_map


def compute_bev_affine_matrix(
    baked: BakedMaps,
    ego_x: float,
    ego_y: float,
    ego_yaw_deg: float,
    pixels_ahead: int = PIXELS_AHEAD_VEHICLE,
) -> np.ndarray:
    """Affine: global map pixel (col, row) -> BEV (col, row). Same warp as road/lane."""
    src_map = _bev_affine_src_map(baked, ego_x, ego_y, ego_yaw_deg, pixels_ahead)
    dst = np.array([[0, MAP_SIZE - 1], [0, 0], [MAP_SIZE - 1, 0]], dtype=np.float32)
    return cv2.getAffineTransform(src_map.astype(np.float32), dst)


def world_to_bev_pixel_xy_from_map(
    wx: float,
    wy: float,
    bev_affine: np.ndarray,
    baked: BakedMaps,
) -> tuple[int, int]:
    """World (m) -> BEV OpenCV (col, row) using the static-layer affine."""
    pts = world_points_to_bev_pixels(np.array([[wx, wy]], dtype=np.float32), bev_affine, baked)
    return int(pts[0, 0]), int(pts[0, 1])


def world_points_to_bev_pixels(
    points_xy: np.ndarray,
    bev_affine: np.ndarray,
    baked: BakedMaps,
) -> np.ndarray:
    """Batch world (N,2) -> BEV (N,2) int col,row."""
    pts = np.asarray(points_xy, dtype=np.float32).reshape(-1, 2)
    if pts.size == 0:
        return np.zeros((0, 2), dtype=np.int32)
    ppm = float(baked.pixels_per_meter)
    off = baked.world_offset
    mc = ppm * (pts[:, 0] - float(off[0]))
    mr = ppm * (pts[:, 1] - float(off[1]))
    map_pts = np.stack([mc, mr], axis=1).reshape(-1, 1, 2).astype(np.float32)
    out = cv2.transform(map_pts, bev_affine).reshape(-1, 2)
    return np.round(out).astype(np.int32)


def bev_pixel_in_margin(
    col: int,
    row: int,
    *,
    margin_px: int = OBJECT_BEV_MARGIN_PX,
) -> bool:
    return (
        -margin_px <= col < MAP_SIZE + margin_px
        and -margin_px <= row < MAP_SIZE + margin_px
    )


def poly_overlaps_bev(pts: np.ndarray, *, margin_px: int = OBJECT_BEV_MARGIN_PX) -> bool:
    """True if polygon AABB intersects the BEV canvas (with margin)."""
    if pts.size == 0:
        return False
    cmin, rmin = pts.min(axis=0)
    cmax, rmax = pts.max(axis=0)
    return not (
        cmax < -margin_px
        or cmin >= MAP_SIZE + margin_px
        or rmax < -margin_px
        or rmin >= MAP_SIZE + margin_px
    )


def filter_dynamic_states_to_bev(
    vehicles: Sequence,
    pedestrians: Sequence,
    bev_affine: np.ndarray,
    baked: BakedMaps,
    *,
    margin_px: int = OBJECT_BEV_MARGIN_PX,
) -> Tuple[List, List]:
    """Drop agents whose center lies outside the current BEV window (+margin)."""

    def _keep(x: float, y: float) -> bool:
        col, row = world_to_bev_pixel_xy_from_map(x, y, bev_affine, baked)
        return bev_pixel_in_margin(col, row, margin_px=margin_px)

    veh = [v for v in vehicles if _keep(v.x, v.y)]
    ped = [p for p in pedestrians if _keep(p.x, p.y)]
    return veh, ped


def world_ego_to_bev_pixel(
    ego_x: float,
    ego_y: float,
    bev_affine: np.ndarray,
    baked: BakedMaps,
    pixels_ahead: int = PIXELS_AHEAD_VEHICLE,
) -> tuple[int, int]:
    """Where raw /Ego_topic (x,y) lands on the BEV (col, row)."""
    return world_to_bev_pixel_xy_from_map(ego_x, ego_y, bev_affine, baked)


# Crop global baked map to a local ROI before warp (full KATRI raster is 10k+ px).
_STATIC_ROI_MARGIN_PX = 96


def _map_roi_from_bev_affine(M: np.ndarray, baked: BakedMaps) -> tuple[int, int, int, int]:
    """Bounding box in map pixels that covers the 320×320 BEV (+margin)."""
    bev_corners = np.array(
        [
            [[0, MAP_SIZE - 1]],
            [[0, 0]],
            [[MAP_SIZE - 1, 0]],
            [[MAP_SIZE - 1, MAP_SIZE - 1]],
        ],
        dtype=np.float32,
    )
    inv_m = cv2.invertAffineTransform(M)
    map_corners = cv2.transform(bev_corners, inv_m).reshape(-1, 2)
    margin = _STATIC_ROI_MARGIN_PX
    c0 = max(0, int(np.floor(map_corners[:, 0].min())) - margin)
    c1 = min(baked.width_pixels, int(np.ceil(map_corners[:, 0].max())) + margin)
    r0 = max(0, int(np.floor(map_corners[:, 1].min())) - margin)
    r1 = min(baked.height_pixels, int(np.ceil(map_corners[:, 1].max())) + margin)
    return c0, r0, c1, r1


def _warp_static(
    baked: BakedMaps,
    src_map: np.ndarray,
    *,
    roi: Optional[tuple[int, int, int, int]] = None,
) -> tuple[np.ndarray, np.ndarray]:
    dst = np.array([[0, MAP_SIZE - 1], [0, 0], [MAP_SIZE - 1, 0]], dtype=np.float32)
    if roi is None:
        M_roi = cv2.getAffineTransform(src_map.astype(np.float32), dst)
        road_src, lane_src = baked.road, baked.lane
    else:
        c0, r0, c1, r1 = roi
        src_adj = src_map.copy()
        src_adj[:, 0] -= float(c0)
        src_adj[:, 1] -= float(r0)
        M_roi = cv2.getAffineTransform(src_adj.astype(np.float32), dst)
        road_src = baked.road[r0:r1, c0:c1]
        lane_src = baked.lane[r0:r1, c0:c1]
    road = cv2.warpAffine(
        road_src, M_roi, (MAP_SIZE, MAP_SIZE), flags=cv2.INTER_LINEAR, borderValue=0
    )
    lane = cv2.warpAffine(
        lane_src, M_roi, (MAP_SIZE, MAP_SIZE), flags=cv2.INTER_LINEAR, borderValue=0
    )
    road = (road > 127).astype(np.uint8) * 255
    lane = (lane > 127).astype(np.uint8) * 255
    return road, lane


def extract_static_layers(
    baked: BakedMaps,
    ego_x: float,
    ego_y: float,
    ego_yaw_deg: float,
    pixels_ahead: int = PIXELS_AHEAD_VEHICLE,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    src_map = _bev_affine_src_map(baked, ego_x, ego_y, ego_yaw_deg, pixels_ahead)
    dst = np.array([[0, MAP_SIZE - 1], [0, 0], [MAP_SIZE - 1, 0]], dtype=np.float32)
    # Full-map affine for world->BEV (dynamic layers, TL, ego overlay).
    M_full = cv2.getAffineTransform(src_map.astype(np.float32), dst)
    roi = _map_roi_from_bev_affine(M_full, baked)
    road, lane = _warp_static(baked, src_map, roi=roi)
    return road, lane, M_full
