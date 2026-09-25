"""World (MORAI ENU meters) to LBC 320x320 BEV pixels."""
from __future__ import annotations

import math
from typing import Tuple

import numpy as np

from .spec import (
    MAP_SIZE,
    PIXELS_PER_METER,
    PIXELS_AHEAD_VEHICLE,
    EGO_PIXEL_COL,
    EGO_PIXEL_ROW,
)

# LBC dataset convention: offset after flip (see birdview_lmdb.world_to_pixel)
DEFAULT_OFFSET = (-80, 160)


def yaw_to_orientation(yaw_deg: float) -> Tuple[float, float]:
    """Unit vectors for ego forward (ori_ox, ori_oy) in LBC birdview_lmdb convention."""
    yaw = math.radians(yaw_deg)
    # Forward in world (east, north) = (cos, sin) for MORAI yaw CCW from east
    ori_ox = math.cos(yaw)
    ori_oy = math.sin(yaw)
    return ori_ox, ori_oy


def world_to_bev_pixel(
    wx: float,
    wy: float,
    ego_x: float,
    ego_y: float,
    ego_yaw_deg: float,
    *,
    offset: Tuple[int, int] = DEFAULT_OFFSET,
    apply_ahead_offset: bool = True,
) -> Tuple[int, int]:
    """
    Map world point to LBC image pixel (row, col) with shape (MAP_SIZE, MAP_SIZE).

    Ego sits at (row=EGO_PIXEL_ROW, col=EGO_PIXEL_COL). Forward is up (decreasing row).
    """
    ori_ox, ori_oy = yaw_to_orientation(ego_yaw_deg)
    ox, oy = ego_x, ego_y
    if apply_ahead_offset:
        # Shift observation center forward by PIXELS_AHEAD_VEHICLE (CARLA map_utils)
        ox += (PIXELS_AHEAD_VEHICLE / PIXELS_PER_METER) * ori_ox
        oy += (PIXELS_AHEAD_VEHICLE / PIXELS_PER_METER) * ori_oy

    pixel_dx = (wx - ox) * PIXELS_PER_METER
    pixel_dy = (wy - oy) * PIXELS_PER_METER
    px = pixel_dx * ori_ox + pixel_dy * ori_oy
    py = -pixel_dx * ori_oy + pixel_dy * ori_ox
    pixel_x = MAP_SIZE - px
    pixel_y = py
    row = int(round(pixel_x + offset[0]))
    col = int(round(pixel_y + offset[1]))
    return row, col


def world_to_bev_pixel_xy(
    wx: float,
    wy: float,
    ego_x: float,
    ego_y: float,
    ego_yaw_deg: float,
    **kwargs,
) -> Tuple[int, int]:
    """Return (x, y) OpenCV-style (col, row) for cv2 drawing."""
    row, col = world_to_bev_pixel(wx, wy, ego_x, ego_y, ego_yaw_deg, **kwargs)
    return col, row


def get_lbc_warp_transform(
    ego_x: float,
    ego_y: float,
    ego_yaw_deg: float,
    width: int = MAP_SIZE,
    ev_to_bottom: int | None = None,
    ppm: float = PIXELS_PER_METER,
) -> np.ndarray:
    """
    Affine 2x3: world meters -> BEV pixel (col, row) for cv2.transform / warpAffine dst.

    Ego at (width/2, width - ev_to_bottom), heading up.
    """
    if ev_to_bottom is None:
        ev_to_bottom = width - EGO_PIXEL_ROW
    yaw = math.radians(ego_yaw_deg)
    forward = np.array([math.cos(yaw), math.sin(yaw)], dtype=np.float32)
    right = np.array([math.cos(yaw - 0.5 * math.pi), math.sin(yaw - 0.5 * math.pi)], dtype=np.float32)
    mpp = 1.0 / ppm
    ego = np.array([ego_x, ego_y], dtype=np.float32)
    ahead_m = PIXELS_AHEAD_VEHICLE / ppm
    ego = ego + ahead_m * forward

    w = float(width)
    bottom_left = ego - ev_to_bottom * mpp * forward - 0.5 * w * mpp * right
    top_left = ego + (w - ev_to_bottom) * mpp * forward - 0.5 * w * mpp * right
    top_right = ego + (w - ev_to_bottom) * mpp * forward + 0.5 * w * mpp * right
    src = np.stack((bottom_left, top_left, top_right), axis=0).astype(np.float32)
    dst = np.array(
        [[0, width - 1], [0, 0], [width - 1, 0]], dtype=np.float32
    )
    return cv2_get_affine(src, dst)


def cv2_get_affine(src: np.ndarray, dst: np.ndarray) -> np.ndarray:
    import cv2
    return cv2.getAffineTransform(src, dst)
