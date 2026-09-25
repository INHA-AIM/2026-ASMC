"""Bake global road/lane uint8 rasters from MGeo JSON."""
from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import List, Sequence, Tuple

import cv2
import numpy as np

from .spec import (
    LBC_LANE_MARKING_WIDTH_PX,
    LBC_ROAD_POLYGON_OUTLINE_WIDTH_PX,
    PIXELS_PER_METER,
)


@dataclass
class BakedMaps:
    road: np.ndarray
    lane: np.ndarray
    world_offset: np.ndarray  # (min_x, min_y)
    pixels_per_meter: float
    width_pixels: int
    height_pixels: int


def _collect_xy_from_road(entries: list) -> Tuple[List[float], List[float]]:
    xs, ys = [], []
    for poly in entries:
        for ring_name in ('points',):
            pts = poly.get(ring_name) or []
            for p in pts:
                xs.append(float(p[0]))
                ys.append(float(p[1]))
        for interior in poly.get('interiors') or []:
            for p in interior.get('points') or []:
                xs.append(float(p[0]))
                ys.append(float(p[1]))
    return xs, ys


def _collect_xy_from_lanes(entries: list) -> Tuple[List[float], List[float]]:
    xs, ys = [], []
    for lb in entries:
        for p in lb.get('points') or []:
            xs.append(float(p[0]))
            ys.append(float(p[1]))
    return xs, ys


def compute_world_bounds(
    road_json: Path,
    lane_json: Path,
    margin_m: float = 50.0,
) -> Tuple[float, float, float, float]:
    with open(road_json, 'r', encoding='utf-8') as f:
        road = json.load(f)
    with open(lane_json, 'r', encoding='utf-8') as f:
        lanes = json.load(f)
    xs, ys = [], []
    rx, ry = _collect_xy_from_road(road)
    lx, ly = _collect_xy_from_lanes(lanes)
    xs.extend(rx)
    ys.extend(ry)
    xs.extend(lx)
    ys.extend(ly)
    if not xs:
        raise ValueError('No coordinates in map JSON')
    return (
        min(xs) - margin_m,
        min(ys) - margin_m,
        max(xs) + margin_m,
        max(ys) + margin_m,
    )


def world_to_map_pixel(
    wx: float,
    wy: float,
    ppm: float,
    world_offset: np.ndarray,
) -> Tuple[int, int]:
    """OpenCV (col, row)."""
    col = int(round(ppm * (wx - float(world_offset[0]))))
    row = int(round(ppm * (wy - float(world_offset[1]))))
    return col, row


def bake_maps(
    road_json: Path,
    lane_json: Path,
    pixels_per_meter: float = PIXELS_PER_METER,
    margin_m: float = 50.0,
) -> BakedMaps:
    min_x, min_y, max_x, max_y = compute_world_bounds(road_json, lane_json, margin_m)
    world_offset = np.array([min_x, min_y], dtype=np.float32)
    w_m = max_x - min_x
    h_m = max_y - min_y
    w_px = max(16, int(round(pixels_per_meter * w_m)))
    h_px = max(16, int(round(pixels_per_meter * h_m)))
    road = np.zeros((h_px, w_px), dtype=np.uint8)
    lane = np.zeros((h_px, w_px), dtype=np.uint8)

    with open(road_json, 'r', encoding='utf-8') as f:
        road_entries = json.load(f)
    for poly in road_entries:
        ext = poly.get('points') or []
        if len(ext) < 3:
            continue
        ext_px = np.array(
            [world_to_map_pixel(p[0], p[1], pixels_per_meter, world_offset) for p in ext],
            dtype=np.int32,
        )
        cv2.fillPoly(road, [ext_px], 255)
        if len(ext_px) >= 2:
            cv2.polylines(
                road,
                [ext_px],
                isClosed=True,
                color=255,
                thickness=LBC_ROAD_POLYGON_OUTLINE_WIDTH_PX,
            )
        for interior in poly.get('interiors') or []:
            pts = interior.get('points') or []
            if len(pts) < 3:
                continue
            int_px = np.array(
                [world_to_map_pixel(p[0], p[1], pixels_per_meter, world_offset) for p in pts],
                dtype=np.int32,
            )
            cv2.fillPoly(road, [int_px], 0)

    with open(lane_json, 'r', encoding='utf-8') as f:
        lane_entries = json.load(f)
    for lb in lane_entries:
        pts = lb.get('points') or []
        if len(pts) < 2:
            continue
        poly_px = np.array(
            [world_to_map_pixel(p[0], p[1], pixels_per_meter, world_offset) for p in pts],
            dtype=np.int32,
        )
        cv2.polylines(
            lane,
            [poly_px],
            False,
            255,
            thickness=LBC_LANE_MARKING_WIDTH_PX,
        )

    return BakedMaps(
        road=road,
        lane=lane,
        world_offset=world_offset,
        pixels_per_meter=pixels_per_meter,
        width_pixels=w_px,
        height_pixels=h_px,
    )
