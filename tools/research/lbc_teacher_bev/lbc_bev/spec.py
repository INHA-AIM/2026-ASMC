"""LBC 7-channel bird's-eye view constants and helpers (LearningByCheating paper spec)."""
from __future__ import annotations

import numpy as np

PIXELS_PER_METER = 5
MAP_SIZE = 320
NUM_CHANNELS = 7
CROP_SIZE = 192
# 20 m forward from ego pose at 5 px/m (LBC/CARLA observation center).
# MORAI ego = rear axle; use pixels_ahead=0 to align anchor with /Ego_topic.
PIXELS_AHEAD_VEHICLE = 100

# LBC privileged BEV: vehicle TL as circles at pole position (channels 2–4); no pedestrian signals.
LBC_VEHICLE_TL_TYPES = frozenset({"car"})

# LearningByCheating bird_view/utils/map_utils.py (PIXELS_PER_METER=5, scale=1.0)
LBC_LANE_MARKING_WIDTH_PX = 2  # draw_lane_marking: pygame.draw.lines(..., 2)
LBC_ROAD_POLYGON_OUTLINE_WIDTH_PX = 10  # pygame.draw.polygon(..., width=10) + fill
TRAFFIC_LIGHT_RADIUS_M = 1.5  # _render_traffic_lights: world_to_pixel_width(1.5)
# BEV window is MAP_SIZE/ppm meters; margin keeps pole markers visible near edges.
# Full BEV ~64 m; extra margin keeps distant poles visible before ego reaches them.
TL_BEV_MARGIN_PX = 160
# Vehicles/pedestrians: ~10 m outside 320 px so they appear before entering the window.
OBJECT_BEV_MARGIN_PX = 50
# Beyond this, intersection phase is not applied (avoids stale wrong-leg phase far away).
TL_PHASE_MAX_DISTANCE_M = 90.0
# Pre-filter /Object_topic agents by ego distance before BEV polygon work (m).
BEV_OBJECT_FILTER_RADIUS_M = 45.0

# CARLA walker bbox.extent ~0.35–0.4 m (half) → ~0.7–0.8 m full footprint in LBC map_utils
LBC_DEFAULT_PEDESTRIAN_LENGTH_M = 0.8
LBC_DEFAULT_PEDESTRIAN_WIDTH_M = 0.8


def lbc_world_to_pixel_width(width_m: float, pixels_per_meter: float = PIXELS_PER_METER) -> int:
    """Same as MapImage.world_to_pixel_width at scale 1.0."""
    return int(pixels_per_meter * width_m)

# Ego in full 320x320 BEV: column x=160, row y=260 (OpenCV: col, row)
EGO_PIXEL_COL = MAP_SIZE // 2
EGO_PIXEL_ROW = 260

# crop_birdview uses first index as row (x in LBC code), second as col (y)
CROP_CENTER_ROW = 260 - CROP_SIZE // 2  # 164
CROP_CENTER_COL = MAP_SIZE // 2       # 160

BACKGROUND = np.array([0, 47, 0], dtype=np.uint8)
COLORS = [
    (102, 102, 102),  # road
    (253, 253, 17),   # lane
    (204, 6, 5),      # red TL
    (250, 210, 1),    # yellow TL
    (39, 232, 51),    # green TL
    (0, 0, 142),      # vehicle
    (220, 20, 60),    # pedestrian
]
# imshow ego overlay: filled polygon like NPC vehicles, distinct from channel-5 blue
EGO_VIS_COLOR = (100, 200, 255)


def stack_birdview(
    road: np.ndarray,
    lane: np.ndarray,
    traffic_red: np.ndarray,
    traffic_yellow: np.ndarray,
    traffic_green: np.ndarray,
    vehicle: np.ndarray,
    pedestrian: np.ndarray,
) -> np.ndarray:
    """Stack layers into (H, W, 7) uint8 birdview."""
    layers = [
        road,
        lane,
        traffic_red,
        traffic_yellow,
        traffic_green,
        vehicle,
        pedestrian,
    ]
    out = []
    for x in layers:
        if x.ndim == 2:
            out.append(x[..., None])
        else:
            out.append(x)
    return np.concatenate(out, axis=2).astype(np.uint8)


def crop_birdview_slices(dx: int = 0, dy: int = 0) -> tuple[slice, slice]:
    """Row/col slices for 192×192 LBC crop (train_image_phase2 / models.common)."""
    x = 260 - CROP_SIZE // 2 + dx
    y = MAP_SIZE // 2 + dy
    half = CROP_SIZE // 2
    return slice(x - half, x + half), slice(y - half, y + half)


def crop_birdview(birdview: np.ndarray, dx: int = 0, dy: int = 0) -> np.ndarray:
    """Crop 192x192 around ego per LBC train_image_phase2 / models.common."""
    row_sl, col_sl = crop_birdview_slices(dx, dy)
    return birdview[row_sl, col_sl].copy()


def crop_visualization_from_full(
    vis_full: np.ndarray, dx: int = 0, dy: int = 0
) -> np.ndarray:
    """Reuse full 320 RGB visualization for 192 crop (avoids second colorize pass)."""
    row_sl, col_sl = crop_birdview_slices(dx, dy)
    return vis_full[row_sl, col_sl]


def visualize_birdview(birdview: np.ndarray) -> np.ndarray:
    """
    0 road, 1 lane, 2 red, 3 yellow, 4 green, 5 vehicle, 6 pedestrian.
    Same as LearningByCheating bird_view/utils/carla_utils.py (no ego layer).
    """
    h, w = birdview.shape[:2]
    canvas = np.zeros((h, w, 3), dtype=np.uint8)
    canvas[...] = BACKGROUND
    n = min(len(COLORS), birdview.shape[2] if birdview.ndim == 3 else 0)
    for i in range(n):
        canvas[birdview[:, :, i] > 0] = COLORS[i]
    return canvas


def get_birdview(observations):
    traffic = observations['traffic']
    if traffic.ndim == 3 and traffic.shape[-1] == 3:
        tr, ty, tg = traffic[...,0], traffic[...,1], traffic[...,2]
    else:
        tr, ty, tg = observations.get('traffic_red', traffic), observations.get('traffic_yellow', traffic), observations.get('traffic_green', traffic)
    return stack_birdview(observations['road'], observations['lane'], tr, ty, tg, observations['vehicle'], observations['pedestrian'])
