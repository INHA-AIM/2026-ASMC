"""OpenCV rendering helpers for camera semantic debug images."""

from __future__ import annotations

from typing import Mapping, Sequence

import cv2
import numpy as np


DRIVABLE_COLOR_BGR = (40, 200, 40)
LANE_COLOR_BGR = (0, 230, 255)


def normalize_probability(probability: np.ndarray) -> np.ndarray:
    """Return a contiguous float32 probability image in the [0, 1] range."""
    value = np.asarray(probability)
    if value.ndim != 2:
        raise ValueError("probability must be a two-dimensional array")
    if value.dtype == np.uint8:
        value = value.astype(np.float32) / 255.0
    else:
        value = value.astype(np.float32, copy=False)
    return np.ascontiguousarray(np.clip(value, 0.0, 1.0))


def hard_mask(probability: np.ndarray, threshold: float) -> np.ndarray:
    if not 0.0 <= threshold <= 1.0:
        raise ValueError("threshold must be between 0 and 1")
    return normalize_probability(probability) >= threshold


def render_overlay(
    image_bgr: np.ndarray,
    drivable_probability: np.ndarray,
    lane_probability: np.ndarray,
    drivable_threshold: float = 0.5,
    lane_threshold: float = 0.5,
    alpha: float = 0.45,
) -> np.ndarray:
    """Overlay thresholded drivable and lane predictions on a BGR image."""
    _validate_image(image_bgr)
    if not 0.0 <= alpha <= 1.0:
        raise ValueError("alpha must be between 0 and 1")
    drivable = hard_mask(drivable_probability, drivable_threshold)
    lane = hard_mask(lane_probability, lane_threshold)
    _validate_spatial_shape(image_bgr, drivable, "drivable")
    _validate_spatial_shape(image_bgr, lane, "lane")

    overlay = image_bgr.copy()
    color_layer = image_bgr.copy()
    color_layer[drivable] = DRIVABLE_COLOR_BGR
    color_layer[lane] = LANE_COLOR_BGR
    active = drivable | lane
    blended = cv2.addWeighted(image_bgr, 1.0 - alpha, color_layer, alpha, 0.0)
    overlay[active] = blended[active]
    return overlay


def render_panel(
    image_bgr: np.ndarray,
    drivable_probability: np.ndarray,
    lane_probability: np.ndarray,
    view: str,
    stamp_text: str = "",
    condition_label: str = "",
    drivable_threshold: float = 0.5,
    lane_threshold: float = 0.5,
    alpha: float = 0.45,
    panel_width: int = 1280,
) -> np.ndarray:
    """Render original, overlay, drivable and lane as a 2-by-2 panel."""
    _validate_image(image_bgr)
    if panel_width < 320:
        raise ValueError("panel_width must be at least 320")

    drivable = hard_mask(drivable_probability, drivable_threshold)
    lane = hard_mask(lane_probability, lane_threshold)
    _validate_spatial_shape(image_bgr, drivable, "drivable")
    _validate_spatial_shape(image_bgr, lane, "lane")
    overlay = render_overlay(
        image_bgr,
        drivable_probability,
        lane_probability,
        drivable_threshold,
        lane_threshold,
        alpha,
    )

    drivable_image = np.zeros_like(image_bgr)
    drivable_image[drivable] = DRIVABLE_COLOR_BGR
    lane_image = np.zeros_like(image_bgr)
    lane_image[lane] = LANE_COLOR_BGR

    tile_width = panel_width // 2
    tile_height = max(1, int(round(image_bgr.shape[0] * tile_width / image_bgr.shape[1])))
    title_parts = [str(view).upper()]
    if condition_label:
        title_parts.append(condition_label)
    if stamp_text:
        title_parts.append(stamp_text)
    context = " | ".join(title_parts)
    tiles = (
        _label_tile(image_bgr, "ORIGINAL | " + context, tile_width, tile_height),
        _label_tile(overlay, "OVERLAY | " + context, tile_width, tile_height),
        _label_tile(
            drivable_image,
            "DRIVABLE >= {:.2f}".format(drivable_threshold),
            tile_width,
            tile_height,
        ),
        _label_tile(
            lane_image,
            "LANE >= {:.2f}".format(lane_threshold),
            tile_width,
            tile_height,
        ),
    )
    top = np.concatenate(tiles[:2], axis=1)
    bottom = np.concatenate(tiles[2:], axis=1)
    return np.ascontiguousarray(np.concatenate((top, bottom), axis=0))


def render_mosaic(
    panels: Mapping[str, np.ndarray],
    views: Sequence[str],
    per_view_width: int = 640,
) -> np.ndarray:
    """Place the latest view panels side by side in the requested view order."""
    if per_view_width < 160:
        raise ValueError("per_view_width must be at least 160")
    if not views:
        raise ValueError("views must not be empty")
    missing = [view for view in views if view not in panels]
    if missing:
        raise ValueError("missing panels for views: {}".format(missing))

    resized = []
    for view in views:
        panel = panels[view]
        _validate_image(panel)
        height = max(1, int(round(panel.shape[0] * per_view_width / panel.shape[1])))
        resized.append(
            cv2.resize(panel, (per_view_width, height), interpolation=cv2.INTER_AREA)
        )
    target_height = min(panel.shape[0] for panel in resized)
    aligned = [
        panel
        if panel.shape[0] == target_height
        else cv2.resize(panel, (per_view_width, target_height), interpolation=cv2.INTER_AREA)
        for panel in resized
    ]
    return np.ascontiguousarray(np.concatenate(aligned, axis=1))


def _label_tile(
    image: np.ndarray, label: str, width: int, height: int
) -> np.ndarray:
    interpolation = cv2.INTER_AREA if width < image.shape[1] else cv2.INTER_LINEAR
    tile = cv2.resize(image, (width, height), interpolation=interpolation)
    bar_height = max(24, min(44, height // 10))
    cv2.rectangle(tile, (0, 0), (width, bar_height), (0, 0, 0), -1)
    font_scale = max(0.42, min(0.75, width / 900.0))
    cv2.putText(
        tile,
        label,
        (10, int(bar_height * 0.72)),
        cv2.FONT_HERSHEY_SIMPLEX,
        font_scale,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    return tile


def _validate_image(image: np.ndarray) -> None:
    if not isinstance(image, np.ndarray) or image.ndim != 3 or image.shape[2] != 3:
        raise ValueError("image_bgr must have shape (height, width, 3)")
    if image.dtype != np.uint8:
        raise ValueError("image_bgr must use uint8 pixels")


def _validate_spatial_shape(
    image: np.ndarray, probability: np.ndarray, name: str
) -> None:
    if probability.shape != image.shape[:2]:
        raise ValueError(
            "{} shape {} does not match image shape {}".format(
                name, probability.shape, image.shape[:2]
            )
        )
