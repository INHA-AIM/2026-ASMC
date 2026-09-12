"""Pure helpers for offline model-grid and soft camera-BEV evaluation.

This module deliberately has no ROS dependency.  It lets the offline audit
measure the effect of letterbox restoration and categorical argmax without
changing the deployment-facing projector or topic contract.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping, Optional

import cv2
import numpy as np

from camera_semantic_perception.geometry import LetterboxGeometry

from .projector import CameraBevProjector, ProjectedSemantic


@dataclass(frozen=True)
class ProjectedProbabilities:
    """Floating-point semantic evidence aligned to one metric BEV grid."""

    drivable_probability: np.ndarray
    road_marking_probabilities: np.ndarray
    coverage: np.ndarray


@dataclass(frozen=True)
class FusedProbabilities:
    """Quality-weighted multi-camera probability fusion result."""

    drivable_probability: np.ndarray
    road_marking_probabilities: np.ndarray
    road_marking_class_id: np.ndarray
    road_marking_confidence: np.ndarray
    coverage: np.ndarray
    source_count: np.ndarray


def native_from_model_homography(geometry: LetterboxGeometry) -> np.ndarray:
    """Return a model-grid pixel to native-image pixel homography."""
    scale_x, scale_y = (float(value) for value in geometry.scale_xy)
    left, top, _, _ = (int(value) for value in geometry.pad_ltrb)
    if scale_x <= 0.0 or scale_y <= 0.0:
        raise ValueError("letterbox scales must be positive")
    return np.asarray(
        [
            [1.0 / scale_x, 0.0, -float(left) / scale_x],
            [0.0, 1.0 / scale_y, -float(top) / scale_y],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def bev_from_model_homography(
    projector: CameraBevProjector, geometry: LetterboxGeometry
) -> np.ndarray:
    """Compose letterbox removal and the calibrated native-image IPM."""
    expected_native = (projector.camera.height, projector.camera.width)
    if tuple(geometry.original_hw) != expected_native:
        raise ValueError(
            "letterbox native shape {} differs from camera {}".format(
                geometry.original_hw, expected_native
            )
        )
    value = (
        projector.homography.bev_from_image
        @ native_from_model_homography(geometry)
    )
    value /= value[2, 2]
    return np.ascontiguousarray(value, dtype=np.float64)


def letterbox_valid_mask(geometry: LetterboxGeometry) -> np.ndarray:
    """Return the unpadded region of a model-grid tensor as uint8."""
    height, width = geometry.target_hw
    left, top, _, _ = geometry.pad_ltrb
    resized_h, resized_w = geometry.resized_hw
    result = np.zeros((height, width), dtype=np.uint8)
    result[top : top + resized_h, left : left + resized_w] = 255
    return result


def _warp(
    source: np.ndarray,
    transform: np.ndarray,
    shape,
    interpolation: int,
) -> np.ndarray:
    height, width = (int(value) for value in shape)
    return cv2.warpPerspective(
        source,
        transform,
        (width, height),
        flags=interpolation,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=0,
    )


def _validate_model_array(value, geometry, name, ndim=2):
    array = np.asarray(value)
    expected = tuple(geometry.target_hw)
    if array.ndim != ndim or tuple(array.shape[-2:]) != expected:
        raise ValueError(
            "{} must end with model-grid shape {}, got {}".format(
                name, expected, array.shape
            )
        )
    return array


def project_model_grid_hard(
    projector: CameraBevProjector,
    geometry: LetterboxGeometry,
    drivable_probability: np.ndarray,
    road_marking_class_id: np.ndarray,
    road_marking_confidence: Optional[np.ndarray] = None,
    valid_mask: Optional[np.ndarray] = None,
) -> ProjectedSemantic:
    """Project model-grid hard outputs without an intermediate native resize."""
    drivable = _validate_model_array(
        drivable_probability, geometry, "drivable probability"
    )
    classes = _validate_model_array(
        road_marking_class_id, geometry, "road-marking class"
    )
    if np.any(classes < 0) or np.any(classes > 3):
        raise ValueError("road-marking class IDs must lie in [0,3]")
    confidence = (
        np.ones(classes.shape, dtype=np.float32)
        if road_marking_confidence is None
        else _validate_model_array(
            road_marking_confidence, geometry, "road-marking confidence"
        )
    )
    source_valid = (
        letterbox_valid_mask(geometry)
        if valid_mask is None
        else (_validate_model_array(valid_mask, geometry, "valid mask") > 0).astype(
            np.uint8
        )
        * 255
    )
    transform = bev_from_model_homography(projector, geometry)
    shape = projector.grid.shape
    coverage = _warp(source_valid, transform, shape, cv2.INTER_NEAREST)
    coverage[(projector.homography.coverage == 0) | (coverage == 0)] = 0
    covered = coverage > 0

    projected_drivable = _warp(
        np.asarray(drivable, dtype=np.float32), transform, shape, cv2.INTER_LINEAR
    )
    projected_classes = _warp(
        np.asarray(classes, dtype=np.uint8), transform, shape, cv2.INTER_NEAREST
    )
    projected_confidence = _warp(
        np.asarray(confidence, dtype=np.float32), transform, shape, cv2.INTER_LINEAR
    )
    projected_drivable[~covered] = 0.0
    projected_classes[~covered] = 0
    projected_confidence[~covered] = 0.0
    return ProjectedSemantic(
        drivable_probability=np.rint(
            np.clip(projected_drivable, 0.0, 1.0) * 255.0
        ).astype(np.uint8),
        road_marking_class_id=np.ascontiguousarray(projected_classes, dtype=np.uint8),
        road_marking_confidence=np.rint(
            np.clip(projected_confidence, 0.0, 1.0) * 255.0
        ).astype(np.uint8),
        coverage=np.ascontiguousarray(coverage, dtype=np.uint8),
    )


def project_model_grid_probabilities(
    projector: CameraBevProjector,
    geometry: LetterboxGeometry,
    drivable_probability: np.ndarray,
    road_marking_probabilities: np.ndarray,
    valid_mask: Optional[np.ndarray] = None,
) -> ProjectedProbabilities:
    """Linearly project full categorical evidence and renormalize in BEV."""
    drivable = _validate_model_array(
        drivable_probability, geometry, "drivable probability"
    ).astype(np.float32, copy=False)
    marking = _validate_model_array(
        road_marking_probabilities,
        geometry,
        "road-marking probabilities",
        ndim=3,
    ).astype(np.float32, copy=False)
    if marking.shape[0] != 4:
        raise ValueError("road-marking probabilities must have four channels")
    if (
        not np.isfinite(drivable).all()
        or not np.isfinite(marking).all()
        or np.any(drivable < 0.0)
        or np.any(drivable > 1.0)
        or np.any(marking < 0.0)
        or np.any(marking > 1.0)
    ):
        raise ValueError("probabilities must be finite values in [0,1]")

    source_valid = (
        letterbox_valid_mask(geometry)
        if valid_mask is None
        else (_validate_model_array(valid_mask, geometry, "valid mask") > 0).astype(
            np.uint8
        )
        * 255
    )
    transform = bev_from_model_homography(projector, geometry)
    shape = projector.grid.shape
    coverage = _warp(source_valid, transform, shape, cv2.INTER_NEAREST)
    coverage[(projector.homography.coverage == 0) | (coverage == 0)] = 0
    covered = coverage > 0

    projected_drivable = _warp(drivable, transform, shape, cv2.INTER_LINEAR)
    projected_marking = np.stack(
        [
            _warp(channel, transform, shape, cv2.INTER_LINEAR)
            for channel in marking
        ],
        axis=0,
    ).astype(np.float32, copy=False)
    projected_drivable[~covered] = 0.0
    projected_marking[:, ~covered] = 0.0
    total = projected_marking.sum(axis=0)
    probability_valid = covered & (total > np.finfo(np.float32).eps)
    projected_marking[:, probability_valid] /= total[probability_valid]
    projected_marking[:, ~probability_valid] = 0.0
    coverage[~probability_valid] = 0
    if np.any(probability_valid) and not np.allclose(
        projected_marking[:, probability_valid].sum(axis=0), 1.0, atol=1e-5
    ):
        raise RuntimeError("projected road-marking probabilities are not normalized")
    if np.any(projected_marking[:, ~probability_valid] != 0.0):
        raise RuntimeError("projected probability leaked outside coverage")
    return ProjectedProbabilities(
        drivable_probability=np.ascontiguousarray(
            np.clip(projected_drivable, 0.0, 1.0), dtype=np.float32
        ),
        road_marking_probabilities=np.ascontiguousarray(
            projected_marking, dtype=np.float32
        ),
        coverage=np.ascontiguousarray(coverage, dtype=np.uint8),
    )


def fuse_probabilities(
    projected_by_view: Mapping[str, ProjectedProbabilities],
    quality_by_view: Mapping[str, np.ndarray],
) -> FusedProbabilities:
    """Fuse full probabilities using the same static geometric quality maps."""
    if not projected_by_view or set(projected_by_view) != set(quality_by_view):
        raise ValueError("projected and quality views must be identical and non-empty")
    views = tuple(projected_by_view)
    shape = np.asarray(quality_by_view[views[0]]).shape
    weights = []
    drivable = []
    marking = []
    for view in views:
        value = projected_by_view[view]
        quality = np.asarray(quality_by_view[view], dtype=np.float32)
        if quality.shape != shape or value.coverage.shape != shape:
            raise ValueError("all projected and quality grids must have one shape")
        if value.drivable_probability.shape != shape:
            raise ValueError("drivable probability shape differs from BEV grid")
        if value.road_marking_probabilities.shape != (4, *shape):
            raise ValueError("road-marking probability shape differs from BEV grid")
        weight = quality.copy()
        weight[value.coverage == 0] = 0.0
        weights.append(weight)
        drivable.append(value.drivable_probability)
        marking.append(value.road_marking_probabilities)

    weight_stack = np.stack(weights)
    weight_sum = weight_stack.sum(axis=0)
    covered = weight_sum > 0.0
    fused_drivable = np.zeros(shape, dtype=np.float32)
    fused_marking = np.zeros((4, *shape), dtype=np.float32)
    fused_drivable[covered] = (
        np.stack(drivable) * weight_stack
    ).sum(axis=0)[covered] / weight_sum[covered]
    fused_marking[:, covered] = (
        np.stack(marking) * weight_stack[:, None, :, :]
    ).sum(axis=0)[:, covered] / weight_sum[covered]
    total = fused_marking.sum(axis=0)
    probability_valid = covered & (total > np.finfo(np.float32).eps)
    fused_marking[:, probability_valid] /= total[probability_valid]
    fused_marking[:, ~probability_valid] = 0.0
    fused_drivable[~probability_valid] = 0.0
    if np.any(probability_valid) and not np.allclose(
        fused_marking[:, probability_valid].sum(axis=0), 1.0, atol=1e-5
    ):
        raise RuntimeError("fused road-marking probabilities are not normalized")
    if np.any(fused_marking[:, ~probability_valid] != 0.0):
        raise RuntimeError("fused probability leaked outside coverage")

    class_id = np.argmax(fused_marking, axis=0).astype(np.uint8)
    confidence = np.max(fused_marking, axis=0).astype(np.float32)
    class_id[~probability_valid] = 0
    confidence[~probability_valid] = 0.0
    return FusedProbabilities(
        drivable_probability=np.ascontiguousarray(fused_drivable),
        road_marking_probabilities=np.ascontiguousarray(fused_marking),
        road_marking_class_id=np.ascontiguousarray(class_id),
        road_marking_confidence=np.ascontiguousarray(confidence),
        coverage=np.ascontiguousarray(
            probability_valid.astype(np.uint8) * 255, dtype=np.uint8
        ),
        source_count=np.ascontiguousarray(
            np.count_nonzero(weight_stack > 0.0, axis=0).astype(np.uint8)
        ),
    )
