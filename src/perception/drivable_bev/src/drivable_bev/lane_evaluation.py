"""Metric helpers shared by lane-vector offline audits."""

from __future__ import annotations

import numpy as np


def nearest_distances(source_xy, target_xy):
    source = np.asarray(source_xy, dtype=np.float64).reshape(-1, 2)
    target = np.asarray(target_xy, dtype=np.float64).reshape(-1, 2)
    if not len(source):
        return np.empty(0, dtype=np.float64)
    if not len(target):
        return np.full(len(source), np.inf, dtype=np.float64)
    result = np.empty(len(source), dtype=np.float64)
    chunk = 2048
    for start in range(0, len(source), chunk):
        delta = source[start : start + chunk, None, :] - target[None, :, :]
        result[start : start + chunk] = np.sqrt(
            np.min(np.sum(delta * delta, axis=2), axis=1)
        )
    return result


def tolerance_metrics(prediction_xy, target_xy, tolerance_m):
    if tolerance_m <= 0.0:
        raise ValueError("metric tolerance must be positive")
    prediction = np.asarray(prediction_xy, dtype=np.float64).reshape(-1, 2)
    target = np.asarray(target_xy, dtype=np.float64).reshape(-1, 2)
    prediction_distance = nearest_distances(prediction, target)
    target_distance = nearest_distances(target, prediction)
    precision = float(np.mean(prediction_distance <= tolerance_m)) if len(prediction) else 0.0
    recall = float(np.mean(target_distance <= tolerance_m)) if len(target) else 0.0
    f1 = 2.0 * precision * recall / max(precision + recall, np.finfo(float).eps)
    finite = prediction_distance[np.isfinite(prediction_distance)]
    return {
        "tolerance_m": float(tolerance_m),
        "precision": precision,
        "recall": recall,
        "f1": f1,
        "prediction_count": int(len(prediction)),
        "target_count": int(len(target)),
        "chamfer_p50_m": float(np.percentile(finite, 50)) if len(finite) else None,
        "chamfer_p95_m": float(np.percentile(finite, 95)) if len(finite) else None,
    }


def points_in_bounds(points_xy, x_min, x_max, y_min, y_max):
    points = np.asarray(points_xy, dtype=np.float64).reshape(-1, 2)
    selected = (
        (points[:, 0] >= float(x_min))
        & (points[:, 0] <= float(x_max))
        & (points[:, 1] >= float(y_min))
        & (points[:, 1] <= float(y_max))
    )
    return np.ascontiguousarray(points[selected])
