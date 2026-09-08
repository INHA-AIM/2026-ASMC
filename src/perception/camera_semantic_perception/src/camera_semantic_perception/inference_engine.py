"""Model-independent preprocessing, batching, and probability restoration."""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Dict, Mapping

import numpy as np

from .backend import SemanticBackend, validate_backend_output
from .geometry import (
    LetterboxGeometry,
    letterbox_bgr,
    restore_class_ids,
    restore_probability,
)


@dataclass(frozen=True)
class ViewPrediction:
    probabilities: Mapping[str, np.ndarray]
    geometry: LetterboxGeometry


@dataclass(frozen=True)
class BatchPrediction:
    views: Mapping[str, ViewPrediction]
    preprocess_ms: float
    inference_ms: float
    postprocess_ms: float


class InferenceEngine:
    def __init__(self, backend: SemanticBackend, restore_original: bool = True):
        self.backend = backend
        self.restore_original = bool(restore_original)

    def infer(self, images_bgr: Mapping[str, np.ndarray]) -> BatchPrediction:
        if not images_bgr:
            raise ValueError("at least one view is required")

        keys = list(images_bgr)
        tensors = []
        geometries: Dict[str, LetterboxGeometry] = {}
        started = time.perf_counter()
        for key in keys:
            tensor, geometry = letterbox_bgr(
                images_bgr[key], target_hw=self.backend.info.input_hw
            )
            tensors.append(tensor)
            geometries[key] = geometry
        batch = np.stack(tensors, axis=0)
        preprocess_ms = (time.perf_counter() - started) * 1000.0

        started = time.perf_counter()
        outputs = dict(self.backend.infer(batch))
        inference_ms = (time.perf_counter() - started) * 1000.0
        validate_backend_output(
            outputs,
            self.backend.info.heads,
            batch.shape,
            self.backend.info.categorical_classes,
        )

        started = time.perf_counter()
        predictions: Dict[str, ViewPrediction] = {}
        for index, key in enumerate(keys):
            geometry = geometries[key]
            per_head = {}
            for head in self.backend.info.heads:
                value = outputs[head][index]
                if self.restore_original:
                    if head in self.backend.info.categorical_classes:
                        value = restore_class_ids(value, geometry)
                    else:
                        value = restore_probability(value, geometry)
                else:
                    value = value.copy()
                per_head[head] = value
            predictions[key] = ViewPrediction(per_head, geometry)
        postprocess_ms = (time.perf_counter() - started) * 1000.0
        return BatchPrediction(
            predictions, preprocess_ms, inference_ms, postprocess_ms
        )
