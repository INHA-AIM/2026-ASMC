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
    categorical_confidences: Mapping[str, np.ndarray]
    model_grid_probabilities: Mapping[str, np.ndarray]
    model_grid_categorical_confidences: Mapping[str, np.ndarray]
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

    def infer(
        self, images_bgr, preserve_model_grid=True, gpu_inputs=None,
    ) -> BatchPrediction:
        stream = getattr(self.backend, 'cuda_stream', None)
        if stream is not None:
            with self.backend._torch.cuda.stream(stream):
                try:
                    return self._infer(images_bgr, preserve_model_grid, gpu_inputs)
                finally:
                    for raw in (gpu_inputs or {}).values():
                        raw.release()
        return self._infer(images_bgr, preserve_model_grid, gpu_inputs)

    def _infer(
        self,
        images_bgr: Mapping[str, np.ndarray],
        preserve_model_grid: bool = True,
        gpu_inputs=None,
    ) -> BatchPrediction:
        if not images_bgr:
            raise ValueError("at least one view is required")

        keys = list(images_bgr)
        groups = {}
        geometries: Dict[str, LetterboxGeometry] = {}
        started = time.perf_counter()
        for key in keys:
            shape = tuple(self.backend.info.input_hw_by_view.get(key, self.backend.info.input_hw))
            if gpu_inputs is not None:
                from asmc_camera.runtime.gpu_input import PreprocessSpec, prepare
                tensor, geometry, _ = prepare(gpu_inputs[key], PreprocessSpec(shape, 'area'))
            else:
                tensor, geometry = letterbox_bgr(images_bgr[key], target_hw=shape)
            groups.setdefault(shape, []).append((key, tensor))
            geometries[key] = geometry
        if gpu_inputs is not None:
            import torch
            stack = torch.stack
        else:
            stack = np.stack
        batches = [([key for key, _ in entries], stack([tensor for _, tensor in entries]))
                   for entries in groups.values()]
        preprocess_ms = (time.perf_counter() - started) * 1000.0

        # One loaded model, one serial worker: batch only equal tensor shapes.
        # Incomplete view sets continue without waiting for a missing camera.
        inference_ms = 0.0
        results = {}
        for group_keys, batch in batches:
            started = time.perf_counter()
            outputs = dict(self.backend.infer(batch))
            inference_ms += (time.perf_counter() - started) * 1000.0
            validate_backend_output(outputs, self.backend.info.heads, batch.shape,
                                    self.backend.info.categorical_classes)
            for index, key in enumerate(group_keys):
                results[key] = (index, outputs)

        started = time.perf_counter()
        predictions: Dict[str, ViewPrediction] = {}
        for key in keys:
            index, outputs = results[key]
            geometry = geometries[key]
            per_head = {}
            categorical_confidences = {}
            model_grid_per_head = {}
            model_grid_categorical_confidences = {}
            for head in self.backend.info.heads:
                output = outputs[head]
                if head in self.backend.info.categorical_classes:
                    class_ids = output.class_ids[index]
                    confidence = output.confidence[index]
                    if preserve_model_grid:
                        model_grid_per_head[head] = class_ids.copy()
                        model_grid_categorical_confidences[head] = confidence.copy()
                    if self.restore_original:
                        class_ids = restore_class_ids(class_ids, geometry)
                        confidence = restore_probability(confidence, geometry)
                    else:
                        class_ids = class_ids.copy()
                        confidence = confidence.copy()
                    per_head[head] = class_ids
                    categorical_confidences[head] = confidence
                else:
                    value = output[index]
                    if preserve_model_grid:
                        model_grid_per_head[head] = value.copy()
                    if self.restore_original:
                        value = restore_probability(value, geometry)
                    else:
                        value = value.copy()
                    per_head[head] = value
            predictions[key] = ViewPrediction(
                probabilities=per_head,
                categorical_confidences=categorical_confidences,
                model_grid_probabilities=model_grid_per_head,
                model_grid_categorical_confidences=(
                    model_grid_categorical_confidences
                ),
                geometry=geometry,
            )
        postprocess_ms = (time.perf_counter() - started) * 1000.0
        return BatchPrediction(
            predictions, preprocess_ms, inference_ms, postprocess_ms
        )
