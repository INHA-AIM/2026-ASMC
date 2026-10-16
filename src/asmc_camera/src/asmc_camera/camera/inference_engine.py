"""Model-independent preprocessing, batching, and probability restoration."""

from __future__ import annotations

import time
from dataclasses import dataclass, field
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
    backend_stages: Mapping = field(default_factory=dict)


class InferenceEngine:
    def __init__(self, backend: SemanticBackend, restore_original: bool = True,
                 resize_interpolation: str = "auto"):
        if resize_interpolation not in ("auto", "linear"):
            raise ValueError("resize_interpolation must be auto or linear")
        self.backend = backend
        self.restore_original = bool(restore_original)
        self.resize_interpolation = resize_interpolation

    def infer(
        self, images_bgr, preserve_model_grid=True, gpu_inputs=None,
        native_outputs=None,
    ) -> BatchPrediction:
        stream = getattr(self.backend, 'cuda_stream', None)
        if stream is not None:
            with self.backend._torch.cuda.stream(stream):
                try:
                    return self._infer(images_bgr, preserve_model_grid, gpu_inputs, native_outputs)
                finally:
                    for raw in (gpu_inputs or {}).values():
                        raw.release()
        return self._infer(images_bgr, preserve_model_grid, gpu_inputs, native_outputs)

    def _infer(
        self,
        images_bgr: Mapping[str, np.ndarray],
        preserve_model_grid: bool = True,
        gpu_inputs=None,
        native_outputs=None,
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
                if self.resize_interpolation != "auto":
                    raise ValueError("explicit linear resize requires the CPU preprocessing path")
                tensor, geometry, _ = prepare(gpu_inputs[key], PreprocessSpec(shape, 'area'))
            else:
                tensor, geometry = letterbox_bgr(images_bgr[key], target_hw=shape,
                                                interpolation=self.resize_interpolation)
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
        backend_stages = {}
        for group_keys, batch in batches:
            started = time.perf_counter()
            outputs = dict(self.backend.infer(batch))
            inference_ms += (time.perf_counter() - started) * 1000.0
            backend_stages[','.join(group_keys)] = dict(getattr(self.backend, 'timings', {}))
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
                requested = None if native_outputs is None else native_outputs.get(key, set())
                want_native = requested is None or head in requested
                want_confidence = requested is None or head + '_confidence' in requested
                want_grid = preserve_model_grid and (requested is None or head == 'road_marking')
                if head in self.backend.info.categorical_classes:
                    class_ids = output.class_ids[index]
                    confidence = output.confidence[index]
                    if want_grid:
                        model_grid_per_head[head] = class_ids.copy()
                        model_grid_categorical_confidences[head] = confidence.copy()
                    if want_native and self.restore_original:
                        class_ids = restore_class_ids(class_ids, geometry)
                    elif want_native:
                        class_ids = class_ids.copy()
                    if want_confidence and self.restore_original:
                        confidence = restore_probability(confidence, geometry)
                    elif want_confidence:
                        confidence = confidence.copy()
                    if want_native:
                        per_head[head] = class_ids
                    if want_confidence:
                        categorical_confidences[head] = confidence
                else:
                    value = output[index]
                    if want_grid:
                        model_grid_per_head[head] = value.copy()
                    if not want_native:
                        continue
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
            predictions, preprocess_ms, inference_ms, postprocess_ms, backend_stages
        )
