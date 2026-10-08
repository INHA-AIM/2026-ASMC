"""Instance results in original-image pixels, without changing ROS box types."""
from dataclasses import dataclass
import time
from typing import List, Optional

import numpy as np

from .backend import Detection


@dataclass
class SegmentationFrame:
    detections: List[Detection]
    masks: List[Optional[np.ndarray]]
    timings: dict


def load_prompt_embeddings(path, weights_sha256):
    if not path:
        raise ValueError("YOLOE requires offline prompt_embeddings; never download during inference")
    with np.load(path, allow_pickle=False) as data:
        if str(data["weights_sha256"].item()) != weights_sha256:
            raise ValueError("prompt embeddings belong to a different checkpoint")
        names = data["names"].tolist()
        embeddings = data["embeddings"].copy()
    if (not names or len(names) != len(set(names)) or
            any(not isinstance(n, str) or not n.strip() for n in names) or
            embeddings.ndim != 3 or embeddings.shape[:2] != (1, len(names)) or
            not np.isfinite(embeddings).all()):
        raise ValueError("invalid prompt embedding names/shape/values")
    return names, embeddings


def select_result(result, names, allowed_names, confidence, mask_array=None):
    """Keep mask indices synchronized when filtering boxes; no sorting or rounding."""
    allowed = {name.lower() for name in allowed_names}
    boxes = getattr(result, "boxes", None)
    detections, masks = [], []
    if boxes is None:
        return detections, masks
    if mask_array is not None and len(mask_array) != len(boxes):
        raise ValueError("mask/box count mismatch")
    for index, box in enumerate(boxes):
        name = str(names.get(int(box.cls[0]), str(int(box.cls[0]))))
        score = float(box.conf[0])
        if name.lower() not in allowed or not np.isfinite(score) or score < confidence:
            continue
        xyxy = [float(x) for x in box.xyxy[0]]
        if not np.isfinite(xyxy).all():
            raise ValueError("nonfinite box coordinates")
        detections.append(Detection(*xyxy, name, score))
        masks.append(None if mask_array is None else mask_array[index].astype(bool, copy=False))
    return detections, masks


class SegmentationDetector:
    """Use the existing startup contract, with optional CPU mask materialization."""
    def __init__(self, **kwargs):
        from .ultralytics_backend import UltralyticsDetector
        self.backend = UltralyticsDetector(**kwargs)
        self.info = self.backend.info
        self.last_masks = []

    def predict_frame(self, image, include_masks=True):
        b = self.backend
        kwargs = dict(stream=True, verbose=False, conf=b.conf_thres,
                      device=b.device, **b.predict_overrides)
        if b.allowed_ids:
            kwargs["classes"] = sorted(b.allowed_ids)
        b._synchronize()
        started = time.perf_counter()
        results = list(b.model(image, **kwargs))
        b._synchronize()
        timings = {"inference_ms": (time.perf_counter() - started) * 1000}
        detections, masks = [], []
        transfer_started = time.perf_counter()
        for result in results:
            bitmap = None
            if include_masks and getattr(result, "masks", None) is not None:
                from ultralytics.utils.ops import scale_masks
                bitmap = scale_masks(result.masks.data[None], image.shape[:2])[0]
                bitmap = (bitmap > .5).cpu().numpy()
            ds, ms = select_result(result, b.names, b.allowed_names, b.conf_thres, bitmap)
            detections.extend(ds)
            masks.extend(ms)
            for key, value in (getattr(result, "speed", {}) or {}).items():
                timings["model_" + key + "_ms"] = float(value)
        timings["mask_transfer_ms"] = (time.perf_counter() - transfer_started) * 1000
        self.last_masks = masks
        return SegmentationFrame(detections, masks, timings)

    def detect(self, image):
        result = self.predict_frame(image, include_masks=False)
        return result.detections, result.timings
