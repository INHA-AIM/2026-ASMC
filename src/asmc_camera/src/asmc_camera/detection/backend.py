"""Detector-agnostic detection types and result filtering."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, List, Mapping, Protocol, Set, Tuple


@dataclass(frozen=True)
class Detection:
    """One box in original-image pixels with the detector's raw label."""

    xmin: float
    ymin: float
    xmax: float
    ymax: float
    source_class_name: str
    confidence: float


class DetectorBackend(Protocol):
    """Minimal backend boundary; ROS and image decoding belong to the caller."""

    info: Mapping[str, object]

    def detect(self, image_bgr) -> Tuple[List[Detection], Mapping[str, float]]:
        ...


def allowed_class_ids(names: Mapping[int, str], allowed_names: Iterable[str]) -> Set[int]:
    """Model class ids whose names match ``allowed_names`` case-insensitively."""
    allowed = {str(name).strip().lower() for name in allowed_names}
    return {int(cid) for cid, name in names.items() if str(name).lower() in allowed}


def filter_result_boxes(
    results: Iterable[object],
    names: Mapping[int, str],
    allowed_ids: Set[int],
    allowed_names: Iterable[str],
    conf_thres: float,
) -> List[Detection]:
    """Convert Ultralytics results to detections exactly like Main/YOLODetection.py.

    The model order is kept. When no class id matched ``allowed_names`` the
    name filter is applied here instead, mirroring the original fallback for
    checkpoints whose names differ in spelling.
    """
    allowed = {str(name).strip().lower() for name in allowed_names}
    detections: List[Detection] = []
    for result in results:
        boxes = getattr(result, "boxes", None)
        if boxes is None or len(boxes) == 0:
            continue
        for box in boxes:
            confidence = float(box.conf[0])
            class_id = int(box.cls[0])
            name = str(names.get(class_id, str(class_id)))
            if not allowed_ids and name.lower() not in allowed:
                continue
            if confidence < conf_thres:
                continue
            x1, y1, x2, y2 = (float(value) for value in box.xyxy[0])
            detections.append(Detection(x1, y1, x2, y2, name, confidence))
    return detections
