"""Build asmc_msgs/CameraObjectBoxArray from detections."""

from __future__ import annotations

from typing import Iterable

from asmc_msgs.msg import CameraObjectBox, CameraObjectBoxArray

from asmc_camera.detection.backend import Detection
from asmc_camera.detection.classes import ClassGroups


def build_box_array(
    header,
    image_width: int,
    image_height: int,
    valid: bool,
    detections: Iterable[Detection],
    groups: ClassGroups,
) -> CameraObjectBoxArray:
    """Keep detector order; drop labels whose group is excluded."""
    message = CameraObjectBoxArray()
    message.header = header
    message.image_width = int(image_width)
    message.image_height = int(image_height)
    message.valid = bool(valid)
    if not valid:
        return message
    for detection in detections:
        group = groups.group_of(detection.source_class_name)
        if group is None:
            continue
        box = CameraObjectBox()
        box.xmin = detection.xmin
        box.ymin = detection.ymin
        box.xmax = detection.xmax
        box.ymax = detection.ymax
        box.class_name = group
        box.source_class_name = detection.source_class_name
        box.confidence = detection.confidence
        box.confidence_valid = True
        message.boxes.append(box)
    return message
