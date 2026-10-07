"""One-frame processing, independent of ROS transport and worker scheduling."""
import time

import cv2
import numpy as np

from asmc_camera.detection.messages import build_box_array
from asmc_camera.detection.hood_mask import HoodMask
from asmc_camera.detection.ego_hood_filter import EgoHoodFilter


class ObjectFrameProcessor:
    def __init__(self, backend, groups, decode=cv2.imdecode, hood_mask=None, ego_hood_filter=None):
        self.backend, self.groups, self.decode = backend, groups, decode
        self.hood_mask = hood_mask if hood_mask is not None else HoodMask()
        self.ego_hood_filter = ego_hood_filter if ego_hood_filter is not None else EgoHoodFilter()
        if self.hood_mask.enabled and self.ego_hood_filter.enabled:
            raise ValueError("hood input mask and ego hood post-filter are mutually exclusive")
        self.last_counts = dict(raw=0, kept=0, excluded=0)

    def process(self, header, data):
        message, _, timings, error = self._process(header, data, False)
        return message, timings, error

    def process_with_raw(self, header, data):
        return self._process(header, data, True)

    def _process(self, header, data, include_raw):
        self.last_counts = dict(raw=0, kept=0, excluded=0)
        def invalid(width, height, stage, exc):
            message = build_box_array(header, width, height, False, [], self.groups)
            return message, message if include_raw else None, timings, (stage, str(exc))
        started = time.perf_counter()
        timings = {}
        try:
            image = self.decode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
            if image is None:
                raise ValueError("JPEG decoder returned no image")
        except Exception as exc:
            timings["decode_ms"] = (time.perf_counter() - started) * 1000
            return invalid(0, 0, "decode", exc)
        timings["decode_ms"] = (time.perf_counter() - started) * 1000
        height, width = image.shape[:2]
        mask_started = time.perf_counter()
        try:
            detector_image = self.hood_mask.apply(image)
        except Exception as exc:
            timings["hood_mask_ms"] = (time.perf_counter() - mask_started) * 1000
            return invalid(width, height, "mask", exc)
        timings["hood_mask_ms"] = (time.perf_counter() - mask_started) * 1000
        try:
            detections, backend_timings = self.backend.detect(detector_image)
            detections = list(detections)
            timings.update(backend_timings)
        except Exception as exc:
            return invalid(width, height, "inference", exc)
        raw = build_box_array(header, width, height, True, detections, self.groups) if include_raw else None
        filter_started = time.perf_counter()
        try:
            kept, excluded = self.ego_hood_filter.partition(detections, width, height)
        except Exception as exc:
            timings["ego_hood_filter_ms"] = (time.perf_counter()-filter_started)*1000
            message = build_box_array(header, width, height, False, [], self.groups)
            return message, raw, timings, ("filter", str(exc))
        timings["ego_hood_filter_ms"] = (time.perf_counter()-filter_started)*1000
        self.last_counts = dict(raw=len(detections), kept=len(kept), excluded=len(excluded))
        message = build_box_array(header, width, height, True, kept, self.groups)
        return message, raw, timings, None
