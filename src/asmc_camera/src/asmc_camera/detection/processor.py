"""One-frame processing, independent of ROS transport and worker scheduling."""
import time

import cv2
import numpy as np

from asmc_camera.detection.messages import build_box_array


class ObjectFrameProcessor:
    def __init__(self, backend, groups, decode=cv2.imdecode):
        self.backend, self.groups, self.decode = backend, groups, decode

    def process(self, header, data):
        started = time.perf_counter()
        timings = {}
        try:
            image = self.decode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
            if image is None:
                raise ValueError("JPEG decoder returned no image")
        except Exception as exc:
            timings["decode_ms"] = (time.perf_counter() - started) * 1000
            return build_box_array(header, 0, 0, False, [], self.groups), timings, ("decode", str(exc))
        timings["decode_ms"] = (time.perf_counter() - started) * 1000
        height, width = image.shape[:2]
        try:
            detections, backend_timings = self.backend.detect(image)
            timings.update(backend_timings)
            message = build_box_array(header, width, height, True, detections, self.groups)
            return message, timings, None
        except Exception as exc:
            return build_box_array(header, width, height, False, [], self.groups), timings, ("inference", str(exc))
