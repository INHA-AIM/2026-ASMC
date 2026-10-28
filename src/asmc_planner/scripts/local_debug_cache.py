"""Bounded exact-stamp LOCAL-1 raster observer; no planning or projection logic."""

import base64
from collections import OrderedDict
import threading


class LocalDebugCache:
    def __init__(self, capacity=32):
        self.capacity = capacity
        self.lock = threading.Lock()
        self.grids = {"probability": OrderedDict(), "coverage": OrderedDict()}
        self.cameras = {}

    def grid(self, key, stamp, width, height, step, encoding, data):
        if (
            key not in self.grids
            or (width, height, step, encoding) != (160, 250, 160, "mono8")
            or len(data) != 40000
        ):
            return False
        value = base64.b64encode(bytes(data)).decode("ascii")
        with self.lock:
            target = self.grids[key]
            target[int(stamp)] = value
            while len(target) > self.capacity:
                target.popitem(last=False)
        return True

    def camera(self, payload):
        # Camera detections remain in original pixels. They are NOT projected
        # into the road a second time or treated as exact scene-time fusion.
        with self.lock:
            if payload["view"] not in self.cameras and len(self.cameras) >= 3:
                return
            self.cameras[payload["view"]] = payload

    def snapshot(self, stamp):
        with self.lock:
            probability = self.grids["probability"].get(int(stamp))
            coverage = self.grids["coverage"].get(int(stamp))
            cameras = list(self.cameras.values())
        valid = probability is not None and coverage is not None
        return dict(
            stamp_ns=str(stamp),
            raster_available=valid,
            reason="exact_source_stamp" if valid else "matching_raster_not_cached",
            width=160,
            height=250,
            resolution=0.1,
            xmax=20.0,
            ymax=8.0,
            probability=probability if valid else None,
            coverage=coverage if valid else None,
            cameras=cameras,
            camera_alignment="latest original-image evidence; not a planner obstacle",
        )
