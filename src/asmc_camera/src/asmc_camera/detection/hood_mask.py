"""YOLO-only exclusion, preserving the source image and its coordinate system."""
import hashlib
import math
from pathlib import Path
import threading

import cv2
import numpy as np
import yaml


class HoodMask:
    def __init__(self, polygon=None, config_sha256=""):
        self.enabled = polygon is not None
        self.config_sha256 = config_sha256
        self.polygon = self._validate(polygon) if self.enabled else ()
        self._cache = {}
        self._lock = threading.Lock()
        self._last_geometry = {}

    @staticmethod
    def _validate(polygon):
        try:
            points = tuple(tuple(float(v) for v in p) for p in polygon)
        except (TypeError, ValueError) as exc:
            raise ValueError("hood mask polygon must contain normalized xy pairs") from exc
        if len(points) < 3 or any(len(p) != 2 for p in points):
            raise ValueError("hood mask polygon requires at least three xy pairs")
        if any(not math.isfinite(v) or not 0 <= v <= 1 for p in points for v in p):
            raise ValueError("hood mask coordinates must be finite and in [0,1]")
        if len(set(points)) != len(points):
            raise ValueError("hood mask polygon has repeated vertices")
        def cross(a, b, c):
            return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
        def intersects(a, b, c, d):
            if max(a[0], b[0]) < min(c[0], d[0]) or max(c[0], d[0]) < min(a[0], b[0]):
                return False
            if max(a[1], b[1]) < min(c[1], d[1]) or max(c[1], d[1]) < min(a[1], b[1]):
                return False
            return cross(a,b,c)*cross(a,b,d) <= 0 and cross(c,d,a)*cross(c,d,b) <= 0
        for i in range(len(points)):
            for j in range(i+1, len(points)):
                if j == i+1 or (i == 0 and j == len(points)-1):
                    continue
                if intersects(points[i], points[(i+1)%len(points)], points[j], points[(j+1)%len(points)]):
                    raise ValueError("hood mask polygon self-intersects")
        area = abs(sum(a[0]*b[1]-b[0]*a[1] for a,b in zip(points,points[1:]+points[:1]))) / 2
        if not 0 < area < 1:
            raise ValueError("hood mask must exclude a nonempty proper subset of the image")
        return points

    @classmethod
    def load(cls, enabled, config_path):
        if not enabled:
            return cls()
        raw = Path(config_path).read_bytes()
        config = yaml.safe_load(raw)
        if not isinstance(config, dict) or config.get("version") != 1 or set(config) != {"version", "polygon"}:
            raise ValueError("hood mask config requires version: 1 and polygon only")
        return cls(config["polygon"], hashlib.sha256(raw).hexdigest())

    def geometry(self, height, width):
        if height < 1 or width < 1:
            raise ValueError("hood mask requires positive image dimensions")
        key = (height, width)
        with self._lock:
            if key not in self._cache:
                pixels = np.array([(min(width-1, int(x*width)), min(height-1, int(y*height)))
                                   for x,y in self.polygon], dtype=np.int32)
                bitmap = np.zeros((height,width), dtype=np.uint8)
                cv2.fillPoly(bitmap, [pixels], 1)
                excluded = int(np.count_nonzero(bitmap))
                if excluded == 0 or excluded == height*width:
                    raise ValueError("hood mask raster leaves no excluded or observable pixels")
                bitmap = bitmap.astype(bool)
                bitmap.setflags(write=False)
                pixels.setflags(write=False)
                visible = np.logical_not(bitmap).astype(np.uint8)*255
                visible.setflags(write=False)
                self._cache[key] = (bitmap, pixels, excluded, visible)
                # Bounding memory for malformed streams that change shape repeatedly.
                if len(self._cache) > 4:
                    del self._cache[next(iter(self._cache))]
            bitmap, pixels, excluded, _ = self._cache[key]
            self._last_geometry = dict(image_width=width, image_height=height,
                                       polygon_pixels=pixels.tolist(), excluded_pixels=excluded)
            return bitmap, pixels

    def apply(self, image):
        if not self.enabled:
            return image
        if image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8:
            raise ValueError("hood mask expects a decoded uint8 BGR image")
        self.geometry(*image.shape[:2])
        with self._lock:
            visible = self._cache[image.shape[:2]][3]
        return cv2.copyTo(image, visible)

    @property
    def info(self):
        with self._lock:
            return dict(enabled=self.enabled, config_sha256=self.config_sha256,
                        polygon_normalized=[list(p) for p in self.polygon], **self._last_geometry)
