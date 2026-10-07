"""Conservative front-camera ego-hood candidate filter, after YOLO inference."""
import hashlib
import math
from pathlib import Path

import yaml


class EgoHoodFilter:
    def __init__(self, enabled=False, min_width_ratio=.60, min_center_y_ratio=.85,
                 min_bottom_ratio=.95, config_sha256=""):
        self.enabled = bool(enabled)
        self.config_sha256 = config_sha256
        self.thresholds = dict(min_width_ratio=min_width_ratio,
                               min_center_y_ratio=min_center_y_ratio,
                               min_bottom_ratio=min_bottom_ratio)
        for name, value in self.thresholds.items():
            if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or not 0 < value <= 1:
                raise ValueError("ego hood %s must be finite and in (0,1]" % name)

    @classmethod
    def load(cls, enabled, config_path):
        if not enabled:
            return cls()
        raw = Path(config_path).read_bytes()
        config = yaml.safe_load(raw)
        required = {"version", "source_class_names", "min_width_ratio", "min_center_y_ratio", "min_bottom_ratio"}
        if not isinstance(config, dict) or set(config) != required or type(config["version"]) is not int or config["version"] != 1:
            raise ValueError("ego hood filter requires version 1 and its five configured fields")
        if config["source_class_names"] != ["car"]:
            raise ValueError("ego hood filter v1 only supports source_class_names: [car]")
        return cls(True, config["min_width_ratio"], config["min_center_y_ratio"],
                   config["min_bottom_ratio"], hashlib.sha256(raw).hexdigest())

    def excludes(self, detection, width, height):
        if not self.enabled or detection.source_class_name.lower() != "car":
            return False
        if width <= 0 or height <= 0:
            raise ValueError("ego hood filter requires positive source dimensions")
        coordinates = (detection.xmin, detection.ymin, detection.xmax, detection.ymax)
        if not all(math.isfinite(v) for v in coordinates):
            return False
        x1, y1, x2, y2 = map(int, coordinates)
        if x2 <= x1 or y2 <= y1:
            return False
        return (x2-x1 >= width*self.thresholds["min_width_ratio"] and
                (y1+y2)*.5 >= height*self.thresholds["min_center_y_ratio"] and
                y2 >= height*self.thresholds["min_bottom_ratio"])

    def partition(self, detections, width, height):
        kept, excluded = [], []
        for detection in detections:
            (excluded if self.excludes(detection, width, height) else kept).append(detection)
        return kept, excluded

    @property
    def info(self):
        return dict(enabled=self.enabled, config_sha256=self.config_sha256,
                    source_class_names=["car"], **self.thresholds)
