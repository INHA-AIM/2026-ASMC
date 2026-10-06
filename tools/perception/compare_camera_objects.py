#!/usr/bin/env python3
"""Run every bag frame through the original loop and new backend + wire adapter.

Requires both catkin workspaces sourced and a CUDA inference environment.
The legacy method is executed unchanged, once per decoded frame; its ROS
scheduler/publisher are replaced only to permit deterministic offline input.
"""
import argparse
import importlib.util
import io
import json
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import rosbag
import rospy
import torch
import ultralytics
import yaml
from ultralytics import YOLO

from asmc_msgs.msg import CameraObjectBoxArray
from asmc_camera.detection.classes import load_class_groups
from asmc_camera.detection.messages import build_box_array
from asmc_camera.detection.ultralytics_backend import UltralyticsDetector, file_sha256
from camera_object_parity import boundary_only


ROOT = Path(__file__).resolve().parents[2]
MAIN = ROOT / "src/lidar+sensorfusion/src/Main"
SHA = "d5ffc1a674953a08e11a8d21e022781b1b23a19b730afc309290bd9fb5305b95"


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def signature(message):
    return [(box.classes, box.xMin, box.yMin, box.xMax, box.yMax)
            for box in message.yolo_bbox_arr]


class ObservedModel:
    """Observe raw legacy results while yielding the same original objects."""
    def __init__(self, model, names):
        self.model, self.names, self.candidates = model, names, []

    def __call__(self, *args, **kwargs):
        self.candidates = []
        for result in self.model(*args, **kwargs):
            for box in result.boxes if result.boxes is not None else []:
                self.candidates.append((self.names[int(box.cls[0])], *map(int, box.xyxy[0]), float(box.conf[0])))
            yield result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--expected-frames", type=int, default=3514)
    parser.add_argument("--device", default="cuda:0")
    args = parser.parse_args()
    rospy.rostime.set_rostime_initialized(True)
    legacy = load_module("legacy_yolo", MAIN / "src/Camera/YOLO/YOLODetection.py")
    adapter = load_module("object_adapter", MAIN / "scripts/camera_objects_yolo_bbox_adapter.py")
    weights = MAIN / "YOLOWeight/yolo11m.pt"
    if file_sha256(weights) != SHA:
        raise RuntimeError("legacy weight SHA mismatch")
    detector = UltralyticsDetector(
        str(ROOT / "src/asmc_camera/models/object_detection/yolo11m.pt"),
        device=args.device, expected_sha256=SHA,
        predict_overrides={"iou": 0.7, "max_det": 300, "agnostic_nms": False})
    groups = load_class_groups(yaml.safe_load(
        (ROOT / "src/asmc_camera/config/camera/object_classes.yaml").read_text())["class_groups"])
    original = legacy.st_LaneDetection.__new__(legacy.st_LaneDetection)
    original.model = YOLO(str(weights))
    original.model.fuse()
    original.model.to(args.device)
    original.device, original.use_cuda, original.conf_thres = args.device, True, 0.7
    original.names = original.model.names
    original.allowed_names = {"bus", "truck", "person", "car"}
    original.allowed_ids = {cid for cid, name in original.names.items()
                            if name.lower() in original.allowed_names}
    original._verify_model_device(weights)
    original.model = ObservedModel(original.model, original.names)
    original.frame_lock = threading.Lock()
    original.stats = legacy.IntervalStats(1e12)
    original.rate = SimpleNamespace(sleep=lambda: None)
    published = []
    original.st_Publisher = SimpleNamespace(publish=published.append)
    legacy.rospy = SimpleNamespace(is_shutdown=lambda: next(legacy.running),
                                   Time=rospy.Time, logwarn=print, loginfo=print)
    rows, errors, exceptions = [], [], []
    timing = {"old_ms": [], "new_ms": []}
    started = time.monotonic()
    with rosbag.Bag(str(args.bag)) as bag:
        for index, (_, message, _) in enumerate(bag.read_messages(topics=["/image_jpeg/compressed"])):
            image = cv2.imdecode(np.frombuffer(message.data, np.uint8), cv2.IMREAD_COLOR)
            if image is None:
                raise RuntimeError("bag contains undecodable frame %s" % index)
            original.st_BGRImg, original.image_header = image, (message.header.stamp, message.header.frame_id)
            original.frame_seq, original.decode_ms = index, 0.0
            legacy.running = iter((False, True))
            before = time.perf_counter()
            original.SpeedSignDetection()
            timing["old_ms"].append((time.perf_counter() - before) * 1000)
            before = time.perf_counter()
            detections, _ = detector.detect(image)
            candidate = build_box_array(message.header, image.shape[1], image.shape[0], True, detections, groups)
            wire = io.BytesIO()
            candidate.serialize(wire)
            received = CameraObjectBoxArray().deserialize(wire.getvalue())
            converted = adapter.convert(received)
            timing["new_ms"].append((time.perf_counter() - before) * 1000)
            old = published.pop()
            old_rows, new_rows = signature(old), signature(converted)
            header_ok = (old.header.stamp == converted.header.stamp == message.header.stamp
                         and old.header.frame_id == converted.header.frame_id == message.header.frame_id)
            if not header_ok or old.objc != len(old_rows) or converted.objc != len(new_rows):
                errors.append({"frame": index, "reason": "header/count"})
            elif old_rows != new_rows:
                candidates = [(d.source_class_name, int(d.xmin), int(d.ymin), int(d.xmax), int(d.ymax), d.confidence)
                              for d in detections]
                candidates += original.model.candidates
                allowed = boundary_only(old_rows, new_rows, candidates)
                record = {"frame": index, "stamp_ns": message.header.stamp.to_nsec(),
                          "old": old_rows, "new": new_rows}
                (exceptions if allowed is not None else errors).append(record)
            rows.append({"stamp_ns": message.header.stamp.to_nsec(), "boxes": new_rows})
            if (index + 1) % 250 == 0:
                print("frames=%d errors=%d boundary=%d elapsed=%.1fs" %
                      (index + 1, len(errors), len(exceptions), time.monotonic() - started), flush=True)
    count = len(rows)
    if count != args.expected_frames:
        errors.append({"reason": "frame count", "expected": args.expected_frames, "actual": count})
    result = {"status": "pass" if not errors else "fail", "frames": count,
              "exact_frames": count - len(errors) - len(exceptions), "errors": errors,
              "boundary_exceptions": exceptions, "weight_sha256": SHA,
              "bag": str(args.bag), "bag_sha256": file_sha256(args.bag),
              "legacy_source_sha256": file_sha256(MAIN / "src/Camera/YOLO/YOLODetection.py"),
              "python": sys.version, "interpreter": sys.executable,
              "torch": torch.__version__, "ultralytics": ultralytics.__version__,
              "gpu": torch.cuda.get_device_name(0), "backend": detector.info,
              "duration_sec": time.monotonic() - started,
              "latency": {name: {"p50": float(np.percentile(values, 50)),
                                 "p95": float(np.percentile(values, 95))}
                          for name, values in timing.items()}}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    args.output.with_suffix(".frames.json").write_text(json.dumps(rows))
    print(json.dumps(result, indent=2), flush=True)
    return 0 if not errors else 1


if __name__ == "__main__":
    raise SystemExit(main())
