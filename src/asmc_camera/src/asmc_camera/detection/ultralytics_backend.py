"""Ultralytics YOLO backend ported from lidar+sensorfusion Main/YOLODetection.py.

Model loading, the CUDA preflight, warm-up and the per-frame predict call
follow the original node so the P2a migration is behaviour preserving.
Optional predict arguments are passed only when configured; leaving them out
keeps the Ultralytics defaults the original node relied on.
"""

from __future__ import annotations

import hashlib
import time
from pathlib import Path
from typing import Iterable, List, Mapping, Optional, Tuple

import numpy as np

from asmc_camera.detection.backend import (
    Detection,
    allowed_class_ids,
    filter_result_boxes,
)


OPTIONAL_PREDICT_ARGS = ("iou", "max_det", "agnostic_nms", "imgsz", "half", "end2end")


def describe_torch_environment(torch) -> List[str]:
    info = [
        "torch=%s" % torch.__version__,
        "torch_cuda=%s" % torch.version.cuda,
        "torch.cuda.is_available=%s" % torch.cuda.is_available(),
    ]
    try:
        info.append("torch_arch_list=%s" % ",".join(torch.cuda.get_arch_list()))
    except Exception as exc:  # pragma: no cover - driverless builds
        info.append("torch_arch_list=<unavailable: %s>" % exc)
    try:
        info.append("device_count=%d" % torch.cuda.device_count())
    except Exception as exc:  # pragma: no cover
        info.append("device_count=<unavailable: %s>" % exc)
    if torch.cuda.is_available():
        try:
            major, minor = torch.cuda.get_device_capability(0)
            info.append("gpu_name=%s" % torch.cuda.get_device_name(0))
            info.append("gpu_capability=sm_%d%d" % (major, minor))
        except Exception as exc:  # pragma: no cover
            info.append("gpu_query=<failed: %s>" % exc)
    return info


def cuda_preflight(torch, device: str) -> Tuple[bool, str]:
    """Run a real kernel; is_available() alone misses architecture mismatches."""
    if not torch.cuda.is_available():
        return False, "torch.cuda.is_available() is False"
    try:
        if torch.cuda.device_count() < 1:
            return False, "torch.cuda.device_count() == 0"
        probe = torch.randn(256, 256, device=device)
        product = probe @ probe
        torch.cuda.synchronize(device)
        value = float(product.sum().item())
        if not np.isfinite(value):
            return False, "CUDA matmul produced a non-finite result"
    except Exception as exc:
        return False, "%s: %s" % (type(exc).__name__, exc)
    return True, "CUDA kernel executed and synchronized"


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


class DetectorStartupError(RuntimeError):
    """Raised when the detector cannot run on the requested device."""


class UltralyticsDetector:
    def __init__(
        self,
        weights: str,
        device: str = "cuda:0",
        require_cuda: bool = True,
        conf_thres: float = 0.7,
        allowed_names: Iterable[str] = ("bus", "truck", "person", "car"),
        expected_sha256: str = "",
        predict_overrides: Optional[Mapping[str, object]] = None,
        log=print,
    ) -> None:
        import torch
        import ultralytics
        from ultralytics import YOLO

        self._torch = torch
        weights_path = Path(weights)
        if not weights_path.is_file():
            raise DetectorStartupError("weights file not found: %s" % weights_path)
        self.weights_sha256 = file_sha256(weights_path)
        if expected_sha256 and self.weights_sha256 != expected_sha256:
            raise DetectorStartupError(
                "weights SHA-256 mismatch: expected %s got %s"
                % (expected_sha256, self.weights_sha256)
            )
        self.conf_thres = float(conf_thres)
        self.allowed_names = tuple(str(name).strip().lower() for name in allowed_names)
        self.device = self._select_device(str(device), bool(require_cuda), log)
        self.use_cuda = self.device.startswith("cuda")
        self.predict_overrides = {
            key: value
            for key, value in dict(predict_overrides or {}).items()
            if key in OPTIONAL_PREDICT_ARGS
        }

        self.model = YOLO(str(weights_path))
        try:
            self.model.fuse()
        except Exception:
            pass
        self.model.to(self.device)
        self.names: Mapping[int, str] = dict(self.model.names)
        self.allowed_ids = allowed_class_ids(self.names, self.allowed_names)
        if not self.allowed_ids:
            log(
                "allowed_ids is empty; checkpoint names differ from allowed_names, "
                "so names are re-checked per box"
            )
        model_device = self._model_device()
        if self.use_cuda and not model_device.startswith("cuda"):
            raise DetectorStartupError(
                "model parameters are on %s but %s was requested; refusing a silent "
                "CPU fallback" % (model_device, self.device)
            )
        warmup_ms = self._warmup()
        self.info = {
            "backend": "ultralytics",
            "weights": str(weights_path),
            "weights_sha256": self.weights_sha256,
            "device": self.device,
            "model_device": model_device,
            "ultralytics": ultralytics.__version__,
            "torch": torch.__version__,
            "conf_thres": self.conf_thres,
            "allowed_names": list(self.allowed_names),
            "predict_overrides": dict(self.predict_overrides),
            "warmup_ms": round(warmup_ms, 1),
        }

    def _select_device(self, requested: str, require_cuda: bool, log) -> str:
        if requested.startswith("cpu"):
            if require_cuda:
                raise DetectorStartupError(
                    "device=%s with require_cuda=true; set require_cuda=false to run "
                    "on CPU" % requested
                )
            log("YOLO running on CPU by explicit request (device=%s)" % requested)
            return "cpu"
        ok, detail = cuda_preflight(self._torch, requested)
        report = "\n  ".join(describe_torch_environment(self._torch))
        if ok:
            log("YOLO CUDA preflight OK: device=%s\n  %s\n  %s" % (requested, report, detail))
            return requested
        message = "YOLO CUDA preflight FAILED: %s\n  requested_device=%s\n  %s" % (
            detail,
            requested,
            report,
        )
        if require_cuda:
            raise DetectorStartupError(message)
        log(message + "\n  require_cuda=false, continuing on CPU")
        return "cpu"

    def _model_device(self) -> str:
        try:
            return str(next(self.model.model.parameters()).device)
        except Exception as exc:
            return "<unknown: %s>" % exc

    def _synchronize(self) -> None:
        if self.use_cuda:
            self._torch.cuda.synchronize(self.device)

    def _warmup(self) -> float:
        # Same image size and call shape as the original node; the predictor
        # is created here with the runtime device string.
        warmup_image = np.zeros((720, 1280, 3), dtype=np.uint8)
        self._synchronize()
        started = time.perf_counter()
        list(
            self.model(
                warmup_image,
                device=self.device,
                stream=True,
                verbose=False,
                conf=self.conf_thres,
                **self.predict_overrides,
            )
        )
        self._synchronize()
        return (time.perf_counter() - started) * 1000.0

    def detect(self, image_bgr: np.ndarray) -> Tuple[List[Detection], Mapping[str, float]]:
        """Return detections in model order and per-stage timings in ms."""
        kwargs = dict(
            stream=True,
            verbose=False,
            conf=self.conf_thres,
            device=self.device,
            **self.predict_overrides,
        )
        if self.allowed_ids:
            kwargs["classes"] = list(self.allowed_ids)
        self._synchronize()
        started = time.perf_counter()
        results = list(self.model(image_bgr, **kwargs))
        self._synchronize()
        inference_ms = (time.perf_counter() - started) * 1000.0
        speed = {}
        for result in results:
            speed = getattr(result, "speed", None) or speed
        detections = filter_result_boxes(
            results, self.names, self.allowed_ids, self.allowed_names, self.conf_thres
        )
        timings = {
            "inference_ms": inference_ms,
            "preprocess_ms": float(speed.get("preprocess", 0.0)),
            "postprocess_ms": float(speed.get("postprocess", 0.0)),
        }
        return detections, timings
