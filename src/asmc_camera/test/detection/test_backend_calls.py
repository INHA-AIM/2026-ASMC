"""Check the legacy predictor call contract without torch or a GPU."""
import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

import numpy as np

from asmc_camera.detection import ultralytics_backend as backend


class BackendCallsTest(unittest.TestCase):
    def test_load_warmup_and_runtime_predict_arguments(self):
        model = Mock()
        model.names = {0: "person", 2: "car", 9: "traffic light"}
        model.model.parameters.return_value = iter([types.SimpleNamespace(device="cuda:0")])
        model.return_value = []
        torch = types.ModuleType("torch")
        torch.__version__ = "fake"
        torch.cuda = Mock()
        ultra = types.ModuleType("ultralytics")
        ultra.__version__ = "fake"
        ultra.YOLO = Mock(return_value=model)
        with tempfile.TemporaryDirectory() as directory:
            weights = Path(directory) / "weights.pt"
            weights.write_bytes(b"fixture")
            with patch.dict(sys.modules, {"torch": torch, "ultralytics": ultra}), patch.object(
                backend, "cuda_preflight", return_value=(True, "fixture")
            ), patch.object(backend, "describe_torch_environment", return_value=[]):
                detector = backend.UltralyticsDetector(
                    str(weights), expected_sha256=backend.file_sha256(weights),
                    predict_overrides={"iou": 0.7, "max_det": 300, "agnostic_nms": False},
                    log=lambda _: None,
                )
                warmup = model.call_args_list[0]
                self.assertEqual((720, 1280, 3), warmup.args[0].shape)
                self.assertNotIn("classes", warmup.kwargs)
                detections, _ = detector.detect(np.zeros((720, 1280, 3), np.uint8))
        self.assertEqual([], detections)
        model.fuse.assert_called_once()
        model.to.assert_called_once_with("cuda:0")
        kwargs = model.call_args.kwargs
        self.assertEqual({0, 2}, set(kwargs.pop("classes")))
        self.assertEqual({"stream": True, "verbose": False, "conf": 0.7,
                          "device": "cuda:0", "iou": 0.7, "max_det": 300,
                          "agnostic_nms": False}, kwargs)

    def test_weight_sha_mismatch_stops_before_loading(self):
        torch = types.ModuleType("torch")
        ultra = types.ModuleType("ultralytics")
        ultra.YOLO = Mock()
        with tempfile.TemporaryDirectory() as directory:
            weights = Path(directory) / "weights.pt"
            weights.write_bytes(b"fixture")
            with patch.dict(sys.modules, {"torch": torch, "ultralytics": ultra}):
                with self.assertRaisesRegex(backend.DetectorStartupError, "SHA-256 mismatch"):
                    backend.UltralyticsDetector(str(weights), expected_sha256="wrong")
        ultra.YOLO.assert_not_called()
