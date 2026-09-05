"""Model-independent camera semantic inference primitives."""

from .backend import BackendInfo, SemanticBackend
from .frame_buffer import EncodedFrame, LatestFrameBuffer
from .geometry import LetterboxGeometry, letterbox_bgr, restore_probability
from .inference_engine import BatchPrediction, InferenceEngine, ViewPrediction

__all__ = [
    "BackendInfo",
    "BatchPrediction",
    "EncodedFrame",
    "InferenceEngine",
    "LatestFrameBuffer",
    "LetterboxGeometry",
    "SemanticBackend",
    "ViewPrediction",
    "letterbox_bgr",
    "restore_probability",
]
