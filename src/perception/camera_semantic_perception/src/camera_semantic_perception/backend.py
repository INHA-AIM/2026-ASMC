"""Backend contract shared by live segmentation models."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Mapping, Sequence, Tuple

import numpy as np


@dataclass(frozen=True)
class BackendInfo:
    name: str
    model_config: str
    heads: Tuple[str, ...]
    input_hw: Tuple[int, int]
    device: str
    checkpoint_path: str
    checkpoint_sha256: str
    checkpoint_epoch: int
    weights: str
    upstream_commit: str


class SemanticBackend(ABC):
    """Convert normalized RGB NCHW batches to foreground probabilities."""

    @property
    @abstractmethod
    def info(self) -> BackendInfo:
        raise NotImplementedError

    @abstractmethod
    def infer(self, batch: np.ndarray) -> Mapping[str, np.ndarray]:
        """Return one float32 `[B,H,W]` probability array per semantic head."""
        raise NotImplementedError


def validate_backend_output(
    outputs: Mapping[str, np.ndarray],
    heads: Sequence[str],
    batch_shape: Sequence[int],
) -> None:
    expected_shape = (int(batch_shape[0]), int(batch_shape[2]), int(batch_shape[3]))
    if set(outputs) != set(heads):
        raise ValueError(
            "backend heads differ from contract: expected={} actual={}".format(
                tuple(heads), tuple(outputs)
            )
        )
    for head in heads:
        value = outputs[head]
        if not isinstance(value, np.ndarray):
            raise TypeError("{} output must be a numpy array".format(head))
        if value.shape != expected_shape:
            raise ValueError(
                "{} output shape must be {}, got {}".format(
                    head, expected_shape, value.shape
                )
            )
        if value.dtype != np.float32:
            raise ValueError("{} output must be float32".format(head))
        if not np.all(np.isfinite(value)):
            raise ValueError("{} output contains non-finite values".format(head))
        if value.size and (float(value.min()) < 0.0 or float(value.max()) > 1.0):
            raise ValueError("{} probability lies outside [0,1]".format(head))
