"""Pure helpers for the canonical traffic-light output contract."""

from __future__ import annotations

from typing import Iterable, List, Sequence, Tuple


STATE_BY_LABEL = {
    "unknown": 0,
    "not_detected": 1,
    "red": 2,
    "yellow": 3,
    "green": 4,
    "red_yellow": 5,
    "red_left": 6,
    "green_left": 7,
    "left": 8,
}


def state_for_label(label: str) -> int:
    return STATE_BY_LABEL.get(str(label), STATE_BY_LABEL["unknown"])


def valid_for_label(label: str) -> bool:
    value = str(label)
    return value not in ("unknown", "not_detected") and value in STATE_BY_LABEL


def ordered_candidates(
    labels: Sequence[str], probabilities: Iterable[float], topk: int
) -> Tuple[List[str], List[float]]:
    pairs = sorted(
        (
            (str(label), float(probability))
            for label, probability in zip(labels, probabilities)
        ),
        key=lambda value: value[1],
        reverse=True,
    )[: max(0, int(topk))]
    return [value[0] for value in pairs], [value[1] for value in pairs]


def populate_detection(
    message,
    *,
    header,
    label: str,
    confidence: float,
    stale: bool,
    inference_latency_ms: float,
    model_id: str,
    candidate_labels: Sequence[str],
    candidate_confidences: Sequence[float],
):
    if len(candidate_labels) != len(candidate_confidences):
        raise ValueError("candidate label/confidence lengths differ")
    message.header = header
    message.state = state_for_label(label)
    message.label = str(label)
    message.confidence = float(confidence)
    message.stale = bool(stale)
    message.valid = bool(valid_for_label(label) and not stale)
    message.inference_latency_ms = float(inference_latency_ms)
    message.model_id = str(model_id)
    message.candidate_labels = list(candidate_labels)
    message.candidate_confidences = [float(value) for value in candidate_confidences]
    return message
