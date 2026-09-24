"""Pure comparison helpers for traffic-light producer parity audits."""

from __future__ import annotations

import json
from typing import Any, Dict, List, Mapping, Sequence

from asmc_perception.traffic_light_contract import state_for_label, valid_for_label


class ParityError(ValueError):
    """Raised when compatibility and canonical results diverge."""


def parse_detail(payload: str) -> Dict[str, Any]:
    try:
        value = json.loads(payload)
    except (TypeError, ValueError) as exc:
        raise ParityError("traffic-light detail is not valid JSON") from exc
    if not isinstance(value, dict):
        raise ParityError("traffic-light detail must be a JSON object")
    label = value.get("label")
    if not isinstance(label, str) or not label:
        raise ParityError("traffic-light detail has no label")
    try:
        probability = float(value.get("prob"))
    except (TypeError, ValueError) as exc:
        raise ParityError("traffic-light detail has no numeric probability") from exc
    raw_candidates = value.get("top", [])
    if not isinstance(raw_candidates, list):
        raise ParityError("traffic-light detail top must be a list")
    candidates: List[List[Any]] = []
    for candidate in raw_candidates:
        if not isinstance(candidate, list) or len(candidate) != 2:
            raise ParityError("traffic-light detail candidate must be [label, confidence]")
        candidates.append([str(candidate[0]), float(candidate[1])])
    return {
        "label": label,
        "prob": probability,
        "top": candidates,
        "stale": bool(value.get("stale", False)),
    }


def compare_details(
    reference: Mapping[str, Any],
    canonical: Mapping[str, Any],
    tolerance: float = 1.0e-4,
) -> Dict[str, float]:
    if reference["label"] != canonical["label"]:
        raise ParityError(
            "label mismatch: reference=%s canonical=%s"
            % (reference["label"], canonical["label"])
        )
    if bool(reference["stale"]) != bool(canonical["stale"]):
        raise ParityError("stale flag mismatch")
    probability_delta = abs(float(reference["prob"]) - float(canonical["prob"]))
    if probability_delta > tolerance:
        raise ParityError("probability delta %.8f exceeds tolerance" % probability_delta)
    reference_top = list(reference["top"])
    canonical_top = list(canonical["top"])
    if len(reference_top) != len(canonical_top):
        raise ParityError("candidate count mismatch")
    candidate_delta = 0.0
    for reference_value, canonical_value in zip(reference_top, canonical_top):
        if reference_value[0] != canonical_value[0]:
            raise ParityError(
                "candidate label mismatch: reference=%s canonical=%s"
                % (reference_value[0], canonical_value[0])
            )
        delta = abs(float(reference_value[1]) - float(canonical_value[1]))
        candidate_delta = max(candidate_delta, delta)
        if delta > tolerance:
            raise ParityError("candidate confidence delta %.8f exceeds tolerance" % delta)
    return {
        "probability_delta": probability_delta,
        "candidate_confidence_delta": candidate_delta,
    }


def validate_typed_result(
    message: Any,
    canonical: Mapping[str, Any],
    *,
    expected_stamp_nsec: int,
    expected_frame_id: str,
    tolerance: float = 1.0e-4,
) -> Dict[str, float]:
    label = str(canonical["label"])
    stale = bool(canonical["stale"])
    if message.label != label:
        raise ParityError("typed/detail label mismatch")
    if int(message.state) != state_for_label(label):
        raise ParityError("typed state enum does not match label")
    if bool(message.stale) != stale:
        raise ParityError("typed/detail stale mismatch")
    expected_valid = bool(valid_for_label(label) and not stale)
    if bool(message.valid) != expected_valid:
        raise ParityError("typed valid flag violates fail-safe rule")
    probability_delta = abs(float(message.confidence) - float(canonical["prob"]))
    if probability_delta > tolerance:
        raise ParityError("typed/detail probability mismatch")
    labels = list(message.candidate_labels)
    confidences = [float(value) for value in message.candidate_confidences]
    if len(labels) != len(confidences):
        raise ParityError("typed candidate arrays have different lengths")
    canonical_top: Sequence[Sequence[Any]] = canonical["top"]
    if labels != [str(value[0]) for value in canonical_top]:
        raise ParityError("typed/detail candidate label mismatch")
    candidate_delta = 0.0
    for confidence, value in zip(confidences, canonical_top):
        delta = abs(confidence - float(value[1]))
        candidate_delta = max(candidate_delta, delta)
        if delta > tolerance:
            raise ParityError("typed/detail candidate confidence mismatch")
    if int(message.header.stamp.to_nsec()) != int(expected_stamp_nsec):
        raise ParityError("typed result did not preserve source image stamp")
    if str(message.header.frame_id) != str(expected_frame_id):
        raise ParityError("typed result did not preserve source image frame_id")
    return {
        "probability_delta": probability_delta,
        "candidate_confidence_delta": candidate_delta,
    }
