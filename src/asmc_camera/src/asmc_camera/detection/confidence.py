"""Validated startup cutoff; downstream consumers may apply stricter policies."""
import math


def validate_confidence(value):
    if isinstance(value, bool):
        raise ValueError('conf_thres must be a finite number in [0,1]')
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError('conf_thres must be a finite number in [0,1]') from exc
    if not math.isfinite(result) or not 0 <= result <= 1:
        raise ValueError('conf_thres must be a finite number in [0,1]')
    return result
