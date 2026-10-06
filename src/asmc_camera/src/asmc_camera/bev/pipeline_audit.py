"""Frame-level, ROS-independent audit state for the lane-vector pipeline."""

from __future__ import annotations

from collections import Counter
from copy import deepcopy
import math
from typing import Mapping


OUTAGE_KEYS = ("no_boundary", "no_yellow", "incomplete")


def _percentile(values, percentile):
    ordered = sorted(float(value) for value in values if math.isfinite(float(value)))
    if not ordered:
        return None
    position = (len(ordered) - 1) * float(percentile) / 100.0
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def _distribution(values):
    finite = [float(value) for value in values if math.isfinite(float(value))]
    return {
        "count": len(finite),
        "min": min(finite) if finite else None,
        "mean": sum(finite) / len(finite) if finite else None,
        "p50": _percentile(finite, 50),
        "p95": _percentile(finite, 95),
        "p99": _percentile(finite, 99),
        "max": max(finite) if finite else None,
    }


def primary_outcome(frame: Mapping) -> str:
    counts = frame.get("counts") or {}
    if frame.get("error"):
        return "pipeline_error"
    if int(counts.get("model_marking_pixels", 0)) <= 0:
        return "no_model_marking_pixels"
    if int(counts.get("evidence", 0)) <= 0:
        return "no_metric_evidence"
    if int(counts.get("nodes", 0)) <= 0:
        return "no_metric_nodes"
    if int(counts.get("tracks", 0)) <= 0:
        return "no_supported_track"
    if int(counts.get("prefit_tracks", 0)) <= 0:
        return "no_shadow_prefit_track"
    if int(counts.get("lanes", 0)) <= 0:
        return "no_fitted_spline"
    if int(counts.get("observations", 0)) <= 0:
        return "no_boundary_candidate"
    if int(counts.get("boundaries", 0)) <= 0:
        return "no_tracked_boundary"
    if int(counts.get("yellow_boundaries", 0)) <= 0:
        return "boundaries_without_yellow"
    if not bool(counts.get("complete", False)):
        return "partial_boundary_family"
    return "complete_boundary_family"


def rejection_counts(frame: Mapping) -> Counter:
    result = Counter()
    projection = frame.get("projection") or {}
    for view, values in projection.items():
        for reason in ("projection_invalid", "below_confidence"):
            count = int((values or {}).get(reason, 0) or 0)
            if count:
                result["projection.{}.{}".format(view, reason)] += count
    for stage in ("association", "prefit", "spline", "boundary_selection"):
        values = frame.get(stage) or {}
        rejected = values.get("rejected") or {}
        for reason, count in rejected.items():
            count = int(count or 0)
            if count:
                result["{}.{}".format(stage, reason)] += count
        # Association exposes its candidate-level rejection counters directly
        # because they are also useful live diagnostics.  Preserve that payload
        # contract and normalize it here instead of silently omitting the most
        # important track-formation losses from offline summaries.
        if stage == "association":
            for reason, count in values.items():
                if not str(reason).startswith("rejected_"):
                    continue
                count = int(count or 0)
                if count:
                    result["association.{}".format(str(reason)[9:])] += count
            conflicts = int(values.get("assignment_conflicts", 0) or 0)
            if conflicts:
                result["association.assignment_conflict"] += conflicts
    return result


class PipelineAuditAccumulator:
    """Accumulate deterministic outage/rejection counters across frames."""

    def __init__(self) -> None:
        self.frames = 0
        self.outcomes = Counter()
        self.rejections = Counter()
        self.rejection_frames = Counter()
        self.rejections_by_outcome = {}
        self.rejection_frames_by_outcome = {}
        self.current_streak = Counter()
        self.longest_streak = Counter()
        self.last_frame = {}

    def update(self, frame: Mapping) -> dict:
        frame = deepcopy(dict(frame))
        outcome = primary_outcome(frame)
        frame["primary_outcome"] = outcome
        self.frames += 1
        self.outcomes[outcome] += 1
        frame_rejections = rejection_counts(frame)
        self.rejections.update(frame_rejections)
        self.rejection_frames.update(frame_rejections.keys())
        outcome_rejections = self.rejections_by_outcome.setdefault(outcome, Counter())
        outcome_rejections.update(frame_rejections)
        outcome_frames = self.rejection_frames_by_outcome.setdefault(outcome, Counter())
        outcome_frames.update(frame_rejections.keys())
        counts = frame.get("counts") or {}
        active = {
            "no_boundary": int(counts.get("boundaries", 0)) <= 0,
            "no_yellow": int(counts.get("yellow_boundaries", 0)) <= 0,
            "incomplete": not bool(counts.get("complete", False)),
        }
        for key in OUTAGE_KEYS:
            self.current_streak[key] = self.current_streak[key] + 1 if active[key] else 0
            self.longest_streak[key] = max(
                self.longest_streak[key], self.current_streak[key]
            )
        self.last_frame = frame
        return self.snapshot()

    def snapshot(self) -> dict:
        denominator = max(self.frames, 1)
        return {
            "frames": int(self.frames),
            "outcomes": dict(sorted(self.outcomes.items())),
            "outcome_rates": {
                key: float(value) / denominator
                for key, value in sorted(self.outcomes.items())
            },
            "rejections": dict(sorted(self.rejections.items())),
            "rejection_frame_counts": dict(sorted(self.rejection_frames.items())),
            "rejection_frame_rates": {
                key: float(value) / denominator
                for key, value in sorted(self.rejection_frames.items())
            },
            "rejections_by_outcome": {
                outcome: dict(sorted(values.items()))
                for outcome, values in sorted(self.rejections_by_outcome.items())
            },
            "rejection_frame_counts_by_outcome": {
                outcome: dict(sorted(values.items()))
                for outcome, values in sorted(self.rejection_frames_by_outcome.items())
            },
            "rejection_frame_rates_by_outcome": {
                outcome: {
                    key: float(value) / max(self.outcomes[outcome], 1)
                    for key, value in sorted(values.items())
                }
                for outcome, values in sorted(self.rejection_frames_by_outcome.items())
            },
            "current_streak_frames": {
                key: int(self.current_streak[key]) for key in OUTAGE_KEYS
            },
            "longest_streak_frames": {
                key: int(self.longest_streak[key]) for key in OUTAGE_KEYS
            },
            "last_frame": deepcopy(self.last_frame),
        }


def summarize_pipeline_frames(frames) -> dict:
    """Aggregate recorded frame audits without trusting process-global counters."""
    accumulator = PipelineAuditAccumulator()
    for frame in frames:
        accumulator.update(frame)
    result = accumulator.snapshot()
    result["available"] = bool(frames)
    result["frame_messages"] = int(len(frames))
    timing_by_stage = {}
    profile_schemas = Counter()
    for frame in frames:
        schema = frame.get("runtime_profile_schema")
        if schema:
            profile_schemas[str(schema)] += 1
        for stage, value in (frame.get("timing_ms") or {}).items():
            try:
                timing_by_stage.setdefault(str(stage), []).append(float(value))
            except (TypeError, ValueError):
                continue
    result["runtime_profile"] = {
        "schemas": dict(sorted(profile_schemas.items())),
        "stages_ms": {
            stage: _distribution(values)
            for stage, values in sorted(timing_by_stage.items())
        },
    }
    return result


def summarize_pipeline_run_groups(groups) -> dict:
    """Aggregate runs without allowing outage streaks to cross run boundaries."""
    groups = [list(group) for group in groups]
    result = summarize_pipeline_frames(
        [frame for group in groups for frame in group]
    )
    per_run = [summarize_pipeline_frames(group) for group in groups]
    result["longest_streak_frames"] = {
        key: max(
            (value["longest_streak_frames"][key] for value in per_run),
            default=0,
        )
        for key in OUTAGE_KEYS
    }
    result["runs"] = len(groups)
    return result
