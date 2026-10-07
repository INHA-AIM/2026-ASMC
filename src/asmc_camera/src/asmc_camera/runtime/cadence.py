"""Admission of fresh observations; never duplicate frames to meet a rate."""
from asmc_camera.bev.performance_monitor import source_time_gate_with_reset


def downstream_gate(stamp, previous, deadline, period, upstream_hz=0):
    """Use one rate controller when a managed producer already fits the ceiling.

    A second independent source-time phase drops slightly early observations
    even when the upstream worker is running at exactly the requested rate.
    Duplicate/old timestamps are still rejected and legacy admission is intact.
    """
    publish, following, reset = source_time_gate_with_reset(
        stamp, previous, deadline, period)
    if 0 < upstream_hz <= 1e9 / period:
        publish = reset or stamp > previous
    return publish, following, reset


class CoherentFrames:
    """Retain the latest frame per view until all views form a fresh batch."""
    def __init__(self, views, max_skew_ns):
        self.views = tuple(views)
        self.max_skew_ns = max_skew_ns
        self.generation = None
        self.pending = {}

    def offer(self, frames, generation):
        if generation != self.generation:
            self.pending.clear()
            self.generation = generation
        self.pending.update({f.view: f for f in frames})
        if not all(view in self.pending for view in self.views):
            return None
        stamps = {v: f.stamp.to_nsec() for v, f in self.pending.items()}
        if max(stamps.values()) - min(stamps.values()) > self.max_skew_ns:
            self.pending.pop(min(stamps, key=stamps.get))
            return None
        return [self.pending[view] for view in self.views]

    def clear(self):
        self.pending.clear()
