"""Render the published family contract without running another fitter."""
import numpy as np
from .shared_lane_family import contiguous_sample_state_segments


def family_segments(message):
    if not message.valid:
        return []
    states = {0: 'invalid', 1: 'observed', 2: 'structure_inferred'}
    segments = []
    for member in message.boundaries:
        support = (np.frombuffer(member.support_state,dtype=np.uint8)
                   if isinstance(member.support_state,(bytes,bytearray,memoryview)) else member.support_state)
        if len(member.polyline) != len(support) or member.color not in (1, 2):
            raise ValueError('malformed family member')
        points = np.asarray([(p.x, p.y) for p in member.polyline], dtype=np.float64).reshape(-1, 2)
        if not np.isfinite(points).all() or any(s not in states for s in support):
            raise ValueError('invalid family coordinates/support')
        for state, values in contiguous_sample_state_segments(points, support):
            segments.append((member.role, state, member.color, values))
    return segments
