"""Bounded exact-frame join for the shared-plane BEV renderer (no ROS imports).

RGB joins by original wire sequence, frame and stamp. Native semantic heads
join by unique frame/stamp inside the current producer epoch; they do not share
the raw JPEG publisher's sequence. Ambiguous repeated stamps are rejected.
The owner serializes calls and resets on epoch/demand changes.
"""
from collections import OrderedDict
import math
import time


def image_key(header):
    return (int(header.stamp.to_nsec()), str(header.frame_id))


class ComparisonFrames:
    def __init__(self, views, calibration_ids, capacity=8, max_age_sec=.5, clock=time.monotonic):
        if capacity < 1 or max_age_sec <= 0:
            raise ValueError('comparison cache bounds must be positive')
        self.views, self.calibration_ids = tuple(views), calibration_ids
        self.capacity, self.max_age, self.clock = capacity, max_age_sec, clock
        self.reset()

    def reset(self):
        self.raw = {v: OrderedDict() for v in self.views}
        self.semantic = {v: OrderedDict() for v in self.views}
        self.contexts = OrderedDict()
        self.session = None
        self.retired_sessions = []
        self.last_output_ns = 0

    def _trim(self, entries):
        now = self.clock()
        for key, (at, _) in tuple(entries.items()):
            if now - at > self.max_age:
                entries.pop(key, None)
        while len(entries) > self.capacity:
            entries.popitem(last=False)

    def add_raw(self, view, message):
        key = (*image_key(message.header), int(message.header.seq))
        self.raw[view][key] = (self.clock(), message)
        self._trim(self.raw[view])

    def add_semantic(self, view, messages):
        key = image_key(messages[0].header)
        if any(image_key(m.header) != key for m in messages):
            raise ValueError('semantic heads differ in camera/stamp')
        entries = self.semantic[view]
        previous = entries.get(key)
        # Multiple native results for the same source stamp cannot be selected
        # safely. Do not use their unrelated publisher seq to guess a raw frame.
        if previous is not None:
            old = previous[1]
            if old is None or tuple(m.header.seq for m in old) != tuple(m.header.seq for m in messages):
                messages = None
        entries[key] = (self.clock(), messages)
        self._trim(entries)

    def add_context(self, message):
        count = len(self.views)
        if (len(message.views) != count or set(message.views) != set(self.views) or
                len(message.image_headers) != count or len(message.calibration_ids) != count or
                len(message.plane_coefficients) != 3*count or
                not all(math.isfinite(x) for x in message.plane_coefficients) or
                not message.producer_session_id or not message.projection_session_id or not message.config_id):
            raise ValueError('invalid projection context')
        if message.header.frame_id != 'base_link':
            raise ValueError('projection context must use base_link')
        for view, header, identity in zip(message.views, message.image_headers, message.calibration_ids):
            if identity != self.calibration_ids[view] or header.stamp.to_nsec() <= 0:
                raise ValueError('projection calibration/source mismatch')
        stamp_ns = message.header.stamp.to_nsec()
        if stamp_ns != max(h.stamp.to_nsec() for h in message.image_headers):
            raise ValueError('projection reference stamp differs from bundle')
        session = (message.producer_session_id, message.generation, message.projection_session_id, message.config_id)
        if session in self.retired_sessions:
            raise ValueError('retired projection producer')
        if self.session is not None and session != self.session:
            if session[:3] == self.session[:3]:
                raise ValueError('projection config changed inside one session')
            retired = (self.retired_sessions + [self.session])[-8:]
            last_output_ns = self.last_output_ns
            self.reset()
            self.retired_sessions = retired
            self.last_output_ns = last_output_ns
        self.session = session
        if stamp_ns > self.last_output_ns:
            self.contexts[stamp_ns] = (self.clock(), message)
        self._trim(self.contexts)

    def latest(self, require_raw=True):
        self._trim(self.contexts)
        for view in self.views:
            self._trim(self.raw[view]); self._trim(self.semantic[view])
        for stamp_ns in sorted(self.contexts, reverse=True):
            if stamp_ns <= self.last_output_ns:
                continue
            context = self.contexts[stamp_ns][1]
            raw, semantic = {}, {}
            for view, header in zip(context.views, context.image_headers):
                raw_entry = self.raw[view].get((*image_key(header), int(header.seq)))
                semantic_entry = self.semantic[view].get(image_key(header))
                if (require_raw and raw_entry is None) or semantic_entry is None or semantic_entry[1] is None:
                    break
                raw[view], semantic[view] = raw_entry[1] if raw_entry else None, semantic_entry[1]
            else:
                self.last_output_ns = stamp_ns
                self.contexts = OrderedDict((k, v) for k, v in self.contexts.items() if k > stamp_ns)
                return context, raw, semantic
        return None
