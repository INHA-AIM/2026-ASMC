"""Bounded exact-source inputs for debug rendering, independent of inference."""
from collections import OrderedDict, Counter, deque


VIEWS = ('left', 'front', 'right')


class ObjectViewCache:
    def __init__(self, capacity=8, ttl=.5):
        self.capacity, self.ttl = capacity, ttl
        self.images = {v: OrderedDict() for v in VIEWS}
        self.observations, self.calibrations = {}, {}
        self.revisions = dict.fromkeys(VIEWS, 0)
        self.epoch, self.identity, self.control_key = 0, None, None
        self.retired_sessions = deque(maxlen=32)
        self.active = True
        self.identity_conflict = False
        self.counts = Counter()

    def clear(self):
        self.epoch += 1
        self.observations.clear()
        for images in self.images.values():
            images.clear()

    def control(self, status):
        key = (status.generation, status.mode, status.managed)
        active = not status.managed or status.mode == 2
        if key != self.control_key and (not active or (self.control_key and key[0] != self.control_key[0])):
            self.clear()
        self.control_key, self.active = key, active

    def expire(self, now):
        for images in self.images.values():
            for stamp in list(images):
                if now-images[stamp][1] > self.ttl:
                    del images[stamp]

    def image(self, view, message, now):
        self.expire(now)
        stamp = message.header.stamp.to_nsec()
        if message.header.frame_id != 'camera_'+view or stamp <= 0:
            self.counts['bad_image_header'] += 1
            return False
        images = self.images[view]
        if images and stamp <= next(reversed(images)):
            self.counts['old_image'] += 1
            return False
        images[stamp] = (message, now)
        while len(images) > self.capacity:
            images.popitem(last=False)
        return True

    def observe(self, observation, now):
        d = observation.detections
        view = d.header.frame_id[7:]
        if view not in VIEWS or d.header.frame_id != 'camera_'+view or d.header.stamp.to_nsec() <= 0:
            self.counts['bad_observation_header'] += 1
            return False
        identity = (observation.producer_session_id, observation.generation,
                    observation.model_id, observation.config_id)
        if not identity[0] or not observation.calibration_id:
            self.counts['missing_identity'] += 1
            return False
        if self.identity != identity:
            old = self.identity
            if old and (identity[0] in self.retired_sessions or
                        (identity[0] == old[0] and identity[1] < old[1])):
                self.counts['old_epoch'] += 1
                return False
            if old and identity[:2] == old[:2]:
                self.clear()
                self.identity_conflict = True
                self.counts['identity_conflict'] += 1
                return False
            if old:
                if identity[0] != old[0]:
                    self.retired_sessions.append(old[0])
                self.clear()
            self.identity = identity
            self.identity_conflict = False
        if self.identity_conflict:
            return False
        if self.control_key and identity[1] != self.control_key[0]:
            self.counts['status_generation_mismatch'] += 1
            return False
        if self.calibrations.get(view) not in (None, observation.calibration_id):
            self.images[view].clear()
            self.observations.pop(view, None)
            self.revisions[view] += 1
        self.calibrations[view] = observation.calibration_id
        previous = self.observations.get(view)
        if d.valid and previous and d.header.stamp <= previous[0].detections.header.stamp:
            self.counts['old_observation'] += 1
            return False
        if not d.valid:
            self.revisions[view] += 1
        self.observations[view] = (observation, now)
        return True

    def pair(self, view, now):
        self.expire(now)
        item = self.observations.get(view)
        if (not self.active or self.identity_conflict or not item or not item[0].detections.valid or now-item[1] > self.ttl or
                (self.control_key and (item[0].generation != self.control_key[0] or self.identity[1] != self.control_key[0]))):
            return None
        observation, received = item
        image = self.images[view].get(observation.detections.header.stamp.to_nsec())
        if image is None:
            return None
        return image[0], observation, received
