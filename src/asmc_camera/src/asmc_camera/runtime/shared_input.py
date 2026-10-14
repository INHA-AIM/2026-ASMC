"""Demand-decoded, bounded leases; callbacks never wait for image processing."""
import copy
import threading
import time
from contextlib import contextmanager
from dataclasses import dataclass


class CacheBusy(RuntimeError):
    pass


class DecodedCache:
    def __init__(self, slots=2):
        if slots != 2:
            raise ValueError('runtime cache requires exactly two slots per camera')
        self.slots, self.rows = slots, {}
        self.lock = threading.RLock()
        self.decodes = self.hits = self.busy = self.errors = 0

    @contextmanager
    def lease(self, camera, key, data, decoder):
        with self.lock:
            rows = self.rows.setdefault(camera, [])
            entry = next((x for x in rows if x['key'] == key), None)
            if entry is None:
                if len(rows) == self.slots:
                    unused = next((x for x in rows if x['refs'] == 0), None)
                    if unused is None:
                        self.busy += 1
                        raise CacheBusy('both decoded slots are in use')
                    rows.remove(unused)
                entry = dict(key=key, refs=0, ready=threading.Event(), value=None, error=None)
                rows.append(entry)
                owner = True
                self.decodes += 1
            else:
                self.hits += 1
                owner = False
            entry['refs'] += 1
        try:
            if owner:
                try:
                    entry['value'] = decoder(data)
                    if entry['value'] is None:
                        raise ValueError('JPEG decoder returned no image')
                    entry['value'].flags.writeable = False
                except Exception as exc:
                    entry['error'] = exc
                    with self.lock:
                        self.errors += 1
                finally:
                    entry['ready'].set()
            else:
                entry['ready'].wait()
            if entry['error'] is not None:
                raise entry['error']
            yield entry['value']
        finally:
            with self.lock:
                entry['refs'] -= 1


@dataclass(frozen=True)
class Delivery:
    header: object
    data: bytes
    received_monotonic: float
    received_ros_ns: int
    decode_key: object
    decode_source: object


class SharedInput:
    def __init__(self, rospy, compressed_type):
        self.ros, self.message_type = rospy, compressed_type
        self.lock = threading.RLock()
        self.callbacks, self.subscribers = {}, {}
        self.last_stamp, self.epochs, self.last_payload = {}, {}, {}
        self.received = {}
        self.cache = DecodedCache()

    def subscribe(self, topic, callback, callback_args=None, **kwargs):
        callback = (lambda message, fn=callback, arg=callback_args: fn(message, arg)) if callback_args is not None else callback
        with self.lock:
            self.callbacks.setdefault(topic, []).append(callback)
            if topic not in self.subscribers:
                self.subscribers[topic] = self.ros.Subscriber(topic, self.message_type,
                    lambda m: self._receive(topic, m), queue_size=1, buff_size=16 << 20)
        broker = self
        class Subscription:
            def unregister(self):
                with broker.lock:
                    if callback in broker.callbacks[topic]:
                        broker.callbacks[topic].remove(callback)
        return Subscription()

    def _receive(self, topic, message):
        received = time.monotonic()
        received_ros = self.ros.Time.now().to_nsec()
        stamp = message.header.stamp.to_nsec()
        payload = bytes(message.data)
        with self.lock:
            previous = self.last_stamp.get(topic, 0)
            if stamp < previous or (stamp == previous and payload != self.last_payload.get(topic)):
                # Equal-stamp, changed payload must never reuse old pixels. The
                # model admission gate still rejects duplicate source stamps.
                self.epochs[topic] = self.epochs.get(topic, 0) + 1
            self.last_stamp[topic] = stamp
            self.last_payload[topic] = payload
            self.received[topic] = self.received.get(topic, 0) + 1
            callbacks = tuple(self.callbacks[topic])
            key = (message.header.frame_id, self.epochs.get(topic, 0), stamp)
        delivery = Delivery(copy.deepcopy(message.header), payload, received,
                            received_ros, (topic, key), self)
        for callback in callbacks:
            try:
                callback(delivery)
            except Exception as exc:
                self.ros.logerr_throttle(2., "camera input consumer failed: %s", exc)

    def lease(self, frame):
        import cv2
        import numpy as np
        topic, key = frame.decode_key
        return self.cache.lease(topic, key, frame.data,
            lambda data: cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR))

    def close(self):
        for subscriber in self.subscribers.values():
            subscriber.unregister()
