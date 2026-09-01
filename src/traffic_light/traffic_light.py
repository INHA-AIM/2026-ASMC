#!/usr/bin/env python3
"""Non-blocking API used by driving/control code to read inference results."""

import json
import threading
import time


NOT_DETECTED = "not_detected"


class TrafficLightSubscriber(object):
    """Subscribe to live_infer.py results and expose the newest value via state()."""

    def __init__(self, state_topic="/traffic_light/state",
                 detail_topic="/traffic_light/detail", stale_sec=1.0,
                 node_name="traffic_light_client"):
        self._state_topic = state_topic
        self._detail_topic = detail_topic
        self._stale = stale_sec
        self._node_name = node_name
        self._lock = threading.Lock()
        self._label = NOT_DETECTED
        self._prob = 0.0
        self._last_ok = 0.0
        self._subscribers = []

    def start(self):
        import rospy
        from std_msgs.msg import String

        if self._subscribers:
            return self
        if not rospy.core.is_initialized():
            rospy.init_node(self._node_name, anonymous=True, disable_signals=True)
        self._subscribers = [
            rospy.Subscriber(self._state_topic, String, self._state_cb, queue_size=1),
            rospy.Subscriber(self._detail_topic, String, self._detail_cb, queue_size=1),
        ]
        return self

    def stop(self):
        for sub in self._subscribers:
            try:
                sub.unregister()
            except Exception:
                pass
        self._subscribers = []

    def _state_cb(self, msg):
        with self._lock:
            self._label = msg.data
            self._last_ok = time.monotonic()

    def _detail_cb(self, msg):
        try:
            result = json.loads(msg.data)
            label = result["label"]
            prob = float(result["prob"])
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            return
        with self._lock:
            self._label = label
            self._prob = prob
            self._last_ok = time.monotonic()

    def state(self):
        """Return the latest label, or not_detected when it is stale."""
        with self._lock:
            if time.monotonic() - self._last_ok > self._stale:
                return NOT_DETECTED
            return self._label

    def detail(self):
        """Return (label, probability), applying the same stale check."""
        with self._lock:
            if time.monotonic() - self._last_ok > self._stale:
                return NOT_DETECTED, 0.0
            return self._label, self._prob

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        self.stop()


# Keep the old import name for existing driving code.
TrafficLightClient = TrafficLightSubscriber


if __name__ == "__main__":
    import rospy

    tl = TrafficLightSubscriber().start()
    print("/traffic_light/state 폴링 (Ctrl-C 종료)")
    try:
        while not rospy.is_shutdown():
            label, prob = tl.detail()
            print("\r\033[K  신호 = %-14s %3.0f%%" %
                  (label.upper(), prob * 100), end="", flush=True)
            time.sleep(0.1)
    finally:
        tl.stop()
