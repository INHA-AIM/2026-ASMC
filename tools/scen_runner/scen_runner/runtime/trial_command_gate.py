"""Owned trial transport gate; never used by manual/operating launches.

MORAI UDP control carries D gear, overriding the SDK's preparation P gear.
Hold the actual transport under brake while checking upstream readiness. Once
released, forward only newly received commands; do not replay/refresh a cache.
"""
from threading import Lock


class TrialCommandGate:
    def __init__(self, publish, hold_factory):
        self.publish=publish
        self.hold_factory=hold_factory
        self.lock=Lock()
        self.released=False
        self.forwarded=0
        self.held=0

    def command(self,message):
        with self.lock:
            if self.released:
                self.publish(message)
                self.forwarded+=1
            else:
                self.publish(self.hold_factory())
                self.held+=1

    def tick(self):
        with self.lock:
            if not self.released:
                self.publish(self.hold_factory())
                self.held+=1

    def release(self):
        with self.lock:
            self.released=True

    def close(self):
        with self.lock:
            self.released=False
            self.publish(self.hold_factory())
