"""Single pending work item; input callbacks never wait for slow computation."""
import threading


class LatestWork:
    def __init__(self):
        self.condition = threading.Condition()
        self.pending = None
        self.closed = False
        self.replaced = 0

    def offer(self, value):
        with self.condition:
            if self.closed:
                return False
            self.replaced += int(self.pending is not None)
            self.pending = value
            self.condition.notify()
            return True

    def pop(self):
        with self.condition:
            while self.pending is None and not self.closed:
                self.condition.wait()
            if self.closed:
                return None
            value, self.pending = self.pending, None
            return value

    def clear(self):
        with self.condition:
            self.pending = None

    def close(self):
        with self.condition:
            self.closed = True
            self.pending = None
            self.condition.notify_all()
