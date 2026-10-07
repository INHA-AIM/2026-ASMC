"""Optional buffered lifecycle observations; no filesystem work in frame callbacks."""
import json
import threading
import time
from pathlib import Path


class LifecycleTrace:
    def __init__(self, path='', capacity=200000, clock=time.monotonic):
        self.path = Path(path) if path else None
        self.capacity, self.clock = capacity, clock
        self.rows, self.dropped = [], 0
        self.lock = threading.Lock()

    def __call__(self, event, **values):
        if self.path is None:
            return
        with self.lock:
            if len(self.rows) < self.capacity:
                self.rows.append(dict(event=event, monotonic=self.clock(), **values))
            else:
                self.dropped += 1

    def close(self):
        if self.path is None:
            return
        with self.lock:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            with self.path.open('w') as stream:
                for row in self.rows:
                    stream.write(json.dumps(row, separators=(',', ':')) + '\n')
                stream.write(json.dumps(dict(event='trace_end', monotonic=self.clock(),
                                             dropped=self.dropped)) + '\n')
