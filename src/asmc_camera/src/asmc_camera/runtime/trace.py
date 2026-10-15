"""Optional buffered lifecycle observations; no filesystem work in frame callbacks."""
import threading
import time
from pathlib import Path


class LifecycleTrace:
    def __init__(self, path='', capacity=4096, clock=time.monotonic):
        self.path = Path(path) if path else None
        self.capacity, self.clock = capacity, clock
        from .jsonl_journal import JsonlJournal
        self.journal = JsonlJournal(self.path, capacity, end_event='trace_end',
                                    time_key='monotonic', clock=clock) if self.path else None
        self.lock = threading.Lock()

    def __call__(self, event, **values):
        if self.path is None:
            return
        with self.lock:
            self.journal.append(dict(event=event, monotonic=self.clock(), **values))

    def close(self):
        if self.path is None:
            return
        with self.lock:
            self.journal.close()
