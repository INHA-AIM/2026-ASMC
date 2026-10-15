"""Bounded asynchronous diagnostics; never retain an entire driving run in RAM."""
from collections import deque
import json
from pathlib import Path
import threading
import time


class JsonlJournal:
    def __init__(self, path, capacity=4096, end_event='journal_end', time_key='at', clock=time.monotonic):
        if capacity <= 0:
            raise ValueError('journal capacity must be positive')
        self.path, self.capacity = Path(path), capacity
        self.end_event, self.time_key, self.clock = end_event, time_key, clock
        self.rows = deque()
        self.lock = threading.RLock()
        self.stop = threading.Event()
        self.dropped = self.written = self.high_water = 0
        self.error = None
        self.closed = False
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.thread = threading.Thread(target=self._write, name='camera-journal', daemon=True)
        self.thread.start()

    def append(self, row):
        with self.lock:
            if self.closed or self.error or len(self.rows) >= self.capacity:
                self.dropped += 1
                return
            self.rows.append(row)
            self.high_water = max(self.high_water, len(self.rows))

    def _write(self):
        try:
            with self.path.open('x') as stream:
                while True:
                    with self.lock:
                        batch = [self.rows.popleft() for _ in range(min(256, len(self.rows)))]
                        finished = self.stop.is_set() and not self.rows
                    for row in batch:
                        stream.write(json.dumps(row, separators=(',', ':')) + '\n')
                    self.written += len(batch)
                    if finished:
                        break
                    if not batch:
                        self.stop.wait(.02)
                ending = dict(event=self.end_event, dropped=self.dropped,
                              written=self.written, high_water=self.high_water)
                ending[self.time_key] = self.clock()
                stream.write(json.dumps(ending)+'\n')
        except Exception as exc:
            self.error = '{}: {}'.format(type(exc).__name__, exc)
            with self.lock:
                self.dropped += len(self.rows)
                self.rows.clear()

    def close(self):
        with self.lock:
            self.closed = True
        self.stop.set()
        self.thread.join(timeout=10.)
        if self.thread.is_alive() or self.error:
            raise RuntimeError('diagnostic journal failed: {}'.format(self.error or 'writer timeout'))
