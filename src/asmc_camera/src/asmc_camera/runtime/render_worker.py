"""One active renderer and one coalesced notification, with an optional rate cap.

Incoming callbacks retain their latest input separately. The worker reads that
latest input *after* waiting for its rate budget, so waiting never queues images.
"""
import math
import threading
import time
from .latest_work import LatestWork


class RenderWorker:
    def __init__(self, render, max_hz=0., on_error=None):
        if not math.isfinite(max_hz) or max_hz < 0:
            raise ValueError('render rate must be finite and nonnegative')
        self.render, self.on_error = render, on_error
        self.period = 1./max_hz if max_hz else 0.
        self.work, self.stop = LatestWork(), threading.Event()
        self.thread = threading.Thread(target=self._run, name='latest-render', daemon=True)
        self.thread.start()

    def offer(self):
        return self.work.offer(True)

    def _run(self):
        deadline = 0.
        while self.work.pop() is not None and not self.stop.is_set():
            if self.stop.wait(max(0., deadline-time.monotonic())):
                break
            # All notifications up to this point are represented by one render
            # of the latest input. An offer during render remains pending.
            self.work.clear()
            started = time.monotonic()
            try:
                rendered = self.render()
            except Exception as error:
                if self.on_error is not None:
                    self.on_error(error)
                else:
                    raise
                rendered = False
            # No-change/housekeeping work does not consume a display budget.
            if rendered is not False:
                deadline = started+self.period

    def close(self):
        self.stop.set()
        self.work.close()
        if threading.current_thread() is not self.thread:
            self.thread.join(timeout=1.)
