"""Stop only this launch's processes, including children with their own session."""
import os
from pathlib import Path
import signal
import subprocess
import time

from .control import alive, process_identity


def descendants(pid):
    result = {}
    pending = [int(pid)]
    seen = set()
    while pending:
        child = pending.pop()
        if child in seen:
            continue
        seen.add(child)
        identity = process_identity(child)
        if identity:
            result[child] = identity
            # Nodes can be forked from a ROS launch worker thread. Visit each
            # thread's children, without scanning unrelated processes.
            for entry in Path(f'/proc/{child}/task').glob('*/children'):
                try:
                    pending.extend(map(int, entry.read_text().split()))
                except (OSError, ValueError):
                    pass
    return result


class OwnedProcess:
    def __init__(self, command, **kwargs):
        self.child = subprocess.Popen(command, start_new_session=True, **kwargs)
        self.identity = process_identity(self.child.pid)
        self.known = {}
        self.last_scan = float('-inf')

    def observe(self, force=False):
        now = time.monotonic()
        if alive(self.identity) and (force or now-self.last_scan >= .5):
            self.known.update(descendants(self.child.pid))
            self.last_scan = now
        return self.child.poll()

    def stop(self, durations=(8, 2, 2)):
        self.observe(force=True)
        for sig, duration in zip((signal.SIGINT, signal.SIGTERM, signal.SIGKILL), durations):
            remaining = [identity for identity in self.known.values() if alive(identity)]
            if not remaining:
                self.child.wait()
                return
            # First give roslaunch its normal child shutdown path. Descendants
            # may be setsid() children, so escalation uses recorded PID tokens.
            targets = [self.identity] if sig == signal.SIGINT and alive(self.identity) else remaining
            for identity in targets:
                if alive(identity):
                    try:
                        os.kill(identity['pid'], sig)
                    except ProcessLookupError:
                        pass
            deadline = time.monotonic()+duration
            while time.monotonic() < deadline:
                self.observe()
                if not any(alive(identity) for identity in self.known.values()):
                    self.child.wait()
                    return
                time.sleep(.05)
        raise RuntimeError('소유한 판제 프로세스의 종료를 확인하지 못했습니다.')
