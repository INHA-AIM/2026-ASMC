"""Offline/test-side resource samples; no ROS master calls in control timers."""
import json
from pathlib import Path
import time


class LocalResources:
    def __init__(self, output):
        self.file = (Path(output) / 'resources.jsonl').open('x')
        self.next = 0.

    def sample(self):
        now = time.monotonic()
        if now < self.next:
            return
        self.next = now + 1.
        processes = []
        for path in Path('/proc').iterdir():
            if not path.name.isdecimal():
                continue
            try:
                argv = (path / 'cmdline').read_bytes().split(b'\0')
                name = Path(argv[0].decode()).name
                if name not in ('asmc_local_planner_node', 'asmc_local_controller_node'):
                    continue
                fields = dict(line.split(':', 1) for line in (path/'status').read_text().splitlines() if ':' in line)
                processes.append(dict(pid=int(path.name), name=name,
                                      rss_kib=int(fields['VmRSS'].split()[0])))
            except (OSError, ValueError, IndexError, UnicodeError, KeyError):
                continue  # Process can exit during a sample.
        self.file.write(json.dumps(dict(received_steady=now, processes=processes))+'\n')
        self.file.flush()

    def close(self):
        self.file.close()
