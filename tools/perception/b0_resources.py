"""Read-only GPU and ROS-node CPU/RSS sampling for an isolated B0 trial."""
import os
from pathlib import Path
import subprocess
import threading
import time
from xmlrpc.client import ServerProxy


def gpu_sample():
    result = subprocess.run(['nvidia-smi', '--query-gpu=utilization.gpu,memory.used,memory.total',
                             '--format=csv,noheader,nounits'], capture_output=True, text=True, timeout=2)
    if result.returncode:
        raise RuntimeError(result.stderr.strip())
    fields = result.stdout.splitlines()[0].split(',')
    return dict(utilization_percent=float(fields[0]), memory_used_mib=float(fields[1]),
                memory_total_mib=float(fields[2]))


def process_sample(pid):
    root = Path('/proc')/str(pid)
    stat = (root/'stat').read_text().rsplit(')', 1)[1].split()
    ticks = int(stat[11])+int(stat[12])
    rss = next(line.split()[1] for line in (root/'status').read_text().splitlines() if line.startswith('VmRSS:'))
    return ticks, int(rss)/1024.


class ResourceSampler:
    def __init__(self, master, node_names, interval=.2):
        self.master, self.names, self.interval = master, node_names, interval
        self.rows, self.errors, self.pids, self.previous = [], [], {}, {}
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.ticks = os.sysconf('SC_CLK_TCK')

    def start(self):
        self.thread.start()

    def _discover(self):
        for name in self.names:
            if name in self.pids:
                continue
            try:
                code, _, pid = ServerProxy(self.master.lookupNode(name)).getPid('/b0_resources')
                if code == 1:
                    self.pids[name] = pid
            except Exception:
                pass  # A node not launched yet is distinct from a zero-RSS process.

    def _loop(self):
        next_discovery = 0
        while not self.stop.is_set():
            started = time.monotonic()
            if started >= next_discovery:
                self._discover()
                next_discovery = started+1.
            row = dict(arrival=started, gpu=None, nodes={})
            try:
                row['gpu'] = gpu_sample()
            except Exception as exc:
                row['gpu_error'] = str(exc)
            for name, pid in list(self.pids.items()):
                try:
                    cpu, rss = process_sample(pid)
                    now = time.monotonic()
                    previous = self.previous.get(name)
                    percent = ((cpu-previous[0])/self.ticks/(now-previous[1])*100 if previous else None)
                    row['nodes'][name] = dict(pid=pid, cpu_percent=percent, rss_mib=rss)
                    self.previous[name] = cpu, now
                except (OSError, StopIteration):
                    self.pids.pop(name, None)
                    self.previous.pop(name, None)
            row['sampling_ms'] = (time.monotonic()-started)*1000
            self.rows.append(row)
            self.stop.wait(max(0., self.interval-(time.monotonic()-started)))

    def close(self):
        self.stop.set()
        self.thread.join(timeout=5)
        return self.rows
