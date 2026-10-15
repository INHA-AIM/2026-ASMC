"""Read-only GPU and ROS-node CPU/RSS sampling for an isolated B0 trial."""
import os
from pathlib import Path
import subprocess
import threading
import time
from xmlrpc.client import ServerProxy, Transport


class _TimeoutTransport(Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = .25
        return connection


def qt_viewers(proc=Path('/proc')):
    """Read actual image-viewer processes; ignore unreaped zombie registrations."""
    found=[]
    for p in proc.iterdir():
        if not p.name.isdigit():continue
        try:
            stat=(p/'stat').read_text().rsplit(')',1)[1].split()
            args=(p/'cmdline').read_bytes().decode().strip('\0').split('\0')
            if stat[0]=='Z' or not args:continue
            is_viewer=Path(args[0]).name=='rqt_image_view' or (
                Path(args[0]).name.startswith('python') and len(args)>1 and Path(args[1]).name=='rqt_image_view')
            if is_viewer:found.append(dict(pid=int(p.name),command=args,
                owned_trial='__name:=camera_runtime_viewer' in args))
        except (OSError,ValueError,IndexError):continue
    return sorted(found,key=lambda r:r['pid'])


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
    def __init__(self, master_uri, node_names, interval=.2, system_metrics=False):
        # xmlrpc.client HTTPConnection/rosgraph.Master transports are not
        # thread safe. Only an immutable URI crosses into the sampler thread.
        self.master_uri, self.names, self.interval = master_uri, node_names, interval
        self.rows, self.errors, self.pids, self.previous = [], [], {}, {}
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.ticks = os.sysconf('SC_CLK_TCK')
        self.system_metrics = system_metrics
        self.previous_system = None
        self.discovery_after = {}

    def _system_sample(self):
        values = [int(v) for v in Path('/proc/stat').read_text().splitlines()[0].split()[1:]]
        # guest ticks are already included in user/nice.
        total, idle = sum(values[:8]), values[3]+values[4]
        old = self.previous_system
        self.previous_system = total, idle
        memory = {line.split(':')[0]: int(line.split()[1]) for line in
                  Path('/proc/meminfo').read_text().splitlines() if line.startswith(('MemTotal:', 'MemAvailable:'))}
        return dict(cpu_percent=(100*(1-(idle-old[1])/(total-old[0])) if old and total>old[0] else None),
                    memory_used_mib=(memory['MemTotal']-memory['MemAvailable'])/1024.,
                    memory_total_mib=memory['MemTotal']/1024.,
                    scope='WSL Linux guest; Windows applications CPU/RAM unavailable')

    def start(self):
        self.thread.start()

    def _discover(self):
        for name in self.names:
            if self.stop.is_set():
                return
            if name in self.pids or time.monotonic() < self.discovery_after.get(name,0):
                continue
            try:
                with ServerProxy(self.master_uri,transport=_TimeoutTransport()) as master:
                    code, _, uri = master.lookupNode('/b0_resources', name)
                if code != 1:
                    continue
                with ServerProxy(uri,transport=_TimeoutTransport()) as node:
                    code, _, pid = node.getPid('/b0_resources')
                if code == 1:
                    self.pids[name] = pid
            except Exception:
                self.discovery_after[name] = time.monotonic()+5.

    def _loop(self):
        next_discovery = 0
        while not self.stop.is_set():
            started = time.monotonic()
            if started >= next_discovery:
                self._discover()
                next_discovery = started+1.
            row = dict(arrival=time.monotonic(), gpu=None, nodes={})
            if self.system_metrics:
                try:
                    row['system'] = self._system_sample()
                except (OSError, ValueError, KeyError) as exc:
                    row['system_error'] = str(exc)
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
