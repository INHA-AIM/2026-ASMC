"""Owned ROS master for one automatic experiment; never attach to a busy port."""
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time


def available_port(preferred):
    for port in [preferred, *range(11400, 12000)]:
        if port == 11311:
            continue
        with socket.socket() as probe:
            try:
                probe.bind(('127.0.0.1', port))
            except OSError:
                continue
            return port
    raise RuntimeError('no independent master port available')


def require_udp_ports_free(ports):
    held = []
    try:
        for port in ports:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            held.append(sock)
            try:
                sock.bind(('0.0.0.0', port))
            except OSError as exc:
                raise RuntimeError('UDP port already owned: %d; existing bridge was not stopped' % port) from exc
    finally:
        for sock in held:
            sock.close()


def descendant(pid, parent):
    seen = set()
    while pid > 1 and pid not in seen:
        if pid == parent:
            return True
        seen.add(pid)
        try:
            # /proc stat comm can contain spaces and parentheses.
            pid = int(Path('/proc/%d/stat' % pid).read_text().rsplit(')', 1)[1].split()[1])
        except (OSError, ValueError, IndexError):
            return False
    return False


class PrivateMaster:
    def __init__(self, output, preferred=11348):
        self.output, self.preferred = Path(output), preferred
        self.child = self.log = None
        self.previous = {}

    def __enter__(self):
        self.port = available_port(self.preferred)
        self.previous = {k: os.environ.get(k) for k in ('ROS_MASTER_URI', 'ROS_HOSTNAME', 'ROS_IP')}
        os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:%d' % self.port
        os.environ['ROS_HOSTNAME'] = '127.0.0.1'
        os.environ.pop('ROS_IP', None)
        self.log = (self.output / 'private-master.log').open('x')
        self.child = subprocess.Popen(['roscore', '-p', str(self.port)], stdout=self.log,
                                      stderr=subprocess.STDOUT, start_new_session=True)
        try:
            import rosgraph
            master = rosgraph.Master('/private_trial_owner')
            for _ in range(100):
                if self.child.poll() is not None:
                    raise RuntimeError('owned master exited during startup')
                try:
                    pid = master.getPid()
                except Exception:
                    time.sleep(.1)
                    continue
                if not descendant(pid, self.child.pid):
                    raise RuntimeError('master port race: refusing a foreign master')
                break
            else:
                raise RuntimeError('private master startup timeout')
            (self.output / 'private-runtime.json').write_text(json.dumps(dict(
                master_uri=os.environ['ROS_MASTER_URI'], master_pid=pid,
                launcher_pid=self.child.pid, shared_master_untouched=True), indent=2))
            return self
        except BaseException:
            self.__exit__(None, None, None)
            raise

    def __exit__(self, *_):
        if self.child and self.child.poll() is None:
            os.killpg(self.child.pid, signal.SIGINT)
            try:
                self.child.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(self.child.pid, signal.SIGTERM)
                self.child.wait(timeout=5)
        if self.log:
            self.log.close()
        for key, value in self.previous.items():
            if value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = value
