"""Stable, observer-only HTTP front door for one worktree's private ROS trials.

No ROS imports/control, inference, video or master forwarding here. Each trial
owns a separate existing dashboard backend; identity and heartbeat gate its data.
"""
import argparse
import fcntl
import hashlib
import http.server
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
import uuid

PROTOCOL = 'asmc-trial-monitor-v1'
# Keep detached child handles while this interpreter lives; no wait/termination
# on trial completion. The listener intentionally spans successive trials.
_FRONTENDS = []


def atomic_json(path, value):
    temp = path.with_suffix('.pending')
    temp.write_text(json.dumps(value))
    os.replace(temp, path)


def read_json(path):
    try: return json.loads(path.read_text())
    except (OSError, ValueError): return {}


def process_start(pid):
    try: return Path('/proc/%d/stat' % pid).read_text().split(') ',1)[1].split()[19]
    except (OSError, IndexError, ValueError): return None


def active_state(raw, now=None):
    if not raw:return dict(phase='idle',data_available=False,heartbeat_age_s=None)
    value = dict(raw)
    now = time.time() if now is None else now
    value['heartbeat_age_s'] = max(0., now-value.get('updated', 0.))
    if value.get('phase') not in ('ended', 'failed'):
        if process_start(value.get('pid', 0)) != value.get('process_start'):
            value.update(phase='interrupted', reason='trial process exited')
        elif value['heartbeat_age_s'] > 3.:
            value.update(phase='stale', reason='trial heartbeat expired')
    value['data_available'] = False
    return value


class FrontDoor(http.server.ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True
    def __init__(self, address, state, workspace):
        self.state, self.workspace = Path(state), Path(workspace)
        self.workers = threading.BoundedSemaphore(4)
        self.metrics_lock = threading.Lock()
        self.metrics = dict(requests=0, errors=0, bytes=0, elapsed_ms=0.)
        self.metrics_instance = None
        super().__init__(address, Handler)
    def process_request(self, request, client_address):
        if not self.workers.acquire(False):
            self.shutdown_request(request); return
        try: super().process_request(request, client_address)
        except BaseException:
            self.workers.release(); raise
    def process_request_thread(self, request, client_address):
        try: super().process_request_thread(request, client_address)
        finally: self.workers.release()


class Handler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(args[2].workspace/'src/asmc_planner'), **kwargs)
    def log_message(self, *args): pass
    def end_headers(self):
        self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate')
        super().end_headers()
    def reply(self, value, status=200):
        body = json.dumps(value, separators=(',', ':')).encode()
        self.send_response(status); self.send_header('Content-Type','application/json; charset=utf-8')
        self.send_header('Content-Length',str(len(body))); self.end_headers()
        self.wfile.write(body)
    def do_POST(self):
        if self.path!='/api/monitor/release-idle':self.send_error(404);return
        # A manual dashboard from this worktree can reclaim its normal port.
        # An active experiment or another workspace can never be displaced.
        if self.headers.get('X-ASMC-Workspace')!=str(self.server.workspace):
            self.reply(dict(error='different workspace'),409);return
        lock=self.server.state.with_name('owner.lock').open('a')
        try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:
            lock.close();self.reply(dict(error='active trial owner'),409);return
        meta=active_state(read_json(self.server.state))
        if meta.get('phase') not in ('ended','failed','interrupted','idle'):
            lock.close();self.reply(dict(error='active trial state'),409);return
        self.server.releasing_lock=lock
        self.reply(dict(released=True))
        threading.Thread(target=self.server.shutdown,daemon=True).start()

    def do_GET(self):
        path = urllib.parse.urlsplit(self.path)
        if path.path == '/api/monitor-health':
            instance=read_json(self.server.state).get('instance_id')
            with self.server.metrics_lock:
                if self.server.metrics_instance!=instance:
                    self.server.metrics_instance=instance
                    self.server.metrics=dict(requests=0,errors=0,bytes=0,elapsed_ms=0.)
                metrics = dict(self.server.metrics)
            self.reply(dict(protocol=PROTOCOL, workspace=str(self.server.workspace),instance_id=instance,metrics=metrics)); return
        if path.path not in ('/api/snapshot','/api/local-scene'):
            # Assets/map are always from this worktree, never another backend.
            super().do_GET(); return
        raw = read_json(self.server.state)
        meta = active_state(raw)
        enabled = meta.get('phase') in ('preparing','driving')
        snapshot = dict(interface_version='v2', received_at={}, shadow={})
        error = None; payload = None; start = time.monotonic()
        try:
            expected = urllib.parse.parse_qs(path.query).get('monitor_instance',[None])[0]
            if expected is not None and expected != meta.get('instance_id'):
                self.reply(dict(error='trial changed'),409); return
            if enabled and raw.get('backend_port'):
                request = urllib.request.Request('http://127.0.0.1:%d%s' % (raw['backend_port'], self.path))
                with urllib.request.urlopen(request, timeout=.2) as response:
                    if response.headers.get('X-ASMC-Trial-Identity') != raw.get('instance_id'):
                        raise ValueError('backend identity mismatch')
                    body = response.read(2*1024*1024+1)
                    if len(body)>2*1024*1024: raise ValueError('observer payload exceeds bound')
                    payload = json.loads(body)
                # A registry change during HTTP must not return the previous run.
                if read_json(self.server.state).get('instance_id') != raw.get('instance_id'):
                    self.reply(dict(error='trial changed'),409); return
                meta['data_available'] = True
                meta['connection'] = 'live'
            else: meta['connection'] = 'no_active_backend'
        except (OSError, ValueError, urllib.error.URLError) as exc:
            error = str(exc); meta.update(connection='waiting', observer_error=error)
            if isinstance(exc,urllib.error.HTTPError):exc.close()
        with self.server.metrics_lock:
            if self.server.metrics_instance != raw.get('instance_id'):
                self.server.metrics_instance=raw.get('instance_id')
                self.server.metrics=dict(requests=0,errors=0,bytes=0,elapsed_ms=0.)
            self.server.metrics['requests']+=1
            self.server.metrics['errors']+=bool(error)
            self.server.metrics['bytes']+=len(body) if payload is not None else 0
            self.server.metrics['elapsed_ms']+=(time.monotonic()-start)*1000.
        if path.path == '/api/local-scene':
            self.reply(payload if meta['data_available'] else dict(error='no current scene'),
                       200 if meta['data_available'] else 409); return
        if meta['data_available']: snapshot=payload
        # This field contains process/run identity, never planning inputs.
        snapshot['trial_monitor']=meta
        self.reply(snapshot)


def ensure_frontend(workspace, state_dir, preferred=8088):
    pointer=state_dir/'endpoint.json'
    previous=read_json(pointer)
    def health(port):
        try:
            with urllib.request.urlopen('http://127.0.0.1:%d/api/monitor-health'%port, timeout=.2) as r:
                value=json.load(r)
            return value.get('protocol')==PROTOCOL and value.get('workspace')==str(workspace)
        except (OSError,ValueError): return False
    if previous.get('port') and health(previous['port']): return previous
    for port in [preferred, *range(18088,18098)]:
        proc=subprocess.Popen([sys.executable,str(Path(__file__).resolve()),'serve',
             '--port',str(port),'--state',str(state_dir/'current.json'),'--workspace',str(workspace)],
             stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,start_new_session=True)
        until=time.monotonic()+2.
        while time.monotonic()<until:
            if proc.poll() is not None: break
            if health(port):
                value=dict(port=port,pid=proc.pid,process_start=process_start(proc.pid),
                           url='http://localhost:%d/web/index_v2.html'%port,
                           preferred_port=preferred,fallback=port!=preferred)
                _FRONTENDS.append(proc)
                atomic_json(pointer,value);return value
            time.sleep(.04)
        if proc.poll() is None: proc.terminate();proc.wait(timeout=2.)
    raise RuntimeError('no observer HTTP port available; existing listeners were preserved')


class TrialMonitor:
    def __init__(self, workspace, root, preferred=8088):
        self.workspace=Path(workspace).resolve();self.root=Path(root)
        key=hashlib.sha256(str(self.workspace).encode()).hexdigest()[:16]
        self.state_dir=Path.home()/'.cache/asmc-local-monitor'/key
        self.state_dir.mkdir(parents=True,exist_ok=True)
        self.lock=(self.state_dir/'owner.lock').open('a')
        fcntl.flock(self.lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        self.endpoint=ensure_frontend(self.workspace,self.state_dir,preferred)
        def git(*args):return subprocess.check_output(['git','-c','safe.directory='+str(self.workspace),*args],cwd=self.workspace,text=True).strip()
        self.meta=dict(protocol=PROTOCOL,instance_id=uuid.uuid4().hex,run_id=self.root.name,
             branch=git('branch','--show-current'),revision=git('rev-parse','HEAD'),
             dirty=bool(git('status','--porcelain')),workspace=str(self.workspace),
             container=socket.gethostname(),master_uri=os.environ['ROS_MASTER_URI'],
             pid=os.getpid(),process_start=process_start(os.getpid()),phase='preparing',reason='',
             backend_port=None,url=self.endpoint['url'],monitoring_enabled=True)
        self.path=self.state_dir/'current.json';self.mutex=threading.Lock();self.stop=threading.Event()
        self.write()
        self.thread=threading.Thread(target=self.beat,daemon=True);self.thread.start()
    def write(self):
        self.meta['updated']=time.time();atomic_json(self.path,self.meta)
    def beat(self):
        while not self.stop.wait(1.):
            with self.mutex:self.write()
    def update(self, **value):
        with self.mutex:self.meta.update(value);self.write()
    def environment(self):
        return {**os.environ,'ASMC_TRIAL_MONITOR_IDENTITY':self.meta['instance_id']}
    def close(self, reason):
        self.stop.set();self.thread.join(timeout=2.)
        self.update(phase='ended',reason=reason,backend_port=None)
        value=dict(self.meta,frontend_pid=self.endpoint['pid'],observer_metrics=None)
        try:
            with urllib.request.urlopen('http://127.0.0.1:%d/api/monitor-health'%self.endpoint['port'],timeout=.2) as r:
                value['observer_metrics']=json.load(r)['metrics']
        except (OSError,ValueError): pass
        with (self.root/'monitor.json').open('x') as f:json.dump(value,f,indent=2)
        self.lock.close()


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('command',choices=['serve'])
    p.add_argument('--port',type=int,required=True);p.add_argument('--state',type=Path,required=True)
    p.add_argument('--workspace',type=Path,required=True);a=p.parse_args()
    with FrontDoor(('0.0.0.0',a.port),a.state,a.workspace) as server:server.serve_forever(poll_interval=.2)


if __name__=='__main__':main()
