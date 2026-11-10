"""Experiment-owned observer, explicitly tied to one private ROS graph.

The HTTP service outlives trials. Trial publishers and controllers do not.
An occupied port is never reclaimed from another process or worktree.
"""
import atexit
import hashlib
import json
import os
from pathlib import Path
import socket
import signal
import subprocess
import sys
import threading
import time
from urllib.parse import urlsplit
from urllib.request import urlopen


def private_master_uri(uri):
    parsed=urlsplit(uri)
    if parsed.scheme!='http' or parsed.hostname not in ('127.0.0.1','localhost') or not parsed.port or parsed.port==11311:
        raise ValueError('automatic observer requires an explicit local private master, never shared 11311')
    return uri


def source_identity(checkout):
    def git(*args):return subprocess.check_output(['git',*args],cwd=checkout,text=True).strip()
    patch=subprocess.check_output(['git','diff','HEAD','--binary'],cwd=checkout)
    return dict(branch=git('branch','--show-current'),revision=git('rev-parse','HEAD'),
                source_dirty=bool(git('status','--porcelain')),patch_sha256=hashlib.sha256(patch).hexdigest(),
                source_root=str(checkout.resolve()))


def ensure_dashboard(output,port,master_uri):
    """Reuse only a verified same-source, same-master observer; otherwise fail."""
    if not port:return None
    private_master_uri(master_uri)
    if not 1024<=port<=65535:raise ValueError('invalid observer HTTP port')
    import rosgraph
    checkout=Path(__file__).resolve().parents[2]
    master=rosgraph.Master('/localization_observer_guard',master_uri=master_uri)
    master_pid=master.getPid()
    identity=source_identity(checkout)
    backend=checkout/'src/asmc_planner/scripts/behavior_dashboard_server.py'
    helper=backend.with_name('dashboard_localization.py')
    expected=dict(master_uri=master_uri,master_pid=master_pid,source_root=identity['source_root'],
                  use_sim_time=bool(master.getParam('/use_sim_time')) if master.hasParam('/use_sim_time') else False,
                  revision=identity['revision'],branch=identity['branch'],
                  backend_sha256=hashlib.sha256(backend.read_bytes()).hexdigest(),
                  helper_sha256=hashlib.sha256(helper.read_bytes()).hexdigest())
    url='http://127.0.0.1:'+str(port)+'/api/observer'
    def inspect():
        with urlopen(url,timeout=1) as response:return json.load(response)['observer']
    with socket.socket() as sock:occupied=sock.connect_ex(('127.0.0.1',port))==0
    if occupied:
        try:observer=inspect()
        except Exception as error:raise RuntimeError('observer port is occupied by an unrecognized service; left untouched') from error
    else:
        directory=Path(output)/'observer';directory.mkdir(exist_ok=True)
        log=directory/('server-'+str(time.time_ns())+'.log')
        env=dict(os.environ,ROS_MASTER_URI=master_uri,ROS_HOSTNAME='127.0.0.1')
        with log.open('x') as stream:
            process=subprocess.Popen(['roslaunch','--wait','asmc_planner','dashboard.launch',
                'dashboard_port:='+str(port),'expected_master_uri:='+master_uri,
                'runtime_label:='+os.environ.get('ASMC_RUNTIME_LABEL',socket.gethostname())],
                env=env,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        deadline=time.monotonic()+15
        while True:
            try:observer=inspect();break
            except Exception:
                if process.poll() is not None or time.monotonic()>deadline:
                    if process.poll() is None:
                        process.send_signal(signal.SIGINT)
                        try:process.wait(5)
                        except subprocess.TimeoutExpired:process.terminate();process.wait(5)
                    raise RuntimeError('own observer failed startup; inspect '+str(log))
                time.sleep(.1)
        (directory/('service-'+str(process.pid)+'.json')).write_text(json.dumps(dict(
            pid=process.pid,log=str(log),expected=expected,observer=observer),indent=2))
    differences={key:dict(expected=value,actual=observer.get(key)) for key,value in expected.items() if observer.get(key)!=value}
    if differences:raise RuntimeError('observer identity mismatch; service left untouched: '+json.dumps(differences))
    return dict(url='http://localhost:'+str(port)+'/web/index_v2.html',identity=observer,
                lifecycle='persistent read-only service; per-trial autonomy dashboard disabled',
                geometry_hz=5,http_poll_hz=10)


class ExperimentReporter:
    """Latched identity plus wall-clock heartbeat, independent of /clock."""
    def __init__(self,output,run_id,trial_id,role,monitoring,conditions=None):
        import rospy
        from std_msgs.msg import String
        self.message=String;self.lock=threading.Lock();self.stop=threading.Event();self.closed=False
        self.stream=(Path(output)/'observer-events.jsonl').open('x')
        self.publisher=rospy.Publisher('/planning/localization/experiment',String,queue_size=1,latch=True)
        self.payload=dict(schema='asmc.localization.experiment.v1',run_id=run_id,trial_id=trial_id,
            role=role,phase='preparing',reason='',master_uri=private_master_uri(os.environ['ROS_MASTER_URI']),
            runtime=os.environ.get('ASMC_RUNTIME_LABEL',socket.gethostname()),started_at=time.time(),
            monitoring=monitoring,conditions=dict(conditions or {}),**source_identity(Path(__file__).resolve().parents[2]))
        self.thread=threading.Thread(target=self._heartbeat,daemon=True);self.thread.start()
        self.previous_hook=sys.excepthook
        def exception_hook(kind,error,traceback):
            self.finish('cancelled' if kind is KeyboardInterrupt else 'failed',str(error))
            self.previous_hook(kind,error,traceback)
        self.exception_hook=exception_hook;sys.excepthook=exception_hook
        atexit.register(self._unexpected_exit)
        self.phase('preparing')

    def _publish(self):
        with self.lock:
            self.payload['heartbeat_at']=time.time();raw=json.dumps(self.payload)
        self.publisher.publish(self.message(data=raw))

    def _heartbeat(self):
        while not self.stop.wait(1):
            try:self._publish()
            except Exception:pass

    def phase(self,phase,reason=''):
        with self.lock:
            self.payload.update(phase=phase,reason=reason)
            self.stream.write(json.dumps(dict(wall_time=time.time(),**self.payload))+'\n');self.stream.flush()
        self._publish()

    def finish(self,phase,reason=''):
        if self.closed:return
        self.stop.set();self.thread.join(2);self.phase(phase,reason)
        time.sleep(.15)  # Deliver terminal state before publisher disconnection.
        self.publisher.unregister();self.stream.close();self.closed=True
        if sys.excepthook is self.exception_hook:sys.excepthook=self.previous_hook

    def _unexpected_exit(self):
        if not self.closed:
            try:self.finish('failed','trial_process_exited_before_completion')
            except Exception:pass
