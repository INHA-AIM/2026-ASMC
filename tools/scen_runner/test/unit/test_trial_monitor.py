import http.server
import json
import os
from pathlib import Path
import tempfile
import threading
import time
import unittest
import urllib.error
import urllib.request
from scen_runner.runtime.trial_monitor import (FrontDoor, atomic_json, active_state,
                                              ensure_frontend, process_start)


class Backend(http.server.BaseHTTPRequestHandler):
    identity='one'
    def log_message(self,*args):pass
    def do_GET(self):
        body=json.dumps(dict(local_experiment=dict(run_id=self.identity,plan_sequence=1))).encode()
        self.send_response(200);self.send_header('X-ASMC-Trial-Identity',self.identity)
        self.send_header('Content-Length',str(len(body)));self.end_headers();self.wfile.write(body)


class TrialMonitorTest(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name)
        assets=self.root/'src/asmc_planner/web';assets.mkdir(parents=True)
        (assets/'index_v2.html').write_text('this branch only')
        self.backend=http.server.ThreadingHTTPServer(('127.0.0.1',0),Backend)
        self.front=FrontDoor(('127.0.0.1',0),self.root/'state.json',self.root)
        for s in (self.backend,self.front):threading.Thread(target=s.serve_forever,daemon=True).start()
        self.meta=dict(pid=os.getpid(),process_start=process_start(os.getpid()),updated=time.time(),
            phase='driving',instance_id='one',backend_port=self.backend.server_port,
            master_uri='http://127.0.0.1:11403',run_id='owned-run',branch='own',revision='abc')
        Backend.identity='one';self.write()
    def tearDown(self):
        for s in (self.front,self.backend):s.shutdown();s.server_close()
        self.temp.cleanup()
    def write(self):atomic_json(self.root/'state.json',self.meta)
    def get(self,path='/api/snapshot'):
        with urllib.request.urlopen('http://127.0.0.1:%d%s'%(self.front.server_port,path),timeout=1.) as r:
            return json.load(r)
    def test_actual_master_and_identity_then_end_hides_all_old_geometry(self):
        r=self.get();self.assertEqual(r['local_experiment']['run_id'],'one')
        self.assertEqual(r['trial_monitor']['master_uri'],self.meta['master_uri'])
        self.meta.update(phase='ended',reason='distance_limit');self.write()
        r=self.get();self.assertNotIn('local_experiment',r)
        self.assertFalse(r['trial_monitor']['data_available'])
    def test_new_run_reconnect_and_old_detail_request_is_rejected(self):
        self.get();self.meta.update(instance_id='two',run_id='next-run',master_uri='http://127.0.0.1:11409')
        Backend.identity='two';self.write()
        r=self.get();self.assertEqual(r['local_experiment']['run_id'],'two')
        with self.assertRaises(urllib.error.HTTPError) as caught:self.get('/api/local-scene?sequence=1&monitor_instance=one')
        self.assertEqual(caught.exception.code,409);caught.exception.close()
    def test_reused_backend_port_with_wrong_identity_cannot_mix_data(self):
        Backend.identity='foreign-run';r=self.get()
        self.assertFalse(r['trial_monitor']['data_available']);self.assertNotIn('local_experiment',r)
    def test_heartbeat_or_process_loss_never_looks_live(self):
        self.meta['updated']=time.time()-4.;self.write();self.assertEqual(self.get()['trial_monitor']['phase'],'stale')
        self.meta.update(updated=time.time(),pid=0,process_start='dead');self.write()
        self.assertEqual(self.get()['trial_monitor']['phase'],'interrupted')
    def test_occupied_preferred_port_is_preserved_and_frontend_is_reusable(self):
        endpoint=ensure_frontend(self.root,self.root,self.backend.server_port)
        try:
            self.assertNotEqual(endpoint['port'],self.backend.server_port)
            self.assertTrue(endpoint['fallback'])
            self.assertEqual(ensure_frontend(self.root,self.root,self.backend.server_port),endpoint)
            with urllib.request.urlopen('http://127.0.0.1:%d/'%self.backend.server_port,timeout=1) as r:
                self.assertEqual(json.load(r)['local_experiment']['run_id'],'one')
        finally:
            from scen_runner.runtime.trial_monitor import _FRONTENDS
            child=next(p for p in _FRONTENDS if p.pid==endpoint['pid']);child.terminate();child.wait(timeout=2)




class ManualHandoffTest(unittest.TestCase):
    setUp=TrialMonitorTest.setUp
    tearDown=TrialMonitorTest.tearDown
    write=TrialMonitorTest.write
    get=TrialMonitorTest.get
    def test_manual_startup_reclaims_only_idle_same_worktree_frontend(self):
        import sys
        workspace=Path(__file__).resolve().parents[4]
        sys.path.insert(0,str(workspace/'src/asmc_planner/scripts'))
        from dashboard_startup import run_on_port
        from scen_runner.runtime.trial_monitor import _FRONTENDS
        state_dir=self.root/'reclaim';state_dir.mkdir()
        meta=dict(self.meta,phase='ended');atomic_json(state_dir/'current.json',meta)
        endpoint=ensure_frontend(workspace,state_dir,self.backend.server_port)
        child=next(p for p in _FRONTENDS if p.pid==endpoint['pid'])
        try:
            self.assertEqual(17,run_on_port(endpoint['port'],http.server.BaseHTTPRequestHandler,lambda _:17,'127.0.0.1'))
            child.wait(timeout=3)
        finally:
            if child.poll() is None:child.terminate();child.wait(timeout=2)
    def test_active_same_worktree_frontend_is_not_reclaimed(self):
        import sys
        workspace=Path(__file__).resolve().parents[4]
        sys.path.insert(0,str(workspace/'src/asmc_planner/scripts'))
        from dashboard_startup import run_on_port
        self.front.workspace=workspace
        called=[]
        self.assertEqual(2,run_on_port(self.front.server_port,http.server.BaseHTTPRequestHandler,
                                      lambda _:called.append(True),'127.0.0.1'))
        self.assertEqual(called,[]);self.assertTrue(self.get()['trial_monitor']['data_available'])


if __name__=='__main__':unittest.main()
