#!/usr/bin/env python3
"""Read-only realtime bag check on a private master. No actuator/bridge launch.

Only own subprocesses are terminated. Supply gaps are measured by the subscriber
wall clock; debug capture cost is a separate producer metric. Never uses the live
11311 master. Output must be a new subdirectory of an existing registered run.
"""
import argparse,json,os,signal,socket,subprocess,threading,time,urllib.request
from pathlib import Path


def main(a):
    if a.port==11311:raise ValueError('operating master is forbidden')
    registration=json.loads((a.run/'registration.json').read_text())
    if registration['run_id']!=a.run_id:raise ValueError('run ID mismatch')
    with socket.socket() as sock:sock.bind(('127.0.0.1',a.port))
    output=a.run/a.analysis_id;output.mkdir(exist_ok=False)
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d'%a.port,ROS_HOSTNAME='localhost')
    children=[];logs=[]
    def spawn(cmd,name):
        f=(output/(name+'.log')).open('x');logs.append(f)
        p=subprocess.Popen(list(map(str,cmd)),stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
        children.append(p);return p
    try:
        master=spawn(['roscore','-p',a.port],'master');time.sleep(2)
        import rospy
        from asmc_msgs.msg import LidarTrackingObservation
        from diagnostic_msgs.msg import DiagnosticArray
        from std_msgs.msg import String
        rospy.init_node('m1_replay_verifier',disable_signals=True)
        rospy.set_param('/use_sim_time',True)
        records=(output/'events.jsonl').open('x');lock=threading.Lock()
        def record(x):
            with lock:records.write(json.dumps(dict(wall=time.monotonic(),**x))+'\n');records.flush()
        def tracks(m):
            o=m.observation
            record(dict(kind='tracks',seq=o.scan_sequence,stamp_ns=o.objects.header.stamp.to_nsec(),
                status=o.objects.status,objects=len(o.objects.objects)))
        def diag(m):
            for s in m.status:record(dict(kind='diagnostics',name=s.name,values={v.key:v.value for v in s.values}))
        def debug(m):
            x=json.loads(m.data);record(dict(kind='debug',stamp_ns=x['stamp_ns'],capture_ms=x.get('capture_ms'),
                sampled=x.get('sampled'),excluded=x.get('hd_map_audit',{}).get('excluded_count')))
        subs=[rospy.Subscriber('/migration/m1/tracks_local',LidarTrackingObservation,tracks,queue_size=1),
              rospy.Subscriber('/migration/m1/diagnostics',DiagnosticArray,diag,queue_size=1)]
        if a.debug:subs.append(rospy.Subscriber('/migration/m1/debug/scan',String,debug,queue_size=1))
        node=spawn([a.node,'__name:=lidar_pipeline','__ns:=/migration/m1',
            '_pipeline_config:='+str(a.config),'_package_dir:='+str(a.repo/'src/asmc_lidar'),
            '_camera_calibration_config:='+str(a.repo/'src/asmc_camera/config/cameras_v3.yaml'),
            '_debug_enabled:='+str(a.debug).lower()], 'pipeline')
        if a.debug:
            spawn(['python3',a.repo/'src/asmc_lidar/scripts/asmc_lidar_monitor_node.py',
                '__name:=m1_monitor','__ns:=/migration/m1','_port:='+str(a.web_port),
                '_pipeline_config:='+str(a.config),'_web_dir:='+str(a.repo/'src/asmc_lidar/web'),
                '_debug_topic_prefix:=debug','_diagnostics_topic:=diagnostics',
                '_objects_local_topic:=objects_local','_objects_map_topic:=objects_map'], 'web')
        time.sleep(3)
        bag=spawn(['rosbag','play','--clock','--quiet',a.bag,'--topics','/velodyne_points','/imu','/gps','/Ego_topic',
            '/perception/camera/objects/observations','/perception/camera/objects/status'],'bag')
        started=time.monotonic();captured=False
        while bag.poll() is None:
            if node.poll() is not None:raise RuntimeError('pipeline exited')
            if time.monotonic()-started>100:raise RuntimeError('replay timeout')
            status=Path('/proc/%d/status'%node.pid).read_text()
            record(dict(kind='rss',rss_kib=int(next(l for l in status.splitlines() if l.startswith('VmRSS:')).split()[1])))
            if a.debug and not captured and time.monotonic()-started>20:
                for route in ('status','frame','drivable'):
                    with urllib.request.urlopen('http://127.0.0.1:%d/api/%s'%(a.web_port,route),timeout=5) as r:
                        (output/(route+'.bin' if route=='frame' else route+'.json')).write_bytes(r.read())
                # Browser checks the real binary PointCloud2->HTTP->canvas path.
                from playwright.sync_api import sync_playwright
                with sync_playwright() as pw:
                    browser=pw.chromium.launch(headless=True,args=['--no-sandbox'])
                    page=browser.new_page(viewport={'width':1500,'height':1000});errors=[]
                    page.on('pageerror',lambda e:errors.append(str(e)))
                    page.goto('http://127.0.0.1:%d'%a.web_port);page.wait_for_timeout(1500)
                    page.locator('#opt-show-pred').check();page.locator('#opt-hide-candidates').check()
                    page.locator('#opt-point-color').select_option('road');page.screenshot(path=str(output/'web-road.png'),full_page=True)
                    page.locator('#opt-point-color').select_option('reason');page.screenshot(path=str(output/'web-reasons.png'),full_page=True)
                    (output/'browser.json').write_text(json.dumps(dict(errors=errors,text=page.locator('body').inner_text()),ensure_ascii=False))
                    browser.close()
                if errors:raise RuntimeError('browser JS error')
                captured=True
            time.sleep(.2)
        time.sleep(1)
        for s in subs:s.unregister()
        rospy.signal_shutdown('owned replay completed');records.close()
        rows=[json.loads(l) for l in (output/'events.jsonl').open()]
        tr=[r for r in rows if r['kind']=='tracks'];valid=[r for r in tr if r['status'] in (1,2)]
        gaps=[(b['wall']-aa['wall'])*1000 for aa,b in zip(tr,tr[1:])]
        rates=[]
        if tr:
            t=tr[0]['wall']+5
            while t+5<=tr[-1]['wall']:
                rates.append(sum(t<=r['wall']<t+5 for r in tr)/5);t+=5
        def percentile(v,q):
            v=sorted(v);return v[int((len(v)-1)*q)] if v else None
        rss=[r['rss_kib'] for r in rows if r['kind']=='rss']
        captures=[r['capture_ms'] for r in rows if r['kind']=='debug' and r.get('capture_ms') is not None]
        diags=[r['values'] for r in rows if r['kind']=='diagnostics']
        summary=dict(run_id=a.run_id,debug=a.debug,scans=len(tr),valid_scans=len(valid),min_full_5s_hz=min(rates) if rates else None,
            max_gap_ms=max(gaps) if gaps else None,rss_first_kib=rss[0],rss_last_kib=rss[-1],rss_max_kib=max(rss),
            debug_capture_p95_ms=percentile(captures,.95),latest_diagnostics=diags[-1] if diags else {},
            browser_checked=captured,limits='Recorded sensor replay; other host workloads uncontrolled; no controlled MORAI/GPU load acceptance or actuation')
        (output/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary))
    finally:
        for p in reversed(children):
            if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
        for p in children:
            try:p.wait(timeout=8)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5)
        for f in logs:f.close()

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('run','repo','node','config','bag'):p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--run-id',required=True);p.add_argument('--analysis-id',required=True)
    p.add_argument('--port',type=int,default=11331);p.add_argument('--web-port',type=int,default=8093)
    p.add_argument('--debug',action='store_true');main(p.parse_args())
