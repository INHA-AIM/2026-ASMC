#!/usr/bin/env python3
"""Same-bag browser A/B trial on an owned ROS master. No control publishers.

Baseline monitor source must be a pinned temporary export, not a live worktree.
Complete metrics and bounded debug metadata are streamed to a registered run.
"""
import argparse
from collections import deque
import json
import os
from pathlib import Path
import socket
import subprocess
import time
import threading
import xmlrpc.client
from check_runtime_ros import stop, percentile


def main(a):
    root=Path(a.output)
    if a.run_id not in root.parts:raise ValueError('registered run ID required in output')
    root.mkdir(parents=True,exist_ok=False)
    with socket.socket() as s:
        if s.connect_ex(('127.0.0.1',a.master_port))==0:raise RuntimeError('trial ROS port occupied')
    os.environ['ROS_MASTER_URI']='http://127.0.0.1:'+str(a.master_port)
    os.environ['ROS_IP']='127.0.0.1'
    processes=[];files=[]
    def start(cmd,name):
        log=(root/(name+'.log')).open('x');files.append(log)
        p=subprocess.Popen(cmd,stdout=log,stderr=log,start_new_session=True);processes.append(p);return p
    try:
        master=start(['roscore','-p',str(a.master_port)],'master')
        deadline=time.monotonic()+15
        while True:
            try:
                if xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/trial')[0]==1:break
            except OSError:pass
            if time.monotonic()>deadline:raise RuntimeError('master timeout')
            time.sleep(.1)
        import rospy
        from asmc_msgs.msg import LidarObjectArray
        from diagnostic_msgs.msg import DiagnosticArray
        from std_msgs.msg import String
        from playwright.sync_api import sync_playwright
        rospy.set_param('/use_sim_time',True);rospy.init_node('lidar_console_trial',disable_signals=True)
        active={};rows=[];lock=threading.Lock()
        def objects(msg):
            with lock:
                if active:active['objects'].append((time.monotonic(),msg.header.stamp.to_nsec()))
        def diagnostics(msg):
            with lock:
                if active:active['diag']={v.key:v.value for v in msg.status[0].values}
        def debug(msg):
            with lock:
                f=active.get('debug_file')
                if f:f.write(msg.data+'\n')
        subscribers=[rospy.Subscriber('/migration/lidar/objects_local',LidarObjectArray,objects,queue_size=1),
                     rospy.Subscriber('/migration/lidar/diagnostics',DiagnosticArray,diagnostics,queue_size=1),
                     rospy.Subscriber('/migration/lidar/debug/scan',String,debug,queue_size=1,buff_size=8*1024*1024)]
        with sync_playwright() as pw:
            browser=pw.chromium.launch(headless=True,args=['--disable-gpu','--no-sandbox'])
            for repeat in range(a.repeats):
                for mode in (('baseline','candidate') if repeat%2==0 else ('candidate','baseline')):
                    name=mode+'-'+str(repeat+1);metadata=(root/(name+'-debug.jsonl')).open('x')
                    with lock:
                        active.clear();active.update(objects=deque(maxlen=4096),diag={},debug_file=metadata)
                    node=start([a.baseline_node if mode=='baseline' else a.node,'__name:=lidar_pipeline',
                                '__ns:=/migration/lidar','_pipeline_config:='+a.config,'_package_dir:='+a.package_share,
                                '_enable_camera_fusion:=false','_debug_enabled:=true','_debug_rate_hz:=5','_debug_max_points:=20000'],name+'-node')
                    source=Path(a.baseline_monitor) if mode=='baseline' else Path(a.package_share)
                    monitor=start(['python3',str(source/'scripts/asmc_lidar_monitor_node.py'),'__name:=console_trial',
                                   '__ns:=/migration/lidar','_port:='+str(a.port),'_pipeline_config:='+a.config,
                                   '_web_dir:='+str(source/'web'),'_drivable_map_file:='+a.map],name+'-monitor')
                    page=browser.new_page(viewport={'width':1440,'height':900});errors=[]
                    page.set_default_timeout(5000)
                    page.on('pageerror',lambda error:errors.append(str(error)))
                    deadline=time.monotonic()+15
                    while True:
                        try:page.goto('http://127.0.0.1:'+str(a.port),wait_until='domcontentloaded',timeout=1000);break
                        except Exception:
                            if time.monotonic()>deadline:raise
                            time.sleep(.1)
                    page.wait_for_function('window.__asmcMonitor !== undefined')
                    player=start(['rosbag','play','--clock','--hz=100','--delay=1','--queue=10',a.bag,
                                  '--topics','/velodyne_points','/imu','/Ego_topic','/gps'],name+'-player')
                    samples=[];start_time=time.monotonic();shot=False
                    while player.poll() is None:
                        value=page.evaluate('''()=>{const s=window.__asmcMonitor.state;return {stamp:s.scene&&s.scene.stamp_ns,
                          age:!Number.isFinite(s.sceneAgeAtReceipt)?null:s.sceneAgeAtReceipt+performance.now()-s.sceneReceivedAt,
                          render:s.lastRenderMs===undefined?null:s.lastRenderMs,frame:s.frameSeq,
                          source:s.scene&&s.scene.seq,rows:document.querySelectorAll('#track-table tbody tr').length,
                          partial:!!(s.scene&&s.scene.missing&&s.scene.missing.length)}}''')
                        value['wall_s']=time.monotonic()-start_time;samples.append(value)
                        if repeat==0 and not shot and value['wall_s']>20:
                            page.screenshot(path=str(root/(name+'.png')));shot=True
                        time.sleep(.1)
                    if player.returncode:raise RuntimeError('player failed')
                    # Exercise the real DOM without altering ROS output/filter state.
                    if mode=='candidate':
                        page.check('#opt-hide-candidates');page.select_option('#opt-color','motion')
                        page.select_option('#opt-color','audit')
                        first=page.locator('#track-table tbody tr').first
                        if first.count():first.click();assert '군집 도로 점' in page.locator('#detail').inner_text()
                        page.locator('details').filter(has=page.locator('#opt-side-show')).locator('summary').click()
                        page.check('#opt-side-show');page.wait_for_timeout(100)
                        if repeat==0:page.screenshot(path=str(root/(name+'-detail.png')))
                    time.sleep(.2);stop(node);stop(monitor)
                    with lock:
                        active['debug_file']=None;metadata.close()
                        received=list(active['objects']);diag=dict(active['diag'])
                    values=[v['age'] for v in samples if v['stamp'] and v['age'] is not None and v['wall_s']>3]
                    rendering=[v['render'] for v in samples if v['render'] is not None and v['wall_s']>3]
                    stamps=[p[1] for p in received]
                    gaps=[1000*(b[0]-a[0]) for a,b in zip(received,received[1:])]
                    result={'mode':mode,'repeat':repeat+1,'errors':errors,'objects':len(stamps),
                      'stamp_errors':sum(b<=a for a,b in zip(stamps,stamps[1:])),
                      'display_age_ms_p50':percentile(values,.5),'display_age_ms_p95':percentile(values,.95),
                      'render_ms_p95':percentile(rendering,.95),'subscriber_gap_ms_max':max(gaps,default=0),
                      'diagnostics':diag,'samples':samples}
                    rows.append(result);(root/(name+'-metrics.json')).write_text(json.dumps(result,indent=2)+'\n')
                    print(json.dumps({k:v for k,v in result.items() if k not in ('samples','diagnostics')}),flush=True)
                    page.close()
            browser.close()
        (root/'summary.json').write_text(json.dumps({'run_id':a.run_id,'runs':rows,
          'note':'Age is monitor first-part to browser display, not camera exposure or physical sensor latency.'},indent=2)+'\n')
        if any(r['errors'] or r['stamp_errors'] for r in rows):raise RuntimeError('browser/stamp errors')
    except Exception as error:
        (root/'failure.json').write_text(json.dumps({'error':str(error),'completed_runs':rows if 'rows' in locals() else []},indent=2)+'\n')
        raise
    finally:
        for p in reversed(processes):stop(p)
        for f in files:f.close()

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('run-id','output','node','baseline-node','baseline-monitor','package-share','bag','config','map'):
        p.add_argument('--'+key,required=True)
    p.add_argument('--master-port',type=int,default=11435);p.add_argument('--port',type=int,default=18092)
    p.add_argument('--repeats',type=int,default=3)
    main(p.parse_args())
