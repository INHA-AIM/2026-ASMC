#!/usr/bin/env python3
"""Paced bag OFF/ON trial on an owned, isolated ROS master. Never vehicle control.

Requires the Noetic build environment. Starts only its master/pipeline/bag player;
finally terminates precisely those process groups. Writes to a new registered run.
"""
import argparse
from collections import deque
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import threading
import time
import xmlrpc.client


def percentile(values, q):
    if not values:return None
    return sorted(values)[max(0,math.ceil(q*len(values))-1)]


def stop(p):
    if p and p.poll() is None:
        os.killpg(p.pid,signal.SIGINT)
        try:p.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid,signal.SIGTERM)
            try:p.wait(timeout=3)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()


def main(a):
    output=Path(a.output)
    if a.run_id not in output.parts:raise ValueError('output must belong to registered run')
    output.mkdir(parents=True,exist_ok=False)
    with socket.socket() as s:
        if s.connect_ex(('127.0.0.1',a.port))==0:raise RuntimeError('isolated master port already occupied')
    uri='http://127.0.0.1:'+str(a.port)
    os.environ['ROS_MASTER_URI']=uri;os.environ['ROS_IP']='127.0.0.1'
    master=node=player=None;handles=[]
    def start(cmd, name):
        f=(output/(name+'.log')).open('x');handles.append(f)
        return subprocess.Popen(cmd,stdout=f,stderr=f,start_new_session=True)
    try:
        master=start(['roscore','-p',str(a.port)],'master')
        deadline=time.monotonic()+15
        while True:
            try:
                if xmlrpc.client.ServerProxy(uri).getPid('/audit')[0]==1:break
            except OSError:pass
            if time.monotonic()>deadline:raise RuntimeError('isolated master not ready')
            time.sleep(.1)
        import rospy
        from asmc_msgs.msg import LidarObjectArray
        from diagnostic_msgs.msg import DiagnosticArray
        from sensor_msgs.msg import PointCloud2
        from std_msgs.msg import String
        rospy.set_param('/use_sim_time',True)
        rospy.init_node('lidar_hd_map_audit_trial',disable_signals=True)
        lock=threading.Lock();active={};subs=[]
        def receive(kind):
            def callback(msg):
                wall=time.monotonic()
                with lock:
                    if not active:return
                    if kind=='trace':
                        active['trace_file'].write(msg.data+'\n')
                        row=json.loads(msg.data)
                        if 'input_to_publish_ms' in row:active['latency'].append(row['input_to_publish_ms'])
                        report=row.get('hd_map_audit')
                        if report:active['audit_ms'].append(report['calculation_ms'])
                    elif kind=='diagnostics':
                        values={kv.key:kv.value for kv in msg.status[0].values}
                        active['diagnostic_file'].write(json.dumps({'wall_s':wall,'values':values})+'\n')
                        active['last_diagnostic']=values
                    else:
                        active[kind].append((wall,msg.header.stamp.to_nsec()))
            return callback
        for topic,kind,typ in [('/migration/lidar/trace','trace',String),
                               ('/migration/lidar/diagnostics','diagnostics',DiagnosticArray),
                               ('/migration/lidar/objects_local','local',LidarObjectArray),
                               ('/velodyne_points','input',PointCloud2)]:
            subs.append(rospy.Subscriber(topic,typ,receive(kind),queue_size=1 if kind=='local' else 10,buff_size=8*1024*1024))
        rows=[]
        for i in range(a.repeats):
            for mode in (('off','audit') if i%2==0 else ('audit','off')):
                name=mode+'-'+str(i+1);trace=(output/(name+'.jsonl')).open('x');diag=(output/(name+'-diagnostics.jsonl')).open('x')
                # Bounded in-memory metrics; complete trace/diagnostics stream to disk.
                with lock:
                    active.update(trace_file=trace,diagnostic_file=diag,latency=deque(maxlen=4096),
                                  audit_ms=deque(maxlen=4096),local=deque(maxlen=4096),input=deque(maxlen=4096),last_diagnostic={})
                node=start([a.node,'__name:=lidar_pipeline','__ns:=/migration/lidar',
                            '_pipeline_config:='+str(Path(a.config_dir)/('config-'+mode+'.yaml')),
                            '_package_dir:='+a.package_share,'_enable_camera_fusion:=false'],name+'-node')
                # Wait for subscriptions before the first bag frame (wall clock).
                deadline=time.monotonic()+20
                while not rospy.has_param('/run_id') or not active['last_diagnostic']:
                    if node.poll() is not None:raise RuntimeError('pipeline exited: '+name)
                    if time.monotonic()>deadline:raise RuntimeError('pipeline diagnostics missing: '+name)
                    time.sleep(.1)
                time.sleep(.5)
                player=start(['rosbag','play','--clock','--hz=100','--delay=1','--queue=10',a.bag,
                              '--topics','/velodyne_points','/imu','/Ego_topic','/gps'],name+'-player')
                rss=deque(maxlen=4096);start_wall=time.monotonic()
                while player.poll() is None:
                    if node.poll() is not None:raise RuntimeError('pipeline exited during bag: '+name)
                    try:
                        resident=int(Path('/proc/'+str(node.pid)+'/statm').read_text().split()[1])*os.sysconf('SC_PAGE_SIZE')
                        rss.append((time.monotonic()-start_wall,resident))
                    except FileNotFoundError:pass
                    time.sleep(.25)
                if player.returncode:raise RuntimeError('bag player failed: '+name)
                time.sleep(.5);stop(node);node=None
                with lock:
                    data={k:list(active[k]) for k in ('local','input','latency','audit_ms')}
                    last=active['last_diagnostic'];active.clear();trace.close();diag.close()
                times=[p[0] for p in data['local']];stamps=[p[1] for p in data['local']]
                gaps=[1000*(b-a) for a,b in zip(times,times[1:])]
                def windows(v):
                    if not v:return []
                    t0=v[0][0];return [sum(t0+5*j<=t<t0+5*(j+1) for t,_ in v)/5 for j in range(int((v[-1][0]-t0)//5))]
                steady=[v for t,v in rss if t>10]
                row={'mode':mode,'repeat':i+1,'published':len(stamps),'input_callbacks':len(data['input']),
                     'input_to_publish_ms_p95':percentile(data['latency'],.95),'input_to_publish_ms_p99':percentile(data['latency'],.99),
                     'subscriber_gap_ms_max':max(gaps,default=0),'input_gap_ms_max':max([1000*(b[0]-a[0]) for a,b in zip(data['input'],data['input'][1:])],default=0),
                     'duplicate_or_regressing_stamps':sum(b<=a for a,b in zip(stamps,stamps[1:])),
                     'full_5s_output_hz':windows(data['local']),'full_5s_input_hz':windows(data['input']),
                     'audit_ms_p95':percentile(data['audit_ms'],.95),
                     'rss_max_bytes':max((v for _,v in rss),default=0),
                     'rss_late_minus_early_bytes':percentile(steady[-40:],.5)-percentile(steady[:40],.5) if len(steady)>=80 else None,
                     'rss_samples':list(rss),'last_diagnostic':last}
                rows.append(row);(output/(name+'-metrics.json')).write_text(json.dumps(row,indent=2)+'\n')
                print(json.dumps({k:v for k,v in row.items() if k not in ('last_diagnostic','rss_samples','full_5s_input_hz','full_5s_output_hz')}),flush=True)
        pairs=[]
        for i in range(1,a.repeats+1):
            off=next(r for r in rows if r['repeat']==i and r['mode']=='off');on=next(r for r in rows if r['repeat']==i and r['mode']=='audit')
            pairs.append({'repeat':i,'p95_increase_ms':on['input_to_publish_ms_p95']-off['input_to_publish_ms_p95'],
                          'gap_increase_ms':on['subscriber_gap_ms_max']-off['subscriber_gap_ms_max']})
        summary={'run_id':a.run_id,'pairs':pairs,'runs':rows,
                 'cost_pass':all(p['p95_increase_ms']<=5 for p in pairs) and all(r['audit_ms_p95']<=5 for r in rows if r['mode']=='audit'),
                 'new_stamp_errors':any(r['duplicate_or_regressing_stamps'] for r in rows),
                 'note':'Paced stored bag, isolated ROS master; no MORAI or vehicle-control publisher. GPS/input faults remain visible.'}
        (output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    finally:
        stop(player);stop(node);stop(master)
        for f in handles:f.close()

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('run-id','output','node','bag','package-share','config-dir'):p.add_argument('--'+key,required=True)
    p.add_argument('--port',type=int,default=11434);p.add_argument('--repeats',type=int,default=3)
    main(p.parse_args())
