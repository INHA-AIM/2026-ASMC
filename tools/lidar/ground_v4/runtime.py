#!/usr/bin/env python3
"""Run each frozen profile on a separate ROS master; no live bridge/control replay."""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
import xmlrpc.client

def stop(proc):
    if proc.poll() is None:
        proc.send_signal(signal.SIGINT)
        try:proc.wait(timeout=8)
        except subprocess.TimeoutExpired:
            proc.terminate();proc.wait(timeout=8)

def main():
    p=argparse.ArgumentParser()
    p.add_argument('bag',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--port',type=int,default=11431)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    root=Path(__file__).resolve().parents[3]
    with socket.socket() as sock:sock.bind(('127.0.0.1',a.port))
    env=dict(os.environ,ROS_MASTER_URI='http://127.0.0.1:'+str(a.port),ROS_IP='127.0.0.1',ROS_HOSTNAME='127.0.0.1')
    logs=[];processes=[]
    def launch(command,name):
        f=(a.output/(name+'.log')).open('x');logs.append(f)
        proc=subprocess.Popen(command,stdout=f,stderr=subprocess.STDOUT,env=env,start_new_session=True)
        processes.append(proc);return proc
    record={'isolated_master':env['ROS_MASTER_URI'],'bag':str(a.bag),'start_s':110,'rate':1.4,
        'replayed_topics':['/imu','/velodyne_points'],'complete':False}
    try:
        master=launch(['roscore','-p',str(a.port)],'master')
        api=xmlrpc.client.ServerProxy(env['ROS_MASTER_URI'])
        for _ in range(30):
            try:
                if api.getPid('/ground_v4_probe')[0]==1:break
            except OSError:pass
            time.sleep(.2)
        else:raise RuntimeError('isolated master not ready')
        for name,method,cfg in [
            ('jaeho_baseline','jaeho','ground_v3/roll_pitch_all.yaml'),
            ('jaeho_support','jaeho','ground_v4/jaeho_support.yaml'),
            ('patchwork_support','patchwork','ground_v4/patchwork_support.yaml')]:
            review=launch(['roslaunch','asmc_lidar','ground_compare_live.launch','rviz:=false','start_bridge:=false',
                'method:='+method,'comparison_config:='+str(root/'src/asmc_lidar/config'/cfg)],name+'_review')
            time.sleep(2)
            if review.poll() is not None:raise RuntimeError('review exited')
            probe=subprocess.Popen([sys.executable,'-B',str(root/'tools/lidar/ground_v3/live_probe.py'),
                str(a.output/(name+'.json')),'--duration','31','--method',method],
                stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,env=env,start_new_session=True)
            processes.append(probe)
            for line in probe.stdout:
                print(line.rstrip(),flush=True)
                if 'sampling live diagnostics' in line:break
            if probe.poll() is not None:raise RuntimeError('probe failed')
            bag=launch(['rosbag','play','--quiet','--start','110','--duration','50','--rate','1.4',str(a.bag),
                '--topics','/imu','/velodyne_points'],name+'_bag')
            output,_=probe.communicate(timeout=45);print(output,flush=True)
            if probe.returncode:raise RuntimeError('probe return code')
            stop(bag);stop(review)
            print('completed '+name,flush=True)
        record['complete']=True
    finally:
        for proc in reversed(processes):stop(proc)
        for stream in logs:stream.close()
        (a.output/'manifest.json').write_text(json.dumps(record,indent=2)+'\n')

if __name__=='__main__':main()
