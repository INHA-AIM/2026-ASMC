#!/usr/bin/env python3
"""Serial native-resolution first-run trials. Failures remain in the index."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from xmlrpc.client import ServerProxy
import yaml
from run_b0_matrix import stop
from yoloe_l_measurements import first_report


def main():
    root=Path(__file__).resolve().parents[2]
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--jobs',type=Path,required=True,help='Explicit frozen JSON protocol; no best-run filtering')
    p.add_argument('--config',type=Path,default=root/'config/experiments/camera_yoloe_l_first_v1.yaml')
    a=p.parse_args();c=yaml.safe_load(a.config.read_text());port=c['conditions']['ros_master_port']
    if os.environ.get('ROS_MASTER_URI')!='http://127.0.0.1:'+str(port):p.error('isolated master URI required')
    a.output.mkdir(parents=True,exist_ok=False)
    jobs=json.loads(a.jobs.read_text());(a.output/'protocol.json').write_text(json.dumps(jobs,indent=2)+'\n')
    results=[]
    for j in jobs:
        label='{phase}-{bag_id}-{prompt}-{batch_mode}-{profile}-r{repeat}'.format(**j)+('-debug' if j.get('debug') else '')
        command=[sys.executable,str(root/'tools/perception/run_camera_lidar_b0.py'),
                 '--config',str(a.config),'--phase',j['phase'],'--profile',j['profile'],'--bag-id',j['bag_id'],
                 '--repeat',str(j['repeat']),'--output',str(a.output/label),
                 '--objects-config',j['runtime_config'],'--objects-class-groups-config',
                 str(root/'src/asmc_camera/config/camera/object_classes_yoloe_l_v1.yaml'),'--objects-views','all']
        if j.get('debug'):command+=['--debug','--seg-debug']
        if j.get('diagnostic'):command+=['--diagnostic']
        core=child=None
        print('START '+label,flush=True)
        with (a.output/(label+'.core.log')).open('w') as cl,(a.output/(label+'.trial.log')).open('w') as log:
            try:
                try:ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/yoloe_l_matrix')
                except (OSError,ConnectionError):pass
                else:raise RuntimeError('existing master; refusing takeover')
                core=subprocess.Popen(['roscore','-p',str(port)],stdout=cl,stderr=subprocess.STDOUT,start_new_session=True)
                deadline=time.monotonic()+15
                while True:
                    try:ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/yoloe_l_matrix');break
                    except (OSError,ConnectionError):
                        if core.poll() is not None or time.monotonic()>deadline:raise RuntimeError('master startup failed')
                        time.sleep(.1)
                child=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);child.wait()
                path=a.output/label/'report.json'
                report=json.loads(path.read_text()) if path.exists() else {}
                row=dict(label=label,job=j,command=command,return_code=child.returncode,
                         report_exists=bool(report),feasibility=first_report(report))
                results.append(row);(a.output/'index.json').write_text(json.dumps(results,indent=2)+'\n')
                print('END '+json.dumps(dict(label=label,passed=row['feasibility']['passed'],failures=row['feasibility']['failures'])),flush=True)
            finally:
                if child:stop(child)
                if core:stop(core)
    return 0 if all(r['report_exists'] for r in results) else 1


if __name__=='__main__':raise SystemExit(main())
