#!/usr/bin/env python3
"""Isolated bag policy trials with identical model/rate/masks, preserving every failure."""
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
from run_coco7_live_trials import provenance

ROOT=Path(__file__).resolve().parents[2]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--runtime',type=Path,required=True)
    p.add_argument('--models',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--revision',required=True)
    p.add_argument('--repeats',type=int,default=3)
    a=p.parse_args()
    if os.environ.get('ROS_MASTER_URI')!='http://127.0.0.1:11314':p.error('isolated ROS master11314 required')
    a.output.mkdir(parents=True,exist_ok=False)
    c=yaml.safe_load((ROOT/'config/experiments/camera_lidar_b0_v1.yaml').read_text())
    c['conditions'].update(ros_master_port=11314,simulator='running outside isolated container',target_hz=dict(yolo=10.,twinlite=10.,resnet18=10.))
    config=a.output/'conditions.yaml';config.write_text(yaml.safe_dump(c))
    runtime=yaml.safe_load(a.runtime.read_text());engine=Path(runtime['weights'])
    (a.output/'provenance.json').write_text(json.dumps(provenance(a.models,engine,a.revision),indent=2)+'\n')
    result=[]
    jobs=[dict(bag_id=bag,policy=policy,repeat=r,diagnostic=False) for r in range(1,a.repeats+1)
          for bag in ('primary','shadow') for policy in ('unique','bounded_reuse')]
    jobs += [dict(bag_id=bag,policy=policy,repeat=1,diagnostic=True) for bag in ('primary','shadow') for policy in ('unique','bounded_reuse')]
    for job in jobs:
        label='{bag_id}-{policy}-r{repeat}'.format(**job)+('-trace' if job['diagnostic'] else '')
        core=child=None
        with (a.output/(label+'.core.log')).open('w') as cl,(a.output/(label+'.trial.log')).open('w') as log:
            try:
                try:ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/coco7_fusion')
                except (OSError,ConnectionError):pass
                else:raise RuntimeError('refusing an existing isolated master')
                core=subprocess.Popen(['roscore','-p','11314'],stdout=cl,stderr=subprocess.STDOUT,start_new_session=True)
                deadline=time.monotonic()+15
                while True:
                    try:ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/coco7_fusion');break
                    except (OSError,ConnectionError):
                        if core.poll() is not None or time.monotonic()>deadline:raise RuntimeError('master startup failed')
                        time.sleep(.1)
                command=[sys.executable,str(ROOT/'tools/perception/run_camera_lidar_b0.py'),
                    '--config',str(config),'--phase','fusion','--profile','C1','--bag-id',job['bag_id'],
                    '--repeat',str(job['repeat']),'--output',str(a.output/label),
                    '--objects-config',str(a.runtime),'--objects-views','all','--objects-side-ego-mask',
                    '--objects-min-rate','9.5','--fusion-yolo-policy',job['policy']]
                if job['diagnostic']:command.append('--diagnostic')
                child=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                child.wait()
                path=a.output/label/'report.json';report=json.loads(path.read_text()) if path.exists() else None
                row=dict(job=job,path=label,return_code=child.returncode,complete=bool(report),passed=bool(report and report['passed']))
                if report:row.update(fusion=report.get('fusion'),checks=report['checks'],failures=report['failures'])
                result.append(row)
            except Exception as exc:result.append(dict(job=job,path=label,complete=False,passed=False,error=str(exc)))
            finally:
                if child:stop(child)
                if core:stop(core)
                (a.output/'index.json').write_text(json.dumps(result,indent=2)+'\n')
                print(json.dumps(result[-1]),flush=True)
    return 0 if all(r['complete'] for r in result) else 1


if __name__=='__main__':raise SystemExit(main())
