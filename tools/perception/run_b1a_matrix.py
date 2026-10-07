#!/usr/bin/env python3
"""Run pinned B1a comparisons serially, retaining failed attempts and closures."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from xmlrpc.client import ServerProxy
import yaml
from b1a_protocol import jobs
from run_b0_matrix import stop


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--fixtures',type=Path,required=True)
    parser.add_argument('--source',type=Path,default=Path('/root/filter_ws'))
    parser.add_argument('--phases',default='offline,realtime')
    parser.add_argument('--smoke',action='store_true')
    parser.add_argument('--revision',required=True,help='host worktree commit; recorded alongside tool SHA')
    args=parser.parse_args()
    root=Path(__file__).resolve().parents[2]
    config=yaml.safe_load((root/'config/experiments/camera_lidar_b1a_v1.yaml').read_text())
    port=config['conditions']['ros_master_port']; uri='http://127.0.0.1:%d'%port
    if os.environ.get('ROS_MASTER_URI')!=uri: parser.error('isolated master required')
    args.output.mkdir(parents=True,exist_ok=False)
    sources=[root/'config/experiments/camera_lidar_b1a_v1.yaml']+list((root/'tools/perception').glob('*b1a*.py'))+list((root/'tools/perception/b1a_probe').glob('*'))
    manifest=dict(config=config,revision=args.revision,code_sha256={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources if p.is_file()},
                  source=json.loads((args.source/'source-manifest.json').read_text()),
                  fixtures=json.loads((args.fixtures/'manifest.json').read_text()),
                  binary_sha256={name:hashlib.sha256((args.source/'devel/lib/Main'/name).read_bytes()).hexdigest() for name in ('main','tunnel_filter_offline_eval')},
                  probe_binary_sha256=hashlib.sha256((args.source/'devel/lib/b1a_probe/b1a_protection_probe').read_bytes()).hexdigest(),
                  generated_launch_sha256=hashlib.sha256((args.source/'b1a_main.launch').read_bytes()).hexdigest())
    (args.output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    protocol=[job for phase in args.phases.split(',') for job in jobs(config,phase)]
    if args.smoke: protocol=[dict(phase='realtime',bag_id='shadow',gps_mode='invalid',profile='T3',repeat=0)]
    (args.output/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n')
    rows=[]
    for job in protocol:
        label='{phase}-{bag_id}-{profile}-{gps_mode}-r{repeat}'.format(**job)
        print('START '+label,flush=True); core=child=None
        with (args.output/(label+'.core.log')).open('w') as corelog,(args.output/(label+'.trial.log')).open('w') as triallog:
            try:
                try: ServerProxy(uri).getPid('/b1a_matrix')
                except (OSError,ConnectionError): pass
                else: raise RuntimeError('refuse to reuse an existing master')
                core=subprocess.Popen(['roscore','-p',str(port)],stdout=corelog,stderr=subprocess.STDOUT,start_new_session=True)
                deadline=time.monotonic()+15
                while True:
                    try: ServerProxy(uri).getPid('/b1a_matrix'); break
                    except (OSError,ConnectionError):
                        if core.poll() is not None or time.monotonic()>deadline: raise RuntimeError('master startup failed')
                        time.sleep(.05)
                command=[sys.executable,str(root/'tools/perception/run_b1a_trial.py'),'--job',json.dumps(job),
                         '--output',str(args.output/label),'--fixtures',str(args.fixtures),'--source',str(args.source)]
                child=subprocess.Popen(command,stdout=triallog,stderr=subprocess.STDOUT,start_new_session=True)
                child.wait()
                report=args.output/label/'report.json'
                rows.append(dict(label=label,job=job,return_code=child.returncode,report_exists=report.exists(),
                                 complete=json.loads(report.read_text())['complete'] if report.exists() else False))
            except Exception as exc:
                rows.append(dict(label=label,job=job,error=str(exc),report_exists=False,complete=False))
            finally:
                if child: stop(child)
                if core: stop(core)
                (args.output/'index.json').write_text(json.dumps(rows,indent=2)+'\n')
        print('END '+json.dumps(rows[-1]),flush=True)
    return 0 if all(r['complete'] for r in rows) else 1


if __name__=='__main__': raise SystemExit(main())
