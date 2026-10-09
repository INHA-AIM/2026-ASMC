#!/usr/bin/env python3
"""Fixed serial seg comparison: all models resident, fresh isolated master per run."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from urllib.parse import urlparse
from xmlrpc.client import ServerProxy
import yaml
from run_b0_matrix import manifest, stop, sha
from seg_measurements import make_protocol


def main():
    root=Path(__file__).resolve().parents[2]
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--revision',required=True)
    p.add_argument('--config',type=Path,default=root/'config/experiments/camera_seg_selection_v1.yaml')
    p.add_argument('--phases',default='camera,debug,fusion')
    p.add_argument('--limit',type=int,help='smoke protocol only; never final evidence')
    a=p.parse_args()
    c=yaml.safe_load(a.config.read_text()); phases=a.phases.split(',')
    if set(phases)-{'camera','debug','fusion'}:p.error('unknown phase')
    uri=urlparse(os.environ.get('ROS_MASTER_URI',''));port=c['conditions']['ros_master_port']
    if uri.hostname not in ('localhost','127.0.0.1') or uri.port!=port:p.error('configured isolated master required')
    a.output.mkdir(parents=True,exist_ok=False)
    print('Fingerprinting fixed inputs',flush=True)
    metadata=manifest(root,c,a.revision)
    metadata['seg_weights_sha256']={str(path):sha(path) for path in Path('/data/models/camera_seg_v1').glob('*.pt')}
    metadata['prompts_sha256']=sha(Path('/data/models/camera_seg_v1/prompts-v1.npz'))
    metadata['text_encoder_sha256']=sha(Path('/data/models/camera_seg_v1/mobileclip2_b.ts'))
    extra=[a.config]+list((root/'tools/perception').glob('*seg*.py'))
    metadata['source_sha256'].update({str(path.relative_to(root)):sha(path) for path in extra})
    metadata.update(all_models_loaded=True,training_performed=False,smoke_only=a.limit is not None)
    (a.output/'manifest.json').write_text(json.dumps(metadata,indent=2)+'\n')
    jobs=make_protocol(c,phases)
    if a.limit is not None:jobs=jobs[:a.limit]
    (a.output/'protocol.json').write_text(json.dumps(jobs,indent=2)+'\n')
    results=[]
    for j in jobs:
        label='{phase}-{bag_id}-{model}-{views}-{profile}-r{repeat}'.format(**j)+('-debug' if j['debug'] else '')
        model=c['object_profiles'][j['model']]
        command=[sys.executable,str(root/'tools/perception/run_camera_lidar_b0.py'),
                 '--config',str(a.config),'--phase',j['phase'],'--profile',j['profile'],'--bag-id',j['bag_id'],
                 '--repeat',str(j['repeat']),'--output',str(a.output/label),
                 '--objects-config',str(root/model['config']),
                 '--objects-class-groups-config',str(root/model['groups']), '--objects-views',j['views']]
        if j['debug']:command+=['--debug','--seg-debug']
        core=child=None
        print('START '+label,flush=True)
        with (a.output/(label+'.core.log')).open('w') as cl, (a.output/(label+'.trial.log')).open('w') as log:
            try:
                try:ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/seg_matrix')
                except (OSError,ConnectionError):pass
                else:raise RuntimeError('master exists; refusing to take over another session')
                core=subprocess.Popen(['roscore','-p',str(port)],stdout=cl,stderr=subprocess.STDOUT,start_new_session=True)
                deadline=time.monotonic()+15
                while True:
                    try:ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/seg_matrix');break
                    except (OSError,ConnectionError):
                        if core.poll() is not None or time.monotonic()>deadline:raise RuntimeError('master startup failed')
                        time.sleep(.1)
                child=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                child.wait()
                path=a.output/label/'report.json'
                report=json.loads(path.read_text()) if path.exists() else None
                row=dict(label=label,job=j,command=command,return_code=child.returncode,
                         report_exists=report is not None,passed=bool(report and report['passed']))
                results.append(row)
                (a.output/'index.json').write_text(json.dumps(results,indent=2)+'\n')
                print('END '+json.dumps(row),flush=True)
            finally:
                if child:stop(child)
                if core:stop(core)
    return 0 if all(r['report_exists'] for r in results) else 1


if __name__=='__main__':raise SystemExit(main())
