#!/usr/bin/env python3
"""Same-input native M1 tunnel optimization comparison; no ROS publishers.

Use a registered run. Strictly compare all trace behavior (including fusion and
wall decisions); remove only listed timing/build identity/cache counters. Replay
processing times are not live input-to-publish latency or real-time acceptance.
"""
import argparse
from collections import defaultdict
import hashlib
import json
import os
from pathlib import Path
import statistics
import subprocess
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from audit_three_view_fusion import sha,stats,measure
import yaml

DIAGNOSTICS={'timings_ms','producer_session_id','config_id','calibration_id',
             'calculation_ms','comparison_ms','total_ms','wait_ms','residual_clusters_reused'}
def behavior(value):
    if isinstance(value,dict):return {k:behavior(v) for k,v in value.items() if k not in DIAGNOSTICS}
    if isinstance(value,list):return [behavior(v) for v in value]
    return value

def main(a):
    registration=json.loads((a.run/'registration.json').read_text())
    if registration['run_id']!=a.run_id:raise ValueError('registered run ID mismatch')
    inputs=json.loads((a.run/'inputs.json').read_text())
    for i in inputs:
        if sha(a.storage/i['path'])!=i['sha256']:raise ValueError('input changed')
    output=a.run/a.analysis_id;output.mkdir(exist_ok=False)
    config=yaml.safe_load((a.repo/'src/asmc_lidar/config/lidar_pipeline.yaml').read_text())
    for mode in ('front','three'):
        config['fusion']['views']='front' if mode=='front' else 'all'
        config['motion']['semantic_static_enabled']=False
        (output/(mode+'.yaml')).write_text(yaml.safe_dump(config,sort_keys=False))
    result=dict(run_id=a.run_id,excluded_diagnostic_keys=sorted(DIAGNOSTICS),runs=[],
                binaries={str(b):sha(b) for b in [a.before,a.after]},
                libraries={name:{p.name:sha(p) for p in lib.glob('*.so')} for name,lib in [('before',a.before_lib),('after',a.after_lib)]},
                live_supply_accepted=False)
    expected={}
    for item in inputs:
        dataset=Path(item['path']).parent.name
        for repeat in range(a.repeats):
            variants=[('front','before'),('front','after'),('three','before'),('three','after')]
            if repeat%2:variants.reverse()
            for mode,version in variants:
                run=output/f'{dataset}-{repeat}-{mode}-{version}';run.mkdir()
                binary=getattr(a,version);env=os.environ.copy()
                env['LD_LIBRARY_PATH']=str(getattr(a,version+'_lib'))+':'+env.get('LD_LIBRARY_PATH','')
                command=[str(binary),'--bag',str(a.storage/item['path']),'--output',str(run/'trace.jsonl'),
                    '--config',str(output/(mode+'.yaml')),'--package-share',str(a.repo/'src/asmc_lidar'),
                    '--camera-calibration',str(a.repo/'src/asmc_camera/config/cameras_v3.yaml'),
                    '--camera-observations-topic','/perception/camera/objects/observations',
                    '--camera-status-topic','/perception/camera/objects/status','--fusion-diagnostics','on']
                with (run/'replay.log').open('x') as log:
                    subprocess.run([sys.executable,__file__,'--measure-child',str(run/'resource.json'),*command],
                        env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
                digest=hashlib.sha256();times=defaultdict(list);scans=reuse=0
                with (run/'trace.jsonl').open() as stream:
                    for line in stream:
                        r=json.loads(line);scans+=1
                        digest.update(json.dumps(behavior(r),sort_keys=True,separators=(',',':')).encode())
                        if r.get('timings_ms'):times['pipeline'].append(r['timings_ms']['total'])
                        for key,value in r.get('tunnel',{}).get('timings_ms',{}).items():times[key].append(value)
                        reuse+=bool(r.get('tunnel',{}).get('residual_clusters_reused'))
                key=(dataset,mode);fingerprint=digest.hexdigest()
                expected.setdefault(key,fingerprint)
                row=dict(dataset=dataset,repeat=repeat,mode=mode,version=version,scans=scans,
                    behavior_sha256=fingerprint,equal=fingerprint==expected[key],reused_scans=reuse,
                    times={k:stats(v) for k,v in times.items()},**json.loads((run/'resource.json').read_text()))
                result['runs'].append(row)
                (output/'summary.json').write_text(json.dumps(result,indent=2))
                print(json.dumps({k:v for k,v in row.items() if k!='times'},sort_keys=True),flush=True)
    result['all_behavior_equal']=all(r['equal'] for r in result['runs'])
    (output/'summary.json').write_text(json.dumps(result,indent=2))
    return 0 if result['all_behavior_equal'] else 1

if __name__=='__main__':
    if len(sys.argv)>1 and sys.argv[1]=='--measure-child':sys.exit(measure(sys.argv[3:],Path(sys.argv[2])))
    p=argparse.ArgumentParser(description=__doc__)
    for field in ('repo','run','storage','before','after','before-lib','after-lib'):p.add_argument('--'+field,type=Path,required=True)
    p.add_argument('--run-id',required=True);p.add_argument('--analysis-id',required=True)
    p.add_argument('--repeats',type=int,default=3)
    sys.exit(main(p.parse_args()))
