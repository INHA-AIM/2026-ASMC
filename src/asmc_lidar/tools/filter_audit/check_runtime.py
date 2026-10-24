#!/usr/bin/env python3
"""Registered-run verification: paired offline replay, immutable outputs, cost/RSS.

No ROS master or operational publisher. Replay Hz is throughput, not live supply.
The separate paced ROS trial measures callback->publish and subscriber gaps.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import time
import yaml


def digest_rows(path):
    digest = hashlib.sha256(); n = 0; processed = 0; costs = []; audits = []
    counts = {}; last = None
    for line in path.open():
        row = json.loads(line); n += 1
        # Configuration/session identity and timings intentionally differ. These
        # are the exact public objects and tracker states, not clipped shadows.
        stable = {k: row.get(k) for k in ('seq','stamp_ns','accepted','typed','tracks','counts')}
        digest.update((json.dumps(stable, sort_keys=True, separators=(',',':'))+'\n').encode())
        if row.get('processed'):
            processed += 1; costs.append(row['timings_ms']['total'])
        if row.get('hd_map_audit',{}).get('enabled'):
            a = row['hd_map_audit']; audits.append(a['calculation_ms'])
            for d in a['objects']: counts[d['decision']] = counts.get(d['decision'],0)+1
    def q(v,p):
        return sorted(v)[max(0,math.ceil(p*len(v))-1)] if v else None
    return {'scans':n,'processed':processed,'objects_and_tracks_sha256':digest.hexdigest(),
            'runtime_ms_p95':q(costs,.95),'audit_ms_p95':q(audits,.95),'audit_ms_max':max(audits,default=0),
            'decisions':counts}


def main(a):
    root=Path(a.output)
    if a.run_id not in root.parts:raise ValueError('output must belong to the registered run ID')
    root.mkdir(parents=True,exist_ok=False)
    pkg=Path(a.package_share);config=yaml.safe_load((pkg/'config/lidar_pipeline.yaml').read_text())
    configs={}
    for mode in ('off','audit'):
        cfg=json.loads(json.dumps(config));cfg['hd_map_audit']={'mode':mode,'manifest':a.manifest,'manifest_sha256':a.manifest_sha256}
        target=root/('config-'+mode+'.yaml');target.write_text(yaml.safe_dump(cfg,sort_keys=False));configs[mode]=target
    results=[]
    for i in range(3):
        for mode in (('off','audit') if i%2==0 else ('audit','off')):
            name=mode+'-'+str(i+1);trace=root/(name+'.jsonl')
            cmd=[a.replay,'--bag',a.bag,'--output',str(trace),'--config',str(configs[mode]),'--package-share',str(pkg)]
            with (root/(name+'.log')).open('x') as log:
                start=time.monotonic();p=subprocess.Popen(cmd,stdout=log,stderr=log)
                _,status,usage=os.wait4(p.pid,0);p.returncode=os.WEXITSTATUS(status)
                if p.returncode:raise RuntimeError('replay failed: '+name)
            value=digest_rows(trace);value.update(mode=mode,repeat=i+1,wall_s=time.monotonic()-start,max_rss_kib=usage.ru_maxrss)
            results.append(value);print(json.dumps(value),flush=True)
    reference=results[0]['objects_and_tracks_sha256']
    assert all(r['objects_and_tracks_sha256']==reference for r in results), 'object/track mutation'
    pairs=[]
    for i in range(1,4):
        off=next(r for r in results if r['repeat']==i and r['mode']=='off')
        on=next(r for r in results if r['repeat']==i and r['mode']=='audit')
        pairs.append(on['runtime_ms_p95']-off['runtime_ms_p95'])
    summary={'run_id':a.run_id,'runs':results,'immutable_objects_and_tracks':True,'p95_runtime_increases_ms':pairs,
             'offline_cost_pass':all(r['audit_ms_p95']<=5 for r in results if r['mode']=='audit') and max(pairs)<=5,
             'live_supply_acceptance':False,'note':'unpaced replay; not real-time publication evidence'}
    (root/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('run-id','output','replay','bag','package-share','manifest','manifest-sha256'):p.add_argument('--'+key,required=True)
    main(p.parse_args())
