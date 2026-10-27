#!/usr/bin/env python3
"""M1 native replay matrix. Run inside a registered run; never publishes ROS data.

Alternates rollback/front, three-view and semantic prior candidates. Matching is
performed by the native binary, not reimplemented here. Logical replay timing is
not live latency/supply acceptance. Original trace files are retained for review.
"""
import argparse
from collections import Counter
import copy
import hashlib
import html
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import yaml


def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for b in iter(lambda:f.read(8*1024*1024),b''): h.update(b)
    return h.hexdigest()


def stats(values):
    a=sorted(values)
    return dict(count=len(a),**{k:(a[min(len(a)-1,int((len(a)-1)*q))] if a else None)
                             for k,q in [('p50',.5),('p95',.95),('p99',.99),('max',1)]})


def behavior(row):
    """Compare every existing non-diagnostic field, including delivered objects."""
    row=copy.deepcopy(row)
    for key in ('producer_session_id','config_id','calibration_id','timings_ms','fusion'):
        row.pop(key,None)
    def clean(x):
        if isinstance(x,dict):
            for k in list(x):
                if k in ('config_id','calibration_id','preview_config_id','calculation_ms','comparison_ms','total_ms','wait_ms',
                         'semantic_static_applied','semantic_reason','group_conflict','protection_group'):
                    x.pop(k)
                else:clean(x[k])
        elif isinstance(x,list):
            for v in x:clean(v)
    clean(row)
    return row


def measure(command, output):
    start=time.perf_counter();child=subprocess.Popen(command)
    _,status,usage=os.wait4(child.pid,0)
    child.returncode=os.WEXITSTATUS(status) if os.WIFEXITED(status) else -os.WTERMSIG(status)
    output.write_text(json.dumps(dict(wall_s=time.perf_counter()-start,user_s=usage.ru_utime,
        system_s=usage.ru_stime,rss_kib=usage.ru_maxrss)))
    return child.returncode


def main(a):
    registration=json.loads((a.run/'registration.json').read_text())
    if registration['run_id']!=a.run_id:raise ValueError('run ID mismatch')
    output=a.run/a.analysis_id;output.mkdir(exist_ok=False)
    inputs=json.loads((a.run/'inputs.json').read_text())
    for item in inputs:
        if sha(a.storage/item['path'])!=item['sha256']:raise ValueError('input changed')
    base=yaml.safe_load((a.repo/'src/asmc_lidar/config/lidar_pipeline.yaml').read_text())
    for mode in ('front','three','semantic'):
        cfg=copy.deepcopy(base);cfg['fusion']['views']='front' if mode=='front' else 'all'
        cfg['motion']['semantic_static_enabled']=mode=='semantic'
        (output/(mode+'.yaml')).write_text(yaml.safe_dump(cfg,sort_keys=False))
    summary=dict(run_id=a.run_id,runs=[],parity=[],limits=['No live supply or latency acceptance',
        'No independent detection/motion GT; side-view matches need RGB review'],
        binaries={str(p):sha(p) for p in (a.before,a.after)},
        libraries={kind:{p.name:sha(p) for p in lib.glob('libasmc_lidar*.so')} for kind,lib in [('before',a.before_lib),('after',a.after_lib)]})
    for item in inputs:
        name=Path(item['path']).parent.name
        canonical=None
        for repeat in range(a.repeats):
            variants=['before','front','three','semantic']
            if repeat%2:variants.reverse()
            for mode in variants:
                directory=output/('%s-%d-%s'%(name,repeat,mode));directory.mkdir()
                config=output/('front.yaml' if mode=='before' else mode+'.yaml')
                binary=a.before if mode=='before' else a.after
                env=os.environ.copy();env['LD_LIBRARY_PATH']=str(a.before_lib if mode=='before' else a.after_lib)+':'+env.get('LD_LIBRARY_PATH','')
                command=[str(binary),'--bag',str(a.storage/item['path']),'--output',str(directory/'trace.jsonl'),
                    '--config',str(config),'--package-share',str(a.repo/'src/asmc_lidar'),
                    '--camera-calibration',str(a.repo/'src/asmc_camera/config/cameras_v3.yaml'),
                    '--camera-observations-topic','/perception/camera/objects/observations',
                    '--camera-status-topic','/perception/camera/objects/status','--fusion-diagnostics','on']
                with (directory/'replay.log').open('x') as log:
                    subprocess.run([sys.executable,__file__,'--measure-child',str(directory/'resource.json'),*command],
                        env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
                digest=hashlib.sha256();times=[];matches=Counter();reasons=Counter();semantic=Counter();scans=0
                with (directory/'trace.jsonl').open() as f:
                    for line in f:
                        r=json.loads(line);scans+=1;digest.update(json.dumps(behavior(r),sort_keys=True).encode())
                        if r.get('timings_ms'):times.append(r['timings_ms']['total'])
                        for d in r.get('fusion',{}).get('decisions',[]):
                            reasons[d['action']+':'+d['reason']]+=1
                            if d['action']=='match': matches[d['camera_id']]+=len(d.get('associations',[]))
                        for t in r.get('tracks',[]):semantic[t.get('semantic_reason','baseline')]+=1
                result=dict(dataset=name,repeat=repeat,variant=mode,scans=scans,behavior_sha=digest.hexdigest(),
                    processing_ms=stats(times),associations=dict(matches),reasons=dict(reasons),semantic=dict(semantic),
                    **json.loads((directory/'resource.json').read_text()))
                summary['runs'].append(result)
                if mode in ('before','front'):
                    if canonical is None:canonical=result['behavior_sha']
                    summary['parity'].append(dict(dataset=name,repeat=repeat,variant=mode,equal=result['behavior_sha']==canonical))
                print(json.dumps(result),flush=True)
                (output/'summary.json').write_text(json.dumps(summary,indent=2))
    summary['rollback_equal']=all(r['equal'] for r in summary['parity'])
    summary['candidate_default_accepted']=False
    (output/'summary.json').write_text(json.dumps(summary,indent=2))
    (output/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>M1 native replay</title>'
        '<style>body{font:16px sans-serif;background:#15202b;color:#eee;margin:2em}pre{white-space:pre-wrap}</style>'
        '<h1>M1 3뷰·정적 보조 대조</h1><p>기록 도착 순서의 native replay. 실시간 수락·독립 GT 평가가 아닙니다.</p>'
        '<pre>'+html.escape(json.dumps(summary,indent=2,ensure_ascii=False))+'</pre>')
    return 0 if summary['rollback_equal'] else 1


if __name__=='__main__':
    if len(sys.argv)>1 and sys.argv[1]=='--measure-child':sys.exit(measure(sys.argv[3:],Path(sys.argv[2])))
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('repo','run','storage','before','after','before-lib','after-lib'):p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--run-id',required=True);p.add_argument('--analysis-id',required=True)
    p.add_argument('--repeats',type=int,default=3)
    sys.exit(main(p.parse_args()))
