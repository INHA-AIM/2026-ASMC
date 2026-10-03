#!/usr/bin/env python3
"""Frozen one-variable runs; inputs are read-only, outputs are never overwritten."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess


def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''): h.update(b)
    return h.hexdigest()


def main():
    p=argparse.ArgumentParser()
    p.add_argument('input',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--repo',type=Path,default=Path(__file__).resolve().parents[3])
    p.add_argument('--profiles',nargs='+')
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    exe=a.repo/'devel/lib/asmc_lidar/asmc_lidar_ground_live_offline'
    profiles=sorted((a.repo/'src/asmc_lidar/config/ground_v2').glob('*.yaml'))
    if a.profiles: profiles=[x for x in profiles if x.stem in a.profiles]
    if not profiles: raise ValueError('no profiles')
    for profile in profiles:
        name=profile.stem;method='all' if name=='v1' else 'patchwork' if name.startswith('pp_') else 'linefit'
        out=a.output/(name+'.jsonl'); manifest=a.output/(name+'.manifest.json')
        if out.exists() or manifest.exists(): raise FileExistsError(out)
        cmd=[str(exe),str(a.input/'snapshots.jsonl'),str(a.repo/'src/asmc_lidar/config/autonav_v3.yaml'),str(out),'guard25',method,str(profile)]
        record={'command':cmd,'profile_id':name,'sha256':{'profile':sha(profile),'binary':sha(exe),'snapshots':sha(a.input/'snapshots.jsonl')},'complete':False}
        manifest.write_text(json.dumps(record,indent=2)+'\n')
        print('starting '+name,flush=True)
        with (a.output/(name+'.log')).open('x') as log: subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT,check=True)
        record['complete']=True;record['sha256']['result']=sha(out)
        manifest.write_text(json.dumps(record,indent=2)+'\n');print('finished '+name,flush=True)


if __name__=='__main__':main()
