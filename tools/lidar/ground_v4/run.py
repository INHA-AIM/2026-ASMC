#!/usr/bin/env python3
"""Frozen V4 candidates: replay the same admitted scans; never overwrite evidence."""
import argparse
import json
from pathlib import Path
import subprocess
import sys

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ground_v3'))
from run import digest

def main():
    p=argparse.ArgumentParser()
    p.add_argument('input',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--profiles',nargs='+',default=['jaeho_height','patchwork_support'])
    a=p.parse_args();root=Path(__file__).resolve().parents[3]
    a.output.mkdir(parents=True,exist_ok=True)
    exe=root/'devel/lib/asmc_lidar/asmc_lidar_ground_live_offline'
    cal=root/'src/asmc_lidar/config/autonav_v3.yaml'
    for name in a.profiles:
        if name not in ['jaeho_height','jaeho_support','patchwork_support']:raise ValueError(name)
        cfg=root/'src/asmc_lidar/config/ground_v4'/(name+'.yaml')
        result=a.output/(name+'.jsonl');meta=a.output/(name+'.manifest.json')
        if result.exists() or meta.exists():raise FileExistsError(result)
        command=[str(exe),str(a.input/'snapshots.jsonl'),str(cal),str(result),'guard25',
            'jaeho' if name.startswith('jaeho') else 'patchwork',str(cfg)]
        record={'command':command,'complete':False,'sha256':{
            'input':digest(a.input/'snapshots.jsonl'),'config':digest(cfg),'binary':digest(exe),
            'calibration':digest(cal),
            'ground_backend_library':digest(root/'devel/lib/libasmc_ground_live_backend.so'),
            'core_library':digest(root/'devel/lib/libasmc_lidar_core.so')}}
        meta.write_text(json.dumps(record,indent=2)+'\n')
        with (a.output/(name+'.log')).open('x') as log:
            subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,check=True)
        record['complete']=True;record['sha256']['result']=digest(result)
        meta.write_text(json.dumps(record,indent=2)+'\n');print('finished '+name,flush=True)

if __name__=='__main__':main()
