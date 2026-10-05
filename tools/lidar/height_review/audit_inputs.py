#!/usr/bin/env python3
"""Hash/causality gate and GT asphalt visibility audit (not a filter)."""
import argparse
import json
import math
from pathlib import Path
import numpy as np
from prepare import rows, sha, dump

def main():
    p=argparse.ArgumentParser();p.add_argument('inputs',type=Path);p.add_argument('output',type=Path)
    a=p.parse_args();root=a.inputs
    manifest=json.loads((root/'manifest.json').read_text())
    for name,item in manifest['files'].items():
        f=root/name
        assert f.stat().st_size==item['bytes'] and sha(f)==item['sha256'], name
    result={'all_hashes_valid':True,'causality':{},'asphalt':[],
            'ring0_m':[1.0,4.6875],
            'flat_ground_first_return_m':{str(h):h/math.tan(math.radians(15)) for h in (1.77,1.80)},
            'caveat':'Analytic flat ground only. Historical Capture has no IMU; asphalt GT is not every ground class.'}
    for name in ('manual','stationary'):
        history={}
        for r in rows(root/name/'snapshots.jsonl'):
            stamp=r['scan_effective_stamp_ns'];arrival=r['scan_arrival_ns'];imu=r['imu_stamp_ns']
            assert 0<imu<=stamp<=arrival and r['imu_arrival_ns']<=arrival
            assert stamp-imu<=100000000 and stamp>history.get(r['phase'],0)
            history[r['phase']]=stamp
        result['causality'][name]={'scans':len(rows(root/name/'snapshots.jsonl')),'future_state_uses':0}
    for r in rows(root/'capture/snapshots.jsonl'):
        pts=np.fromfile(root/'capture'/r['cloud_path'],dtype='<f4').reshape(-1,4)
        with np.load(root/'capture'/(r['input_id']+'.labels.npz')) as archive:
            labels=archive['semantic']
        assert len(labels)==len(pts)
        radius=np.linalg.norm(pts[:,:2],axis=1)
        asphalt=(labels==127)&np.isfinite(pts).all(axis=1)&(radius>=1)
        count=int((asphalt&(radius<4.6875)).sum())
        result['asphalt'].append({'input_id':r['input_id'],'points':int(asphalt.sum()),
            'ring0_points':count,'minimum_radius_m':float(radius[asphalt].min()) if asphalt.any() else None,
            'median_z_m':float(np.median(pts[asphalt,2])) if asphalt.any() else None})
    result['frames_without_ring0_asphalt']=sum(r['ring0_points']==0 for r in result['asphalt'])
    dump(a.output,result)
    print(json.dumps({k:v for k,v in result.items() if k!='asphalt'}))

if __name__=='__main__':main()
