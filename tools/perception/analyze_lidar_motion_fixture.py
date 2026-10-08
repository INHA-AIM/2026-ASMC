#!/usr/bin/env python3
"""Match independent synthetic objects by mutual unique distance, preserve misses."""
import argparse
import gzip
import json
import math
from pathlib import Path
import numpy as np


def read(path,event=None):
    opener=gzip.open if path.suffix=='.gz' else open
    with opener(path,'rt') as f:
        return [json.loads(line) for line in f if event is None or ('"event":"'+event+'"') in line[:100]]


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--truth',type=Path,required=True)
    p.add_argument('--profile',choices=['legacy','common_pitch','common_roll_pitch'],default='common_roll_pitch');p.add_argument('--trace',type=Path,required=True);p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    truth={}
    for r in json.loads(args.truth.read_text()):
        key=r['stamp_ns']//1000000*1000000 if args.profile=='legacy' else r['stamp_ns']
        if key in truth:raise ValueError('ambiguous legacy millisecond truth mapping')
        truth[key]=r
    states=read(args.trace,'tracking_state');grounds=read(args.trace,'ground_points');records=[]
    for row in states:
        gt=truth[row['stamp_ns']]
        # C++ trace centers are leveled sensor coordinates. Invert the exact
        # independent fixture matrix to compare in sensor/base coordinates.
        roll=gt['roll'] if args.profile=='common_roll_pitch' else 0.
        cr,sr=math.cos(roll),math.sin(roll);cp,sp=math.cos(gt['pitch']),math.sin(gt['pitch'])
        level=np.array([[cp,sp*sr,sp*cr],[0,cr,-sr],[-sp,cp*sr,cp*cr]])
        obs=[o for o in row['objects'] if o['semantic']['lidar_misses']==0]
        centers=[level.T@np.array(o['semantic']['center'])+np.array([.8,0,1.45]) for o in obs]
        targets=gt['objects']
        distance=np.array([[np.linalg.norm(a-np.array(g['center_base'])) for g in targets] for a in centers])
        for j,g in enumerate(targets):
            i=int(distance[:,j].argmin()) if len(obs) else None
            unique=(i is not None and int(distance[i].argmin())==j and distance[i,j]<1.
                and np.sum(np.abs(distance[:,j]-distance[i,j])<=1e-6)==1
                and np.sum(np.abs(distance[i]-distance[i,j])<=1e-6)==1)
            record=dict(stamp_ns=row['stamp_ns'],name=g['name'],matched=bool(unique),truth_speed_mps=g['absolute_speed_mps'],
                quantitative_speed_truth=gt.get('quantitative_speed_truth',False))
            if unique:
                o=obs[i];record.update(id=o['semantic']['id'],center_error_m=float(distance[i,j]),
                    legacy_speed_mps=o['legacy_speed_mps'],kf_init_count=o['kf_init_count'],
                    speed_error_mps=abs(o['legacy_speed_mps']-g['absolute_speed_mps']),
                    scope='legacy scalar diagnostic, not validated public velocity')
            records.append(record)
    errors=[r['speed_error_mps'] for r in records if r['matched'] and r['kf_init_count']>=2 and r['quantitative_speed_truth']]
    report=dict(records=records,frames=len(states),expected_frames=len(truth),
        unmatched=len([r for r in records if not r['matched']]),
        straight_truth_speed_error_p50_p95_mps=np.quantile(errors,[.5,.95]).tolist() if errors else None,
        ground_valid_frames=sum(r['plane_valid'] for r in grounds),
        public_velocity_validity=False,
        limitation='synthetic static/moving and attitude fixture, no claim of real bag speed accuracy')
    args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='records'}))

if __name__=='__main__':main()
