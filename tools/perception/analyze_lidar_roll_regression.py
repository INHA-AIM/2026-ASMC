#!/usr/bin/env python3
"""Separate coordinate and roll effects, retaining unmatched actual observations."""
import argparse
import gzip
import json
import math
from pathlib import Path
import numpy as np
from lidar_continuous_measurements import speed_events


def records(path,event=None):
    with (gzip.open(path,'rt') if path.suffix=='.gz' else path.open()) as f:
        for line in f:
            if event and ('"event":"'+event+'"') not in line[:100]:continue
            yield json.loads(line)


def geometry_pairs(a,b,distance_limit=.25):
    # IDs are not comparable across executions. This diagnostic pairing never
    # claims independent GT; ambiguous or unmatched observations remain listed.
    aa=[o for o in a['objects'] if o['lidar_misses']==0]
    bb=[o for o in b['objects'] if o['lidar_misses']==0]
    if not aa or not bb:return [],len(aa),len(bb)
    d=np.array([[math.dist(x['center'],y['center']) for y in bb] for x in aa])
    paired=[]
    for i in range(len(aa)):
        j=int(d[i].argmin())
        if (d[i,j]<=distance_limit and int(d[:,j].argmin())==i
                and np.sum(np.abs(d[i]-d[i,j])<=1e-6)==1
                and np.sum(np.abs(d[:,j]-d[i,j])<=1e-6)==1):
            paired.append(dict(a_id=aa[i]['id'],b_id=bb[j]['id'],distance_m=float(d[i,j]),
                class_equal=(aa[i]['class'],aa[i]['raw_class'])==(bb[j]['class'],bb[j]['raw_class'])))
    return paired,len(aa)-len(paired),len(bb)-len(paired)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('root',type=Path)
    p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    result=dict(scope='fixed raw inputs; spatial correspondence is diagnostic, not GT',bags={})
    for bag in ('primary','shadow'):
        profiles={name:list(records(args.root/f'{bag}-{name}-normal-r1.jsonl'))
            for name in ('legacy','common_pitch','common_roll_pitch')}
        report=dict(profiles={},comparisons=[])
        for name,rows in profiles.items():
            trace=args.root/f'{bag}-{name}-normal-r1.trace.jsonl.gz'
            tracking=list(records(trace,'tracking_state'))
            jumps=speed_events(tracking)
            report['profiles'][name]=dict(frames=len(rows),ready=sum(r['ready'] for r in rows),
                ground_fit_valid=sum(r.get('plane_valid',False) for r in rows),
                speed_jumps_over_2mps=len(jumps),speed_jumps=jumps,
                new_track_observations=sum(o.get('kf_init_count')==1 for r in rows for o in r['objects']),
                velocity_accuracy_accepted=False)
        for old,new in (('legacy','common_pitch'),('common_pitch','common_roll_pitch')):
            # Ground/tracker coordinates differ across roll. Obtain transforms
            # from actual voxel input/output pairs, not current IMU substitution.
            transforms={}
            for name in (old,new):
                transforms[name]={}
                for row in records(args.root/f'{bag}-{name}-normal-r1.trace.jsonl.gz','ground_points'):
                    pts=np.asarray(row['points'],dtype=float)
                    if len(pts)<3:continue
                    rotation=np.linalg.lstsq(pts[:,1:4],pts[:,4:7],rcond=None)[0].T
                    transforms[name][row['stamp_ns']]=rotation
            ta={r['stamp_ns']:r for r in profiles[old]};tb={r['stamp_ns']:r for r in profiles[new]}
            pairs=[];missing_a=missing_b=0;class_diffs=[];transform_missing=[]
            for ns in sorted(ta.keys()&tb.keys()):
                ra,rb=ta[ns],tb[ns]
                if any((ns//1000000*1000000 if name=='legacy' else ns) not in transforms[name]
                       for name in (old,new)):
                    transform_missing.append(ns)
                    missing_a+=sum(o['lidar_misses']==0 for o in ra['objects'])
                    missing_b+=sum(o['lidar_misses']==0 for o in rb['objects'])
                    continue
                prepared=[]
                for name,row in ((old,ra),(new,rb)):
                    key=ns//1000000*1000000 if name=='legacy' else ns
                    R=transforms[name].get(key)
                    objects=[]
                    for o in row['objects']:
                        out=dict(o)
                        if R is not None:out['center']=(R.T@np.array(o['center'])+np.array([.8,0,1.45])).tolist()
                        objects.append(out)
                    prepared.append(dict(row,objects=objects))
                found,ma,mb=geometry_pairs(*prepared)
                pairs.extend(found);missing_a+=ma;missing_b+=mb
                class_diffs.extend(dict(stamp_ns=ns,**r) for r in found if not r['class_equal'])
            distances=[r['distance_m'] for r in pairs]
            report['comparisons'].append(dict(reference=old,candidate=new,matched_observations=len(pairs),
                reference_unmatched=missing_a,candidate_unmatched=missing_b,class_differences=class_diffs,
                missing_transform_stamps_ns=transform_missing,
                center_distance_p50_p95_m=np.quantile(distances,[.5,.95]).tolist() if distances else None,
                independent_object_preservation_accepted=False))
        result['bags'][bag]=report
    args.output.write_text(json.dumps(result,indent=2)+'\n')

if __name__=='__main__':main()
