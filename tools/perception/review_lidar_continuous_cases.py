#!/usr/bin/env python3
"""Retain preselected raw scan correspondences; figures are diagnostic, not GT."""
import argparse
import gzip
import json
from pathlib import Path
import cv2
import numpy as np


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root',type=Path,required=True);p.add_argument('--cases',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    selection=json.loads(args.cases.read_text()); cases=selection['cases']; collected={}
    profiles=('legacy','common_pitch','common_roll_pitch')
    for bag in ('primary','shadow'):
        chosen={r['raw_lidar_stamp_ns']:r for r in cases if '/case' in r['case'] and bag in r['case']}
        for profile in profiles:
            lookup={(ns//1000000*1000000 if profile=='legacy' else ns):ns for ns in chosen}
            if len(lookup)!=len(chosen):raise ValueError('ambiguous millisecond case selection')
            with gzip.open(args.root/f'{bag}-{profile}-normal-r1.trace.jsonl.gz','rt') as f:
                for line in f:
                    if not any('"event":"'+event+'"' in line[:100] for event in ('ground_points','tracking_state')):continue
                    row=json.loads(line);ns=lookup.get(row['stamp_ns'])
                    if ns is not None:collected.setdefault(ns,{}).setdefault(profile,{})[row['event']]=row
    reports=[]
    for case in cases:
        ns=case['raw_lidar_stamp_ns']; data=collected.get(ns,{})
        if any(set(data.get(profile,{}))!={'ground_points','tracking_state'} for profile in profiles):
            raise ValueError('incomplete fixed case '+case['case'])
        report=dict(case,independent_GT=False,profiles={})
        canvas=np.full((740,1500,3),255,np.uint8)
        for column,profile in enumerate(profiles):
            pts=np.asarray(data[profile]['ground_points']['points']);kept=pts[:,7]>0
            objects=[o for o in data[profile]['tracking_state']['objects'] if o['semantic']['lidar_misses']==0]
            report['profiles'][profile]=dict(voxel_points=len(pts),kept_points=int(kept.sum()),
                plane=data[profile]['ground_points']['plane'],plane_valid=data[profile]['ground_points']['plane_valid'],
                measured_objects=len(objects))
            cv2.putText(canvas,profile,(column*500+12,25),cv2.FONT_HERSHEY_SIMPLEX,.65,(0,0,0),1)
            for row,axis,limits in ((0,2,(-4,8)),(1,1,(-15,15))):
                origin_y=50+row*325
                cv2.rectangle(canvas,(column*500+20,origin_y),(column*500+480,origin_y+280),(0,0,0),1)
                for is_kept,color in ((False,(180,180,180)),(True,(30,150,30))):
                    xyz=pts[kept==is_kept,1:4]
                    visible=(xyz[:,0]>=0)&(xyz[:,0]<=60)&(xyz[:,axis]>=limits[0])&(xyz[:,axis]<=limits[1])
                    xyz=xyz[visible];u=(column*500+20+xyz[:,0]/60*460).astype(int)
                    v=(origin_y+280-(xyz[:,axis]-limits[0])/(limits[1]-limits[0])*280).astype(int)
                    canvas[v,u]=color
                label='raw sensor X/Z' if axis==2 else 'raw sensor X/Y'
                cv2.putText(canvas,label,(column*500+20,origin_y+300),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,0,0),1)
        name=case['case'].replace('/','-')
        cv2.putText(canvas,'grey: input voxel centroids, green: retained; no independent object GT',(15,730),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,0,0),1)
        cv2.imwrite(str(args.output/(name+'.png')),canvas)
        with gzip.open(args.output/(name+'.json.gz'),'wt') as f:json.dump(dict(selection=case,trace=data),f)
        reports.append(report)
    (args.output/'review.json').write_text(json.dumps(dict(selection=selection,cases=reports,
        scope='same raw source ns; legacy trace has explicit unique millisecond anchor; figures use raw centroid coordinates',
        acceptance=False,required_followup='independent visible-object labels and OBB/miss review'),indent=2)+'\n')

if __name__=='__main__':main()
