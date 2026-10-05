#!/usr/bin/env python3
"""Observed asphalt-mask changes, NOT motion-compensated surface stability GT."""
import argparse
import itertools
import json
from pathlib import Path
import numpy as np

def main():
    p=argparse.ArgumentParser()
    for name in ['input','baseline','candidate','output']:
        p.add_argument(name,type=Path)
    a=p.parse_args()
    paths=[a.input/'snapshots.jsonl',a.baseline,a.candidate/'jaeho_height.jsonl',
           a.candidate/'jaeho_support.jsonl',a.candidate/'patchwork_support.jsonl']
    streams=[path.open() for path in paths]
    values={};changes={};previous={};counts={};frames=0
    for lines in itertools.zip_longest(*streams):
        if None in lines:raise ValueError('stream length mismatch')
        src,base,height,support,pw=map(json.loads,lines)
        if any(row['input_id']!=src['input_id'] for row in [base,height,support,pw]):
            raise ValueError('input mismatch')
        with np.load(a.input/'labels'/(src['input_id']+'.npz')) as label:
            asphalt=(label['semantic']==127)&(np.asarray(base['raw_to_voxel'])>=0)
        rows={'jaeho_baseline':base['methods']['jaeho'],
              'jaeho_height':height['methods']['jaeho'],
              'jaeho_support':support['methods']['jaeho'],
              'patchwork_baseline':base['methods']['patchwork'],
              'patchwork_support':pw['methods']['patchwork']}
        if asphalt.sum()<10:continue
        for name,row in rows.items():
            fraction=float(np.mean(np.asarray(row['mask'])[asphalt]==2))
            values.setdefault(name,[]).append(fraction)
            now=(src['phase'],src['scan_effective_stamp_ns'],fraction)
            old=previous.get(name)
            if old and old[0]==now[0] and 0<now[1]-old[1]<300000000:
                delta=abs(now[2]-old[2]);changes.setdefault(name,[]).append(delta)
                counts[name]=counts.get(name,0)+int(delta>.20)
            previous[name]=now
        frames+=1
    for stream in streams:stream.close()
    result={'frames_with_10plus_asphalt':frames,'limits':[
        'Adjacent observed points can change with pose/occlusion: not world-surface jitter truth.',
        'Diagnostic threshold: >20 percentage-point change, same phase, gap <300ms.',
        'Frame percentile is distinct from point-weighted aggregate residual.'],
        'methods':{name:{'residual_fraction':{f'p{q}':float(np.percentile(v,q)) for q in [50,95,99,100]},
            'absolute_adjacent_change':{f'p{q}':float(np.percentile(changes[name],q)) for q in [50,95,99,100]},
            'adjacent_pairs':len(changes[name]),'changes_over_20pp':counts.get(name,0)} for name,v in values.items()}}
    with a.output.open('x') as out:json.dump(result,out,indent=2);out.write('\n')

if __name__=='__main__':main()
