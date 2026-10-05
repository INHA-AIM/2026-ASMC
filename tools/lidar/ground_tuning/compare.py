#!/usr/bin/env python3
"""Exact V1 or repeated-candidate masks and diagnostic comparison."""
import argparse
import itertools
import json
from pathlib import Path


def main():
    p=argparse.ArgumentParser();p.add_argument('first',type=Path);p.add_argument('second',type=Path)
    p.add_argument('--diagnostics',action='store_true');p.add_argument('--output',type=Path)
    a=p.parse_args();n=0
    with a.first.open() as left,a.second.open() as right:
        for pair in itertools.zip_longest(left,right):
            if None in pair:raise ValueError('length mismatch')
            x,y=map(json.loads,pair)
            for k in ('input_id','stamp_ns','raw_to_voxel'):
                if x[k]!=y[k]:raise AssertionError((n,k))
            for name,v in y['methods'].items():
                keys=['mask','structural_removed']+(['pre_guard_mask','reasons','events'] if a.diagnostics else [])
                for k in keys:
                    if x['methods'][name][k]!=v[k]:raise AssertionError((x['input_id'],name,k))
            n+=1
    if not n:raise AssertionError('empty comparison')
    report={'frames':n,'exact':True,'diagnostics':a.diagnostics,'first':str(a.first),'second':str(a.second)}
    if a.output:
        with a.output.open('x') as f:json.dump(report,f,indent=2)
    print(json.dumps(report))


if __name__=='__main__':main()
