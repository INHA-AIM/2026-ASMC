#!/usr/bin/env python3
"""Verify that new scoring reproduces frozen V1 counts and all topology values."""
import argparse
import json
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('old',type=Path);p.add_argument('new',type=Path)
a=p.parse_args();x=json.loads(a.old.read_text());y=json.loads(a.new.read_text())
assert x['frames']==y['frames']
checks=0
for m in ('jaeho','patchwork','linefit'):
    name='v1_'+m
    assert x['topology'][m]==y['topology'][name],m
    for c,row in x['classes'][m].items():
        for k,v in row.items():
            if k in y['classes'][name][c]:assert v==y['classes'][name][c][k],(m,c,k);checks+=1
    for c,row in x['objects'][m].items():
        for k,v in row.items():
            k2='complete_loss' if k=='complete_ground_loss' else k
            if k2 in y['objects'][name][c]:assert v==y['objects'][name][c][k2],(m,c,k);checks+=1
print(json.dumps({'frames':x['frames'],'matched_count_fields':checks,'all_topology_exact':True}))
