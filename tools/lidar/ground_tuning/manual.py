#!/usr/bin/env python3
"""Unlabelled IMU sequence diagnostics, explicitly not ground/object accuracy."""
import argparse
from contextlib import ExitStack
from collections import defaultdict
import itertools
import json
from pathlib import Path
import numpy as np

p=argparse.ArgumentParser();p.add_argument('input',type=Path);p.add_argument('results',type=Path);p.add_argument('output',type=Path)
a=p.parse_args()
if a.output.exists():raise FileExistsError(a.output)
files=sorted(a.results.glob('*.jsonl'));frames=[];times=defaultdict(list)
with ExitStack() as stack:
    src=stack.enter_context((a.input/'snapshots.jsonl').open())
    streams=[stack.enter_context(f.open()) for f in files]
    for lines in itertools.zip_longest(src,*streams):
        if None in lines:raise ValueError('length mismatch')
        source,*rows=map(json.loads,lines)
        frame={'input_id':source['input_id'],'pitch_deg':float(np.degrees(source['pitch'])),
               'roll_deg':float(np.degrees(source['roll'])),'methods':{}}
        for file,row in zip(files,rows):
            if row['input_id']!=source['input_id'] or row['stamp_ns']!=source['scan_effective_stamp_ns']:raise ValueError('identity mismatch')
            for m,r in row['methods'].items():
                name='v1_'+m if file.stem=='v1' else file.stem
                mask=np.asarray(r['mask']);flags=np.asarray(r['reasons'],dtype=np.uint32)
                frame['methods'][name]={'roi':int(np.count_nonzero(mask)),
                    'ground':int(np.sum(mask==1)),'nonground':int(np.sum(mask==2)),
                    'unclassified':int(np.sum(mask==3)),
                    'guard_restored':int(np.count_nonzero(flags&2048)) if m=='patchwork' else 0}
                times[name].append(row['prepare_ms']+sum(r[k] for k in ('copy_ms','segment_ms','guard_ms','restore_ms')))
        frames.append(frame)
summary={'frames':len(frames),'label_status':'unlabelled; no ground or actor accuracy claim',
    'pitch_deg_range':[min(f['pitch_deg'] for f in frames),max(f['pitch_deg'] for f in frames)],
    'guard_highest_frames':[f['input_id'] for f in sorted(frames,key=lambda f:f['methods']['v1_patchwork']['guard_restored'],reverse=True)[:10]],
    'accounted_stage_sum_ms':{n:{'p'+str(q):float(np.percentile(v,q)) for q in (50,95,99)} for n,v in times.items()},
    'observations':frames}
a.output.write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps({k:v for k,v in summary.items() if k!='observations'}))
