#!/usr/bin/env python3
"""Keep unverified Capture palettes out of negatives without deleting raw GT."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import shutil


def quarantine(row,runs):
    if row['run'] not in runs:return row
    row=dict(row,issues=list(row['issues']))
    row['issues'].append(dict(reason='unverified_semantic_palette_run'))
    row['annotation_complete']=False
    return row


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--gt',type=Path,required=True)
    p.add_argument('--runs',required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();runs=set(a.runs.split(','));a.output.mkdir(parents=True,exist_ok=False)
    source=json.loads((a.gt/'summary.json').read_text())
    if not runs<=set(source['splits']):raise ValueError('unknown Capture run')
    shutil.copytree(a.gt/'masks',a.output/'masks')
    counts=Counter();issues=Counter();groups=Counter()
    with (a.output/'frames.jsonl').open('w') as out:
        for line in (a.gt/'frames.jsonl').read_text().splitlines():
            row=quarantine(json.loads(line),runs);out.write(json.dumps(row)+'\n')
            counts['frames']+=1;counts[row['split']+'_frames']+=1;counts['annotations']+=len(row['annotations'])
            counts['complete_frames' if row['annotation_complete'] else 'quarantined_frames']+=1
            if row['annotation_complete']:
                counts[row['split']+'_complete']+=1;groups.update(ann['group'] for ann in row['annotations'])
            issues.update(issue['reason'] for issue in row['issues'])
    source.update(counts=dict(counts),issues=dict(issues),complete_group_counts=dict(groups),
                  supersedes_manifest_sha256=source['manifest_sha256'],
                  manifest_sha256=hashlib.sha256((a.output/'frames.jsonl').read_bytes()).hexdigest(),
                  unverified_palette_runs=sorted(runs),
                  palette_reason='initial kcity runs include different actor palette (e.g. 255,0,0); no verified BBox mapping, not confirmed negatives')
    (a.output/'summary.json').write_text(json.dumps(source,indent=2)+'\n');print(json.dumps(dict(counts)))


if __name__=='__main__':main()
