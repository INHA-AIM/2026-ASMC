#!/usr/bin/env python3
"""Preserve Capture request lineage without treating request time as exposure time."""
import argparse
import hashlib
import json
from pathlib import Path


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--gt',type=Path,required=True)
    p.add_argument('--datasets',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    tables={};sha={}
    for run in sorted(a.datasets.iterdir()):
        path=run/'manifest.jsonl'
        if path.is_file():
            rows=[json.loads(line) for line in path.read_text().splitlines()]
            table={str(r['frame_id']).zfill(6):r for r in rows}
            if len(table)!=len(rows):raise ValueError('duplicate Capture frame IDs: '+run.name)
            tables[run.name]=table;sha[run.name]=hashlib.sha256(path.read_bytes()).hexdigest()
    counts=dict(frames=0,linked=0,missing=0)
    with (a.output/'lineage.jsonl').open('w') as out:
        for line in (a.gt/'frames.jsonl').read_text().splitlines():
            row=json.loads(line);source=tables.get(row['run'],{}).get(row['frame'])
            result=dict(run=row['run'],view=row['view'],frame=row['frame'],split=row['split'],
                        linked=source is not None,exposure_time_verified=False)
            if source is not None:
                result.update(capture_manifest_sha256=sha[row['run']],capture_record=source)
            else:result['reason']='no_capture_manifest_frame_match'
            out.write(json.dumps(result)+'\n');counts['frames']+=1;counts['linked' if source else 'missing']+=1
    summary=dict(counts=counts,manifest_sha256=sha,
                 gt_manifest_sha256=hashlib.sha256((a.gt/'frames.jsonl').read_bytes()).hexdigest(),
                 lineage_sha256=hashlib.sha256((a.output/'lineage.jsonl').read_bytes()).hexdigest(),
                 timing='requested ros_time_ns and original morai_sim_time_ms preserved verbatim; unit/exposure not inferred')
    (a.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(counts))


if __name__=='__main__':main()
