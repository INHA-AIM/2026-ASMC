#!/usr/bin/env python3
"""Create a new ledger revision from observed native GT and verified component schemas."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
from collections import defaultdict
from prepare_camera_seg_gt import parse_bbox
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src/data_collection/src'))
from data_collection.capture_collection import observed_components


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',type=Path,required=True,help='Synced Capture dataset')
    p.add_argument('--ledger',type=Path,required=True)
    p.add_argument('--calibration',type=Path,default=ROOT/'config/morai/capture_asset_ground_v2.json')
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if a.output.exists():raise ValueError('new immutable ledger revision required')
    sources=defaultdict(set)
    colors=defaultdict(lambda:defaultdict(set))
    for line in (a.run/'manifest.jsonl').read_text().splitlines():
        row=json.loads(line)
        for relative in row['paths'].get('bbox',{}).values():
            for box in parse_bbox((a.run/relative).read_text(encoding='utf-8-sig')):
                sources[box['actor_id']].add(box['gt_source_label'])
                colors[box['actor_id']][box['gt_source_label']].add(tuple(box['rgba']))
    ledger=[json.loads(l) for l in a.ledger.read_text().splitlines()]
    assets=json.loads(a.calibration.read_text())['assets']
    reviewed={};snapshots={}
    for row in ledger:
        review=row.get('placement_review',{})
        if not row.get('placement_approved') or not review.get('snapshot_sha256'):continue
        path=(ROOT.parent/review['gallery']).with_name('review-snapshot.json')
        digest=hashlib.sha256(path.read_bytes()).hexdigest()
        if digest!=review['snapshot_sha256']:raise ValueError('placement review snapshot SHA mismatch')
        snapshot=snapshots.setdefault(digest,json.loads(path.read_text()))
        root=str(row['native_ids'][0])
        matches=[s for s in snapshot['snapshots'] if s['native_id']==root and s['api_name']==row['api_name']]
        if len(matches)!=1:raise ValueError('reviewed parent identity absent or ambiguous')
        for code,spec in assets.get(row['api_name'],{}).get('components',{}).items():
            native=root if int(code)==0 else str(int(root)*10000+int(code))
            candidates={tuple(b['rgba']) for b in matches[0]['native_boxes']
                if b['actor_id']==native and b['gt_source_label']==spec['gt_source_label']}
            if len(candidates)==1 and native in sources:
                reviewed.setdefault(native,{})[spec['gt_source_label']]=next(iter(candidates))
    rows=observed_components(ledger,sources,assets,colors,reviewed)
    for r in rows:
        r['component_evidence']=dict(source_ledger_sha256=hashlib.sha256(a.ledger.read_bytes()).hexdigest(),
            manifest_sha256=hashlib.sha256((a.run/'manifest.jsonl').read_bytes()).hexdigest(),
            calibration_sha256=hashlib.sha256(a.calibration.read_bytes()).hexdigest())
        r['component_evidence']['review_snapshot_sha256']=sorted(snapshots)
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(''.join(json.dumps(r)+'\n' for r in rows))
    print(json.dumps(dict(rows=len(rows),native_ids=sum(len(r.get('native_ids',[])) for r in rows),output=str(a.output))))


if __name__=='__main__':main()
