#!/usr/bin/env python3
"""Read-only v8 verification; no training or dataset mutation."""
import argparse
import json
from pathlib import Path

from camera_pretrain_policy import sha, validate_box, validate_dataset_contract
from prepare_camera_pretrain import read_rows, verify_export


def verify(root, rgb_sha=False):
    meta=validate_dataset_contract(root,ignore_aware=True)
    rows=list(read_rows(root/'manifest.jsonl'))
    result=verify_export(root,rows,meta['parents'])
    ids=set()
    for row in rows:
        try:
            if row['id'] in ids:raise ValueError('duplicate image ID')
            ids.add(row['id'])
            if row['supervision'] not in ('complete','partial','reviewed_negative'):
                raise ValueError('unknown completeness state')
            positives=[a for a in row['annotations'] if a['supervision']=='positive']
            if row['split']=='train' and not positives and row['supervision']!='reviewed_negative':
                if not (row.get('spatial_background_only') and row['supervision']=='partial' and
                        row.get('background_safe_grid') and row.get('known_class_background') is False):
                    raise ValueError('unreviewed empty training image')
                import numpy as np
                grid=root/row['background_safe_grid']
                if sha(grid)!=row['background_safe_grid_sha256']:
                    raise ValueError('spatial-only background SHA mismatch')
                with np.load(grid) as z:
                    cells=np.unpackbits(z['packed'])
                    if cells.size!=14400 or not cells.any():
                        raise ValueError('empty/invalid spatial-only safe grid')
            for ann in row['annotations']:
                if ann['supervision'] in ('positive','ignore'):
                    validate_box(ann['xyxy'],row['width'],row['height'])
            role={'selection':'val','heldout':'test'}.get(row['split'],row['split'])
            if sha(root/'labels'/role/(row['id']+'.txt'))!=row['export_label_sha256']:
                raise ValueError('YOLO file SHA changed')
            if rgb_sha and sha(row['image'])!=row['image_sha256']:
                raise ValueError('RGB source SHA changed')
        except (ValueError,OSError) as e:
            result['failures'].append(dict(id=row['id'],reason=str(e)))
    result['passed']=not result['failures']
    result['rgb_sha_rechecked']=rgb_sha
    result['training_performed']=False
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--dataset',required=True,type=Path)
    p.add_argument('--rgb-sha',action='store_true')
    p.add_argument('--report',type=Path,help='New evidence file outside immutable dataset')
    args=p.parse_args();result=verify(args.dataset,args.rgb_sha)
    if args.report:
        try:
            args.report.resolve().relative_to(args.dataset.resolve())
            inside=True
        except ValueError:
            inside=False
        if inside:
            p.error('verification report must be outside immutable dataset')
        with args.report.open('x') as f:json.dump(result,f,indent=2)
    print(json.dumps(result,indent=2))
    raise SystemExit(0 if result['passed'] else 1)
