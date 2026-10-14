#!/usr/bin/env python3
"""Propose genuine negatives using synchronized semantic GT, never absent bbox files."""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
from PIL import Image

from camera_pretrain_policy import load_policy, sha, uniform, view_masks
from capture_rgb_quality import validate_rendered_rgb
from prepare_camera_seg_gt import SEMANTIC
from prepare_camera_pretrain import read_rows, write_json


def has_ambiguous_map_objects(semantic_bgr, ego):
    """Map drums share standing_object with poles; absence of actor GT is insufficient.

    These images require RGB review or partial spatial background supervision,
    never automatic whole-image negative adoption. This is a proposal gate only.
    """
    return bool(np.any(np.all(semantic_bgr == [37, 178, 113], axis=2) & ~ego))


def prepare(args):
    if args.output.exists():
        raise ValueError('choose an immutable background proposal revision')
    args.output.mkdir(parents=True)
    policy=load_policy();masks=view_masks(policy)
    keys=np.array([b+(g<<8)+(r<<16) for r,g,b in SEMANTIC],dtype=np.uint32)
    candidates=[];counts=Counter();groups=defaultdict(list)
    for run in args.run:
        manifest=run/'manifest.jsonl';manifest_sha=sha(manifest)
        for frame in read_rows(manifest):
            for view in ('front','left','right'):
                try:
                    if frame.get('valid') is False:
                        raise ValueError('source Capture marked invalid')
                    paths=frame['paths']
                    relative=paths['semantic'][view];semantic=run/relative
                    if frame.get('sha256') and sha(semantic)!=frame['sha256'][relative]:raise ValueError('semantic SHA changed')
                    sem=cv2.imread(str(semantic))
                    if sem is None:raise ValueError('semantic decode failed')
                    ego=masks[view].geometry(*sem.shape[:2])[0]
                    packed=sem[:,:,0].astype(np.uint32)+(sem[:,:,1].astype(np.uint32)<<8)+(sem[:,:,2].astype(np.uint32)<<16)
                    if np.any(np.isin(packed,keys)&~ego):
                        counts['target_semantic_present']+=1;continue
                    if has_ambiguous_map_objects(sem, ego):
                        counts['map_object_requires_RGB_or_partial_supervision']+=1;continue
                    rgb=run/paths['intensity'][view];instance=run/paths['instance'][view]
                    for kind in ('intensity','instance'):
                        file=run/paths[kind][view]
                        if frame.get('sha256') and sha(file)!=frame['sha256'][paths[kind][view]]:raise ValueError(kind+' SHA changed')
                    image=cv2.imread(str(rgb));inst=cv2.imread(str(instance))
                    if image is None or inst is None or image.shape!=sem.shape or inst.shape!=sem.shape:
                        raise ValueError('RGB/GT dimensions disagree')
                    validate_rendered_rgb(Image.fromarray(cv2.cvtColor(image,cv2.COLOR_BGR2RGB)))
                    height,width=image.shape[:2]
                    pixel=hashlib.sha256(f'{width}x{height}:'.encode()+cv2.cvtColor(image,cv2.COLOR_BGR2RGB).tobytes()).hexdigest()
                    key=f'background_{run.name}_{view}_{frame["frame_id"]:06d}'
                    group=run.name+'/'+view
                    row=dict(id=key,source='capture:'+run.name,image=str(rgb.resolve()),
                        image_sha256=sha(rgb),pixel_sha256=pixel,split='train',split_group='capture:'+run.name,
                        width=width,height=height,view=view,correlation_group=f'capture:{run.name}:{frame["frame_id"]//30}',
                        annotations=[],supervision='reviewed_negative',ego_mask_applied=True,
                        mask_config_sha256=masks[view].config_sha256,weather='unknown',tunnel='unknown',
                        background_review_group=group,negative_basis='zero target semantic pixels outside Ego; synchronized RGB/instance checksums',
                        provenance=dict(source_manifest=str(manifest.resolve()),source_manifest_sha256=manifest_sha,
                            semantic=str(semantic.resolve()),semantic_sha256=sha(semantic),instance=str(instance.resolve()),
                            instance_sha256=sha(instance),bbox_file_absence_used_as_truth=False))
                    row['provenance']['historical_file_sha_available']=bool(frame.get('sha256'))
                    candidates.append(row);groups[group].append(row)
                except (ValueError,OSError,KeyError) as exc:
                    counts['not_proven:'+str(exc)]+=1
        print(json.dumps(dict(run=run.name,candidates=len(candidates))),flush=True)
    # These are proposals until every source/view sample sheet is inspected.
    (args.output/'candidates.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in candidates))
    samples={}
    for group,rows in groups.items():
        selected=sorted(rows,key=lambda r:uniform(policy['seed'],r['id']))[:50]
        samples[group]=[r['id'] for r in selected]
        sheet=np.full((int(np.ceil(len(selected)/5))*120,5*200,3),30,np.uint8)
        for i,r in enumerate(selected):
            image=cv2.imread(r['image']);image=cv2.resize(image,(200,100))
            x,y=(i%5)*200,(i//5)*120;sheet[y:y+100,x:x+200]=image
            cv2.putText(sheet,str(i)+' '+r['view']+' '+r['id'].rsplit('_',1)[-1],(x+2,y+115),cv2.FONT_HERSHEY_SIMPLEX,.36,(255,255,255),1)
        cv2.imwrite(str(args.output/(group.replace('/','__')+'.jpg')),sheet)
    write_json(args.output/'review.json',dict(status='pending',candidate_manifest_sha256=sha(args.output/'candidates.jsonl'),
        samples=samples,groups={g:dict(decision='pending',sampled=len(v),total=len(groups[g])) for g,v in samples.items()},
        counters=dict(counts),limit=policy['background_limit'],missing_bbox_not_negative=True))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',type=Path,action='append',required=True)
    p.add_argument('--output',type=Path,required=True)
    prepare(p.parse_args())
