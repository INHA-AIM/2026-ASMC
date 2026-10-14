#!/usr/bin/env python3
"""Apply individual, pinned training reviews to an immutable dataset revision.

Never starts training, activates a dataset, changes evaluation rows, propagates
sample decisions to a source, or edits parent exports. RGB symlinks are retained;
all other files are independent copies. Existing loader/loss contracts are used.
"""
import argparse
from collections import Counter
from copy import deepcopy
import json
from pathlib import Path
import shutil

import numpy as np
from PIL import Image
import yaml

from camera_pretrain_policy import (CLASSES, sha, validate_dataset_contract,
    view_masks, load_policy, epoch_order, guarded_data_yaml)
from prepare_camera_pretrain import read_rows, write_json, yolo_lines, verify_export
from review_camera_supervision_policy import (distribution, indexes,
    assert_isolation, TARGET_SEMANTIC, frame_paths)
from prepare_camera_spatial_background import OFFICIAL_PALETTE_RGB


def apply_reviews(rows, reviews):
    result=deepcopy(rows); lookup={r['id']:r for r in result}; changes=[]
    for kind in ('vehicles','duplicates','backgrounds','complete_images'):
        seen=set()
        for review in reviews.get(kind,[]):
            key=(review['id'],review.get('annotation_index'))
            if key in seen:raise ValueError('duplicate review')
            seen.add(key)
            row=lookup[review['id']]
            if row['split']!='train':raise ValueError('evaluation mutation forbidden')
            if row['image_sha256']!=review['image_sha256']:raise ValueError('review image SHA mismatch')
            if review['decision'] in ('retain','defer'):continue
            if kind=='duplicates':
                ann=row['annotations'][review['annotation_index']]
                keeper=row['annotations'][review['duplicate_of']]
                if review['decision']!='exclude_duplicate' or ann!=review['original_annotation']:
                    raise ValueError('duplicate identity mismatch')
                if ann is keeper or ann['supervision']!='positive' or keeper['supervision']!='positive':
                    raise ValueError('duplicate keeper must be a different positive')
                if keeper['training_class']!=review['target_class']:
                    raise ValueError('duplicate keeper class mismatch')
                a,b=ann['xyxy'],keeper['xyxy']
                inter=max(0,min(a[2],b[2])-max(a[0],b[0]))*max(0,min(a[3],b[3])-max(a[1],b[1]))
                union=(a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-inter
                if union<=0 or inter/union<.95:raise ValueError('duplicate overlap insufficient')
                ann['preflight_previous_annotation']=deepcopy(ann)
                ann.update(supervision='excluded',supervision_reasons=['reviewed_duplicate_annotation'],preflight_review=deepcopy(review))
                changes.append(dict(id=row['id'],kind=kind,annotation_index=review['annotation_index'],before='positive',after='excluded'))
            elif kind=='vehicles':
                if review['decision']!='relabel' or review['target_class'] not in CLASSES:
                    raise ValueError('invalid class decision')
                ann=row['annotations'][review['annotation_index']]
                if ann!=review['original_annotation']:raise ValueError('annotation identity mismatch')
                if ann['supervision']!='positive':raise ValueError('review scope is positive class only')
                before=ann['training_class']; after=review['target_class']
                ann['preflight_previous_annotation']=deepcopy(ann)
                ann.update(training_class=after,canonical=after,
                    consumer_group='vehicle' if after in ('car','bus','truck','motorcycle','bicycle') else 'obstacle',
                    preflight_review=deepcopy(review))
                changes.append(dict(id=row['id'],kind=kind,annotation_index=review['annotation_index'],before=before,after=after))
            else:
                if not review['all_targets_annotated'] or review['review_scope']!='camera_objects_v2_16':
                    raise ValueError('full-frame review missing')
                if row['supervision']!='partial':raise ValueError('expected partial parent')
                if kind=='backgrounds':
                    if row['annotations'] or not row.get('spatial_background_only'):
                        raise ValueError('negative review must have no annotations and pinned spatial background')
                    if review['decision']!='reviewed_negative':raise ValueError('invalid negative decision')
                    row['negative_basis']='individual RGB full16 review plus pinned target-semantic absence; no source propagation'
                    row.pop('spatial_background_only',None)
                elif review['decision']!='complete' or not any(a['supervision']=='positive' for a in row['annotations']):
                    raise ValueError('invalid positive complete review')
                row.update(previous_supervision='partial',supervision=review['decision'],
                    annotation_complete=True,review_scope='camera_objects_v2_16',
                    review_record=deepcopy(review),known_class_background=False)
                changes.append(dict(id=row['id'],kind=kind,before='partial',after=row['supervision']))
    return result,changes


def prepare(args):
    parent=validate_dataset_contract(args.source,ignore_aware=True)
    pins={f:sha(args.source/f) for f in ('pretrain.json','manifest.jsonl')}
    reviews=json.loads(args.reviews.read_text())
    if reviews['source_metadata_sha256']!=pins['pretrain.json'] or reviews['source_manifest_sha256']!=pins['manifest.jsonl']:
        raise ValueError('review parent differs')
    for name,digest in reviews['evidence_sha256'].items():
        if sha(args.reviews.parent/name)!=digest:raise ValueError('visual evidence changed')
    if args.output.exists():raise ValueError('immutable revision exists')
    old=list(read_rows(args.source/'manifest.jsonl'));rows,changes=apply_reviews(old,reviews)
    policy=load_policy(args.source/'policy.json');masks=view_masks(policy);frames={}
    changed_ids={c['id'] for c in changes};semantic_checks=[]
    for r in rows:
        if r['id'] not in changed_ids:continue
        if sha(r['image'])!=r['image_sha256']:raise ValueError('reviewed RGB changed')
        if r['supervision']!='reviewed_negative':continue
        sem,_,_=frame_paths(r,args.data_root,frames)
        if sha(sem)!=r['background_semantic_sha256']:raise ValueError('semantic pin mismatch')
        rgb=np.array(Image.open(sem).convert('RGB'));target=np.zeros(rgb.shape[:2],bool)
        for name in TARGET_SEMANTIC:target|=(rgb==OFFICIAL_PALETTE_RGB[name]).all(2)
        if r.get('ego_mask_applied'):target&=~masks[r['view']].geometry(r['height'],r['width'])[0]
        if target.any():raise ValueError('negative has actor semantic pixels')
        semantic_checks.append(dict(id=r['id'],target_pixels=0,semantic_sha256=sha(sem)))
    # No hardlinks: later writes cannot mutate the parent.
    shutil.copytree(args.source,args.output,symlinks=True,
        ignore=shutil.ignore_patterns('_SUCCESS','_PREPARED'))
    for r in rows:
        if r['id'] not in changed_ids:continue
        lines=yolo_lines(r);label=args.output/'labels/train'/(r['id']+'.txt')
        label.write_text('\n'.join(lines)+ ('\n' if lines else ''))
        side=args.output/'supervision/train'/(r['id']+'.json')
        write_json(side,{k:r.get(k) for k in ['supervision','annotations','known_class_background','background_policy','background_safe_grid','background_safe_grid_sha256']})
        r.update(export_label_sha256=sha(label),supervision_sha256=sha(side))
    (args.output/'manifest.jsonl').write_text(''.join(json.dumps(r,ensure_ascii=False)+'\n' for r in rows))
    train=[r for r in rows if r['split']=='train'];order,balance=epoch_order(train,policy['seed'],alignment=16)
    (args.output/'epoch-000-order.jsonl').write_text(''.join(json.dumps(dict(id=train[i]['id']))+'\n' for i in order))
    for role in ('selection','heldout'):
        assert [r for r in rows if r['split']==role]==[r for r in old if r['split']==role]
        assert_isolation(train,indexes(read_rows(args.output/f'reserved-{role}.jsonl')))
    parents={str(args.source):pins['manifest.jsonl']}
    verification=verify_export(args.output,rows,parents)
    if not verification['passed']:raise ValueError(verification['failures'])
    for r,prior in zip(rows,old):
        if len(r['annotations'])!=len(prior['annotations']):raise ValueError('annotation count changed')
        for a,b in zip(r['annotations'],prior['annotations']):
            for field in ('xyxy','original_label','original_category_id'):
                if a.get(field)!=b.get(field):raise ValueError('raw annotation/geometry changed')
            if a.get('supervision')!=b.get('supervision') and not (
                b.get('supervision')=='positive' and a.get('supervision')=='excluded'
                and a.get('preflight_review',{}).get('decision')=='exclude_duplicate'
                and a.get('preflight_previous_annotation')==b):
                raise ValueError('unreviewed supervision change')
        role={'selection':'val','heldout':'test'}.get(r['split'],r['split'])
        if sha(args.output/'supervision'/role/(r['id']+'.json'))!=r['supervision_sha256']:
            raise ValueError('sidecar roundtrip')
        if r.get('background_safe_grid') and sha(args.output/r['background_safe_grid'])!=r['background_safe_grid_sha256']:
            raise ValueError('grid roundtrip')
    verification.update(parent_unchanged=True,evaluation_rows_unchanged=True,
        reserved_split_isolation=True,raw_labels_and_boxes_preserved=True,
        individual_review_only=True,negative_semantic_checks=semantic_checks,
        optimizer_updates=0,training_started=False)
    write_json(args.output/'verification.json',verification)
    summary=json.loads((args.source/'summary.json').read_text())
    summary.update(version=args.output.name,splits={role:distribution([r for r in rows if r['split']==role]) for role in ('train','selection','heldout')},balance=balance,
        split_membership_changed=False,preflight_changes=dict(Counter(c['kind'] for c in changes)),training_started=False)
    write_json(args.output/'summary.json',summary);write_json(args.output/'preflight-changes.json',changes)
    shutil.copy2(args.reviews,args.output/'preflight-reviews.json')
    (args.output/'data.yaml').write_text(yaml.safe_dump(guarded_data_yaml(args.output),sort_keys=False))
    meta=deepcopy(parent)
    meta.update(parents=parents,source_stage=str(args.source),activation_decision_pending=True,
        preflight_revision=dict(parent_metadata_sha256=pins['pretrain.json'],review_sha256=sha(args.reviews),
            change_counts=dict(Counter(c['kind'] for c in changes)),loss_formula_changed=False,
            vehicle_population_error_rate_measured=False,full_dataset_visual_review=False))
    if 'restoration_verification' in meta:
        meta['parent_restoration_verification']=meta.pop('restoration_verification')
    files=set(parent['artifact_shas'])|{'preflight-changes.json','preflight-reviews.json'}
    meta['artifact_shas']={f:sha(args.output/f) for f in sorted(files)}
    write_json(args.output/'pretrain.json',meta)
    for f,digest in pins.items():
        if sha(args.source/f)!=digest:raise ValueError('parent changed')
    (args.output/'_SUCCESS').write_text(sha(args.output/'pretrain.json')+'\n')
    validate_dataset_contract(args.output,ignore_aware=True)
    print(json.dumps(dict(passed=True,output=str(args.output),metadata_sha256=sha(args.output/'pretrain.json'),changes=summary['preflight_changes'])))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('source','reviews','output','data-root'):p.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args()
    for key,value in vars(a).items():setattr(a,key,value.resolve())
    prepare(a)
