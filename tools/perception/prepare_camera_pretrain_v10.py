#!/usr/bin/env python3
"""Publish a reviewed whole-lineage resplit with consistent 4px/64px² supervision.

Originals and previous revisions are read-only. Approval records name individual
reviewed evaluation images; candidate panels are never implicitly approved.
"""
import argparse
from collections import Counter, defaultdict, deque
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil
import sys
import threading
import time

import cv2
import numpy as np
from PIL import Image
import yaml
from camera_pretrain_policy import (CLASSES, ROOT, sha, load_policy, tiny_reasons,
    view_masks, epoch_order, guarded_data_yaml, uniform, validate_box)
from prepare_camera_pretrain import read_rows, yolo_lines, verify_export, write_json
from prepare_camera_spatial_background import semantic_grid, OFFICIAL_PALETTE_RGB
from review_camera_supervision_policy import (assert_isolation, indexes, distribution,
    frame_paths, recorded_conditions, weather, TARGET_SEMANTIC)
import review_camera_supervision_policy as review_module


def apply_policy(row, policy):
    r = deepcopy(row)
    for ann in r['annotations']:
        if ann['supervision'] not in ('positive', 'ignore'):
            continue
        validate_box(ann['xyxy'], r['width'], r['height'])
        if ann.get('training_class') not in CLASSES:
            raise ValueError('unresolved class: '+r['id'])
        reasons = tiny_reasons(ann['xyxy'], r['width'], r['height'], policy, ann.get('visible_pixels'))
        if ann['supervision'] == 'positive' and reasons:
            ann.update(supervision='ignore', pre_v10_supervision='positive')
        if reasons:
            ann['supervision_reasons'] = sorted(set(ann.get('supervision_reasons', [])+reasons))
    r['applied_policy'] = dict(short_px=policy['tiny_short_px'], area_px2=policy['tiny_area_px2'],
                              reference_tensor=960, native_also_checked=True, tiny_treatment='ignore')
    return r


def reviewed_panel(folder, role):
    decisions=json.loads((folder/'reviews.json').read_text())
    candidates={r['id']:r for suffix in ('candidates','extra-candidates')
                for r in read_rows(folder/f'{role}-{suffix}.jsonl')}
    accepted=[]
    for item in decisions[role]:
        r=deepcopy(candidates[item['id']])
        if sha(r['image']) != item['image_sha256']:
            raise ValueError('review image identity changed')
        if item['decision'] != 'accept':
            continue
        if not item.get('all_targets_annotated') or item.get('review_scope') != 'camera_objects_v2_16':
            raise ValueError('complete evaluation requires explicit full16 inspection')
        r['previous_completeness']=r['supervision']
        r.update(supervision='complete', annotation_complete=True, review_scope='camera_objects_v2_16',
                 review_record=item, evaluation_role='reviewed_resplit_development' if role=='selection' else 'reserved_final')
        accepted.append(r)
    if len({r['id'] for r in accepted})!=len(accepted):raise ValueError('duplicate review')
    return accepted


def additional_negatives(args, rows, count):
    reviews=json.loads(args.background_reviews.read_text())
    approved={g for g,x in reviews['groups'].items() if x['decision']=='accept'}
    exceptions={x['id'] for x in reviews.get('exceptions',[])}
    old={r['pixel_sha256'] for r in rows};old_ids={r['id'] for r in rows}
    roles={r['split_group']:r['split'] for r in rows};groups=defaultdict(list)
    for row in read_rows(args.background_candidates):
        if (row.get('background_review_group') not in approved or row['id'] in exceptions or
            row['id'] in old_ids or row['pixel_sha256'] in old or roles.get(row['split_group'])!='train'):
            continue
        groups[row['background_review_group']].append(row)
    queues=deque(deque(sorted(rs,key=lambda r:uniform(20261013,r['id']))) for _,rs in sorted(groups.items()))
    selected=[]
    while queues and len(selected)<count:
        q=queues.popleft();r=deepcopy(q.popleft())
        if q:queues.append(q)
        if r['pixel_sha256'] in old:continue
        r['provenance']['background_reviews_sha256']=sha(args.background_reviews)
        r['background_review_inherited_from']=str(args.background_reviews)
        selected.append(r);old.add(r['pixel_sha256'])
    return selected


def build(args, prepared=None):
    start=time.time();cv2.setNumThreads(1)
    if args.output.exists():raise ValueError('immutable output exists; use another revision')
    policy=load_policy(args.policy);masks=view_masks(policy)
    if (policy['tiny_short_px'],policy['tiny_area_px2'])!=(4,64):raise ValueError('unexpected approved policy')
    review_module.CONDITIONS=recorded_conditions(args.data_root)
    reserved={role:list(read_rows(args.proposal/'full-resplit'/f'{role}.jsonl')) for role in ('train','selection','heldout')}
    for i,left in enumerate(reserved):
        for right in list(reserved)[i+1:]:assert_isolation(reserved[left],indexes(reserved[right]))
    original={r['id']:r for r in read_rows(args.source/'manifest.jsonl')}
    if prepared is None:
        train=[apply_policy(r,policy) for r in reserved['train'] if r['candidate_train_eligible']]
        extra=additional_negatives(args,[r for rs in reserved.values() for r in rs],args.extra_backgrounds)
        train.extend(extra)
        roles={'train':train,**{role:[apply_policy(r,policy) for r in reviewed_panel(args.review,role)] for role in ('selection','heldout')}}
    else:
        # Caller supplies pinned individual reviews, immutable patches and roles.
        roles={role:[apply_policy(r,policy) for r in prepared[role]] for role in reserved}
        train=roles['train'];extra=[]
    for role in ('selection','heldout'):
        pool={r['id'] for r in reserved[role]}
        if any(r['id'] not in pool or r['split']!=role for r in roles[role]):raise ValueError('panel escaped reserved lineage')
        if any(not any(a['supervision']=='positive' and a['training_class']==c for r in roles[role] for a in r['annotations']) for c in CLASSES):
            raise ValueError('evaluation missing a class')
        assert_isolation(train,indexes(reserved[role]))
    rows=[r for rs in roles.values() for r in rs]
    if len({r['id'] for r in rows})!=len(rows):raise ValueError('duplicate image IDs')
    args.output.mkdir(parents=True)
    for folder in ('images','labels','supervision','background'):
        for role in ('train','val','test'):(args.output/folder/role).mkdir(parents=True)
    frames={};lock=threading.Lock()
    extras=[OFFICIAL_PALETTE_RGB[c] for c in policy['expanded_safe_semantic_classes']]
    conditions=review_module.CONDITIONS
    def process(row):
        r=deepcopy(row);im=Path(r['image']);raw=im.read_bytes()
        if hashlib.sha256(raw).hexdigest()!=r['image_sha256']:raise ValueError('source RGB changed: '+r['id'])
        rgb=cv2.imdecode(np.frombuffer(raw,np.uint8),cv2.IMREAD_COLOR)
        if rgb is None or rgb.shape[:2]!=(r['height'],r['width']):raise ValueError('RGB decode/shape')
        rgb=cv2.cvtColor(rgb,cv2.COLOR_BGR2RGB)
        pixel=hashlib.sha256(f"{r['width']}x{r['height']}:".encode()+rgb.tobytes()).hexdigest()
        legacy=hashlib.sha256(rgb.tobytes()).hexdigest()
        if r['pixel_sha256'] not in (pixel,legacy):raise ValueError('decoded pixel SHA: '+r['id'])
        if r.get('ego_mask_applied'):
            if r['view'] not in masks or sha(ROOT/policy['mask_files'][r['view']])!=r['mask_config_sha256']:
                raise ValueError('mask identity mismatch: '+r['id'])
        r['weather']=weather(r)
        if r['source'] in conditions:r['condition_provenance']=conditions[r['source']]
        grid=None;old=original.get(r['id']);added_cells=0
        if old and old.get('background_safe_grid'):
            path=args.source/old['background_safe_grid']
            if sha(path)!=old['background_safe_grid_sha256']:raise ValueError('old background grid changed')
            with np.load(path) as z:grid=np.unpackbits(z['packed']).reshape(120,120).astype(bool)
        prov=r.get('provenance',{})
        has_pinned_gt=bool(prov.get('source_run') or prov.get('semantic') or
                          prov.get('original_annotation',{}).get('source_sha256',{}).get('semantic'))
        if r['source'].startswith('capture:') and not has_pinned_gt:
            if r['supervision']!='complete' or not r.get('review_record',{}).get('all_targets_annotated'):
                raise ValueError('Capture without pinned GT or full16 review: '+r['id'])
            r['spatial_gt_scope']='manually reviewed bbox only; no semantic expansion inferred'
        if r['source'].startswith('capture:') and has_pinned_gt:
            with lock:sem,inst,f=frame_paths(r,args.data_root,frames)
            prov=r.get('provenance',{});expected=None;expected_instance=None
            if f:
                run=im.parents[3];expected=f['sha256'][f['paths']['semantic'][r['view']]]
                expected_instance=f['sha256'][f['paths']['instance'][r['view']]]
            elif prov.get('semantic'):
                expected=prov['semantic_sha256'];expected_instance=prov['instance_sha256']
            else:
                checksums=prov.get('original_annotation',{}).get('source_sha256',{})
                expected=checksums.get('semantic');expected_instance=checksums.get('instance')
            if expected is None or sha(sem)!=expected:raise ValueError('semantic identity unavailable: '+r['id'])
            if expected_instance is None or sha(inst)!=expected_instance:raise ValueError('instance identity unavailable: '+r['id'])
            sem_rgb=np.array(Image.open(sem).convert('RGB'))
            if sem_rgb.shape!=rgb.shape:raise ValueError('semantic/RGB shape')
            ego=masks[r['view']].geometry(r['height'],r['width'])[0] if r.get('ego_mask_applied') else None
            if r['supervision']=='reviewed_negative':
                target=np.zeros(sem_rgb.shape[:2],bool)
                for name in TARGET_SEMANTIC:target|=(sem_rgb==OFFICIAL_PALETTE_RGB[name]).all(axis=2)
                if ego is not None:target&=~ego
                if target.any():raise ValueError('negative contains target semantic pixels: '+r['id'])
            if r.get('spatial_background_only'):
                if r['supervision']!='partial' or any(a['supervision']=='positive' for a in r['annotations']):
                    raise ValueError('invalid spatial-only background contract')
                # Rebuild instead of inheriting a whole-image negative assertion.
                grid=semantic_grid(sem_rgb,[OFFICIAL_PALETTE_RGB[c] for c in policy['safe_semantic_classes']],ego=ego)
                if not grid.any():raise ValueError('spatial background has no safe cells')
            expansion=semantic_grid(sem_rgb,extras,ego=ego)
            if grid is not None:added_cells=int((expansion&~grid).sum());grid|=expansion
            elif r['supervision']=='partial':
                # A row without a pinned previous grid gets a fresh conservative
                # semantic-only grid. Never infer actor pixels as safe background.
                grid=semantic_grid(sem_rgb,[OFFICIAL_PALETTE_RGB[c] for c in policy['safe_semantic_classes']],ego=ego)
            r['background_semantic_sha256']=expected
        role={'selection':'val','heldout':'test'}.get(r['split'],r['split'])
        r['known_class_background']=bool(grid is None and r['supervision']=='partial')
        r['background_policy']='spatial_v2'
        r['background_policy_revision']='v3_reviewed_semantic_expansion'
        if grid is not None:
            rel=Path('background')/role/(r['id']+'.npz');np.savez_compressed(args.output/rel,packed=np.packbits(grid))
            r.update(background_safe_grid=str(rel),background_safe_grid_sha256=sha(args.output/rel))
        else:
            r.pop('background_safe_grid',None);r.pop('background_safe_grid_sha256',None)
        image_rel=Path('images')/role/(r['id']+im.suffix.lower());(args.output/image_rel).symlink_to(im.resolve())
        label=args.output/'labels'/role/(r['id']+'.txt');lines=yolo_lines(r);label.write_text('\n'.join(lines)+('\n' if lines else ''))
        side=args.output/'supervision'/role/(r['id']+'.json')
        write_json(side,{k:r.get(k) for k in ['supervision','annotations','known_class_background','background_policy','background_safe_grid','background_safe_grid_sha256']})
        r.update(export_image=str(image_rel),export_label_sha256=sha(label),supervision_sha256=sha(side))
        return r,dict(id=r['id'],rgb_sha_verified=True,pixel_sha_verified=True,mask_sha_verified=bool(r.get('ego_mask_applied')),added_safe_cells=added_cells)
    accepted=[];audits=[]
    with ThreadPoolExecutor(max_workers=args.workers) as pool, (args.output/'audit.jsonl').open('w') as stream:
        for i,(r,audit) in enumerate(pool.map(process,rows)):
            accepted.append(r);audits.append(audit);stream.write(json.dumps(audit)+'\n')
            if i%1000==0:stream.flush();print(json.dumps(dict(processed=i,total=len(rows),seconds=round(time.time()-start))),flush=True)
    (args.output/'manifest.jsonl').write_text(''.join(json.dumps(r,ensure_ascii=False)+'\n' for r in accepted))
    write_json(args.output/'policy.json',policy)
    for role in ('selection','heldout'):
        shutil.copy2(args.proposal/'full-resplit'/f'{role}.jsonl',args.output/f'reserved-{role}.jsonl')
    shutil.copy2(args.review/'reviews.json',args.output/'reviews.json')
    shutil.copy2(args.background_reviews,args.output/'background-reviews.json')
    excluded=[dict(id=r['id'],reason='ignore_only_not_negative') for r in reserved['train'] if not r['candidate_train_eligible']]
    write_json(args.output/'exclusions.json',excluded)
    train=[r for r in accepted if r['split']=='train'];order,balance=epoch_order(train,policy['seed'],alignment=16)
    (args.output/'epoch-000-order.jsonl').write_text(''.join(json.dumps(dict(id=train[i]['id']))+'\n' for i in order))
    parents={str(args.source):sha(args.source/'manifest.jsonl')}
    verification=verify_export(args.output,accepted,parents)
    for r in accepted:
        if r.get('background_safe_grid'):
            p=args.output/r['background_safe_grid']
            if sha(p)!=r['background_safe_grid_sha256']:raise ValueError('grid roundtrip')
            with np.load(p) as z:
                if np.unpackbits(z['packed']).size!=14400:raise ValueError('grid shape')
        for a in r['annotations']:
            if a['supervision']=='positive' and tiny_reasons(a['xyxy'],r['width'],r['height'],policy,a.get('visible_pixels')):
                raise ValueError('threshold consistency failure')
    verification.update(all_rgb_sha_verified=True,all_pixel_sha_verified=True,whole_reserved_pool_isolation=True,
                        split_membership_changed=True,pretrained_restart_required=True)
    write_json(args.output/'verification.json',verification)
    if not verification['passed']:raise ValueError('export roundtrip failed')
    summary=dict(version=args.output.name,classes=list(CLASSES),splits={role:distribution([r for r in accepted if r['split']==role]) for role in roles},
       balance=balance,added_reviewed_negatives=len(extra),ignored_only_excluded=len(excluded),
       added_safe_cells=sum(x['added_safe_cells'] for x in audits),reserved_pools={k:len(v) for k,v in reserved.items()},
       historical_evaluation_exposure=True,independent_scene_counts_claimed=False,training_started=False)
    write_json(args.output/'summary.json',summary)
    (args.output/'data.yaml').write_text(yaml.safe_dump(guarded_data_yaml(args.output),sort_keys=False))
    meta=json.loads((args.source/'pretrain.json').read_text())
    meta.update(parents=parents,policy_sha256=policy['policy_sha256'],source_stage=str(args.source),
        restart_from_original_pretrained_required=True,final_version=True,
        recommended_training=dict(imgsz=960,rect=False,multi_scale=0,**policy['augmentation']))
    meta['artifact_shas']={p:sha(args.output/p) for p in ['manifest.jsonl','policy.json','data.yaml','verification.json',
         'summary.json','reviews.json','background-reviews.json','reserved-selection.jsonl','reserved-heldout.jsonl','epoch-000-order.jsonl','audit.jsonl']}
    write_json(args.output/'pretrain.json',meta)
    # Publication is separate: independent loss/loader and lineage tests must pass.
    (args.output/'_PREPARED').write_text(sha(args.output/'pretrain.json')+'\n')
    print(json.dumps(dict(prepared=True,images=len(accepted),output=str(args.output))),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('source','proposal','review','policy','data-root','background-reviews','background-candidates','output'):
        p.add_argument('--'+name,required=True,type=Path)
    p.add_argument('--extra-backgrounds',type=int,default=1000)
    p.add_argument('--workers',type=int,default=2)
    args=p.parse_args()
    for key,value in vars(args).items():
        if isinstance(value,Path):setattr(args,key,value.resolve())
    build(args)
