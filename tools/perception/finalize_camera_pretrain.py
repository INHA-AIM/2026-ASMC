#!/usr/bin/env python3
"""Publish a new immutable v8 version from verified positives and reviewed GT negatives."""
import argparse
from collections import Counter, defaultdict, deque
from datetime import datetime, timezone
import json
from pathlib import Path

import yaml

from camera_pretrain_policy import CLASSES, class_images, epoch_order, sha, uniform, guarded_data_yaml
from prepare_camera_pretrain import (build_gallery, read_rows, verify_export, view_masks,
                                    write_json, yolo_lines)


def finalize(args):
    if args.output.exists():
        raise ValueError('final version already exists')
    stage=args.prepared
    if not json.loads((stage/'verification.json').read_text())['passed']:
        raise ValueError('positive stage failed verification')
    previous=json.loads((stage/'pretrain.json').read_text())
    policy=json.loads((stage/'policy.json').read_text())
    rows=list(read_rows(stage/'manifest.jsonl'))
    # Older individually corrected annotations can already have original_xyxy.
    # Keep that source geometry in addition to the pre-mask geometry.
    prior_originals={}
    for parent in previous['parents']:
        for old in read_rows(Path(parent)/'manifest.jsonl'):
            fixes={str(a.get('id',i)):a['original_xyxy'] for i,a in enumerate(old['annotations'])
                   if a.get('original_xyxy') is not None}
            if fixes:prior_originals[old.get('id',old.get('key'))]=fixes
    for r in rows:
        for ann in r['annotations']:
            prior=prior_originals.get(r['id'],{}).get(ann.get('id'))
            if prior is not None:ann['source_original_xyxy']=prior
        if r.get('provenance',{}).get('source_run'):
            r['supervision']='partial'
            r['completeness_reason']='native actor GT does not establish coverage of fixed map props'
    parent_manifest=sha(stage/'manifest.jsonl')
    review=json.loads(args.reviews.read_text())
    candidates_path=args.backgrounds/'candidates.jsonl'
    proposal=json.loads((args.backgrounds/'review.json').read_text())
    if sha(candidates_path)!=proposal['candidate_manifest_sha256'] or review['proposal_sha256']!=sha(args.backgrounds/'review.json'):
        raise ValueError('background proposal/review SHA mismatch')
    approved={g for g,v in review['groups'].items() if v['decision']=='accept'}
    individual=set(review.get('accepted_image_ids',[]))
    candidates=list(read_rows(candidates_path))
    by_group=defaultdict(list)
    for r in candidates:
        if r['background_review_group'] in approved or r['id'] in individual:
            by_group[r['background_review_group']].append(r)
    queues=deque(deque(sorted(v,key=lambda r:uniform(policy['seed'],r['id'])))
                 for g,v in sorted(by_group.items()))
    seen={r['pixel_sha256'] for r in rows}
    seen_roles={r['split_group']:r['split'] for r in rows}
    added=[];rejected=Counter()
    while queues and len(added)<policy['background_limit']:
        q=queues.popleft();r=q.popleft()
        if q:queues.append(q)
        if r['pixel_sha256'] in seen:
            rejected['exact_duplicate']+=1;continue
        if r['split_group'] in seen_roles and seen_roles[r['split_group']]!='train':
            rejected['frozen_evaluation_lineage']+=1;continue
        for file, expected in [(r['image'],r['image_sha256']),
             (r['provenance']['semantic'],r['provenance']['semantic_sha256']),
             (r['provenance']['instance'],r['provenance']['instance_sha256'])]:
            if sha(file)!=expected:raise ValueError('background source changed since review')
        r['provenance']['background_reviews_sha256']=sha(args.reviews)
        added.append(r);seen.add(r['pixel_sha256'])
    rows.extend(added)
    args.output.mkdir(parents=True)
    for folder in ('images','labels','supervision'):
        for role in ('train','val','test'):(args.output/folder/role).mkdir(parents=True)
    for n,r in enumerate(rows):
        if n%10000==0:print(json.dumps(dict(finalized=n,total=len(rows))),flush=True)
        role={'selection':'val','heldout':'test'}.get(r['split'],r['split'])
        image=Path(r['image'])
        if sha(image)!=r['image_sha256']:raise ValueError('source RGB changed since positive preparation')
        r['export_image']=f'images/{role}/{r["id"]}{image.suffix.lower()}'
        (args.output/r['export_image']).symlink_to(image.resolve())
        label=args.output/'labels'/role/(r['id']+'.txt');lines=yolo_lines(r)
        label.write_text('\n'.join(lines)+('\n' if lines else ''));r['export_label_sha256']=sha(label)
        write_json(args.output/'supervision'/role/(r['id']+'.json'),dict(supervision=r['supervision'],
            annotations=r['annotations'],image_sha256=r['image_sha256'],policy_sha256=policy['policy_sha256']))
    (args.output/'manifest.jsonl').write_text(''.join(json.dumps(r,ensure_ascii=False)+'\n' for r in rows))
    write_json(args.output/'policy.json',policy)
    write_json(args.output/'background-reviews.json',review)
    (args.output/'exclusions.jsonl').write_bytes((stage/'exclusions.jsonl').read_bytes())
    train=[r for r in rows if r['split']=='train']
    order,balance=epoch_order(train,policy['seed'],alignment=16)
    (args.output/'epoch-000-order.jsonl').write_text(''.join(json.dumps(dict(id=train[i]['id']))+'\n' for i in order))
    counts=Counter((r['split'],a['supervision']) for r in rows for a in r['annotations'])
    states=Counter((r['split'],r['supervision']) for r in rows)
    per_class=defaultdict(Counter);per_view=defaultdict(Counter);per_source=defaultdict(Counter)
    subtypes=defaultdict(Counter)
    for r in rows:
        for a in r['annotations']:
            label=a.get('training_class')
            if label:
                per_class[label][r['split']+'/'+a['supervision']]+=1
                per_view[r['view']][a['supervision']]+=1
                per_source[r['source']][a['supervision']]+=1
                subtypes[a.get('subtype') or a.get('source_semantic_label') or label][a['supervision']]+=1
    summary=dict(version=args.output.name,created_utc=datetime.now(timezone.utc).isoformat(),classes=list(CLASSES),
        images_by_split=dict(Counter(r['split'] for r in rows)),
        objects={s+'/'+v:n for (s,v),n in counts.items()},
        supervision_images={s+'/'+v:n for (s,v),n in states.items()},
        class_objects={c:dict(per_class[c]) for c in CLASSES},train_class_images=dict(class_images(train)),
        class_eval_images={role:dict(class_images([r for r in rows if r['split']==role])) for role in ('selection','heldout')},
        view_objects={c:dict(v) for c,v in per_view.items()},source_objects={c:dict(v) for c,v in per_source.items()},
        subtype_objects={c:dict(v) for c,v in subtypes.items()},balance=balance,
        background_images=len(added),background_rejections=dict(rejected),
        source_positive_stage_manifest_sha256=parent_manifest,
        original_source_train_images=json.loads((stage/'summary.json').read_text())['source_train_images'],training_performed=False,
        exclusion_reasons=json.loads((stage/'summary.json').read_text())['exclusion_reasons'],
        evaluation_splits_moved=False,original_files_changed=False,
        independent_scene_counts_claimed=False,whole_stack_acceptance=False)
    write_json(args.output/'summary.json',summary)
    verified=verify_export(args.output,rows,previous['parents'])
    frozen={r['id']:r['split'] for r in read_rows(stage/'manifest.jsonl') if r['split']!='train'}
    actual={r['id']:r['split'] for r in rows if r['split']!='train'}
    if frozen!=actual:verified['failures'].append(dict(reason='frozen evaluation membership changed'))
    verified['passed']=not verified['failures']
    verified['frozen_evaluation_images']=len(actual)
    write_json(args.output/'verification.json',verified)
    (args.output/'data.yaml').write_text(yaml.safe_dump(guarded_data_yaml(args.output),sort_keys=False))
    meta=dict(previous,parents=dict(previous['parents'],**{str(stage):parent_manifest}),
        artifact_shas={p:sha(args.output/p) for p in ('manifest.jsonl','policy.json','epoch-000-order.jsonl','background-reviews.json','data.yaml')},
        final_version=True,source_stage=str(stage.resolve()))
    write_json(args.output/'pretrain.json',meta)
    examples=defaultdict(list)
    for r in rows:
        kinds={a['supervision'] for a in r['annotations']}
        if r['supervision']=='reviewed_negative':kinds.add('reviewed_negative')
        if r['supervision']=='partial':kinds.add('partial')
        if any(a.get('ego_mask_intersection') for a in r['annotations']):kinds.add('mask_edge')
        for k in kinds:
            if len(examples[k])<10:examples[k].append(r)
    if args.gallery:build_gallery(args.gallery,summary,examples,view_masks(policy))
    if args.gallery:
        # Explicit rejected negatives illustrate why absence from actor GT is
        # insufficient. No boxes or object identities are invented here.
        rejected_ids={item['id'] for item in review.get('exceptions',[])}
        rejected_examples=[r for r in candidates if r['id'] in rejected_ids][:12]
        if rejected_examples:
            build_gallery(args.gallery/'rejected-backgrounds',
                dict(reason='Fixed map props visible in RGB but absent from actor semantic GT; NOT adopted as negatives'),
                {'rejected_background':rejected_examples},view_masks(policy))
            with (args.gallery/'index.html').open('a') as f:
                f.write('<p><a href="rejected-backgrounds/index.html">Rejected background examples: fixed map props missing from actor GT</a></p>')
    if sha(stage/'manifest.jsonl')!=parent_manifest:raise ValueError('immutable positive stage changed')
    if not verified['passed']:raise ValueError('final validation failed')
    (args.output/'_SUCCESS').write_text(sha(args.output/'pretrain.json')+'\n')
    print(json.dumps(dict(images=summary['images_by_split'],objects=summary['objects'],backgrounds=len(added),verified=True),indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--prepared',type=Path,required=True);p.add_argument('--backgrounds',type=Path,required=True)
    p.add_argument('--reviews',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--gallery',type=Path)
    finalize(p.parse_args())
