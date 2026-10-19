#!/usr/bin/env python3
"""Immutable reviewed label patch, not a replacement for an active training set.

Requires a pinned RGB review ledger. Preserves all parent annotations; changes
whole-image supervision to partial because map semantics are not full16 GT.
"""
import argparse
from collections import Counter
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import cv2
import numpy as np
from PIL import Image
from camera_pretrain_policy import CLASSES, load_policy, view_masks, sha, tiny_reasons
from prepare_fixed_map_drums import silhouette_ratio
from prepare_camera_pretrain import yolo_lines, verify_export


def decide(p, component, rejected):
    reasons=[]
    ratio=silhouette_ratio(component)
    if ratio<.7:reasons.append('possible_joined_instances_or_occlusion')
    x1,y1,x2,y2=p['xyxy']
    if min(x2-x1,y2-y1)<12:reasons.append('insufficient_RGB_identity_for_batch_adoption')
    if p['conflicting_existing_labels']:reasons.append('existing_label_conflict')
    if p['proposal_id'] in rejected:reasons.append('RGB_review_ambiguous_or_non_drum')
    return reasons,ratio


def export(a):
    if a.output.exists():raise ValueError('immutable output exists')
    ledger=json.loads(a.review.read_text())
    if ledger['proposals_sha256']!=sha(a.proposals):raise ValueError('review/proposal mismatch')
    if ledger['status']!='reviewed':raise ValueError('review pending')
    parents={}
    for name in ('reserved-selection.jsonl','reserved-heldout.jsonl','manifest.jsonl'):
        path=a.dataset/name
        if path.exists():parents.update((r['id'],r) for r in map(json.loads,path.open()))
    manifest_sha=sha(a.dataset/'manifest.jsonl');policy=load_policy(a.dataset/'policy.json');masks=view_masks(policy)
    a.output.mkdir(parents=True)
    for part in ('images','labels','supervision'):
        for role in ('train','val','test'):(a.output/part/role).mkdir(parents=True)
    cv2.setNumThreads(1);rejected=set(ledger['rejected_proposal_ids'])
    def job(rec):
        r=parents[rec['id']];sem=Path(rec['semantic'])
        if sha(Path(r['image']))!=rec['image_sha256'] or sha(sem)!=rec['semantic_sha256']:raise ValueError('source changed')
        s=np.array(Image.open(sem).convert('RGB'));mask=(s==[113,178,37]).all(2)
        if r.get('ego_mask_applied'):mask[masks[r['view']].geometry(r['height'],r['width'])[0]]=False
        _,labels,stats,_=cv2.connectedComponentsWithStats(mask.astype('uint8'),8)
        annotations=[];decisions=[]
        for p in rec['proposals']:
            x,y,w,h,area=map(int,stats[p['component_id']])
            if [x,y,x+w,y+h]!=p['xyxy'] or area!=p['visible_pixels']:raise ValueError('component alignment changed')
            reasons,ratio=decide(p,labels[y:y+h,x:x+w]==p['component_id'],rejected)
            decisions.append(dict(p,status='defer' if reasons else 'accepted',reasons=reasons,silhouette_ratio=ratio))
            if reasons:continue
            tiny=tiny_reasons(p['xyxy'],r['width'],r['height'],policy,p['visible_pixels'])
            annotations.append(dict(id='map_drum_'+p['proposal_id'],training_class='traffic_drum',subtype='map_fixed_orange_drum',xyxy=p['xyxy'],visible_pixels=p['visible_pixels'],supervision='ignore' if tiny else 'positive',ignore_reasons=tiny,provenance=dict(method=p['method'],semantic=str(sem),semantic_sha256=rec['semantic_sha256'],review_sha256=sha(a.review),instance_id_is_not_unique=True)))
        if not annotations:return None,dict(id=r['id'],proposals=decisions)
        new=dict(r,annotations=r['annotations']+annotations,supervision='partial',annotation_complete=False,known_class_background=False)
        new['map_drum_review']=dict(parent_supervision=r['supervision'],active_manifest_member=rec['active_manifest_member'],original_annotation_count=len(r['annotations']),added=len(annotations),remaining_unreviewed_map_regions=True)
        new.pop('negative_basis',None)
        role={'selection':'val','heldout':'test'}.get(r['split'],r['split']);image=Path(r['image']);new['export_image']=f'images/{role}/{r["id"]}{image.suffix}'
        (a.output/new['export_image']).symlink_to(image.resolve())
        label=a.output/'labels'/role/(r['id']+'.txt');label.write_text('\n'.join(yolo_lines(new))+'\n');new['export_label_sha256']=sha(label)
        (a.output/'supervision'/role/(r['id']+'.json')).write_text(json.dumps(new,ensure_ascii=False)+'\n')
        return new,dict(id=r['id'],proposals=decisions)
    rows=[];decisions=[]
    with ThreadPoolExecutor(max_workers=2) as pool:
        for row,decision in pool.map(job,map(json.loads,a.proposals.open())):
            if row:rows.append(row)
            decisions.append(decision)
    (a.output/'manifest.jsonl').write_text(''.join(json.dumps(r,ensure_ascii=False)+'\n' for r in rows))
    (a.output/'decisions.jsonl').write_text(''.join(json.dumps(r,ensure_ascii=False)+'\n' for r in decisions))
    verify=verify_export(a.output,rows,{str(a.dataset):manifest_sha})
    # Validate preservation, not merely agreement with the new exporter.
    for r in rows:
        n=r['map_drum_review']['original_annotation_count']
        if r['annotations'][:n]!=parents[r['id']]['annotations']:raise ValueError('parent annotation changed')
    if not verify['passed']:raise ValueError(verify)
    summary=dict(status='verified_label_patch_not_standalone_training_dataset',images=len(rows),added_boxes=sum(r['map_drum_review']['added'] for r in rows),split_images=Counter(r['split'] for r in rows),split_added_boxes=Counter(),corrected_negative_ids=[r['id'] for r in rows if r['map_drum_review']['parent_supervision']=='reviewed_negative'],active_train_images=sum(r['split']=='train' and r['map_drum_review']['active_manifest_member'] for r in rows),deferred_boxes=sum(p['status']=='defer' for d in decisions for p in d['proposals']),parent_manifest_sha256=manifest_sha,review_sha256=sha(a.review),active_training_changed=False,classes=CLASSES)
    for r in rows:summary['split_added_boxes'][r['split']]+=r['map_drum_review']['added']
    (a.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');(a.output/'verification.json').write_text(json.dumps(verify,indent=2)+'\n')
    negative_review=ledger.get('confirmed_incorrect_negative_ids',[])
    corrected=set(summary['corrected_negative_ids'])
    removals=[dict(id=key, action='quarantine_from_negative_pool', reason='RGB-confirmed map drum; identity/size gates do not permit a reliable positive patch', parent_supervision=parents[key]['supervision']) for key in negative_review if key not in corrected]
    if any(parents[key]['supervision']!='reviewed_negative' for key in negative_review):raise ValueError('negative review parent mismatch')
    (a.output/'quarantine-removals.json').write_text(json.dumps(removals,indent=2)+'\n')
    summary['confirmed_incorrect_negative_images']=len(negative_review)
    summary['negative_images_quarantined']=len(removals)
    (a.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    (a.output/'README.txt').write_text('Immutable label patch. No data.yaml: not an independent training dataset. Merge by id into a new reviewed revision; preserve split roles and partial supervision. Never concatenate to parent (duplicates). Active dataset unchanged. Full-image GT completeness is NOT asserted.\n')
    print(json.dumps(summary))

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for arg in ('dataset','proposals','review','output'):p.add_argument('--'+arg,type=Path,required=True)
    export(p.parse_args())
