#!/usr/bin/env python3
"""Restore Capture vehicle lineage, merge reviewed drum patches and prepare v11.

No training, checkpoint mutation or active-dataset update. Uses the existing
SHA/GT/grid/export pipeline. Evaluation rows require individual review records;
partial review remains recall-only. Old train exposure is explicitly recorded.
"""
import argparse,json,shutil
from collections import Counter
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
from camera_pretrain_policy import sha,CLASSES
from prepare_camera_pretrain import read_rows,write_json
from prepare_camera_pretrain_v10 import build


def merge_patch(parent,patch):
    n=patch['map_drum_review']['original_annotation_count']
    if patch['annotations'][:n]!=parent['annotations'] or patch['image_sha256']!=parent['image_sha256']:
        raise ValueError('patch parent identity mismatch: '+parent['id'])
    r=deepcopy(patch)
    for k in ('candidate_train_eligible','candidate_policy','candidate_previous_split'):
        if k in parent:r[k]=parent[k]
    return r


def prepare(a):
    if a.output.exists() or (a.stage/'full-resplit').exists():raise ValueError('choose new immutable output/stage')
    source_sha=sha(a.source/'manifest.jsonl');pool={}
    for role in ('train','selection','heldout'):pool.update((r['id'],r) for r in read_rows(a.pool/(role+'.jsonl')))
    active={r['id']:r for r in read_rows(a.source/'manifest.jsonl')};pool.update(deepcopy(active))
    assignment=json.loads(a.assignment.read_text())['groups'];patches={r['id']:r for r in read_rows(a.patch/'manifest.jsonl')}
    patch_summary=json.loads((a.patch/'summary.json').read_text())
    if patch_summary['parent_manifest_sha256']!=source_sha:raise ValueError('patch parent manifest differs')
    remove={r['id'] for r in json.loads((a.patch/'quarantine-removals.json').read_text())}
    negative=json.loads(a.negative_audit.read_text());ambiguous={r['id'] for r in negative['rows'] if r['standing_pixels']>0}
    if any(not r['semantic_sha_matches'] for r in negative['rows']):raise ValueError('negative audit identity failure')
    old_role={key:r['split'] for key,r in pool.items()};excluded=[];changed=[]
    for key in list(pool):
        r=pool[key]
        if key in remove:excluded.append(dict(id=key,reason='confirmed_incorrect_negative_without_adopted_positive'));del pool[key];continue
        if key in patches:r=merge_patch(r,patches[key])
        historical=r.get('provenance',{}).get('review_record')
        if r['supervision']=='complete' and not r.get('review_record') and historical:
            pinned=(historical.get('image_sha256')==r['image_sha256'] or
                    bool(r.get('review_evidence_sha256') and historical.get('evidence') and
                         sha(historical['evidence'])==r['review_evidence_sha256']))
            if (not historical.get('all_targets_annotated') or historical.get('review_scope')!='camera_objects_v2_16' or
                not pinned):
                raise ValueError('historical RGB review cannot be verified: '+key)
            r['review_record']=deepcopy(historical)
            r['restored_historical_review']=True
        r['split']=assignment[r['split_group']]
        if old_role[key]!=r['split']:changed.append(dict(id=key,before=old_role[key],after=r['split']))
        if r['supervision']=='reviewed_negative' and key in ambiguous:
            r.update(supervision='partial',spatial_background_only=True,known_class_background=False,previous_supervision='reviewed_negative',completeness_reason='standing_object may include target map props; supervise only pinned safe semantic cells')
            r.pop('negative_basis',None)
        r['candidate_train_eligible']=bool(any(x['supervision']=='positive' for x in r['annotations']) or r['supervision']=='reviewed_negative' or r.get('spatial_background_only'))
        pool[key]=r
    reviews=json.loads(a.reviews.read_text());selection=[]
    for key in reviews['retained_selection_ids']:
        r=pool[key]
        if r['split']!='selection' or active[key]['supervision']!='complete':raise ValueError('invalid retained selection')
        selection.append(r)
    for review in reviews['selection']:
        r=pool[review['id']]
        if sha(r['image'])!=review['image_sha256'] or r['split']!='selection':raise ValueError('new evaluation identity/role mismatch')
        if review['decision']=='defer':continue
        r=deepcopy(r);r['review_record']=review;r['evaluation_role']='source_restored_development'
        if review['decision']=='accept':
            if not review['all_targets_annotated'] or review['review_scope']!='camera_objects_v2_16':raise ValueError('missing full16 review')
            r.update(supervision='complete',annotation_complete=True)
        else:r.update(supervision='partial',annotation_complete=False,evaluation_precision_unavailable=True)
        selection.append(r)
    heldout=[deepcopy(r) for r in active.values() if r['split']=='heldout']
    # Do not silently apply new map labels to the frozen final panel.
    if any(assignment[r['split_group']]!='heldout' for r in heldout):raise ValueError('heldout reassignment')
    roles=dict(train=[r for r in pool.values() if r['split']=='train' and r['candidate_train_eligible']],selection=selection,heldout=heldout)
    for role in roles:
        if len({r['id'] for r in roles[role]})!=len(roles[role]):raise ValueError('duplicate role IDs')
    a.stage.mkdir(parents=True,exist_ok=True);(a.stage/'full-resplit').mkdir()
    for role in roles:
        (a.stage/'full-resplit'/(role+'.jsonl')).write_text(''.join(json.dumps(r,ensure_ascii=False)+'\n' for r in pool.values() if r['split']==role))
    shutil.copy2(a.reviews,a.stage/'reviews.json')
    heldout_review_records=[dict(r['review_record'],decision='accept') for r in heldout]
    doc=json.loads((a.stage/'reviews.json').read_text());doc['heldout']=heldout_review_records;write_json(a.stage/'reviews.json',doc)
    oldtrain={r['id'] for r in active.values() if r['split']=='train'}
    plan=dict(source=str(a.source),source_manifest_sha256=source_sha,restored_comp_train_images=sum(r['split_group']=='capture_scenario_family:comp_sample' for r in roles['train']),
      counts={role:len(rows) for role,rows in roles.items()},changed_images=len(changed),heldout_ids_and_annotations_preserved=True,
      new_selection_seen_by_current_model=[r['id'] for r in selection if r['id'] in oldtrain],
      continuation_selection_cannot_be_claimed_independent=True,restart_decision='pending epoch4 assessment; clean new selection requires fresh pretrained',
      spatial_background_only_images=sum(bool(r.get('spatial_background_only')) for r in roles['train']),exclusions=excluded,training_started=False)
    write_json(a.stage/'restoration-plan.json',plan)
    args=SimpleNamespace(source=a.source,proposal=a.stage,review=a.stage,policy=a.source/'policy.json',data_root=a.data_root,
         background_reviews=a.source/'background-reviews.json',background_candidates=Path('/unused'),output=a.output,extra_backgrounds=0,workers=2)
    build(args,prepared=roles)
    # Prepared output is not visible to any running trainer. Finalize metadata
    # only after all export checks; activation is a separate user decision.
    write_json(a.output/'restoration-plan.json',plan)
    meta=json.loads((a.output/'pretrain.json').read_text());meta.update(restoration=True,activation_decision_pending=True)
    meta['artifact_shas']['restoration-plan.json']=sha(a.output/'restoration-plan.json')
    write_json(a.output/'pretrain.json',meta);(a.output/'_PREPARED').write_text(sha(a.output/'pretrain.json')+'\n')
    if sha(a.source/'manifest.jsonl')!=source_sha:raise ValueError('active parent changed')
    print(json.dumps(plan,ensure_ascii=False))

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('source','pool','assignment','patch','negative-audit','reviews','stage','output','data-root'):p.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args()
    for name,value in vars(a).items():setattr(a,name,value.resolve())
    prepare(a)
