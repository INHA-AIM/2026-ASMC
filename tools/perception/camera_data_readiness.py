"""Offline bbox readiness: review bottlenecks, immutable splits and optional masks."""
import copy
from collections import Counter, defaultdict
import html
import json
from pathlib import Path
from urllib.parse import quote

from camera_data_audit import bucket_review_states, classify, training_names, training_label, save


OPTIONAL_SEGMENTATION_ISSUES = {
    'invalid_polygon', 'polygon_out_of_bounds', 'invalid_rle',
    'compressed_rle_needs_decode', 'unlinked_actor_semantic_pixels',
    'unverified_semantic_palette_run',
}


def bbox_rows(rows):
    result=copy.deepcopy(rows)
    for row in result:
        for item in [row,*row['annotations']]:
            warnings=[i for i in item['issues'] if i in OPTIONAL_SEGMENTATION_ISSUES]
            item['auxiliary_segmentation_issues']=sorted(set(item.get('auxiliary_segmentation_issues',[])+warnings))
            item['issues']=[i for i in item['issues'] if i not in OPTIONAL_SEGMENTATION_ISSUES]
        # Visibility is not proved by a missing mask; only independent image review can clear this.
        row['dataset_annotation_kind']='bbox'
    return result


def preserve_splits(rows, previous):
    assignments={r['id']:r for r in previous}
    if len(assignments)!=len(previous):raise ValueError('duplicate previous split IDs')
    group_roles=defaultdict(set)
    for row in previous:group_roles[row['split_group']].add(row['split'])
    result=copy.deepcopy(rows)
    for row in result:
        old=assignments.get(row['id'])
        if old:
            if old.get('image_sha256')!=row.get('image_sha256'):
                raise ValueError('previous split image SHA changed')
            if old['split_group']!=row['split_group']:
                raise ValueError('previous scene group changed')
            role=old['split']
        else:
            roles=group_roles.get(row['split_group'],set())
            if len(roles)>1:raise ValueError('ambiguous previous scene split')
            role=next(iter(roles),row.get('frozen_split'))
        if role not in (None,'train','selection','heldout'):
            raise ValueError('invalid previous split')
        if row.get('frozen_split') and role!=row['frozen_split']:
            raise ValueError('Capture frozen split changed')
        if role:
            row['frozen_split']=role
            row['split_preservation_basis']='previous_image' if old else 'previous_scene'
    return result


def readiness_report(rows, policy, reviews, samples):
    # Bucket promotion tests the original source mapping, not already repaired boxes.
    sample_ids={i for ids in samples.values() for i in ids}
    original_samples=copy.deepcopy([r for r in rows if r['id'] in sample_ids])
    for row in original_samples:
        for ann in row['annotations']:
            ann['canonical'],ann['disposition']=classify(ann['original_label'],policy,row['source'])
            if ann.get('original_xyxy') is not None:ann['xyxy']=ann['original_xyxy']
    states=bucket_review_states(original_samples,policy,reviews,samples)
    buckets=defaultdict(list)
    by_id={r['id']:r for r in rows}
    for row in rows:
        for label in {a['original_label'] for a in row['annotations']} or {'__empty__'}:
            buckets[row['source']+'|'+str(label)].append(row)
    accepted=[r for r in rows if r['eligible']]
    classes={name:{role:dict(images=0,objects=0,groups=0) for role in ('train','selection','heldout')}
             for name in training_names(policy)}
    subtypes=defaultdict(Counter);groups=defaultdict(set);conditions=defaultdict(Counter)
    size_bins=Counter();optional=Counter();class_conditions=defaultdict(Counter);class_sizes=defaultdict(Counter)
    for row in accepted:
        for field in ('weather','tunnel','view','distance'):
            conditions[field][str(row[field]) if row.get(field) is not None else 'unknown']+=1
        optional.update(row.get('auxiliary_segmentation_issues',[]))
        names=Counter(training_label(a) for a in row['annotations'] if training_label(a))
        for name,count in names.items():
            target=classes[name][row['split']];target['images']+=1;target['objects']+=count
            groups[(name,row['split'])].add(row['split_cluster'])
            for field in ('weather','tunnel','view','distance'):
                class_conditions[(name,row['split'],field)][str(row[field]) if row.get(field) is not None else 'unknown']+=1
        for ann in row['annotations']:
            optional.update(ann.get('auxiliary_segmentation_issues',[]))
            if training_label(ann):
                subtypes[ann.get('source_semantic_label',ann['canonical'])][row['split']]+=1
                a=ann['xyxy'];long=max(a[2]-a[0],a[3]-a[1]);short=min(a[2]-a[0],a[3]-a[1])
                size='below16x8' if long<16 or short<8 else ('16x8_to24x12' if long<24 or short<12 else 'at_least24x12')
                size_bins[size]+=1;class_sizes[(training_label(ann),row['split'])][size]+=1
    for name in classes:
        for role in classes[name]:
            classes[name][role]['groups']=len(groups[(name,role)])
            classes[name][role]['conditions_by_image']={field:dict(class_conditions[(name,role,field)])
                for field in ('weather','tunnel','view','distance')}
            classes[name][role]['source_pixel_sizes_by_object']=dict(class_sizes[(name,role)])
    progress=[]
    for key,items in buckets.items():
        if not items[0]['source'].startswith('roboflow:'):continue
        source,label=key.rsplit('|',1);target,disposition=classify(label,policy,source)
        name=policy.get('training_map',{}).get(target,target)
        state=states.get(key,dict(sample_ids=[],verified_ids=[],rejected_ids=[],approved=False,state='pending'))
        problems=Counter(reason for r in items for reason in r['reasons'])
        structural=Counter(i for r in items for i in [*r['issues'],*(x for a in r['annotations'] for x in a['issues'])])
        priority=(0 if name in ('box','plastic_barrier') else 1 if name not in
                  ('person','car','bus','truck','dog') and disposition=='candidate' else 2 if disposition=='candidate' else 3)
        needed=[role for role in ('train','selection','heldout') if name in classes and not classes[name][role]['objects']]
        progress.append(dict(bucket=key,source=source,original_label=label,canonical=target,
            training_class=name,meaning=disposition,images=len(items),accepted=sum(r['eligible'] for r in items),
            structure_issues=dict(structural),sample=state,
            stale_review_ids=[i for i in state['sample_ids'] if i in reviews and
                reviews[i].get('image_sha256')!=by_id[i].get('image_sha256')],
            split_issues={k:v for k,v in problems.items() if any(s in k for s in ('split','duplicate','search_incomplete'))},
            blockers=dict(problems),individual_accepted=sum(r['eligible'] and r['review_basis']=='individual' for r in items),
            available_splits=dict(Counter(r['split'] for r in items)),missing_class_splits=needed,priority=priority))
    progress.sort(key=lambda r:(r['priority'],-sum(r['available_splits'].get(s,0) for s in r['missing_class_splits']),r['bucket']))
    missing={name:[role for role,v in counts.items() if not v['objects']] for name,counts in classes.items()}
    return dict(schema_version=1,images=len(rows),accepted_images=len(accepted),
        class_order=training_names(policy),classes=classes,semantic_subtypes={k:dict(v) for k,v in subtypes.items()},
        conditions={k:dict(v) for k,v in conditions.items()},source_pixel_size_bins=dict(size_bins),
        optional_segmentation_diagnostics=dict(optional),buckets=progress,
        missing_class_splits={k:v for k,v in missing.items() if v},
        target_split_coverage=not any(missing.values()),
        interpretation='Source meaning, sampled completeness, structure and split blockers are separate. A pending bucket is not a bad dataset. Coverage alone does not prove enough data for training.',
        teacher_approval_required_for_direct_mapping=False,training_performed=False,
        additional_collection_priorities=['plastic_barrier','box','tunnel','FOGGY','occlusion','distance'],
        collection_started=False)


def progress_gallery(report, output):
    save(output/'review-progress.json',report)
    esc=lambda x:html.escape(str(x))
    rows=[]
    for r in report['buckets']:
        state=r['sample']
        rows.append('<tr><td><a href="index.html?bucket='+quote(r['bucket'],safe='')+'">'+esc(r['bucket'])+'</a></td><td>'+esc(r['training_class'] or r['meaning'])+'</td><td>'+str(r['accepted'])+'/'+str(r['images'])+'</td><td>'+esc(state['state'])+' '+str(len(state['verified_ids']))+'/'+str(len(state['sample_ids']))+'</td><td>'+esc(r['structure_issues'])+'</td><td>'+esc(r['split_issues'])+'</td><td>'+esc(r['missing_class_splits'])+'</td></tr>')
    page='''<!doctype html><meta charset="utf-8"><title>2026-10-10 · bbox v4 검수 진행표</title>
<style>body{font:15px system-ui;margin:24px}table{border-collapse:collapse}td,th{border:1px solid #bbb;padding:8px}pre{white-space:pre-wrap}</style>
<h1>2026-10-10 · bbox v4 검수 진행표</h1><p>명확한 원본 라벨은 teacher 승인 없이 활용합니다. 대기는 자료 불량을 의미하지 않습니다. 박스/PE 방벽을 우선 표시하며 자료 채택과 전체 학습 준비를 구분합니다.</p>
<p><a href="index.html">개별 검수</a> · <a href="review-progress.json">전체 수치·대기 원인</a></p>
<h2>클래스·split별 준비 상태</h2><pre>__CLASSES__</pre><p>자료가 없는 split: __MISSING__. 충분한 학습량이나 정확도를 보장하는 판정은 아닙니다.</p>
<table><tr><th>소스 | 원본 라벨 — 고정 표본 열기</th><th>목표 종류</th><th>채택/원본</th><th>표본 확인</th><th>구조 문제</th><th>분할 문제</th><th>부족 split</th></tr>__ROWS__</table>'''
    (output/'progress.html').write_text(page.replace('__ROWS__',''.join(rows)).replace('__CLASSES__',esc(json.dumps(report['classes'],ensure_ascii=False,indent=2))).replace('__MISSING__',esc(report['missing_class_splits'])))
