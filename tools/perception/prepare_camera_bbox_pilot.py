#!/usr/bin/env python3
"""Deterministic image sampling; never remove boxes or move frozen partitions."""
import argparse
from collections import Counter, defaultdict
import copy
import hashlib
import json
import math
from pathlib import Path

from camera_data_audit import (export_dataset, load_label_policy, read_jsonl,
                               save, sha, training_label, write_jsonl)


def classes(row):
    return {training_label(a) for a in row['annotations'] if training_label(a)}


def family(row):
    if row['source'].startswith('capture:'):
        # 40-frame windows are correlation accounting, not independent scenes.
        view, frame = row['source_key'].rsplit('/', 1)
        return (row['source'], view, int(frame)//40)
    return (row['source'], row.get('augmentation_family') or row['id'])


def uniform(seed, identity):
    n = int(hashlib.sha256(f'{seed}:{identity}'.encode()).hexdigest()[:16], 16)
    return (n + 1) / (2**64 + 1)


def sample_train(rows, config, limit=None):
    rows = sorted((r for r in rows if r['eligible'] and r['split']=='train'), key=lambda r:r['id'])
    limit = min(limit or config['max_train_images'], len(rows))
    frequency = Counter(c for r in rows for c in classes(r))
    rare = {c for c,n in frequency.items() if n < config['rare_image_threshold']}
    priority = set(config['priority_classes'])
    source_cap = math.floor(limit * config['source_fraction_cap'])
    capture_cap = math.floor(limit * config['capture_fraction_cap'])
    chosen=[]; sources=Counter(); families=Counter(); capture=0
    deferred=[]; blocked=Counter()
    def rank(row):
        names=classes(row)
        tier=0 if names & rare else 1 if names & priority else 2
        weight=sum(1/math.sqrt(frequency[c]) for c in names) or 1
        return tier, -math.log(uniform(config['seed'], row['id'])) / weight, row['id']
    ranked=sorted(rows,key=rank)
    def add(row):
        nonlocal capture
        is_capture=row['source'].startswith('capture:')
        if sources[row['source']] >= source_cap or (is_capture and capture >= capture_cap):
            blocked['source_or_capture_cap']+=1
            return False
        chosen.append(copy.deepcopy(row)); sources[row['source']]+=1
        capture+=int(is_capture);families[family(row)]+=1
        return True
    # Retain scarce positives first, then spread the rest over distinct lineages.
    for tier in range(3):
        deferred=[]
        for row in ranked:
            if len(chosen)==limit:break
            if rank(row)[0]!=tier:continue
            if tier and families[family(row)]:
                deferred.append(row);continue
            add(row)
        for row in deferred:
            if len(chosen)==limit:break
            add(row)
    chosen.sort(key=lambda r:(uniform(config['seed'], 'order:'+r['id']),r['id']))
    got=Counter(c for r in chosen for c in classes(r))
    return chosen, dict(requested=limit, selected=len(chosen), image_frequency_before=dict(frequency),
        image_frequency_after=dict(got), rare_classes=sorted(rare),
        rare_images_not_selected={c:frequency[c]-got[c] for c in sorted(rare)},
        source_images=dict(sources), capture_images=capture, source_cap=source_cap,
        capture_cap=capture_cap, lineage_groups=len(families),
        correlated_extra_images=sum(n-1 for n in families.values()),
        blocked_attempts=dict(blocked), all_target_boxes_preserved=True)


def evaluation_candidates(rows, train_rows):
    """Separate partial-label recall data from complete, subtype-resolved GT."""
    forbidden_pixels={r['pixel_sha256'] for r in train_rows if r['split']=='train'}
    result=[]; rejected=Counter()
    for row in rows:
        if row['split'] not in ('selection','heldout'):continue
        if row['pixel_sha256'] in forbidden_pixels or any(
            'cross_split' in reason or 'duplicate' in reason or 'missing_capture_bbox' in reason
            for reason in row.get('reasons',[])):
            rejected['leak_or_missing_source']+=1;continue
        if row.get('issues') or not classes(row):
            rejected['invalid_or_no_resolved_gt']+=1;continue
        # Unresolved boxes are retained as ignore regions for recall-only inspection.
        item=copy.deepcopy(row)
        review=row.get('review_record') or {}
        complete=(row.get('annotation_complete') is True and
                  row.get('review_scope')=='camera_objects_v2_16' and
                  review.get('decision')=='accept' and review.get('all_targets_annotated') is True and
                  not any(a['disposition']=='ambiguous' or a.get('issues') for a in row['annotations']))
        item['evaluation_scope']='complete' if complete else 'recall_only'
        item['evaluation_original_eligible']=row['eligible']
        item['eligible']=True
        result.append(item)
    return result,dict(rejected)


def sample_evaluation(rows, role, maximum, seed):
    rows=sorted((r for r in rows if r['split']==role),key=lambda r:r['id'])
    frequency=Counter(c for r in rows for c in classes(r))
    selected=[];remaining=list(rows);coverage=Counter()
    # Greedy deficit coverage, deterministic hash tie break. No metric/prediction input.
    while remaining and len(selected)<maximum:
        def score(r):
            return (sum(1/(1+coverage[c])/math.sqrt(frequency[c]) for c in classes(r)),
                    uniform(seed,r['id']))
        row=max(remaining,key=score);remaining.remove(row);selected.append(row)
        coverage.update(classes(row))
    return sorted(selected,key=lambda r:r['id'])


def verify_membership(source, chosen):
    originals={r['id']:r for r in source}
    seen=set();pixels=defaultdict(set);groups=defaultdict(set)
    for row in chosen:
        old=originals[row['id']]
        if row['id'] in seen:raise ValueError('duplicate pilot image')
        if row['split']!=old['split']:raise ValueError('frozen split changed')
        if row['annotations']!=old['annotations']:raise ValueError('annotations changed')
        if row['image_sha256']!=old['image_sha256']:raise ValueError('source SHA changed')
        seen.add(row['id']);pixels[row['pixel_sha256']].add(row['split'])
        groups[row.get('split_cluster',row['split_group'])].add(row['split'])
    if any(len(v)>1 for v in pixels.values()):raise ValueError('pixel leakage')
    if any(len(v)>1 for v in groups.values()):raise ValueError('group leakage')
    return dict(images=len(chosen),pixel_leaks=0,group_leaks=0,original_annotations_preserved=True)


def run(args):
    cfg=json.loads(args.config.read_text());data=args.data_root.resolve()
    source=data/cfg['source_dataset'];raw=list(read_jsonl(data/cfg['source_curated']))
    accepted=list(read_jsonl(source/'manifest.jsonl'))
    train,stats=sample_train(accepted,cfg,args.limit)
    candidates,rejections=evaluation_candidates(raw,accepted)
    evaluation=[r for role in ('selection','heldout') for r in sample_evaluation(
        candidates,role,cfg['max_eval_images_per_split'],cfg['seed'])]
    chosen=train+evaluation
    check=verify_membership(raw,chosen)
    policy=load_label_policy(args.config.parent.parent/'datasets/camera_object_labels_v2.json')
    export_dataset(chosen,policy,args.output,fixed_class_order=True)
    # Every sample keeps hashes, original annotations, source/run/view and lineage.
    manifest=sha(args.output/'manifest.jsonl')
    save(args.output/'sampling.json',dict(config_sha256=sha(args.config),source_manifest_sha256=sha(source/'manifest.jsonl'),
        source_curated_sha256=sha(data/cfg['source_curated']),pilot_manifest_sha256=manifest,
        train=stats,evaluation_candidate_rejections=rejections,verification=check,
        evaluation={role:dict(images=sum(r['split']==role for r in evaluation),
            complete_images=sum(r['split']==role and r['evaluation_scope']=='complete' for r in evaluation),
            class_gt=dict(Counter(training_label(a) for r in evaluation if r['split']==role
                for a in r['annotations'] if training_label(a)))) for role in ('selection','heldout')},
        seed=cfg['seed'],unselected_training_images=len([r for r in accepted if r['split']=='train'])-len(train)))
    return dict(output=str(args.output),sampling=json.loads((args.output/'sampling.json').read_text()))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,required=True)
    parser.add_argument('--data-root',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--limit',type=int)
    print(json.dumps(run(parser.parse_args()),ensure_ascii=False,indent=2))
