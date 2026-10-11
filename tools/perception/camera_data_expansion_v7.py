#!/usr/bin/env python3
"""Add reviewed source cohorts to an immutable bbox parent; never runs a teacher.

Direct source meanings do not need teacher approval. Individual corrections and
cross-split similarity decisions remain separately bound to the original image.
"""
import argparse
from collections import Counter, defaultdict
import copy
import json
import math
from pathlib import Path
import random

from camera_data_audit import (classify, digest, export_dataset, label_metadata,
    load_label_policy, normalize, read_jsonl, save, sha, training_names, write_jsonl)
from camera_teacher_expansion import apply_decision


def direct_samples(rows, seed=20261009):
    buckets = defaultdict(list)
    for row in rows:
        for label in {a['original_label'] for a in row['annotations']}:
            buckets[row['source'] + '|' + label].append(row)
    result = {}
    for key, members in sorted(buckets.items()):
        members = sorted(members, key=lambda r: r['id'])
        random.Random(str(seed) + key).shuffle(members)
        result[key] = [r['id'] for r in members[:50]]
    return result


def direct_cohort(rows, cohort, policy, admission_policy=None):
    """Require every fixed source/class sample, with whole-image inspection."""
    by_id = {r['id']: r for r in rows}
    if len(by_id) != len(rows) or not rows:
        raise ValueError('empty or duplicate direct cohort')
    if set(by_id) != set(cohort['member_ids']):
        raise ValueError('direct cohort membership changed')
    if any(r['source'] != cohort['source'] for r in rows):
        raise ValueError('direct cohort source changed')
    expected = direct_samples(rows, cohort.get('seed', 20261009))
    if cohort['samples'] != expected:
        raise ValueError('direct fixed sample membership changed')
    reviews = cohort['sample_reviews']
    ids = {i for members in expected.values() for i in members}
    if set(reviews) != ids:
        raise ValueError('direct sample reviews incomplete')
    for ident, decision in reviews.items():
        if (decision.get('image_sha256') != by_id[ident]['image_sha256']
                or not decision.get('correct') or not decision.get('all_targets_annotated')
                or not decision.get('evidence') or not decision.get('reviewer')):
            raise ValueError('direct sample failed or missing bound RGB evidence')
    for row in rows:
        if any(normalize(a['original_label']) in {normalize(label) for label in
                (admission_policy or {}).get('individual_only_source_labels', {}).get(row['source'], [])}
               for a in row['annotations']):
            raise ValueError('source label requires individual RGB review')
        if not any(a.get('training_class') for a in row['annotations']):
            raise ValueError('direct cohort cannot create an unreviewed negative')
        if any(i not in ('review_pending', 'review_rejected') for i in row['issues']):
            raise ValueError('direct source structural issue')
        for ann in row['annotations']:
            canonical, disposition = classify(ann['original_label'], policy, row['source'])
            if disposition == 'ambiguous' or ann['issues']:
                raise ValueError('mixed or structurally invalid annotation in direct cohort')
            if (ann['canonical'], ann['disposition']) != (canonical, disposition):
                raise ValueError('direct cohort contains previously corrected source labels')
    return dict(source=cohort['source'], members=len(rows), samples=len(ids),
                sample_reviews_sha256=digest(json.dumps(reviews, sort_keys=True)),
                teacher_approval_required=False)


def expand(rows, policy, cohorts, decisions, admission_policy=None):
    result = copy.deepcopy(rows)
    by_id = {r['id']: r for r in result}
    if len(by_id) != len(result):
        raise ValueError('duplicate parent IDs')
    changed = set()
    reports = []
    for cohort in cohorts:
        members = [by_id[i] for i in cohort['member_ids']]
        reports.append(direct_cohort(members, cohort, policy, admission_policy))
        for row in members:
            if row['eligible']:
                continue  # A parent admission can never be overwritten.
            if row['id'] in changed:
                raise ValueError('overlapping direct cohorts')
            if any(any(s in reason for s in ('split', 'duplicate', 'representative'))
                   for reason in row['reasons']):
                continue  # Resolve existing leakage evidence separately.
            row['prior_reasons_before_v7'] = row['reasons']
            row['prior_row_issues_before_v7'] = row['issues']
            row.update(eligible=True, reasons=[], issues=[], review_basis='source_class_sample',
                       admission_tier='v7_direct_source_cohort',
                       direct_cohort_evidence=reports[-1])
            # Sample approval is NOT per-image independently complete GT.
            changed.add(row['id'])
    for ident, decision in decisions.items():
        row = by_id[ident]
        if row['eligible'] or ident in changed:
            raise ValueError('individual correction overlaps an accepted/cohort image')
        # Semantic review may not erase old exact/split evidence. pHash candidate
        # flags are rechecked against ALL opposing images by the separate gate.
        if any(('duplicate' in reason or 'representative' in reason or
                ('split' in reason and 'similar' not in reason)) for reason in row['reasons']):
            raise ValueError('individual correction cannot waive duplicate/split evidence')
        updated = apply_decision(row, decision, policy)
        updated.update(review_basis='independent_rgb_review', admission_tier='v7_individual_rgb',
                       prior_reasons_before_v7=row['reasons'])
        by_id[ident] = updated
        changed.add(ident)
    return [by_id[r['id']] for r in rows], changed, reports


def similarity_pairs(rows, changed):
    """Check every pHash pair, not only the first convenient comparison."""
    import numpy as np
    lookup = np.array([bin(i).count('1') for i in range(256)], dtype=np.uint8)
    opposing = {}
    for role in ('train', 'selection', 'heldout'):
        # New admissions in different splits must also be checked against each
        # other. A cohort review cannot waive cross-split scene evidence.
        members = [r for r in rows if r['split'] != role and r.get('phash')]
        opposing[role] = (members, np.array([int(r['phash'], 16) for r in members], dtype=np.uint64))
    pairs = []
    for row in rows:
        if row['id'] not in changed or not row['eligible']:
            continue
        if not row.get('phash'):
            raise ValueError('new image missing perceptual hash')
        members, hashes = opposing[row['split']]
        distances = lookup[np.bitwise_xor(hashes, np.uint64(int(row['phash'], 16))).view(np.uint8).reshape(-1, 8)].sum(axis=1)
        for index in np.flatnonzero(distances <= 6):
            other = members[index]
            pairs.append(dict(new_id=row['id'], opposing_id=other['id'],
                new_image_sha256=row['image_sha256'], opposing_image_sha256=other['image_sha256'],
                distance=int(distances[index])))
    return pairs


def leakage_gate(rows, changed, pairs, reviews):
    pixels = defaultdict(list)
    scenes = defaultdict(set)
    for row in rows:
        pixels[row['pixel_sha256']].append(row)
        scenes[row['split_group']].add(row['split'])
    for row in rows:
        if row['id'] in changed and len(scenes[row['split_group']]) > 1:
            row.update(eligible=False, reasons=['v7_source_group_split_conflict'])
    for members in pixels.values():
        old = [r for r in members if r['eligible'] and r['id'] not in changed]
        new = sorted((r for r in members if r['eligible'] and r['id'] in changed), key=lambda r: r['id'])
        conflict = len({r['split'] for r in members}) > 1
        for index, row in enumerate(new):
            if conflict or old or index:
                row.update(eligible=False, reasons=['v7_exact_pixel_duplicate_or_split_conflict'])
    by_id = {r['id']: r for r in rows}
    known = set()
    for pair in pairs:
        key = pair['new_id'] + ':' + pair['opposing_id']
        known.add(key)
        review = reviews.get(key, {})
        if review:
            if any(review.get(k) != pair[k] for k in ('new_image_sha256', 'opposing_image_sha256')):
                raise ValueError('similarity review image SHA mismatch')
            if not review.get('evidence') or not review.get('reviewer'):
                raise ValueError('similarity review missing evidence')
            if review.get('decision') not in ('different_scene', 'same_scene', 'uncertain'):
                raise ValueError('invalid similarity decision')
        pair['review'] = review
        row = by_id[pair['new_id']]
        if row['eligible'] and review.get('decision') != 'different_scene':
            row.update(eligible=False, reasons=['v7_similarity_review_required' if
                review.get('decision') != 'same_scene' else 'v7_confirmed_cross_split_scene'])
    if set(reviews) - known:
        raise ValueError('similarity review not in current candidate pairs')


def coverage(rows, policy):
    names = training_names(policy)
    counts = {n: {s: dict(images=0, objects=0) for s in ('train','selection','heldout')} for n in names}
    groups = defaultdict(set)
    subtypes = defaultdict(Counter)
    conditions = defaultdict(Counter)
    for row in rows:
        if not row['eligible']:
            continue
        objects = Counter(a['training_class'] for a in row['annotations'] if a.get('training_class'))
        for name, number in objects.items():
            counts[name][row['split']]['images'] += 1
            counts[name][row['split']]['objects'] += number
            groups[(name,row['split'])].add(row['split_group'])
            for field in ('weather','view','tunnel'):
                conditions[(name,row['split'],field)][str(row.get(field,'unknown'))] += 1
        for subtype in {a.get('source_subtype') or a.get('source_semantic_label') for a in row['annotations'] if a.get('training_class')}:
            subtypes[subtype or 'unknown'][row['split']] += 1
    for name in names:
        for split, count in counts[name].items():
            count['recording_or_source_groups'] = len(groups[(name,split)])
            count['verified_independent_scenes'] = None
            count['conditions_by_image'] = {f: dict(conditions[(name,split,f)]) for f in ('weather','view','tunnel')}
    obstacle = policy['groups']['obstacle']
    rare = ['deer','bicycle','motorcycle']
    weights = {name: 1/math.sqrt(max(50,counts[name]['train']['images'])) for name in obstacle+rare}
    pools = {}
    for pool, members, slots in [('obstacles',obstacle,10),('rare_compact',rare,2)]:
        total = sum(weights[n] for n in members)
        pools[pool] = dict(slots=slots, probabilities={n: weights[n]/total for n in members})
    return dict(classes=counts, subtype_images={k:dict(v) for k,v in subtypes.items()},
        group_count_is_not_verified_scene_diversity=True,
        collection_proposal=dict(executable=False, actors_per_layout=12, pools=pools,
            weight='1 / sqrt(max(train_presence_images, 50))',
            excluded_new_spawns=['person','dog','car','bus','truck','traffic_sign','tree','fire','electric_scooter'],
            availability='unverified: next pilot must validate exact API names and ground placement'),
        training_performed=False, collection_started=False)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('parent','reviews','output','dataset'):
        p.add_argument('--'+key,type=Path,required=True)
    p.add_argument('--labels',type=Path,default=Path(__file__).resolve().parents[2]/'config/datasets/camera_object_labels_v2.json')
    p.add_argument('--near-reviews',type=Path)
    p.add_argument('--admission-policy',type=Path,default=Path(__file__).resolve().parents[2]/'config/datasets/camera_object_adoption_v7.json')
    p.add_argument('--prepare-only',action='store_true')
    a=p.parse_args()
    policy=load_label_policy(a.labels)
    rows=list(read_jsonl(a.parent))
    package=json.loads(a.reviews.read_text())
    if package['parent_sha256']!=sha(a.parent):
        raise ValueError('parent SHA mismatch')
    admission_policy=json.loads(a.admission_policy.read_text())
    result,changed,reports=expand(rows,policy,package.get('cohorts',[]),package.get('individual',{}),admission_policy)
    pairs=similarity_pairs(result,changed)
    if a.prepare_only:
        a.output.mkdir(parents=True,exist_ok=False)
        save(a.output/'similarity-pairs.json',pairs)
        write_jsonl(a.output/'candidate-frames.jsonl',[r for r in result if r['id'] in changed])
        print(json.dumps(dict(candidates=len(changed),similarity_pairs=len(pairs))))
        return
    near=json.loads(a.near_reviews.read_text()) if a.near_reviews else {}
    leakage_gate(result,changed,pairs,near)
    # Old admissions and every split assignment are immutable, even if the new
    # review expands the same source's heldout membership.
    for old,new in zip(rows,result):
        if old['split']!=new['split'] or old['split_group']!=new['split_group']:
            raise ValueError('parent split changed')
        if old['eligible'] and old!=new:
            raise ValueError('parent admission changed')
    a.output.mkdir(parents=True,exist_ok=False)
    write_jsonl(a.output/'curated.jsonl',result)
    save(a.output/'similarity-pairs.json',pairs)
    save(a.output/'coverage.json',coverage(result,policy))
    export_dataset(result,policy,a.dataset,True)
    new=[r for r in result if r['id'] in changed and r['eligible']]
    summary=dict(parent=str(a.parent.resolve()),parent_sha256=sha(a.parent),
        reviews_sha256=sha(a.reviews),near_reviews_sha256=sha(a.near_reviews) if a.near_reviews else None,
        labels_sha256=sha(a.labels),source_profiles_sha256=policy['source_profiles_sha256'],
        admission_policy_sha256=sha(a.admission_policy),
        previous_images=sum(r['eligible'] for r in rows),new_images=len(new),
        new_images_by_split=dict(Counter(r['split'] for r in new)),
        new_objects_by_class=dict(Counter(x['training_class'] for r in new for x in r['annotations'] if x.get('training_class'))),
        new_images_by_source=dict(Counter(r['source'] for r in new)),
        pending_changed_images=len(changed)-len(new),direct_cohorts=reports,
        existing_admissions_unchanged=True,existing_split_assignments_unchanged=True,
        expanded_heldout_is_previously_inspected_not_unseen=True,
        dataset=str(a.dataset.resolve()),verification=json.loads((a.dataset/'verification.json').read_text()),
        training_performed=False,collection_started=False)
    save(a.output/'summary.json',summary)
    print(json.dumps(summary))


if __name__=='__main__':
    main()
