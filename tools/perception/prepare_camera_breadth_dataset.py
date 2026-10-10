#!/usr/bin/env python3
"""Export source-mapped, possibly incomplete training labels under explicit policy.

The strict reviewed dataset and its evaluation assignments remain immutable.
This admission route does not assert complete 16-class image annotation.
"""
import argparse
import copy
from collections import Counter, defaultdict
import html
import json
from pathlib import Path

from camera_data_audit import (export_dataset, label_metadata, load_label_policy,
    normalize, read_jsonl, save, semantic_names, sha, training_names, valid_box, write_jsonl)


def expand(rows, policy, reviews, config=None):
    config = config or {}
    result = copy.deepcopy(rows)
    candidates = set()
    rejected = Counter()
    for row in result:
        if row['eligible']:
            row['admission_tier'] = 'previous_reviewed_subset'
            continue
        if not row['source'].startswith('roboflow:') or row['split'] != 'train':
            continue
        if any('split' in r or 'duplicate' in r for r in row['reasons']):
            rejected['split_or_duplicate_review'] += 1
            continue
        if not row['annotations']:
            rejected['empty_annotations'] += 1
            continue
        if any(not valid_box(a['xyxy'], row['width'], row['height'])
               for a in row['annotations']):
            rejected['invalid_geometry'] += 1
            continue
        decision = reviews.get(row['id'], {})
        # Only already inspected class changes, never teacher proposals or new boxes.
        note = decision.get('notes', '').lower()
        trusted = (decision.get('reviewer', '').startswith('Codex independent RGB')
                   or decision.get('human_confirmed') is True)
        uncertain = any(t in note for t in ('uncertain', 'ambiguous', 'unconfirmed',
                                           'insufficient', 'cannot be securely'))
        overrides = decision.get('overrides', [])
        resolved_ids = set()
        if (trusted and not uncertain and not decision.get('requires_human_confirmation')
                and not decision.get('additions')
                and not any('xyxy' in a for a in overrides)):
            if overrides and decision.get('image_sha256') != row['image_sha256']:
                raise ValueError('class review image SHA mismatch')
            by_id = {a['id']: a for a in row['annotations']}
            for change in overrides:
                if str(change['id']) not in by_id:
                    raise ValueError('class review annotation ID mismatch')
                target = change.get('canonical')
                if target not in semantic_names(policy):
                    raise ValueError('breadth review must resolve a target class')
                ann = by_id[str(change['id'])]
                ann.update(canonical=target, disposition='candidate')
                label_metadata(ann, policy)
                resolved_ids.add(str(change['id']))
        # This source's PE label is known to mix drums and rectangular barriers.
        # Vehicle subclass tolerance does not resolve those different obstacles.
        needs_review = config.get('individual_label_review', {}).get(row['source'], [])
        if any(normalize(a['original_label']) in needs_review and str(a['id']) not in resolved_ids
               for a in row['annotations']):
            rejected['known_mixed_source_label'] += 1
            continue
        if any(a['disposition'] == 'ambiguous' for a in row['annotations']):
            rejected['unresolved_mixed_or_composite'] += 1
            continue
        if not any(a['canonical'] for a in row['annotations']):
            rejected['excluded_only_not_assumed_negative'] += 1
            continue
        candidates.add(row['id'])
        row.update(eligible=True, previous_admission_reasons=list(row['reasons']), reasons=[],
                   admission_tier='source_mapped_breadth_training',
                   review_basis='user_authorized_source_mapping',
                   breadth_quality=dict(complete_target_annotation_asserted=False,
                       vehicle_subclass_noise_accepted=True,
                       original_review_decision=decision.get('decision'),
                       existing_rgb_class_overrides_applied=bool(resolved_ids),
                       teacher_truth_used=False))
    # Prefer an existing reviewed representative, then deterministic source/ID.
    # Do not choose a version merely because it contains more boxes.
    pixels = defaultdict(list)
    all_roles = defaultdict(set)
    for row in result:
        all_roles[row['pixel_sha256']].add(row['split'])
        if row['eligible']:
            pixels[row['pixel_sha256']].append(row)
    for pixel, group in pixels.items():
        if len(all_roles[pixel]) > 1:
            for row in group:
                if row['id'] in candidates:
                    row['eligible'] = False
                    row['reasons'] = ['breadth_exact_pixel_cross_split']
                    rejected['exact_pixel_cross_split'] += 1
            continue
        ranked = sorted(group, key=lambda r: (r['id'] in candidates, r['source'], r['id']))
        for row in ranked[1:]:
            if row['id'] not in candidates:
                raise ValueError('previous reviewed subset contains duplicate representatives')
            row['eligible'] = False
            row['reasons'] = ['breadth_exact_pixel_representative_preserved']
            rejected['exact_pixel_duplicate'] += 1
    added = [r for r in result if r['id'] in candidates and r['eligible']]
    return result, dict(new_images=len(added),
        new_objects=sum(bool(a['canonical']) for r in added for a in r['annotations']),
        new_by_source=dict(Counter(r['source'] for r in added)),
        new_classes=dict(Counter(a['training_class'] for r in added
                                for a in r['annotations'] if a['canonical'])),
        rejected_training_candidates=dict(rejected),
        new_images_with_previous_individual_review_failure=sum(
            r['breadth_quality']['original_review_decision'] == 'reject' for r in added),
        new_images_with_previous_sample_failure=sum(
            'source_class_sample_failed' in r['previous_admission_reasons'] for r in added),
        complete_target_annotation_asserted=False)


def run(args):
    policy = load_label_policy(args.labels)
    config = json.loads(args.config.read_text())
    if config.get('admission_policy') != 'user_authorized_breadth_training_v1':
        raise ValueError('explicit breadth admission policy required')
    before = list(read_jsonl(args.previous))
    reviews = json.loads(args.reviews.read_text())
    rows, stats = expand(before, policy, reviews, config)
    by_id = {r['id']: r for r in rows}
    assert all(r['split'] == by_id[r['id']]['split'] for r in before)
    assert all(by_id[r['id']]['eligible'] for r in before if r['eligible'])
    assert all([(a['id'], a['xyxy']) for a in r['annotations']] ==
               [(a['id'], a['xyxy']) for a in by_id[r['id']]['annotations']] for r in before)
    assert all(r['eligible'] == by_id[r['id']]['eligible'] and r['annotations'] ==
               by_id[r['id']]['annotations'] for r in before if r['split'] != 'train')
    args.output.mkdir(parents=True, exist_ok=False)
    write_jsonl(args.output/'curated.jsonl', rows)
    export_dataset(rows, policy, args.export, fixed_class_order=True)
    accepted = [r for r in rows if r['eligible']]
    summary = dict(schema_version=1, policy=config['admission_policy'],
        previous_curated_sha256=sha(args.previous), config_sha256=sha(args.config),
        labels_sha256=sha(args.labels), reviews_sha256=sha(args.reviews),
        source_profiles_sha256=policy['source_profiles_sha256'],
        previous_images=sum(r['eligible'] for r in before), images=len(accepted),
        roboflow_images=sum(r['source'].startswith('roboflow:') for r in accepted),
        capture_images=sum(r['source'].startswith('capture:') for r in accepted),
        objects=sum(bool(a['canonical']) for r in accepted for a in r['annotations']),
        splits=dict(Counter(r['split'] for r in accepted)),
        objects_by_class=dict(Counter(a['training_class'] for r in accepted
                                    for a in r['annotations'] if a['canonical'])),
        quality_tiers=dict(Counter(r['admission_tier'] for r in accepted)),
        evaluation_membership_and_labels_unchanged=True, original_geometry_changes=0,
        existing_split_changes=0, previous_accepted_lost=0,
        strict_quality_equivalent=False, training_performed=False, runtime_changes=False,
        pilot_usable=True, all_16_classes_covered=all(
            any(a.get('training_class') == name for r in accepted for a in r['annotations'])
            for name in training_names(policy)), **stats)
    save(args.output/'breadth-summary.json', summary)
    # The common exporter verifies file/ID/geometry integrity, not semantic accuracy.
    saved = json.loads((args.export/'summary.json').read_text())
    saved.update(admission_policy=config['admission_policy'],
                 quality_tiers=summary['quality_tiers'],
                 complete_target_annotation_asserted=False,
                 strict_quality_equivalent=False)
    save(args.export/'summary.json', saved)
    args.gallery.mkdir(parents=True, exist_ok=False)
    save(args.gallery/'breadth-summary.json', summary)
    table = ''.join('<tr><td>'+html.escape(k)+'</td><td>'+str(v)+'</td></tr>'
                    for k,v in sorted(summary['objects_by_class'].items()))
    page = ('<!doctype html><html lang="ko"><meta charset="utf-8">'
        '<title>대량 활용 bbox v5 — 2026-10-11</title>'
        '<style>body{font:16px system-ui;max-width:1100px;margin:auto;padding:24px;line-height:1.7}'
        'td,th{padding:8px;border:1px solid #ccc}table{border-collapse:collapse}pre{white-space:pre-wrap}</style>'
        '<h1>대량 활용 bbox v5 — 2026-10-11</h1><a href="../index.html">전체 목차</a>'
        '<p>사용자 결정에 따라 차량 세부 클래스 혼입과 완전성 미확인을 허용하는 train 확장입니다. '
        '이전 엄격 검수 자료와 평가 split·라벨을 유지합니다. teacher 예측이나 파일명으로 혼합 '
        '종류를 자동 확정하지 않았습니다. 미주석 객체가 없는 완전한16종 정답이라고 주장하지 않습니다.</p>'
        '<p>신호등 등 제외 라벨은 YOLO txt에 넣지 않습니다. 빈 주석·제외 라벨만 있는 미검수 영상을 '
        'negative로 추가하지 않습니다. 미해석 혼합 박스와 분할·중복·좌표 오류는 분리합니다.</p>'
        '<h2>클래스별 전체 객체</h2><table>'+table+'</table><h2>수량·품질 상태</h2><pre>'
        +html.escape(json.dumps(summary, ensure_ascii=False, indent=2))+'</pre></html>')
    (args.gallery/'index.html').write_text(page)
    print(json.dumps(summary, ensure_ascii=False))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ('previous','reviews','labels','config','output','export','gallery'):
        parser.add_argument('--'+key, required=True, type=Path)
    run(parser.parse_args())
