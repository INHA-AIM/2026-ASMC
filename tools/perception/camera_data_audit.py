#!/usr/bin/env python3
"""Lossless dataset inventory, explicit review gates and reproducible bbox export.

Raw assets never change. Unreviewed external labels, incomplete images, unknown
subtypes and ambiguous composites are quarantined rather than assumed negative.
"""
import argparse
from collections import Counter, defaultdict
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import math
import os
from pathlib import Path
import random
import re

from PIL import Image, ImageDraw


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def digest(text):
    return hashlib.sha256(text.encode()).hexdigest()


def save(path, value):
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')


def read_jsonl(path):
    with Path(path).open() as f:
        for line in f:
            if line.strip():
                yield json.loads(line)


def write_jsonl(path, rows):
    with Path(path).open('w') as f:
        for row in rows:
            f.write(json.dumps(row, ensure_ascii=False) + '\n')


def normalize(name):
    return re.sub(r'\s+', ' ', str(name).strip().lower().replace('_', ' ').replace('-', ' '))


def classify(name, policy):
    key = normalize(name)
    target = policy['aliases'].get(key)
    if target:
        return target, 'candidate'
    if key in policy['excluded'] or re.fullmatch(r'[34]? ?traffic light.*', key) or key.endswith(' light'):
        return None, 'excluded'
    return None, 'ambiguous'


def phash(image):
    """64-bit pHash is only a review signal, never deletion evidence."""
    import numpy as np
    from scipy.fftpack import dct
    pixels = np.asarray(image.convert('L').resize((32, 32), getattr(Image, 'Resampling', Image).LANCZOS), dtype=float)
    low = dct(dct(pixels, axis=0), axis=1)[:8, :8]
    return '%016x' % int(''.join('1' if x else '0' for x in (low > np.median(low)).flatten()), 2)


def valid_box(box, width, height):
    return (len(box) == 4 and all(isinstance(v, (int, float)) and math.isfinite(v) for v in box)
            and 0 <= box[0] < box[2] <= width and 0 <= box[1] < box[3] <= height)


def polygon_issues(segmentation, width, height):
    if not segmentation:
        return []
    if isinstance(segmentation, dict):
        size = segmentation.get('size')
        counts = segmentation.get('counts')
        if size != [height, width] or not isinstance(counts, (str, list)):
            return ['invalid_rle']
        if isinstance(counts, list) and (any(not isinstance(v, int) or v < 0 for v in counts) or sum(counts) != width * height):
            return ['invalid_rle']
        # Compressed RLE remains native, not promoted to verified masks here.
        return ['compressed_rle_needs_decode'] if isinstance(counts, str) else []
    if not isinstance(segmentation, list):
        return ['invalid_polygon']
    for polygon in segmentation:
        if not isinstance(polygon, list) or len(polygon) < 6 or len(polygon) % 2:
            return ['invalid_polygon']
        if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in polygon):
            return ['invalid_polygon']
        if any(not 0 <= v <= (width if i % 2 == 0 else height) for i, v in enumerate(polygon)):
            return ['polygon_out_of_bounds']
    return []


def base_row(source, key, image, annotations, issues, frozen_split=None, view='unknown'):
    return dict(id=digest(source + ':' + key), source=source, source_key=key,
        image=str(Path(image).resolve()), annotations=annotations, issues=issues,
        frozen_split=frozen_split, split_group=source, view=view, weather='unknown',
        tunnel='unknown', annotation_complete=False, provenance_verified=False)


def import_coco(export, project, version, policy):
    rows = []
    for annotation_path in sorted(Path(export).rglob('*.json')):
        try:
            data = json.loads(annotation_path.read_text())
        except (ValueError, UnicodeError):
            continue
        if not isinstance(data, dict) or not all(k in data for k in ('images', 'annotations', 'categories')):
            continue
        categories = {c['id']: c['name'] for c in data['categories']}
        duplicate_categories = len(categories) != len(data['categories'])
        by_image = defaultdict(list)
        ids = {}
        duplicate_annotations = set()
        image_ids = {i['id'] for i in data['images']}
        orphan_annotations = False
        for ann in data['annotations']:
            if ann.get('id') in ids:
                duplicate_annotations.add(ann['image_id'])
                duplicate_annotations.add(ids[ann.get('id')])
            ids[ann.get('id')] = ann['image_id']
            by_image[ann['image_id']].append(ann)
            orphan_annotations |= ann['image_id'] not in image_ids
        image_counts = Counter(i['id'] for i in data['images'])
        for image in data['images']:
            issues = []
            if image_counts[image['id']] > 1:
                issues.append('duplicate_image_id')
            if duplicate_categories:
                issues.append('duplicate_category_id')
            if orphan_annotations:
                issues.append('orphan_annotation_in_source')
            if image['id'] in duplicate_annotations:
                issues.append('duplicate_annotation_id')
            path = (annotation_path.parent / image['file_name']).resolve()
            try:
                path.relative_to(Path(export).resolve())
            except ValueError:
                issues.append('unsafe_image_path')
            width, height = image.get('width', 0), image.get('height', 0)
            annotations = []
            for ann in by_image[image['id']]:
                name = categories.get(ann['category_id'])
                if name is None:
                    issues.append('unknown_category_id')
                    name = '__unknown_category__'
                target, disposition = classify(name, policy)
                box = ann.get('bbox', [])
                xyxy = [box[0], box[1], box[0] + box[2], box[1] + box[3]] if len(box) == 4 else []
                item_issues = [] if valid_box(xyxy, width, height) else ['invalid_bbox']
                item_issues += polygon_issues(ann.get('segmentation'), width, height)
                if ann.get('iscrowd'):
                    item_issues.append('crowd_not_individual')
                annotations.append(dict(id=str(ann.get('id')), original_label=name,
                    original_category_id=ann['category_id'], canonical=target, disposition=disposition,
                    xyxy=xyxy, native_segmentation=ann.get('segmentation'), issues=item_issues))
            source = 'roboflow:' + project
            key = str(version) + '/' + str(annotation_path.relative_to(export)) + ':' + str(image['id'])
            row = base_row(source, key, path, annotations, issues)
            row.update(width=width, height=height, version=version,
                       annotation_path=str(annotation_path), original_split=annotation_path.parent.name)
            # Preserve filename lineage without assuming adjacent images independent.
            stem = re.split(r'\.rf\.', Path(image['file_name']).stem)[0]
            row['augmentation_family'] = stem
            rows.append(row)
    return rows


def import_capture(manifest, datasets, policy):
    rows = []
    for item in read_jsonl(manifest):
        source = 'capture:' + item['run']
        key = item['view'] + '/' + item['frame']
        annotations = []
        for index, ann in enumerate(item['annotations']):
            name = ann['source_class_name']
            target, disposition = classify(name, policy)
            issues = []
            if not ann.get('subtype_verified'):
                issues.append('capture_subtype_unknown')
            mask = item.get('masks')
            annotations.append(dict(id=str(index), original_label=name,
                canonical=target, disposition=disposition, xyxy=ann['xyxy'],
                actor_id=ann.get('actor_id'), native_mask=mask,
                native_mask_root=str(Path(manifest).parent.resolve()), mask_index=index, issues=issues))
        issues = [i['reason'] for i in item.get('issues', [])]
        if not item.get('bbox_file_present'):
            issues.append('missing_capture_bbox_source')
        row = base_row(source, key, Path(datasets) / item['image'], annotations,
                       issues, item['split'], item['view'])
        row.update(annotation_complete=item.get('annotation_complete', False),
                   provenance_verified=True, capture_manifest_sha256=sha(manifest) if not rows else None,
                   original_annotation=item, expected_image_sha256=item.get('source_sha256',{}).get('intensity'))
        rows.append(row)
    return rows


def image_facts(row):
    row = dict(row)
    if 'unsafe_image_path' in row['issues']:
        return row
    try:
        path = Path(row['image'])
        row['image_sha256'] = sha(path)
        if row.get('expected_image_sha256') and row['image_sha256'] != row['expected_image_sha256']:
            row['issues'].append('source_image_changed_since_gt')
        with Image.open(path) as im:
            im.load()
            rgb = im.convert('RGB')
            width, height = rgb.size
            if row.get('width') is not None and (row['width'], row['height']) != (width, height):
                row['issues'].append('image_dimensions_disagree')
            row.update(width=width, height=height,
                pixel_sha256=hashlib.sha256(f'{width}x{height}:'.encode() + rgb.tobytes()).hexdigest())
            row['phash'] = phash(rgb)
        for ann in row['annotations']:
            if not valid_box(ann['xyxy'], width, height) and 'invalid_bbox' not in ann['issues']:
                ann['issues'].append('invalid_bbox')
    except (OSError, ValueError, Image.DecompressionBombError):
        row['issues'].append('missing_or_broken_image')
    return row


def review_samples(rows, policy):
    buckets = defaultdict(list)
    for row in rows:
        if row['source'].startswith('capture:'):
            continue
        for name in {a['original_label'] for a in row['annotations']} or {'__empty__'}:
            buckets[row['source'] + '|' + name].append(row)
    samples = {}
    for bucket, candidates in sorted(buckets.items()):
        candidates = sorted(candidates, key=lambda row: row['id'])
        random.Random(str(policy['review']['seed']) + bucket).shuffle(candidates)
        samples[bucket] = [r['id'] for r in candidates[:policy['review']['samples_per_source_class']]]
    return samples


def apply_review(row, decision, policy):
    row = dict(row)
    row['annotations'] = [dict(ann) for ann in row['annotations']]
    if decision is None:
        return row, False
    if decision.get('image_sha256') != row.get('image_sha256'):
        raise ValueError('review image hash mismatch: ' + row['id'])
    if decision.get('decision') != 'accept':
        row['issues'] = row['issues'] + ['review_rejected']
        return row, False
    if decision.get('bbox_policy') != policy['bbox_policy'] or not decision.get('all_targets_annotated'):
        return row, False
    canonical_names = {n for names in policy['groups'].values() for n in names}
    overrides = {str(a['id']): a for a in decision.get('overrides', [])}
    known_ids = {a['id'] for a in row['annotations']}
    if set(overrides) - known_ids:
        raise ValueError('unknown annotation override')
    for ann in row['annotations']:
        override = overrides.get(ann['id'])
        if override:
            target = override.get('canonical')
            if target not in canonical_names and target is not None:
                raise ValueError('unknown canonical class')
            ann.update(canonical=target, disposition='candidate' if target else 'excluded')
            if 'xyxy' in override:
                if not valid_box(override['xyxy'], row['width'], row['height']):
                    raise ValueError('invalid reviewed bbox')
                ann['xyxy'] = override['xyxy']
            # Geometry errors cannot be waived by a class override.
            if 'capture_subtype_unknown' in ann['issues']:
                ann['issues'] = [i for i in ann['issues'] if i != 'capture_subtype_unknown']
    row['annotation_complete'] = True
    return row, True


class UnionFind:
    def __init__(self, ids):
        self.parent = {i: i for i in ids}

    def find(self, value):
        root = value
        while self.parent[root] != root:
            root = self.parent[root]
        while value != root:
            value, self.parent[value] = self.parent[value], root
        return root

    def union(self, a, b):
        ra, rb = self.find(a), self.find(b)
        self.parent[max(ra, rb)] = min(ra, rb)


def assign_splits(rows, policy):
    """Groups include all rows, even quarantined ones, to protect frozen splits."""
    groups = UnionFind({r['split_group'] for r in rows})
    seen = {}
    for row in rows:
        for kind in ('image_sha256', 'pixel_sha256'):
            value = row.get(kind)
            if value:
                key = kind + ':' + value
                if key in seen:
                    groups.union(row['split_group'], seen[key])
                else:
                    seen[key] = row['split_group']
    frozen = defaultdict(set)
    for row in rows:
        if row.get('frozen_split'):
            frozen[groups.find(row['split_group'])].add(row['frozen_split'])
    roles = {}
    for group in groups.parent:
        root = groups.find(group)
        existing = frozen[root]
        if len(existing) == 1:
            roles[root] = next(iter(existing))
        elif len(existing) > 1:
            roles[root] = 'quarantine_split_conflict'
        else:
            value = int(digest(str(policy['split']['seed']) + root)[:16], 16) / 2 ** 64
            a, b, _ = policy['split']['ratios']
            roles[root] = 'train' if value < a else ('selection' if value < a+b else 'heldout')
    for row in rows:
        root = groups.find(row['split_group'])
        role = roles[root]
        row['split_cluster'] = root
        # Preserve frozen assignment; exclude conflicts rather than moving runs.
        row['split'] = row.get('frozen_split') or role
        if role == 'quarantine_split_conflict':
            row['eligible'] = False
            row['reasons'].append('frozen_split_overlap')


def annotation_signature(row):
    return sorted((a['canonical'], tuple(a['xyxy'])) for a in row['annotations'] if a['canonical'])


def near_duplicate_candidates(rows, distance=6):
    """Banded pHash search. Cross-split candidates wait for visual review.

    Seven disjoint bands guarantee a shared band for <=6 changed bits. Repeated
    hashes are compacted; candidate-search truncation explicitly quarantines.
    """
    bands = defaultdict(set)
    representatives = defaultdict(list)
    pairs = []
    truncated = False
    for row in rows:
        value = row.get('phash')
        if not value:
            continue
        number = int(value, 16)
        keys = [(index, (number >> (index * 9)) & (1023 if index == 6 else 511)) for index in range(7)]
        candidates = set().union(*(bands[k] for k in keys))
        if len(candidates) > 5000:
            row['eligible'] = False
            row['reasons'].append('near_duplicate_search_requires_review')
            truncated = True
            continue
        for previous_hash in sorted(candidates):
            difference = bin(number ^ int(previous_hash, 16)).count('1')
            if difference > distance:
                continue
            for previous in representatives[previous_hash]:
                if row['split_group'] == previous['split_group'] or row.get('pixel_sha256') == previous.get('pixel_sha256'):
                    continue
                ratio = (row['width'] / row['height']) / (previous['width'] / previous['height'])
                if not .98 <= ratio <= 1.02:
                    continue
                pair = dict(a=previous['id'], b=row['id'], distance=difference,
                            confirmed=False, same_split=row['split'] == previous['split'])
                pairs.append(pair)
                if not pair['same_split']:
                    for item in (row, previous):
                        item['eligible'] = False
                        if 'cross_split_similar_image_review' not in item['reasons']:
                            item['reasons'].append('cross_split_similar_image_review')
        for key in keys:
            bands[key].add(value)
        if not any(r['split_group'] == row['split_group'] for r in representatives[value]):
            representatives[value].append(row)
    blocked = {r['phash'] for r in rows if 'cross_split_similar_image_review' in r['reasons']}
    for row in rows:
        if row.get('phash') in blocked:
            row['eligible'] = False
            if 'cross_split_similar_image_review' not in row['reasons']:
                row['reasons'].append('cross_split_similar_image_review')
        if truncated:
            row['eligible'] = False
            row['reasons'].append('near_duplicate_search_incomplete')
    return pairs


def curate(rows, policy, reviews=None):
    reviews = reviews or {}
    samples = review_samples(rows, policy)
    # One failed sample quarantines its source/class bucket until full review.
    failed_buckets = {bucket for bucket, ids in samples.items()
                      if any(reviews.get(i, {}).get('decision') == 'reject' for i in ids)}
    approved_buckets = {bucket for bucket, ids in samples.items() if ids and all(
        reviews.get(i, {}).get('decision') == 'accept'
        and reviews[i].get('all_targets_annotated')
        and reviews[i].get('bbox_policy') == policy['bbox_policy']
        and not reviews[i].get('overrides') for i in ids)} - failed_buckets
    result = []
    for original in rows:
        row, reviewed = apply_review(original, reviews.get(original['id']), policy)
        buckets = {row['source'] + '|' + a['original_label'] for a in row['annotations']} or {row['source'] + '|__empty__'}
        capture_verified = row['provenance_verified'] and row['annotation_complete']
        bucket_verified = bool(buckets) and buckets <= approved_buckets
        reasons = list(row['issues'])
        for ann in row['annotations']:
            reasons += ann['issues']
            if ann['disposition'] == 'ambiguous':
                reasons.append('ambiguous_label:' + ann['original_label'])
        if not (reviewed or capture_verified or bucket_verified):
            reasons.append('visual_review_required')
        if buckets & failed_buckets and not reviewed:
            reasons.append('source_class_sample_failed')
        if not (row['annotation_complete'] or bucket_verified):
            reasons.append('annotation_completeness_unverified')
        row.update(eligible=not reasons, reasons=sorted(set(reasons)),
                   review_basis='individual' if reviewed else ('capture_gt' if capture_verified else ('source_class_sample' if bucket_verified else 'unreviewed')))
        result.append(row)
    assign_splits(result, policy)
    # Equal pixels with different raw boxes/classes must not be auto-selected.
    duplicates = defaultdict(list)
    for row in result:
        if row.get('pixel_sha256'):
            duplicates[row['pixel_sha256']].append(row)
    for cluster in duplicates.values():
        eligible = [r for r in cluster if r['eligible']]
        signatures = {json.dumps(annotation_signature(r)) for r in cluster}
        if len(signatures) > 1:
            for row in eligible:
                row['eligible'] = False
                row['reasons'].append('duplicate_annotation_conflict')
        elif len(eligible) > 1:
            for row in sorted(eligible, key=lambda r: r['id'])[1:]:
                row['eligible'] = False
                row['reasons'].append('exact_duplicate_representative_preserved')
    return result, samples


def inventory(args):
    policy = json.loads(args.labels.read_text())
    report = json.loads((args.root / 'download-report.json').read_text())
    rows, unsupported = [], []
    for project in report['projects']:
        if project['status'] != 'downloaded':
            continue
        incoming = import_coco(Path(project['raw_path']) / 'export', project['project'], project['version'], policy)
        if not incoming:
            unsupported.append(dict(project=project['project'], reason='no_supported_instance_annotations'))
        rows.extend(incoming)
    if args.capture_manifest:
        rows.extend(import_capture(args.capture_manifest, args.capture_datasets, policy))
    ids = [r['id'] for r in rows]
    if len(ids) != len(set(ids)):
        raise ValueError('duplicate inventory IDs')
    args.output.mkdir(parents=True, exist_ok=False)
    with ThreadPoolExecutor(max_workers=args.workers) as pool:
        with (args.output / 'inventory.jsonl').open('w') as f:
            for count, row in enumerate(pool.map(image_facts, rows), 1):
                f.write(json.dumps(row, ensure_ascii=False) + '\n')
                if count % 1000 == 0:
                    print('inventoried', count, flush=True)
    save(args.output / 'inventory-summary.json', dict(images=len(rows),
        unsupported=unsupported, source_report_sha256=sha(args.root / 'download-report.json'),
        labels_sha256=sha(args.labels), capture_manifest_sha256=sha(args.capture_manifest) if args.capture_manifest else None))


def build_gallery(rows, samples, output):
    ids = {i for group in samples.values() for i in group}
    ids |= {r['id'] for r in rows if any(a['disposition'] == 'ambiguous' for a in r['annotations'])}
    # Every issue remains in JSONL; paginated galleries avoid silently truncating review.
    chosen = [r for r in rows if r['id'] in ids or r['issues']]
    output.mkdir(parents=True, exist_ok=True)
    entries = []
    for row in chosen:
        if 'missing_or_broken_image' in row['issues']:
            continue
        image_path = output / (row['id'] + '.jpg')
        if not image_path.exists():
            with Image.open(row['image']) as im:
                im = im.convert('RGB')
                draw = ImageDraw.Draw(im)
                for ann in row['annotations']:
                    if valid_box(ann['xyxy'], *im.size):
                        color = 'lime' if ann['canonical'] else 'orange'
                        draw.rectangle(ann['xyxy'], outline=color, width=3)
                        draw.text((ann['xyxy'][0], max(0, ann['xyxy'][1]-14)), ann['original_label'] + ' -> ' + str(ann['canonical']), fill=color)
                im.thumbnail((768, 576))
                im.save(image_path, quality=85)
        entries.append(dict(id=row['id'], image_sha256=row.get('image_sha256'), source=row['source'],
            annotations=[{k:a.get(k) for k in ('id','original_label','canonical','xyxy','issues')} for a in row['annotations']],
            issues=row['issues'], image=image_path.name))
    # Only JSON download; no web server writes or silent approval.
    data = json.dumps(entries, ensure_ascii=False).replace('<', '\\u003c')
    template = '''<!doctype html><meta charset="utf-8"><title>Roboflow 데이터 감사 · 검수 대기</title>
<style>body{font:16px system-ui;max-width:1200px;margin:auto}img{max-width:100%}article{border:1px solid #aaa;padding:12px;margin:15px 0}button{margin:5px}pre{white-space:pre-wrap}</style>
<h1>Roboflow 데이터 감사 · 검수 대기</h1><p>승인: 보이는 모든 대상에 주석이 있고 박스가 가시 영역에 맞는지 확인. 불명확하면 거부/보류. 주황색은 제외 또는 미확정 라벨입니다. 사람이 검수하기 전에는 학습에서 격리됩니다.</p>
<input id="q" placeholder="클래스·프로젝트 검색"><button id="prev">이전</button><button id="next">다음</button><button id="export">검수 JSON 다운로드</button><p id="count"></p><div id="rows"></div>
<script>const rows=DATA;let page=0,decisions={};const q=document.querySelector('#q');
function draw(){const filtered=rows.filter(r=>JSON.stringify(r).toLowerCase().includes(q.value.toLowerCase()));const box=document.querySelector('#rows');box.replaceChildren();document.querySelector('#count').textContent=filtered.length+'개 / 페이지 '+(page+1);
for(const r of filtered.slice(page*30,(page+1)*30)){const a=document.createElement('article'),p=document.createElement('pre'),img=document.createElement('img');p.textContent=r.source+'\n'+r.id+'\n'+JSON.stringify(r.annotations,null,2)+'\n'+r.issues.join(',');img.src=r.image;img.loading='lazy';a.append(p,img);
for(const [label,value] of [['완전 주석 승인','accept'],['거부·격리','reject']]){const b=document.createElement('button');b.textContent=label;b.onclick=()=>{decisions[r.id]={image_sha256:r.image_sha256,decision:value,all_targets_annotated:value==='accept',bbox_policy:'visible_extent_xyxy_exclusive',reviewer:'local-browser-review'};b.textContent=label+' ✓';};a.append(b);}box.append(a);}}
q.oninput=()=>{page=0;draw()};document.querySelector('#prev').onclick=()=>{page=Math.max(0,page-1);draw()};document.querySelector('#next').onclick=()=>{page++;draw()};document.querySelector('#export').onclick=()=>{const a=document.createElement('a');a.href=URL.createObjectURL(new Blob([JSON.stringify(decisions,null,2)],{type:'application/json'}));a.download='reviews.json';a.click();URL.revokeObjectURL(a.href);};draw();</script>'''
    (output / 'index.html').write_text(template.replace('DATA', data))
    save(output / 'review-samples.json', samples)


def export_dataset(rows, policy, output):
    output.mkdir(parents=True, exist_ok=False)
    accepted = [r for r in rows if r['eligible']]
    names = [name for group in policy['groups'].values() for name in group
             if any(a['canonical'] == name for r in accepted for a in r['annotations'])]
    ids = {name:index for index,name in enumerate(names)}
    by_role = Counter()
    for role in ('train','val','test'):
        (output / 'images' / role).mkdir(parents=True, exist_ok=True)
        (output / 'labels' / role).mkdir(parents=True, exist_ok=True)
    for row in accepted:
        role = {'selection':'val', 'heldout':'test', 'train':'train'}[row['split']]
        folder = output / 'images' / role
        labels = output / 'labels' / role
        folder.mkdir(parents=True, exist_ok=True)
        labels.mkdir(parents=True, exist_ok=True)
        image = Path(row['image'])
        target = folder / (row['id'] + image.suffix.lower())
        os.symlink(image.resolve(), target)
        lines = []
        for ann in row['annotations']:
            if ann['canonical'] is None:
                continue
            x1,y1,x2,y2 = ann['xyxy'];w,h=row['width'],row['height']
            lines.append('%d %.10f %.10f %.10f %.10f' % (ids[ann['canonical']],(x1+x2)/2/w,(y1+y2)/2/h,(x2-x1)/w,(y2-y1)/h))
        (labels / (row['id'] + '.txt')).write_text('\n'.join(lines) + ('\n' if lines else ''))
        by_role[row['split']] += 1
    import yaml
    # Absolute root intentionally describes a local, provenance-linked artifact.
    (output / 'data.yaml').write_text(yaml.safe_dump(dict(path=str(output.resolve()),train='images/train',val='images/val',test='images/test',names=names),sort_keys=False))
    write_jsonl(output / 'manifest.jsonl', accepted)
    hashes = defaultdict(set)
    for row in accepted:
        hashes[row['pixel_sha256']].add(row['split'])
    leaks = [h for h, roles in hashes.items() if len(roles) > 1]
    if leaks:
        raise ValueError('cross-split pixel leakage')
    save(output / 'summary.json', dict(images=len(accepted), classes=names, splits=dict(by_role),
        exact_pixel_leaks=len(leaks), manifest_sha256=sha(output / 'manifest.jsonl'),
        labels_sha256=digest(json.dumps(policy, sort_keys=True)),
        training_performed=False, unknown_subtypes_exported=0,
        split_ready=all(by_role[role] > 0 for role in ('train','selection','heldout')),
        missing_target_classes=[n for group in policy['groups'].values() for n in group if n not in names],
        full_target_coverage=all(n in names for group in policy['groups'].values() for n in group)))


def curate_command(args):
    policy = json.loads(args.labels.read_text())
    rows = list(read_jsonl(args.inventory))
    reviews = json.loads(args.reviews.read_text()) if args.reviews else {}
    rows, samples = curate(rows, policy, reviews)
    near = near_duplicate_candidates(rows)
    args.output.mkdir(parents=True, exist_ok=False)
    write_jsonl(args.output / 'curated.jsonl', rows)
    write_jsonl(args.output / 'near-duplicate-candidates.jsonl', near)
    reason_counts = Counter(reason for r in rows for reason in r['reasons'])
    source_counts = {}
    for source in sorted({r['source'] for r in rows}):
        subset = [r for r in rows if r['source'] == source]
        source_counts[source] = dict(images=len(subset), accepted=sum(r['eligible'] for r in subset),
            original_labels=dict(Counter(a['original_label'] for r in subset for a in r['annotations'])),
            accepted_classes=dict(Counter(a['canonical'] for r in subset if r['eligible'] for a in r['annotations'] if a['canonical'])))
    save(args.output / 'audit-summary.json', dict(images=len(rows),accepted=sum(r['eligible'] for r in rows),
        quarantined=sum(not r['eligible'] for r in rows), reasons=dict(reason_counts), sources=source_counts,
        inventory_sha256=sha(args.inventory), reviews_sha256=sha(args.reviews) if args.reviews else None,
        review_samples=sum(map(len, samples.values())), near_duplicate_pairs=len(near),
        conditions='Unverified weather/tunnel/view remain unknown'))
    if args.gallery:
        build_gallery(rows, samples, args.gallery)
    if args.export:
        export_dataset(rows, policy, args.export)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    sub = p.add_subparsers(dest='command', required=True)
    inv = sub.add_parser('inventory')
    inv.add_argument('--root', type=Path, required=True)
    inv.add_argument('--labels', type=Path, required=True)
    inv.add_argument('--capture-manifest', type=Path)
    inv.add_argument('--capture-datasets', type=Path)
    inv.add_argument('--workers', type=int, default=2)
    inv.add_argument('--output', type=Path, required=True)
    cur = sub.add_parser('curate')
    cur.add_argument('--inventory', type=Path, required=True)
    cur.add_argument('--labels', type=Path, required=True)
    cur.add_argument('--reviews', type=Path)
    cur.add_argument('--output', type=Path, required=True)
    cur.add_argument('--gallery', type=Path)
    cur.add_argument('--export', type=Path)
    args = p.parse_args()
    if args.command == 'inventory':
        if args.capture_manifest and not args.capture_datasets:
            p.error('--capture-datasets is required with Capture')
        inventory(args)
    else:
        curate_command(args)


if __name__ == '__main__':
    main()
