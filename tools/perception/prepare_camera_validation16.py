#!/usr/bin/env python3
"""Export an additive, reviewed validation set; never mutate training or old eval.

Input records must contain an explicit image-wide review, not just a teacher
confidence. All train rows are denied, including images not reached this epoch.
"""
import argparse
from collections import Counter
import hashlib
import html
import json
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw
import yaml

from camera_pretrain_policy import CLASSES, load_policy, sha, view_masks
from prepare_camera_pretrain import normalize_row, read_rows, yolo_lines


def pixel_hash(path):
    im = Image.open(path).convert('RGB')
    return hashlib.sha256(f'{im.width}x{im.height}:'.encode()+im.tobytes()).hexdigest()


def deny_index(rows):
    fields = ('id', 'image_sha256', 'pixel_sha256', 'source_pixel_sha256',
              'source', 'split_group', 'correlation_group')
    return {k: {r[k] for r in rows if r.get(k)} for k in fields}


def check_leak(row, deny):
    hits = [k for k, values in deny.items() if row.get(k) in values]
    if row.get('pixel_sha256') in deny.get('source_pixel_sha256', set()):
        hits.append('source_pixel_sha256')
    if hits:
        raise ValueError('training overlap '+row['id']+': '+','.join(hits))


def check_review(row):
    review = row.get('review_record', {})
    if not (review.get('decision') == 'accept' and review.get('all_targets_annotated') is True
            and review.get('review_scope') == 'camera_objects_v2_16'
            and review.get('evidence') and review.get('notes')):
        raise ValueError('explicit full16 RGB review required')
    if not Path(review['evidence']).is_file():
        raise ValueError('missing review evidence')


def build(review_file, train_file, output, gallery):
    if output.exists():
        raise ValueError('immutable export exists; use a new revision')
    train = [r for r in read_rows(train_file) if r['split'] == 'train']
    deny = deny_index(train)
    policy = load_policy()
    masks = view_masks(policy)
    cache = {'parent_shas': {str(review_file.parent): sha(review_file)}}
    rows = []
    seen = set()
    for old in read_rows(review_file):
        check_review(old)
        if sha(old['image']) != old['image_sha256']:
            raise ValueError('review/source SHA mismatch')
        old['pixel_sha256'] = pixel_hash(old['image'])
        check_leak(old, deny)
        if old['pixel_sha256'] in seen:
            raise ValueError('duplicate validation pixels')
        seen.add(old['pixel_sha256'])
        im = cv2.imread(old['image'])
        if im is None:
            raise ValueError('unreadable image')
        row = normalize_row(review_file.parent, old, im, policy, masks, cache)
        row.update(split='selection', frozen_source_split=old.get('original_split'),
                   evaluation_role='reviewed_16class_development_validation',
                   scene_group=old.get('scene_group', old['source']),
                   candidate_selection=old.get('candidate_selection', 'native or source GT; RGB reviewed'))
        row['review_evidence_sha256'] = sha(old['review_record']['evidence'])
        if row['supervision'] != 'complete':
            raise ValueError('partial image cannot enter complete validation core')
        check_leak(row, deny)
        rows.append(row)
    for folder in ['images/val', 'labels/val', 'supervision/val']:
        (output/folder).mkdir(parents=True)
    gallery.mkdir(parents=True, exist_ok=True)
    cards = []
    for i, r in enumerate(rows):
        link = Path('images/val')/(r['id']+Path(r['image']).suffix)
        (output/link).symlink_to(Path(r['image']).resolve())
        label = output/'labels/val'/(r['id']+'.txt')
        label.write_text('\n'.join(yolo_lines(r))+'\n')
        r.update(export_image=str(link), export_label_sha256=sha(label))
        side = output/'supervision/val'/(r['id']+'.json')
        side.write_text(json.dumps({'annotations':r['annotations'], 'supervision':'complete'}))
        # Re-read actual serialized labels; numeric IDs and coordinates checked.
        if label.read_text().splitlines() != yolo_lines(r):
            raise ValueError('YOLO roundtrip mismatch')
        if json.loads(side.read_text())['annotations'] != r['annotations']:
            raise ValueError('sidecar roundtrip mismatch')
        im = Image.open(r['image']).convert('RGB')
        if r['view'] in masks:
            arr = np.asarray(im).copy()
            arr[masks[r['view']].geometry(im.height, im.width)[0]] = 0
            im = Image.fromarray(arr)
        d = ImageDraw.Draw(im)
        for a in r['annotations']:
            if a['supervision'] == 'excluded': continue
            color = '#ff00ff' if a['supervision']=='ignore' else '#00ff60'
            d.rectangle(a['xyxy'], outline=color, width=2)
            d.text(tuple(a['xyxy'][:2]), a['training_class'], fill=color)
        im.thumbnail((1280, 800))
        name = f'{i:04d}.jpg'; im.save(gallery/name, quality=93)
        names = sorted({a['training_class'] for a in r['annotations'] if a['supervision']=='positive'})
        cards.append('<figure data-classes="'+','.join(names)+'"><figcaption>'+html.escape(
            f'{i:03d} | {", ".join(names)} | {r["weather"]} | {r["source"]}')+
            f'</figcaption><a href="{name}"><img loading="lazy" src="{name}"></a></figure>')
    (output/'manifest.jsonl').write_text(''.join(json.dumps(r,ensure_ascii=False)+'\n' for r in rows))
    counts = {}
    for c in CLASSES:
        rr = [r for r in rows if any(a['training_class']==c and a['supervision']=='positive' for a in r['annotations'])]
        counts[c] = dict(images=len(rr), boxes=sum(a['training_class']==c and a['supervision']=='positive' for r in rr for a in r['annotations']),
            sources=dict(Counter(r['source'] for r in rr)), scenes=len({r['scene_group'] for r in rr}),
            weather=dict(Counter(r['weather'] for r in rr)), views=dict(Counter(r['view'] for r in rr)))
    summary = dict(version=output.name, images=len(rows), classes=counts,
        positives=sum(a['supervision']=='positive' for r in rows for a in r['annotations']),
        ignores=sum(a['supervision']=='ignore' for r in rows for a in r['annotations']),
        weather=dict(Counter(r['weather'] for r in rows)), sources=dict(Counter(r['source'].split(':')[0] for r in rows)),
        train_images_denied=len(train), train_manifest_sha256=sha(train_file), review_manifest_sha256=sha(review_file),
        manifest_sha256=sha(output/'manifest.jsonl'), all16_present=all(counts[c]['images'] for c in CLASSES),
        full_coverage_sufficient=False,
        limitations=['Development validation, not an unseen heldout test.',
            'Image count does not imply independent scene count; per-class source/weather shortages remain.',
            'AIM dog/deer/bicycle discovery used old teacher proposals; not an unbiased recall benchmark.',
            'Exact/pixel/source/run/group checks do not prove absence of every unknown cross-project scene copy.'])
    (output/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
    (output/'data.yaml').write_text(yaml.safe_dump(dict(path=str(output.resolve()),
        train='_EVALUATION_ONLY_DO_NOT_TRAIN', val='images/val', names=list(CLASSES),
        requires_ignore_aware_evaluator=True),sort_keys=False))
    (output/'verification.json').write_text(json.dumps(dict(passed=True, exact_train_overlap=0,
        pixel_train_overlap=0, source_run_group_overlap=0, all_rows_complete=True,
        serialized_labels_verified=True, manifest_sha256=summary['manifest_sha256']),indent=2)+'\n')
    (output/'reservation.json').write_text(json.dumps(dict(purpose='deny future training inclusion',
        sources=sorted({r['source'] for r in rows}), split_groups=sorted({r['split_group'] for r in rows}),
        image_hashes=sorted({r['image_sha256'] for r in rows}),
        pixel_hashes=sorted({r['pixel_sha256'] for r in rows})),indent=2)+'\n')
    table=''.join(f'<tr><td>{c}</td><td>{s["images"]}</td><td>{s["boxes"]}</td><td>{s["scenes"]}</td></tr>' for c,s in counts.items())
    (gallery/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>2026-10-13 · 16-class validation review</title>'
        '<style>body{background:#161a20;color:#eee;font:16px sans-serif;padding:20px}img{max-width:100%}figure{margin:12px 0}td{padding:5px 20px}select{font-size:18px}</style>'
        '<h1>2026-10-13 · 16종 검수 평가본</h1><p>기존 618장 회귀 진단은 보존. 녹색=양성, 자홍색=ignore. 독립 장면·날씨 부족과 teacher 발견 편향을 함께 보고합니다.</p>'
        '<table><tr><th>Class</th><th>Images</th><th>Boxes</th><th>Source/run groups</th></tr>'+table+'</table>'
        '<select onchange="document.querySelectorAll(\'figure\').forEach(e=>e.hidden=this.value!==\'all\'&&!e.dataset.classes.split(\',\').includes(this.value))"><option>all</option>'+
        ''.join('<option>'+c+'</option>' for c in CLASSES)+'</select>'+''.join(cards))
    print(json.dumps(summary,ensure_ascii=False,indent=2))


if __name__ == '__main__':
    p=argparse.ArgumentParser();p.add_argument('--reviews',type=Path,required=True)
    p.add_argument('--train-manifest',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--gallery',type=Path,required=True);a=p.parse_args()
    build(a.reviews,a.train_manifest,a.output,a.gallery)
