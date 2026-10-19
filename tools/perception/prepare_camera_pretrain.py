#!/usr/bin/env python3
"""Immutable full-coverage v8 export; raw images stay linked and masks apply in the loader."""
import argparse
from collections import Counter, defaultdict
from datetime import datetime, timezone
import hashlib
import html
import json
from pathlib import Path

import cv2
import numpy as np
import yaml

from camera_pretrain_policy import (CLASSES, ROOT, class_images, epoch_order, load_policy,
    sha, supervision_state, tiny_reasons, validate_box, view_masks, guarded_data_yaml)


def read_rows(path):
    with Path(path).open() as f:
        for line in f:
            if line.strip():
                yield json.loads(line)


def write_json(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n')


def source_image(dataset, row):
    if 'image' in row:
        return Path(row['image'])
    role = {'selection': 'val', 'heldout': 'test'}.get(row['split'], row['split'])
    files = list((dataset/'images'/role).glob(row['key']+'.*'))
    if len(files) != 1:
        raise ValueError('source image link not unique')
    return files[0].resolve()


def instance_patch(row, image_path, ann, roi, cache):
    """Only decode masks for boxes intersecting Ego; never infer visibility from bbox area."""
    x1, y1, x2, y2 = roi
    if row.get('source_run'):
        if 'packed' not in cache:
            instance_path = image_path.parents[2]/'instance'/row['view']/image_path.name
            manifest = image_path.parents[3]/'manifest.jsonl'
            frames = cache['frame_indexes'].get(str(manifest))
            if frames is None:
                frames = {r['frame_id']: r for r in read_rows(manifest)}
                cache['frame_indexes'][str(manifest)] = frames
            frame = frames[row['frame_id']]
            relative = frame['paths']['instance'][row['view']]
            if sha(instance_path) != frame['sha256'][relative]:
                raise ValueError('source instance SHA mismatch')
            # PIL RGBA agrees with native RGBA tags; OpenCV would read BGRA.
            from PIL import Image
            rgba = np.array(Image.open(instance_path).convert('RGBA'))
            if rgba.shape[:2] != cache['shape']:
                raise ValueError('RGB/instance dimensions disagree')
            cache['packed'] = np.ascontiguousarray(rgba).view(np.uint32).reshape(rgba.shape[:2])
        tag = np.asarray(ann['rgba'], dtype=np.uint8).view(np.uint32).item()
        return cache['packed'][y1:y2, x1:x2] == tag
    if ann.get('native_mask'):
        path = Path(ann['native_mask_root'])/ann['native_mask']
        if str(path) not in cache:
            with np.load(path) as z:
                cache[str(path)] = (z['masks'].copy(), int(z['width']), int(z['height']))
        masks, width, height = cache[str(path)]
        if (height, width) != cache['shape']:
            raise ValueError('packed instance dimensions disagree')
        return np.unpackbits(masks[ann['mask_index']], axis=-1)[:, :width][y1:y2, x1:x2].astype(bool)
    return None


def normalize_row(dataset, old, image, policy, masks, frame_indexes):
    width, height = image.shape[1], image.shape[0]
    original = source_image(dataset, old)
    view = old.get('view', 'unknown')
    ego = masks[view].geometry(height, width)[0] if view in masks else np.zeros((height, width), bool)
    cache = dict(shape=(height, width), frame_indexes=frame_indexes)
    anns = []
    for i, a in enumerate(old['annotations']):
        a = dict(a)
        if a.get('original_xyxy') is not None:
            a['source_original_xyxy'] = list(a['original_xyxy'])
        label = a.get('training_class') or a.get('canonical')
        if label not in CLASSES:
            if a.get('disposition') == 'excluded':
                anns.append(dict(a, supervision='excluded', training_class=None))
                continue
            raise ValueError('unresolved target class: '+str(label))
        box = list(a['xyxy'])
        validate_box(box, width, height)
        visible = a.get('visible_pixels')
        if visible is None and old.get('original_annotation'):
            originals = old['original_annotation']['annotations']
            if str(a.get('id', '')).isdecimal() and int(a['id']) < len(originals):
                visible = originals[int(a['id'])].get('visible_pixels')
        roi = (int(np.floor(box[0])), int(np.floor(box[1])),
               int(np.ceil(box[2])), int(np.ceil(box[3])))
        x1, y1, x2, y2 = roi
        mask_intersects = bool(ego[y1:y2, x1:x2].any())
        disposition, reasons = 'positive', []
        if visible == 0:
            disposition, reasons = 'excluded', ['fully_occluded_no_visible_instance_pixels']
        if mask_intersects:
            pixels = instance_patch(old, original, a, roi, cache)
            if pixels is not None:
                pixels &= ~ego[y1:y2, x1:x2]
                ys, xs = np.nonzero(pixels)
                visible = len(xs)
                if not visible:
                    disposition, reasons = 'excluded', ['ego_no_remaining_instance_pixels']
                else:
                    box = [int(xs.min()+x1), int(ys.min()+y1), int(xs.max()+x1+1), int(ys.max()+y1+1)]
            elif ego[y1:y2, x1:x2].all():
                disposition, reasons = 'ignore', ['ego_fully_covered_bbox_without_instance']
        if disposition == 'positive':
            reasons = tiny_reasons(box, width, height, policy, visible)
            if reasons:
                disposition = 'ignore'
        anns.append(dict(a, id=str(a.get('id', i)), training_class=label,
                         original_xyxy=list(a['xyxy']), xyxy=box, visible_pixels=visible,
                         supervision=disposition, supervision_reasons=reasons,
                         ego_mask_intersection=mask_intersects))
    key = old.get('id') or old['key']
    source = old.get('source') or 'capture:'+old['source_run']
    frame = old.get('frame_id')
    if frame is None and source.startswith('capture:'):
        frame = int(old['source_key'].rsplit('/', 1)[-1])
    correlation = (f'{source}:{int(frame)//30}' if frame is not None else
                   source+':'+str(old.get('augmentation_family') or key))
    row = dict(id=key, source=source, image=str(original), image_sha256=old['image_sha256'],
        pixel_sha256=old['pixel_sha256'], split=old['split'], width=width, height=height,
        split_group=old.get('split_cluster', old['split_group']), view=view,
        correlation_group=correlation, annotations=anns,
        supervision=supervision_state(old, bool(old.get('source_run'))),
        ego_mask_applied=view in masks, mask_config_sha256=masks[view].config_sha256 if view in masks else None,
        weather=old.get('weather', 'unknown'), tunnel=old.get('tunnel', 'unknown'),
        parent_dataset=dataset.name, parent_manifest_sha256=cache['frame_indexes']['parent_shas'][str(dataset)],
        parent_row_id=key,
        provenance={k:old[k] for k in ('source_key', 'source_run', 'frame_id', 'layout_id',
            'ledger_sha256', 'source_manifest_sha256', 'annotation_sha256', 'review_record',
            'annotation_complete', 'breadth_quality', 'original_annotation') if k in old})
    return row


def yolo_lines(row):
    lines = []
    for a in row['annotations']:
        if a['supervision'] != 'positive':
            continue
        x1, y1, x2, y2 = a['xyxy']
        w, h = row['width'], row['height']
        lines.append('{} {:.10f} {:.10f} {:.10f} {:.10f}'.format(
            CLASSES.index(a['training_class']), (x1+x2)/2/w, (y1+y2)/2/h, (x2-x1)/w, (y2-y1)/h))
    return lines


def verify_export(output, rows, parent_shas):
    pixels, groups = defaultdict(set), defaultdict(set)
    failures = []
    for r in rows:
        role = {'selection':'val', 'heldout':'test'}.get(r['split'], r['split'])
        try:
            actual = (output/'labels'/role/(r['id']+'.txt')).read_text().splitlines()
            if actual != yolo_lines(r):
                raise ValueError('class/coordinates/count re-read mismatch')
            side = json.loads((output/'supervision'/role/(r['id']+'.json')).read_text())
            if side['annotations'] != r['annotations'] or side['supervision'] != r['supervision']:
                raise ValueError('ignore sidecar mismatch')
            image = output/r['export_image']
            if image.resolve() != Path(r['image']).resolve() or not image.is_file():
                raise ValueError('broken/wrong image link')
            pixels[r['pixel_sha256']].add(r['split'])
            groups[r['split_group']].add(r['split'])
        except (ValueError, OSError) as exc:
            failures.append(dict(id=r['id'], reason=str(exc)))
    for kind, index in [('pixel', pixels), ('lineage', groups)]:
        for key, roles in index.items():
            if len(roles) > 1:
                failures.append(dict(reason=kind+'_split_leak', key=key, roles=sorted(roles)))
    for dataset, expected in parent_shas.items():
        if sha(Path(dataset)/'manifest.jsonl') != expected:
            failures.append(dict(reason='parent manifest changed', dataset=dataset))
    return dict(passed=not failures, images=len(rows), failures=failures,
        checks=['all source RGB SHA and decoded pixel hashes during preparation',
                'actual YOLO labels and supervision sidecars', 'fixed16 IDs and coordinates',
                'source links', 'frozen split and lineage separation', 'immutable parents'],
        near_duplicate_scene_independence_claimed=False)


def prepare(args):
    policy = load_policy(args.policy)
    inventory = json.loads(args.inventory.read_text())
    if args.output.exists():
        raise ValueError('immutable version exists; choose a new revision')
    parents = [args.data_root/'dataset_versions'/d['dataset'] for d in inventory['datasets']]
    parent_shas = {str(p): sha(p/'manifest.jsonl') for p in parents}
    for p, pin in zip(parents, inventory['datasets']):
        if parent_shas[str(p)] != pin['manifest_sha256']:
            raise ValueError('parent manifest differs from pinned inventory')
    originals = [(p, r) for p in parents for r in read_rows(p/'manifest.jsonl')]
    eval_pixels = {r['pixel_sha256'] for _, r in originals if r['split'] != 'train'}
    args.output.mkdir(parents=True)
    for folder in ('images', 'labels', 'supervision'):
        for role in ('train', 'val', 'test'):
            (args.output/folder/role).mkdir(parents=True)
    masks = view_masks(policy)
    frame_indexes = {'parent_shas': parent_shas}
    rows, exclusions, seen, gallery = [], [], set(), defaultdict(list)
    for n, (dataset, old) in enumerate(originals):
        if n % 1000 == 0:
            print(json.dumps(dict(processed=n, total=len(originals), adopted=len(rows))), flush=True)
        try:
            path = source_image(dataset, old)
            if sha(path) != old['image_sha256']:
                raise ValueError('source RGB SHA mismatch')
            image = cv2.imread(str(path))
            if image is None:
                raise ValueError('RGB decode failed')
            from PIL import Image
            rgb = Image.open(path).convert('RGB')
            raw_pixels = rgb.tobytes()
            # Earlier v7 hashes include dimensions; route exports used bytes alone.
            # Verify both source contracts, then use ONE dimensions-qualified key.
            pixel = hashlib.sha256(f'{rgb.width}x{rgb.height}:'.encode()+raw_pixels).hexdigest()
            legacy_pixel = hashlib.sha256(raw_pixels).hexdigest()
            if old['pixel_sha256'] not in (pixel, legacy_pixel):
                raise ValueError('source decoded pixels changed')
            row = normalize_row(dataset, old, image, policy, masks, frame_indexes)
            row['source_pixel_sha256'] = old['pixel_sha256']
            row['pixel_sha256'] = pixel
            if row['split'] == 'train' and (pixel in eval_pixels or legacy_pixel in eval_pixels):
                raise ValueError('frozen evaluation overlap')
            if pixel in seen:
                raise ValueError('exact duplicate')
            if row['split'] == 'train' and not any(a['supervision']=='positive' for a in row['annotations']):
                exclusions.append(dict(id=row['id'], reason='no_usable_positive_not_negative', row=row))
                if len(gallery['ignore_only']) < 6:
                    gallery['ignore_only'].append(row)
                continue
            role = {'selection':'val', 'heldout':'test'}.get(row['split'], row['split'])
            row['export_image'] = f'images/{role}/{row["id"]}{path.suffix.lower()}'
            (args.output/row['export_image']).symlink_to(path.resolve())
            label = args.output/'labels'/role/(row['id']+'.txt')
            lines = yolo_lines(row)
            label.write_text('\n'.join(lines)+('\n' if lines else ''))
            row['export_label_sha256'] = sha(label)
            write_json(args.output/'supervision'/role/(row['id']+'.json'), dict(
                supervision=row['supervision'], annotations=row['annotations'],
                image_sha256=row['image_sha256'], policy_sha256=policy['policy_sha256']))
            seen.add(pixel)
            rows.append(row)
            kinds = {a['supervision'] for a in row['annotations']}
            for kind in kinds:
                if len(gallery[kind]) < 12:
                    gallery[kind].append(row)
            if any(a.get('ego_mask_intersection') for a in row['annotations']) and len(gallery['mask_edge']) < 12:
                gallery['mask_edge'].append(row)
            if row['supervision']=='partial' and len(gallery['partial']) < 8:
                gallery['partial'].append(row)
        except (ValueError, OSError, KeyError) as exc:
            exclusions.append(dict(id=old.get('id', old.get('key')), parent_dataset=dataset.name, reason=str(exc)))
    (args.output/'manifest.jsonl').write_text(''.join(json.dumps(r, ensure_ascii=False)+'\n' for r in rows))
    (args.output/'exclusions.jsonl').write_text(''.join(json.dumps(r, ensure_ascii=False)+'\n' for r in exclusions))
    write_json(args.output/'policy.json', policy)
    train = [r for r in rows if r['split']=='train']
    order, balance = epoch_order(train, policy['seed'], alignment=16)
    (args.output/'epoch-000-order.jsonl').write_text(''.join(json.dumps(dict(id=train[i]['id']))+'\n' for i in order))
    states = Counter((r['split'], r['supervision']) for r in rows)
    objects = Counter((r['split'], a['supervision']) for r in rows for a in r['annotations'])
    classes = {c:dict(positive=sum(a['training_class']==c and a['supervision']=='positive'
                                  for r in train for a in r['annotations']),
                     ignore=sum(a['training_class']==c and a['supervision']=='ignore'
                                for r in train for a in r['annotations'])) for c in CLASSES}
    summary = dict(version=args.output.name, created_utc=datetime.now(timezone.utc).isoformat(),
        source_train_images=inventory['unique_train_images_by_recorded_pixel_hash'],
        images_by_split=dict(Counter(r['split'] for r in rows)),
        supervision_images={f'{s}/{v}':n for (s,v),n in states.items()},
        objects={f'{s}/{v}':n for (s,v),n in objects.items()}, classes=classes,
        train_class_images=dict(class_images(train)), balance=balance,
        exclusion_reasons=dict(Counter(r['reason'] for r in exclusions)),
        background_images=0, background_status='separate native-GT and RGB review required',
        training_performed=False, original_files_changed=False, evaluation_splits_moved=False)
    write_json(args.output/'summary.json', summary)
    verification = verify_export(args.output, rows, parent_shas)
    write_json(args.output/'verification.json', verification)
    meta = dict(schema_version='camera-pretrain-1', classes=list(CLASSES),
        requires_ignore_aware_trainer=True, original_data_root=str(args.data_root.resolve()),
        policy_sha256=policy['policy_sha256'], parents=parent_shas,
        artifact_shas={p:sha(args.output/p) for p in ('manifest.jsonl', 'policy.json', 'epoch-000-order.jsonl')},
        recommended_training=dict(imgsz=960, rect=False, multi_scale=0, **policy['augmentation']),
        checkpoint_selection=dict(selection_only=True, heldout_final_only=True,
            metrics=['class recall', 'complete-GT F1', 'person/motorcycle regression'],
            unsupported_class_is_not_pass=True))
    write_json(args.output/'pretrain.json', meta)
    (args.output/'data.yaml').write_text(yaml.safe_dump(guarded_data_yaml(args.output), sort_keys=False))
    if args.gallery:
        build_gallery(args.gallery, summary, gallery, masks)
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    if not verification['passed']:
        raise ValueError('v8 verification failed; inspect verification.json')


def build_gallery(output, summary, examples, masks):
    output.mkdir(parents=True, exist_ok=True)
    cards = []
    for kind, rows in examples.items():
        for i, row in enumerate(rows):
            original = cv2.imread(row['image'])
            masked = masks[row['view']].apply(original) if row['view'] in masks else original.copy()
            before = original.copy()
            for a in row['annotations']:
                if a.get('training_class') not in CLASSES:
                    continue
                b = list(map(lambda v:int(round(v)), a['original_xyxy']))
                cv2.rectangle(before, tuple(b[:2]), tuple(b[2:]), (0,255,255), 1)
                b = list(map(lambda v:int(round(v)), a['xyxy']))
                color = {'positive':(60,230,60), 'ignore':(255,40,220), 'excluded':(0,160,255)}[a['supervision']]
                cv2.rectangle(masked, tuple(b[:2]), tuple(b[2:]), color, 2)
                if a['supervision']=='positive':
                    cv2.putText(masked, a['training_class'], tuple(b[:2]), cv2.FONT_HERSHEY_SIMPLEX,.4,color,1)
            pair = np.concatenate([before, masked], axis=1)
            file=f'{kind}-{i:02}.jpg'; cv2.imwrite(str(output/file), pair)
            cards.append('<article><h3>'+html.escape(kind+' '+row['view']+' '+row['id'])+
                '</h3><img loading="lazy" src="'+file+'"><details><summary>GT details</summary><pre>'+
                html.escape(json.dumps(row['annotations'],ensure_ascii=False,indent=2))+'</pre></details></article>')
    (output/'index.html').write_text('<meta charset="utf-8"><title>E v8 pretrain 2026-10-12</title>'+
        '<style>body{background:#101827;color:white;font:16px system-ui}img{max-width:100%}pre{white-space:pre-wrap}</style>'+
        '<h1>E v8 pretrain — 2026-10-12</h1><p>Left: original GT. Right: Ego mask + green positive, magenta ignore, orange excluded.</p><pre>'+
        html.escape(json.dumps(summary,ensure_ascii=False,indent=2))+'</pre>'+''.join(cards))


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--inventory', type=Path, required=True)
    p.add_argument('--data-root', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--policy', type=Path)
    p.add_argument('--gallery', type=Path)
    prepare(p.parse_args())
