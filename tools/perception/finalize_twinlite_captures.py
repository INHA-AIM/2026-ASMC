#!/usr/bin/env python3
"""Freeze audited Capture targets into lineage-separated train/val/test splits.

This step reuses the expensive RGB/Semantic audit without overwriting it. The
six sample-b runs form validation; the earlier comp sample session forms test.
All later object-collection layouts and remaining valid captures remain train.
Weather is read from collection reports, never guessed from image appearance.
"""
import argparse
from collections import Counter, defaultdict
import hashlib
import html
import json
from pathlib import Path
import random
import sqlite3
import sys

import cv2
import numpy as np

from prepare_twinlite_captures import process, read_json, sha, dump, now, source_path


def previews(rows, roots, data_root, gallery):
    """Show complete conditions/views/splits rather than adjacent start frames."""
    cells = defaultdict(list)
    for row in rows.values():
        if row.get('in_target_conditions'):
            cells[(row['split'],row['weather'],row['hour'],row['view'])].append(row)
    rng = random.Random(20261017)
    selected = []
    for cell, candidates in sorted(cells.items()):
        candidates.sort(key=lambda r:r['sample_id'])
        choice = rng.choice(candidates)
        marking = max(candidates,key=lambda r:r['class_pixels']['stopline'])
        selected.append(choice)
        if marking['sample_id'] != choice['sample_id']:
            selected.append(marking)
    result = []
    for index,row in enumerate(selected):
        rgb = cv2.imread(str(source_path(roots,row['source_ref'])))
        if rgb is None:raise RuntimeError('preview RGB missing')
        geometry = row['geometry']; h,w = geometry['target_hw']
        rh,rw = geometry['resized_hw']; left,top,_,_ = geometry['pad_ltrb']
        canvas = np.full((h,w,3),114,np.uint8)
        resized = cv2.resize(rgb,(rw,rh),interpolation=cv2.INTER_AREA if rh<rgb.shape[0] or rw<rgb.shape[1] else cv2.INTER_LINEAR)
        canvas[top:top+rh,left:left+rw] = resized
        pack = cv2.imread(str(data_root/row['packed_targets']),cv2.IMREAD_UNCHANGED)
        da = pack&1; mark = (pack>>1)&3
        color = np.zeros_like(canvas); color[da>0] = (40,200,40)
        for c,value in [(1,(255,255,255)),(2,(0,230,255)),(3,(0,0,255))]:color[mark==c] = value
        overlay = canvas.copy(); active = (da>0)|(mark>0)
        overlay[active] = cv2.addWeighted(canvas,.5,color,.5,0)[active]
        path = 'preview-{:03d}.jpg'.format(index)
        if not cv2.imwrite(str(gallery/path),np.hstack([canvas,overlay])):
            raise RuntimeError('preview write failed')
        result.append(dict(sample_id=row['sample_id'],image=path,split=row['split'],
                           weather=row['weather'],hour=row['hour'],view=row['view']))
    dump(gallery/'preview.json',result)
    return result


def report_conditions(evidence):
    """Read explicit per-run conditions, rejecting contradictory provenance."""
    result = {}
    for path in sorted(evidence.glob('*.json')):
        document = read_json(path)
        if not isinstance(document, dict):
            continue
        runs = document.get('runs', [])
        if not isinstance(runs, list):
            continue
        for run in runs:
            if not isinstance(run, dict) or not run.get('run_id'):
                continue
            weather = str(run.get('weather', '')).upper()
            hour = run.get('hour')
            if weather not in ('SUNNY', 'FOGGY') or hour not in (11, 13, 15, 17):
                continue
            value = (weather, hour)
            prior = result.get(run['run_id'])
            if prior and (prior['weather'], prior['hour']) != value:
                raise ValueError('conflicting collection conditions: '+run['run_id'])
            result[run['run_id']] = dict(weather=weather, hour=hour,
                evidence=str(path), evidence_sha256=sha(path.read_bytes()),
                confidence='recorded by collection runner')
    return result


def partition(row):
    run = row['run_id']
    if '_kcity_comp_sample_b_' in run:
        return 'val', 'capture_session:20260904_comp_sample_b'
    if '_kcity_comp_sample_' in run:
        return 'test', 'capture_session:20260903_20260904_comp_sample'
    # Unregistered/manual probes are training only. Their independence is not
    # asserted; retaining them never creates another evaluation scene.
    return 'train', row['split_group']


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--data-root', type=Path, required=True)
    p.add_argument('--parent-version', default='twinlite_morai_v7_large_capture_r1')
    p.add_argument('--dataset-version', default='twinlite_morai_v7_large_capture_r2')
    p.add_argument('--silver-root', type=Path, required=True)
    p.add_argument('--capture-root', type=Path, required=True)
    p.add_argument('--gallery', type=Path, required=True)
    p.add_argument('--parent-gallery', type=Path, required=True)
    p.add_argument('--code-root', type=Path, required=True)
    a = p.parse_args()
    cv2.setNumThreads(1)
    parent = a.data_root/'dataset_versions'/a.parent_version
    if not (parent/'_SUCCESS').is_file():
        raise RuntimeError('parent audit has not completed')
    output = a.data_root/'dataset_versions'/a.dataset_version
    output.mkdir(parents=True, exist_ok=False)
    a.gallery.mkdir(parents=True, exist_ok=False)
    conditions = report_conditions(a.silver_root/'collection-evidence-20261012')
    meta = read_json(parent/'dataset.json')
    roots = dict(capture_raw=a.capture_root, silver=a.silver_root)
    rows = {}
    with sqlite3.connect('file:'+str(parent/'audit.sqlite')+'?mode=ro', uri=True) as db:
        for (text,) in db.execute('select row from samples order by rank'):
            row = json.loads(text)
            rows[row['pixel_sha256']] = row
        conflicts = {r[0] for r in db.execute('select pixel from conflicts')}

    # ENOMEM is a transient I/O/resource failure, not evidence of bad imagery.
    # Retry explicitly and fail closed if it remains unresolved.
    retry_ids = set()
    for line in (parent/'rejected.jsonl').open():
        r = json.loads(line)
        if 'Cannot allocate memory' in r.get('rejected', ''):
            retry_ids.add((r['custom_name'], r['view']))
    recovered = 0
    for line in (parent/'candidate-manifest.jsonl').open():
        record = json.loads(line)
        if (record['custom_name'], record['view']) not in retry_ids:
            continue
        result = process(record, roots, meta['semantic_policy'])
        if 'rejected' in result:
            raise RuntimeError('resource-error retry failed: '+str(result))
        row = result['row']
        if row['pixel_sha256'] in conflicts:
            continue
        previous = rows.get(row['pixel_sha256'])
        if previous:
            if previous['supervision_sha256'] != row['supervision_sha256']:
                raise RuntimeError('conflicting supervision during recovery')
            continue
        target = a.data_root/row['packed_targets']
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(result['encoded'])
        row['packed_target_file_sha256'] = sha(result['encoded'])
        rows[row['pixel_sha256']] = row
        recovered += 1

    counters = defaultdict(Counter)
    pixels = defaultdict(Counter)
    positives = defaultdict(Counter)
    groups = {}; samples = set(); checked = 0
    streams = {split:(output/(split+'.jsonl')).open('w') for split in ('train','val','test')}
    run_counts = Counter()
    try:
        for row in sorted(rows.values(), key=lambda r:r['sample_id']):
            row['previous_split'] = row['split']
            row['split'], row['split_group'] = partition(row)
            split, group = row['split'], row['split_group']
            if group in groups and groups[group] != split:
                raise RuntimeError('lineage leakage: '+group)
            groups[group] = split
            if row['sample_id'] in samples:
                raise RuntimeError('duplicate sample ID')
            samples.add(row['sample_id'])
            evidence = conditions.get(row['run_id'])
            if evidence:
                row.update(weather=evidence['weather'], hour=evidence['hour'])
                row['condition_provenance'] = evidence
            else:
                row['condition_provenance'] = dict(confidence='recorded run name; unknown stays unknown')
            # This collector report exists in the preserved v4 evaluation record;
            # retain its uncertainty rather than upgrading it to RPC verification.
            if row['run_id'] == '20260903_211826_kcity_comp_sample_sunny_sim_unknown_wheel':
                row['hour'] = 13
                row['condition_provenance'] = dict(confidence='collector-reported estimate, not RPC verified',
                    evidence='docs/records/camera/twinlitenet-v4-evaluation.md: sunny 1pm collector report')
            row['dataset_version'] = a.dataset_version
            row['in_target_conditions'] = row['weather'] in ('SUNNY','FOGGY') and row['hour'] in (11,13,15)
            packed_path = a.data_root/row['packed_targets']
            blob = packed_path.read_bytes()
            packed = cv2.imdecode(np.frombuffer(blob, np.uint8), cv2.IMREAD_UNCHANGED)
            if (sha(blob) != row['packed_target_file_sha256'] or packed is None
                    or list(packed.shape) != row['geometry']['target_hw']
                    or packed.dtype != np.uint8 or np.any(packed & 240)
                    or sha(packed.tobytes()) != row['supervision_sha256']):
                raise RuntimeError('packed target verification failed: '+row['sample_id'])
            valid = (packed >> 3) & 1
            marking = (packed >> 1) & 3
            drivable = packed & 1
            if int(valid.sum()) != row['valid_pixels'] or int(drivable.sum()) != row['drivable_pixels']:
                raise RuntimeError('target count mismatch')
            for name, identifier in [('background',0),('white_lane',1),('yellow_lane',2),('stopline',3)]:
                count = int(np.count_nonzero((marking == identifier)&(valid > 0)))
                if count != row['class_pixels'][name]:
                    raise RuntimeError('class count mismatch')
            streams[split].write(json.dumps(row, ensure_ascii=False)+'\n')
            counters['split'][split] += 1
            counters['conditions'][(split,row['weather'],str(row['hour']))] += 1
            counters['views'][(split,row['view'])] += 1
            counters['sources'][row['source_ref']['root']] += 1
            for name,count in row['class_pixels'].items():
                pixels[split][name] += count; positives[split][name] += int(count > 0)
            pixels[split]['drivable'] += row['drivable_pixels']
            positives[split]['drivable'] += int(row['drivable_pixels'] > 0)
            run_counts[(split,row['run_id'])] += 1
            checked += 1
            if checked % 3000 == 0:
                print('re-read verified targets',checked,flush=True)
    finally:
        for stream in streams.values():stream.close()

    if set(counters['split']) != {'train','val','test'}:
        raise RuntimeError('missing split')
    coverage = {}
    for split in ('train','val','test'):
        coverage[split] = {w+':'+str(h):counters['conditions'][(split,w,str(h))]
                          for w in ('SUNNY','FOGGY') for h in (11,13,15)}
        if not all(coverage[split].values()):
            raise RuntimeError('a required weather/time cell is absent: '+split)
        if not all(positives[split][name] for name in ('drivable','white_lane','yellow_lane','stopline')):
            raise RuntimeError('a supervised class is absent: '+split)
    exposures = {}
    desired = max(coverage['train'].values())
    for condition,count in coverage['train'].items():
        exposures[condition] = dict(images=count,epoch_exposures=min(desired,2*count),
                                    maximum_repeats=2)
    meta.update(dataset_version=a.dataset_version,created_at_utc=now(),
        parent=dict(dataset_version=a.parent_version,metadata_sha256=sha((parent/'dataset.json').read_bytes())),
        split_counts=dict(counters['split']),class_pixels=dict(pixels),positive_images=dict(positives),
        view_images=[dict(split=s,view=v,images=n) for (s,v),n in sorted(counters['views'].items())],
        conditions=[dict(split=s,weather=w,hour=h,images=n) for (s,w,h),n in sorted(counters['conditions'].items())],
        source_images=dict(counters['sources']),split_groups=groups,
        source_paths_recorded={k:str(v) for k,v in roots.items()},
        source_roots_required=sorted(counters['sources']),
        split_manifest_sha256={s:sha((output/(s+'.jsonl')).read_bytes()) for s in streams},
        runs=[dict(split=s,run_id=r,images=n) for (s,r),n in sorted(run_counts.items())],
        balance_conditions=True,condition_coverage=coverage,train_epoch_exposures=exposures,
        selection_metric='six_condition_macro_mean_head_iou',
        split_policy=dict(seed=20261017,unit='whole collection session/layout, all views/conditions together',
            validation='20260904 comp sample-b, all six conditions',
            test='20260903/04 comp sample session, all conditions',
            training='all other valid captures, including every later YOLO collection layout',
            ratios='determined by complete session sizes; no random frame split or valid image discard',
            evaluation='report both pooled scores and equal-weight six-condition macro scores; never repeat val/test',
            independence='same K-City map; separate capture sessions, not a geographic or historically unseen blind test',
            historical_usage='old Medium models used these images; no previous MORAI weights initialize the new BDD Large run'),
        resource_recovery=dict(retried=len(retry_ids),recovered=recovered,unresolved=0),
        verification=dict(meta['verification'],all_packed_targets_reloaded=True,
            target_images_verified=checked,all_weather_time_cells_present=True,
            all_four_supervised_classes_present=True,split_group_leakage=False,
            exact_RGB_duplicate_and_conflict_checked=True,
            evaluation='new BDD Large split; previously used Medium development data, not blind competition acceptance'))
    if retry_ids:
        meta['rejections'].pop('[Errno 12] Cannot allocate memory',None)
    code_files = ['tools/perception/prepare_twinlite_captures.py','tools/perception/finalize_twinlite_captures.py',
        'scripts/train_twinlite.py','scripts/evaluate_twinlite.py','src/asmc_camera/src/asmc_camera/training/twinlite/dataset.py',
        'src/asmc_camera/src/asmc_camera/training/twinlite/batching.py',
        'src/data_collection/src/data_collection/mask_baker.py','tools/perception/capture_rgb_quality.py',
        'src/asmc_camera/src/asmc_camera/training/twinlite/tasks.py',
        'src/asmc_camera/src/asmc_camera/training/twinlite/losses.py']
    meta['code_sha256'] = {f:sha((a.code_root/f).read_bytes()) for f in code_files}
    dump(output/'dataset.json',meta)
    dump(output/'verification.json',meta['verification'])
    dump(output/'condition-provenance.json',conditions)
    dump(output/'_SUCCESS',dict(completed_at=now(),dataset_sha256=sha((output/'dataset.json').read_bytes())))
    dump(a.gallery/'dataset-summary.json',meta)
    table = '<tr><th>Split</th><th>SUNNY11</th><th>SUNNY13</th><th>SUNNY15</th><th>FOGGY11</th><th>FOGGY13</th><th>FOGGY15</th><th>전체</th></tr>'
    for split in ('train','val','test'):
        table += '<tr><td>'+split+'</td>'+''.join('<td>'+format(n,',')+'</td>' for n in coverage[split].values())+'<td>'+format(counters['split'][split],',')+'</td></tr>'
    preview = previews(rows,roots,a.data_root,a.gallery)
    cards = ''.join('<figure><img loading="lazy" src="'+html.escape(p['image'])+'"><figcaption>'+html.escape('{} · {} {}시 · {} · {}'.format(p['split'],p['weather'],p['hour'],p['view'],p['sample_id']))+'</figcaption></figure>' for p in preview)
    gpu_report = read_json(a.parent_gallery/'gpu-capacity.json')
    if gpu_report:dump(a.gallery/'gpu-capacity.json',gpu_report)
    (a.gallery/'index.html').write_text('<!doctype html><html lang="ko"><meta charset="utf-8"><title>TwinLite Large Capture 학습 준비</title><style>body{font:16px system-ui;background:#111827;color:#e5e7eb;max-width:1500px;margin:25px auto;padding:20px}a{color:#7dd3fc}table{border-collapse:collapse;width:100%}td,th{padding:12px;border:1px solid #475569}img{max-width:100%}figure{margin:18px 0}figcaption{overflow-wrap:anywhere}pre{white-space:pre-wrap}</style><a href="../index.html">ASMC 실험 목록</a><h1>TwinLite Large · 전체 Capture 학습 준비</h1><p><strong>학습 시작 안 함 · RGB/Semantic 전수검수와 입력 준비 완료</strong></p><p>전방 960×544 · 좌우 640×480. RGB/255, INTER_AREA 축소, 정답 최근접 보간. 주행가능노면 binary + 흰차선/황색차선/정지선/background 4-class.</p><table>'+table+'</table><p>전체 열에는 17시 및 조건 unknown 자료도 포함됩니다. SUNNY13 test의 기존 sim_unknown run은 과거 수집자 보고에 따른 추정으로 별도 기록했습니다.</p><p>모든 뷰·같은 배치의 날씨/시간은 한 split. 평가는 여섯 조건을 같은 비중으로 집계하며, train만 부족한 조건을 최대 두 번 노출합니다. 같은 K-City 배경과 과거 평가 사용 이력이 있으므로 비공개 독립 시험으로 설명하지 않습니다.</p><p><a href="dataset-summary.json">전체 집계·검증·출처</a></p><h2>원본 / 학습 GT 비교</h2>'+cards+'</html>')
    print(json.dumps(dict(dataset=str(output),splits=meta['split_counts'],conditions=coverage,
                         transient_recovery=meta['resource_recovery'],training_started=False),indent=2))


if __name__ == '__main__':
    main()
