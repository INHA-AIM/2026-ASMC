#!/usr/bin/env python3
"""Read-only audit of frozen validation size rules, provenance and gallery selection."""
import argparse
from collections import Counter, defaultdict
import hashlib
import html
from html.parser import HTMLParser
import json
from pathlib import Path
import sys


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def annotations(row):
    return [(a.get('id'), a.get('training_class'), a.get('supervision'), a['xyxy'])
            for a in row['annotations']]


class GalleryImages(HTMLParser):
    def __init__(self):
        super().__init__()
        self.ids = []

    def handle_starttag(self, tag, attrs):
        if tag == 'img':
            self.ids.append(Path(dict(attrs).get('src', '')).stem)


def audit(run, dataset, out):
    # Use the exact frozen policy and evaluator already used by this run.
    sys.path.insert(0, str(run/'code/tools/perception'))
    from camera_pretrain_policy import tiny_reasons, CLASSES
    from evaluate_camera_bbox_pilot import frame_counts
    from PIL import Image, ImageDraw, ImageOps
    out.mkdir(parents=True, exist_ok=False)
    policy = json.loads((dataset/'policy.json').read_text())
    rows = [json.loads(line) for line in (run/'evaluation/manifest.jsonl').open()]
    by_id = {r['id']: r for r in rows}
    sources, families, views = Counter(), Counter(), Counter()
    states, reasons = Counter(), Counter()
    class_sources, class_images = defaultdict(Counter), defaultdict(set)
    class_actors, class_groups = defaultdict(set), defaultdict(set)
    complete_gt = Counter()
    violations, sha_errors, evaluated_ignore, label_errors, scope_errors = [], [], [], [], []
    report_rows = []
    for r in rows:
        family = r['source'].split(':', 1)[0]
        sources[r['source']] += 1
        families[family] += 1
        views[r.get('view', 'unknown')] += 1
        if digest(r['image']) != r['image_sha256']:
            sha_errors.append(r['id'])
        pos = []
        for a in r['annotations']:
            states[a['supervision']] += 1
            why = tiny_reasons(a['xyxy'], r['width'], r['height'], policy,
                               a.get('visible_pixels'))
            if a['supervision'] == 'positive':
                pos.append(a)
                c = a['training_class']
                class_sources[c][family] += 1
                class_images[c].add(r['id'])
                class_groups[c].add(r.get('correlation_group', r['id']))
                if a.get('actor_id') is not None:
                    class_actors[c].add((r['source'], str(a['actor_id'])))
                if r['supervision'] in ('complete', 'reviewed_negative'):
                    complete_gt[c] += 1
                if why:
                    violations.append(dict(id=r['id'], annotation=a.get('id'), reasons=why))
            elif a['supervision'] == 'ignore':
                reasons.update(a.get('supervision_reasons', []))
        c, _ = frame_counts(r, [], .4)
        if sum(v['gt'] for v in c.values()) != len(pos):
            evaluated_ignore.append(r['id'])
        scope = r.get('applied_policy', {})
        if any(scope.get(k) != v for k, v in dict(short_px=4, area_px2=64,
                    reference_tensor=960, native_also_checked=True, tiny_treatment='ignore').items()):
            scope_errors.append(r['id'])
        label_path = dataset/'labels/val'/Path(r['export_image']).with_suffix('.txt').name
        actual = [list(map(float, x.split())) for x in label_path.read_text().splitlines() if x.strip()]
        expected = []
        for a in pos:
            x, y, X, Y = a['xyxy']
            expected.append([CLASSES.index(a['training_class']), (x+X)/2/r['width'],
                (y+Y)/2/r['height'], (X-x)/r['width'], (Y-y)/r['height']])
        if len(actual) != len(expected) or any(len(x) != 5 or max(abs(u-v) for u, v in zip(x,y)) > 1e-6
                                             for x, y in zip(actual, expected)):
            label_errors.append(r['id'])
        report_rows.append(dict(id=r['id'], source=r['source'], family=family,
            view=r.get('view', 'unknown'), classes=sorted({a['training_class'] for a in pos}),
            positive=len(pos), ignore=sum(a['supervision']=='ignore' for a in r['annotations'])))
    # Inspect parent membership and train leakage without changing split or data.
    val_hashes = {r['image_sha256'] for r in rows}
    val_pixels = {r[k] for r in rows for k in ('pixel_sha256', 'source_pixel_sha256') if r.get(k)}
    val_groups = {r['split_group'] for r in rows}
    leaks, mismatches, found = [], [], set()
    role_sizes = Counter()
    heldout_states, heldout_bad = Counter(), []
    for line in (dataset/'manifest.jsonl').open():
        r = json.loads(line)
        role_sizes[r['split']] += 1
        if r['id'] in by_id:
            found.add(r['id'])
            original = by_id[r['id']]
            if r['split'] != 'selection' or annotations(r) != annotations(original) or r['image_sha256'] != original['image_sha256']:
                mismatches.append(r['id'])
        if r['split'] == 'train':
            hits = []
            if r['image_sha256'] in val_hashes: hits.append('image_sha')
            if any(r.get(k) in val_pixels for k in ('pixel_sha256', 'source_pixel_sha256')): hits.append('pixel_sha')
            if r['split_group'] in val_groups: hits.append('split_group')
            if hits: leaks.append(dict(id=r['id'], reasons=hits))
        if r['split'] == 'heldout':
            for a in r['annotations']:
                heldout_states[a['supervision']] += 1
                if a['supervision'] == 'positive' and tiny_reasons(a['xyxy'], r['width'], r['height'], policy, a.get('visible_pixels')):
                    heldout_bad.append(dict(id=r['id'], annotation=a.get('id')))
    capture = [r for r in rows if r['source'].startswith('capture:')]
    nearby = sum(any(x['view']==r['view'] and x['id']!=r['id'] and x['source']==r['source'] and
                    abs(x['provenance']['frame_id']-r['provenance']['frame_id']) <= 5 for x in capture)
                 for r in capture)
    gallery = GalleryImages()
    gallery.feed((run/'gallery/index.html').read_text())
    class_report = {c:dict(positive=sum(class_sources[c].values()), complete_gt=complete_gt[c],
        images=len(class_images[c]), source_families=dict(class_sources[c]),
        capture_actor_ids=len(class_actors[c]), recorded_correlation_groups=len(class_groups[c])) for c in CLASSES}
    result = dict(dataset=str(dataset), frozen_run=str(run), policy=dict(short_px=policy['tiny_short_px'],
        area_px2=policy['tiny_area_px2'], reference_tensor=policy['imgsz'], native_also_checked=True,
        equal_passes=True, tiny_treatment='ignore', range_cutoff=False),
        manifest_sha256=digest(run/'evaluation/manifest.jsonl'), dataset_policy_sha256=digest(dataset/'policy.json'),
        images=len(rows), sources=dict(sources), source_families=dict(families), views=dict(views),
        annotations=dict(states), classes=class_report, ignore_reasons=dict(reasons),
        positive_size_or_visibility_violations=violations, frozen_scope_errors=scope_errors,
        source_sha_errors=sha_errors, label_file_errors=label_errors, evaluator_gt_count_errors=evaluated_ignore,
        parent_annotation_mismatches=mismatches, parent_missing_ids=sorted(set(by_id)-found),
        train_leaks=leaks, split_images=dict(role_sizes), heldout_states=dict(heldout_states),
        heldout_positive_size_violations=heldout_bad,
        capture=dict(images=len(capture), unique_frame_indexes=len({r['provenance']['frame_id'] for r in capture}),
            recorded_groups=len({r['correlation_group'] for r in capture}),
            same_view_neighbor_within_five_capture_indexes=nearby,
            independent_scene_count_claimed=False),
        weather=dict(Counter(r.get('weather','unknown') for r in rows)),
        tunnel=dict(Counter(str(r.get('tunnel','unknown')) for r in rows)),
        current_failure_gallery=dict(images=len(gallery.ids),
            source_families=dict(Counter(by_id[i]['source'].split(':')[0] for i in gallery.ids)),
            policy='top 32 error-count frames, not a representative sample'),
        model_or_dataset_changed=False, heldout_predictions_loaded=False)
    result['structural_checks_passed'] = not any([violations, scope_errors, sha_errors, label_errors,
        evaluated_ignore, mismatches, result['parent_missing_ids'], leaks, heldout_bad])
    (out/'audit.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    (out/'frames.json').write_text(json.dumps(report_rows,ensure_ascii=False,indent=2)+'\n')
    cards=[]
    for i, (r, rr) in enumerate(zip(rows, report_rows)):
        base=Image.open(r['image']).convert('RGB')
        for include_ignore in (False, True):
            im=base.copy();draw=ImageDraw.Draw(im)
            for a in r['annotations']:
                if a['supervision']=='excluded' or a['supervision']=='ignore' and not include_ignore:continue
                color='#ff4fa3' if a['supervision']=='ignore' else '#00ffd2'
                draw.rectangle(a['xyxy'],outline=color,width=2)
                if a['supervision']=='positive':draw.text((a['xyxy'][0],max(0,a['xyxy'][1]-12)),a['training_class'],fill=color)
            im=ImageOps.contain(im,(960,720));im.save(out/f'{i:03d}-{int(include_ignore)}.jpg',quality=84)
        cards.append('<article data-family="'+rr['family']+'" data-view="'+rr['view']+'" data-classes=" '+
            ' '.join(rr['classes'])+' "><h3>'+str(i+1)+' · '+html.escape(rr['family']+' / '+rr['view'])+
            '</h3><p>'+html.escape(', '.join(rr['classes']))+f' · positive {rr["positive"]} / ignore {rr["ignore"]}</p>'+
            f'<a class="picture" href="{i:03d}-0.jpg" target="_blank"><img loading="lazy" data-index="{i:03d}" src="{i:03d}-0.jpg"></a><small>'+html.escape(r['id'])+'</small></article>')
    body='''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>2026-10-14 · v11 val 크기·출처·연속 장면 감사</title><style>body{font:16px/1.6 system-ui;background:#101827;color:#eee;margin:24px auto;padding:0 20px;max-width:1400px}a{color:#81cfff}table{border-collapse:collapse;width:100%}td,th{padding:8px;border:1px solid #506080}img{max-width:100%}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(320px,1fr));gap:18px}article{background:#243346;padding:12px;overflow-wrap:anywhere}article[hidden]{display:none}select{padding:8px}small{color:#b6bfd0}</style><h1>v11 val · 4px/64px² 적용과 출처 편중 감사</h1><p><a href="../index.html">ASMC 실험 목록</a> · <a href="audit.json">전수 검사 JSON</a> · <a href="frames.json">전체 205장 목록</a></p>'''
    body+='<p>원본과960 letterbox 기준 모두 검사합니다. 양성359개 / ignore800개. 기준 미달 양성 '+str(len(violations))+'개. 이 페이지는 실제 평가본 전체를 보여주며, 분할·정답·추론·학습을 변경하지 않습니다.</p>'
    body+='<p><b>출처 균형은 충분하지 않습니다.</b> Capture158장은 한 run, Roboflow36장은 Park 한 프로젝트이며 AIM11장입니다. 차량·상자 GT는 전부 Park입니다. Capture158장 중 '+str(nearby)+'장은 같은 뷰에서±5 Capture index 이내의 다른 표본이 있습니다. 이를 독립 장면 수로 세지 않습니다.</p>'
    body+='<p>기존 본학습 갤러리는 오류가 많은32장을 골라 표시하며 현재30장은 Capture입니다. E8→E12 차량 실패 비교는14장 중11장이 Park입니다. 아래 전체 목록과 목적이 다릅니다. 4px/64px²는 최소 GT 기준이며 먼 객체를 모두 제거하거나 예측 박스를 크기로 숨기는 설정이 아닙니다.</p>'
    body+='<table><tr><th>클래스</th><th>GT / complete GT</th><th>Capture</th><th>Roboflow</th><th>AIM</th><th>Capture actor ID 수</th></tr>'
    for c,v in class_report.items():
        body+='<tr><td>'+c+'</td><td>'+str(v['positive'])+' / '+str(v['complete_gt'])+'</td>'+''.join('<td>'+str(v['source_families'].get(k,0))+'</td>' for k in ('capture','roboflow','aim'))+'<td>'+str(v['capture_actor_ids'])+'</td></tr>'
    body+='</table><h2>전체 val — 기본은 양성만 표시</h2><p>청록: 양성 / 분홍: ignore. 원본 RGB 위의 GT입니다. 실제 추론은 기존 뷰별 Ego mask를 적용합니다. 사진을 클릭하면 크게 열립니다.</p>'
    body+='<select id="family"><option value="">모든 출처</option>'+''.join('<option>'+k+'</option>' for k in families)+'</select> <select id="cls"><option value="">모든 클래스</option>'+''.join('<option>'+c+'</option>' for c in CLASSES)+'</select> <select id="view"><option value="">모든 뷰</option>'+''.join('<option>'+v+'</option>' for v in views)+'</select> <label><input id="ignore" type="checkbox">ignore 함께 보기</label><p id="count"></p><div class="grid">'+''.join(cards)+'</div>'
    body+='''<script>function update(){const f=document.querySelector('#family').value,c=document.querySelector('#cls').value,v=document.querySelector('#view').value,s=document.querySelector('#ignore').checked?1:0;let n=0;document.querySelectorAll('article').forEach(a=>{a.hidden=!!((f&&a.dataset.family!==f)||(v&&a.dataset.view!==v)||(c&&!a.dataset.classes.includes(' '+c+' ')));if(!a.hidden)n++;const im=a.querySelector('img'),src=im.dataset.index+'-'+s+'.jpg';if(im.getAttribute('src')!==src)im.src=src;a.querySelector('.picture').href=src});document.querySelector('#count').textContent=n+' / 205장 표시'}document.querySelectorAll('select,input').forEach(e=>e.onchange=update);update();</script></html>'''
    (out/'index.html').write_text(body,encoding='utf-8')
    print(json.dumps({k:result[k] for k in ('images','annotations','structural_checks_passed','source_families','capture')},ensure_ascii=False))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('run','dataset','output'):p.add_argument('--'+name,type=Path,required=True)
    args=p.parse_args()
    audit(args.run.resolve(),args.dataset.resolve(),args.output.resolve())
