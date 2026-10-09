#!/usr/bin/env python3
"""Compare actual frozen predictor cutoffs and show every FP plus recovered GT."""
import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import html
import json
from pathlib import Path

import cv2
from analyze_coco7_broad_selection import score, selected_frames
from analyze_coco7_weather import load_jsonl
from evaluate_coco7_models import evaluate, COCO7
from seg_measurements import iou


def observations(gt, predictions, cut):
    matched_gt, fps = {}, []
    for row in predictions:
        key = row['image']
        _, _, pairs, known, boxes = evaluate(gt[key]['annotations'], row['predictions'], cut)
        matched = {p for _, p, _ in pairs}
        for g, p, overlap in pairs:
            # Index in the full annotation array is stable across all cutoffs.
            known_indices = [n for n, ann in enumerate(gt[key]['annotations']) if ann.get('subtype_verified') and ann['source_class_name'] in COCO7]
            identity = (key, known_indices[g])
            matched_gt[identity] = dict(image=key, box=boxes[p], gt=known[g], iou=overlap)
        unknown = [a for a in gt[key]['annotations'] if not
                   (a.get('subtype_verified') and a['source_class_name'] in COCO7)]
        for n, box in enumerate(boxes):
            if n in matched or any(iou(box['xyxy'], a['xyxy']) >= .5 for a in unknown):
                continue
            overlaps = [a for a in known if iou(box['xyxy'], a['xyxy']) >= .5]
            kind = 'wrong_label_overlap' if any(a['source_class_name'] != box['source_class_name'] for a in overlaps) else (
                'duplicate_overlap' if overlaps else 'unmatched_box')
            fps.append(dict(image=key, box=box, kind=kind,
                            overlapping_gt_labels=[a['source_class_name'] for a in overlaps]))
    return matched_gt, fps


def same_fp(a, b):
    return a['image'] == b['image'] and a['box']['source_class_name'] == b['box']['source_class_name'] and iou(a['box']['xyxy'], b['box']['xyxy']) >= .95


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root', type=Path, required=True)
    p.add_argument('--datasets', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=False)
    manifest = a.root/'gt-v2/frames.jsonl'
    rows = [r for r in load_jsonl(manifest) if r['split']=='selection' and r['annotation_complete']]
    conditions_path = a.root/'broad-model-conditions-v1.json'
    exclusions = json.loads(conditions_path.read_text())['reviewed_exclusions']
    paths = {c: a.root/('confidence-followup-actual%02d-v1'%round(c*100))/'yolo26l-1280-selection' for c in (.3,.4)}
    paths[.5] = a.root/'broad-model-selected-selection50-v1/yolo26l-1280-selection'
    report = dict(created_utc=datetime.now(timezone.utc).isoformat(), scope='Actual predictor cutoffs; original metrics retained; exact two reviewed frames excluded only in supplemental metrics.',
                  original_frames=len(rows), exclusions=exclusions, source_sha256={}, thresholds=[], gallery=[])
    metrics, observations_by_cut, signature = {}, {}, None
    for cut, directory in sorted(paths.items()):
        summary = json.loads((directory/'summary.json').read_text())
        if summary['model']['conf_thres'] != cut or summary['frozen_confidence'] != cut:
            raise ValueError('requires actual frozen predictor cutoff')
        current_signature = dict(engine_sha256=summary['model']['weights_sha256'],
            backend=summary['model']['backend'], task=summary['model']['task'], width=summary['width'],
            split=summary['split'], masks=summary['masks'],
            ultralytics=summary['model']['ultralytics'], tensorrt=summary['model']['tensorrt'])
        if signature is not None and signature != current_signature:
            raise ValueError('model, input geometry, runtime or mask differs between cutoffs')
        signature = current_signature
        report['comparison_signature'] = signature
        if summary['gt_manifest_sha256'] != hashlib.sha256(manifest.read_bytes()).hexdigest():
            raise ValueError('different manifest')
        predictions = load_jsonl(directory/'predictions.jsonl')
        gt, included = selected_frames(rows, predictions, exclusions)
        metrics[cut] = score(gt, included, cut)
        observations_by_cut[cut] = observations(gt, included, cut)
        matched, fps = observations_by_cut[cut]
        pooled = metrics[cut]['pooled']
        if len(matched) != pooled['total_tp'] or len(fps) != pooled['false_positives']:
            raise ValueError('gallery accounting mismatch')
        report['thresholds'].append(dict(confidence=cut, original=summary['thresholds'][0],
            reviewed=metrics[cut], fp_kinds=dict(Counter(f['kind'] for f in fps))))
        for f in (directory/'summary.json', directory/'predictions.jsonl'):
            report['source_sha256'][str(f)] = hashlib.sha256(f.read_bytes()).hexdigest()
    baseline_gt, baseline_fp = observations_by_cut[.5]
    cards = []
    for entry in report['thresholds']:
        cut = entry['confidence']
        matched, fps = observations_by_cut[cut]
        entry['gained_gt_vs_050'] = len(set(matched)-set(baseline_gt))
        entry['lost_gt_vs_050'] = len(set(baseline_gt)-set(matched))
        entry['added_fp_vs_050'] = sum(not any(same_fp(f,b) for b in baseline_fp) for f in fps)
        for f in fps:
            f.update(cut=cut, category='FP', added_vs_050=not any(same_fp(f,b) for b in baseline_fp))
        recovered = [dict(matched[k], cut=cut, category='recovered_TP', kind='new_correct_gt', added_vs_050=True)
                     for k in sorted(set(matched)-set(baseline_gt))]
        for item in fps+recovered:
            image_path = a.datasets/item['image']
            if hashlib.sha256(image_path.read_bytes()).hexdigest()!=gt[item['image']]['source_sha256']['intensity']:
                raise ValueError('RGB changed')
            image = cv2.imread(str(image_path))
            if image is None: raise ValueError('RGB decode failed')
            for ann in gt[item['image']]['annotations']:
                xy = list(map(int, ann['xyxy']))
                color = (255,120,20) if ann.get('subtype_verified') else (0,165,255)
                cv2.rectangle(image, tuple(xy[:2]), tuple(xy[2:]), color, 1)
                cv2.putText(image,ann['source_class_name'],(xy[0],max(12,xy[1]-3)),cv2.FONT_HERSHEY_SIMPLEX,.4,color,1)
            box = item['box']; xy = list(map(int,box['xyxy']))
            cv2.rectangle(image,tuple(xy[:2]),tuple(xy[2:]),(0,255,0),2)
            caption='%s %.3f | cutoff %.2f | %s'%(box['source_class_name'],box['confidence'],cut,item['category'])
            cv2.putText(image,caption,(8,20),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,255,0),1)
            filename='%04d.jpg'%len(report['gallery'])
            if not cv2.imwrite(str(a.output/filename),image): raise ValueError('image write failed')
            item['rendered']=filename
            report['gallery'].append(item)
            title='%s · %s · %s'%(caption,item['kind'],item['image'])
            cards.append('<figure data-cut="%.2f" data-kind="%s" data-added="%s" data-category="%s"><figcaption>%s</figcaption><a href="%s" target="_blank"><img loading="lazy" src="%s"></a></figure>'%(cut,item['kind'],int(item['added_vs_050']),item['category'],html.escape(title),filename,filename))
    for f in (manifest,conditions_path): report['source_sha256'][str(f)]=hashlib.sha256(f.read_bytes()).hexdigest()
    table='<table><tr><th>Cut</th><th>TP</th><th>FP</th><th>macro F1</th><th>Added correct GT / lost GT vs .50</th><th>Wrong label / unmatched</th></tr>'
    for t in report['thresholds']:
        s=t['reviewed']['pooled']; kinds=t['fp_kinds']
        table+='<tr><td>%.2f</td><td>%d</td><td>%d</td><td>%.4f</td><td>%d / %d</td><td>%d / %d</td></tr>'%(t['confidence'],s['total_tp'],s['false_positives'],s['macro_f1'],t['gained_gt_vs_050'],t['lost_gt_vs_050'],kinds.get('wrong_label_overlap',0),kinds.get('unmatched_box',0))
    document='''<!doctype html><html lang="ko"><meta charset="utf-8"><title>Confidence 0.30/0.40/0.50 실제 추론 비교</title>
<style>body{font:16px system-ui;max-width:1450px;margin:24px auto;padding:12px}table{border-collapse:collapse}td,th{border:1px solid #bbb;padding:10px}figure{border:1px solid #bbb;padding:12px;margin:16px 0}img{max-width:100%}figcaption{overflow-wrap:anywhere;padding:8px}nav{position:sticky;top:0;background:white;padding:12px}select{font:inherit;margin:6px}</style>
<a href="/">갤러리 목차</a><h1>Confidence 0.30 / 0.40 / 0.50 실제 추론 비교</h1>
<p>YOLO26l detect · front1280 · 동일 Capture selection · mask ON. 실제 predictor cutoff별 재추론. 표는 검토된 정확히 2장 제외 2250장; 원본 2252장 점수는 report.json에 보존.</p>
<p>초록: 해당 검출. 파랑: 검수 GT. 주황: 종류 미확인 GT. 다른 차량 종류로 잘못 검출한 것도 FP다. unmatched는 GT 대응이 없다는 뜻이며 사람이 검토하기 전 배경 오검출로 확정하지 않는다. 프레임별 박스 수이며 고유 객체 수가 아니다.</p>TABLE
<nav><label>Cut <select id="cut"><option value="0.40">0.40</option><option value="0.30">0.30</option><option value="0.50">0.50</option><option value="">전체</option></select></label>
<label>종류 <select id="category"><option value="FP">오검출</option><option value="recovered_TP">0.50 대비 회복한 정답</option><option value="">전체</option></select></label>
<label>형태 <select id="kind"><option value="">전체</option><option value="unmatched_box">GT 미대응</option><option value="wrong_label_overlap">다른 객체 라벨과 겹침</option><option value="duplicate_overlap">중복</option></select></label>
<label><input id="added" type="checkbox" checked>0.50 대비 추가된 사례만</label><span id="count"></span></nav>CARDS
<script>function draw(){let count=0;document.querySelectorAll('figure').forEach(f=>{const show=['cut','kind','category'].every(k=>!document.getElementById(k).value||f.dataset[k]===document.getElementById(k).value)&&(!document.getElementById('added').checked||f.dataset.added==='1');f.hidden=!show;count+=show;});document.getElementById('count').textContent=count+'장';}document.querySelectorAll('select,input').forEach(e=>e.onchange=draw);draw();</script></html>'''
    (a.output/'index.html').write_text(document.replace('TABLE',table+'</table>').replace('CARDS',''.join(cards)))
    (a.output/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps([dict(cut=t['confidence'],tp=t['reviewed']['pooled']['total_tp'],fp=t['reviewed']['pooled']['false_positives'],f1=t['reviewed']['pooled']['macro_f1'],fp_kinds=t['fp_kinds'],gained=t['gained_gt_vs_050'],lost=t['lost_gt_vs_050']) for t in report['thresholds']]))


if __name__=='__main__': main()
