#!/usr/bin/env python3
"""Review saved confidence sweeps; never rerun or invent unlabelled obstacle GT."""
import argparse
import hashlib
import html
import json
from pathlib import Path
import cv2
from analyze_coco7_broad_selection import score, selected_frames
from analyze_coco7_confidence_followup import observations
from analyze_coco7_weather import load_jsonl


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--reference', type=Path, required=True)
    p.add_argument('--predictions', type=Path, required=True)
    p.add_argument('--datasets', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=False)
    manifest = a.reference/'gt-v2/frames.jsonl'
    rows = [r for r in load_jsonl(manifest) if r['split']=='selection' and r['annotation_complete']]
    exclusions = json.loads((a.reference/'broad-model-conditions-v1.json').read_text())['reviewed_exclusions']
    gt, predictions = selected_frames(rows, load_jsonl(a.predictions), exclusions)
    report = dict(scope='Saved predictor 0.10 results filtered at each cutoff, not actual cutoff reruns. COCO7 verified subtypes only.',
                  previously_reviewed_exclusions=exclusions,
                  source_sha256={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in (manifest,a.predictions)},
                  metrics=[], cases=[])
    reference, _ = observations(gt, predictions, .4)
    page = ['<!doctype html><meta charset="utf-8"><title>YOLOE confidence 검토</title>',
            '<style>body{font:16px system-ui;margin:24px}img{max-width:100%}figure{border:1px solid #aaa;padding:8px}table{border-collapse:collapse}td{padding:8px;border:1px solid #aaa}</style>',
            '<a href="/">갤러리 목차</a><h1>YOLOE L1280 · confidence 0.20 / 0.40</h1>',
            '<p>동일 predictor 0.10 원본의 threshold 대조입니다. 실제 선정 cutoff 재추론은 별도로 확인합니다. 초록: 검출, 파랑: 확인된 GT, 주황: 종류 미확인 GT. GT 미대응을 육안 검토 없이 배경 오검출로 확정하지 않습니다.</p>']
    for cut in (.2,.4):
        matched, fps = observations(gt, predictions, cut)
        metrics = score(gt,predictions,cut)
        report['metrics'].append(dict(confidence=cut,score=metrics))
        page.append('<h2>Cut %.2f · TP %d / FP %d / macro F1 %.4f</h2>' %
                    (cut,metrics['pooled']['total_tp'],metrics['pooled']['false_positives'],metrics['pooled']['macro_f1']))
        recovered = [dict(matched[k],kind='recovered_TP_vs_040') for k in sorted(set(matched)-set(reference))]
        for item in fps+recovered:
            row=gt[item['image']]; source=a.datasets/item['image']
            if hashlib.sha256(source.read_bytes()).hexdigest()!=row['source_sha256']['intensity']:
                raise ValueError('source RGB mismatch')
            image=cv2.imread(str(source))
            if image is None:raise ValueError('cannot decode source')
            for ann in row['annotations']:
                xy=list(map(int,ann['xyxy']));color=(255,120,20) if ann.get('subtype_verified') else (0,165,255)
                cv2.rectangle(image,tuple(xy[:2]),tuple(xy[2:]),color,1)
            box=item['box'];xy=list(map(int,box['xyxy']))
            cv2.rectangle(image,tuple(xy[:2]),tuple(xy[2:]),(0,255,0),2)
            caption=f"{box['source_class_name']} {box['confidence']:.3f} / cut {cut:.2f} / {item['kind']}"
            cv2.putText(image,caption,(8,20),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,255,0),1)
            name=f"{len(report['cases']):04}.jpg"
            if not cv2.imwrite(str(a.output/name),image):raise ValueError('image write failed')
            report['cases'].append(dict(item,confidence_cutoff=cut,rendered=name))
            page.append(f'<figure><figcaption>{html.escape(caption)} · {html.escape(item["image"])}</figcaption><a href="{name}"><img loading="lazy" src="{name}"></a></figure>')
    (a.output/'index.html').write_text(''.join(page))
    (a.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')


if __name__=='__main__':main()
