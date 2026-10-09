#!/usr/bin/env python3
"""Render all selection-GT false positives at a frozen candidate threshold."""
import argparse
from collections import Counter
import hashlib
import html
import json
from pathlib import Path

import cv2
from analyze_coco7_weather import load_jsonl
from evaluate_coco7_models import evaluate, COCO7
from seg_measurements import iou


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--gt', type=Path, required=True)
    p.add_argument('--predictions', type=Path, required=True)
    p.add_argument('--datasets', type=Path, required=True)
    p.add_argument('--confidence', type=float, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=False)
    gt = {r['image']: r for r in load_jsonl(a.gt/'frames.jsonl') if r['split'] == 'selection' and r['annotation_complete']}
    report = dict(confidence=a.confidence, scope='All class-aware box-GT FP; cross-label overlaps distinguished, not excused.', false_positives=[], total_tp=0)
    page = ['<!doctype html><meta charset="utf-8"><style>body{font:16px system-ui}img{max-width:95%}</style>',
            '<h1>Confidence false-positive review</h1><p>Green: returned box. Blue: verified GT. Orange: unknown subtype. '
            'Class-confusion overlaps remain FP in the raw-label metric.</p>']
    seen = set()
    for row in load_jsonl(a.predictions):
        key = row['image']
        if key in seen or key not in gt or row['source_sha256']['intensity'] != gt[key]['source_sha256']['intensity']:
            raise ValueError('duplicate, foreign, or different RGB')
        seen.add(key)
        counts, _, pairs, known, predictions = evaluate(gt[key]['annotations'], row['predictions'], a.confidence)
        report['total_tp'] += sum(v['tp'] for v in counts.values())
        matched = {pr for _, pr, _ in pairs}
        unknown = [ann for ann in gt[key]['annotations'] if not (
            ann.get('subtype_verified') and ann['source_class_name'] in COCO7)]
        expected_fp = sum(v['fp'] for v in counts.values())
        frame_fp = 0
        for n, box in enumerate(predictions):
            if n in matched or any(iou(box['xyxy'], ann['xyxy']) >= .5 for ann in unknown):
                continue
            frame_fp += 1
            overlaps = [ann for ann in known if iou(box['xyxy'], ann['xyxy']) >= .5]
            kind = 'overlapping_verified_object' if overlaps else 'unmatched_box'
            image = cv2.imread(str(a.datasets/key))
            if image is None or hashlib.sha256((a.datasets/key).read_bytes()).hexdigest() != gt[key]['source_sha256']['intensity']:
                raise ValueError('missing or modified source RGB')
            for ann in gt[key]['annotations']:
                xy = list(map(int, ann['xyxy']))
                color = (255, 120, 20) if ann.get('subtype_verified') else (0, 165, 255)
                cv2.rectangle(image, tuple(xy[:2]), tuple(xy[2:]), color, 2)
                cv2.putText(image, ann['source_class_name'], (xy[0], max(12, xy[1]-4)), cv2.FONT_HERSHEY_SIMPLEX, .4, color, 1)
            xy = list(map(int, box['xyxy']))
            cv2.rectangle(image, tuple(xy[:2]), tuple(xy[2:]), (0, 255, 0), 2)
            label = '%s %.3f' % (box['source_class_name'], box['confidence'])
            cv2.putText(image, label, (max(0, xy[0]), max(15, xy[1]-16)), cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 255, 0), 1)
            index = len(report['false_positives'])
            filename = '%03d.jpg' % index
            cv2.imwrite(str(a.output/filename), image)
            report['false_positives'].append(dict(image=key, box=box, kind=kind,
                overlapping_gt_labels=[ann['source_class_name'] for ann in overlaps], rendered=filename))
            page.append('<figure><figcaption>%s · %s · %s</figcaption><img loading="lazy" src="%s"></figure>' %
                        (html.escape(key), html.escape(label), kind, filename))
        if frame_fp != expected_fp:
            raise ValueError('rendered FP count differs from evaluator')
    if seen != set(gt):
        raise ValueError('incomplete selection coverage')
    report['fp_kinds'] = dict(Counter(r['kind'] for r in report['false_positives']))
    (a.output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    (a.output/'index.html').write_text(''.join(page))


if __name__ == '__main__': main()
