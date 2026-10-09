#!/usr/bin/env python3
"""Identical-frame raw-label evaluation. Unknown asset subtypes are not negatives."""
import argparse
from collections import Counter
import gc
import hashlib
import html
import json
from pathlib import Path
import time
import yaml

import cv2
import numpy as np
from asmc_camera.detection.yolo_batch_backend import BatchedYoloDetector, COCO7
from asmc_camera.detection.batched_backend import BatchedSegmentationDetector
from asmc_camera.detection.ultralytics_backend import file_sha256
from asmc_camera.detection.hood_mask import HoodMask, load_view_masks
from seg_measurements import iou, match_boxes
from coco7_masks import add_mask_profile,load_masks

ROOT = Path(__file__).resolve().parents[2]
MODELS = ('yolo11m', 'yolo26m', 'yolo26l', 'yolo26m-seg', 'yolo26l-seg', 'yoloe-26m-seg', 'yoloe-26l-seg')
CONFIDENCES = [round(.10+.05*i, 2) for i in range(13)]


def evaluate(gt, predictions, confidence):
    known = [dict(a, group=a['source_class_name']) for a in gt
             if a.get('subtype_verified') and a['source_class_name'] in COCO7]
    unknown = [a for a in gt if a not in known and
               not (a.get('subtype_verified') and a['source_class_name'] in COCO7)]
    predicted = [dict(p, group=p['source_class_name']) for p in predictions
                 if p['confidence'] >= confidence and p['source_class_name'] in COCO7]
    pairs = match_boxes(known, predicted)
    matched = {p for _, p, _ in pairs}
    # Unknown Vehicle/Obstacle may be a target. Keep the predictions in the
    # evidence but do not fabricate a false-positive label on that region.
    ignored = {n for n,p in enumerate(predicted) if n not in matched and
               any(iou(p['xyxy'], a['xyxy']) >= .5 for a in unknown)}
    counts = {label: Counter(gt=sum(a['source_class_name']==label for a in known),
        tp=sum(known[g]['source_class_name']==label for g,_,_ in pairs),
        fp=sum(p['source_class_name']==label and n not in matched|ignored for n,p in enumerate(predicted)))
        for label in COCO7}
    for value in counts.values(): value['fn'] = value['gt']-value['tp']
    return counts, len(ignored), pairs, known, predicted


def summary(counts):
    labels = {}
    for name,c in counts.items():
        labels[name] = dict(c, evaluable=bool(c['gt']),
            f1=2*c['tp']/(2*c['tp']+c['fp']+c['fn']) if c['gt'] else None,
            recall=c['tp']/c['gt'] if c['gt'] else None)
    scores = [v['f1'] for v in labels.values() if v['evaluable']]
    return dict(labels=labels, macro_f1=float(np.mean(scores)) if scores else None,
                false_positives=sum(v['fp'] for v in counts.values()))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--runtime-config', type=Path, help='Explicit prompted YOLOE configuration; one model/width per run')
    p.add_argument('--gt', type=Path, required=True)
    p.add_argument('--datasets', type=Path, required=True)
    p.add_argument('--models', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--split', choices=['selection','heldout'], default='selection')
    p.add_argument('--model', choices=MODELS, nargs='+', default=[m for m in MODELS if not m.startswith('yoloe-')])
    p.add_argument('--width', type=int, nargs='+', default=[640,960,1280])
    p.add_argument('--ego-mask-off', action='store_true', help='Explicit original-input control, including all side GT frames')
    p.add_argument('--frozen-confidence', type=float, choices=CONFIDENCES,
                   help='Evaluate only the selection-frozen confidence; never sweep heldout data')
    p.add_argument('--predict-confidence', type=float, choices=CONFIDENCES, default=.25,
                   help='Actual predictor cutoff; use the frozen value to validate actual selected inference')
    add_mask_profile(p)
    a = p.parse_args(); a.output.mkdir(parents=True, exist_ok=True)
    if a.split=='heldout' and a.frozen_confidence is None:
        p.error('heldout requires --frozen-confidence fixed on selection data')
    confidences=([a.frozen_confidence] if a.frozen_confidence is not None else
                 CONFIDENCES if a.runtime_config else [c for c in CONFIDENCES if c>=.25])
    if a.predict_confidence > min(confidences):
        p.error('predictor cutoff cannot exceed an evaluated threshold')
    display_confidence=a.frozen_confidence if a.frozen_confidence is not None else .4 if a.runtime_config else .7
    manifest = a.gt/'frames.jsonl'
    rows = [json.loads(line) for line in manifest.read_text().splitlines()]
    rows = [r for r in rows if r['split']==a.split and r['annotation_complete']]
    masks=load_masks(a.mask_profile,not a.ego_mask_off)
    all_results = []
    for model in a.model:
        for width in a.width:
            out = a.output/f'{model}-{width}-{a.split}'
            if out.exists(): raise RuntimeError('refusing to overwrite a prior run: '+str(out))
            out.mkdir(); (out/'gallery').mkdir()
            if a.runtime_config:
                cfg=yaml.safe_load(a.runtime_config.read_text())
                if len(a.model)!=1 or len(a.width)!=1 or cfg.get('front_width')!=width or cfg.get('model_id')!=model or not model.startswith('yoloe-'):
                    raise ValueError('explicit YOLOE config requires exactly its model and width')
                cfg['conf_thres']=a.predict_confidence
                detector=BatchedSegmentationDetector(**cfg)
            else:
                if model.startswith('yoloe-'): raise ValueError('YOLOE requires --runtime-config')
                engine = a.models/(model+'.engine')
                detector = BatchedYoloDetector(weights=str(engine), expected_sha256=file_sha256(engine),
                    runtime_backend='tensorrt', task='segment' if 'seg' in model else 'detect', front_width=width, conf_thres=a.predict_confidence)
            counts = {c: {label: Counter() for label in COCO7} for c in confidences}
            mask_ious = {c: [] for c in confidences}
            if 'seg' in model: (out/'masks').mkdir()
            ignored = Counter(); latencies=[]; gallery=[]; actor_history={}; transitions=Counter()
            with (out/'predictions.jsonl').open('w') as handle:
                for n,row in enumerate(rows):
                    source=a.datasets/row['image']
                    if file_sha256(source)!=row['source_sha256']['intensity']:
                        raise RuntimeError('RGB SHA mismatch '+row['image'])
                    image = cv2.imread(str(source))
                    if image is None: raise RuntimeError('missing source image '+row['image'])
                    started=time.monotonic()
                    frame=detector.infer_images({row['view']:masks[row['view']].apply(image)},[row['view']])[row['view']]
                    latencies.append((time.monotonic()-started)*1000)
                    predicted=[dict(source_class_name=d.source_class_name, confidence=d.confidence,
                        xyxy=[d.xmin,d.ymin,d.xmax,d.ymax]) for d in frame.detections]
                    mask_path = None
                    gt_masks = None
                    if frame.masks and any(m is not None for m in frame.masks):
                        mask_path = 'masks/'+str(n)+'.npz'
                        np.savez_compressed(out/mask_path, masks=np.packbits(np.stack(frame.masks),axis=2), width=image.shape[1])
                        if row.get('masks'):
                            with np.load(a.gt/row['masks']) as data:
                                gt_masks=np.unpackbits(data['masks'],axis=2)[:,:,:image.shape[1]].astype(bool)
                    handle.write(json.dumps(dict(run=row['run'], view=row['view'], frame=row['frame'],
                        image=row['image'], source_sha256=row['source_sha256'], predictions=predicted, masks=mask_path, timings=frame.timings))+'\n')
                    for confidence in confidences:
                        measured, excluded, pairs, known, ps = evaluate(row['annotations'], predicted, confidence)
                        for label in COCO7: counts[confidence][label].update(measured[label])
                        ignored[confidence] += excluded
                        if gt_masks is not None:
                            kept_indices=[i for i,d in enumerate(predicted) if d['confidence']>=confidence and d['source_class_name'] in COCO7]
                            for g,pr,_ in pairs:
                                original=next(i for i,ann in enumerate(row['annotations']) if ann['actor_id']==known[g]['actor_id'])
                                gm,pm=gt_masks[original],frame.masks[kept_indices[pr]]
                                union=np.count_nonzero(gm|pm)
                                mask_ious[confidence].append(float(np.count_nonzero(gm&pm)/union) if union else 0.)
                        if confidence == display_confidence:
                            present = {known[g]['actor_id'] for g,_,_ in pairs}
                            for gt in known:
                                key=(row['run'],row['view'],gt['actor_id'])
                                old=actor_history.get(key); hit=gt['actor_id'] in present
                                if old and old[0]+1==int(row['frame']) and old[1] and not hit:
                                    transitions[gt['source_class_name']] += 1
                                actor_history[key]=(int(row['frame']),hit)
                    if n % max(1,len(rows)//50) == 0:
                        rendered=image.copy()
                        mask,_=masks[row['view']].geometry(*image.shape[:2])
                        rendered[mask]=(rendered[mask]*.7+np.array([0,0,255])*.3).astype(np.uint8)
                        for gt in row['annotations']:
                            x1,y1,x2,y2=map(int,gt['xyxy']);cv2.rectangle(rendered,(x1,y1),(x2,y2),(255,120,0),1)
                        for d,bitmap in zip(frame.detections,frame.masks):
                            if d.confidence<display_confidence:continue
                            if bitmap is not None:rendered[bitmap]=(rendered[bitmap]*.7+np.array([0,255,0])*.3).astype(np.uint8)
                            x1,y1,x2,y2=map(int,[d.xmin,d.ymin,d.xmax,d.ymax]);cv2.rectangle(rendered,(x1,y1),(x2,y2),(0,255,0),1)
                            cv2.putText(rendered,d.source_class_name+' %.2f'%d.confidence,(max(0,x1),max(14,y1)),cv2.FONT_HERSHEY_SIMPLEX,.4,(0,255,0),1)
                        name=f'{n:05}.jpg';cv2.imwrite(str(out/'gallery'/name),rendered)
                        gallery.append(f'<figure><figcaption>{html.escape(row["image"])}</figcaption><img loading="lazy" width="900" src="{name}"></figure>')
                    if (n+1)%100==0: print(model,width,n+1,len(rows),flush=True)
            thresholds=[dict(confidence=c,**summary(counts[c]),ignored_unknown_subtype=ignored[c], mask_iou=float(np.mean(mask_ious[c])) if mask_ious[c] else None, mask_pairs=len(mask_ious[c])) for c in confidences]
            report=dict(model=detector.info,width=width,split=a.split,frames=len(rows),thresholds=thresholds,
                runtime_config_sha256=file_sha256(a.runtime_config) if a.runtime_config else None,
                code_sha256={str(f.relative_to(ROOT)):file_sha256(f) for f in
                    [Path(__file__),ROOT/'tools/perception/coco7_masks.py']+
                    sorted((ROOT/'src/asmc_camera/src/asmc_camera/detection').glob('*.py'))},
                gt_manifest_sha256=file_sha256(manifest),masks={v:m.info for v,m in masks.items()},
                metric_scope='verified raw COCO7 labels only; other prompts retained as observations, not scored as background',
                latency_ms={name:float(np.percentile(latencies,q)) for name,q in [('p50',50),('p95',95),('p99',99)]},
                matched_to_missed_transitions=dict(transitions),
                frozen_confidence=a.frozen_confidence,mask_profile=a.mask_profile,
                missing_gt_classes=[c for c in COCO7 if not counts[display_confidence][c]['gt']],
                interpretation='known subtype GT only; unknown-region predictions ignored, not negatives; actor captures are inspection only')
            (out/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
            (out/'gallery/index.html').write_text('<!doctype html><meta charset="utf-8"><h1>Raw COCO7 · blue GT / green output at '+str(display_confidence)+' / red Ego mask</h1>'+''.join(gallery))
            all_results.append(report);(a.output/'index.json').write_text(json.dumps(all_results,indent=2)+'\n')
            del detector;gc.collect()
            import torch
            torch.cuda.empty_cache()


if __name__=='__main__': main()
