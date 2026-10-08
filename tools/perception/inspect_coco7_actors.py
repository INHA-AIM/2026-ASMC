#!/usr/bin/env python3
"""Identical controlled RGB captures; inspection only, never actor-state pixel GT."""
import argparse
import gc
import hashlib
import html
import json
from pathlib import Path

import cv2
import numpy as np
from asmc_camera.detection.hood_mask import HoodMask, load_view_masks
from asmc_camera.detection.ultralytics_backend import file_sha256
from asmc_camera.detection.yolo_batch_backend import BatchedYoloDetector

ROOT = Path(__file__).resolve().parents[2]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--captures', type=Path, nargs='+', required=True)
    p.add_argument('--models', type=Path, required=True)
    p.add_argument('--candidates', type=Path, required=True,
                   help='JSON list of model/width/confidence chosen on selection GT')
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args(); a.output.mkdir(parents=True, exist_ok=False)
    masks = load_view_masks(['front', 'left', 'right'],
        HoodMask.load(True, ROOT/'src/asmc_camera/config/camera/yolo_hood_mask_v1.yaml'), True,
        {v: ROOT/f'src/asmc_camera/config/camera/yolo_ego_mask_{v}_v1.yaml' for v in ['left', 'right']})
    fixtures = []
    rejected = []
    for capture in a.captures:
        registry = json.loads((capture/'capture.json').read_text())
        for case in registry['cases']:
            # Preserve floating/spawn-failed fixtures in the registry and index,
            # but do not turn them into performance evidence for ground actors.
            if case.get('error') or not case.get('state') or (
                    abs(case.get('ground_bottom_error_m', 0)) > .2):
                rejected.append(dict(registry=str(capture), index=case['index'],
                                     reason=case.get('error', 'ground placement failed')))
                continue
            view = ('left' if str(case.get('placement', '')).startswith('left') else
                    'right' if str(case.get('placement', '')).startswith('right') else 'front')
            frames = sorted([r for r in case['frames'] if r['view'] == view],
                            key=lambda r: r['source_stamp_ns'])
            if frames:
                fixtures.append((capture, case, frames))
    report = dict(scope='qualitative identical-image comparison; no pixel GT, recall, or actor association claimed',
                  rejected_fixtures=rejected, candidates=[], masks={v: m.info for v,m in masks.items()})
    page = ['<!doctype html><meta charset="utf-8"><style>body{font:16px system-ui;margin:24px}img{max-width:95%}</style>',
            '<h1>MORAI controlled captures · raw COCO7</h1>',
            '<p>These are reviewed images, not pixel ground truth. A detection of the expected class may belong to another actor. Red is the Ego exclusion area.</p>']
    for config in json.loads(a.candidates.read_text()):
        name=f'{config["model"]}-{config["width"]}'
        out=a.output/name;out.mkdir()
        weight=a.models/(config['model']+'.engine')
        detector=BatchedYoloDetector(weights=str(weight),expected_sha256=file_sha256(weight),
            runtime_backend='tensorrt', task='segment' if 'seg' in config['model'] else 'detect',
            front_width=config['width'],conf_thres=config['confidence'])
        page.append(f'<h2>{html.escape(name)} · conf {config["confidence"]}</h2>')
        results=[]
        with (out/'predictions.jsonl').open('w') as handle:
            for capture,case,frames in fixtures:
                records=[]
                for n,row in enumerate(frames):
                    image_path=capture/row['image']
                    if file_sha256(image_path)!=row['sha256']:
                        raise RuntimeError('source SHA mismatch '+str(image_path))
                    image=cv2.imread(str(image_path));view=row['view']
                    result=detector.infer_images({view:masks[view].apply(image)},[view])[view]
                    boxes=[dict(label=d.source_class_name,confidence=d.confidence,
                                xyxy=[d.xmin,d.ymin,d.xmax,d.ymax]) for d in result.detections]
                    record=dict(registry=str(capture),case=case['index'],asset=case['asset'],
                        placement=case.get('placement'),**row,predictions=boxes)
                    handle.write(json.dumps(record)+'\n');records.append(record)
                    if n in {0,len(frames)//2,len(frames)-1}:
                        render=image.copy();ego,_=masks[view].geometry(*image.shape[:2])
                        render[ego]=(render[ego]*.7+np.array([0,0,255])*.3).astype(np.uint8)
                        for detection,mask in zip(result.detections,result.masks):
                            if mask is not None:
                                render[mask]=(render[mask]*.75+np.array([0,255,0])*.25).astype(np.uint8)
                            xy=list(map(int,[detection.xmin,detection.ymin,detection.xmax,detection.ymax]))
                            cv2.rectangle(render,tuple(xy[:2]),tuple(xy[2:]),(0,255,0),1)
                            cv2.putText(render,f'{detection.source_class_name} {detection.confidence:.2f}',
                                (max(0,xy[0]),max(14,xy[1])),cv2.FONT_HERSHEY_SIMPLEX,.45,(0,255,0),1)
                        filename=f'{capture.name}-{case["index"]:03}-{n:02}.jpg'
                        cv2.imwrite(str(out/filename),render)
                        page.append(f'<figure><figcaption>{html.escape(case["asset"]["api_name"])} / {html.escape(str(case.get("placement")))} / {view} / {n}</figcaption><img loading="lazy" src="{name}/{filename}"></figure>')
                results.append(dict(registry=str(capture),case=case['index'],asset=case['asset'],
                    frames=len(records),interpretation='inspect raw predictions; no quantitative hit assignment'))
        report['candidates'].append(dict(config=config,backend=detector.info,fixtures=results))
        (a.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
        del detector;gc.collect()
        import torch
        torch.cuda.empty_cache()
    (a.output/'index.html').write_text(''.join(page))


if __name__ == '__main__':
    main()
