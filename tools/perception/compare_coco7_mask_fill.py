#!/usr/bin/env python3
"""Selection-only fill ablation; geometry unchanged, no operating config promotion."""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np
from asmc_camera.detection.hood_mask import HoodMask,load_view_masks
from asmc_camera.detection.ultralytics_backend import file_sha256
from asmc_camera.detection.yolo_batch_backend import BatchedYoloDetector
from evaluate_coco7_models import evaluate

ROOT=Path(__file__).resolve().parents[2]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--gt',type=Path,required=True)
    p.add_argument('--datasets',type=Path,required=True)
    p.add_argument('--models',type=Path,required=True)
    p.add_argument('--failures',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--erosion-px',type=int,default=0,help='Diagnostic only: shrink exclusion inward; efficacy must also be checked')
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    if not 0<=a.erosion_px<=16: p.error('erosion must be in [0,16]')
    gt={r['image']:r for r in map(json.loads,a.gt.read_text().splitlines()) if r['split']=='selection' and r['annotation_complete']}
    failures=json.loads(a.failures.read_text())
    names=[r['image'] for r in failures['differences'] if r['new_missed_actor_ids']]
    # Keep ordinary/own-body scenes in the same diagnostic; a fill that restores
    # a lost object may also make the Ego body look like another car.
    for view in ('front','left','right'):
        candidates=[r['image'] for r in gt.values() if r['view']==view]
        names+=candidates[::max(1,len(candidates)//5)][:5]
    names=list(dict.fromkeys(names))
    masks=load_view_masks(['front','left','right'],HoodMask.load(True,ROOT/'src/asmc_camera/config/camera/yolo_hood_mask_v1.yaml'),True,
        {v:ROOT/f'src/asmc_camera/config/camera/yolo_ego_mask_{v}_v1.yaml' for v in ('left','right')})
    engine=a.models/'yolo26m-seg.engine'
    detector=BatchedYoloDetector(weights=str(engine),expected_sha256=file_sha256(engine),runtime_backend='tensorrt',
        task='segment',front_width=960,conf_thres=.25)
    records=[];page=['<!doctype html><meta charset="utf-8"><h1>Selection mask fill ablation</h1><p>Same exclusion polygon. Actual modified inference input below; blue GT, green predictions at0.65. No heldout used for fill tuning.</p>']
    for index,name in enumerate(names):
        row=gt[name];source=a.datasets/name
        if file_sha256(source)!=row['source_sha256']['intensity']:raise RuntimeError('RGB SHA mismatch')
        image=cv2.imread(str(source));view=row['view'];bitmap,_=masks[view].geometry(*image.shape[:2])
        if a.erosion_px:
            bitmap=cv2.erode(bitmap.astype(np.uint8),np.ones((2*a.erosion_px+1,2*a.erosion_px+1),np.uint8)).astype(bool)
        for fill in ('OFF',0,32,64,96,128,160,192,224,'median','mean'):
            if fill=='OFF': modified=image;value=None
            else:
                value=(np.median(image[bitmap],axis=0).astype(np.uint8) if fill=='median' else
                       np.mean(image[bitmap],axis=0).astype(np.uint8) if fill=='mean' else np.full(3,fill,np.uint8))
                modified=image.copy();modified[bitmap]=value
            result=detector.infer_images({view:modified},[view])[view]
            boxes=[dict(source_class_name=d.source_class_name,confidence=d.confidence,
                xyxy=[d.xmin,d.ymin,d.xmax,d.ymax]) for d in result.detections]
            _,_,pairs,known,_=evaluate(row['annotations'],boxes,.65)
            hits=sorted(known[g]['actor_id'] for g,_,_ in pairs)
            records.append(dict(image=name,view=view,fill=fill,fill_bgr=value.tolist() if value is not None else None,
                source_sha256=row['source_sha256'],matched_actor_ids=hits,predictions=boxes))
            render=modified.copy()
            for box,color in [(b,(255,120,0)) for b in known]+[(b,(0,255,0)) for b in boxes if b['confidence']>=.65]:
                x,y,r,b=map(int,box['xyxy']);cv2.rectangle(render,(x,y),(r,b),color,1)
                cv2.putText(render,box['source_class_name']+(f" {box['confidence']:.3f}" if 'confidence' in box else ' GT'),
                    (max(0,x),max(14,y)),cv2.FONT_HERSHEY_SIMPLEX,.4,color,1)
            file=f'{index:03}-{fill}.jpg';cv2.imwrite(str(a.output/file),render)
            page.append(f'<figure><figcaption>{name} fill {fill} matched {hits}</figcaption><img loading="lazy" width="900" src="{file}"></figure>')
        print(name,'complete',flush=True)
    (a.output/'report.json').write_text(json.dumps(dict(model=detector.info,masks={v:m.info for v,m in masks.items()},erosion_px=a.erosion_px,
        records=records,scope='Selection diagnostic only; restores must pass full ON/OFF and Ego false-positive review before promotion'),indent=2)+'\n')
    (a.output/'index.html').write_text(''.join(page))


if __name__=='__main__':main()
