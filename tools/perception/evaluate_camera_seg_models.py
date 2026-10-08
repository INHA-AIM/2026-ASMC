#!/usr/bin/env python3
"""Infer selection split only, save original outputs and evaluate fixed/swept conf."""
import argparse
from collections import Counter
import hashlib
import html
import json
from pathlib import Path
import time

import cv2
import numpy as np
import yaml
from asmc_camera.detection.classes import load_class_groups
from asmc_camera.detection.hood_mask import HoodMask, load_view_masks
from asmc_camera.detection.segmentation import SegmentationDetector
from asmc_camera.detection.ultralytics_backend import UltralyticsDetector
from seg_measurements import GROUPS, INFRASTRUCTURE, frame_metrics, summarize, choose_threshold


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--gt',type=Path,required=True)
    p.add_argument('--datasets',type=Path,required=True)
    p.add_argument('--config',type=Path,required=True)
    p.add_argument('--groups',type=Path,required=True)
    p.add_argument('--hood-config',type=Path,required=True)
    p.add_argument('--side-ego-mask',action='store_true',help='apply separate left/right Ego silhouettes')
    p.add_argument('--left-ego-mask-config',type=Path)
    p.add_argument('--right-ego-mask-config',type=Path)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--baseline',action='store_true')
    p.add_argument('--batched',action='store_true',help='YOLOE-l native view packing; never substitute old 640px inference')
    args=p.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    cfg=yaml.safe_load(args.config.read_text())
    hood=HoodMask.load(True,args.hood_config)
    view_masks=load_view_masks(['front','left','right'],hood,args.side_ego_mask,dict(
        left=args.left_ego_mask_config or args.hood_config.parent/'yolo_ego_mask_left_v1.yaml',
        right=args.right_ego_mask_config or args.hood_config.parent/'yolo_ego_mask_right_v1.yaml'))
    groups=load_class_groups(yaml.safe_load(args.groups.read_text())['class_groups'])
    weights=cfg['weights'].replace('$(optenv ASMC_SEG_MODELS /data/models/camera_seg_v1)','/data/models/camera_seg_v1').replace('$(find asmc_camera)','/root/ws/src/asmc_camera')
    prompt=cfg.get('prompt_embeddings','').replace('$(optenv ASMC_SEG_MODELS /data/models/camera_seg_v1)','/data/models/camera_seg_v1')
    kwargs=dict(weights=weights,conf_thres=.10,allowed_names=cfg['allowed_names'],
                expected_sha256=cfg['expected_sha256'],predict_overrides=cfg.get('predict_overrides',{}))
    if not args.baseline:
        kwargs.update(model_kind=cfg.get('model_kind','yolo'),prompt_embeddings=prompt)
    if args.batched:
        from asmc_camera.detection.batched_backend import BatchedSegmentationDetector
        kwargs.update(prompt_embeddings=cfg['prompt_embeddings'], runtime_backend=cfg['runtime_backend'],batch_mode=cfg['batch_mode'],
                      source_checkpoint_sha256=cfg.get('source_checkpoint_sha256',''),expected_prompt_sha256=cfg.get('expected_prompt_sha256',''))
        detector=BatchedSegmentationDetector(**kwargs)
    else:
        detector=(UltralyticsDetector if args.baseline else SegmentationDetector)(**kwargs)
    rows=[json.loads(line) for line in (args.gt/'frames.jsonl').read_text().splitlines()]
    selected=[r for r in rows if r['split']=='selection' and r['annotation_complete']]
    sweep=[round(.10+.05*i,2) for i in range(13)]
    sums={c:dict(counts={g:Counter() for g in GROUPS},masks=[],names=Counter()) for c in sweep}
    all_latency=[]; labels=Counter(); gallery=[]; continuity={}; gaps=Counter(); total_gt=Counter()
    (args.output/'masks').mkdir()
    (args.output/'gallery').mkdir()
    with (args.output/'predictions.jsonl').open('w') as out:
        for index,row in enumerate(selected):
            image=cv2.imread(str(args.datasets/row['image']))
            if image is None:raise ValueError('RGB decode failed: '+row['image'])
            image_input=view_masks[row['view']].apply(image)
            started=time.monotonic()
            if args.baseline:
                detections,timings=detector.detect(image_input); masks=[None]*len(detections)
            elif args.batched:
                result=detector.infer_images({row['view']:image_input},[row['view']])[row['view']]
                detections,masks,timings=result.detections,result.masks,result.timings
            else:
                result=detector.predict_frame(image_input); detections,masks,timings=result.detections,result.masks,result.timings
            all_latency.append((time.monotonic()-started)*1000)
            predicted=[dict(source_class_name=d.source_class_name,group=groups.group_of(d.source_class_name),
                            confidence=d.confidence,xyxy=[d.xmin,d.ymin,d.xmax,d.ymax],mask_index=n)
                       for n,d in enumerate(detections)]
            if any(m is None for m in masks) and masks and not args.baseline:
                raise ValueError('seg box without corresponding instance mask')
            filename=str(index)+'.npz'
            if masks and not args.baseline:
                np.savez_compressed(args.output/'masks'/filename,masks=np.packbits(np.stack(masks),axis=2),width=row['width'])
            gt_masks=[]
            if row.get('masks'):
                with np.load(args.gt/row['masks']) as data:
                    gt_masks=np.unpackbits(data['masks'],axis=2)[:,:,:row['width']].astype(bool)
            out.write(json.dumps(dict(run=row['run'],view=row['view'],frame=row['frame'],
                       image=row['image'],source_sha256=row['source_sha256'],predictions=predicted,
                       masks='masks/'+filename if masks and not args.baseline else None,timings=timings))+'\n')
            pairs_at_7=[]
            for confidence in sweep:
                indices=[n for n,d in enumerate(predicted) if d['confidence']>=confidence and d['source_class_name'] not in INFRASTRUCTURE]
                evaluated=[predicted[n] for n in indices]
                count,values,names,pairs=frame_metrics(row['annotations'],evaluated,gt_masks,[masks[n] for n in indices])
                state=sums[confidence]
                for g in GROUPS:state['counts'][g].update(count[g])
                state['masks'].extend(values);state['names'].update(names)
                if confidence==.7:pairs_at_7=pairs
            present={row['annotations'][r]['actor_id'] for r,_,_ in pairs_at_7}
            for ann in row['annotations']:
                key=(row['run'],row['view'],ann['actor_id'])
                matched=ann['actor_id'] in present
                previous=continuity.get(key)
                if previous and previous[0]==int(row['frame'])-1 and previous[1] and not matched:
                    gaps[ann['group']]+=1
                continuity[key]=(int(row['frame']),matched)
                total_gt[ann['group']]+=1
            for d in predicted:labels[d['source_class_name']]+=1
            # Uniform fixed samples, including missed objects, not only attractive detections.
            if index%max(1,len(selected)//60)==0:
                rendered=image.copy()
                for ann in row['annotations']:
                    x1,y1,x2,y2=map(int,ann['xyxy']);cv2.rectangle(rendered,(x1,y1),(x2,y2),(255,100,0),1)
                for d,mask in zip(predicted,masks):
                    if d['confidence']<.7:continue
                    if mask is not None:rendered[mask]=(rendered[mask]*.65+np.array([0,230,50])*.35).astype(np.uint8)
                    x1,y1,x2,y2=map(int,d['xyxy']);cv2.rectangle(rendered,(x1,y1),(x2,y2),(0,255,0),1)
                    cv2.putText(rendered,d['source_class_name']+' %.2f'%d['confidence'],(max(0,x1),max(12,y1)),cv2.FONT_HERSHEY_SIMPLEX,.4,(0,255,0),1)
                name='%04d.jpg'%index;cv2.imwrite(str(args.output/'gallery'/name),rendered)
                gallery.append('<figure><figcaption>'+html.escape(row['run']+' '+row['view']+' '+row['frame'])+'</figcaption><img loading="lazy" width="960" src="'+name+'"></figure>')
            if (index+1)%100==0:print('selection',index+1,'/',len(selected),flush=True)
    thresholds=[dict(confidence=c,metrics=summarize(sums[c]['counts'],sums[c]['masks'],sums[c]['names'])) for c in sweep]
    result=dict(selection_frames=len(selected),thresholds=thresholds,selected_threshold=choose_threshold(thresholds),
                fixed_07=next(r for r in thresholds if r['confidence']==.7),
                model=detector.info,hood=hood.info,ego_masks_by_view={v:m.info for v,m in view_masks.items()},
                source_config_sha256=hashlib.sha256(args.config.read_bytes()).hexdigest(),
                gt_manifest_sha256=hashlib.sha256((args.gt/'frames.jsonl').read_bytes()).hexdigest(),
                input_latency_ms={key:float(np.percentile(all_latency,q)) for key,q in [('p50',50),('p95',95),('p99',99)]},
                labels=dict(labels),matched_to_missed_transitions=dict(gaps),observed_gt_frames=dict(total_gt),
                temporal_scope='visible GT actor/run/view only; no absent-actor interpolation',
                excluded_infrastructure=sorted(INFRASTRUCTURE),evaluation_scope='actor GT; infrastructure observations retained but not scored',
                heldout_evaluated=False,training_performed=False)
    (args.output/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    (args.output/'gallery/index.html').write_text('<!doctype html><meta charset="utf-8"><title>Seg selection</title><h1>Selection only · blue GT / green predictions at 0.7</h1>'+''.join(gallery))
    print(json.dumps(result['selected_threshold']),flush=True)


if __name__=='__main__':main()
