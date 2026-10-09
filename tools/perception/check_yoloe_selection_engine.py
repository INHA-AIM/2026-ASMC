#!/usr/bin/env python3
"""Same-image checkpoint/engine comparison, retaining FP16 disagreements for review."""
import argparse
from collections import Counter
import gc
import json
from pathlib import Path
import cv2
import numpy as np
import yaml
from asmc_camera.detection.batched_backend import BatchedSegmentationDetector
from asmc_camera.detection.ultralytics_backend import file_sha256
from coco7_masks import load_masks
from seg_measurements import match_boxes


def compare(reference, current):
    pairs=match_boxes(reference,current)
    return pairs, dict(reference=len(reference),engine=len(current),matched=len(pairs),
                      reference_unmatched=len(reference)-len(pairs),engine_unmatched=len(current)-len(pairs))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,required=True)
    p.add_argument('--gt',type=Path,required=True)
    p.add_argument('--datasets',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--frames',type=int,default=36)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    manifest=a.gt/'frames.jsonl'
    available=[json.loads(l) for l in manifest.read_text().splitlines()]
    available=[r for r in available if r['split']=='selection' and r['annotation_complete']]
    # Select each camera explicitly: all engine shapes must be exercised.
    rows=[]
    for view in ('front','left','right'):
        subset=[r for r in available if r['view']==view]
        if not subset:raise ValueError('no parity input for '+view)
        rows += [subset[i] for i in np.linspace(0,len(subset)-1,max(1,a.frames//3),dtype=int)]
    cfg=yaml.safe_load(a.config.read_text());cfg['conf_thres']=.4
    checkpoint=Path(cfg['weights']).parent/(cfg['model_id']+'.pt')
    reference_cfg=dict(cfg,weights=str(checkpoint),expected_sha256=file_sha256(checkpoint),runtime_backend='pytorch')
    masks=load_masks('black-v1',True)
    refs=[];infos={}
    for backend,config in [('pytorch',reference_cfg),('tensorrt',cfg)]:
        detector=BatchedSegmentationDetector(**config);infos[backend]=detector.info
        totals=Counter();records=[]
        for i,row in enumerate(rows):
            path=a.datasets/row['image']
            if file_sha256(path)!=row['source_sha256']['intensity']:raise ValueError('RGB SHA mismatch')
            image=cv2.imread(str(path));view=row['view']
            result=detector.infer_images({view:masks[view].apply(image)},[view])[view]
            boxes=[dict(group=d.source_class_name,label=d.source_class_name,confidence=d.confidence,
                        xyxy=[d.xmin,d.ymin,d.xmax,d.ymax]) for d in result.detections]
            if backend=='pytorch':
                refs.append(boxes)
                if result.masks:np.savez_compressed(a.output/f'{i}.npz',masks=np.packbits(np.stack(result.masks),axis=2),width=image.shape[1])
            else:
                pairs,counts=compare(refs[i],boxes);totals.update(counts);details=[]
                ref_masks=[]
                if refs[i]:
                    with np.load(a.output/f'{i}.npz') as data:ref_masks=np.unpackbits(data['masks'],axis=2)[:,:,:int(data['width'])].astype(bool)
                for r,c,overlap in pairs:
                    x,y=ref_masks[r],result.masks[c];union=np.count_nonzero(x|y)
                    details.append(dict(label=boxes[c]['label'],box_iou=overlap,
                        mask_iou=float(np.count_nonzero(x&y)/union) if union else 1.,
                        coordinate_delta=max(abs(x-y) for x,y in zip(refs[i][r]['xyxy'],boxes[c]['xyxy'])),
                        confidence_delta=boxes[c]['confidence']-refs[i][r]['confidence']))
                records.append(dict(image=row['image'],view=view,source_sha256=row['source_sha256'],
                                    reference=refs[i],engine=boxes,pairs=details,counts=counts))
        if backend=='pytorch':
            (a.output/'reference.json').write_text(json.dumps(dict(backend=infos[backend],rows=rows,predictions=refs),indent=2)+'\n')
        del detector;gc.collect()
        import torch
        torch.cuda.empty_cache()
    report=dict(backends=infos,frames=len(rows),totals=dict(totals),records=records,
        runtime_config_sha256=file_sha256(a.config),tool_sha256=file_sha256(Path(__file__)),
        gt_manifest_sha256=file_sha256(manifest),ego_masks={v:m.info for v,m in masks.items()},
        interpretation='Exact labels matched one-to-one at IoU>=0.5. FP16 differences retained; no blanket exemption.')
    (a.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(frames=len(rows),totals=dict(totals))),flush=True)


if __name__=='__main__':main()
