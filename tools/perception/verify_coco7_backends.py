#!/usr/bin/env python3
"""Fixed-image FP32/FP16 correspondence and all three Ego-mask OFF controls."""
import argparse
import gc
import json
from pathlib import Path
import cv2
import numpy as np
from asmc_camera.detection.yolo_batch_backend import BatchedYoloDetector, COCO7
from asmc_camera.detection.ultralytics_backend import file_sha256
from asmc_camera.detection.hood_mask import HoodMask, load_view_masks
from evaluate_coco7_models import evaluate
from seg_measurements import match_boxes

ROOT = Path(__file__).resolve().parents[2]


def predictions(frame):
    return [dict(source_class_name=d.source_class_name,group=d.source_class_name,confidence=d.confidence,
                 xyxy=[d.xmin,d.ymin,d.xmax,d.ymax]) for d in frame.detections]


def release(detector):
    del detector
    gc.collect()
    import torch
    torch.cuda.empty_cache()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--gt',type=Path,required=True)
    p.add_argument('--datasets',type=Path,required=True)
    p.add_argument('--models',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--model',nargs='+',default=['yolo11m','yolo26m','yolo26l','yolo26m-seg','yolo26l-seg'])
    p.add_argument('--width',type=int,nargs='+',default=[640,960,1280])
    p.add_argument('--confidence',type=float,default=.7)
    p.add_argument('--image',nargs='+',help='Explicit selection GT hard cases, in addition to every side-overlap control')
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    rows=[json.loads(line) for line in (a.gt/'frames.jsonl').read_text().splitlines()]
    rows=[r for r in rows if r['split']=='selection' and r['annotation_complete']]
    masks=load_view_masks(['front','left','right'],HoodMask.load(True,ROOT/'src/asmc_camera/config/camera/yolo_hood_mask_v1.yaml'),True,
        {v:ROOT/f'src/asmc_camera/config/camera/yolo_ego_mask_{v}_v1.yaml' for v in ['left','right']})
    selected=[]
    for view in ('front','left','right'):
        subset=[r for r in rows if r['view']==view and any(g.get('subtype_verified') and g['source_class_name'] in COCO7 for g in r['annotations'])]
        selected.extend(subset[::max(1,len(subset)//5)][:5])
    if a.image:
        by_image={r['image']:r for r in rows}
        if set(a.image)-by_image.keys():raise ValueError('hard case absent from complete selection GT')
        selected=[by_image[name] for name in a.image]
    # Include every actual GT mask overlap on either side, not just easy samples.
    overlap=[]
    for row in rows:
        if row['view']=='front' or not row.get('masks'):continue
        with np.load(a.gt/row['masks']) as data:gm=np.unpackbits(data['masks'],axis=2)[:,:,:row['width']].astype(bool)
        ego,_=masks[row['view']].geometry(row['height'],row['width'])
        objects=[g['actor_id'] for n,g in enumerate(row['annotations']) if g.get('subtype_verified') and g['source_class_name'] in COCO7 and np.any(gm[n]&ego)]
        if objects:overlap.append(dict(row,overlap_actors=objects))
    selected+= [r for r in overlap if r['image'] not in {s['image'] for s in selected}]
    report=dict(samples=len(selected),confidence=a.confidence,side_overlap_frames=len(overlap),parity=[],mask_controls=[],
                acceptance=dict(box_iou=.95,confidence_delta=.03,mask_iou=.95),all_side_overlap_frames=[r['image'] for r in overlap])
    def save():(a.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    for model in a.model:
        for width in a.width:
            task='segment' if 'seg' in model else 'detect'
            cached=[]
            for backend,suffix in [('pytorch','.pt'),('tensorrt','.engine')]:
                weight=a.models/(model+suffix)
                detector=BatchedYoloDetector(weights=str(weight),expected_sha256=file_sha256(weight),runtime_backend=backend,
                    task=task,front_width=width,conf_thres=a.confidence)
                outputs=[]
                for row in selected:
                    image=cv2.imread(str(a.datasets/row['image']))
                    if image is None:raise RuntimeError('missing image '+row['image'])
                    frame=detector.infer_images({row['view']:masks[row['view']].apply(image)},[row['view']])[row['view']]
                    outputs.append(frame)
                cached.append(outputs)
                if backend=='tensorrt':
                    for row in overlap:
                        image=cv2.imread(str(a.datasets/row['image']));view=row['view']
                        on=detector.infer_images({view:masks[view].apply(image)},[view])[view]
                        off=detector.infer_images({view:image},[view])[view]
                        _,_,on_pairs,on_gt,_=evaluate(row['annotations'],predictions(on),a.confidence)
                        _,_,off_pairs,off_gt,_=evaluate(row['annotations'],predictions(off),a.confidence)
                        on_ids={on_gt[g]['actor_id'] for g,_,_ in on_pairs};off_ids={off_gt[g]['actor_id'] for g,_,_ in off_pairs}
                        report['mask_controls'].append(dict(model=model,width=width,image=row['image'],
                            overlap_actors=row['overlap_actors'],on_matched=sorted(on_ids),off_matched=sorted(off_ids),
                            new_misses=sorted(off_ids-on_ids),on=predictions(on),off=predictions(off)))
                del detector;gc.collect()
                import torch
                torch.cuda.empty_cache()
            for row,pt,trt in zip(selected,*cached):
                pboxes,tboxes=predictions(pt),predictions(trt);pairs=match_boxes(pboxes,tboxes,threshold=.95)
                delta=[abs(pboxes[g]['confidence']-tboxes[t]['confidence']) for g,t,_ in pairs]
                mi=[]
                if task=='segment':
                    for g,t,_ in pairs:
                        x,y=pt.masks[g],trt.masks[t];u=np.count_nonzero(x|y)
                        mi.append(float(np.count_nonzero(x&y)/u) if u else 1.)
                report['parity'].append(dict(model=model,width=width,image=row['image'],
                    pytorch=pboxes,tensorrt=tboxes,pairs=pairs,confidence_deltas=delta,mask_ious=mi,
                    passed=len(pairs)==len(pboxes)==len(tboxes) and all(d<=.03 for d in delta) and all(v>=.95 for v in mi)))
            print(model,width,'completed',flush=True);save()
    report['parity_failures']=sum(not r['passed'] for r in report['parity'])
    report['new_side_mask_misses']=sum(len(r['new_misses']) for r in report['mask_controls'])
    save()


if __name__=='__main__':main()
