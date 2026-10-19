#!/usr/bin/env python3
"""CPU-only, fixed-confidence selection diagnostic during uninterrupted training."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import time


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for b in iter(lambda: f.read(1048576), b''):
            h.update(b)
    return h.hexdigest()


def main(args):
    os.environ['CUDA_VISIBLE_DEVICES'] = ''
    os.environ['OMP_NUM_THREADS'] = str(args.threads)
    os.environ['OPENBLAS_NUM_THREADS'] = '1'
    os.nice(10)
    import torch
    import cv2
    import ultralytics
    from ultralytics import YOLO
    torch.set_num_threads(args.threads)
    torch.set_num_interop_threads(1)
    cv2.setNumThreads(1)
    run = args.run.resolve()
    frozen = run/'code'
    sys.path.insert(0, str(frozen/'tools/perception'))
    sys.path.insert(0, str(frozen/'src/asmc_camera/src'))
    from evaluate_camera_bbox_pilot import summarize
    from camera_pretrain_policy import CLASSES
    from asmc_camera.detection.hood_mask import HoodMask
    weights = run/f'main/fit/weights/epoch{args.epoch-1}.pt'
    checkpoint = torch.load(weights, map_location='cpu')
    if checkpoint.get('epoch') != args.epoch-1 or checkpoint.get('optimizer') is None:
        raise ValueError('Not a complete epoch checkpoint')
    del checkpoint
    weights_sha = sha(weights)
    manifest = run/'evaluation/manifest.jsonl'
    rows = [json.loads(x) for x in manifest.read_text().splitlines() if x]
    if any(r['split'] != 'selection' for r in rows):
        raise ValueError('Selection-only evaluation required')
    masks = {v:HoodMask.load(True,frozen/'src/asmc_camera/config/camera'/f) for v,f in (
        ('front','yolo_hood_mask_v1.yaml'),('left','yolo_ego_mask_left_v1.yaml'),('right','yolo_ego_mask_right_v1.yaml'))}
    args.output.mkdir(parents=True,exist_ok=False)
    def save(name, data):
        tmp = args.output/(name+'.tmp')
        tmp.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n')
        tmp.replace(args.output/name)
    start = time.time()
    save('progress.json',dict(status='loading',epoch=args.epoch,completed=0,total=len(rows)))
    model = YOLO(str(weights), task='detect')
    predictions = []
    with (args.output/'predictions.jsonl').open('w') as stream:
        for i,row in enumerate(rows):
            if sha(row['image']) != row['image_sha256']:
                raise ValueError('Image SHA mismatch')
            view = row.get('view','unknown')
            image = cv2.imread(row['image'])
            if view in masks:
                if row.get('ego_mask_applied') and row['mask_config_sha256'] != masks[view].config_sha256:
                    raise ValueError('Frozen mask mismatch')
                image = masks[view].apply(image)
            size = (480,640) if view in ('left','right') else 1280
            tick = time.monotonic()
            result = model.predict(image,imgsz=size,device='cpu',quantize=32,conf=.1,iou=.7,
                                   max_det=300,nms=False,rect=True,verbose=False)[0]
            if model.predictor.model.fp16 or next(model.model.parameters()).is_cuda:
                raise RuntimeError('Expected CPU FP32')
            boxes=[]
            for b in result.boxes.cpu():
                label=str(result.names[int(b.cls.item())]).replace(' ','_')
                if label not in CLASSES:raise ValueError('Unexpected trained class '+label)
                boxes.append(dict(class_name=label,confidence=float(b.conf.item()),xyxy=b.xyxy[0].tolist()))
            item=dict(id=row['id'],image_sha256=row['image_sha256'],predictions=boxes,
                      wall_ms=(time.monotonic()-tick)*1000,view=view,ego_mask_applied=view in masks)
            predictions.append(item);stream.write(json.dumps(item)+'\n');stream.flush()
            save('progress.json',dict(status='running',epoch=args.epoch,completed=i+1,total=len(rows),seconds=time.time()-start))
    metrics=summarize(rows,predictions,.4)
    save('evaluation.json',dict(control={'selection':{'class':metrics}},threshold_policy='fixed confidence .40; no tuning',heldout_used=False))
    cfg=run/'code/config/experiments/camera_bbox_main_v11.json'
    save('inference.json',dict(weights_sha256=weights_sha,requested_weights=str(weights),actual_weights=str(weights),
        manifest_sha256=sha(manifest),config_sha256=sha(cfg),evaluator_source_sha256=sha(__file__),
        metric_evaluator_sha256=sha(frozen/'tools/perception/evaluate_camera_bbox_pilot.py'),
        front_width=1280,side_shape=[480,640],head='Detect',actual_fp16=False,device='cpu',threads=args.threads,
        torch=torch.__version__,ultralytics=ultralytics.__version__,mask_shas={v:m.config_sha256 for v,m in masks.items()},
        gpu_inference=False,latency_kind='CPU diagnostic during training; not deployment timing',
        elapsed_seconds=time.time()-start,epoch=args.epoch,created_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime())))
    if sha(weights)!=weights_sha:raise ValueError('Checkpoint changed during inference')
    save('progress.json',dict(status='complete',epoch=args.epoch,completed=len(rows),total=len(rows),seconds=time.time()-start,
                             macro_f1_fixed16=metrics['macro_f1_fixed16']))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',type=Path,required=True)
    p.add_argument('--epoch',type=int,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--threads',type=int,default=2)
    main(p.parse_args())
