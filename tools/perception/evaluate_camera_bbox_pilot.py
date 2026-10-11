#!/usr/bin/env python3
"""One-to-one bbox evaluation; partial GT never supplies precision/FP/F1."""
import argparse
from collections import Counter,defaultdict
import json
from pathlib import Path
import sys
import time

from camera_data_audit import read_jsonl,save,sha,training_label
from seg_measurements import match_boxes
from capture_size_scope import ignored_prediction_indices


def accepted_name(name):
    if name in ('car','truck','bus'):return 'motor_vehicle'
    if name in ('dog','deer'):return 'animal'
    return name


def scoped_complete(row,pixel_scope):
    # Accurate actor boxes do not prove exhaustive labels for fixed map props.
    # Tiny unresolved actors alone cannot promote a partial image to full GT.
    return row.get('evaluation_scope')=='complete'


def evaluation_weights(requested, cfg, policy='pretrained'):
    """Keep the original COCO baseline distinct from a remapped initialization."""
    requested=Path(requested)
    if requested.name!='initial.pt' or policy=='initialization':
        return requested,'training_initialization' if requested.name=='initial.pt' else 'trained_checkpoint'
    run_path=requested.parent/'run.json'
    record=json.loads(run_path.read_text())
    if record['before_training_checkpoint']['sha256']!=sha(requested):
        raise ValueError('initial checkpoint provenance mismatch')
    if record['model_key']=='yolo26l':
        dataset=Path(json.loads((requested.parent.parent/'comparison.json').read_text())['dataset'])
        # dataset_versions/<version> is directly below the original data root.
        pretrained=dataset.parents[1]/cfg['models']['yolo26l']
        if sha(pretrained)!=record['initial_weights_sha256']:
            raise ValueError('pretrained baseline weight changed')
        return pretrained,'original_COCO80_pretrained; fixed16 initialization is a separate diagnostic'
    return requested,'pretrained_YOLOE_detect_with_fixed16_text_embeddings; before fine-tuning'


def frame_counts(row,predictions,confidence,level='class',pixel_scope=False):
    name=accepted_name if level=='accepted' else lambda n:n
    predictions=[dict(p,group=name(p['class_name'])) for p in predictions if p['confidence']>=confidence]
    gt=[];ignored=[]
    for a in row['annotations']:
        label=training_label(a)
        if not label:
            if a.get('disposition')!='excluded':ignored.append(a)
            continue
        box=a['xyxy'];w=box[2]-box[0];h=box[3]-box[1]
        long,short=(24,12) if label in ('car','truck','bus') else (16,8)
        if pixel_scope and (max(w,h)<long or min(w,h)<short):
            ignored.append(a);continue
        gt.append(dict(a,group=name(label)))
    pairs=match_boxes(gt,predictions,.5)
    excluded=ignored_prediction_indices(predictions,ignored,{p for _,p,_ in pairs},
        dict(ignore_prediction_ioa=.5))
    counts=defaultdict(Counter)
    complete=scoped_complete(row,pixel_scope)
    for a in gt:counts[a['group']]['gt']+=1
    for g,p,_ in pairs:counts[gt[g]['group']]['tp']+=1
    if complete:
        for a in gt:counts[a['group']]['complete_gt']+=1
        for g,p,_ in pairs:counts[gt[g]['group']]['complete_tp']+=1
        for i,p in enumerate(predictions):
            if i not in excluded:counts[p['group']]['complete_predictions']+=1
    for a in ignored:
        if training_label(a):counts[name(training_label(a))]['ignored_gt']+=1
    return counts,pairs


def summarize(rows,predictions,confidence,level='class',pixel_scope=False):
    by_id={p['id']:p for p in predictions}
    if len(by_id)!=len(predictions) or set(by_id)!={r['id'] for r in rows}:
        raise ValueError('prediction membership mismatch')
    counts=defaultdict(Counter);groups=defaultdict(set)
    for row in rows:
        pred=by_id[row['id']]
        if pred['image_sha256']!=row['image_sha256']:raise ValueError('prediction image SHA mismatch')
        c,_=frame_counts(row,pred['predictions'],confidence,level,pixel_scope)
        for name,v in c.items():counts[name].update(v);groups[name].add(row['split_group'])
    result={}
    for name,c in counts.items():
        tp=c['complete_tp'];n=c['complete_predictions'];gt=c['complete_gt']
        precision=tp/n if n else 0.0 if gt else None
        recall=tp/gt if gt else None
        f1=2*tp/(n+gt) if gt else None
        result[name]=dict(c,precision=precision,recall=recall,f1=f1,
            annotated_recall=c['tp']/c['gt'] if c['gt'] else None,
            complete_fp=n-tp,complete_fn=gt-tp,independent_source_groups=len(groups[name]),
            sufficient_sample=gt>=20,source_generalization_limited=len(groups[name])<2)
    scores=[c['f1'] for c in result.values() if c['sufficient_sample'] and c['f1'] is not None]
    return dict(confidence=confidence,classes=result,macro_f1=sum(scores)/len(scores) if scores else None,
        complete_images=sum(scoped_complete(r,pixel_scope) for r in rows),
        partial_images=sum(not scoped_complete(r,pixel_scope) for r in rows),
        scoped_native_capture_images=sum(r.get('evaluation_scope')!='complete' and scoped_complete(r,pixel_scope) for r in rows),
        macro_policy='>=20 complete GT; source/run correlation separately flagged, not independent-scene claims',
        pixel_scope='vehicle 24/12px, other 16/8px' if pixel_scope else 'all original GT')


def infer(args):
    started_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime())
    import cv2
    import torch
    import ultralytics
    from ultralytics import YOLO
    sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'src/asmc_camera/src'))
    from asmc_camera.detection.hood_mask import HoodMask
    repo=Path(__file__).resolve().parents[2]
    cfg=json.loads(args.config.read_text())
    names=list(json.loads((args.dataset/'summary.json').read_text())['classes'])
    rows=[r for r in read_jsonl(args.dataset/'manifest.jsonl') if r['split'] in ('selection','heldout')]
    temporal_path=args.dataset/'temporal-probe.jsonl'
    temporal=list(read_jsonl(temporal_path)) if temporal_path.exists() and not getattr(args,'skip_temporal',False) else []
    inference_rows=list({r['id']:r for r in rows+temporal}.values())
    args.output.mkdir(parents=True,exist_ok=False)
    masks={view:HoodMask.load(True,repo/'src/asmc_camera/config/camera'/file) for view,file in
        [('front','yolo_hood_mask_v1.yaml'),('left','yolo_ego_mask_left_v1.yaml'),('right','yolo_ego_mask_right_v1.yaml')]}
    actual_weights,baseline_kind=evaluation_weights(args.weights,cfg,args.baseline_policy)
    model=YOLO(str(actual_weights),task='detect')
    if 'Segment' in type(model.model.model[-1]).__name__:raise ValueError('not detection-only checkpoint')
    predictions=[];latencies=[];ignored_names=Counter()
    # Warm up once; timing is offline PT inference, never whole-stack latency.
    first=cv2.imread(rows[0]['image'])
    model.predict(first,imgsz=args.front_width,device=0,quantize=16,conf=.1,nms=False,verbose=False)
    if not model.predictor.model.fp16:raise RuntimeError('evaluation silently disabled FP16')
    with (args.output/'predictions.jsonl').open('w') as stream:
        for row in inference_rows:
            if sha(row['image'])!=row['image_sha256']:raise ValueError('evaluation source changed')
            image=cv2.imread(row['image']);view=row.get('view','unknown')
            if view in masks:image=masks[view].apply(image)
            size=(480,640) if view in ('left','right') else args.front_width
            torch.cuda.synchronize();started=time.monotonic()
            r=model.predict(image,imgsz=size,device=0,quantize=16,conf=.1,iou=.7,max_det=300,
                            nms=False,rect=True,verbose=False)[0]
            torch.cuda.synchronize();ms=(time.monotonic()-started)*1000;latencies.append(ms)
            boxes=[]
            for box in r.boxes.cpu():
                label=str(r.names[int(box.cls.item())]).replace(' ','_')
                if label not in names:ignored_names[label]+=1;continue
                boxes.append(dict(class_name=label,confidence=float(box.conf.item()),xyxy=box.xyxy[0].tolist()))
            item=dict(id=row['id'],image_sha256=row['image_sha256'],predictions=boxes,
                wall_ms=ms,view=view,ego_mask_applied=view in masks)
            stream.write(json.dumps(item)+'\n');stream.flush();predictions.append(item)
    import numpy as np
    save(args.output/'inference.json',dict(weights_sha256=sha(actual_weights),
        evaluator_source_sha256=sha(__file__),
        started_utc=started_utc,finished_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),
        torch=torch.__version__,ultralytics=ultralytics.__version__,opencv=cv2.__version__,
        cuda=torch.version.cuda,gpu=torch.cuda.get_device_name(0),actual_fp16=model.predictor.model.fp16,
        requested_weights=str(args.weights),requested_weights_sha256=sha(args.weights),
        actual_weights=str(actual_weights),baseline_kind=baseline_kind,
        manifest_sha256=sha(args.dataset/'manifest.jsonl'),config_sha256=sha(args.config),
        temporal_manifest_sha256=sha(temporal_path) if temporal else None,
        front_width=args.front_width,side_shape=[480,640],head=type(model.model.model[-1]).__name__,
        mask_shas={k:v.config_sha256 for k,v in masks.items()},unknown_view_mask_policy='no assumed mask',
        latency_kind='offline PyTorch batch1 inference wall; not integrated runtime or TensorRT',
        p50_ms=float(np.percentile(latencies,50)),p95_ms=float(np.percentile(latencies,95)),
        p99_ms=float(np.percentile(latencies,99)),ignored_non_target_predictions=dict(ignored_names)))
    inference=json.loads((args.output/'inference.json').read_text())
    inference['latency_by_view']={view:dict(count=len(values),p50_ms=float(np.percentile(values,50)),
        p95_ms=float(np.percentile(values,95)),p99_ms=float(np.percentile(values,99)))
        for view in sorted({p['view'] for p in predictions})
        for values in [[p['wall_ms'] for p in predictions if p['view']==view]]}
    save(args.output/'inference.json',inference)
    # Selection is the only threshold search. Heldout uses the frozen choice.
    selection=[r for r in rows if r['split']=='selection'];heldout=[r for r in rows if r['split']=='heldout']
    def preds(part):return [p for p in predictions if p['id'] in {r['id'] for r in part}]
    frozen=getattr(args,'fixed_confidence',None)
    sweep=[summarize(selection,preds(selection),value,pixel_scope=True)
        for value in ([frozen] if frozen is not None else [round(.1+.05*i,2) for i in range(13)])]
    available=[s for s in sweep if s['macro_f1'] is not None]
    chosen=max(available,key=lambda s:(s['macro_f1'],s['confidence']))['confidence'] if available else .4
    save(args.output/'evaluation.json',dict(selection_confidence=chosen,
        threshold_selection_available=bool(available) and frozen is None,
        threshold_policy='frozen diagnostic; no threshold search' if frozen is not None else 'selection-only search',selection_sweep=sweep,
        selected_selection=summarize(selection,preds(selection),chosen,pixel_scope=True),
        selected_heldout=summarize(heldout,preds(heldout),chosen,pixel_scope=True),
        selected_heldout_accepted=summarize(heldout,preds(heldout),chosen,'accepted',pixel_scope=True),
        control={role:{level:summarize(part,preds(part),.4,level,pixel_scope=True)
            for level in ('class','accepted')} for role,part in [('selection',selection),('heldout',heldout)]},
        all_gt_control={role:summarize(part,preds(part),.4) for role,part in [('selection',selection),('heldout',heldout)]},
        temporal_metrics=dict(status='not_evaluable_from_sparse_sample',
            reason='sparse accuracy samples have no continuous sensor timing')))
    if temporal:
        from camera_bbox_temporal import analyze
        save(args.output/'temporal.json',dict(
            control={level:analyze(temporal,predictions,.4,level) for level in ('class','accepted')},
            selected={level:analyze(temporal,predictions,chosen,level) for level in ('class','accepted')},
            manifest_sha256=sha(temporal_path),threshold_selected_on_sparse_selection_only=True))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',required=True,type=Path);p.add_argument('--dataset',required=True,type=Path)
    p.add_argument('--weights',required=True,type=Path);p.add_argument('--output',required=True,type=Path)
    p.add_argument('--front-width',required=True,type=int,choices=[960,1280])
    p.add_argument('--baseline-policy',choices=['pretrained','initialization'],default='pretrained',
        help='original pretrained baseline or actual fixed-class training initialization')
    p.add_argument('--fixed-confidence',type=float,default=None)
    p.add_argument('--skip-temporal',action='store_true',help='diagnostic accuracy only; preserve the original temporal results')
    infer(p.parse_args())
