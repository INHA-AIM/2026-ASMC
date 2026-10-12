#!/usr/bin/env python3
"""Same-image COCO7 pretrained evaluation using existing pilot GT and metrics.

Partial GT is recall-only; no training or label changes. Runtime engine quality
is compared to runtime engine quality, not the pilot's different YOLO head.
"""
import argparse
from collections import Counter
import json
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src/asmc_camera/src'))
from camera_data_audit import read_jsonl,save,sha,training_label
from evaluate_camera_bbox_pilot import summarize
from asmc_camera.detection.rfdetr_core import COCO7


def seven_rows(rows):
    # Unsupported GT is excluded from recall, not an ignore region that could
    # conceal a target-class false positive. Never mutate the original rows.
    return [dict(r,annotations=[dict(a,training_class=None,disposition='excluded')
        if training_label(a) not in COCO7 else dict(a) for a in r['annotations']]) for r in rows]


def validate_saved_evaluation(dataset, output, rows, predictions):
    """Reject partial runs, stale splits and another image's cached predictions."""
    info=json.loads((output/'inference.json').read_text())
    for name,key in [('manifest.jsonl','dataset_manifest_sha256'),
                     ('temporal-probe.jsonl','temporal_manifest_sha256')]:
        if info.get(key)!=sha(dataset/name):raise ValueError('evaluation manifest SHA mismatch: '+name)
    pred={p['id']:p for p in predictions}
    if len(pred)!=len(predictions):raise ValueError('duplicate prediction image ID')
    for row in rows:
        if row['id'] not in pred:raise ValueError('incomplete evaluation run: '+row['id'])
        if pred[row['id']].get('image_sha256')!=row['image_sha256']:
            raise ValueError('prediction/image SHA mismatch: '+row['id'])
    return info


def quality_gate(baseline,candidate):
    eligible=[c for c,v in baseline['classes'].items() if v.get('sufficient_sample') and v['f1'] is not None]
    if not eligible:return dict(passed=False,reason='no complete GT class with >=20 objects',unassessed=list(COCO7))
    bm=sum(baseline['classes'][c]['f1'] for c in eligible)/len(eligible)
    cm=sum(candidate['classes'].get(c,{}).get('f1') or 0 for c in eligible)/len(eligible)
    drops={c:(candidate['classes'].get(c,{}).get('annotated_recall') or 0)-v['annotated_recall']
        for c,v in baseline['classes'].items() if c in COCO7 and v.get('gt',0)>=20}
    return dict(passed=cm>=bm-.01-1e-9 and all(v>=-.02-1e-9 for v in drops.values()),
        macro_classes=eligible,baseline_macro_f1=bm,candidate_macro_f1=cm,macro_delta=cm-bm,
        annotated_recall_deltas=drops,unassessed=[c for c in COCO7 if c not in drops],
        source_independence='existing source concentration preserved; not seven-class generalization acceptance')


def next_step(quality_pass,load_pass,batch_wait_cause=False,small_object_cause=False,attempt='l-704',parity_pass=True):
    if not parity_pass:return 'resolve_backend_parity; no_model_ranking_or_scale_fallback'
    if quality_pass and load_pass:return 'final_repeat_yolo_vs_rf; skip_M_other_resolutions_X_XL'
    if batch_wait_cause:return 'l-704-2plus1'
    if quality_pass and not load_pass:
        return 'm-576-batch3' if attempt=='l-640' else 'l-640-batch3' if attempt=='l-704' else 'limited_X_XL_quality_screen'
    if load_pass and small_object_cause and attempt=='l-704':return 'l-front960-sides640-2plus1'
    return 'limited_X_XL_quality_screen; RF_selection_deferred'


def infer(a):
    import cv2
    import numpy as np
    from asmc_camera.detection.hood_mask import HoodMask
    from asmc_camera.detection.rfdetr_core import preprocess,decode
    a.output.mkdir(parents=True,exist_ok=False)
    rows=[r for r in read_jsonl(a.dataset/'manifest.jsonl') if r['split'] in ('selection','heldout')]
    temporal=list(read_jsonl(a.dataset/'temporal-probe.jsonl')) if (a.dataset/'temporal-probe.jsonl').exists() else []
    rows=list({r['id']:r for r in rows+temporal}.values())
    if a.limit:rows=rows[:a.limit]
    parity_starts={0,3,6,9};seen_reference=set()
    for i,row in enumerate(rows):
        for ann in row['annotations']:
            label=training_label(ann);key=(row['split'],row.get('view'),label)
            if label in COCO7 and key not in seen_reference:
                seen_reference.add(key)
                if len(parity_starts)<24:parity_starts.add(i//3*3)
    masks={v:HoodMask.load(True,ROOT/'src/asmc_camera/config/camera'/f) for v,f in
        [('front','yolo_hood_mask_v1.yaml'),('left','yolo_ego_mask_left_v1.yaml'),('right','yolo_ego_mask_right_v1.yaml')]}
    cfg=json.loads(a.runtime_config.read_text()) if a.runtime_config else {}
    backend=None;raw=None
    if a.kind=='rfdetr-pt':
        import torch
        from rfdetr import RFDETRLarge,RFDETRMedium
        torch.backends.cuda.matmul.allow_tf32=False
        torch.backends.cudnn.allow_tf32=False
        model=(RFDETRLarge if a.scale=='l' else RFDETRMedium)(pretrain_weights=str(a.weights),device='cuda')
        raw=model.model.model.eval().cuda()
        def run(batch):
            with torch.inference_mode():
                result=raw(torch.from_numpy(batch).cuda())
            torch.cuda.synchronize()
            return result['pred_boxes'].float().cpu().numpy(),result['pred_logits'].float().cpu().numpy()
    elif a.kind=='rfdetr-trt':
        from asmc_camera.detection.rfdetr_backend import RFDetrBackend
        backend=RFDetrBackend(**dict(cfg,weights=str(a.weights),expected_sha256=sha(a.weights),conf_thres=.1))
        run=backend.raw
    else:
        from asmc_camera.detection.yolo_batch_backend import BatchedYoloDetector
        backend=BatchedYoloDetector(weights=str(a.weights),expected_sha256=sha(a.weights),conf_thres=.1,
            runtime_backend='tensorrt',task='detect',front_width=1280,batch_mode='2plus1',allowed_names=COCO7)
    predictions=[];parity={};latencies=[]
    with (a.output/'predictions.jsonl').open('w') as stream:
        for start in range(0,len(rows),3):
            part=rows[start:start+3];images=[]
            for row in part:
                if sha(row['image'])!=row['image_sha256']:raise ValueError('evaluation image changed')
                image=cv2.imread(row['image'])
                if image is None:raise ValueError('missing RGB')
                if row.get('view') in masks:image=masks[row['view']].apply(image)
                images.append(image)
            started=time.monotonic()
            if a.kind.startswith('rfdetr'):
                shapes=cfg.get('rfdetr_input_shapes',{})
                per_shapes=[shapes.get(r.get('view'),[a.shape,a.shape]) for r in part]
                # First candidates have one common square; rectangular followup
                # uses individual inference, never pads into a different shape.
                if len({tuple(s) for s in per_shapes})==1:
                    batch=np.stack([preprocess(im,s) for im,s in zip(images,per_shapes)])
                    boxes,logits=run(batch);ds=decode(boxes,logits,[im.shape[:2] for im in images],.1)
                    if a.kind=='rfdetr-pt' and start in parity_starts:
                        parity['input_%d'%start]=batch;parity['boxes_%d'%start]=boxes;parity['logits_%d'%start]=logits
                        parity['hw_%d'%start]=np.array([im.shape[:2] for im in images])
                        if start==0:
                            for n in (1,2):
                                b,l=run(batch[:n])
                                key='b%d'%n
                                parity['input_'+key]=batch[:n];parity['boxes_'+key]=b;parity['logits_'+key]=l
                                parity['hw_'+key]=np.array([im.shape[:2] for im in images[:n]])
                else:
                    ds=[]
                    for im,s in zip(images,per_shapes):
                        b,l=run(preprocess(im,s)[None]);ds+=decode(b,l,[im.shape[:2]],.1)
            else:
                ds=[]
                for row,image in zip(part,images):
                    view=row.get('view') if row.get('view') in masks else 'left'
                    ds.append(backend.infer_images({view:image},[view])[view].detections)
            elapsed=(time.monotonic()-started)*1000;latencies.append(elapsed)
            for row,dd in zip(part,ds):
                item=dict(id=row['id'],image_sha256=row['image_sha256'],view=row.get('view'),
                    ego_mask_applied=row.get('view') in masks,group_wall_ms=elapsed,
                    predictions=[dict(class_name=d.source_class_name,confidence=d.confidence,
                        xyxy=[d.xmin,d.ymin,d.xmax,d.ymax]) for d in dd])
                predictions.append(item);stream.write(json.dumps(item)+'\n');stream.flush()
    if parity:np.savez_compressed(a.output/'parity-reference.npz',**parity)
    save(a.output/'inference.json',dict(kind=a.kind,weights_sha256=sha(a.weights),
        dataset_manifest_sha256=sha(a.dataset/'manifest.jsonl'),temporal_manifest_sha256=sha(a.dataset/'temporal-probe.jsonl'),
        evaluator_sha256=sha(__file__),mask_shas={v:m.config_sha256 for v,m in masks.items()},
        parity_sample_indices=sorted(parity_starts),
        unknown_view_mask_policy='no assumed mask',shape=a.shape,config=cfg,images=len(predictions),
        saved_confidence_floor=.1,latency_scope='offline batch groups; not ROS/full-stack p50/p95/p99',
        group_wall_ms={str(q):float(np.percentile(latencies,q)) for q in [50,95,99]},
        backend_info=backend.info if backend else {'precision':'PyTorch FP32 reference','tf32':False,'no_training':True}))


def analyze(a):
    rows=seven_rows([r for r in read_jsonl(a.dataset/'manifest.jsonl') if r['split'] in ('selection','heldout')])
    predictions=list(read_jsonl(a.output/'predictions.jsonl'));pred={p['id']:p for p in predictions}
    temporal=list(read_jsonl(a.dataset/'temporal-probe.jsonl'))
    validate_saved_evaluation(a.dataset,a.output,rows+temporal,predictions)
    split={s:[r for r in rows if r['split']==s] for s in ['selection','heldout']}
    sweep=[summarize(split['selection'],[pred[r['id']] for r in split['selection']],round(.1+.05*i,2),pixel_scope=True) for i in range(13)]
    eligible=[s for s in sweep if s['macro_f1'] is not None]
    conf=max(eligible,key=lambda s:(s['macro_f1'],s['confidence']))['confidence'] if eligible else .4
    report=dict(selection_sweep=sweep,selected_confidence=conf,heldout_tuned=False,
        control={s:summarize(rr,[pred[r['id']] for r in rr],.4,pixel_scope=True) for s,rr in split.items()},
        selected={s:summarize(rr,[pred[r['id']] for r in rr],conf,pixel_scope=True) for s,rr in split.items()})
    # Preserve the original-class pixel scope before collapsing names; vehicle
    # 24/12 and other 16/8 cutoffs must not change with the geometry metric.
    from evaluate_camera_bbox_pilot import match_boxes
    geometry={}
    for role,rr in split.items():
        total=Counter();matched=Counter()
        for r in rr:
            gt=[]
            for ann in r['annotations']:
                label=training_label(ann)
                if not label:continue
                x1,y1,x2,y2=ann['xyxy'];long,short=(24,12) if label in ('car','truck','bus') else (16,8)
                if max(x2-x1,y2-y1)<long or min(x2-x1,y2-y1)<short:continue
                gt.append(dict(ann,group='geometry'));total[label]+=1
            pp=[dict(v,group='geometry') for v in pred[r['id']]['predictions'] if v['confidence']>=.4]
            for g,_,_ in match_boxes(gt,pp,.5):matched[training_label(gt[g])]+=1
        geometry[role]={c:dict(gt=n,tp=matched[c],recall=matched[c]/n) for c,n in total.items()}
    report['geometry_at_040']=geometry
    from camera_bbox_temporal import analyze as temporal_analyze
    tr=seven_rows(temporal)
    report['temporal']={str(c):temporal_analyze(tr,predictions,c) for c in sorted({.4,conf})}
    save(a.output/'evaluation.json',report)
    if a.baseline:
        baseline_predictions=list(read_jsonl(a.baseline/'predictions.jsonl'))
        validate_saved_evaluation(a.dataset,a.baseline,rows+temporal,baseline_predictions)
        b=json.loads((a.baseline/'evaluation.json').read_text())
        # The operating YOLO stays at .40. Its separately swept threshold is
        # diagnostic, not a new operating baseline or a heldout-tuned setting.
        report['quality_gate']={mode:quality_gate(b['control']['heldout'],report[mode]['heldout']) for mode in ['control','selected']}
        report['quality_gate_selection']={mode:quality_gate(b['control']['selection'],report[mode]['selection']) for mode in ['control','selected']}
        save(a.output/'evaluation.json',report)
    print(json.dumps(dict(confidence=conf,heldout=report['selected']['heldout']['classes'],gate=report.get('quality_gate'))))


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('phase',choices=['infer','analyze'])
    p.add_argument('--kind',choices=['rfdetr-pt','rfdetr-trt','yolo-trt'],default='rfdetr-trt')
    p.add_argument('--weights',type=Path);p.add_argument('--runtime-config',type=Path)
    p.add_argument('--dataset',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--baseline',type=Path);p.add_argument('--scale',choices=['m','l'],default='l')
    p.add_argument('--shape',type=int,default=704);p.add_argument('--limit',type=int)
    a=p.parse_args();globals()[a.phase](a)


if __name__=='__main__':main()
