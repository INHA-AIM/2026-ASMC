#!/usr/bin/env python3
"""Check saved FP32 references against ONNX or the exact runtime TRT engine.

One-to-one class-specific box matching; threshold crossings are reported, not
silently discarded. Raw output error and batch 1/2/3 coverage are retained.
"""
import argparse
import json
from pathlib import Path
import sys
import numpy as np
from scipy.optimize import linear_sum_assignment

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src/asmc_camera/src'))
from asmc_camera.detection.rfdetr_core import decode
from camera_data_audit import save,sha


def iou(a,b):
    x1,y1,x2,y2=max(a.xmin,b.xmin),max(a.ymin,b.ymin),min(a.xmax,b.xmax),min(a.ymax,b.ymax)
    overlap=max(0,x2-x1)*max(0,y2-y1)
    union=(a.xmax-a.xmin)*(a.ymax-a.ymin)+(b.xmax-b.xmin)*(b.ymax-b.ymin)-overlap
    return overlap/union if union else 0.


def compare(reference,candidate,threshold):
    matches=[];unmatched=[]
    for name in sorted({d.source_class_name for d in reference+candidate}):
        aa=[d for d in reference if d.source_class_name==name]
        bb=[d for d in candidate if d.source_class_name==name]
        overlaps=np.array([[iou(a,b) for b in bb] for a in aa])
        differences=np.array([[abs(a.confidence-b.confidence) for b in bb] for a in aa])
        k=min(len(aa),len(bb))+1
        # Duplicate query boxes may nearly coincide. Maximize pairs meeting
        # BOTH declared tolerances first, then geometric pairs, then IoU.
        score=((overlaps>=.98)&(differences<=.02))*k*k+(overlaps>=.5)*k+overlaps
        pairs=list(zip(*linear_sum_assignment(-score))) if aa and bb else []
        used_a=set();used_b=set()
        for i,j in pairs:
            if overlaps[i,j]<.5:continue
            used_a.add(i);used_b.add(j)
            matches.append(dict(class_name=name,iou=float(overlaps[i,j]),
                confidence_delta=abs(aa[i].confidence-bb[j].confidence),
                reference_xyxy=[aa[i].xmin,aa[i].ymin,aa[i].xmax,aa[i].ymax],
                candidate_xyxy=[bb[j].xmin,bb[j].ymin,bb[j].xmax,bb[j].ymax],
                reference_confidence=aa[i].confidence,candidate_confidence=bb[j].confidence))
        for side,ds,used in [('reference',aa,used_a),('candidate',bb,used_b)]:
            for i,d in enumerate(ds):
                if i not in used:unmatched.append(dict(side=side,class_name=name,confidence=d.confidence,
                    threshold_boundary=abs(d.confidence-threshold)<=.02))
    passed=all(m['iou']>=.98 and m['confidence_delta']<=.02 for m in matches) and all(u['threshold_boundary'] for u in unmatched)
    return dict(passed=passed,matches=matches,unmatched=unmatched,threshold=threshold)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--reference',type=Path,required=True);p.add_argument('--model',type=Path,required=True)
    p.add_argument('--runtime-config',type=Path);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--allow-fp32-diagnostic',action='store_true',help='Parity diagnosis only; not runtime qualification')
    p.add_argument('--thresholds',type=float,nargs='+',default=[.1,.4,.7],help='Explicit output thresholds being qualified')
    p.add_argument('--kind',choices=['onnx','tensorrt'],required=True);a=p.parse_args()
    reference_metadata=json.loads((a.reference.parent/'inference.json').read_text())
    model_metadata=json.loads((a.model.with_suffix('.metadata.json') if a.kind=='tensorrt' else a.model.parent/'onnx.json').read_text())
    if reference_metadata['weights_sha256']!=model_metadata['checkpoint_sha256']:
        raise ValueError('reference/checkpoint SHA mismatch')
    if reference_metadata.get('backend_info',{}).get('tf32') is not False:
        raise ValueError('reference must explicitly disable TF32')
    if not a.thresholds or any(not 0<=v<=1 for v in a.thresholds):p.error('thresholds must be in [0,1]')
    if a.kind=='onnx':
        import onnxruntime as ort
        session=ort.InferenceSession(str(a.model),providers=['CPUExecutionProvider'])
        def run(batch):return session.run(['dets','labels'],{session.get_inputs()[0].name:batch})
    else:
        from asmc_camera.detection.rfdetr_backend import RFDetrBackend
        config=json.loads(a.runtime_config.read_text())
        engine=RFDetrBackend(**dict(config,weights=str(a.model),expected_sha256=sha(a.model),allow_fp32_diagnostic=a.allow_fp32_diagnostic))
        run=engine.raw
    data=np.load(a.reference);records=[];batches=set()
    for key in sorted(k[6:] for k in data.files if k.startswith('input_')):
        batch=data['input_'+key];batches.add(len(batch));boxes,logits=run(batch)
        rb,rl=data['boxes_'+key],data['logits_'+key];hw=data['hw_'+key]
        if boxes.shape!=rb.shape or logits.shape!=rl.shape:raise ValueError('raw output shape mismatch')
        item=dict(key=key,batch=len(batch),raw_box_max_error=float(np.max(np.abs(rb-boxes))),
            raw_logit_max_error=float(np.max(np.abs(rl-logits))),thresholds=[])
        for threshold in a.thresholds:
            reference=decode(rb,rl,hw,threshold);candidate=decode(boxes,logits,hw,threshold)
            item['thresholds'].extend(dict(compare(r,c,threshold),image_in_batch=i) for i,(r,c) in enumerate(zip(reference,candidate)))
        records.append(item)
    report=dict(kind=a.kind,thresholds=a.thresholds,fp32_diagnostic=a.allow_fp32_diagnostic,model_sha256=sha(a.model),reference_sha256=sha(a.reference),batch_sizes=sorted(batches),
        passed=batches=={1,2,3} and any(c['matches'] for r in records for c in r['thresholds'])
            and all(c['passed'] for r in records for c in r['thresholds']),records=records)
    save(a.output,report)
    print(json.dumps(dict(passed=report['passed'],batch_sizes=report['batch_sizes'],output=str(a.output))))
    raise SystemExit(0 if report['passed'] else 1)


if __name__=='__main__':main()
