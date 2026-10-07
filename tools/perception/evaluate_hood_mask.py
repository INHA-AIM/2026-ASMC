#!/usr/bin/env python3
"""Compare full-image/masked YOLO against fixed, independent MORAI BBox2D labels."""
import argparse
import hashlib
import json
from pathlib import Path
import time

import cv2
import numpy as np
from scipy.optimize import linear_sum_assignment

from asmc_camera.detection.hood_mask import HoodMask
from asmc_camera.detection.ultralytics_backend import UltralyticsDetector

ROOT=Path(__file__).resolve().parents[2]


def iou(a,b):
    intersection=max(0,min(a[2],b[2])-max(a[0],b[0]))*max(0,min(a[3],b[3])-max(a[1],b[1]))
    union=(a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-intersection
    return intersection/union if union>0 else 0.


def label(name):
    return 'person' if name.lower()=='person' else 'vehicle' if name.lower() in ('car','bus','truck') else name.lower()


def matching(ground_truth,detections):
    if not ground_truth or not detections:return []
    weights=np.zeros((len(ground_truth),len(detections)))
    for i,g in enumerate(ground_truth):
        for j,d in enumerate(detections):
            overlap=iou(g['xyxy'],d['xyxy'])
            # First maximize the number of one-to-one matches, then their IoU.
            # A unit bonus can trade away one match for several higher overlaps.
            if g['label']==label(d['source_class_name']) and overlap>=.5:
                weights[i,j]=min(len(ground_truth),len(detections))+1+overlap
    rows,cols=linear_sum_assignment(-weights)
    return [int(i) for i,j in zip(rows,cols) if weights[i,j]>0]


def boxes(detections):
    return [dict(source_class_name=d.source_class_name,confidence=d.confidence,
                 xyxy=[d.xmin,d.ymin,d.xmax,d.ymax]) for d in detections]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--mask-config',type=Path,default=ROOT/'src/asmc_camera/config/camera/yolo_hood_mask_v1.yaml')
    parser.add_argument('--hood-bag',type=Path)
    parser.add_argument('--hood-reference',type=Path,help='fixed unmasked observations; known hood region is separately visually verified')
    args=parser.parse_args()
    if bool(args.hood_bag)!=bool(args.hood_reference):parser.error('hood bag and reference must be supplied together')
    args.output.mkdir(parents=True,exist_ok=True)
    manifest=json.loads(args.manifest.read_text());mask=HoodMask.load(True,args.mask_config)
    detector=UltralyticsDetector(str(ROOT/'src/asmc_camera/models/object_detection/yolo11m.pt'),
        expected_sha256='d5ffc1a674953a08e11a8d21e022781b1b23a19b730afc309290bd9fb5305b95',
        predict_overrides=dict(iou=.7,max_det=300,agnostic_nms=False))
    records=[];misses=[];counts={k:dict(gt=0,baseline_tp=0,mask_tp=0,new_misses=0) for k in ('person','vehicle')}
    timings={k:[] for k in ('full_inference_ms','mask_inference_ms','mask_ms')}
    for index,f in enumerate(manifest['frames']):
        raw=Path(f['image']).read_bytes();image=cv2.imdecode(np.frombuffer(raw,np.uint8),1)
        if image is None:raise ValueError('undecodable evaluation image '+f['image'])
        before=time.perf_counter();masked=mask.apply(image);timings['mask_ms'].append((time.perf_counter()-before)*1000)
        predictions={}
        # Alternate order to avoid assigning all first-call overhead to one path.
        for path,im in ([('full',image),('mask',masked)] if index%2==0 else [('mask',masked),('full',image)]):
            detections,stages=detector.detect(im);predictions[path]=boxes(detections)
            timings[path+'_inference_ms'].append(stages['inference_ms'])
        gt=f['ground_truth'];full=set(matching(gt,predictions['full']));new=set(matching(gt,predictions['mask']))
        lost=sorted(full-new)
        for j,g in enumerate(gt):
            c=counts[g['label']];c['gt']+=1;c['baseline_tp']+=int(j in full);c['mask_tp']+=int(j in new);c['new_misses']+=int(j in lost)
        row=dict(**f,image_sha256=hashlib.sha256(raw).hexdigest(),full=predictions['full'],mask=predictions['mask'],new_misses=lost)
        records.append(row)
        if lost:
            misses.append(dict(frame=index,image=f['image'],lost=[gt[j] for j in lost]))
            out=image.copy()
            for j in lost:
                b=list(map(int,gt[j]['xyxy']));cv2.rectangle(out,tuple(b[:2]),tuple(b[2:]),(0,0,255),2)
            cv2.imwrite(str(args.output/('loss-%03d.png'%index)),out)
        if (index+1)%50==0:print('GT frames',index+1,'new misses',len(misses),flush=True)
    hood=[]
    if args.hood_bag:
        import rosbag
        reference={f['stamp_ns']:f for f in json.loads(args.hood_reference.read_text())}
        with rosbag.Bag(str(args.hood_bag)) as bag:
            for _,message,_ in bag.read_messages(topics=['/image_jpeg/compressed']):
                old=reference.get(message.header.stamp.to_nsec())
                if old is None:raise ValueError('missing fixed reference stamp')
                image=cv2.imdecode(np.frombuffer(message.data,np.uint8),1);h,w=image.shape[:2]
                def is_hood(name,b):
                    return name=='car' and b[2]-b[0]>=w*.6 and (b[1]+b[3])*.5>=h*.85 and b[3]>=h*.95
                old_hood=[b for b in old['boxes'] if is_hood(b[0],b[1:])]
                if not old_hood:continue
                ds,_=detector.detect(mask.apply(image))
                remaining=[d for d in boxes(ds) if is_hood(d['source_class_name'],d['xyxy'])]
                hood.append(dict(stamp_ns=message.header.stamp.to_nsec(),baseline_boxes=old_hood,mask_boxes=boxes(ds),remaining_hood=remaining))
        (args.output/'hood.json').write_text(json.dumps(hood,indent=2)+'\n')
    (args.output/'frames.json').write_text(json.dumps(records,indent=2)+'\n')
    result=dict(mask=mask.info,manifest_sha256=hashlib.sha256(args.manifest.read_bytes()).hexdigest(),
                model=detector.info,frames=len(records),counts=counts,new_misses=misses,
                hood_positive_frames=len(hood),hood_remaining_frames=sum(bool(r['remaining_hood']) for r in hood),
                timing={k:dict(p50_ms=float(np.percentile(v,50)),p95_ms=float(np.percentile(v,95))) for k,v in timings.items()},
                real_object_gate=all(c['baseline_tp']>0 and c['new_misses']==0 for c in counts.values()),
                hood_gate=bool(hood) and not any(r['remaining_hood'] for r in hood))
    result['pass']=result['real_object_gate'] and result['hood_gate']
    (args.output/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2),flush=True)
    return 0 if result['pass'] else 1


if __name__=='__main__':raise SystemExit(main())
