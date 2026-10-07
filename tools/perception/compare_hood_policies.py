#!/usr/bin/env python3
"""Compare raw/mask/post-filter outputs; never infer bonnet truth from a rule."""
import argparse
import hashlib
import json
from pathlib import Path
import cv2
import numpy as np
from asmc_camera.detection.ego_hood_filter import EgoHoodFilter
from asmc_camera.detection.hood_mask import HoodMask
from asmc_camera.detection.ultralytics_backend import UltralyticsDetector
from evaluate_hood_mask import boxes, matching
ROOT=Path(__file__).resolve().parents[2]

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args(); args.output.mkdir(parents=True,exist_ok=True)
    manifest=json.loads(args.manifest.read_text())
    mask=HoodMask.load(True,ROOT/'src/asmc_camera/config/camera/yolo_hood_mask_v1.yaml')
    filt=EgoHoodFilter.load(True,ROOT/'src/asmc_camera/config/camera/yolo_ego_hood_filter_v1.yaml')
    backend=UltralyticsDetector(str(ROOT/'src/asmc_camera/models/object_detection/yolo11m.pt'),
        expected_sha256='d5ffc1a674953a08e11a8d21e022781b1b23a19b730afc309290bd9fb5305b95',
        predict_overrides=dict(iou=.7,max_det=300,agnostic_nms=False))
    rows=[];counts={name:dict(person=0,vehicle=0) for name in ('mask_new_misses','filter_new_misses')}
    exclusions=[]
    for i,frame in enumerate(manifest['frames']):
        image=cv2.imread(frame['image']); h,w=image.shape[:2]
        predictions={}
        for name,im in ([('full',image),('mask',mask.apply(image))] if i%2==0 else [('mask',mask.apply(image)),('full',image)]):
            detections,timings=backend.detect(im); predictions[name]=boxes(detections)
            if name=='full': full_detections=detections
        kept,removed=filt.partition(full_detections,w,h);predictions['filter']=boxes(kept)
        gt=frame.get('ground_truth',[]); raw_matches=set(matching(gt,predictions['full']))
        misses={}
        for name in ('mask','filter'):
            misses[name]=sorted(raw_matches-set(matching(gt,predictions[name])))
            for index in misses[name]:counts[name+'_new_misses'][gt[index]['label']]+=1
        rows.append(dict(**frame,**predictions,new_misses=misses,image_sha256=hashlib.sha256(Path(frame['image']).read_bytes()).hexdigest()))
        for j,b in enumerate(boxes(removed)):
            exclusions.append(dict(frame=i,image=frame['image'],box=b,review='pending_independent_review'))
        if removed or any(misses.values()):
            overlay=image.copy()
            for d in removed:cv2.rectangle(overlay,(int(d.xmin),int(d.ymin)),(int(d.xmax),int(d.ymax)),(0,0,255),2)
            cv2.imwrite(str(args.output/('review-%03d.jpg'%i)),overlay)
        if (i+1)%50==0:print(i+1,counts,flush=True)
    result=dict(frames=len(rows),counts=counts,excluded_boxes=len(exclusions),
        reviewed_exclusions=0,acceptance='pending_independent_exclusion_review',mask=mask.info,
        filter=filt.info,model=backend.info,manifest_sha256=hashlib.sha256(args.manifest.read_bytes()).hexdigest())
    for name,value in [('frames',rows),('excluded-review',exclusions),('summary',result)]:
        (args.output/(name+'.json')).write_text(json.dumps(value,indent=2)+'\n')
    print(json.dumps(result,indent=2))
if __name__=='__main__':main()
