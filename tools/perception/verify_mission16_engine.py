#!/usr/bin/env python3
"""Check paired native-space one2one outputs before/after TensorRT."""
import json,sys,hashlib,gc,argparse
from pathlib import Path
import cv2,yaml,torch,numpy as np
from asmc_camera.detection.yolo_batch_backend import BatchedYoloDetector
from asmc_camera.detection.hood_mask import HoodMask
from asmc_camera.detection.output_nms import iou

def main():
 p=argparse.ArgumentParser();p.add_argument('--workspace',type=Path,required=True);p.add_argument('--pt-fp32',action='store_true');a=p.parse_args();w=a.workspace
 rows=[json.loads(s) for s in (w/'evaluation-manifest.jsonl').read_text().splitlines()]
 def path(r):return Path('/data')/r['image'].split('/data/',1)[1] if '/data/' in r['image'] else Path(r['image'])
 chosen=[]
 for v in ('front','left','right','unknown'):
  chosen.extend([r for r in rows if r.get('view','unknown')==v and path(r).is_file()][:12])
 masks={v:HoodMask.load(True,w/'src/asmc_camera/config/camera'/f) for v,f in [('front','yolo_hood_mask_v1.yaml'),('left','yolo_ego_mask_left_v1.yaml'),('right','yolo_ego_mask_right_v1.yaml')]}
 outputs={}
 for backend,config in [('pt','epoch8-pt.yaml'),('trt','epoch8-engine.yaml')]:
  cfg=yaml.safe_load((w/config).read_text())
  if backend=='pt' and a.pt_fp32:cfg['inference_quantize']=None
  b=BatchedYoloDetector(**cfg);out=[]
  for r in chosen:
   v=r.get('view','unknown');v=v if v in masks else 'front';img=cv2.imread(str(path(r)));img=masks[v].apply(img) if r.get('view') in masks else img
   ds=b.infer_images({v:img},[v])[v].detections
   out.append([dict(class_name=d.source_class_name,confidence=d.confidence,xyxy=[d.xmin,d.ymin,d.xmax,d.ymax]) for d in ds])
  # Check active subsets and true batch2/batch3 packing with all three cameras.
  batch_checks=[]
  for order in (['left'],['right','front'],['left','right','front']):
   image=cv2.imread(str(path(chosen[0])));ims={v:cv2.resize(image,(1280,720) if v=='front' else (640,480)) for v in order}
   result=b.infer_images(ims,order);assert list(result)==[v for group in __import__('asmc_camera.detection.batching',fromlist=['batch_groups']).batch_groups(order,'2plus1') for v in group]
   batch_checks.append(dict(order=order,outputs=list(result)))
  outputs[backend]=dict(info=b.info,predictions=out,batch_checks=batch_checks)
  del b;gc.collect();torch.cuda.empty_cache()
 failures=[];boundary=[];pairs=[]
 for r,aa,bb in zip(chosen,outputs['pt']['predictions'],outputs['trt']['predictions']):
  options=sorted([(iou(x['xyxy'],y['xyxy']),i,j) for i,x in enumerate(aa) for j,y in enumerate(bb) if x['class_name']==y['class_name']],reverse=True);useda=set();usedb=set()
  for overlap,i,j in options:
   if overlap<.5 or i in useda or j in usedb:continue
   useda.add(i);usedb.add(j);diff=abs(aa[i]['confidence']-bb[j]['confidence']);entry=dict(id=r['id'],label=aa[i]['class_name'],iou=overlap,confidence_difference=diff)
   pairs.append(entry)
   if overlap<.98 or diff>.02:failures.append(entry)
  for side,boxes,used in [('pt',aa,useda),('trt',bb,usedb)]:
   for i,b in enumerate(boxes):
    if i not in used:
     entry=dict(id=r['id'],side=side,box=b)
     (boundary if abs(b['confidence']-.4)<=.02 else failures).append(entry)
 report=dict(images=len(chosen),pairs=pairs,failures=failures,threshold_boundary=boundary,passed=not failures,outputs=outputs)
 (w/('engine-parity-fp32-reference.json' if a.pt_fp32 else 'engine-parity.json')).write_text(json.dumps(report,indent=2));print(json.dumps(dict(images=len(chosen),pairs=len(pairs),failures=len(failures),boundary=len(boundary),passed=not failures)))
if __name__=='__main__':main()
