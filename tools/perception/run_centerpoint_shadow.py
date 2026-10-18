#!/usr/bin/env python3
"""Official MMDetection3D CenterPoint-Pillar single-scan domain-transfer probe."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import time
import numpy as np
import torch
import mmdet3d
from mmdet3d.apis import init_model,inference_detector
from mmengine.config import Config


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--inputs',type=Path,required=True)
    p.add_argument('--weights',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--limit',type=int);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    torch.set_num_threads(2);torch.manual_seed(0);np.random.seed(0)
    config=Path(mmdet3d.__file__).parent/'.mim/configs/centerpoint/centerpoint_pillar02_second_secfpn_head-circlenms_8xb4-cyclic-20e_nus-3d.py'
    cfg=Config.fromfile(config)
    # Explicitly probe one scan. Never duplicate the current scan as fake 10-sweep input.
    cfg.test_dataloader.dataset.pipeline=[step for step in cfg.test_dataloader.dataset.pipeline
        if step['type']!='LoadPointsFromMultiSweeps']
    model=init_model(cfg,str(a.weights),device='cuda:0')
    checkpoint=torch.load(a.weights,map_location='cpu')
    model.load_state_dict(checkpoint['state_dict'],strict=True)
    model.eval();torch.cuda.reset_peak_memory_stats()
    meta=json.loads((a.inputs/'manifest.json').read_text());scans=meta['rows'][:a.limit] if a.limit else meta['rows']
    classes=list(checkpoint['meta']['CLASSES']);times=[];counts=Counter();by_threshold={str(t):Counter() for t in (.1,.4,.7)}
    for i,scan in enumerate(scans):
        with np.load(a.inputs/f'{scan["stamp_ns"]}.npz') as data:xyzi=data['xyzi'].copy()
        # Model feature 5 is sweep time offset; zero is correct for this single current scan.
        points=np.column_stack((xyzi,np.zeros(len(xyzi),np.float32))).astype(np.float32)
        if i==0:
            for _ in range(3):inference_detector(model,points.copy())
            torch.cuda.synchronize()
        start=time.perf_counter();sample,_=inference_detector(model,points)
        torch.cuda.synchronize();latency=(time.perf_counter()-start)*1000
        prediction=sample.pred_instances_3d
        boxes=prediction.bboxes_3d.tensor.cpu().numpy();scores=prediction.scores_3d.cpu().numpy();labels=prediction.labels_3d.cpu().numpy()
        detections=[]
        for box,score,label in zip(boxes,scores,labels):
            name=classes[int(label)];counts[name]+=1
            for threshold,count in by_threshold.items():
                if score>=float(threshold):count[name]+=1
            detections.append(dict(label=name,score=float(score),box=box.tolist()))
        record=dict(scan,latency_ms=latency,detections=detections)
        with (a.output/'observations.jsonl').open('a') as stream:stream.write(json.dumps(record)+'\n')
        times.append(latency)
        if (i+1)%40==0:print('centerpoint-pillar',i+1,'/',len(scans),flush=True)
    summary=dict(model='CenterPoint-Pillar',scans=len(scans),classes=classes,class_detections=dict(counts),
        by_score_threshold={t:dict(v) for t,v in by_threshold.items()},
        latency_ms={f'p{q}':float(np.percentile(times,q)) for q in (50,95,99)},
        torch_peak_allocated_mib=torch.cuda.max_memory_allocated()/2**20,torch_peak_reserved_mib=torch.cuda.max_memory_reserved()/2**20,
        weights_sha256=hashlib.sha256(a.weights.read_bytes()).hexdigest(),strict_checkpoint=True,
        config=str(config),config_sha256=hashlib.sha256(config.read_bytes()).hexdigest(),resolved_config=cfg.to_dict(),
        mmdet3d=mmdet3d.__version__,torch=torch.__version__,precision='official FP32',
        input_features='raw x/y/z/intensity + current-scan time offset 0',
        box_convention='LiDARInstance3DBoxes: raw sensor x/y/bottom_z/dx/dy/dz/yaw/vx/vy; no base_link transform',
        training_domain='nuScenes 10 sweeps; this run only one VLP16 scan; no box class',
        velocity_valid=False,object_gt=False,
        scope='isolated model process, shared GPU with other chat; no end-to-end/real-time/velocity acceptance')
    (a.output/'summary.json').write_text(json.dumps(summary,indent=2,default=str)+'\n')
    print(json.dumps({k:v for k,v in summary.items() if k!='resolved_config'}),flush=True)


if __name__=='__main__':main()
