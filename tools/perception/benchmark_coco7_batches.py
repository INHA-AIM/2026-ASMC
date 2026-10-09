#!/usr/bin/env python3
"""Fixed real RGB, full backend-call wall timing; NOT ROS Hz or pure kernel timing."""
import argparse
import gc
import hashlib
import json
from pathlib import Path
import statistics
import time

MODELS = ('yolo26m', 'yolo26l', 'yolo26m-seg', 'yolo26l-seg')
VIEWS = ['front', 'left', 'right']


def planned_pixels(width, mode):
    from asmc_camera.detection.batching import batch_groups, shapes_for_front
    shapes=shapes_for_front(width)
    total=0
    for group in batch_groups(VIEWS,mode):
        h=max(shapes[v][0] for v in group)
        w=max(shapes[v][1] for v in group)
        total+=h*w*len(group)
    return total


def stage_totals(results, groups):
    """A shared batch timer is present on EACH view; count it just once."""
    total=dict(inference_ms=0., mask_transfer_ms=0.)
    for group in groups:
        first=results[group[0]].timings
        if int(first['batch_size']) != len(group):
            raise ValueError('batch timing size mismatch')
        for view in group:
            current=results[view].timings
            if any(current[key] != first[key] for key in ('inference_ms','batch_size')):
                raise ValueError('inconsistent shared batch timing')
        total['inference_ms']+=first['inference_ms']
        # Backend reads its postprocess clock once per view when assigning
        # timers. Those values differ by microseconds; use the last/max once.
        total['mask_transfer_ms']+=max(results[v].timings['mask_transfer_ms'] for v in group)
    return total


def main():
    import cv2
    import numpy as np
    import torch
    from asmc_camera.detection.batching import batch_groups
    from asmc_camera.detection.hood_mask import HoodMask, load_view_masks
    from asmc_camera.detection.yolo_batch_backend import BatchedYoloDetector
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--images',type=Path,required=True,help='front/left/right.jpg plus manifest.json')
    parser.add_argument('--models',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--revision',required=True)
    parser.add_argument('--cycles',type=int,default=40)
    parser.add_argument('--repeats',type=int,default=3)
    args=parser.parse_args()
    if args.cycles < 10 or args.repeats < 1: parser.error('at least 10 cycles and one repeat required')
    args.output.mkdir(parents=True,exist_ok=False)
    root=Path(__file__).resolve().parents[2]
    masks=load_view_masks(VIEWS,HoodMask.load(True,root/'src/asmc_camera/config/camera/yolo_hood_mask_v1.yaml'),
                         True,{v:root/f'src/asmc_camera/config/camera/yolo_ego_mask_{v}_v1.yaml' for v in VIEWS[1:]})
    sources=json.loads((args.images/'manifest.json').read_text())
    images={}
    for view in VIEWS:
        row=next(r for r in sources if r['view']==view)
        file=args.images/(view+'.jpg')
        if hashlib.sha256(file.read_bytes()).hexdigest()!=row['sha256']:raise ValueError('RGB SHA mismatch')
        raw=cv2.imread(str(file))
        if raw is None:raise ValueError('invalid JPEG')
        images[view]=masks[view].apply(raw)
        row['shape_hw']=list(raw.shape[:2])
    import pynvml
    pynvml.nvmlInit(); gpu=pynvml.nvmlDeviceGetHandleByIndex(0)
    report=dict(revision=args.revision,device=torch.cuda.get_device_name(0),source=sources,
                mask={v:masks[v].info for v in VIEWS},runs=[],
                scope='fixed-image backend wall demand with MORAI running; no ROS throughput or full-stack acceptance',
                confidence=.7,head='one-to-many',normal_output_not_republished=True)
    def save(): (args.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    configs=[(model,width,mode) for model in MODELS for width in (640,960,1280)
             for mode in ('sequential','2plus1','batch3')]
    save()
    try:
        for repeat in range(args.repeats):
            # Rotate configurations to avoid always measuring a model first/last.
            offset=repeat*11%len(configs)
            for model,width,mode in configs[offset:]+configs[:offset]:
                entry=dict(model=model,width=width,batch=mode,repeat=repeat+1,
                           tensor_pixels=planned_pixels(width,mode),samples=[])
                report['runs'].append(entry)
                detector=None
                try:
                    weight=args.models/(model+'.engine')
                    detector=BatchedYoloDetector(weights=str(weight),expected_sha256=hashlib.sha256(weight.read_bytes()).hexdigest(),
                        runtime_backend='tensorrt',task='segment' if 'seg' in model else 'detect',
                        front_width=width,batch_mode=mode,conf_thres=.7)
                    entry['backend']=detector.info
                    for _ in range(5): detector.infer_images(images,VIEWS)
                    groups=batch_groups(VIEWS,mode)
                    for _ in range(args.cycles):
                        torch.cuda.synchronize()
                        start=time.monotonic()
                        outputs=detector.infer_images(images,VIEWS)
                        torch.cuda.synchronize()
                        wall_ms=(time.monotonic()-start)*1000
                        memory=pynvml.nvmlDeviceGetMemoryInfo(gpu)
                        entry['samples'].append(dict(cycle_wall_ms=wall_ms,**stage_totals(outputs,groups),
                            detections={v:len(outputs[v].detections) for v in VIEWS},
                            whole_gpu_used_mib=memory.used/2**20,
                            gpu_utilization=pynvml.nvmlDeviceGetUtilizationRates(gpu).gpu))
                    wall=[s['cycle_wall_ms'] for s in entry['samples']]
                    entry['summary']=dict(mean_ms=statistics.mean(wall),p50_ms=float(np.percentile(wall,50)),
                        p95_ms=float(np.percentile(wall,95)),p99_ms=float(np.percentile(wall,99)),
                        max_ms=max(wall),mean_predict_ms=statistics.mean(s['inference_ms'] for s in entry['samples']),
                        mean_mask_transfer_ms=statistics.mean(s['mask_transfer_ms'] for s in entry['samples']),
                        observed_peak_whole_gpu_mib=max(s['whole_gpu_used_mib'] for s in entry['samples']),
                        mean_cycle_demand_at_15hz=statistics.mean(wall)*15/1000)
                    print(model,width,mode,repeat+1,round(entry['summary']['p50_ms'],2),flush=True)
                except Exception as exc:
                    entry['error']=str(exc)
                    print('FAILED',model,width,mode,repr(exc),flush=True)
                finally:
                    if detector is not None: del detector
                    gc.collect();torch.cuda.empty_cache();save()
    finally:
        pynvml.nvmlShutdown();save()


if __name__=='__main__':main()
