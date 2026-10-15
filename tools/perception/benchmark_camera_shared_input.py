#!/usr/bin/env python3
"""Paired serial diagnostic of real decode/H2D counts; not a live supply test."""
import argparse
from contextlib import ExitStack
import hashlib
import json
from pathlib import Path
import time

import cv2
import numpy as np

from asmc_camera.camera.backends.twinlite import TwinLiteBackend
from asmc_camera.camera.inference_engine import InferenceEngine
from asmc_camera.detection.hood_mask import HoodMask
from asmc_camera.detection.yolo_batch_backend import BatchedYoloDetector
from asmc_camera.detection.mission import CLASSES
from asmc_camera.runtime.shared_input import DecodedCache
from asmc_camera.runtime.gpu_input import RawGpuCache


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--samples',type=int,default=30)
    a=p.parse_args(); package=a.root/'src/asmc_camera'
    rows=[json.loads(l) for l in Path('/data/dataset_versions/twinlite_morai_v6_road_marking_actor_negative/val.jsonl').read_text().splitlines()]
    first=rows[0]
    selected={r['view']:r for r in rows if r['run_id']==first['run_id'] and r['frame_id']==first['frame_id']}
    blobs={v:cv2.imencode('.jpg',cv2.imread('/data/'+r['image']))[1].tobytes() for v,r in selected.items()}
    weights=package/'models/object_detection/yolo26l-finetuned_e8.engine'
    detector=BatchedYoloDetector(str(weights),hashlib.sha256(weights.read_bytes()).hexdigest(),
        allowed_names=CLASSES,conf_thres=.4,runtime_backend='tensorrt',front_width=960,
        head_profile='mission16_one2one_nms',nms_iou=.5,direct_detect=True)
    lane=TwinLiteBackend(str(a.root/'artifacts/perception_eval/runs/twinlite_medium_morai_v6_road_marking_init_b_b12_e18_20260908/best_mean.pt'),
        input_hw=(480,640),input_hw_by_view={'front':(544,960)},channels_last=True)
    engine=InferenceEngine(lane)
    masks={v:HoodMask.load(True,package/'config/camera'/f) for v,f in dict(front='yolo_hood_mask_v1.yaml',left='yolo_ego_mask_left_v1.yaml',right='yolo_ego_mask_right_v1.yaml').items()}
    cache,gpu=DecodedCache(),RawGpuCache()
    results={}
    for phase,shared in [('separate_a',False),('shared_gpu',True),('separate_b',False)]:
        decode_count=0; samples=[]
        def decode(blob):
            nonlocal decode_count
            decode_count+=1
            return cv2.imdecode(np.frombuffer(blob,np.uint8),cv2.IMREAD_COLOR)
        def step(index):
            if not shared:
                images={v:masks[v].apply(decode(blob)) for v,blob in blobs.items()}
                detector.infer_images(images,list(images))
                engine.infer({v:decode(blob) for v,blob in blobs.items()})
                return
            # Independent model demands; retained cache entries match exact keys.
            for role in ('objects','lane'):
                with ExitStack() as leases:
                    images={v:leases.enter_context(cache.lease(v,(phase,index),blob,decode)) for v,blob in blobs.items()}
                    inputs={v:leases.enter_context(gpu.lease(v,(phase,index),image)) for v,image in images.items()}
                    if role=='objects':detector.infer_images(images,list(images),gpu_frames=inputs,gpu_masks=masks)
                    else:engine.infer(images,gpu_inputs=inputs)
        for i in range(3):step(-3+i)
        decode_before=decode_count
        upload_before=detector.input_uploads+lane.input_uploads+gpu.uploads
        byte_before=detector.input_upload_bytes+lane.input_upload_bytes+gpu.bytes
        hits_before=gpu.hits
        for i in range(a.samples):
            wall,cpu=time.perf_counter(),time.process_time()
            step(i)
            samples.append(dict(wall_ms=(time.perf_counter()-wall)*1000,cpu_ms=(time.process_time()-cpu)*1000))
        results[phase]=dict(samples=samples,decode_calls=decode_count-decode_before,
            image_h2d_calls=detector.input_uploads+lane.input_uploads+gpu.uploads-upload_before,
            image_h2d_bytes=detector.input_upload_bytes+lane.input_upload_bytes+gpu.bytes-byte_before,
            shared_upload_hits=gpu.hits-hits_before,
            wall_ms={f'p{q}':float(np.percentile([x['wall_ms'] for x in samples],q)) for q in (50,95,99)},
            cpu_ms_mean=float(np.mean([x['cpu_ms'] for x in samples])))
    gpu.close()
    result=dict(scope='Serial saved-JPEG A/B/A diagnostic, both models loaded; excludes live concurrency, ResNet, GUI and ROS. Image H2D counters exclude cached mask/coordinate constants.',
                samples_per_phase=a.samples,results=results)
    a.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:{n:v for n,v in r.items() if n!='samples'} for k,r in results.items()},indent=2))


if __name__=='__main__':main()
