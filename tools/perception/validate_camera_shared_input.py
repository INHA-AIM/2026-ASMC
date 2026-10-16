#!/usr/bin/env python3
"""Stored RGB/GT parity and side384/480 comparison; no ROS/control/weight updates."""
import argparse
from collections import defaultdict
from dataclasses import replace
import gc
import hashlib
import json
from pathlib import Path
import time

import cv2
import numpy as np
import torch

from asmc_camera.camera.backends.twinlite import TwinLiteBackend
from asmc_camera.camera.geometry import letterbox_bgr
from asmc_camera.camera.inference_engine import InferenceEngine
from asmc_camera.detection.batching import pack_image
from asmc_camera.detection.hood_mask import HoodMask
from asmc_camera.detection.yolo_batch_backend import BatchedYoloDetector, COCO7
from asmc_camera.detection.mission import CLASSES
from asmc_camera.runtime.gpu_input import RawGpuCache, PreprocessSpec, prepare


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def paths_in_container(value):
    marker = '/2026-ai-sw-mobility-competition/data/'
    return Path('/data/' + value.split(marker, 1)[1]) if marker in value else Path(value)


def compare(a, b):
    pairs, unmatched = [], []
    remaining = list(b)
    for x in a:
        candidates = [(i, y) for i, y in enumerate(remaining) if x.source_class_name == y.source_class_name]
        def iou(y):
            inter = max(0, min(x.xmax,y.xmax)-max(x.xmin,y.xmin))*max(0,min(x.ymax,y.ymax)-max(x.ymin,y.ymin))
            union = (x.xmax-x.xmin)*(x.ymax-x.ymin)+(y.xmax-y.xmin)*(y.ymax-y.ymin)-inter
            return inter/union if union else 0.
        if candidates:
            i, y = max(candidates, key=lambda pair: iou(pair[1]))
            overlap = iou(y)
            if overlap >= .5:
                pairs.append(dict(iou=overlap, confidence_delta=abs(x.confidence-y.confidence)))
                remaining.pop(i)
                continue
        unmatched.append(x)
    boundary = [x for x in unmatched+remaining if abs(x.confidence-.4) <= .02]
    return dict(matched=len(pairs), min_iou=min([r['iou'] for r in pairs], default=1.),
                max_confidence_delta=max([r['confidence_delta'] for r in pairs], default=0.),
                unmatched=len(unmatched)+len(remaining), threshold_boundary=len(boundary),
                pass_=all(r['iou'] >= .98 and r['confidence_delta'] <= .02 for r in pairs)
                    and len(unmatched)+len(remaining) == len(boundary))


def main():
    from contextlib import ExitStack
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--validation-manifest', type=Path)
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=False)
    package = a.root/'src/asmc_camera'
    data = Path('/data')
    results = dict(scope='Stored independent RGB and native GT; MORAI background load may remain. No acceptance throughput claim.',
                   direct_parity=[], gpu_detection_parity=[], preprocess=[], twin_parity=[], gt={})
    source = data/'dataset_versions/twinlite_morai_v6_road_marking_actor_negative/val.jsonl'
    runs = defaultdict(lambda: defaultdict(dict))
    for row in map(json.loads, source.read_text().splitlines()):
        runs[row['run_id']][row['frame_id']][row['view']] = row
    batches = []
    for run, frames in sorted(runs.items()):
        keys = sorted(k for k, rows in frames.items() if set(rows) == {'front','left','right'})
        for index in sorted(set([0, len(keys)//2, len(keys)-1])):
            rows = frames[keys[index]]
            images = {v:cv2.imread(str(data/r['image'])) for v,r in rows.items()}
            if any(image is None for image in images.values()):
                raise ValueError('missing stored RGB')
            batches.append((run, rows, images))
    results['input_manifest_sha256'] = digest(source)
    results['input_images'] = [dict(id=r['sample_id'], sha256=digest(data/r['image']))
                                for _, rows, _ in batches for r in rows.values()]
    masks = {v:HoodMask.load(True, package/'config/camera'/f) for v,f in dict(
        front='yolo_hood_mask_v1.yaml', left='yolo_ego_mask_left_v1.yaml', right='yolo_ego_mask_right_v1.yaml').items()}
    cuda = RawGpuCache()
    # Confirm normalized pixels independently of either network.
    for run, rows, images in batches:
        for view, image in images.items():
            shape = (544,960) if view == 'front' else (480,640)
            with cuda.lease(view, rows[view]['sample_id'], image) as raw:
                lane, _, _ = prepare(raw, PreprocessSpec(shape, 'area'))
                expected, _ = letterbox_bgr(image, shape)
                lane_error = float(np.max(np.abs(lane.cpu().numpy()-expected))*255.)
                obj, _, _ = prepare(raw, PreprocessSpec(shape, 'linear', masks[view].config_sha256, 'float16'), masks[view])
                packed, _ = pack_image(masks[view].apply(image), view, shapes={view:shape})
                expected_obj = torch.from_numpy(packed.copy()).permute(2,0,1).flip(0).half().div_(255.).numpy()
                results['preprocess'].append(dict(id=rows[view]['sample_id'], lane_max_uint8_error=lane_error,
                    objects_max_normalized_error=float(np.max(np.abs(obj.cpu().numpy()-expected_obj)))))
    for profile, allowed, head in [('finetuned_e8', CLASSES, 'mission16_one2one_nms'),
                                 ('pretrained_coco7', COCO7, 'legacy_coco7')]:
        weights = package/'models/object_detection'/('yolo26l-'+profile+'.engine')
        kwargs = dict(weights=str(weights), expected_sha256=digest(weights), allowed_names=allowed,
                      conf_thres=.4, front_width=960, head_profile=head, nms_iou=.5, runtime_backend='tensorrt')
        old = BatchedYoloDetector(**kwargs)
        new = BatchedYoloDetector(direct_detect=True, **kwargs)
        # Cover all 205 development images, including new obstacle kinds.
        val = a.validation_manifest or data/'dataset_versions/camera_objects_bbox_validation16_v1-r2/manifest.jsonl'
        for index, row in enumerate(map(json.loads, val.read_text().splitlines())):
            view = row.get('view')
            view = view if view in masks else 'left'
            image = cv2.imread(str(paths_in_container(row['image'])))
            if image is None:
                raise ValueError('missing validation RGB')
            # Unknown-view Roboflow data must not receive a guessed ego mask.
            if row.get('view') in masks:
                image = masks[view].apply(image)
            x = old.infer_images({view:image}, [view])[view].detections
            y = new.infer_images({view:image}, [view])[view].detections
            results['direct_parity'].append(dict(profile=profile, image=row['id'], **compare(x,y)))
            if index % 50 == 0:
                print('direct', profile, index, flush=True)
        for run, rows, images in batches:
            (a.output/'partial.json').write_text(json.dumps(results,indent=2)+'\n')
            with ExitStack() as stack:
                gpu = {v:stack.enter_context(cuda.lease(v,rows[v]['sample_id'],im)) for v,im in images.items()}
                x = new.infer_images({v:masks[v].apply(im) for v,im in images.items()},list(images))
                y = new.infer_images(images,list(images),gpu_frames=gpu,gpu_masks=masks)
                for view in images:
                    results['gpu_detection_parity'].append(dict(profile=profile, image=rows[view]['sample_id'],
                        **compare(x[view].detections,y[view].detections)))
        del old, new
        gc.collect()  # offline model teardown only; runtime GC policy is untouched.
        torch.cuda.empty_cache()
    twin = TwinLiteBackend(checkpoint=str(a.root/'artifacts/perception_eval/runs/twinlite_medium_morai_v6_road_marking_init_b_b12_e18_20260908/best_mean.pt'),
        input_hw=(480,640), input_hw_by_view={'front':(544,960)}, channels_last=True)
    engine = InferenceEngine(twin)
    counts = defaultdict(lambda:np.zeros((5,3),np.int64))
    for run, rows, images in batches:
        with ExitStack() as stack:
            gpu = {v:stack.enter_context(cuda.lease(v,rows[v]['sample_id'],im)) for v,im in images.items()}
            native480 = engine.infer(images)
            gpu480 = engine.infer(images,gpu_inputs=gpu)
        twin._info = replace(twin.info, input_hw=(384,640))
        native384 = engine.infer(images)
        twin._info = replace(twin.info, input_hw=(480,640))
        for view, row in rows.items():
            x,y = native480.views[view], gpu480.views[view]
            results['twin_parity'].append(dict(id=row['sample_id'],
                drivable_mean_abs=float(np.mean(np.abs(x.probabilities['drivable']-y.probabilities['drivable']))),
                drivable_max_abs=float(np.max(np.abs(x.probabilities['drivable']-y.probabilities['drivable']))),
                marking_mismatch=float(np.mean(x.probabilities['road_marking'] != y.probabilities['road_marking']))))
            gtroot = data/'datasets'/run/'derived/perception_targets_native_v3_actor_negative'
            filename = Path(row['image']).name
            drivable = cv2.imread(str(gtroot/'drivable_masks'/view/filename),0)
            marking = cv2.imread(str(gtroot/'road_marking_masks'/view/filename),0)
            ignore = cv2.imread(str(gtroot/'drivable_ignore_masks'/view/filename),0)
            if drivable is None or marking is None or ignore is None:
                raise ValueError('missing native validation GT')
            if rows['front']['frame_id'] == min(r['frame_id'] for rr, rs, _ in batches if rr == run for r in rs.values()):
                panels = []
                masks_for_gallery = [None, marking,
                    native384.views[view].probabilities['road_marking'],
                    native480.views[view].probabilities['road_marking']]
                palette = np.array([[0,0,0],[255,255,0],[0,255,255],[0,0,255]],np.uint8)
                for title, mask in zip(['RGB','Native GT','Side384 control','Side480 candidate'], masks_for_gallery):
                    panel = images[view].copy()
                    if mask is not None:
                        valid = mask > 0
                        panel[valid] = cv2.addWeighted(panel,.6,palette[mask],.4,0)[valid]
                    panel = cv2.resize(panel,(480,270 if view=='front' else 360))
                    cv2.putText(panel,title,(8,24),cv2.FONT_HERSHEY_SIMPLEX,.65,(0,255,0),2)
                    panels.append(panel)
                cv2.imwrite(str(a.output/(run+'-'+view+'.jpg')),np.hstack(panels))
            for name, prediction in [('side384',native384),('side480',native480),('gpu480',gpu480)]:
                pr = prediction.views[view].probabilities
                targets = [drivable>0] + [marking==c for c in range(4)]
                preds = [pr['drivable'] >= .5] + [pr['road_marking']==c for c in range(4)]
                for h,(target,pred) in enumerate(zip(targets,preds)):
                    valid = ignore==0 if h==0 else np.ones_like(target,bool)
                    counts[(name,view)][h] += [np.count_nonzero(pred&target&valid),
                        np.count_nonzero((pred|target)&valid), np.count_nonzero(target&valid)]
    results['gt'] = {name+':'+view:dict(drivable_iou=float(c[0,0]/max(1,c[0,1])),
        marking_iou=[float(x[0]/max(1,x[1])) for x in c[1:]], target_pixels=c[:,2].tolist())
        for (name,view),c in counts.items()}
    results['gpu_cache'] = dict(uploads=cuda.uploads,hits=cuda.hits,bytes=cuda.bytes,busy=cuda.busy)
    cuda.close()
    results['direct_pass'] = all(r['pass_'] for r in results['direct_parity'])
    results['gpu_detection_pass'] = all(r['pass_'] for r in results['gpu_detection_parity'])
    results['twin_gpu_pass'] = all(r['drivable_mean_abs'] <= .001 and r['marking_mismatch'] <= .001 for r in results['twin_parity'])
    (a.output/'report.json').write_text(json.dumps(results,indent=2)+'\n')
    print(json.dumps({k:v for k,v in results.items() if k.endswith('pass') or k in ('gt','gpu_cache')},indent=2),flush=True)


if __name__ == '__main__':
    main()
