#!/usr/bin/env python3
"""Paired native-GT validation of the exact TwinLite deployment checkpoint."""
import argparse
from collections import defaultdict
from contextlib import ExitStack
import json
from pathlib import Path
import time

import cv2
import numpy as np

from asmc_camera.camera.backends.twinlite import TwinLiteBackend
from asmc_camera.camera.backends.twinlite_tensorrt import TwinLiteTensorRTBackend
from asmc_camera.camera.inference_engine import InferenceEngine
from asmc_camera.runtime.gpu_input import RawGpuCache
from asmc_camera.runtime.tensorrt import sha256_file


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--checkpoint', type=Path, required=True)
    p.add_argument('--engine-manifest', type=Path, required=True)
    p.add_argument('--validation', type=Path, default=Path('/data/dataset_versions/twinlite_morai_v6_road_marking_actor_negative/val.jsonl'))
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--limit', type=int, default=0, help='0=all validation frames; positive=first N triplets')
    p.add_argument('--gpu-preprocess', action='store_true')
    p.add_argument('--family-fixture', type=Path, help='Pinned RGB/pose manifest; compare the production family pipeline on an isolated ROS master')
    p.add_argument('--data-root', type=Path, default=Path('/data'))
    a=p.parse_args(); a.output.mkdir(parents=True,exist_ok=False)
    kwargs=dict(checkpoint=str(a.checkpoint),input_hw=(480,640),input_hw_by_view={'front':(544,960)})
    baseline=TwinLiteBackend(**kwargs,channels_last=True)
    candidate=TwinLiteTensorRTBackend(**kwargs,engine_manifest=str(a.engine_manifest))
    engines=[InferenceEngine(baseline),InferenceEngine(candidate)]
    gpu=RawGpuCache() if a.gpu_preprocess else None
    if a.family_fixture:
        from shadow_family_parity import run
        try:
            return run(a, engines, gpu)
        finally:
            if gpu: gpu.close()
    groups=defaultdict(dict)
    for row in map(json.loads,a.validation.read_text().splitlines()):
        groups[(row['run_id'],row['frame_id'])][row['view']]=row
    cases=sorted(groups.items())
    if a.limit: cases=cases[:a.limit]
    counts=defaultdict(lambda:np.zeros((5,3),np.int64))
    samples=[]; timing=[]; render=[]; batch1=[]
    for index,((run,frame),rows) in enumerate(cases):
        if set(rows) != {'front','left','right'}: raise ValueError('validation lacks a full three-view set')
        images={v:cv2.imread('/data/'+r['image']) for v,r in rows.items()}
        if any(im is None for im in images.values()): raise ValueError('missing validation RGB')
        starts=time.perf_counter()
        reference=engines[0].infer(images)
        base_ms=(time.perf_counter()-starts)*1000
        starts=time.perf_counter()
        with ExitStack() as leases:
            inputs={v:leases.enter_context(gpu.lease(v,(run,frame),image)) for v,image in images.items()} if gpu else None
            predicted=engines[1].infer(images,gpu_inputs=inputs)
        timing.append(dict(reference_ms=base_ms,candidate_ms=(time.perf_counter()-starts)*1000,
                           backend=predicted.backend_stages))
        if index == 0:
            # A stopped side camera must not prevent the surviving B1 path.
            saved = predicted.views['left'].probabilities['road_marking'].copy()
            for view in ('left', 'right'):
                solo = engines[1].infer({view: images[view]}).views[view]
                paired = predicted.views[view]
                delta = float(np.percentile(np.abs(solo.probabilities['drivable'] - paired.probabilities['drivable']), 99))
                mismatch = float(np.mean(solo.probabilities['road_marking'] != paired.probabilities['road_marking']))
                batch1.append(dict(view=view, probability_abs_p99=delta, class_mismatch=mismatch,
                                   pass_=delta <= .02 and mismatch <= .005))
            if not np.array_equal(saved, predicted.views['left'].probabilities['road_marking']):
                raise ValueError('a later backend call modified an earlier prediction')
        for view,row in rows.items():
            x,y=reference.views[view],predicted.views[view]
            gtroot=Path('/data/datasets')/run/'derived/perception_targets_native_v3_actor_negative'
            filename=Path(row['image']).name
            targets=[cv2.imread(str(gtroot/folder/view/filename),0)
                     for folder in ('drivable_masks','road_marking_masks','drivable_ignore_masks')]
            if any(t is None for t in targets): raise ValueError('missing native GT: '+row['sample_id'])
            da,mark,ignore=targets
            if any(t.shape!=images[view].shape[:2] for t in targets): raise ValueError('native GT geometry mismatch')
            differences=[float(np.percentile(np.abs(x.probabilities['drivable']-y.probabilities['drivable']),99)),
                         float(np.percentile(np.abs(x.categorical_confidences['road_marking']-y.categorical_confidences['road_marking']),99))]
            mismatch=float(np.mean(x.probabilities['road_marking']!=y.probabilities['road_marking']))
            sample=dict(id=row['sample_id'],image_sha256=sha256_file(Path('/data')/row['image']),
                        probability_abs_p99=differences,class_mismatch=mismatch,
                        pass_=max(differences)<=.02 and mismatch<=.005)
            samples.append(sample)
            for name,result in [('pytorch',x),('tensorrt',y)]:
                for h,(truth,pred) in enumerate(zip([da>0]+[mark==c for c in range(4)],
                    [result.probabilities['drivable']>=.5]+[result.probabilities['road_marking']==c for c in range(4)])):
                    valid=ignore==0 if h==0 else mark<4
                    delta=[np.count_nonzero(pred&truth&valid),np.count_nonzero((pred|truth)&valid),np.count_nonzero(truth&valid)]
                    for group in (view,run+':'+view): counts[(name,group)][h]+=delta
            if index%max(1,len(cases)//18)==0 or not sample['pass_']:
                palette=np.array([[0,0,0],[255,255,0],[0,255,255],[0,0,255]],np.uint8)
                panels=[]
                for title,mask in [('GT',mark),('PyTorch',x.probabilities['road_marking']),('TensorRT',y.probabilities['road_marking'])]:
                    panel=images[view].copy(); active=(mask>0)&(mask<4)
                    overlay=palette[np.clip(mask,0,3)]
                    panel[active]=cv2.addWeighted(panel,.55,overlay,.45,0)[active]
                    panel=cv2.resize(panel,(480,270 if view=='front' else 360))
                    cv2.putText(panel,title,(8,23),cv2.FONT_HERSHEY_SIMPLEX,.65,(0,255,0),2)
                    panels.append(panel)
                name='{:04d}-{}.jpg'.format(index,view)
                cv2.imwrite(str(a.output/name),np.hstack(panels));render.append(dict(file=name,**sample))
        if index%25==0: print('validated {}/{} triplets'.format(index+1,len(cases)),flush=True)
    quality=[]
    for group in sorted({key[1] for key in counts}):
        x,y=counts[('pytorch',group)],counts[('tensorrt',group)]
        bx=x[:,0]/np.maximum(1,x[:,1]);by=y[:,0]/np.maximum(1,y[:,1])
        quality.append(dict(group=group,reference_iou=bx.tolist(),candidate_iou=by.tolist(),
                            gt_pixels=x[:,2].tolist(),iou_drop=(bx-by).tolist(),
                            pass_=bool(np.all((bx-by)[x[:,2]>0]<=.005))))
    result=dict(schema='twinlite_trt_validation_v1',images=len(samples),triplets=len(cases),
        checkpoint_sha256=sha256_file(a.checkpoint),engine_manifest_sha256=sha256_file(a.engine_manifest),
        validation_sha256=sha256_file(a.validation),gpu_preprocess=a.gpu_preprocess,
        samples=samples,quality=quality,timing=timing,
        side_batch1=batch1,batch1_pass=all(r['pass_'] for r in batch1),
        numerical_pass=all(r['pass_'] for r in samples),quality_pass=all(r['pass_'] for r in quality),
        scope='Native GT, fixed v6 validation; repeated adjacent frames are not independent scenes.')
    (a.output/'report.json').write_text(json.dumps(result,indent=2)+'\n')
    html='<meta charset="utf-8"><title>TwinLite TensorRT 검증</title><h1>PyTorch / TensorRT native GT 비교</h1>'
    html+=''.join('<p>{}</p><img loading="lazy" width="1440" src="{}">'.format(r['id'],r['file']) for r in render)
    (a.output/'index.html').write_text(html)
    if gpu:gpu.close()
    print(json.dumps({k:result[k] for k in ('images','numerical_pass','quality_pass')},indent=2))
    return 0 if all(result[k] for k in ('numerical_pass','quality_pass','batch1_pass')) else 1


if __name__=='__main__':raise SystemExit(main())
