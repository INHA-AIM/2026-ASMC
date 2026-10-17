#!/usr/bin/env python3
"""CPU-only inference for pinned contiguous windows; leaves live GPU alone."""
import argparse
import hashlib
import json
from pathlib import Path
import time


def main():
    import cv2
    import numpy as np
    import torch
    from asmc_camera.camera.backends.twinlite import TwinLiteBackend
    from asmc_camera.camera.inference_engine import InferenceEngine
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--inputs', type=Path, required=True)
    parser.add_argument('--checkpoint', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    torch.set_num_threads(2)
    cv2.setNumThreads(1)
    args.output.mkdir(parents=True, exist_ok=False)
    index = {(r['run_id'], r['front_ns'], r['view']): r
             for r in map(json.loads, (args.inputs / 'rgb-index.jsonl').read_text().splitlines())}
    rows = list(map(json.loads, (args.inputs / 'continuous.jsonl').read_text().splitlines()))
    backend = TwinLiteBackend(checkpoint=str(args.checkpoint), model_config='large', device='cpu',
                              amp=False, input_hw=(480,640), input_hw_by_view={'front':(544,960)})
    engine = InferenceEngine(backend)
    manifest = []
    for i, row in enumerate(rows):
        stamp = row['views']['front']['source_ns']
        images = {}
        for view in ('front','left','right'):
            src = index[row['run_id'],stamp,view]
            blob = (args.inputs / src['path']).read_bytes()
            if hashlib.sha256(blob).hexdigest() != src['sha256']:
                raise ValueError('RGB SHA mismatch')
            images[view] = cv2.imdecode(np.frombuffer(blob,np.uint8),cv2.IMREAD_COLOR)
        started = time.perf_counter()
        prediction = engine.infer(images,native_outputs={v:set() for v in images})
        values = {}
        for view, result in prediction.views.items():
            values[view+'_class'] = result.model_grid_probabilities['road_marking'].astype(np.uint8)
            values[view+'_confidence'] = np.rint(np.clip(
                result.model_grid_categorical_confidences['road_marking'],0,1)*255).astype(np.uint8)
        path = args.output / (row['run_id']+'-'+str(stamp)+'.npz')
        np.savez_compressed(path,**values)
        manifest.append(dict(row,grid=path.name,grid_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                             inference_ms=1000*(time.perf_counter()-started)))
        if i%12==11:
            print('CPU grids',i+1,'/',len(rows),flush=True)
    (args.output/'manifest.jsonl').write_text(''.join(json.dumps(r,sort_keys=True)+'\n' for r in manifest))
    (args.output/'summary.json').write_text(json.dumps(dict(
        checkpoint_sha256=hashlib.sha256(args.checkpoint.read_bytes()).hexdigest(),
        backend='pytorch_fp32_cpu',threads=2,triplets=len(rows),
        input_hw_by_view={'front':[544,960],'left':[480,640],'right':[480,640]},
        note='CPU candidate grids; not new TensorRT semantic parity or live acceptance.'),indent=2)+'\n')


if __name__ == '__main__':
    main()
