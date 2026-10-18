#!/usr/bin/env python3
"""Compare two fixed models through the production shadow family path.

Run on an isolated ROS master. This is an offline development comparison, not
live readiness acceptance or a new lane ground-truth measurement.
"""
import argparse
from pathlib import Path

from asmc_camera.camera.backends.twinlite_tensorrt import TwinLiteTensorRTBackend
from asmc_camera.camera.backends.twinlite import TwinLiteBackend
from asmc_camera.camera.inference_engine import InferenceEngine
from asmc_camera.runtime.gpu_input import RawGpuCache

from shadow_family_parity import run


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('checkpoint', 'reference-checkpoint', 'family-fixture', 'output'):
        parser.add_argument('--' + name, type=Path, required=True)
    parser.add_argument('--engine-manifest', type=Path)
    parser.add_argument('--reference-engine-manifest', type=Path)
    parser.add_argument('--backend', choices=['tensorrt', 'pytorch'], default='tensorrt')
    parser.add_argument('--device', choices=['cuda', 'cpu'], default='cuda')
    parser.add_argument('--model-config', choices=['nano', 'small', 'medium', 'large'], default='large')
    parser.add_argument('--reference-model-config', choices=['nano', 'small', 'medium', 'large'], default='medium')
    parser.add_argument('--data-root', type=Path, default=Path('/data'))
    parser.add_argument('--limit', type=int, default=0, help='0=all frames; otherwise per run')
    args = parser.parse_args()
    if args.backend == 'tensorrt' and (args.device != 'cuda' or not args.engine_manifest
                                      or not args.reference_engine_manifest):
        parser.error('TensorRT requires CUDA and both engine manifests')
    import cv2
    import torch
    torch.set_num_threads(2)
    cv2.setNumThreads(1)
    args.output.mkdir(parents=True, exist_ok=False)
    args.reference_label = args.reference_model_config + ' ' + args.backend + ' ' + args.device
    args.candidate_label = args.model_config + ' ' + args.backend + ' ' + args.device
    engines = []
    for checkpoint, manifest, scale in (
            (args.reference_checkpoint, args.reference_engine_manifest, args.reference_model_config),
            (args.checkpoint, args.engine_manifest, args.model_config)):
        kwargs = dict(checkpoint=str(checkpoint), model_config=scale, device=args.device,
                      input_hw=(480, 640), input_hw_by_view={'front': (544, 960)})
        backend = (TwinLiteTensorRTBackend(**kwargs, engine_manifest=str(manifest))
                   if args.backend == 'tensorrt' else TwinLiteBackend(**kwargs, amp=False))
        engines.append(InferenceEngine(backend))
    cache = RawGpuCache() if args.backend == 'tensorrt' else None
    try:
        return run(args, engines, cache)
    finally:
        if cache:
            cache.close()


if __name__ == '__main__':
    main()
