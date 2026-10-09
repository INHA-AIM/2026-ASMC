#!/usr/bin/env python3
"""Prepare SHA-bound closed-set checkpoints and one-to-many FP16 engines."""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import struct
import time

MODELS = {'yolo11m': 'detect', 'yolo26m': 'detect', 'yolo26l': 'detect',
          'yolo26m-seg': 'segment', 'yolo26l-seg': 'segment'}


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda: f.read(1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--directory', type=Path, required=True)
    p.add_argument('--model', choices=MODELS, required=True)
    p.add_argument('--stage', choices=['onnx', 'engine'], required=True)
    args = p.parse_args()
    directory = args.directory.resolve()
    directory.mkdir(parents=True, exist_ok=True)
    checkpoint = directory/(args.model+'.pt')
    started = time.monotonic()
    if args.stage == 'onnx':
        import torch
        import ultralytics
        from ultralytics import YOLO
        model = YOLO(str(checkpoint), task=MODELS[args.model])
        # nms=None exports raw one-to-many outputs; predictor performs class-aware NMS.
        exported = Path(model.export(format='onnx', dynamic=True, batch=1,
            imgsz=[736,1280], device=0, nms=None, opset=17, simplify=True))
        import onnx
        graph = onnx.load(str(exported))
        metadata = {item.key: item.value for item in graph.metadata_props}
        for key in ('names', 'imgsz', 'args', 'kpt_shape', 'end2end'):
            if key in metadata:
                metadata[key] = ast.literal_eval(metadata[key])
        for key in ('stride', 'batch', 'channels'):
            if key in metadata:
                metadata[key] = int(metadata[key])
        if metadata.get('end2end'):
            raise RuntimeError('export changed one-to-many head')
        (directory/(args.model+'.metadata.json')).write_text(json.dumps(metadata, indent=2)+'\n')
        record = dict(checkpoint_sha256=sha(checkpoint), onnx_sha256=sha(exported),
            ultralytics=ultralytics.__version__, torch=torch.__version__, metadata=metadata)
    else:
        import tensorrt as trt
        logger = trt.Logger(trt.Logger.WARNING)
        builder = trt.Builder(logger)
        network = builder.create_network(1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
        parser = trt.OnnxParser(network, logger)
        if not parser.parse((directory/(args.model+'.onnx')).read_bytes()):
            raise RuntimeError('\n'.join(str(parser.get_error(i)) for i in range(parser.num_errors)))
        if not builder.platform_has_fast_fp16:
            raise RuntimeError('FP16 unsupported')
        config = builder.create_builder_config()
        config.set_flag(trt.BuilderFlag.FP16)
        config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, 1024**3)
        timing_path = directory/'timing-cache.bin'
        cache = config.create_timing_cache(timing_path.read_bytes() if timing_path.exists() else b'')
        config.set_timing_cache(cache, False)
        profile = builder.create_optimization_profile()
        # All selected front widths, side batch 1/2, and padding-only batch 3.
        profile.set_shape(network.get_input(0).name, (1,3,384,640), (2,3,480,640), (3,3,736,1280))
        config.add_optimization_profile(profile)
        plan = builder.build_serialized_network(network, config)
        if plan is None:
            raise RuntimeError('TensorRT build failed')
        timing_path.write_bytes(config.get_timing_cache().serialize())
        metadata = json.loads((directory/(args.model+'.metadata.json')).read_text())
        metadata['args'].update(quantize=16, dynamic=True, batch=3)
        encoded = json.dumps(metadata).encode()
        engine = directory/(args.model+'.engine')
        engine.write_bytes(struct.pack('<i', len(encoded))+encoded+bytes(plan))
        record = dict(checkpoint_sha256=sha(checkpoint),
            onnx_sha256=sha(directory/(args.model+'.onnx')),
            engine_sha256=sha(engine), tensorrt=trt.__version__,
            minimum=[1,3,384,640], optimum=[2,3,480,640], maximum=[3,3,736,1280],
            fp16_enabled=True, workspace_bytes=1024**3, metadata=metadata)
    record.update(model=args.model, task=MODELS[args.model], stage=args.stage,
                  elapsed_sec=time.monotonic()-started)
    (directory/(args.model+'.'+args.stage+'.provenance.json')).write_text(json.dumps(record, indent=2)+'\n')
    print(json.dumps(record), flush=True)


if __name__ == '__main__':
    main()
