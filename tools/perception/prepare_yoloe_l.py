#!/usr/bin/env python3
"""Prepare checkpoint-bound prompts and bounded FP16 engines, outside live ROS."""
import argparse
import gc
import hashlib
import json
import os
from pathlib import Path
import re
import struct
import time
import numpy as np
import yaml


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def profile_names(policy, profile):
    """Validate a vocabulary before using its identifier in output paths."""
    if not re.fullmatch(r'[a-z][a-z0-9_]*', profile):
        raise ValueError('unsafe prompt profile identifier')
    profiles = policy['prompt_profiles']
    if profile not in profiles:
        raise ValueError('unknown prompt profile: ' + profile)
    names = profiles[profile]
    if not isinstance(names, list) or not names:
        raise ValueError('prompt profile must contain names')
    if any(not isinstance(name, str) or not name.strip() or name != name.strip()
           for name in names):
        raise ValueError('prompt names must be nonempty, trimmed strings')
    if len(set(names)) != len(names):
        raise ValueError('duplicate prompt names')
    return names


def export_profile(width):
    if width not in (640,960,1280):
        raise ValueError('unsupported front width')
    front_hw=(((width*720//1280+31)//32)*32,width)
    return front_hw,(1,3,384,640),(2,3,480,640),(3,3,max(480,front_hw[0]),width)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--directory', type=Path, required=True)
    p.add_argument('--config', type=Path, required=True)
    p.add_argument('--profile', required=True, help='profile in the supplied frozen config')
    p.add_argument('--stage', choices=['prompts','onnx','engine'], required=True)
    p.add_argument('--scale', choices=['m','l','x'], default='l')
    p.add_argument('--front-width', type=int, choices=[640,960,1280], default=1280)
    p.add_argument('--variant', action='store_true', help='Keep width-specific exports separate from legacy files')
    a = p.parse_args()
    if a.scale == 'x' and a.stage != 'prompts':
        p.error('X teacher is PyTorch only; TensorRT export is outside this experiment')
    policy = yaml.safe_load(a.config.read_text())
    names = profile_names(policy, a.profile)
    a.directory.mkdir(parents=True, exist_ok=True)
    os.chdir(a.directory)
    checkpoint = a.directory/('yoloe-26'+a.scale+'-seg.pt')
    stem = a.profile+('-'+str(a.front_width) if a.variant else '')
    front_hw,minimum,optimum,maximum = export_profile(a.front_width)
    cache = a.directory/(a.profile+'.npz')
    onnx_path = a.directory/(stem+'.onnx')
    meta_path = a.directory/(stem+'.metadata.json')
    started = time.monotonic()
    if a.stage in ('prompts','onnx'):
        import torch
        import ultralytics
        from ultralytics import YOLOE
        model = YOLOE(str(checkpoint))
        if a.stage == 'prompts':
            embeddings = model.get_text_pe(names)
            np.savez_compressed(cache, names=np.array(names), embeddings=embeddings.cpu().numpy(),
                                weights_sha256=np.array(sha(checkpoint)))
            record=dict(checkpoint_sha256=sha(checkpoint),prompt_sha256=sha(cache),
                        text_encoder_sha256=sha(a.directory/'mobileclip2_b.ts'),names=names,
                        profile=a.profile,stage=a.stage,ultralytics=ultralytics.__version__,torch=torch.__version__)
            (a.directory/(a.profile+'.prompts.provenance.json')).write_text(json.dumps(record,indent=2)+'\n')
            print(json.dumps(record),flush=True)
            return
        from asmc_camera.detection.segmentation import load_prompt_embeddings
        cached_names,values=load_prompt_embeddings(cache,sha(checkpoint))
        if cached_names!=names: raise ValueError('cached prompt/order mismatch')
        embeddings=torch.as_tensor(values)
        model.set_classes(names, embeddings)
        exported = Path(model.export(format='onnx', dynamic=True, batch=1, imgsz=list(front_hw),
                                     device=0, nms=None, opset=17, simplify=True))
        if exported != onnx_path:
            exported.replace(onnx_path)
        import onnx
        graph = onnx.load(str(onnx_path))
        metadata = {item.key: item.value for item in graph.metadata_props}
        # ONNX stores Python repr values; engine headers require JSON values.
        import ast
        for key in ('names','imgsz','args','kpt_shape'):
            if key in metadata: metadata[key] = ast.literal_eval(metadata[key])
        for key in ('stride','batch','channels'):
            if key in metadata: metadata[key] = int(metadata[key])
        metadata.update(names=dict(enumerate(names)), batch=3, task='segment', imgsz=list(front_hw))
        metadata['end2end']=ast.literal_eval(str(metadata['end2end']))
        if metadata['end2end']: raise RuntimeError('export silently changed the checkpoint head')
        meta_path.write_text(json.dumps(metadata,indent=2)+'\n')
        del model, embeddings; gc.collect(); torch.cuda.empty_cache()
        record = dict(checkpoint_sha256=sha(checkpoint), prompt_sha256=sha(cache),
                      onnx_sha256=sha(onnx_path), text_encoder_sha256=sha(a.directory/'mobileclip2_b.ts'),
                      ultralytics=ultralytics.__version__, torch=torch.__version__)
    else:
        import tensorrt as trt
        logger = trt.Logger(trt.Logger.WARNING)
        builder = trt.Builder(logger)
        network = builder.create_network(1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
        parser = trt.OnnxParser(network, logger)
        if not parser.parse(onnx_path.read_bytes()):
            raise RuntimeError('\n'.join(str(parser.get_error(i)) for i in range(parser.num_errors)))
        if not builder.platform_has_fast_fp16: raise RuntimeError('FP16 unsupported; refusing fallback')
        config = builder.create_builder_config()
        config.set_flag(trt.BuilderFlag.FP16)
        config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, 1024**3)
        timing_path=a.directory/'tensorrt.timing-cache'
        timing_cache=config.create_timing_cache(timing_path.read_bytes() if timing_path.exists() else b'')
        config.set_timing_cache(timing_cache,False)
        profile = builder.create_optimization_profile()
        profile.set_shape(network.get_input(0).name, minimum, optimum, maximum)
        config.add_optimization_profile(profile)
        plan = builder.build_serialized_network(network, config)
        if plan is None: raise RuntimeError('TensorRT build failed')
        timing_path.write_bytes(config.get_timing_cache().serialize())
        metadata = json.loads(meta_path.read_text())
        if metadata['end2end']: raise RuntimeError('engine input must preserve one-to-many head')
        metadata['args'].update(quantize=16, dynamic=True, batch=3)
        encoded = json.dumps(metadata).encode()
        engine_path = a.directory/(stem+'.engine')
        engine_path.write_bytes(struct.pack('<i',len(encoded))+encoded+bytes(plan))
        with trt.Runtime(logger) as runtime:
            engine = runtime.deserialize_cuda_engine(plan)
            io = [dict(name=engine.get_tensor_name(i),dtype=str(engine.get_tensor_dtype(engine.get_tensor_name(i))),
                       shape=list(engine.get_tensor_shape(engine.get_tensor_name(i)))) for i in range(engine.num_io_tensors)]
        record = dict(engine_sha256=sha(engine_path), tensorrt=trt.__version__, io=io,
                      minimum=list(minimum), optimum=list(optimum), maximum=list(maximum),
                      internal_precision='FP16 enabled; I/O dtype recorded separately',workspace_bytes=1024**3)
    record.update(profile=a.profile,names=names,stage=a.stage,scale=a.scale,front_width=a.front_width,elapsed_sec=time.monotonic()-started)
    (a.directory/(stem+'.'+a.stage+'.provenance.json')).write_text(json.dumps(record,indent=2)+'\n')
    print(json.dumps(record),flush=True)


if __name__ == '__main__': main()
