#!/usr/bin/env python3
"""Pinned RF-DETR export in an isolated SDK environment, TRT build in ROS ABI.

Does not install packages, train, alter the reference checkout or select a model.
Every stage writes into a new directory; failures/partial outputs are preserved.
"""
import argparse
import hashlib
from importlib.metadata import version
import json
from pathlib import Path
import subprocess
import time

PIN='eca736acab9f6fe93f2cbc82a1d9f6edfb53f0ea'


def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
    return h.hexdigest()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('stage',choices=['onnx','engine'])
    p.add_argument('--source',type=Path,required=True)
    p.add_argument('--checkpoint',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--shape',type=int,nargs=2,default=[704,704])
    p.add_argument('--scale',choices=['l','m'],default='l')
    p.add_argument('--precision-policy',choices=['fp16','fp16-no-tf32','fp16-stable','fp16-decoder-fp32','fp32-diagnostic'],default='fp16-no-tf32')
    a=p.parse_args();start=time.monotonic()
    if any(v<=0 or v%32 for v in a.shape):p.error('M/L shapes must be multiples of 32')
    record=dict(stage=a.stage,source_sha=PIN,checkpoint_sha256=sha(a.checkpoint),
                shape=a.shape,scale=a.scale,started_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()))
    if a.stage=='onnx':
        head=subprocess.check_output(['git','-C',str(a.source),'rev-parse','HEAD'],text=True).strip()
        if head!=PIN:raise ValueError('RF-DETR reference source changed')
        a.output.mkdir(parents=True,exist_ok=False)
        from rfdetr import RFDETRLarge,RFDETRMedium
        import onnx
        import torch
        import rfdetr
        model=(RFDETRLarge if a.scale=='l' else RFDETRMedium)(pretrain_weights=str(a.checkpoint),device='cuda')
        exported=model.export(output_dir=str(a.output),shape=tuple(a.shape),batch_size=3,
                              dynamic_batch=True,format='onnx',fp16=False,opset_version=17,verbose=False)
        graph=onnx.load(str(exported));onnx.checker.check_model(graph)
        if len(graph.graph.input)!=1 or len(graph.graph.output)!=2:
            raise ValueError('expected detection-only RF-DETR graph')
        record.update(onnx=str(exported),onnx_sha256=sha(exported),
            input_name=graph.graph.input[0].name,output_names=[o.name for o in graph.graph.output],
            torch=torch.__version__,cuda=torch.version.cuda,rfdetr=version('rfdetr'),
            output_contract='normalized cxcywh then raw sparse COCO logits',
            precision='FP32 ONNX reference, FP16 engine build is separate')
        if record['output_names']!=['dets','labels']:
            raise ValueError('RF-DETR output order changed: '+str(record['output_names']))
        (a.output/'onnx.json').write_text(json.dumps(record,indent=2)+'\n')
    else:
        import tensorrt as trt
        old=json.loads((a.output/'onnx.json').read_text())
        if old['checkpoint_sha256']!=record['checkpoint_sha256'] or old['shape']!=a.shape:
            raise ValueError('RF-DETR export provenance mismatch')
        onnx=Path(old['onnx'])
        if sha(onnx)!=old['onnx_sha256']:raise ValueError('ONNX SHA mismatch')
        target=a.output/'model.engine'
        if target.exists():raise ValueError('preserve previous engine and use new output')
        logger=trt.Logger(trt.Logger.WARNING);builder=trt.Builder(logger)
        network=builder.create_network(1<<int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
        parser=trt.OnnxParser(network,logger)
        if not parser.parse(onnx.read_bytes()):
            raise RuntimeError('\n'.join(str(parser.get_error(i)) for i in range(parser.num_errors)))
        config=builder.create_builder_config()
        if a.precision_policy!='fp32-diagnostic':config.set_flag(trt.BuilderFlag.FP16)
        if a.precision_policy!='fp16':config.clear_flag(trt.BuilderFlag.TF32)
        protected=[]
        if a.precision_policy in ('fp16-stable','fp16-decoder-fp32'):
            config.set_flag(trt.BuilderFlag.OBEY_PRECISION_CONSTRAINTS)
            for i in range(network.num_layers):
                layer=network.get_layer(i)
                sensitive=layer.type in (trt.LayerType.NORMALIZATION,trt.LayerType.SOFTMAX,trt.LayerType.REDUCE)
                if a.precision_policy=='fp16-decoder-fp32':
                    sensitive |= any(n in layer.name for n in ('/transformer/','/bbox_embed/','/class_embed/'))
                    # bbox reparameterization (Slice/Mul/Add/Exp/Concat) lives
                    # at graph root, outside bbox_embed. Keep normalized center
                    # coordinates FP32; half rounding is material for tiny boxes.
                    sensitive |= layer.name.startswith('/') and layer.name.count('/')==1
                if (sensitive
                        and layer.num_inputs and layer.get_input(0).dtype in (trt.float16,trt.float32)
                        and any(layer.get_output(j).dtype in (trt.float16,trt.float32) for j in range(layer.num_outputs))):
                    layer.precision=trt.float32
                    for j in range(layer.num_outputs):
                        if layer.get_output(j).dtype in (trt.float16,trt.float32):layer.set_output_type(j,trt.float32)
                    protected.append(layer.name)
        config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE,1024**3)
        profile=builder.create_optimization_profile();hw=tuple(a.shape)
        profile.set_shape(old['input_name'],(1,3)+hw,(3,3)+hw,(3,3)+hw)
        config.add_optimization_profile(profile)
        plan=builder.build_serialized_network(network,config)
        if plan is None:raise RuntimeError('RF-DETR TensorRT build failed')
        target.write_bytes(bytes(plan))
        engine_started=record['started_utc']
        record.update(old,stage='engine',engine_sha256=sha(target),tensorrt=trt.__version__,
            class_ids={'1':'person','2':'bicycle','3':'car','4':'motorcycle','6':'bus','8':'truck','18':'dog'},
            batch_profile={'minimum':1,'optimum':3,'maximum':3},fp16_enabled=a.precision_policy!='fp32-diagnostic',
            started_utc=engine_started,onnx_started_utc=old['started_utc'],
            precision_policy=a.precision_policy,fp32_protected_layers=protected,
            elapsed_seconds=time.monotonic()-start)
        target.with_suffix('.metadata.json').write_text(json.dumps(record,indent=2)+'\n')
    print(json.dumps(record))


if __name__=='__main__':main()
