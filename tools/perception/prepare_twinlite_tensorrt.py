#!/usr/bin/env python3
"""Export pinned MORAI TwinLite weights; build engines only on an idle GPU."""
import argparse
import json
from pathlib import Path
import platform
import re
import time

import numpy as np
import torch

from asmc_camera.camera.backends.twinlite import TwinLiteBackend
from asmc_camera.runtime.tensorrt import sha256_file


class ExportHeads(torch.nn.Module):
    def __init__(self, model, reduced):
        super().__init__()
        self.model, self.reduced = model, reduced

    def forward(self, images):
        da, marking = self.model(images)
        if not self.reduced:
            return da, marking
        probability = torch.softmax(da.float(), 1)[:, 1]
        confidence, classes = torch.softmax(marking.float(), 1).max(1)
        return probability, classes.to(torch.int32), confidence


def build_engine(onnx_path, engine_path, shape, max_batch, precision, fp32_sensitive):
    import tensorrt as trt
    logger = trt.Logger(trt.Logger.WARNING)
    builder = trt.Builder(logger)
    network = builder.create_network(1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
    parser = trt.OnnxParser(network, logger)
    if not parser.parse(onnx_path.read_bytes()):
        raise RuntimeError('\n'.join(str(parser.get_error(i)) for i in range(parser.num_errors)))
    config = builder.create_builder_config()
    config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, 1024 * 1024 * 1024)
    # FP32 control must not silently use TF32; FP16 keeps numerically sensitive
    # reductions/attention in FP32 only when explicitly selected and recorded.
    config.clear_flag(trt.BuilderFlag.TF32)
    if precision == 'fp16':
        config.set_flag(trt.BuilderFlag.FP16)
    constrained = []
    if fp32_sensitive:
        config.set_flag(trt.BuilderFlag.OBEY_PRECISION_CONSTRAINTS)
        for i in range(network.num_layers):
            layer = network.get_layer(i)
            if layer.type in (trt.LayerType.SOFTMAX, trt.LayerType.MATRIX_MULTIPLY):
                layer.precision = trt.float32
                constrained.append(layer.name)
    profile = builder.create_optimization_profile()
    profile.set_shape('images', (1, 3) + shape, (max_batch, 3) + shape, (max_batch, 3) + shape)
    config.add_optimization_profile(profile)
    started = time.monotonic()
    serialized = builder.build_serialized_network(network, config)
    if serialized is None:
        raise RuntimeError('TensorRT build failed')
    engine_path.write_bytes(bytes(serialized))
    return dict(build_seconds=time.monotonic()-started, fp32_layers=constrained)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--checkpoint', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--model-config', choices=['nano', 'small', 'medium', 'large'], default='medium')
    p.add_argument('--model-prefix', help='Artifact basename; defaults to twinlite_<scale>')
    p.add_argument('--output-mode', choices=['logits', 'reduced'], default='logits')
    p.add_argument('--precision', choices=['fp32', 'fp16'], default='fp16')
    p.add_argument('--fp32-sensitive', action='store_true')
    p.add_argument('--export-only', action='store_true')
    a = p.parse_args()
    prefix = a.model_prefix or 'twinlite_' + a.model_config
    if not re.fullmatch(r'[a-zA-Z0-9_-]+', prefix):
        p.error('--model-prefix must be a filename basename')
    a.output.mkdir(parents=True, exist_ok=False)
    torch.set_num_threads(2)
    torch.manual_seed(20261015)
    backend = TwinLiteBackend(str(a.checkpoint), model_config=a.model_config, device='cpu', amp=False,
                             input_hw=(480,640), input_hw_by_view={'front': (544,960)})
    if backend.info.heads != ('drivable', 'road_marking'):
        raise ValueError('deployment requires the MORAI road-marking checkpoint')
    model = ExportHeads(backend._model, a.output_mode == 'reduced').eval()
    names = (['drivable_logits', 'road_marking_logits'] if a.output_mode == 'logits'
             else ['drivable_probability', 'road_marking_class_id', 'road_marking_confidence'])
    result = dict(schema='asmc_twinlite_tensorrt_v1', checkpoint=str(a.checkpoint),
        checkpoint_sha256=backend.info.checkpoint_sha256, checkpoint_epoch=backend.info.checkpoint_epoch,
        weights=backend.info.weights, upstream_commit=backend.info.upstream_commit,
        model_config=backend.info.model_config, torch=torch.__version__, opset=17,
        class_names={'drivable':['background','drivable'],
                     'road_marking': list(backend.info.categorical_classes['road_marking'])},
        platform=platform.platform(), engines=[], completed=False)
    try:
        import onnx
        result['onnx'] = onnx.__version__
        for name, hw, batch in [('front', (544,960), 1), ('sides', (480,640), 2)]:
            dummy = torch.rand((batch,3)+hw)
            onnx_path = a.output / (prefix + '_' + name + '.onnx')
            with torch.inference_mode():
                torch.onnx.export(model, dummy, str(onnx_path), opset_version=17,
                    input_names=['images'], output_names=names,
                    dynamic_axes={key:{0:'batch'} for key in ['images']+names},
                    do_constant_folding=True)
            onnx.checker.check_model(onnx.load(str(onnx_path)))
            row = dict(hw=list(hw), max_batch=batch, onnx=onnx_path.name,
                       onnx_sha256=sha256_file(onnx_path), output_mode=a.output_mode,
                       precision=a.precision, fp32_sensitive=a.fp32_sensitive)
            # Independent ONNX CPU execution checks the spatially fixed graph,
            # including dynamic side B1/B2. No TensorRT build before it passes.
            import onnxruntime as ort
            options = ort.SessionOptions(); options.intra_op_num_threads = 2
            session = ort.InferenceSession(str(onnx_path), sess_options=options, providers=['CPUExecutionProvider'])
            row['onnx_parity'] = []
            for n in range(1, batch+1):
                x = torch.rand((n,3)+hw)
                with torch.inference_mode(): expected = model(x)
                actual = session.run(names, {'images':x.numpy()})
                errors = []
                for left,right in zip(expected,actual):
                    error = float(np.max(np.abs(left.numpy().astype(np.float64)-right)))
                    errors.append(error)
                    if not np.all(np.isfinite(right)) or not np.allclose(left.numpy(),right,atol=1e-4,rtol=1e-4):
                        # Reduced IDs can legitimately differ at near-equal
                        # logits. Use logits export first to diagnose this.
                        raise ValueError('ONNX output differs: {} batch{} {}'.format(name,n,errors))
                row['onnx_parity'].append(dict(batch=n,max_abs=errors))
            if not a.export_only:
                import tensorrt as trt
                result.update(tensorrt=trt.__version__, gpu_name=torch.cuda.get_device_name(0),
                              compute_capability=list(torch.cuda.get_device_capability(0)))
                engine_path = onnx_path.with_suffix('.engine')
                row.update(build_engine(onnx_path,engine_path,hw,batch,a.precision,a.fp32_sensitive))
                row.update(file=engine_path.name,sha256=sha256_file(engine_path),bytes=engine_path.stat().st_size)
            result['engines'].append(row)
            (a.output/'manifest.json').write_text(json.dumps(result,indent=2)+'\n')
        result['completed'] = True
    except Exception as exc:
        result['error'] = '{}: {}'.format(type(exc).__name__,exc)
        raise
    finally:
        (a.output/'manifest.json').write_text(json.dumps(result,indent=2)+'\n')


if __name__ == '__main__':
    main()
