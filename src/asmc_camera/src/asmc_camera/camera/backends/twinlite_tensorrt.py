"""Pinned TwinLite MORAI deployment; explicit TensorRT, never silent fallback."""
import json
from pathlib import Path
import time

import numpy as np

from ..backend import BackendInfo, CategoricalOutput, SemanticBackend
from ...runtime.tensorrt import TensorRTRunner, sha256_file


def load_manifest(path, checkpoint, input_hw, input_hw_by_view, model_config):
    path = Path(path).resolve()
    manifest = json.loads(path.read_text())
    if manifest.get('schema') != 'asmc_twinlite_tensorrt_v1':
        raise ValueError('unsupported TwinLite engine manifest')
    if manifest.get('completed') is not True:
        raise ValueError('TwinLite engine build is incomplete')
    from . import PINNED_TWINLITE_COMMIT
    if manifest['upstream_commit'] != PINNED_TWINLITE_COMMIT:
        raise ValueError('TwinLite upstream mismatch')
    if manifest['model_config'] != model_config or manifest['checkpoint_sha256'] != sha256_file(checkpoint):
        raise ValueError('TwinLite checkpoint/config mismatch')
    expected_classes = ['background', 'white_lane', 'yellow_lane', 'stopline']
    if manifest['class_names'] != {'drivable': ['background', 'drivable'], 'road_marking': expected_classes}:
        raise ValueError('TwinLite head/class order mismatch')
    wanted = {tuple(input_hw)} | {tuple(v) for v in input_hw_by_view.values()}
    if wanted != {tuple(row['hw']) for row in manifest['engines']}:
        raise ValueError('TwinLite configured/engine input sizes differ')
    if len(wanted) != len(manifest['engines']):
        raise ValueError('duplicate TwinLite engine shape')
    for row in manifest['engines']:
        if row['output_mode'] not in ('logits', 'reduced'):
            raise ValueError('unknown TwinLite engine output mode')
        if not 1 <= row['max_batch'] <= 2:
            raise ValueError('unsupported TwinLite engine batch')
        engine_path = (path.parent / row['file']).resolve()
        if engine_path.parent != path.parent or sha256_file(engine_path) != row['sha256']:
            raise ValueError('TwinLite engine path/SHA mismatch')
    return manifest, path.parent


class TwinLiteTensorRTBackend(SemanticBackend):
    def __init__(self, checkpoint, engine_manifest, model_config='medium', device='cuda',
                 input_hw=(480, 640), input_hw_by_view=None, weights='auto', **unused):
        import torch
        import tensorrt as trt
        self._torch = torch
        self._device = torch.device(device)
        if self._device.type != 'cuda' or not torch.cuda.is_available():
            raise RuntimeError('TwinLite TensorRT requires CUDA')
        shapes = {v: tuple(s) for v, s in (input_hw_by_view or {}).items()}
        manifest, directory = load_manifest(engine_manifest, checkpoint, input_hw, shapes, model_config)
        if weights not in ('auto', manifest['weights']):
            raise ValueError('requested weights differ from exported weights')
        if manifest['tensorrt'] != trt.__version__ or manifest['gpu_name'] != torch.cuda.get_device_name(self._device):
            raise RuntimeError('TwinLite engine environment mismatch; rebuild for this GPU/runtime')
        self.cuda_stream = torch.cuda.Stream(device=self._device)
        self.runners, self.modes, self.inputs, self.host_outputs = {}, {}, {}, {}
        self.input_uploads = self.input_upload_bytes = 0
        self.timings = {}
        self._info = BackendInfo(name='twinlite_tensorrt', model_config=model_config,
            heads=('drivable', 'road_marking'), input_hw=tuple(input_hw), device=str(self._device),
            checkpoint_path=str(checkpoint), checkpoint_sha256=manifest['checkpoint_sha256'],
            checkpoint_epoch=manifest['checkpoint_epoch'], weights=manifest['weights'],
            upstream_commit=manifest['upstream_commit'],
            categorical_classes={'road_marking': tuple(manifest['class_names']['road_marking'])},
            input_hw_by_view=shapes)
        with torch.cuda.stream(self.cuda_stream):
            for row in manifest['engines']:
                hw = tuple(row['hw'])
                runner = TensorRTRunner.load(directory / row['file'], row['sha256'], self._device, self.cuda_stream)
                names = ({'drivable_logits', 'road_marking_logits'} if row['output_mode'] == 'logits'
                         else {'drivable_probability', 'road_marking_class_id', 'road_marking_confidence'})
                if set(runner.output_names) != names:
                    raise ValueError('TwinLite engine outputs differ from manifest')
                self.runners[hw], self.modes[hw] = runner, row['output_mode']
                for batch in range(1, row['max_batch'] + 1):
                    shape = (batch, 3) + hw
                    dtype = runner.dtypes[runner.input_names[0]]
                    if dtype != torch.float32:
                        raise ValueError('TwinLite normalized input binding must be float32')
                    self.inputs[shape] = torch.zeros(shape, dtype=dtype, device=self._device)
                    # Allocate device output buffers before ACTIVE.
                    outputs = runner.execute(self.inputs[shape])
                    expected = ({'drivable_logits': (batch, 2) + hw,
                                 'road_marking_logits': (batch, 4) + hw}
                                if row['output_mode'] == 'logits' else
                                {name: (batch,) + hw for name in names})
                    for name, value in outputs.items():
                        expected_dtype = torch.int32 if name == 'road_marking_class_id' else torch.float32
                        if tuple(value.shape) != expected[name] or value.dtype != expected_dtype:
                            raise ValueError('TwinLite output binding shape/dtype mismatch: ' + name)
                    self.host_outputs[shape] = {
                        'drivable': torch.empty((batch,) + hw, dtype=torch.float32, pin_memory=True),
                        'class_ids': torch.empty((batch,) + hw, dtype=torch.uint8, pin_memory=True),
                        'confidence': torch.empty((batch,) + hw, dtype=torch.float32, pin_memory=True)}
        self.cuda_stream.synchronize()
        self.events = [torch.cuda.Event(enable_timing=True) for _ in range(5)]

    @property
    def info(self):
        return self._info

    def infer(self, batch):
        torch = self._torch
        shape = tuple(batch.shape)
        if shape not in self.inputs:
            raise ValueError('TwinLite input shape not prepared: {}'.format(shape))
        started = time.perf_counter()
        tensor = self.inputs[shape]
        events = self.events
        with torch.cuda.stream(self.cuda_stream), torch.inference_mode():
            events[0].record()
            if isinstance(batch, np.ndarray):
                if batch.dtype != np.float32:
                    raise ValueError('TwinLite CPU input must be normalized float32')
                self.input_uploads += 1
                self.input_upload_bytes += batch.nbytes
                tensor.copy_(torch.from_numpy(np.ascontiguousarray(batch)), non_blocking=True)
            else:
                if batch.device != tensor.device or batch.dtype != torch.float32:
                    raise ValueError('TwinLite GPU input device/dtype mismatch')
                tensor.copy_(batch, non_blocking=True)
            events[1].record()
            runner = self.runners[shape[2:]]
            outputs = runner.execute(tensor)
            events[2].record()
            if self.modes[shape[2:]] == 'logits':
                drivable = torch.softmax(outputs['drivable_logits'].float(), dim=1)[:, 1]
                confidence, classes = torch.softmax(outputs['road_marking_logits'].float(), dim=1).max(dim=1)
            else:
                drivable = outputs['drivable_probability']
                confidence = outputs['road_marking_confidence']
                classes = outputs['road_marking_class_id']
            events[3].record()
            host = self.host_outputs[shape]
            host['drivable'].copy_(drivable, non_blocking=True)
            host['class_ids'].copy_(classes, non_blocking=True)
            host['confidence'].copy_(confidence, non_blocking=True)
            events[4].record()
        events[4].synchronize()
        self.timings = dict(input_copy_gpu_ms=events[0].elapsed_time(events[1]),
            engine_gpu_ms=events[1].elapsed_time(events[2]),
            reduction_gpu_ms=events[2].elapsed_time(events[3]),
            d2h_gpu_ms=events[3].elapsed_time(events[4]),
            backend_wall_ms=(time.perf_counter() - started) * 1000.,
            engine_includes_reduction=self.modes[shape[2:]] == 'reduced')
        # The public backend contract owns host arrays, including callers that
        # retain earlier predictions. Pinned staging buffers stay private.
        return {'drivable': host['drivable'].numpy().copy(),
                'road_marking': CategoricalOutput(host['class_ids'].numpy().copy(),
                                                  host['confidence'].numpy().copy())}
