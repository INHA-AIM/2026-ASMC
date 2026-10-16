"""TensorRT 10 execution shared by detector and semantic adapters.

One instance belongs to one worker. Outputs are borrowed device buffers until
the next execution of the same shape; adapters must finish consuming them first.
"""
from pathlib import Path
import hashlib

_LOGGER = None


def sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


class TensorRTRunner:
    def __init__(self, engine, context, device, stream, max_shapes=8):
        import torch
        import tensorrt as trt
        self.torch, self.trt = torch, trt
        self.engine, self.context = engine, context
        self.device, self.stream = device, stream
        self.max_shapes = max_shapes
        self.buffers = {}
        self.input_names = [engine.get_tensor_name(i) for i in range(engine.num_io_tensors)
                            if engine.get_tensor_mode(engine.get_tensor_name(i)) == trt.TensorIOMode.INPUT]
        self.output_names = sorted(engine.get_tensor_name(i) for i in range(engine.num_io_tensors)
                                   if engine.get_tensor_mode(engine.get_tensor_name(i)) == trt.TensorIOMode.OUTPUT)
        if len(self.input_names) != 1:
            raise ValueError('camera engine must have exactly one input')
        dtypes = {trt.float32: torch.float32, trt.float16: torch.float16,
                  trt.int32: torch.int32, trt.int64: torch.int64, trt.uint8: torch.uint8}
        self.dtypes = {name: dtypes[engine.get_tensor_dtype(name)]
                       for name in self.input_names + self.output_names}

    @classmethod
    def load(cls, path, expected_sha256, device, stream):
        import tensorrt as trt
        global _LOGGER
        if not expected_sha256 or sha256_file(path) != expected_sha256:
            raise ValueError('TensorRT engine SHA mismatch')
        if _LOGGER is None:
            _LOGGER = trt.Logger(trt.Logger.WARNING)
        logger = _LOGGER
        runtime = trt.Runtime(logger)
        engine = runtime.deserialize_cuda_engine(Path(path).read_bytes())
        if engine is None:
            raise RuntimeError('engine incompatible with this GPU/TensorRT; rebuild explicitly')
        context = engine.create_execution_context()
        if context is None:
            raise RuntimeError('cannot create TensorRT execution context')
        result = cls(engine, context, device, stream)
        result.runtime, result.logger = runtime, logger
        return result

    def execute(self, tensor):
        name = self.input_names[0]
        if not tensor.is_cuda or not tensor.is_contiguous() or tensor.dtype != self.dtypes[name]:
            raise ValueError('engine input device/layout/dtype mismatch')
        if tensor.device != self.torch.device(self.device):
            # torch.device('cuda') and tensor.device('cuda:0') are equivalent here.
            if str(self.device) != 'cuda' or tensor.device.index != self.torch.cuda.current_device():
                raise ValueError('engine input is on another CUDA device')
        shape = tuple(tensor.shape)
        low, _, high = self.engine.get_tensor_profile_shape(name, 0)
        if len(shape) != len(low) or any(not a <= v <= b for v, a, b in zip(shape, low, high)):
            raise ValueError('input outside TensorRT profile: {}'.format(shape))
        if not self.context.set_input_shape(name, shape):
            raise ValueError('TensorRT rejected input shape')
        if shape not in self.buffers:
            if len(self.buffers) >= self.max_shapes:
                raise RuntimeError('TensorRT shape cache limit exceeded')
            outputs = {}
            for output in self.output_names:
                out_shape = tuple(self.context.get_tensor_shape(output))
                if any(v <= 0 for v in out_shape):
                    raise ValueError('unresolved TensorRT output shape')
                outputs[output] = self.torch.empty(out_shape, device=self.device, dtype=self.dtypes[output])
            self.buffers[shape] = outputs
        outputs = self.buffers[shape]
        self.context.set_tensor_address(name, tensor.data_ptr())
        for output, value in outputs.items():
            self.context.set_tensor_address(output, value.data_ptr())
        if not self.context.execute_async_v3(self.stream.cuda_stream):
            raise RuntimeError('TensorRT execution failed')
        return outputs
