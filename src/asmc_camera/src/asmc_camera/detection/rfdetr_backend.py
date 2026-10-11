"""Opt-in TensorRT RF-DETR detection backend; never imports the RF Python SDK."""
import json
import time
from pathlib import Path

import numpy as np

from .rfdetr_core import COCO7, decode, groups, preprocess, validate_metadata
from .segmentation import SegmentationFrame
from .ultralytics_backend import DetectorStartupError, cuda_preflight, file_sha256


class RFDetrBackend:
    batched = True

    def __init__(self, weights, expected_sha256, allowed_names=COCO7, conf_thres=.4,
                 device='cuda:0', require_cuda=True, runtime_backend='tensorrt',
                 task='detect', batch_mode='batch3', rfdetr_input_shapes=None, **unused):
        import torch
        import tensorrt as trt
        self.torch, self.trt, self.device = torch, trt, device
        if runtime_backend != 'tensorrt' or task != 'detect' or not require_cuda or device != 'cuda:0':
            raise DetectorStartupError('RF-DETR experiment requires cuda:0 TensorRT detection')
        good, report = cuda_preflight(torch, device)
        if not good:
            raise DetectorStartupError(report)
        p = Path(weights)
        if not expected_sha256 or file_sha256(p) != expected_sha256:
            raise DetectorStartupError('RF-DETR engine SHA mismatch')
        self.metadata = json.loads(p.with_suffix('.metadata.json').read_text())
        self.allowed = list(allowed_names)
        if set(self.allowed) != set(COCO7) or len(self.allowed) != 7:
            raise DetectorStartupError('RF-DETR experiment requires exactly COCO7')
        self.conf, self.mode = float(conf_thres), batch_mode
        self.shapes = rfdetr_input_shapes or {v:[704,704] for v in ('front','left','right')}
        try:
            validate_metadata(self.metadata,expected_sha256,trt.__version__,self.shapes,
                              require_fp16=not unused.get('allow_fp32_diagnostic',False))
        except ValueError as exc:
            raise DetectorStartupError(str(exc)) from exc
        groups(['front','left','right'], self.mode, self.shapes)
        start = time.monotonic()
        self.logger = trt.Logger(trt.Logger.WARNING)
        self.runtime = trt.Runtime(self.logger)
        self.engine = self.runtime.deserialize_cuda_engine(p.read_bytes())
        if self.engine is None:
            raise DetectorStartupError('RF-DETR TensorRT deserialization failed')
        for i in range(self.engine.num_io_tensors):
            if self.engine.get_tensor_dtype(self.engine.get_tensor_name(i)) not in (trt.float16,trt.float32):
                raise DetectorStartupError('RF-DETR requires floating point I/O')
        self.context = self.engine.create_execution_context()
        self.input_name = self.metadata['input_name']
        self.output_names = self.metadata['output_names']
        self.stream = torch.cuda.Stream(device=device)
        self.buffers = {}
        load_ms = (time.monotonic()-start)*1000
        start = time.monotonic()
        for order in (['front'],['left','right'],['front','left','right']):
            self.infer_images({v:np.zeros((720,1280,3) if v=='front' else (480,640,3),np.uint8) for v in order}, order)
        self.info = dict(backend='tensorrt', model_kind='rfdetr', task='detect', weights=str(p),
            weights_sha256=expected_sha256, device=device, model_device=device,
            tensorrt=trt.__version__, torch=torch.__version__, batch_mode=batch_mode,
            fp16_enabled=self.metadata.get('fp16_enabled'),precision_policy=self.metadata.get('precision_policy','fp16'),
            fp32_protected_layer_count=len(self.metadata.get('fp32_protected_layers',[])),
            source_checkpoint_sha256=self.metadata.get('checkpoint_sha256'),
            shapes=self.shapes, conf_thres=self.conf, model_load_ms=load_ms,
            warmup_ms=(time.monotonic()-start)*1000, masks_materialized=False,
            preprocessing='RGB float resize; ImageNet mean/std; complete FOV, square stretch',
            head='official sigmoid/global top300; sparse COCO slots; no added NMS')

    def raw(self, batch):
        torch, trt = self.torch, self.trt
        shape = tuple(batch.shape)
        with torch.cuda.stream(self.stream):
            dtype = torch.float16 if self.engine.get_tensor_dtype(self.input_name)==trt.float16 else torch.float32
            tensor = torch.as_tensor(batch, device=self.device, dtype=dtype).contiguous()
            if not self.context.set_input_shape(self.input_name, shape):
                raise ValueError('RF-DETR input outside engine profile')
            self.context.set_tensor_address(self.input_name, tensor.data_ptr())
            outputs = {}
            for name in self.output_names:
                output_shape = tuple(self.context.get_tensor_shape(name))
                if any(v <= 0 for v in output_shape):
                    raise ValueError('RF-DETR unresolved output shape')
                dtype = torch.float16 if self.engine.get_tensor_dtype(name)==trt.float16 else torch.float32
                key=(name,output_shape,dtype)
                if key not in self.buffers:
                    self.buffers[key]=torch.empty(output_shape, device=self.device, dtype=dtype)
                outputs[name]=self.buffers[key]
                self.context.set_tensor_address(name,outputs[name].data_ptr())
            if not self.context.execute_async_v3(self.stream.cuda_stream):
                raise RuntimeError('RF-DETR TensorRT execution failed')
        self.stream.synchronize()
        return [outputs[name].float().cpu().numpy() for name in self.output_names]

    def infer_images(self, images, order):
        output={}
        for group in groups(order, self.mode, self.shapes):
            start=time.monotonic()
            batch=np.stack([preprocess(images[v], self.shapes[v]) for v in group])
            prepared=time.monotonic()
            boxes,logits=self.raw(batch)
            executed=time.monotonic()
            detections=decode(boxes,logits,[images[v].shape[:2] for v in group],self.conf,self.allowed)
            ended=time.monotonic()
            for view,ds in zip(group,detections):
                output[view]=SegmentationFrame(ds,[None]*len(ds),dict(
                    backend_preprocess_ms=(prepared-start)*1000,inference_ms=(executed-prepared)*1000,
                    backend_decode_ms=(ended-executed)*1000,batch_size=len(group),
                    batch_tensor_pixels=int(np.prod(batch.shape[:1]+batch.shape[2:]))))
        return output
