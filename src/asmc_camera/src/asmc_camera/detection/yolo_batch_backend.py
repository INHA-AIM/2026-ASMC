"""Closed-vocabulary detect/seg runtime with invertible three-view packing."""
import time
from pathlib import Path

import numpy as np

from .batching import batch_groups, pack_image, shapes_for_front
from .segmentation import SegmentationFrame, select_result
from .ultralytics_backend import cuda_preflight, file_sha256, DetectorStartupError

COCO7 = ('person', 'dog', 'bicycle', 'car', 'motorcycle', 'bus', 'truck')


class BatchedYoloDetector:
    batched = True

    def __init__(self, weights, expected_sha256, allowed_names=COCO7,
                 conf_thres=.7, device='cuda:0', require_cuda=True,
                 runtime_backend='pytorch', task='detect', front_width=1280,
                 batch_mode='2plus1', **unused):
        import torch
        import ultralytics
        from ultralytics import YOLO
        self.torch, self.device = torch, device
        self.task, self.conf = task, float(conf_thres)
        self.mode, self.shapes = batch_mode, shapes_for_front(front_width)
        batch_groups(['front', 'left', 'right'], batch_mode)
        self.allowed = list(allowed_names)
        if task not in ('detect', 'segment') or set(self.allowed) != set(COCO7):
            raise DetectorStartupError('closed-set runtime requires exactly COCO7 and detect/segment')
        if not require_cuda or not device.startswith('cuda'):
            raise DetectorStartupError('three-view runtime requires CUDA; no CPU fallback')
        ok, report = cuda_preflight(torch, device)
        if not ok:
            raise DetectorStartupError(report)
        path = Path(weights)
        actual = file_sha256(path)
        if not expected_sha256 or actual != expected_sha256:
            raise DetectorStartupError('missing or mismatched model/engine SHA')
        if runtime_backend not in ('pytorch', 'tensorrt'):
            raise DetectorStartupError('unknown runtime backend')
        if (path.suffix == '.engine') != (runtime_backend == 'tensorrt'):
            raise DetectorStartupError('checkpoint/engine backend mismatch')
        start = time.monotonic()
        self.model = YOLO(str(path), task=task)
        if runtime_backend == 'pytorch':
            self.model.fuse()
            self.model.to(device)
        if runtime_backend == 'tensorrt':
            # Reading names through YOLO.names creates a predictor with default
            # head options. Read the export header before constructing the one
            # predictor that actually runs this fixed profile.
            import json
            import struct
            with path.open('rb') as handle:
                metadata_size = struct.unpack('<i', handle.read(4))[0]
                if not 0 < metadata_size < 1024*1024:
                    raise DetectorStartupError('invalid engine metadata size')
                metadata = json.loads(handle.read(metadata_size))
            if metadata.get('end2end') is not False or metadata.get('task') != task:
                raise DetectorStartupError('engine task/head metadata mismatch')
            self.names = {int(k): v for k, v in metadata['names'].items()}
        else:
            self.names = self.model.names
        self.allowed_ids = [i for i, name in self.names.items() if name in self.allowed]
        if len(self.allowed_ids) != len(COCO7):
            raise DetectorStartupError('checkpoint vocabulary does not contain COCO7')
        load_ms = (time.monotonic()-start)*1000
        warm = time.monotonic()
        for views in (['front'], ['left'], ['left', 'right'], ['front', 'left', 'right']):
            images = {v: np.zeros((720,1280,3) if v == 'front' else (480,640,3), np.uint8) for v in views}
            self.infer_images(images, views)
        runtime = self.model.predictor.model
        expected_format = 'engine' if runtime_backend == 'tensorrt' else 'pt'
        if runtime.device.type != 'cuda' or runtime.format != expected_format or runtime.end2end:
            raise DetectorStartupError('runtime device/format/head differs from one-to-many profile')
        self.info = dict(backend=runtime_backend, model_kind='yolo', task=task,
            weights=str(path), weights_sha256=actual, allowed_names=self.allowed,
            batch_mode=batch_mode, shapes=self.shapes, device=device, model_device=device,
            ultralytics=ultralytics.__version__, torch=torch.__version__,
            conf_thres=self.conf, model_load_ms=load_ms,
            warmup_ms=(time.monotonic()-warm)*1000,
            head='one-to-many', masks_materialized=task == 'segment')
        if runtime_backend == 'tensorrt':
            import tensorrt
            self.info.update(tensorrt=tensorrt.__version__, input_fp16=runtime.fp16,
                             export_metadata=runtime.metadata)

    def infer_images(self, images, order):
        from ultralytics.utils.ops import scale_masks
        output = {}
        for group in batch_groups(order, self.mode):
            common = (max(self.shapes[v][0] for v in group),
                      max(self.shapes[v][1] for v in group)) if self.mode == 'batch3' else None
            packed = [pack_image(images[v], v, common, self.shapes) for v in group]
            tensor_hw = packed[0][0].shape[:2]
            self.torch.cuda.synchronize(self.device)
            start = time.monotonic()
            results = list(self.model.predict([p for p, _ in packed], stream=False,
                verbose=False, conf=self.conf, classes=self.allowed_ids,
                iou=.7, max_det=300, agnostic_nms=False,
                imgsz=list(tensor_hw), rect=False, nms=True, device=self.device))
            self.torch.cuda.synchronize(self.device)
            inference_ms = (time.monotonic()-start)*1000
            if len(results) != len(group):
                raise ValueError('batch/result count mismatch')
            post = time.monotonic()
            for view, (_, transform), result in zip(group, packed, results):
                bitmap = None
                if self.task == 'segment' and getattr(result, 'masks', None) is not None:
                    bitmap = (scale_masks(result.masks.data[None], tensor_hw)[0] > .5).cpu().numpy()
                detections, masks = select_result(result, result.names, self.allowed, self.conf, bitmap)
                if self.task == 'segment' and any(m is None for m in masks):
                    raise ValueError('segmentation result missing instance mask')
                output[view] = SegmentationFrame(
                    [transform.box(d) for d in detections], [transform.mask(m) for m in masks],
                    dict(inference_ms=inference_ms, batch_size=len(group),
                         batch_tensor_pixels=tensor_hw[0]*tensor_hw[1]*len(group)))
            for view in group:
                output[view].timings['mask_transfer_ms'] = (time.monotonic()-post)*1000
        return output
