"""Closed-vocabulary detect/seg runtime with invertible three-view packing."""
import time
from pathlib import Path

import numpy as np

from .batching import batch_groups, pack_image, shapes_for_front
from .segmentation import SegmentationFrame, select_result
from .ultralytics_backend import cuda_preflight, file_sha256, DetectorStartupError
from .confidence import validate_confidence

COCO7 = ('person', 'dog', 'bicycle', 'car', 'motorcycle', 'bus', 'truck')


class BatchedYoloDetector:
    batched = True

    def __init__(self, weights, expected_sha256, allowed_names=COCO7,
                 conf_thres=.7, device='cuda:0', require_cuda=True,
                 runtime_backend='pytorch', task='detect', front_width=1280,
                 batch_mode='2plus1', head_profile='legacy_coco7', nms_iou=.5, inference_quantize=16,
                 direct_detect=False, **unused):
        import torch
        import ultralytics
        from ultralytics import YOLO
        self.torch, self.device = torch, device
        self.task, self.conf = task, validate_confidence(conf_thres)
        self.direct_detect = bool(direct_detect)
        if self.direct_detect and (task != 'detect' or runtime_backend != 'tensorrt'):
            raise DetectorStartupError('direct detection requires TensorRT detect')
        self.supports_gpu_input = self.direct_detect
        self.cuda_stream = torch.cuda.Stream(device=device) if self.direct_detect else None
        self.input_uploads = self.input_upload_bytes = 0
        self.mode, self.shapes = batch_mode, shapes_for_front(front_width)
        batch_groups(['front', 'left', 'right'], batch_mode)
        self.allowed = list(allowed_names)
        from .mission import CLASSES
        self.mission = head_profile == 'mission16_one2one_nms'
        if head_profile not in ('legacy_coco7', 'mission16_one2one_nms'):
            raise DetectorStartupError('unknown head profile')
        self.inference_quantize = inference_quantize
        self.nms_iou = float(nms_iou)
        if not 0 <= self.nms_iou <= 1:
            raise DetectorStartupError('invalid NMS IoU')
        expected_names = CLASSES if self.mission else COCO7
        if task not in ('detect', 'segment') or set(self.allowed) != set(expected_names) or (self.mission and task != 'detect'):
            raise DetectorStartupError('runtime vocabulary/task differs from the selected head profile')
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
        if self.direct_detect:
            from ultralytics.nn.autobackend import AutoBackend
            try:
                self.runtime = AutoBackend(str(path), device=torch.device(device), fp16=True, verbose=False)
                from asmc_camera.runtime.tensorrt import TensorRTRunner
                self.trt_runner = TensorRTRunner(self.runtime.backend.model,
                    self.runtime.backend.context, torch.device(device), self.cuda_stream)
            except Exception as exc:
                raise DetectorStartupError('TensorRT engine cannot load in this GPU/runtime environment; '
                    'use the supported asmc-ros-noetic environment or rebuild from package .pt: ' + str(exc)) from exc
            self.model = None
        else:
            self.model = YOLO(str(path), task=task)
        if runtime_backend == 'pytorch':
            # The checkpoint stores end2end=False. Fuse only after predictor
            # selects the requested head, otherwise one2one can be deleted.
            if not self.mission:
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
            if metadata.get('end2end') is not self.mission or metadata.get('task') != task:
                raise DetectorStartupError('engine task/head metadata mismatch')
            self.names = {int(k): v for k, v in metadata['names'].items()}
        else:
            self.names = self.model.names
        self.allowed_ids = [i for i, name in self.names.items() if name in self.allowed]
        if len(self.allowed_ids) != len(expected_names):
            raise DetectorStartupError('checkpoint vocabulary does not contain the selected allowed names')
        if self.mission and tuple(self.names[i] for i in sorted(self.names)) != CLASSES:
            raise DetectorStartupError('mission16 checkpoint class ID order differs')
        load_ms = (time.monotonic()-start)*1000
        warm = time.monotonic()
        for views in (['front'], ['left'], ['left', 'right'], ['front', 'left', 'right']):
            images = {v: np.zeros((720,1280,3) if v == 'front' else (480,640,3), np.uint8) for v in views}
            self.infer_images(images, views)
        runtime = self.runtime if self.direct_detect else self.model.predictor.model
        expected_format = 'engine' if runtime_backend == 'tensorrt' else 'pt'
        if runtime.device.type != 'cuda' or runtime.format != expected_format or bool(runtime.end2end) != self.mission:
            raise DetectorStartupError('runtime device/format/head differs from the selected profile')
        self.info = dict(backend=runtime_backend, model_kind='yolo', task=task,
            weights=str(path), weights_sha256=actual, allowed_names=self.allowed,
            batch_mode=batch_mode, shapes=self.shapes, device=device, model_device=device,
            ultralytics=ultralytics.__version__, torch=torch.__version__,
            conf_thres=self.conf, model_load_ms=load_ms,
            warmup_ms=(time.monotonic()-warm)*1000,
            head='one-to-one + external same-class NMS' if self.mission else 'one-to-many', nms_iou=self.nms_iou, masks_materialized=task == 'segment')
        self.info['direct_detect'] = self.direct_detect
        if runtime_backend == 'tensorrt':
            import tensorrt
            self.info.update(tensorrt=tensorrt.__version__, input_fp16=runtime.fp16,
                             export_metadata=runtime.metadata)

    def infer_images(self, images, order, gpu_frames=None, gpu_masks=None):
        if self.cuda_stream is not None:
            with self.torch.cuda.stream(self.cuda_stream):
                try:
                    return self._infer_images(images, order, gpu_frames, gpu_masks)
                finally:
                    for raw in (gpu_frames or {}).values():
                        raw.release()
        return self._infer_images(images, order, gpu_frames, gpu_masks)

    def _execute_direct(self, tensor):
        values = self.trt_runner.execute(tensor)
        outputs = [values[name] for name in self.trt_runner.output_names]
        return outputs[0] if len(outputs) == 1 else outputs

    def _infer_images(self, images, order, gpu_frames=None, gpu_masks=None):
        from ultralytics.utils.ops import scale_masks
        output = {}
        for group in batch_groups(order, self.mode):
            common = (max(self.shapes[v][0] for v in group),
                      max(self.shapes[v][1] for v in group)) if self.mode == 'batch3' else None
            tensor = None
            if gpu_frames is not None:
                if not self.direct_detect or self.mode == 'batch3':
                    raise ValueError('shared GPU input requires direct 2plus1/sequential detection')
                from asmc_camera.runtime.gpu_input import PreprocessSpec, prepare
                from .batching import ImageTransform
                values, packed = [], []
                for view in group:
                    mask = (gpu_masks or {}).get(view)
                    value, geometry, _ = prepare(gpu_frames[view], PreprocessSpec(
                        tuple(self.shapes[view]), 'linear',
                        mask.config_sha256 if mask and mask.enabled else '',
                        'float16' if self.runtime.fp16 else 'float32', normalization='divide255'), mask)
                    values.append(value)
                    left, top, _, _ = geometry.pad_ltrb
                    packed.append((None, ImageTransform(geometry.original_hw, geometry.resized_hw,
                                                       top, left, geometry.target_hw)))
                tensor = self.torch.stack(values)
                tensor_hw = tuple(tensor.shape[2:])
            else:
                packed = [pack_image(images[v], v, common, self.shapes) for v in group]
                tensor_hw = packed[0][0].shape[:2]
            if not self.direct_detect:
                self.torch.cuda.synchronize(self.device)
            start = time.monotonic()
            if self.direct_detect:
                from ultralytics.utils.nms import non_max_suppression
                if tensor is None:
                    # Same pixels and FP16 normalization as BasePredictor.preprocess.
                    bgr = np.stack([p for p, _ in packed])
                    self.input_uploads += 1
                    self.input_upload_bytes += bgr.nbytes
                    tensor = self.torch.from_numpy(bgr).to(self.device).permute(0, 3, 1, 2)
                    tensor = tensor.flip(1).contiguous()
                    tensor = (tensor.half() if self.runtime.fp16 else tensor.float()).div_(255.)
                with self.torch.inference_mode():
                    raw = self._execute_direct(tensor)
                    selected = non_max_suppression(raw, self.conf, self.nms_iou,
                        self.allowed_ids, False, max_det=300, end2end=self.mission)
                    from ultralytics.utils.ops import clip_boxes
                    for value in selected:
                        clip_boxes(value[:, :4], tensor_hw)
                    results = [value.cpu().numpy() for value in selected]
            else:
                results = list(self.model.predict([p for p, _ in packed], stream=False,
                    verbose=False, conf=self.conf, classes=self.allowed_ids,
                    iou=self.nms_iou, max_det=300, agnostic_nms=False,
                    imgsz=list(tensor_hw), rect=False, nms=not self.mission, device=self.device,
                    **({"quantize": self.inference_quantize} if self.mission else {})))
                self.torch.cuda.synchronize(self.device)
            inference_ms = (time.monotonic()-start)*1000
            if len(results) != len(group):
                raise ValueError('batch/result count mismatch')
            post = time.monotonic()
            for view, (_, transform), result in zip(group, packed, results):
                bitmap = None
                if self.task == 'segment' and getattr(result, 'masks', None) is not None:
                    bitmap = (scale_masks(result.masks.data[None], tensor_hw)[0] > .5).cpu().numpy()
                if self.direct_detect:
                    from .backend import Detection
                    if not np.isfinite(result).all():
                        raise ValueError('nonfinite direct detection output')
                    detections = [Detection(*[float(v) for v in r[:4]], self.names[int(r[5])], float(r[4]))
                                  for r in result]
                    masks = [None] * len(detections)
                else:
                    detections, masks = select_result(result, result.names, self.allowed, self.conf, bitmap)
                if self.mission and getattr(self, 'capture_pre_nms_evaluation', False):
                    if not hasattr(self, 'pre_nms_evaluation'):
                        self.pre_nms_evaluation = {}
                    self.pre_nms_evaluation[view] = []
                    for detection in detections:
                        clipped = transform.box(detection)
                        self.pre_nms_evaluation[view].append(dict(class_name=detection.source_class_name,
                            confidence=detection.confidence,
                            xyxy=[clipped.xmin,clipped.ymin,clipped.xmax,clipped.ymax],
                            nms_xyxy=[detection.xmin,detection.ymin,detection.xmax,detection.ymax]))
                if self.mission:
                    from .output_nms import suppress
                    indexed = [dict(class_name=d.source_class_name, confidence=d.confidence,
                                    xyxy=[d.xmin,d.ymin,d.xmax,d.ymax], index=i)
                               for i,d in enumerate(detections)]
                    kept, _ = suppress(indexed, self.conf, self.nms_iou)
                    detections, masks = ([detections[p['index']] for p in kept],
                                         [masks[p['index']] for p in kept])
                if self.task == 'segment' and any(m is None for m in masks):
                    raise ValueError('segmentation result missing instance mask')
                output[view] = SegmentationFrame(
                    [transform.box(d) for d in detections], [transform.mask(m) for m in masks],
                    dict(inference_ms=inference_ms, batch_size=len(group),
                         batch_tensor_pixels=tensor_hw[0]*tensor_hw[1]*len(group)))
            for view in group:
                output[view].timings['mask_transfer_ms'] = (time.monotonic()-post)*1000
        return output
