"""Opt-in YOLOE-l batch runtime. Exported engines retain their baked names."""
import json
import time
from pathlib import Path

import numpy as np

from .batching import SHAPES, batch_groups, pack_image
from .segmentation import SegmentationFrame, load_prompt_embeddings, select_result
from .ultralytics_backend import cuda_preflight, file_sha256, DetectorStartupError


class BatchedSegmentationDetector:
    batched = True

    def __init__(self, weights, prompt_embeddings, conf_thres=.7, allowed_names=(),
                 expected_sha256='', device='cuda:0', require_cuda=True,
                 batch_mode='2plus1', runtime_backend='tensorrt',
                 source_checkpoint_sha256='',expected_prompt_sha256='', **unused):
        import torch
        import ultralytics
        from ultralytics import YOLO, YOLOE
        self.torch, self.device = torch, device
        self.mode, self.conf = batch_mode, float(conf_thres)
        batch_groups(['front', 'left', 'right'], batch_mode)
        if not require_cuda or not device.startswith('cuda'):
            raise DetectorStartupError('YOLOE-l study requires CUDA; no CPU fallback')
        ok, report = cuda_preflight(torch, device)
        if not ok:
            raise DetectorStartupError(report)
        path = Path(weights)
        actual = file_sha256(path)
        if not expected_sha256 or expected_sha256 != actual:
            raise DetectorStartupError('missing or mismatched experiment weight/engine SHA')
        started = time.monotonic()
        self.names_list = [str(n) for n in allowed_names]
        if not self.names_list or len(set(self.names_list)) != len(self.names_list):
            raise DetectorStartupError('nonempty unique prompt names required')
        prompt_sha=file_sha256(Path(prompt_embeddings))
        if expected_prompt_sha256 and expected_prompt_sha256!=prompt_sha:
            raise DetectorStartupError('prompt SHA mismatch')
        names,embeddings=load_prompt_embeddings(prompt_embeddings,
            source_checkpoint_sha256 if runtime_backend=='tensorrt' else actual)
        if names!=self.names_list:
            raise DetectorStartupError('prompt/config order mismatch')
        if runtime_backend == 'tensorrt':
            if path.suffix != '.engine':
                raise DetectorStartupError('TensorRT runtime requires engine')
            self.model = YOLO(str(path), task='segment')
            # AutoBackend initializes at the first prediction. It checks engine names below.
        elif runtime_backend == 'pytorch':
            self.model = YOLOE(str(path))
            self.model.set_classes(names, torch.as_tensor(embeddings))
            if self.model.model.model[-1].end2end:
                raise DetectorStartupError('expected official YOLOE-l one-to-many checkpoint head')
            self.model.fuse()
            self.model.to(device)
        else:
            raise DetectorStartupError('unknown experiment runtime backend')
        self.runtime_backend = runtime_backend
        load_ms = (time.monotonic()-started)*1000
        warm = time.monotonic()
        # Exercise every admissible shape/batch, including an absent side camera.
        for views in (['front'], ['left'], ['left', 'right'], ['front', 'left', 'right']):
            images = {v: np.zeros((720,1280,3) if v == 'front' else (480,640,3), np.uint8) for v in views}
            self.infer_images(images, views)
        self.info = dict(backend=runtime_backend, model_kind='yoloe', weights=str(path),
            weights_sha256=actual, prompt_embeddings_sha256=file_sha256(Path(prompt_embeddings)),
            prompt_names=self.names_list, batch_mode=batch_mode, shapes=SHAPES,
            device=device, model_device=device, ultralytics=ultralytics.__version__,
            torch=torch.__version__, conf_thres=self.conf, model_load_ms=load_ms,
            warmup_ms=(time.monotonic()-warm)*1000, masks_materialized=True)
        self.info.update(head='one-to-many', postprocessing='class-aware NMS, IoU 0.7, max_det 300')

    def infer_images(self, images, order):
        from ultralytics.utils.ops import scale_masks
        results_by_view = {}
        for group in batch_groups(order, self.mode):
            common = SHAPES['front'] if self.mode == 'batch3' else None
            prepared = [pack_image(images[v], v, common) for v in group]
            tensor_hw = prepared[0][0].shape[:2]
            if any(p.shape[:2] != tensor_hw for p,_ in prepared):
                raise ValueError('inconsistent batch shapes')
            self.torch.cuda.synchronize(self.device)
            start = time.monotonic()
            results = list(self.model.predict([p for p,_ in prepared],
                stream=False, verbose=False, conf=self.conf, iou=.7, max_det=300,
                agnostic_nms=False, imgsz=list(tensor_hw), rect=False,
                nms=True, device=self.device))
            self.torch.cuda.synchronize(self.device)
            inference_ms = (time.monotonic()-start)*1000
            if len(results) != len(group):
                raise ValueError('batch/result count mismatch')
            post_started = time.monotonic()
            for view, (_, transform), result in zip(group, prepared, results):
                names = dict(result.names)
                if [names[k] for k in sorted(names)] != self.names_list:
                    raise DetectorStartupError('exported engine prompt names/order mismatch')
                bitmap = None
                if getattr(result, 'masks', None) is not None:
                    bitmap = scale_masks(result.masks.data[None], tensor_hw)[0]
                    bitmap = (bitmap > .5).cpu().numpy()
                detections, masks = select_result(result, names, self.names_list, self.conf, bitmap)
                if any(m is None for m in masks):
                    raise ValueError('seg detection without mask')
                results_by_view[view] = SegmentationFrame(
                    [transform.box(d) for d in detections], [transform.mask(m) for m in masks],
                    dict(inference_ms=inference_ms, batch_size=len(group),
                         batch_tensor_pixels=tensor_hw[0]*tensor_hw[1]*len(group)))
            post_ms = (time.monotonic()-post_started)*1000
            for view in group:
                results_by_view[view].timings['mask_transfer_ms'] = post_ms
        return results_by_view
