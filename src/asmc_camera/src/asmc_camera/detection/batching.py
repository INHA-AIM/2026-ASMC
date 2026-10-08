"""Explicit, invertible experiment-only packing of differently sized views."""
from dataclasses import dataclass
from typing import Sequence

import cv2
import numpy as np

from .backend import Detection


SHAPES = {'front': (736, 1280), 'left': (480, 640), 'right': (480, 640)}


@dataclass(frozen=True)
class ImageTransform:
    original_hw: tuple
    resized_hw: tuple
    top: int
    left: int
    tensor_hw: tuple

    def box(self, detection):
        oh, ow = self.original_hw
        rh, rw = self.resized_hw
        def x(value): return float(np.clip((value-self.left)*ow/rw, 0, ow))
        def y(value): return float(np.clip((value-self.top)*oh/rh, 0, oh))
        return Detection(x(detection.xmin), y(detection.ymin),
                         x(detection.xmax), y(detection.ymax),
                         detection.source_class_name, detection.confidence)

    def mask(self, bitmap):
        if bitmap is None:
            return None
        if tuple(bitmap.shape) != self.tensor_hw:
            raise ValueError('mask must be restored to packed tensor pixels first')
        rh, rw = self.resized_hw
        crop = bitmap[self.top:self.top+rh, self.left:self.left+rw]
        oh, ow = self.original_hw
        return cv2.resize(crop.astype(np.uint8), (ow, oh),
                          interpolation=cv2.INTER_NEAREST).astype(bool)


def pack_image(image, view, common_hw=None):
    if view not in SHAPES or image.ndim != 3 or image.shape[2] != 3:
        raise ValueError('invalid view/image')
    oh, ow = image.shape[:2]
    if oh <= 0 or ow <= 0:
        raise ValueError('empty image')
    target_h, target_w = SHAPES[view]
    scale = min(target_h/oh, target_w/ow)
    rh, rw = int(round(oh*scale)), int(round(ow*scale))
    tensor_h, tensor_w = common_hw or (target_h, target_w)
    if tensor_h < target_h or tensor_w < target_w:
        raise ValueError('common tensor cannot shrink a view')
    # Side views keep their own resize; larger batches add padding, never upscale sides.
    top, left = (tensor_h-rh)//2, (tensor_w-rw)//2
    packed = np.full((tensor_h, tensor_w, 3), 114, np.uint8)
    packed[top:top+rh, left:left+rw] = cv2.resize(image, (rw, rh))
    return packed, ImageTransform((oh, ow), (rh, rw), top, left, (tensor_h, tensor_w))


def batch_groups(views: Sequence[str], mode: str):
    if len(views) != len(set(views)) or set(views)-set(SHAPES):
        raise ValueError('views must be unique front/left/right')
    if mode == 'sequential':
        return [[v] for v in views]
    if mode == 'batch3':
        return [list(views)] if views else []
    if mode == '2plus1':
        groups = []
        for view in views:
            if view == 'front':
                groups.append(['front'])
            elif not any(view in g for g in groups):
                groups.append([v for v in views if v in ('left', 'right')])
        return groups
    raise ValueError('unknown batch mode')
