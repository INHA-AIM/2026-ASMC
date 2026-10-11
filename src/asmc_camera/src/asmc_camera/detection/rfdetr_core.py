"""Torch-free RF-DETR preprocessing and official sigmoid/top-300 box decoding.

COCO IDs are sparse category slots, not contiguous Ultralytics IDs. Preserve the
official global query/class top-k before restricting output to COCO7. No NMS.
"""
import cv2
import numpy as np

from .backend import Detection

COCO7_IDS = {1: 'person', 2: 'bicycle', 3: 'car', 4: 'motorcycle',
             6: 'bus', 8: 'truck', 18: 'dog'}
COCO7 = ('person', 'dog', 'bicycle', 'car', 'motorcycle', 'bus', 'truck')
MEAN = np.array([.485, .456, .406], np.float32)
STD = np.array([.229, .224, .225], np.float32)


def validate_metadata(metadata, expected_sha256, tensorrt_version, shapes, require_fp16=True):
    if require_fp16 and not metadata.get('fp16_enabled'):
        raise ValueError('FP32 diagnostic engine is not an FP16 experiment candidate')
    if metadata.get('engine_sha256') != expected_sha256 or metadata.get('tensorrt') != tensorrt_version:
        raise ValueError('RF-DETR engine metadata/runtime mismatch')
    if metadata.get('class_ids') != {str(k):v for k,v in COCO7_IDS.items()}:
        raise ValueError('RF-DETR sparse COCO mapping mismatch')
    if metadata.get('output_names') != ['dets','labels'] or not metadata.get('input_name'):
        raise ValueError('RF-DETR raw output contract mismatch')
    if metadata.get('batch_profile') != {'minimum':1,'optimum':3,'maximum':3}:
        raise ValueError('RF-DETR must support actual batch 1/2/3')
    if any(list(s)!=metadata.get('shape') for s in shapes.values()):
        raise ValueError('RF-DETR fixed engine shape mismatch')


def preprocess(image, shape):
    if image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8:
        raise ValueError('RF-DETR requires uint8 BGR images')
    if len(shape) != 2 or any(not isinstance(v, int) or v <= 0 or v % 32 for v in shape):
        raise ValueError('RF-DETR M/L shape must be positive multiples of 32')
    rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.
    rgb = cv2.resize(rgb, (shape[1], shape[0]), interpolation=cv2.INTER_LINEAR)
    return np.ascontiguousarray(((rgb-MEAN)/STD).transpose(2, 0, 1))


def decode(boxes, logits, original_hw, confidence=.4, allowed_names=COCO7):
    boxes, logits = np.asarray(boxes), np.asarray(logits)
    if boxes.ndim != 3 or boxes.shape[-1] != 4 or logits.ndim != 3 or boxes.shape[:2] != logits.shape[:2]:
        raise ValueError('RF-DETR output shape mismatch')
    if logits.shape[2] not in (90, 91) or len(original_hw) != len(boxes):
        raise ValueError('RF-DETR COCO slot/batch mismatch')
    if not 0 <= confidence <= 1 or not np.isfinite(boxes).all() or not np.isfinite(logits).all():
        raise ValueError('invalid RF-DETR outputs/confidence')
    allowed = set(allowed_names)
    if not allowed.issubset(COCO7):
        raise ValueError('experiment only supports original COCO7 names')
    result = []
    for bb, ll, (h, w) in zip(boxes, logits, original_hw):
        if h<=0 or w<=0:
            raise ValueError('invalid original image dimensions')
        # Stable sorting matches the pinned official decoder's tie policy.
        flat = 1. / (1. + np.exp(-np.clip(ll.reshape(-1).astype(np.float32), -80, 80)))
        selected = np.argsort(-flat, kind='stable')[:300]
        scores = flat[selected]
        objects = []
        for index, score in zip(selected, scores):
            if score < confidence:
                continue
            query, cid = divmod(int(index), logits.shape[2])
            name = COCO7_IDS.get(cid)
            if name not in allowed:
                continue
            cx, cy, bw, bh = bb[query].astype(np.float64)
            x1, x2 = np.clip([(cx-bw/2)*w, (cx+bw/2)*w], 0, w)
            y1, y2 = np.clip([(cy-bh/2)*h, (cy+bh/2)*h], 0, h)
            if x2 <= x1 or y2 <= y1:
                continue
            objects.append(Detection(float(x1), float(y1), float(x2), float(y2), name, float(score)))
        result.append(objects)
    return result


def groups(order, mode, shapes):
    if not order or len(set(order)) != len(order) or set(order)-set(shapes):
        raise ValueError('invalid RF-DETR view order')
    if mode == 'batch3':
        if len({tuple(shapes[v]) for v in order}) != 1:
            raise ValueError('batch3 requires equal RF-DETR shapes')
        return [list(order)]
    if mode == '2plus1':
        sides = [v for v in order if v != 'front']
        if len({tuple(shapes[v]) for v in sides}) > 1:
            raise ValueError('side batch requires equal RF-DETR shapes')
        return ([sides] if sides else []) + ([['front']] if 'front' in order else [])
    raise ValueError('RF-DETR experiment supports batch3 or 2plus1')
