"""Pinned Ultralytics integration: CPU-testable loader, full-coverage sampler and ignore loss.

No optimizer or model inference is started by importing this module.
"""
import hashlib
import json
import random
from pathlib import Path

import cv2
import numpy as np
import torch
from torch import nn
from torch.utils.data import Dataset, Sampler, DataLoader
import ultralytics
from ultralytics.utils.loss import v8DetectionLoss
from ultralytics.utils.tal import make_anchors

from camera_pretrain_policy import (CLASSES, epoch_order, letterbox_geometry,
    resolve_data_path, transform_boxes, validate_dataset_contract, view_masks, sha)


def classification_keep(batch, anchors, strides, foreground):
    """Ignore intersecting feature cells, not just their centers; valid positives win."""
    complete = batch['supervision_complete'].to(anchors.device).bool()
    keep = complete[:, None].expand(-1, len(anchors)).clone()
    # v9: synchronized semantic GT can certify background locally even when
    # the entire RGB frame has not been exhaustively annotated. Every pixel
    # in a feature cell must be safe; nearest-neighbour point sampling alone
    # would erase tiny unknown objects.
    if 'background_safe_grid' in batch:
        grid = batch['background_safe_grid'].to(anchors.device).float()
        base = 8
        for stride in strides.unique():
            step = int(stride.item())
            if step not in (8, 16, 32) or grid.ndim != 3:
                raise ValueError('unsupported spatial background grid/stride')
            reduced = 1-torch.nn.functional.max_pool2d(
                1-grid[:, None], step//base, step//base)[:, 0]
            chosen = strides[:, 0] == stride
            xy = (anchors[chosen]*strides[chosen]/step).long()
            safe = reduced[:, xy[:, 1], xy[:, 0]].bool()
            keep[:, chosen] |= safe
    boxes = batch['ignore_bboxes'].to(anchors.device)
    indexes = batch['ignore_batch_idx'].to(anchors.device).long()
    if len(boxes) != len(indexes) or foreground.shape != keep.shape:
        raise ValueError('supervision/anchor dimensions disagree')
    centers, half = anchors * strides, strides / 2
    for image in range(len(complete)):
        for box in boxes[indexes == image]:
            # Positive intersection between stride-cell footprint and native ignore box.
            hit = ((centers[:, 0]+half[:, 0] > box[0]) &
                   (centers[:, 0]-half[:, 0] < box[2]) &
                   (centers[:, 1]+half[:, 0] > box[1]) &
                   (centers[:, 1]-half[:, 0] < box[3]))
            keep[image, hit] = False
    return keep | foreground.bool()


def known_class_keep(batch, anchors, strides, foreground, classes):
    """Suppress duplicates of a documented class inside its known box only.

    Other classes in a partial image remain unknown. Full-cell containment
    and existing ignore regions protect neighbours, tiny GT and overlaps.
    """
    keep = torch.zeros((len(foreground), len(anchors), classes),
                       device=anchors.device, dtype=torch.bool)
    enabled = batch.get('known_class_background')
    if enabled is None or not enabled.any():
        return keep
    height, width = batch['img'].shape[-2:]
    centers, half = anchors*strides, strides/2
    boxes = batch['bboxes'].to(anchors.device)
    indices = batch['batch_idx'].to(anchors.device).long()
    labels = batch['cls'].to(anchors.device).flatten().long()
    allowed = classification_keep(dict(batch, supervision_complete=torch.ones(
        len(foreground), device=anchors.device, dtype=torch.bool)), anchors, strides,
        torch.zeros_like(foreground))
    for i, b in enumerate(boxes):
        image = int(indices[i])
        if not bool(enabled[image]):
            continue
        center = b[:2]*b.new_tensor([width, height])
        radius = b[2:]*b.new_tensor([width, height])/2
        hit = ((centers[:, 0]-half[:, 0] >= center[0]-radius[0]) &
               (centers[:, 0]+half[:, 0] <= center[0]+radius[0]) &
               (centers[:, 1]-half[:, 0] >= center[1]-radius[1]) &
               (centers[:, 1]+half[:, 0] <= center[1]+radius[1]))
        keep[image, :, labels[i]] |= hit & allowed[image]
    return keep


class _CaptureAssignment(nn.Module):
    def __init__(self, original, context):
        super().__init__()
        self.original, self.context = original, context

    def forward(self, *args, **kwargs):
        output = self.original(*args, **kwargs)
        self.context['foreground'] = output[3]
        return output


class _MaskedBCE(nn.Module):
    def __init__(self, original, context):
        super().__init__()
        self.original, self.context = original, context

    def forward(self, prediction, target):
        ctx = self.context
        keep = classification_keep(ctx['batch'], ctx['anchors'], ctx['strides'], ctx['foreground'])
        if 'known_class_background' in ctx['batch']:
            class_keep = known_class_keep(ctx['batch'], ctx['anchors'], ctx['strides'],
                ctx['foreground'], prediction.shape[-1])
            return self.original(prediction, target) * (keep.unsqueeze(-1) | class_keep)
        return self.original(prediction, target) * keep.unsqueeze(-1)


class IgnoreDetectionLoss(v8DetectionLoss):
    @classmethod
    def wrap(cls, original):
        if type(original) is not v8DetectionLoss:
            raise ValueError('unsupported detection criterion: '+type(original).__name__)
        new = cls.__new__(cls)
        new.__dict__.update(original.__dict__)
        new._supervision_context = {}
        new.assigner = _CaptureAssignment(original.assigner, new._supervision_context)
        new.bce = _MaskedBCE(original.bce, new._supervision_context)
        return new

    def get_assigned_targets_and_loss(self, preds, batch):
        for key in ('supervision_complete', 'ignore_bboxes', 'ignore_batch_idx'):
            if key not in batch:
                raise ValueError('ignore-aware batch missing '+key)
        anchors, strides = make_anchors(preds['feats'], self.stride, 0.5)
        self._supervision_context.update(batch=batch, anchors=anchors, strides=strides)
        try:
            return super().get_assigned_targets_and_loss(preds, batch)
        finally:
            self._supervision_context.clear()


def wrap_criterion(original):
    """Retain the original E2E weighting schedule and assignment parameters."""
    if ultralytics.__version__ != '8.4.150':
        raise ValueError('ignore loss requires validated Ultralytics 8.4.150')
    if hasattr(original, 'one2many') and hasattr(original, 'one2one'):
        original.one2many = IgnoreDetectionLoss.wrap(original.one2many)
        original.one2one = IgnoreDetectionLoss.wrap(original.one2one)
        return original
    return IgnoreDetectionLoss.wrap(original)


class PretrainDataset(Dataset):
    rect = False
    mosaic = False

    def __init__(self, root, role='train', augment=False):
        self.root = Path(root)
        self.meta = validate_dataset_contract(root, ignore_aware=True)
        self.policy = json.loads((self.root/'policy.json').read_text())
        required = self.meta.get('requires_spatial_background_trainer')
        if required and (required != 2 or self.policy.get('spatial_background_version') != required):
            raise ValueError('spatial background policy/loader contract mismatch')
        with (self.root/'manifest.jsonl').open() as f:
            self.rows = [json.loads(line) for line in f if line.strip()]
        self.rows = [r for r in self.rows if r['split'] == role]
        if required and any(r.get('background_policy') != 'spatial_v2' or
                            'known_class_background' not in r for r in self.rows):
            raise ValueError('spatial background supervision missing')
        if not self.rows:
            raise ValueError('no images for '+role)
        self.augment = augment
        self.imgsz = self.policy['imgsz']
        self.masks = view_masks(self.policy)
        self.labels = []
        self.im_files = []
        for r in self.rows:
            if r['ego_mask_applied'] and r['mask_config_sha256'] != self.masks[r['view']].config_sha256:
                raise ValueError('runtime Ego mask differs from dataset policy')
            path = resolve_data_path(r['image'], self.root, self.meta['original_data_root'])
            self.im_files.append(str(path))
            anns = [a for a in r['annotations'] if a['supervision']=='positive']
            self.labels.append(dict(cls=np.array([CLASSES.index(a['training_class']) for a in anns],
                dtype=np.float32).reshape(-1, 1), bboxes=np.array([a['xyxy'] for a in anns],
                dtype=np.float32).reshape(-1, 4)))

    def __len__(self):
        return len(self.rows)

    def __getitem__(self, index):
        row = self.rows[index]
        path = Path(self.im_files[index])
        raw = path.read_bytes()
        if hashlib.sha256(raw).hexdigest() != row['image_sha256']:
            raise ValueError('source RGB SHA changed since preparation')
        image = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
        if image is None or image.shape[:2] != (row['height'], row['width']):
            raise ValueError('source RGB shape changed')
        if row['ego_mask_applied']:
            image = self.masks[row['view']].apply(image)
        width, height = row['width'], row['height']
        _, _, left, top, rw, rh = letterbox_geometry(width, height, self.imgsz)
        image = cv2.resize(image, (rw, rh), interpolation=cv2.INTER_LINEAR)
        image = cv2.copyMakeBorder(image, top, self.imgsz-rh-top, left, self.imgsz-rw-left,
                                  cv2.BORDER_CONSTANT, value=(114, 114, 114))
        flip = self.augment and random.random() < self.policy['augmentation']['fliplr']
        if self.augment:
            from ultralytics.data.augment import RandomHSV
            aug = self.policy['augmentation']
            image = RandomHSV(aug['hsv_h'], aug['hsv_s'], aug['hsv_v'])({'img':image})['img']
        if flip:
            image = np.fliplr(image)
        positives = [a for a in row['annotations'] if a['supervision']=='positive']
        ignores = [a for a in row['annotations'] if a['supervision']=='ignore']
        boxes = transform_boxes([a['xyxy'] for a in positives], width, height, self.imgsz, flip)
        xywh = np.empty_like(boxes)
        xywh[:, :2] = (boxes[:, :2]+boxes[:, 2:])/2/self.imgsz
        xywh[:, 2:] = (boxes[:, 2:]-boxes[:, :2])/self.imgsz
        sample = dict(img=torch.from_numpy(np.ascontiguousarray(image[..., ::-1].transpose(2, 0, 1))),
            cls=torch.tensor([CLASSES.index(a['training_class']) for a in positives], dtype=torch.float32).reshape(-1, 1),
            bboxes=torch.from_numpy(xywh), batch_idx=torch.zeros(len(positives), dtype=torch.float32),
            ignore_bboxes=torch.from_numpy(transform_boxes([a['xyxy'] for a in ignores], width, height, self.imgsz, flip)),
            ignore_batch_idx=torch.zeros(len(ignores), dtype=torch.long),
            supervision_complete=torch.tensor(row['supervision'] in ('complete', 'reviewed_negative')),
            im_file=str(path), ori_shape=(height, width), resized_shape=(self.imgsz, self.imgsz),
            ratio_pad=((rw/width, rh/height), (left, top)), dataset_id=row['id'])
        if self.policy.get('spatial_background_version') == 2:
            grid = np.zeros((self.imgsz//8, self.imgsz//8), dtype=bool)
            if row.get('background_safe_grid'):
                file = self.root/row['background_safe_grid']
                if sha(file) != row['background_safe_grid_sha256']:
                    raise ValueError('background supervision SHA changed')
                with np.load(file) as stored:
                    grid = np.unpackbits(stored['packed']).reshape(grid.shape).astype(bool)
                if flip:
                    grid = np.fliplr(grid)
            sample['background_safe_grid'] = torch.from_numpy(np.ascontiguousarray(grid))
            sample['known_class_background'] = torch.tensor(row['known_class_background'])
        return sample

    @staticmethod
    def collate_fn(samples):
        result = {}
        for k in samples[0]:
            values = [s[k] for s in samples]
            if k in ('img', 'supervision_complete', 'background_safe_grid', 'known_class_background'):
                result[k] = torch.stack(values)
            elif k in ('cls', 'bboxes', 'ignore_bboxes'):
                result[k] = torch.cat(values)
            elif k in ('batch_idx', 'ignore_batch_idx'):
                result[k] = torch.cat([v+i for i, v in enumerate(values)])
            else:
                result[k] = tuple(values)
        return result


class FullCoverageSampler(Sampler):
    def __init__(self, dataset, batch_size=4):
        self.dataset, self.batch_size, self.epoch = dataset, batch_size, 0
        self.order, self.report = epoch_order(dataset.rows, dataset.policy['seed'], batch_size=batch_size, alignment=16)

    def set_epoch(self, epoch):
        self.epoch = epoch
        self.order, self.report = epoch_order(self.dataset.rows, self.dataset.policy['seed'], epoch,
                                              self.batch_size, alignment=16)

    def __len__(self):
        return len(self.order)

    def __iter__(self):
        return iter(self.order)


def seed_worker(worker_id):
    seed = torch.initial_seed() % 2**32
    np.random.seed(seed)
    random.seed(seed)


def build_pretrain_loader(root, mode, batch_size, workers=2):
    dataset = PretrainDataset(root, 'train' if mode=='train' else 'selection', augment=mode=='train')
    sampler = FullCoverageSampler(dataset, batch_size) if mode=='train' else None
    generator = torch.Generator().manual_seed(dataset.policy['seed'])
    return DataLoader(dataset, batch_size=batch_size, sampler=sampler, shuffle=False,
        num_workers=workers, collate_fn=dataset.collate_fn, worker_init_fn=seed_worker,
        generator=generator, pin_memory=True, drop_last=False)
