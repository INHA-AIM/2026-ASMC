"""Opt-in bounded raw uint8 uploads and model-specific GPU preprocessing.

No CUDA pointer crosses ROS. Leases include queued GPU work, not just Python use.
The CPU geometry/mask raster remains authoritative; interpolation is validated
against OpenCV before this candidate can replace the separate CPU input path.
"""
from contextlib import contextmanager
from dataclasses import dataclass
import threading

import numpy as np

from asmc_camera.camera.geometry import compute_letterbox_geometry
from .shared_input import CacheBusy

_constants = {}
_constants_lock = threading.RLock()


def _constant(key, build):
    with _constants_lock:
        if key not in _constants:
            if len(_constants) >= 32:
                del _constants[next(iter(_constants))]
            value = build()
            event = None
            if value[0].is_cuda:
                import torch
                event = torch.cuda.Event()
                event.record(torch.cuda.current_stream(value[0].device))
            _constants[key] = value, event
        value, event = _constants[key]
        if event is not None:
            import torch
            torch.cuda.current_stream(value[0].device).wait_event(event)
        return value


@dataclass(frozen=True)
class PreprocessSpec:
    target_hw: tuple
    interpolation: str
    mask_sha: str = ''
    dtype: str = 'float32'
    layout: str = 'contiguous'
    padding: int = 114
    color: str = 'RGB'
    normalization: str = '1/255'


class RawGpuFrame:
    def __init__(self, image, device):
        import torch
        self.torch = torch
        self.device = device
        self.original_hw = tuple(image.shape[:2])
        self.upload_stream = torch.cuda.Stream(device=device)
        # Keep CPU storage alive until the upload and all leases are finished.
        self.host = np.array(image, copy=True)
        with torch.cuda.stream(self.upload_stream):
            self.tensor = torch.from_numpy(self.host).to(device)
            self.ready = torch.cuda.Event()
            self.ready.record()
        self.lock = threading.RLock()
        self.prepared = {}
        self.events = []
        self.upload_bytes = image.nbytes

    def acquire(self):
        self.torch.cuda.current_stream(self.device).wait_event(self.ready)

    def release(self):
        event = self.torch.cuda.Event()
        event.record(self.torch.cuda.current_stream(self.device))
        with self.lock:
            self.events = [e for e in self.events if not e.query()] + [event]

    def idle(self):
        with self.lock:
            return self.ready.query() and all(e.query() for e in self.events)

    def close(self):
        self.ready.synchronize()
        for event in self.events:
            event.synchronize()
        self.prepared.clear()


class RawGpuCache:
    def __init__(self, device='cuda:0', factory=RawGpuFrame):
        self.device, self.factory = device, factory
        self.rows = {}
        self.lock = threading.RLock()
        self.condition = threading.Condition(self.lock)
        self.uploads = self.hits = self.busy = self.bytes = 0
        self.prepares = self.prepared_hits = self.resizes = 0
        self.closed = False

    @contextmanager
    def lease(self, camera, key, image):
        with self.lock:
            if self.closed:
                raise RuntimeError('GPU input cache is closed')
            rows = self.rows.setdefault(camera, [])
            row = next((r for r in rows if r['key'] == key), None)
            if row is None:
                if len(rows) == 2:
                    old = next((r for r in rows if r['refs'] == 0 and r['value'].idle()), None)
                    if old is None:
                        self.busy += 1
                        raise CacheBusy('both GPU input slots are in use')
                    rows.remove(old)
                value = self.factory(image, self.device)
                value.metrics = self
                row = dict(key=key, value=value, refs=0)
                rows.append(row)
                self.uploads += 1
                self.bytes += image.nbytes
            else:
                self.hits += 1
            row['refs'] += 1
        try:
            row['value'].acquire()
            yield row['value']
        finally:
            with self.lock:
                try:
                    row['value'].release()
                finally:
                    row['refs'] -= 1
                    self.condition.notify_all()

    def close(self):
        with self.lock:
            self.closed = True
            if not self.condition.wait_for(lambda: all(r['refs'] == 0 for rows in self.rows.values() for r in rows), timeout=5.):
                raise RuntimeError('GPU input leases still active during shutdown')
            values = [r['value'] for rows in self.rows.values() for r in rows]
        for value in values:
            value.close()
        with self.lock:
            self.rows.clear()


def _area_axis(value, source, target, axis):
    """Separable pixel-overlap area resize (adaptive pooling is different)."""
    import torch
    if source == target:
        return value
    if target > source:
        raise ValueError('area path requires downsampling')
    scale = source / target
    start = np.arange(target, dtype=np.float64) * scale
    end = start + scale
    output = None
    for offset in range(int(np.ceil(scale)) + 1):
        index = np.floor(start).astype(np.int64) + offset
        weight = np.maximum(0., np.minimum(end, index + 1.) - np.maximum(start, index)) / scale
        indices, weights = _constant(('area', source, target, offset, str(value.device)), lambda: (
            torch.as_tensor(np.clip(index, 0, source-1), device=value.device),
            torch.as_tensor(weight, dtype=torch.float32, device=value.device)))
        shape = [1] * value.ndim
        shape[axis] = target
        term = value.index_select(axis, indices) * weights.view(shape)
        output = term if output is None else output + term
    return output


def _linear_uint8(image, target_hw):
    """Match OpenCV's uint8 INTER_LINEAR fixed-point vertical SIMD rounding."""
    import torch
    def axis(source, target):
        key = ('linear', source, target, str(image.device))
        def build():
            fraction = ((np.arange(target)+.5)*source/target-.5).astype(np.float32)
            index = np.floor(fraction).astype(np.int64)
            fraction -= index
            fraction = np.where((index < 0) | (index >= source-1), 0., fraction)
            index = np.clip(index, 0, source-1)
            values = (index, np.minimum(index+1, source-1),
                      np.rint((1-fraction)*2048).astype(np.int32),
                      np.rint(fraction*2048).astype(np.int32))
            return tuple(torch.as_tensor(v, device=image.device) for v in values)
        return _constant(key, build)
    height, width = image.shape[:2]
    target_h, target_w = target_hw
    x0,x1,a,b = axis(width,target_w)
    y0,y1,c,d = axis(height,target_h)
    value = image.to(torch.int32)
    horizontal = value.index_select(1,x0)*a[None,:,None] + value.index_select(1,x1)*b[None,:,None]
    return (((horizontal.index_select(0,y0)>>4)*c[:,None,None]>>16)
          + ((horizontal.index_select(0,y1)>>4)*d[:,None,None]>>16)+2)>>2


def prepare(raw, spec, mask=None):
    import torch
    import torch.nn.functional as F
    if spec.interpolation not in ('linear', 'area') or spec.dtype not in ('float16', 'float32'):
        raise ValueError('unsupported GPU preprocessing spec')
    if spec.color != 'RGB' or spec.normalization not in ('1/255', 'divide255') or spec.layout != 'contiguous':
        raise ValueError('unsupported color/range/layout')
    geometry = compute_letterbox_geometry(raw.original_hw, spec.target_hw)
    raw.acquire()  # The consuming worker may use a non-default CUDA stream.
    with raw.lock:
        cached = raw.prepared.get(spec)
        if cached is not None:
            tensor, ready = cached
            torch.cuda.current_stream(raw.device).wait_event(ready)
            raw.metrics.prepared_hits += 1
            return tensor, geometry, True
        if len(raw.prepared) >= 4:
            raise CacheBusy('too many preprocessing variants for one frame')
        image = raw.tensor
        if mask is not None and mask.enabled:
            if spec.mask_sha != mask.config_sha256:
                raise ValueError('GPU mask/config mismatch')
            bitmap, _ = mask.geometry(*raw.original_hw)
            excluded, fill = _constant(('mask', raw.original_hw, spec.mask_sha, mask.fill_bgr, str(raw.device)), lambda: (
                torch.as_tensor(np.array(bitmap, copy=True), device=raw.device),
                torch.tensor(mask.fill_bgr, dtype=torch.uint8, device=raw.device)))
            image = torch.where(excluded[..., None], fill, image)
        elif spec.mask_sha:
            raise ValueError('GPU preprocessing mask missing')
        rh, rw = geometry.resized_hw
        value = image.permute(2, 0, 1).unsqueeze(0).float()
        if (rh, rw) != raw.original_hw:
            if spec.interpolation == 'area':
                value = _area_axis(value, raw.original_hw[1], rw, 3)
                value = _area_axis(value, raw.original_hw[0], rh, 2)
            else:
                value = _linear_uint8(image, (rh,rw)).permute(2,0,1).unsqueeze(0).float()
            value = value.round().clamp_(0, 255)
            raw.metrics.resizes += 1
        left, top, right, bottom = geometry.pad_ltrb
        value = F.pad(value, (left, right, top, bottom), value=float(spec.padding))
        tensor = value[0].flip(0).contiguous()
        if spec.dtype == 'float16':
            tensor = tensor.half()
        if spec.dtype == 'float16' or spec.normalization == 'divide255':
            tensor.div_(255.)
        else:
            tensor.mul_(1. / 255.)
        ready = torch.cuda.Event()
        ready.record()
        raw.prepared[spec] = tensor, ready
        raw.metrics.prepares += 1
        return tensor, geometry, False
