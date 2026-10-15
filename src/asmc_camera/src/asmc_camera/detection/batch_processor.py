"""Decode once and retain per-frame failures while batching valid observations."""
from contextlib import ExitStack
import time
import cv2
import numpy as np
from .messages import build_box_array


def process_batch(frames, processors, backend, include_raw=False):
    with ExitStack() as leases:
        return _process_batch(frames, processors, backend, include_raw, leases)


def _process_batch(frames, processors, backend, include_raw, leases):
    images, rows = {}, {}
    gpu_frames, gpu_masks = {}, {}
    gpu_enabled = bool(frames) and getattr(backend, 'supports_gpu_input', False) and all(
        getattr(getattr(f, 'decode_source', None), 'gpu_enabled', False) for f in frames)
    for frame in frames:
        p = processors[frame.view]
        p.last_image, p.last_masks = None, []
        p.last_counts = dict(raw=0, kept=0, excluded=0)
        timings = {}
        start = time.monotonic()
        try:
            image = (leases.enter_context(frame.decode_source.lease(frame)) if getattr(frame, 'decode_source', None)
                     else p.decode(np.frombuffer(frame.data, np.uint8), cv2.IMREAD_COLOR))
            if image is None:
                raise ValueError('JPEG decoder returned no image')
            p.last_image = image
            h, w = image.shape[:2]
            timings['decode_ms'] = (time.monotonic()-start)*1000
        except Exception as exc:
            rows[frame.view] = (frame, build_box_array(frame.header,0,0,False,[],p.groups, p.label_mode),None,timings,('decode',str(exc)))
            continue
        start = time.monotonic()
        try:
            if gpu_enabled:
                gpu_frames[frame.view] = leases.enter_context(frame.decode_source.gpu_lease(frame, image))
                gpu_masks[frame.view] = p.hood_mask
                images[frame.view] = image
            else:
                images[frame.view] = p.hood_mask.apply(image)
            timings['hood_mask_ms'] = (time.monotonic()-start)*1000
            rows[frame.view] = (frame, w, h, timings)
        except Exception as exc:
            rows[frame.view] = (frame, build_box_array(frame.header,w,h,False,[],p.groups, p.label_mode),None,timings,('mask',str(exc)))
    error, detected = None, {}
    if images:
        try:
            detected = (backend.infer_images(images, list(images), gpu_frames=gpu_frames, gpu_masks=gpu_masks)
                        if gpu_enabled else backend.infer_images(images, list(images)))
            if set(detected) != set(images):
                raise ValueError('backend omitted a batch view')
            if any(len(r.detections) != len(r.masks) for r in detected.values()):
                raise ValueError('backend mask/box count mismatch')
        except Exception as exc:
            error = ('inference', str(exc))
    output = []
    for frame in frames:
        row, p = rows[frame.view], processors[frame.view]
        if len(row) == 5:
            output.append(row)
            continue
        _, w, h, timings = row
        if error:
            output.append((frame,build_box_array(frame.header,w,h,False,[],p.groups, p.label_mode),None,timings,error))
            continue
        result = detected[frame.view]
        timings.update(result.timings)
        raw = build_box_array(frame.header,w,h,True,result.detections,p.groups, p.label_mode) if include_raw else None
        try:
            kept, excluded = p.ego_hood_filter.partition(result.detections,w,h)
            # Preserve result/mask correspondence if a future explicit filter is used.
            p.last_masks = [m for d,m in zip(result.detections,result.masks) if any(d is k for k in kept)]
            p.last_counts = dict(raw=len(result.detections),kept=len(kept),excluded=len(excluded))
            message = build_box_array(frame.header,w,h,True,kept,p.groups, p.label_mode)
            output.append((frame,message,raw,timings,None))
        except Exception as exc:
            output.append((frame,build_box_array(frame.header,w,h,False,[],p.groups, p.label_mode),raw,timings,('filter',str(exc))))
    return output
