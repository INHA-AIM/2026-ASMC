"""Decode once and retain per-frame failures while batching valid observations."""
import time
import cv2
import numpy as np
from .messages import build_box_array


def process_batch(frames, processors, backend, include_raw=False):
    images, rows = {}, {}
    for frame in frames:
        p = processors[frame.view]
        p.last_image, p.last_masks = None, []
        p.last_counts = dict(raw=0, kept=0, excluded=0)
        timings = {}
        start = time.monotonic()
        try:
            image = p.decode(np.frombuffer(frame.data, np.uint8), cv2.IMREAD_COLOR)
            if image is None:
                raise ValueError('JPEG decoder returned no image')
            p.last_image = image
            h, w = image.shape[:2]
            timings['decode_ms'] = (time.monotonic()-start)*1000
        except Exception as exc:
            rows[frame.view] = (frame, build_box_array(frame.header,0,0,False,[],p.groups),None,timings,('decode',str(exc)))
            continue
        start = time.monotonic()
        try:
            images[frame.view] = p.hood_mask.apply(image)
            timings['hood_mask_ms'] = (time.monotonic()-start)*1000
            rows[frame.view] = (frame, w, h, timings)
        except Exception as exc:
            rows[frame.view] = (frame, build_box_array(frame.header,w,h,False,[],p.groups),None,timings,('mask',str(exc)))
    error, detected = None, {}
    if images:
        try:
            detected = backend.infer_images(images, list(images))
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
            output.append((frame,build_box_array(frame.header,w,h,False,[],p.groups),None,timings,error))
            continue
        result = detected[frame.view]
        timings.update(result.timings)
        raw = build_box_array(frame.header,w,h,True,result.detections,p.groups) if include_raw else None
        try:
            kept, excluded = p.ego_hood_filter.partition(result.detections,w,h)
            # Preserve result/mask correspondence if a future explicit filter is used.
            p.last_masks = [m for d,m in zip(result.detections,result.masks) if any(d is k for k in kept)]
            p.last_counts = dict(raw=len(result.detections),kept=len(kept),excluded=len(excluded))
            message = build_box_array(frame.header,w,h,True,kept,p.groups)
            output.append((frame,message,raw,timings,None))
        except Exception as exc:
            output.append((frame,build_box_array(frame.header,w,h,False,[],p.groups),raw,timings,('filter',str(exc))))
    return output
