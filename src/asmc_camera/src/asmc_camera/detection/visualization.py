"""Debug drawing only; detector coordinates and operational results stay intact."""
import math

import cv2
import numpy as np


def render_object_overlay(image, detection, hood_mask=None, raw_detection=None, ego_hood_filter=None,
                          mask_label='hood mask'):
    output = image.copy()
    if hood_mask is not None and hood_mask.enabled:
        bitmap, polygon = hood_mask.geometry(*image.shape[:2])
        output[bitmap] = (output[bitmap].astype(float)*.7 + np.array([0,0,255])*.3).astype(np.uint8)
        cv2.polylines(output,[polygon],True,(0,0,255),2)
    height, width = output.shape[:2]
    color = (40, 220, 40) if detection.valid else (40, 40, 230)
    title = 'YOLO objects: {}'.format(len(detection.boxes)) if detection.valid else 'YOLO result INVALID'
    cv2.putText(output, title, (10, 26), cv2.FONT_HERSHEY_SIMPLEX, .65, color, 2)
    if hood_mask is not None:
        label = mask_label + ' ON ' + hood_mask.config_sha256[:8] if hood_mask.enabled else mask_label + ' OFF'
        cv2.putText(output,label,(10,50),cv2.FONT_HERSHEY_SIMPLEX,.55,(255,255,255),1)
    if ego_hood_filter is not None:
        state = 'ON ' + ego_hood_filter.config_sha256[:8] if ego_hood_filter.enabled else 'OFF'
        cv2.putText(output, 'ego hood FILTER '+state, (10,74), cv2.FONT_HERSHEY_SIMPLEX,.55,(255,255,255),1)
    if raw_detection is not None and raw_detection.valid and detection.valid:
        from collections import Counter
        def key(box):
            return (box.xmin,box.ymin,box.xmax,box.ymax,box.class_name,
                    box.source_class_name,box.confidence,box.confidence_valid)
        remaining = Counter(key(box) for box in detection.boxes)
        excluded = []
        for box in raw_detection.boxes:
            if remaining[key(box)]:
                remaining[key(box)] -= 1
            else:
                excluded.append(box)
        for box in excluded:
            if not all(math.isfinite(v) for v in (box.xmin,box.ymin,box.xmax,box.ymax)):
                continue
            x1,y1,x2,y2 = [int(v) for v in (box.xmin,box.ymin,box.xmax,box.ymax)]
            cv2.rectangle(output,(x1,y1),(x2,y2),(160,160,160),2)
            cv2.putText(output,'excluded: ego hood',(max(0,x1),max(98,y1-6)),cv2.FONT_HERSHEY_SIMPLEX,.5,(180,180,180),1)
        cv2.putText(output, 'raw %d / kept %d / excluded %d' % (len(raw_detection.boxes),len(detection.boxes),len(excluded)),
                    (10,98),cv2.FONT_HERSHEY_SIMPLEX,.5,(255,255,255),1)
    if not detection.valid:
        return output
    for box in detection.boxes:
        values = (box.xmin, box.ymin, box.xmax, box.ymax, box.confidence)
        if not box.confidence_valid or not all(math.isfinite(v) for v in values):
            continue
        if box.xmax <= box.xmin or box.ymax <= box.ymin:
            continue
        if box.xmax < 0 or box.ymax < 0 or box.xmin >= width or box.ymin >= height:
            continue
        x1, x2 = (max(0, min(width-1, int(v))) for v in (box.xmin, box.xmax))
        y1, y2 = (max(0, min(height-1, int(v))) for v in (box.ymin, box.ymax))
        cv2.rectangle(output, (x1,y1), (x2,y2), color, 2)
        label = '{} [{}] {:.2f}'.format(box.source_class_name, box.class_name, box.confidence)
        cv2.putText(output, label, (x1,max(45,y1-6)), cv2.FONT_HERSHEY_SIMPLEX, .5, color, 1)
    return output
