"""Debug drawing only; detector coordinates and operational results stay intact."""
import math

import cv2


def render_object_overlay(image, detection):
    output = image.copy()
    height, width = output.shape[:2]
    color = (40, 220, 40) if detection.valid else (40, 40, 230)
    title = 'YOLO objects: {}'.format(len(detection.boxes)) if detection.valid else 'YOLO result INVALID'
    cv2.putText(output, title, (10, 26), cv2.FONT_HERSHEY_SIMPLEX, .65, color, 2)
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
