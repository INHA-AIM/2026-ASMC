"""Explicit post-inference class-aware NMS; never changes matching/GT rules."""
import math

POLICY = {'name': 'same_class_nms_v1', 'iou': .7, 'confidence': .4,
          'class_agnostic': False, 'comparison': 'IoU strictly greater than threshold',
          'tie_break': 'stable original prediction order',
          'scope': 'postprocess existing decoded predictions; no head switch'}


def iou(a, b):
    inter = max(0, min(a[2], b[2])-max(a[0], b[0])) * max(0, min(a[3], b[3])-max(a[1], b[1]))
    union = (a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1])-inter
    return inter/union if union > 0 else 0.


def suppress(predictions, confidence=.4, threshold=.7):
    if not 0 <= confidence <= 1 or not 0 <= threshold <= 1:
        raise ValueError('Invalid confidence/IoU')
    eligible = []
    for i, pred in enumerate(predictions):
        box, score = pred['xyxy'], pred['confidence']
        if len(box) != 4 or not all(math.isfinite(v) for v in [*box, score]) or not 0 <= score <= 1:
            raise ValueError('Invalid prediction')
        if box[2] < box[0] or box[3] < box[1] or not pred['class_name']:
            raise ValueError('Invalid bbox/class')
        if score >= confidence:
            eligible.append(i)
    keep, removed = [], []
    for index in sorted(eligible, key=lambda i: -predictions[i]['confidence']):
        pred = predictions[index]
        winner = next((j for j in keep if predictions[j]['class_name'] == pred['class_name']
                       and iou(predictions[j]['xyxy'], pred['xyxy']) > threshold), None)
        if winner is None:
            keep.append(index)
        else:
            removed.append({'index': index, 'kept_index': winner, 'iou': iou(predictions[winner]['xyxy'], pred['xyxy'])})
    return [predictions[i] for i in keep], removed
