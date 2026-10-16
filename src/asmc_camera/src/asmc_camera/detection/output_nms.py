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


def nms_box(prediction):
    # An offline cache may retain the actual packed-tensor coordinates.
    # Runtime NMS precedes source-image clipping, which can change IoU.
    return prediction.get('nms_xyxy', prediction['xyxy'])


def suppress(predictions, confidence=.4, threshold=.7, exclusive_iou=None):
    if not 0 <= confidence <= 1 or not 0 <= threshold <= 1 or (exclusive_iou is not None and not 0 <= exclusive_iou <= 1):
        raise ValueError('Invalid confidence/IoU')
    eligible = []
    for i, pred in enumerate(predictions):
        box, score = nms_box(pred), pred['confidence']
        geometries = (box, pred['xyxy'])
        if any(len(b) != 4 or not all(math.isfinite(v) for v in b) for b in geometries) or not math.isfinite(score) or not 0 <= score <= 1:
            raise ValueError('Invalid prediction')
        if any(b[2] < b[0] or b[3] < b[1] for b in geometries) or not pred['class_name']:
            raise ValueError('Invalid bbox/class')
        if score >= confidence:
            eligible.append(i)
    keep, removed = [], []
    for index in sorted(eligible, key=lambda i: -predictions[i]['confidence']):
        pred = predictions[index]
        winner = next((j for j in keep if predictions[j]['class_name'] == pred['class_name']
                       and iou(nms_box(predictions[j]), nms_box(pred)) > threshold), None)
        if winner is None:
            keep.append(index)
        else:
            removed.append({'index': index, 'kept_index': winner, 'iou': iou(nms_box(predictions[winner]), nms_box(pred))})
    if exclusive_iou is not None:
        # Only these mutually exclusive names may describe one object. Never
        # suppress a rider/person against a bicycle, stroller or motorcycle.
        families = ({'car', 'bus', 'truck'}, {'dog', 'deer'})
        exclusive_keep = []
        for index in keep:
            pred = predictions[index]
            family = next((f for f in families if pred['class_name'] in f), set())
            winner = next((j for j in exclusive_keep if predictions[j]['class_name'] != pred['class_name']
                and predictions[j]['class_name'] in family
                and iou(nms_box(predictions[j]), nms_box(pred)) > exclusive_iou), None)
            if winner is None:
                exclusive_keep.append(index)
            else:
                removed.append(dict(index=index, kept_index=winner,
                    iou=iou(nms_box(predictions[winner]), nms_box(pred)), reason='exclusive_names'))
        keep = exclusive_keep
    return [predictions[i] for i in keep], removed
