"""Evaluation-only pixel scope: preserve raw GT, avoid inventing negatives."""
import math


def validate_scope(scope):
    if (not all(isinstance(scope.get(k),(int,float)) and math.isfinite(scope[k])
                and scope[k]>0 for k in ('min_long_edge','min_short_edge'))
            or scope['min_short_edge']>scope['min_long_edge']
            or not 0<scope.get('ignore_prediction_ioa',0)<=1):
        raise ValueError('invalid pixel evaluation scope')
    return scope


def in_scope(annotation, scope):
    if scope is None:
        return True
    box=annotation['xyxy']
    if len(box)!=4 or not all(math.isfinite(v) for v in box) or box[2]<=box[0] or box[3]<=box[1]:
        raise ValueError('invalid GT geometry cannot be hidden as tiny')
    width,height=box[2]-box[0],box[3]-box[1]
    return max(width,height)>=scope['min_long_edge'] and min(width,height)>=scope['min_short_edge']


def partition(annotations, scope):
    main,ignored=[],[]
    for annotation in annotations:
        (main if in_scope(annotation,scope) else ignored).append(annotation)
    return main,ignored


def ignored_prediction_indices(predictions, ignored_gt, matched_indices, scope):
    """Prediction IoA prevents a large wrong box being excused by a tiny GT."""
    if scope is None:
        return set()
    indices=set()
    for index,prediction in enumerate(predictions):
        if index in matched_indices:
            continue
        a=prediction['xyxy'];area=max(0,a[2]-a[0])*max(0,a[3]-a[1])
        if not area:
            continue
        for annotation in ignored_gt:
            b=annotation['xyxy']
            intersection=max(0,min(a[2],b[2])-max(a[0],b[0]))*max(0,min(a[3],b[3])-max(a[1],b[1]))
            if intersection/area>=scope['ignore_prediction_ioa']:
                indices.add(index)
                break
    return indices
