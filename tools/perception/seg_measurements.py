"""Seg selection metrics. No GT means unavailable, never a perfect score."""
from collections import Counter
import numpy as np
from scipy.optimize import linear_sum_assignment

GROUPS = ('person', 'animal', 'vehicle', 'obstacle')
INFRASTRUCTURE = {'traffic light', 'stop sign', 'fire hydrant', 'parking meter'}


def iou(a, b):
    x = max(0., min(a[2], b[2])-max(a[0], b[0]))
    y = max(0., min(a[3], b[3])-max(a[1], b[1]))
    intersection = x*y
    union = max(0., a[2]-a[0])*max(0., a[3]-a[1])+max(0., b[2]-b[0])*max(0., b[3]-b[1])-intersection
    return intersection/union if union else 0.


def match_boxes(gt, predicted, threshold=.5):
    if not gt or not predicted:
        return []
    overlaps = np.array([[iou(a['xyxy'], b['xyxy']) if a['group'] == b['group'] else 0.
                          for b in predicted] for a in gt])
    # Cardinality first, then IoU: a bad edge cannot displace two valid edges.
    score = (overlaps >= threshold).astype(float)*(min(len(gt),len(predicted))+1)+overlaps*(overlaps >= threshold)
    rows, cols = linear_sum_assignment(-score)
    return [(int(r),int(c),float(overlaps[r,c])) for r,c in zip(rows,cols) if overlaps[r,c] >= threshold]


def frame_metrics(gt, predicted, gt_masks=None, predicted_masks=None):
    pairs = match_boxes(gt, predicted)
    gmatched, pmatched = {r for r,_,_ in pairs}, {c for _,c,_ in pairs}
    counts = {g: Counter(gt=sum(x['group']==g for x in gt),
                         tp=sum(gt[r]['group']==g for r,_,_ in pairs),
                         fn=sum(x['group']==g and r not in gmatched for r,x in enumerate(gt)),
                         fp=sum(x['group']==g and c not in pmatched for c,x in enumerate(predicted))) for g in GROUPS}
    masks, names = [], Counter()
    for r,c,overlap in pairs:
        if gt[r].get('subtype_verified'):
            names['evaluated'] += 1
            names['correct'] += gt[r]['source_class_name'].lower() == predicted[c]['source_class_name'].lower()
        if gt_masks is not None and predicted_masks is not None and predicted_masks[c] is not None:
            a,b = gt_masks[r], predicted_masks[c]
            union = np.count_nonzero(a|b)
            masks.append(float(np.count_nonzero(a&b)/union) if union else 0.)
    return counts, masks, names, pairs


def summarize(counts, mask_values, names):
    groups = {}
    for group in GROUPS:
        c = counts[group]
        if not c['gt']:
            groups[group] = dict(evaluable=False, **dict(c))
            continue
        denominator = 2*c['tp']+c['fp']+c['fn']
        groups[group] = dict(evaluable=True, f1=2*c['tp']/denominator if denominator else 0.,
                             recall=c['tp']/c['gt'], precision=c['tp']/(c['tp']+c['fp']) if c['tp']+c['fp'] else 0., **dict(c))
    scores = [v['f1'] for v in groups.values() if v['evaluable']]
    return dict(groups=groups, macro_f1=float(np.mean(scores)) if scores else None,
                mask_iou=float(np.mean(mask_values)) if mask_values else None,
                mask_pairs=len(mask_values), fine_names=dict(names))


def choose_threshold(rows):
    available = [r for r in rows if r['metrics']['macro_f1'] is not None]
    return max(available, key=lambda r:(r['metrics']['macro_f1'],r['confidence'])) if available else None


def make_protocol(config, phases):
    jobs = []
    cells = [(m,v,p) for m in ('yolo26','yoloe') for v in ('front','all') for p in ('C1','C4','C5')]
    for repeat in range(1,config['repeats']+1):
        shifted = cells[repeat-1:]+cells[:repeat-1]
        if 'camera' in phases:
            jobs += [dict(phase='camera',bag_id='camera',model=m,views=v,profile=p,repeat=repeat,debug=False) for m,v,p in shifted]
        if 'debug' in phases:
            jobs += [dict(phase='camera',bag_id='camera',model=m,views='front',profile='C1',repeat=repeat,debug=True) for m in ('yolo26','yoloe')]
    if 'fusion' in phases:
        jobs += [dict(phase='fusion',bag_id=bag,model=m,views='front',profile='C1',repeat=repeat,debug=False)
                 for repeat in (1,2) for bag in ('primary','shadow') for m in ('baseline','yolo26','yoloe')]
    return jobs
