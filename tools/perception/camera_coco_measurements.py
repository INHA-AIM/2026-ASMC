"""Pure checks and sampling for the fixed YOLO11m COCO experiment."""
from collections import defaultdict

BASELINE_NAMES = frozenset(('bus', 'truck', 'person', 'car'))


def validate_profiles(baseline, expanded, groups, model_names=None):
    errors = []
    if set(baseline.get('allowed_names', [])) != BASELINE_NAMES:
        errors.append('baseline must retain the original four labels')
    names = expanded.get('allowed_names', [])
    if not isinstance(names, list) or len(names) != 80 or len(set(names)) != 80:
        errors.append('expanded allowlist must contain exactly 80 unique labels')
    for key in set(baseline) | set(expanded):
        if key != 'allowed_names' and baseline.get(key) != expanded.get(key):
            errors.append('class expansion changed '+key)
    if groups.get('excluded') != []:
        errors.append('expanded groups must preserve infrastructure labels')
    if model_names is not None and set(names) != set(model_names.values()):
        errors.append('expanded allowlist does not match checkpoint names')
    return errors


def make_protocol(config, phases):
    jobs = []
    for repeat in range(1, config['repeats']+1):
        profiles = ['baseline', 'all'] if repeat % 2 else ['all', 'baseline']
        for phase in ('camera', 'debug'):
            if phase in phases:
                jobs += [dict(phase='camera', classes=name, bag_id='camera', repeat=repeat,
                              debug=phase == 'debug') for name in profiles]
    if 'fusion' in phases:
        for repeat in range(1, config['fusion']['repeats']+1):
            profiles = ['baseline', 'all'] if repeat % 2 else ['all', 'baseline']
            for bag_id in config['fusion']['bags']:
                jobs += [dict(phase='fusion', classes=name, bag_id=bag_id, repeat=repeat,
                              debug=False) for name in profiles]
    return jobs


def select_gallery(rows):
    """First, highest confidence, and three uniform samples; at most five/label."""
    by_label = defaultdict(list)
    for index, row in enumerate(rows):
        found = {box['source_class_name'] for box in row['all']}-BASELINE_NAMES
        for label in found:
            by_label[label].append(index)
    selected = {}
    for label, indices in sorted(by_label.items()):
        best = max(indices, key=lambda i: max(
            box['confidence'] for box in rows[i]['all'] if box['source_class_name'] == label))
        uniform = [indices[round((len(indices)-1)*fraction)] for fraction in (0., .5, 1.)]
        selected[label] = sorted(set([indices[0], best]+uniform))
    return selected


def source_box_signature(boxes):
    return [(box['source_class_name'], tuple(box['xyxy']), box['confidence']) for box in boxes]


def common_label_comparison(reference, expanded):
    """Keep relative ordering; do not silently disregard a top-max_det change."""
    retained = [box for box in expanded if box['source_class_name'] in BASELINE_NAMES]
    return dict(exact=source_box_signature(reference) == source_box_signature(retained),
                baseline_count=len(reference), retained_count=len(retained),
                baseline=reference, retained=retained)
