"""Offline bbox supervision policy. No Torch/ROS imports or source mutations."""
from collections import Counter, defaultdict, deque
import hashlib
import json
import math
from pathlib import Path

import cv2
import numpy as np

CLASSES = ('person', 'dog', 'deer', 'bicycle', 'car', 'motorcycle', 'bus', 'truck',
           'bollard', 'box', 'concrete_barrier', 'steel_barricade', 'traffic_drum',
           'plastic_barrier', 'traffic_cone', 'stroller')
ROOT = Path(__file__).resolve().parents[2]


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def uniform(seed, key):
    return (int(hashlib.sha256(f'{seed}:{key}'.encode()).hexdigest()[:16], 16) + 1) / (2**64 + 1)


def load_policy(path=None):
    path = Path(path or ROOT / 'config/datasets/camera_pretrain_policy_v1.json')
    p = json.loads(path.read_text())
    if p['schema_version'] != 'camera-pretrain-1' or p['maximum_repeats'] != 2:
        raise ValueError('unsupported supervision policy')
    if p['rect'] or p['multi_scale'] or p['imgsz'] != 960:
        raise ValueError('v1 requires square 960 without multiscale')
    p['policy_sha256'] = sha(path)
    return p


def view_masks(policy):
    import sys
    sys.path.insert(0, str(ROOT / 'src/asmc_camera/src'))
    from asmc_camera.detection.hood_mask import HoodMask
    return {v: HoodMask.load(True, ROOT / file) for v, file in policy['mask_files'].items()}


def letterbox_geometry(width, height, target=960):
    if min(width, height, target) <= 0:
        raise ValueError('positive dimensions required')
    gain = target / max(width, height)
    rw, rh = round(width * gain), round(height * gain)
    left, top = (target-rw)//2, (target-rh)//2
    return rw / width, rh / height, left, top, rw, rh


def transform_boxes(boxes, width, height, target=960, flip=False):
    boxes = np.asarray(boxes, dtype=np.float32).reshape(-1, 4).copy()
    gx, gy, left, top, _, _ = letterbox_geometry(width, height, target)
    boxes *= np.array([gx, gy, gx, gy], dtype=np.float32)
    boxes += np.array([left, top, left, top], dtype=np.float32)
    if flip:
        boxes[:, [0, 2]] = target - boxes[:, [2, 0]]
    return boxes


def validate_box(box, width, height):
    x1, y1, x2, y2 = map(float, box)
    if not all(math.isfinite(v) for v in box) or not 0 <= x1 < x2 <= width or not 0 <= y1 < y2 <= height:
        raise ValueError('invalid/out-of-image bbox')


def tiny_reasons(box, width, height, policy, visible_pixels=None):
    validate_box(box, width, height)
    reasons = []
    for scope, b in [('native', box), ('tensor', transform_boxes([box], width, height, policy['imgsz'])[0])]:
        w, h = float(b[2]-b[0]), float(b[3]-b[1])
        if min(w, h) < policy['tiny_short_px'] or w*h < policy['tiny_area_px2']:
            reasons.append(scope + '_near_point')
    if visible_pixels is not None and visible_pixels < policy['minimum_visible_pixels']:
        reasons.append('insufficient_visible_instance_pixels')
    return reasons


def supervision_state(row, native_route=False):
    if row.get('supervision') == 'reviewed_negative':
        return 'reviewed_negative'
    # Native actor/semantic completeness does not cover visually identical fixed
    # map drums/barriers. Only an explicit full16 RGB review proves background.
    review = row.get('review_record') or {}
    return 'complete' if (row.get('annotation_complete') is True and
        row.get('review_scope') == 'camera_objects_v2_16' and
        review.get('decision') == 'accept' and review.get('all_targets_annotated') is True and
        review.get('review_scope') == 'camera_objects_v2_16') else 'partial'


def resolve_data_path(path, dataset, original_data_root):
    path = Path(path)
    if path.is_file():
        return path
    try:
        relocated = Path(dataset).parents[1] / path.relative_to(original_data_root)
    except ValueError:
        raise ValueError('source file unavailable outside data root; mount its original directory read-only: ' + str(path))
    if not relocated.is_file():
        raise ValueError('missing relocated source: ' + str(relocated))
    return relocated


def class_images(rows):
    return Counter(c for r in rows for c in {a['training_class'] for a in r['annotations']
                   if a['supervision'] == 'positive'})


def epoch_order(rows, seed=20261011, epoch=0, batch_size=4, alignment=1):
    """Full coverage plus <=one extra per image; no replacement-only sampling."""
    freq = class_images(rows)
    reference = sorted(freq.values())[len(freq)//2] if freq else 1
    grouped = defaultdict(list)
    repeats = Counter()
    for i, row in enumerate(rows):
        labels = {a['training_class'] for a in row['annotations'] if a['supervision'] == 'positive'}
        weight = max([min(2.0, max(1.0, math.sqrt(reference/freq[c]))) for c in labels] or [1.0])
        # Keep epoch length fixed for LR, accumulation and epoch-boundary resume.
        n = 1 + int(uniform(seed, f'repeat:{row["id"]}') < weight-1)
        repeats[row['id']] = n
        grouped[row['correlation_group']].extend([i] * n)
    padding = (-sum(repeats.values())) % alignment
    available = sorted((i for i, r in enumerate(rows) if repeats[r['id']]==1),
                       key=lambda i: uniform(seed, 'alignment:'+rows[i]['id']))
    if len(available) < padding:
        raise ValueError('optimizer-boundary alignment exceeds maximum repeat allowance')
    for i in available[:padding]:
        repeats[rows[i]['id']] += 1
        grouped[rows[i]['correlation_group']].append(i)
    # Rotate groups round-robin; diverse groups first within each batch.
    groups = sorted(grouped, key=lambda g: uniform(seed, f'group:{epoch}:{g}'))
    queues = {g: deque(sorted(grouped[g], key=lambda i: uniform(seed, f'item:{epoch}:{rows[i]["id"]}')))
              for g in groups}
    active = deque(groups)
    order = []
    while active:
        g = active.popleft()
        order.append(queues[g].popleft())
        if queues[g]:
            active.append(g)
    exposures = Counter(c for i in order for c in {a['training_class'] for a in rows[i]['annotations']
                        if a['supervision'] == 'positive'})
    collisions = sum(len(batch)-len({rows[i]['correlation_group'] for i in batch})
                     for batch in (order[i:i+batch_size] for i in range(0, len(order), batch_size)))
    return order, dict(images=len(rows), exposures=len(order), class_image_frequency=dict(freq),
        class_exposures=dict(exposures), maximum_repeats=max(repeats.values(), default=0),
        all_images_included=len(set(order)) == len(rows), batch_group_collisions=collisions,
        optimizer_alignment=alignment, alignment_extra_images=padding,
        correlation_groups=len(groups), independent_scene_count_claimed=False,
        order_sha256=hashlib.sha256('\n'.join(rows[i]['id'] for i in order).encode()).hexdigest())


def validate_dataset_contract(dataset, ignore_aware=False):
    meta = json.loads((Path(dataset)/'pretrain.json').read_text())
    if not ignore_aware:
        raise ValueError('requires ignore-aware trainer; a standard data.yaml-only run is unsupported')
    if meta['classes'] != list(CLASSES) or not meta['requires_ignore_aware_trainer']:
        raise ValueError('fixed16/ignore contract mismatch')
    success = Path(dataset)/'_SUCCESS'
    if not meta.get('final_version') or not success.is_file() or success.read_text().strip() != sha(Path(dataset)/'pretrain.json'):
        raise ValueError('requires finalized immutable dataset with _SUCCESS')
    for file, expected in meta['artifact_shas'].items():
        if sha(Path(dataset)/file) != expected:
            raise ValueError('dataset artifact SHA mismatch: '+file)
    import yaml
    names = yaml.safe_load((Path(dataset)/'data.yaml').read_text())['names']
    if names != list(CLASSES):
        raise ValueError('data.yaml class order differs from fixed16 contract')
    if not json.loads((Path(dataset)/'verification.json').read_text())['passed']:
        raise ValueError('dataset verification incomplete')
    return meta


def validate_source_paths(dataset, meta):
    """Fail before importing GPU packages if an external image mount is absent."""
    count=0
    with (Path(dataset)/'manifest.jsonl').open() as f:
        for line in f:
            if not line.strip():continue
            row=json.loads(line)
            resolve_data_path(row['image'],dataset,meta['original_data_root'])
            count+=1
    return count


def pretrain_data_dict(dataset):
    # The verified loader uses original source paths, including /data relocation.
    # Avoid Ultralytics' standard symlink-path scanner and label cache entirely.
    return dict(path=str(Path(dataset).resolve()),train=str(dataset),val=str(dataset),
                test=str(dataset),names=dict(enumerate(CLASSES)),nc=len(CLASSES),channels=3,
                yaml_file=str(Path(dataset)/'data.yaml'))


def guarded_data_yaml(dataset):
    # Stock `yolo train data=data.yaml` must fail its image-path preflight.
    # The dedicated trainer supplies verified paths through get_dataset instead.
    return dict(path=str(Path(dataset).resolve()),
        train='_IGNORE_AWARE_TRAINER_REQUIRED/train',
        val='_IGNORE_AWARE_TRAINER_REQUIRED/val',
        test='_IGNORE_AWARE_TRAINER_REQUIRED/test',names=list(CLASSES),
        requires_ignore_aware_trainer=True)
