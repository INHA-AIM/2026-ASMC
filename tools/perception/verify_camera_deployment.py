#!/usr/bin/env python3
"""Compare a packaged PT and TensorRT engine through the actual camera backend."""
import argparse
from collections import Counter
import gc
import hashlib
import json
from pathlib import Path
import time

import cv2
import numpy as np
import torch
import yaml

from asmc_camera.detection.hood_mask import HoodMask
from asmc_camera.detection.output_nms import iou
from asmc_camera.detection.yolo_batch_backend import BatchedYoloDetector


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for arg in ('checkpoint', 'engine', 'config', 'manifest', 'output'):
        p.add_argument('--' + arg, type=Path, required=True)
    p.add_argument('--samples-per-view', type=int, default=48)
    a = p.parse_args()
    rows = [json.loads(line) for line in a.manifest.read_text().splitlines()]
    views = ('front', 'left', 'right')
    # Deterministic route-wide selection, then add examples for every class.
    chosen = {}
    for view in views:
        candidates = [r for r in rows if r.get('view') == view]
        for index in np.linspace(0, len(candidates)-1, min(a.samples_per_view, len(candidates)), dtype=int):
            r = candidates[index]; chosen[r['id']] = r
    cfg = yaml.safe_load(a.config.read_text())
    for label in cfg['allowed_names']:
        found = 0
        for r in rows:
            if r.get('view') in views and any(x.get('training_class', x.get('canonical')) == label and
                                            x.get('supervision') == 'positive' for x in r['annotations']):
                chosen[r['id']] = r; found += 1
                if found == 3: break
    rows = list(chosen.values())
    package = Path(__file__).resolve().parents[2] / 'src/asmc_camera'
    masks = {v: HoodMask.load(True, package/'config/camera'/name) for v, name in (
        ('front', 'yolo_hood_mask_v1.yaml'), ('left', 'yolo_ego_mask_left_v1.yaml'),
        ('right', 'yolo_ego_mask_right_v1.yaml'))}
    images = {}
    for r in rows:
        assert sha(r['image']) == r['image_sha256']
        images[r['id']] = masks[r['view']].apply(cv2.imread(r['image']))
    results = {}; runtime = {}; batch_checks = []
    for name, backend, path, direct in [('pt', 'pytorch', a.checkpoint, False),
                                      ('engine', 'tensorrt', a.engine, True)]:
        kwargs = dict(cfg, weights=str(path), expected_sha256=sha(path),
                      runtime_backend=backend, direct_detect=direct)
        model = BatchedYoloDetector(**kwargs)
        runtime[name] = model.info
        values = {}
        for r in rows:
            ds = model.infer_images({r['view']: images[r['id']]}, [r['view']])[r['view']].detections
            values[r['id']] = [dict(class_name=d.source_class_name, confidence=d.confidence,
                                    xyxy=[d.xmin, d.ymin, d.xmax, d.ymax]) for d in ds]
        results[name] = values
        if direct:
            im = {v: images[next(r['id'] for r in rows if r['view'] == v)] for v in views}
            for order in (['front'], ['left', 'right'], ['right', 'front'], list(views)):
                value = model.infer_images({v: im[v] for v in order}, order)
                assert set(value) == set(order)
                assert all(d.source_class_name in cfg['allowed_names'] for f in value.values() for d in f.detections)
                batch_checks.append(dict(requested=order, returned=list(value)))
            durations = []
            for _ in range(30):
                torch.cuda.synchronize(); start = time.monotonic()
                model.infer_images(im, list(views)); torch.cuda.synchronize()
                durations.append((time.monotonic()-start)*1000)
            runtime[name]['three_view_offline_ms'] = dict(p50=float(np.percentile(durations,50)),
                                                         p95=float(np.percentile(durations,95)))
        del model; gc.collect(); torch.cuda.empty_cache()
    pairs = []; failures = []; boundary = []
    for row in rows:
        left, right = (results[name][row['id']] for name in ('pt', 'engine'))
        choices = sorted([(iou(x['xyxy'],y['xyxy']),i,j) for i,x in enumerate(left)
                          for j,y in enumerate(right) if x['class_name'] == y['class_name']], reverse=True)
        used_a = set(); used_b = set()
        for overlap,i,j in choices:
            if overlap < .5 or i in used_a or j in used_b: continue
            used_a.add(i); used_b.add(j)
            diff = abs(left[i]['confidence']-right[j]['confidence'])
            item = dict(id=row['id'], class_name=left[i]['class_name'], iou=overlap, confidence_difference=diff)
            pairs.append(item)
            if overlap < .98 or diff > .02: failures.append(item)
        for source,boxes,used in [('pt',left,used_a),('engine',right,used_b)]:
            for index,box in enumerate(boxes):
                if index not in used:
                    item = dict(id=row['id'], source=source, box=box)
                    (boundary if abs(box['confidence']-cfg['conf_thres']) <= .02 else failures).append(item)
    report = dict(passed=not failures, images=len(rows), paired_boxes=len(pairs),
                  views=dict(Counter(r['view'] for r in rows)), pairs=pairs, failures=failures,
                  threshold_boundary=boundary, checkpoint_sha256=sha(a.checkpoint),
                  engine_sha256=sha(a.engine), config_sha256=sha(a.config),
                  manifest_sha256=sha(a.manifest), runtime=runtime, batch_checks=batch_checks,
                  predictions=results, image_ids=[r['id'] for r in rows],
                  timing_scope='offline backend 3-view cycle only; not ROS sensor-to-fusion latency')
    a.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(dict(passed=report['passed'], images=len(rows), pairs=len(pairs),
                         failures=len(failures), threshold_boundary=len(boundary))))
    if failures: raise SystemExit(1)


if __name__ == '__main__':
    main()
