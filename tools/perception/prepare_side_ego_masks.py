#!/usr/bin/env python3
"""Calibrate side silhouettes from manually identified Ego GT, without GPU inference."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
import yaml

from asmc_camera.detection.hood_mask import HoodMask


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def corner_component(semantic, instance, view):
    if view not in ('left', 'right') or semantic.shape != instance.shape:
        raise ValueError('side GT view/shape mismatch')
    # These black pixels were identified as our own body/mirror by paired RGB
    # inspection. Black alone is NOT a global MORAI Ego semantic class.
    black = np.all(semantic == 0, axis=2) & np.all(instance == 0, axis=2)
    _, labels = cv2.connectedComponents(black.astype(np.uint8), connectivity=8)
    seed = int(labels[-1, -1 if view == 'left' else 0])
    if seed == 0:
        raise ValueError('expected body corner is not black in both GT channels')
    bitmap = labels == seed
    if not .05 < bitmap.mean() < .30:
        raise ValueError('ambiguous Ego component; inspect source RGB instead of expanding the mask')
    return bitmap


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--gt', type=Path, required=True)
    p.add_argument('--datasets', type=Path, required=True)
    p.add_argument('--configs', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=False)
    a.configs.mkdir(parents=True, exist_ok=True)
    rows = [json.loads(line) for line in (a.gt/'frames.jsonl').read_text().splitlines()]
    report = dict(calibration_split='train only', geometric_validation_splits=['train', 'selection'],
                  inference_performed=False, source_manifest_sha256=sha(a.gt/'frames.jsonl'),
                  method='Black semantic AND instance corner component, RGB-confirmed; 1px dilation and 0.75px contour simplification',
                  views={})
    articles = []
    for view in ('left', 'right'):
        samples = []
        for run in sorted({r['run'] for r in rows if r['split'] == 'train'}):
            subset = [r for r in rows if r['run'] == run and r['view'] == view]
            if not subset:
                raise ValueError('missing train view: '+run+' '+view)
            samples.extend(subset[i] for i in sorted({0, len(subset)//2, len(subset)-1}))
        silhouettes, sources = [], []
        for row in samples:
            image = a.datasets/row['image']
            semantic = Path(str(image).replace('/intensity/', '/semantic/'))
            instance = Path(str(image).replace('/intensity/', '/instance/'))
            sem, inst = cv2.imread(str(semantic)), cv2.imread(str(instance))
            if sem is None or inst is None:
                raise ValueError('missing GT channels: '+str(image))
            silhouettes.append(corner_component(sem, inst, view))
            sources.append(dict(image=row['image'], source_sha256=dict(
                intensity=sha(image), semantic=sha(semantic), instance=sha(instance))))
        if len({m.shape for m in silhouettes}) != 1:
            raise ValueError('camera shapes changed between calibration runs')
        union = np.logical_or.reduce(silhouettes)
        dilated = cv2.dilate(union.astype(np.uint8), np.ones((3,3), np.uint8))
        contours, _ = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        polygon = cv2.approxPolyDP(max(contours, key=cv2.contourArea), .75, True).reshape(-1,2)
        h, w = union.shape
        config = dict(version=1, polygon=[[(float(x)+.1)/w, (float(y)+.1)/h] for x,y in polygon])
        path = a.configs/('yolo_ego_mask_'+view+'_v1.yaml')
        path.write_text(yaml.safe_dump(config, sort_keys=False))
        mask = HoodMask.load(True, path)
        bitmap, pixels = mask.geometry(h, w)
        missed = int(np.count_nonzero(union & ~bitmap))
        if missed:
            raise ValueError('polygon fails to cover calibration silhouette')
        overlap = Counter(); frames = annotations = 0; cases = []
        for row in rows:
            if row['view'] != view or row['split'] not in ('train', 'selection') or not row.get('masks'):
                continue
            frames += 1
            with np.load(a.gt/row['masks']) as data:
                gt_masks = np.unpackbits(data['masks'], axis=2)[:,:,:row['width']].astype(bool)
            projected, _ = mask.geometry(row['height'], row['width'])
            for ann, gt in zip(row['annotations'], gt_masks):
                annotations += 1
                count = int(np.count_nonzero(gt & projected))
                if count:
                    overlap[ann['group']] += count
                    cases.append(dict(run=row['run'], frame=row['frame'], actor=ann['actor_id'], pixels=count))
        record = dict(config_sha256=sha(path), calibration_samples=len(samples), image_width=w, image_height=h,
                      excluded_pixels=int(bitmap.sum()), excluded_fraction=float(bitmap.mean()),
                      calibration_silhouette_pixels=int(union.sum()), missed_silhouette_pixels=missed,
                      pixels_outside_sample_union=int(np.count_nonzero(bitmap & ~union)),
                      GT_frames_with_masks_checked=frames, GT_annotations_checked=annotations,
                      visible_actor_pixel_overlap_by_group=dict(overlap), overlap_cases=cases, sources=sources)
        report['views'][view] = record
        for index in [0, len(samples)//2, len(samples)-1]:
            row = samples[index]; image = cv2.imread(str(a.datasets/row['image']))
            render = image.copy()
            render[bitmap] = (render[bitmap]*.65 + np.array([0,0,255])*.35).astype(np.uint8)
            cv2.polylines(render, [pixels], True, (0,0,255), 1)
            name = view+'-'+str(index)
            cv2.imwrite(str(a.output/(name+'-overlay.jpg')), render)
            cv2.imwrite(str(a.output/(name+'-input.jpg')), mask.apply(image))
            articles.append('<h2>'+view+' '+row['frame']+'</h2><p>Overlay / actual masked input (no new inference)</p>'
                            '<img width="640" src="'+name+'-overlay.jpg"><img width="640" src="'+name+'-input.jpg">')
    (a.output/'summary.json').write_text(json.dumps(report, indent=2)+'\n')
    (a.output/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>Side Ego masks</title>'
                                    '<h1>Train-calibrated body and mirror masks</h1><p>GPU detection/recall still pending. '
                                    'Black is a reviewed component in these captures, not a general Ego class.</p>'+''.join(articles))
    print(json.dumps({v:{k:x for k,x in r.items() if k != 'sources'} for v,r in report['views'].items()}))


if __name__ == '__main__':
    main()
