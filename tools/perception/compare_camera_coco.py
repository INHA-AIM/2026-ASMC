#!/usr/bin/env python3
"""Same decoded/masked frames, fixed YOLO11m; four labels versus all COCO labels."""
import argparse
from collections import Counter
import hashlib
import html
import json
from pathlib import Path
import time

import cv2
import numpy as np
import rosbag
import yaml

from asmc_camera.detection.classes import load_class_groups
from asmc_camera.detection.hood_mask import HoodMask
from asmc_camera.detection.ultralytics_backend import UltralyticsDetector
from camera_coco_measurements import common_label_comparison, select_gallery, validate_profiles
from evaluate_hood_mask import boxes, matching
from b0_measurements import quantiles


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def draw(image, predictions, title, mask):
    output = image.copy()
    bitmap, polygon = mask.geometry(*image.shape[:2])
    output[bitmap] = (output[bitmap]*.7+np.array([0, 0, 255])*.3).astype(np.uint8)
    cv2.polylines(output, [polygon], True, (0, 0, 255), 2)
    for box in predictions:
        x1, y1, x2, y2 = map(int, box['xyxy'])
        cv2.rectangle(output, (x1, y1), (x2, y2), (40, 220, 40), 2)
        label = '%s [%s] %.2f' % (box['source_class_name'], box['group'], box['confidence'])
        cv2.putText(output, label, (max(0, x1), max(55, y1-5)), cv2.FONT_HERSHEY_SIMPLEX, .5, (40, 220, 40), 1)
    cv2.putText(output, title+' | hood mask ON', (10, 26), cv2.FONT_HERSHEY_SIMPLEX, .7, (255, 255, 255), 2)
    return output


def main():
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--config', type=Path, default=root/'config/experiments/camera_coco_classes_v1.yaml')
    parser.add_argument('--bags', default='camera,primary,shadow')
    parser.add_argument('--skip-gt', action='store_true')
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text())
    args.output.mkdir(parents=True, exist_ok=False)
    sources = {}
    detectors, groups = {}, {}
    mask_path = root/'src/asmc_camera/config/camera/yolo_hood_mask_v1.yaml'
    mask = HoodMask.load(True, mask_path)
    for name, entry in config['object_profiles'].items():
        path = root/entry['config']
        parameters = yaml.safe_load(path.read_text())
        sources[name] = parameters
        groups[name] = load_class_groups(yaml.safe_load((root/entry['groups']).read_text())['class_groups'])
        detectors[name] = UltralyticsDetector(
            str(root/'src/asmc_camera/models/object_detection/yolo11m.pt'),
            device=parameters['device'], require_cuda=parameters['require_cuda'],
            conf_thres=parameters['conf_thres'], allowed_names=parameters['allowed_names'],
            expected_sha256=parameters['expected_sha256'], predict_overrides=parameters['predict_overrides'])
    errors = validate_profiles(sources['baseline'], sources['all'],
        yaml.safe_load((root/config['object_profiles']['all']['groups']).read_text())['class_groups'], detectors['all'].names)
    if errors:
        raise ValueError('; '.join(errors))
    summary = dict(models={name: detector.info for name, detector in detectors.items()}, mask=mask.info,
                   experiment_config_sha256=sha(args.config), bags={}, gt={}, gallery_review='pending_user_review',
                   semantics='class coverage experiment; not camera-only driving acceptance')
    timings = {name: [] for name in detectors}
    gallery = args.output/'gallery'
    gallery.mkdir()
    figures = []

    def infer(image, index):
        masked = mask.apply(image)
        result = {}
        order = ['baseline', 'all'] if index % 2 == 0 else ['all', 'baseline']
        for name in order:
            detections, stages = detectors[name].detect(masked)
            result[name] = [dict(box, group=groups[name].group_of(box['source_class_name'])) for box in boxes(detections)]
            timings[name].append(stages['inference_ms'])
        return result

    bag_paths = dict(camera=config['camera']['bag'], **config['fusion']['bags'])
    for bag_id in args.bags.split(',') if args.bags else []:
        path = Path(bag_paths[bag_id])
        rows, counts, differences = [], Counter(), []
        with rosbag.Bag(str(path)) as bag, (args.output/(bag_id+'.jsonl')).open('w') as stream:
            for index, (_, message, _) in enumerate(bag.read_messages(topics=['/image_jpeg/compressed'])):
                image = cv2.imdecode(np.frombuffer(message.data, np.uint8), cv2.IMREAD_COLOR)
                if image is None:
                    raise RuntimeError('undecodable bag frame %s/%d' % (bag_id, index))
                predictions = infer(image, index)
                comparison = common_label_comparison(predictions['baseline'], predictions['all'])
                row = dict(index=index, stamp_ns=str(message.header.stamp.to_nsec()), frame_id=message.header.frame_id,
                           image_width=image.shape[1], image_height=image.shape[0],
                           image_sha256=hashlib.sha256(message.data).hexdigest(), **predictions)
                stream.write(json.dumps(row)+'\n')
                rows.append(row)
                counts.update(box['source_class_name'] for box in row['all'])
                if not comparison['exact']:
                    differences.append(dict(index=index, stamp_ns=row['stamp_ns'], **comparison))
                if (index+1) % 250 == 0:
                    print(bag_id, index+1, 'common-label differences', len(differences), flush=True)
        selected = select_gallery(rows)
        selected_indices = sorted({i for indices in selected.values() for i in indices})
        # Also show a bounded first set of four-label differences, including shadow failures.
        selected_indices = set(selected_indices+[row['index'] for row in differences[:10]])
        with rosbag.Bag(str(path)) as bag:
            for index, (_, message, _) in enumerate(bag.read_messages(topics=['/image_jpeg/compressed'])):
                if index not in selected_indices:
                    continue
                row = rows[index]
                image = cv2.imdecode(np.frombuffer(message.data, np.uint8), cv2.IMREAD_COLOR)
                panel = cv2.hconcat([draw(image, row[name], name, mask) for name in ('baseline', 'all')])
                filename = '%s-%05d.jpg' % (bag_id, index)
                if not cv2.imwrite(str(gallery/filename), panel):
                    raise RuntimeError('gallery image write failed')
                labels = sorted(label for label, indices in selected.items() if index in indices)
                figures.append(dict(image=filename, bag=bag_id, index=index, stamp_ns=row['stamp_ns'], labels=labels,
                                    review='pending_independent_review_no_new_class_GT'))
        (args.output/(bag_id+'-differences.json')).write_text(json.dumps(differences, indent=2)+'\n')
        summary['bags'][bag_id] = dict(frames=len(rows), label_counts=dict(counts),
            common_label_exact_frames=len(rows)-len(differences), common_label_difference_frames=len(differences),
            gallery_selection=selected, expected_frames=config['camera'].get('expected_front_frames') if bag_id=='camera' else None)
        if bag_id=='camera' and len(rows)!=config['camera']['expected_front_frames']:
            raise RuntimeError('incomplete camera bag comparison')
        (args.output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')

    if not args.skip_gt:
        for name, relative in config['ground_truth'].items():
            path = root/relative
            manifest = json.loads(path.read_text())
            records, losses, counts = [], [], Counter()
            with (args.output/(name+'.jsonl')).open('w') as stream:
                for index, frame in enumerate(manifest['frames']):
                    image_path = Path(frame['image'])
                    image = cv2.imread(str(image_path))
                    if image is None:
                        raise RuntimeError('missing GT image '+str(image_path))
                    predictions = infer(image, index)
                    # Preserve the prior person/car/bus/truck GT scoring convention.
                    # New-label cases are reviewed separately, not counted as GT accuracy.
                    old = set(matching(frame['ground_truth'], predictions['baseline']))
                    new = set(matching(frame['ground_truth'], predictions['all']))
                    lost = sorted(old-new)
                    for gt in frame['ground_truth']:
                        counts['gt_'+gt['label']] += 1
                    counts['baseline_matches'] += len(old)
                    counts['all_matches'] += len(new)
                    counts['new_misses'] += len(lost)
                    row = dict(index=index, **frame, **predictions, new_misses=lost, image_sha256=sha(image_path))
                    stream.write(json.dumps(row)+'\n')
                    records.append(row)
                    if lost:
                        losses.append(row)
                        panel = cv2.hconcat([draw(image, row[p], p, mask) for p in ('baseline', 'all')])
                        filename = '%s-loss-%03d.jpg' % (name, index)
                        cv2.imwrite(str(gallery/filename), panel)
                        figures.append(dict(image=filename, manifest=name, index=index, labels=['GT new miss'],
                                            review='new_loss_not_covered_by_prior_mask_exception'))
            summary['gt'][name] = dict(frames=len(records), manifest_sha256=sha(path), counts=dict(counts),
                                       new_miss_frames=[row['index'] for row in losses],
                                       scope='existing person/vehicle GT; new COCO categories lack independent GT')
            (args.output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    summary['inference_ms'] = {name: quantiles(values) for name, values in timings.items()}
    (gallery/'review.json').write_text(json.dumps(figures, indent=2)+'\n')
    markup = '<!doctype html><meta charset="utf-8"><title>YOLO11m class comparison</title><style>body{font-family:sans-serif;margin:24px}img{width:100%;max-width:1600px}figure{margin:24px 0}pre{white-space:pre-wrap}</style><h1>YOLO11m: baseline four / all 80</h1><p>Both inputs use the same semantic hood mask. Left: baseline. Right: all COCO. Infrastructure detection is not avoidance approval. New class cases require independent review.</p>'
    for row in figures:
        markup += '<figure><figcaption>'+html.escape(json.dumps(row, ensure_ascii=False))+'</figcaption><a href="'+row['image']+'"><img loading="lazy" src="'+row['image']+'"></a></figure>'
    (gallery/'index.html').write_text(markup, encoding='utf-8')
    summary['complete'] = True
    (args.output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    print(json.dumps(summary, indent=2), flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
