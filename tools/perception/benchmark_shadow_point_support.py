#!/usr/bin/env python3
"""Isolated saved-bag point-support comparison; never publishes or deletes points.

This measures raw-point support, not wall-removal accuracy, MOT accuracy, or
closed-loop throughput. Run prepare once, then one model per process.
"""
import argparse
from bisect import bisect_left
from collections import Counter
import hashlib
import html
import json
from pathlib import Path
import platform
import sys
import time

import cv2
import numpy as np


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, ensure_ascii=False) + '\n')


def distribution(values):
    return {f'p{q}': float(np.percentile(values, q)) for q in (50, 95, 99)} if values else {}


def nearest_index(stamps, stamp, maximum_ns=50_000_000):
    i = bisect_left(stamps, stamp)
    candidates = [j for j in (i - 1, i) if 0 <= j < len(stamps)]
    if not candidates:
        return None
    result = min(candidates, key=lambda j: (abs(stamps[j] - stamp), stamps[j]))
    return result if abs(stamps[result] - stamp) <= maximum_ns else None


def project(points, lidar, camera):
    from asmc_camera.bev.calibration import _rotation_x, _rotation_y, _rotation_z
    a = np.radians([float(lidar['rot'][k]) for k in ('roll', 'pitch', 'yaw')])
    rotation = _rotation_z(a[2]) @ _rotation_y(a[1]) @ _rotation_x(a[0])
    translation = np.array([float(lidar['pos'][k]) for k in ('x', 'y', 'z')])
    optical = camera.ground_to_optical(points @ rotation.T + translation)
    homogeneous = optical @ camera.intrinsic.T
    pixels = np.full((len(points), 2), np.nan)
    positive = optical[:, 2] > 0
    pixels[positive] = homogeneous[positive, :2] / homogeneous[positive, 2, None]
    valid = (positive & np.isfinite(pixels).all(axis=1) &
             (pixels[:, 0] >= 0) & (pixels[:, 0] < camera.width) &
             (pixels[:, 1] >= 0) & (pixels[:, 1] < camera.height))
    return pixels, valid, optical[:, 2]


def box_indices(pixels, visible, box):
    x1, y1, x2, y2 = box
    return np.flatnonzero(visible & (pixels[:, 0] >= x1) & (pixels[:, 0] < x2) &
                          (pixels[:, 1] >= y1) & (pixels[:, 1] < y2))


def split_support(points, pixels, depth, indices, box, radius=.55, depth_gap=.75):
    """Experimental geometry adapter, not a reproduction of a learned paper.

    Return every connected component plus one tentative foreground support.
    All unselected points remain unknown; this function is not a deletion mask.
    """
    from scipy.spatial import cKDTree
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components
    if not len(indices):
        return [], np.empty(0, dtype=np.int64)
    pairs = cKDTree(points[indices]).query_pairs(radius, output_type='ndarray')
    if len(pairs):
        pairs = pairs[np.abs(depth[indices[pairs[:, 0]]] - depth[indices[pairs[:, 1]]]) <= depth_gap]
    rows = np.r_[pairs[:, 0], pairs[:, 1]]
    cols = np.r_[pairs[:, 1], pairs[:, 0]]
    graph = coo_matrix((np.ones(len(rows)), (rows, cols)), shape=(len(indices), len(indices)))
    count, labels = connected_components(graph, directed=False)
    components = [indices[labels == k] for k in range(count)]
    x1, y1, x2, y2 = box
    central = ((pixels[:, 0] >= x1 + .2 * (x2 - x1)) & (pixels[:, 0] < x2 - .2 * (x2 - x1)) &
               (pixels[:, 1] >= y1 + .2 * (y2 - y1)) & (pixels[:, 1] < y2 - .2 * (y2 - y1)))
    eligible = [c for c in components if len(c) >= 3 and np.any(central[c])]
    selected = min(eligible, key=lambda c: float(np.median(depth[c]))) if eligible else np.empty(0, dtype=np.int64)
    return components, selected


def support_stats(indices, points, depth):
    result = dict(points=int(len(indices)))
    if len(indices):
        result.update(depth_p05_m=float(np.percentile(depth[indices], 5)),
                      depth_p95_m=float(np.percentile(depth[indices], 95)),
                      observed_mean_xyz=points[indices].mean(axis=0).tolist(),
                      observed_min_xyz=points[indices].min(axis=0).tolist(),
                      observed_max_xyz=points[indices].max(axis=0).tolist())
    return result


def prepare(a):
    import rosbag
    from sensor_msgs import point_cloud2
    from asmc_camera.bev.calibration import load_calibration_snapshot, validate_sensor_set_snapshot
    root, out = a.root, a.output
    out.mkdir(parents=True, exist_ok=False)
    (out / 'rgb').mkdir()
    (out / 'clouds').mkdir()
    calibration = root / 'src/asmc_camera/config/cameras_v3.yaml'
    sensors = root / 'config/morai/sensors/ASMC_sensor_set_v3.json'
    document, cameras = load_calibration_snapshot(calibration)
    validate_sensor_set_snapshot(document, cameras, sensors)
    images = {v: [] for v in cameras}
    scans, signals = [], []
    topics = {c.image_topic: v for v, c in cameras.items()}
    with rosbag.Bag(str(a.bag)) as bag:
        metadata = bag.get_type_and_topic_info().topics
        for topic, msg, recorded in bag.read_messages():
            if topic in topics:
                images[topics[topic]].append((msg.header.stamp.to_nsec(), recorded.to_nsec(), bytes(msg.data)))
            elif topic == '/velodyne_points':
                if msg.header.frame_id != 'velodyne':
                    raise ValueError('expected raw velodyne frame')
                # Preserve raw ordinal -> finite-point lineage explicitly.
                raw = np.asarray(list(point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=False)), dtype=np.float32)
                finite = np.isfinite(raw).all(axis=1)
                stamp = msg.header.stamp.to_nsec()
                np.savez_compressed(out / 'clouds' / f'{stamp}.npz', xyz=raw[finite], raw_index=np.flatnonzero(finite))
                scans.append(dict(stamp_ns=stamp, bag_ns=recorded.to_nsec(), frame=msg.header.frame_id,
                                  raw_points=len(raw), finite_points=int(finite.sum())))
            elif topic == '/Ego_topic':
                signals.append(dict(topic=topic, stamp_ns=msg.header.stamp.to_nsec(),
                                    velocity_xyz=[getattr(msg.velocity, k) for k in 'xyz'],
                                    heading_deg=msg.heading, development_only=True))
    rows, matches = [], Counter()
    for view in images:
        images[view].sort(key=lambda x: x[0])
    stamps = {v: [x[0] for x in values] for v, values in images.items()}
    for i, scan in enumerate(scans):
        for view, camera in cameras.items():
            j = nearest_index(stamps[view], scan['stamp_ns'])
            row = dict(scan_index=i, **scan, view=view)
            if j is None:
                rows.append(dict(row, matched=False)); continue
            stamp, recorded, blob = images[view][j]
            image = cv2.imdecode(np.frombuffer(blob, np.uint8), cv2.IMREAD_COLOR)
            if image is None or image.shape[:2] != (camera.height, camera.width):
                raise ValueError('image does not match pinned camera calibration')
            name = f'rgb/{view}-{stamp}.jpg'
            (out / name).write_bytes(blob)
            row.update(matched=True, image=name, camera_stamp_ns=stamp, camera_bag_ns=recorded,
                       offset_ms=(stamp - scan['stamp_ns']) / 1e6, image_sha256=hashlib.sha256(blob).hexdigest())
            rows.append(row); matches[view] += 1
    (out / 'frames.jsonl').write_text(''.join(json.dumps(r) + '\n' for r in rows))
    (out / 'signals.jsonl').write_text(''.join(json.dumps(r) + '\n' for r in signals))
    write_json(out / 'manifest.json', dict(bag=str(a.bag), bag_sha256=sha(a.bag), scans=len(scans),
        topics={t: dict(type=m.msg_type, messages=m.message_count) for t, m in metadata.items()},
        matches_by_view=dict(matches), maximum_timestamp_offset_ms=50,
        calibration_sha256=sha(calibration), sensor_set_sha256=sha(sensors),
        source_sha256=sha(__file__), cameras=list(cameras),
        extra_camera_excluded='camera4 points upward/front in v3; not a calibrated rear camera',
        coordinate='raw velodyne -> lidar mount -> base_link -> optical; no leveling or ground removal',
        scope='offline nearest header stamp, may use later image; not causal arrival validation',
        gt_available=False, operational_defaults_changed=False))
    print(json.dumps(dict(scans=len(scans), matches=dict(matches))), flush=True)


def make_detector(a):
    import torch
    from asmc_camera.detection.segmentation import SegmentationDetector
    from asmc_camera.detection.yolo_batch_backend import BatchedYoloDetector
    from asmc_camera.detection.mission import CLASSES
    if a.model == 'e8':
        path = a.root / 'src/asmc_camera/models/object_detection/yolo26l-finetuned_e8.engine'
        detector = BatchedYoloDetector(str(path), sha(path), allowed_names=CLASSES, conf_thres=.4,
            runtime_backend='tensorrt', front_width=960, head_profile='mission16_one2one_nms',
            nms_iou=.5, direct_detect=True)
        return detector, path
    if a.model in ('yolo26s-seg', 'yoloe26s-seg'):
        path = a.weights / ('yolo26s-seg.pt' if a.model == 'yolo26s-seg' else 'yoloe-26s-seg.pt')
        prompt = a.weights / 'prompts-v1.npz'
        if a.model == 'yoloe26s-seg':
            with np.load(prompt, allow_pickle=False) as data:
                allowed = data['names'].tolist()
        else:
            from ultralytics import YOLO
            vocabulary = YOLO(str(path)).names
            allowed = list(vocabulary.values())
        detector = SegmentationDetector(weights=str(path), expected_sha256=sha(path), allowed_names=allowed,
            conf_thres=.4, predict_overrides=dict(imgsz=640, half=False, end2end=True, max_det=300),
            model_kind='yoloe' if a.model == 'yoloe26s-seg' else 'yolo',
            prompt_embeddings=str(prompt) if a.model == 'yoloe26s-seg' else '')
        return detector, path
    # Official PIDNet model definition, with every inference parameter checked.
    sys.path.insert(0, str(a.pidnet_code))
    from models.pidnet import get_pred_model
    detector = get_pred_model('pidnet-s', 19).eval()
    path = a.pidnet_weights
    state = torch.load(path, map_location='cpu')
    state = state.get('state_dict', state)
    expected = detector.state_dict()
    loaded = {}
    for key, value in state.items():
        candidates = (key, key[6:], key[7:])
        matching = [k for k in candidates if k in expected and expected[k].shape == value.shape]
        if matching:
            loaded[matching[0]] = value
    missing = sorted(set(expected) - set(loaded))
    if missing:
        raise ValueError('incomplete PIDNet checkpoint: ' + str(missing[:10]))
    detector.load_state_dict(loaded, strict=True)
    return detector.cuda(), path


def infer(a):
    import torch
    from asmc_camera.bev.calibration import load_calibration_snapshot
    from asmc_camera.detection.hood_mask import HoodMask
    out = a.output / a.model
    out.mkdir(parents=True, exist_ok=False)
    (out / 'gallery').mkdir(); (out / 'indices').mkdir()
    cv2.setNumThreads(1); torch.set_num_threads(2)
    torch.manual_seed(0); np.random.seed(0)
    _, cameras = load_calibration_snapshot(a.root / 'src/asmc_camera/config/cameras_v3.yaml')
    lidar = json.loads((a.root / 'config/morai/sensors/ASMC_sensor_set_v3.json').read_text())['Lidar3DList'][0]
    masks = {v: HoodMask.load(True, a.root / 'src/asmc_camera/config/camera' / name) for v, name in
             dict(front='yolo_hood_mask_v1.yaml', left='yolo_ego_mask_left_v1.yaml', right='yolo_ego_mask_right_v1.yaml').items()}
    rows = [json.loads(l) for l in (a.inputs / 'frames.jsonl').read_text().splitlines()]
    rows = [r for r in rows if r['matched']]
    if a.limit:
        rows = rows[:a.limit]
    torch.cuda.reset_peak_memory_stats()
    before_free, total = torch.cuda.mem_get_info()
    detector, path = make_detector(a)
    timings, labels, totals, images = [], Counter(), Counter(), []
    pid_warm = set()
    palette = np.asarray([(128,64,128),(244,35,232),(70,70,70),(102,102,156),(190,153,153),
        (153,153,153),(250,170,30),(220,220,0),(107,142,35),(152,251,152),(70,130,180),
        (220,20,60),(255,0,0),(0,0,142),(0,0,70),(0,60,100),(0,80,100),(0,0,230),(119,11,32)], np.uint8)
    with (out / 'observations.jsonl').open('w') as stream:
        for n, row in enumerate(rows):
            stamp, view = row['stamp_ns'], row['view']
            image = cv2.imread(str(a.inputs / row['image']))
            with np.load(a.inputs / 'clouds' / f'{stamp}.npz') as data:
                points, raw_index = data['xyz'].copy(), data['raw_index'].copy()
            start = time.perf_counter()
            pixels, visible, depth = project(points, lidar, cameras[view])
            projection_ms = (time.perf_counter() - start) * 1000
            model_image = masks[view].apply(image)
            if a.model == 'pidnet-s':
                # Preserve aspect, use multiples of 32; front 544x960, sides 480x640.
                h, w = ((544, 960) if view == 'front' else (480, 640))
                array = cv2.resize(model_image, (w, h))[:, :, ::-1].copy().astype(np.float32) / 255
                array = (array - [0.485, .456, .406]) / [.229, .224, .225]
                tensor = torch.from_numpy(array.astype(np.float32).transpose(2, 0, 1)[None]).cuda()
                if view not in pid_warm:
                    with torch.inference_mode():
                        for _ in range(3): detector(tensor)
                    pid_warm.add(view)
                torch.cuda.synchronize(); start = time.perf_counter()
                with torch.inference_mode():
                    logits = detector(tensor)
                    logits = torch.nn.functional.interpolate(logits, image.shape[:2], mode='bilinear', align_corners=False)
                    semantic = logits.argmax(1)[0].byte().cpu().numpy()
                torch.cuda.synchronize(); infer_ms = (time.perf_counter() - start) * 1000
                idx = np.flatnonzero(visible)
                point_class = semantic[pixels[idx, 1].astype(int), pixels[idx, 0].astype(int)]
                counts = Counter(map(int, point_class))
                obs = dict(row, inference_ms=infer_ms, projection_ms=projection_ms,
                           projected_semantic_points=dict(counts), class_pixels=dict(Counter(map(int, semantic.ravel()))))
                for key, value in counts.items(): labels[str(key)] += value
                render = (image.astype(float) * .55 + palette[semantic][:, :, ::-1] * .45).astype(np.uint8)
                if row['scan_index'] % 16 == 0:
                    cv2.imwrite(str(out / 'indices' / f'{stamp}-{view}-semantic.png'), semantic)
                    np.savez_compressed(out / 'indices' / f'{stamp}-{view}.npz', raw_index=raw_index[idx], semantic=point_class)
            else:
                torch.cuda.synchronize(); start = time.perf_counter()
                frame = (detector.infer_images({view: model_image}, [view])[view] if a.model == 'e8'
                         else detector.predict_frame(model_image))
                torch.cuda.synchronize(); infer_ms = (time.perf_counter() - start) * 1000
                render, boxes, lineage = image.copy(), [], {}
                start = time.perf_counter()
                for k, (d, mask) in enumerate(zip(frame.detections, frame.masks)):
                    box = [d.xmin, d.ymin, d.xmax, d.ymax]
                    inside = box_indices(pixels, visible, box)
                    components, selected = split_support(points, pixels, depth, inside, box)
                    masked = (inside[mask[pixels[inside, 1].astype(int), pixels[inside, 0].astype(int)]]
                              if mask is not None else np.empty(0, dtype=np.int64))
                    entry = dict(label=d.source_class_name, confidence=d.confidence, xyxy=box,
                                 box=support_stats(inside, points, depth),
                                 geometry=support_stats(selected, points, depth),
                                 mask=support_stats(masked, points, depth) if mask is not None else None,
                                 component_sizes=sorted((len(c) for c in components), reverse=True))
                    boxes.append(entry); labels[d.source_class_name] += 1
                    totals['box_points'] += len(inside); totals['geometry_points'] += len(selected)
                    totals['mask_points'] += len(masked); totals['detections'] += 1
                    totals['detections_at_07'] += int(d.confidence >= .7)
                    totals['with_box_support'] += int(len(inside) > 0)
                    totals['with_geometry_support'] += int(len(selected) > 0)
                    if row['scan_index'] % 16 == 0:
                        lineage[f'{k}_box_raw_index'] = raw_index[inside]
                        lineage[f'{k}_geometry_raw_index'] = raw_index[selected]
                        lineage[f'{k}_mask_raw_index'] = raw_index[masked]
                        if mask is not None:
                            render[mask] = (render[mask] * .7 + np.array([40, 210, 40]) * .3).astype(np.uint8)
                        for ids, color in ((inside, (0,0,255)), (selected, (0,255,255))):
                            for p in pixels[ids].astype(int): cv2.circle(render, tuple(p), 1, color, -1)
                        x1,y1,x2,y2 = map(int, box)
                        cv2.rectangle(render, (x1,y1), (x2,y2), (255,180,0), 1)
                        cv2.putText(render, f'{d.source_class_name} {d.confidence:.2f}', (max(0,x1),max(15,y1)),
                                    cv2.FONT_HERSHEY_SIMPLEX, .45, (0,255,255), 1)
                geometry_ms = (time.perf_counter() - start) * 1000
                # Gallery work is excluded below by timing the algorithm again without rendering.
                start = time.perf_counter()
                for d in frame.detections:
                    box = [d.xmin, d.ymin, d.xmax, d.ymax]
                    split_support(points, pixels, depth, box_indices(pixels, visible, box), box)
                geometry_only_ms = (time.perf_counter() - start) * 1000
                obs = dict(row, boxes=boxes, inference_ms=infer_ms, projection_ms=projection_ms,
                           geometry_only_ms=geometry_only_ms, support_and_render_ms=geometry_ms)
                if lineage: np.savez_compressed(out / 'indices' / f'{stamp}-{view}.npz', **lineage)
            timings.append({k:obs[k] for k in ('inference_ms','projection_ms','geometry_only_ms') if k in obs})
            stream.write(json.dumps(obs, ensure_ascii=False) + '\n')
            if row['scan_index'] % 16 == 0:
                name = f'{row["scan_index"]:03d}-{view}.jpg'
                cv2.imwrite(str(out / 'gallery' / name), render)
                images.append(dict(file=name, scan_index=row['scan_index'], view=view))
            if (n + 1) % 100 == 0: print(a.model, n + 1, '/', len(rows), flush=True)
    free, _ = torch.cuda.mem_get_info()
    summary = dict(model=a.model, frames=len(rows), views=dict(Counter(r['view'] for r in rows)),
        weights=str(path), weights_sha256=sha(path), labels=dict(labels), totals=dict(totals),
        timings_ms={k:distribution([t[k] for t in timings if k in t]) for k in ('inference_ms','projection_ms','geometry_only_ms')},
        timings_by_view_ms={v:distribution([t['inference_ms'] for r,t in zip(rows,timings) if r['view']==v]) for v in cameras},
        torch_peak_allocated_mib=torch.cuda.max_memory_allocated()/2**20,
        torch_peak_reserved_mib=torch.cuda.max_memory_reserved()/2**20,
        gpu_total_mib=total/2**20, gpu_free_before_load_mib=before_free/2**20, gpu_free_after_mib=free/2**20,
        model_info=getattr(detector, 'info', {}), gallery=images, script_sha256=sha(__file__),
        input_manifest_sha256=sha(a.inputs/'manifest.json'), torch=torch.__version__, python=platform.python_version(),
        geometry=dict(radius_m=.55, depth_gap_m=.75, min_component_points=3, selection='nearest central supported component'),
        gt_accuracy_evaluable=False, object_deletion_enabled=False, operational_defaults_changed=False,
        scope='model-isolated offline diagnostic; raw points, no production ground/cluster/tracker or concurrent lane/control; not realtime acceptance')
    if a.model == 'pidnet-s':
        summary.update(pidnet_revision=a.pidnet_revision, complete_checkpoint=True, ceiling_class_available=False,
                       latency_excludes='CPU resize/normalization and input tensor upload; includes forward/logit resize/argmax/CPU transfer')
    elif a.model == 'yoloe26s-seg':
        summary['prompt_sha256'] = sha(a.weights/'prompts-v1.npz')
    write_json(out/'summary.json', summary)
    (out/'gallery/index.html').write_text('<!doctype html><meta charset="utf-8"><title>Point support</title>'
        '<p>Offline only. Red: box support; yellow: tentative geometry support; green: instance mask. No GT accuracy claim.</p>' + ''.join(
        '<figure><figcaption>'+html.escape(f'{a.model} scan {r["scan_index"]} {r["view"]}')+'</figcaption><img loading="lazy" width="960" src="'+r['file']+'"></figure>' for r in images))
    print(json.dumps({k:v for k,v in summary.items() if k not in ('model_info','gallery')}, ensure_ascii=False), flush=True)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('phase', choices=('prepare','infer'))
    p.add_argument('--root',type=Path,required=True)
    p.add_argument('--bag',type=Path)
    p.add_argument('--inputs',type=Path)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--model',choices=('e8','yolo26s-seg','yoloe26s-seg','pidnet-s'))
    p.add_argument('--weights',type=Path,default=Path('/data/models/camera_seg_v1'))
    p.add_argument('--pidnet-code',type=Path)
    p.add_argument('--pidnet-weights',type=Path)
    p.add_argument('--pidnet-revision',default='')
    p.add_argument('--limit',type=int)
    a=p.parse_args()
    if a.phase=='prepare': prepare(a)
    else: infer(a)


if __name__=='__main__': main()
