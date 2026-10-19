#!/usr/bin/env python3
"""Same-source-clock M0 / alternating M1 / joint M1 family comparison.

No model inference or control. Pixel and temporal metrics remain internal
consistency measures, not independent geometric ground truth.
"""
import argparse
from collections import defaultdict
from copy import deepcopy
import html
import json
from pathlib import Path
import time

import cv2
import numpy as np
import yaml

from asmc_camera.bev.calibration import calibrations_from_dict
from asmc_camera.bev.imu_projection import attitude_camera, correct_evidence
from asmc_camera.bev.model_grid_projection import ModelGridGroundProjector, MetricEvidence
from review_source_tunnel_motion import source_signals
from review_tunnel_imu_motion import read, fit, family_distances, temporal, render, VIEWS
from review_family_motion import interpolate, sha, stats
from evaluate_imu_width_candidate import support_intervals, lost_support

NAMES = ('M0', 'M1_source_width0', 'M1_source_joint')


def main(args):
    args.output.mkdir(parents=True, exist_ok=False)
    cv2.setNumThreads(1)
    cfg = yaml.safe_load((args.config / 'bev/lane_fitting.yaml').read_text())
    cams = calibrations_from_dict(yaml.safe_load((args.config / 'cameras_v3.yaml').read_text()))
    p = {v: ModelGridGroundProjector(cams[v], model_hw=(544, 960) if v == 'front' else (480, 640),
        source_mask=1 << i, max_ground_range_m=cfg['max_ground_range_m'],
        x_bounds_m=(cfg['evidence_x_min_m'], cfg['evidence_x_max_m']),
        y_bounds_m=(cfg['evidence_y_min_m'], cfg['evidence_y_max_m'])) for i, v in enumerate(VIEWS)}
    capture = args.trial / 'capture'
    imu, ego, signals = source_signals(capture, read(args.trial / 'poses.jsonl'))
    frames = read(capture / 'frames.jsonl')
    records, pictures, stages = [], [], defaultdict(list)
    saved = set()
    for index, frame in enumerate(frames):
        source = {v: frame['views'][v]['lineage']['sensor_timing'] for v in VIEWS}
        if any(not frame['views'][v]['lineage']['source_timing_valid'] or
               source[v]['bridge_session_id'] != signals['bridge_session_id'] for v in VIEWS):
            raise ValueError('invalid source lineage')
        times = {v: source[v]['source_sec'] * 10**9 + source[v]['source_nsec'] for v in VIEWS}
        if max(times.values()) - min(times.values()) > 35e6: raise ValueError('source skew')
        corrected = {}
        for v in VIEWS:
            pose = interpolate(imu, times[v], 'imu')
            if pose is None: raise ValueError('source bracket unavailable')
            corrected[v] = attitude_camera(cams[v], pose['q_xyzw'])
        phase = min(ego, key=lambda r: abs(r['source_ns'] - times['front']))['phase']
        arrays = dict(np.load(capture / frame['grid']))
        maps = {v: {k: cv2.distanceTransform((arrays[v + '_class'] != k).astype(np.uint8), cv2.DIST_L2, 5)
                    for k in (1, 2)} for v in VIEWS}
        base = {v: p[v].extract(arrays[v + '_class'], arrays[v + '_confidence'], {1: .5, 2: .5}) for v in VIEWS}
        corrected_evidence = MetricEvidence.concatenate([correct_evidence(p[v], base[v], corrected[v]) for v in VIEWS])
        variants = {}
        for name in NAMES:
            config = deepcopy(cfg)
            if name != 'M0': config['shared_lane_family'].update(width_prior_strength=0., joint_width_fit_enabled=name.endswith('joint'))
            start = time.perf_counter()
            family, _, _ = fit(MetricEvidence.concatenate(list(base.values())) if name == 'M0' else corrected_evidence, config)
            stages[name].append(1000 * (time.perf_counter() - start))
            family['semantic_model_px'] = family_distances(family, cams if name == 'M0' else corrected, p, arrays, maps)
            variants[name] = family
        row = dict(run_id='live-motion-r1', phase=phase, index=index, stamp_ns=times['front'], variants=variants,
                   grid_sha256=sha(capture / frame['grid']))
        records.append(row)
        if (phase not in saved and index > 10) or index == 18:
            image_index = {(row['run_id'], row['stamp_ns'], v): dict(path=frame['views'][v]['rgb'], sha256=frame['views'][v]['rgb_sha256']) for v in VIEWS}
            pictures.append(render(args.trial, row, variants, cams, corrected, arrays, None, image_index, args.output,
                                   names=NAMES, rgb_root=capture))
            saved.add(phase)
        if (index + 1) % 40 == 0: print('joint family source review', index + 1, flush=True)
    pairs = temporal(records, {'live-motion-r1': {'/Ego_topic': ego}}, variants=NAMES)
    pixels = defaultdict(lambda: defaultdict(list)); losses = []; invalid = []
    for row in records:
        for name, f in row['variants'].items():
            for key, value in f['semantic_model_px'].items():
                if key.startswith('observed/') and value.get('count'): pixels[name][key].append(value['p50'])
        a, b = row['variants']['M0'], row['variants'][NAMES[-1]]
        if a['valid'] and not b['valid']: invalid.append(row['index'])
        for before, after in zip(a['boundaries'], b['boundaries']):
            loss = lost_support(support_intervals(before), support_intervals(after))
            if loss > 1e-6: losses.append(dict(index=row['index'], phase=row['phase'], role=before['role'], lost_m=loss,
                                              candidate_intervals=support_intervals(after)))
    report = dict(schema='joint_family_source_review_v1', frames=len(records), independent_gt=False,
        tool_sha256=sha(__file__), configs={n: sha(args.config / n) for n in ('cameras_v3.yaml', 'bev/lane_fitting.yaml')},
        fitter_sha256=sha(args.config.parent / 'src/asmc_camera/bev/shared_lane_family.py'),
        validity={n: sum(r['variants'][n]['valid'] for r in records) for n in NAMES}, new_invalid=invalid,
        pixel_distance={n: {k: stats(v) for k, v in values.items()} for n, values in pixels.items()},
        stages_ms={n: stats(v) for n, v in stages.items()}, support_loss=losses, temporal_pairs=len(pairs),
        temporal={phase: {n: stats([r['variants'][n]['p95'] for r in pairs if phase == 'all' or r['phase'] == phase]) for n in NAMES}
                  for phase in ('all', 'accelerate', 'hold', 'brake', 'rest', 'accelerate2', 'steer', 'brake2')},
        width_step={n: {side: stats([abs(b['variants'][n]['widths_m'][i] - a['variants'][n]['widths_m'][i])
            for a, b in zip(records, records[1:]) if a['phase'] == b['phase'] and 0 < b['stamp_ns'] - a['stamp_ns'] <= 200e6])
            for i, side in enumerate(('left', 'right'))} for n in NAMES})
    (args.output / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
    (args.output / 'frames.jsonl').write_text(''.join(json.dumps(r) + '\n' for r in records))
    page = '<!doctype html><meta charset="utf-8"><title>IMU 공동 폭·곡선 계산 대조</title><style>body{background:#14202a;color:#eee;font:17px sans-serif;margin:24px}img{width:100%}pre{white-space:pre-wrap}</style><h1>원본 시각 IMU: 폭·곡선 공동 계산</h1><p>열: M0 / 이전 M1+폭 완화 / 공동 계산 M1. 원본 RGB·자체 투영. 독립 GT 정확도가 아닙니다.</p>'
    page += ''.join('<img loading="lazy" src="' + p + '">' for p in pictures)
    page += '<pre>' + html.escape(json.dumps(report, ensure_ascii=False, indent=2)) + '</pre>'
    (args.output / 'index.html').write_text(page)
    print(json.dumps({k: report[k] for k in ('validity', 'new_invalid', 'pixel_distance', 'support_loss', 'stages_ms')}, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('trial', 'config', 'output'): parser.add_argument('--' + name, type=Path, required=True)
    main(parser.parse_args())
