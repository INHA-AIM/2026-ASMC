#!/usr/bin/env python3
"""Targeted pixel regressions and the five historical boundary-drop failures.

Old bag attitude is receive-time sensitivity only. Support is removed after
association, not from RGB. Raw/merged/fitted reprojection is internal consistency.
"""
import argparse
from collections import defaultdict
from copy import deepcopy
import html
import json
from pathlib import Path

import cv2
import numpy as np
import yaml

from asmc_camera.bev.calibration import calibrations_from_dict
from asmc_camera.bev.imu_projection import attitude_camera, correct_evidence
from asmc_camera.bev.model_grid_projection import ModelGridGroundProjector, MetricEvidence
from asmc_camera.bev.lane_evidence import reduce_evidence_to_nodes
from asmc_camera.bev.lane_tracking import associate_lane_nodes_with_audit
from asmc_camera.bev.boundary_candidates import select_shadow_lane_tracks_with_audit
from asmc_camera.bev.shared_lane_family import _assign_observed_roles, _config, fit_shared_lane_family_with_audit
from review_family_motion import interpolate, overlay, sha
from review_source_tunnel_motion import source_signals
from review_family_dropout import cases, describe
from review_tunnel_imu_motion import read, fit, stage_distances, VIEWS


def main(args):
    args.output.mkdir(parents=True, exist_ok=False)
    cv2.setNumThreads(1)
    cfg = yaml.safe_load((args.config / 'bev/lane_fitting.yaml').read_text())
    cams = calibrations_from_dict(yaml.safe_load((args.config / 'cameras_v3.yaml').read_text()))
    projectors = {v: ModelGridGroundProjector(cams[v], model_hw=(544, 960) if v == 'front' else (480, 640),
        source_mask=1 << i, max_ground_range_m=cfg['max_ground_range_m'],
        x_bounds_m=(cfg['evidence_x_min_m'], cfg['evidence_x_max_m']),
        y_bounds_m=(cfg['evidence_y_min_m'], cfg['evidence_y_max_m'])) for i, v in enumerate(VIEWS)}
    capture = args.trial / 'capture'
    frames = read(capture / 'frames.jsonl')
    imu, _, _ = source_signals(capture, read(args.trial / 'poses.jsonl'))
    records = read(args.review / 'frames.jsonl')
    # One coverage regression and the three greatest right-view frame-median
    # regressions, selected by the already fixed metric, not by appearance.
    worst = sorted(records, key=lambda r: r['variants']['M1_source_width0']['semantic_model_px'].get(
        'observed/right/1', {}).get('p50', 0) - r['variants']['M0']['semantic_model_px'].get(
        'observed/right/1', {}).get('p50', 0), reverse=True)
    indices = sorted(set([18] + [r['index'] for r in worst[:3]]))
    details, gallery = [], []
    for index in indices:
        sample, record = frames[index], records[index]
        if sha(capture / sample['grid']) != record['grid_sha256']:
            raise ValueError('source grid SHA mismatch')
        arrays = dict(np.load(capture / sample['grid']))
        corrected = {}
        for view in VIEWS:
            raw = sample['views'][view]['lineage']['sensor_timing']
            q = interpolate(imu, raw['source_sec'] * 10**9 + raw['source_nsec'], 'imu')
            if q is None: raise ValueError('source attitude unavailable')
            corrected[view] = attitude_camera(cams[view], q['q_xyzw'])
        base = {v: projectors[v].extract(arrays[v + '_class'], arrays[v + '_confidence'], {1: .5, 2: .5}) for v in VIEWS}
        maps = {v: {label: cv2.distanceTransform((arrays[v + '_class'] != label).astype(np.uint8), cv2.DIST_L2, 5)
                    for label in (1, 2)} for v in VIEWS}
        stages = {}
        for name in ('M0', 'M1_source_width0'):
            moving = name.startswith('M1')
            used = {v: correct_evidence(projectors[v], base[v], corrected[v]) for v in VIEWS} if moving else base
            c = deepcopy(cfg)
            if moving: c['shared_lane_family']['width_prior_strength'] = 0.
            _, nodes, _ = fit(MetricEvidence.concatenate(list(used.values())), c)
            stages[name] = stage_distances(used, nodes, corrected if moving else cams, projectors, arrays, maps)
        panels = []
        for view in ('left', 'front', 'right'):
            image = sample['views'][view]
            if sha(capture / image['rgb']) != image['rgb_sha256']: raise ValueError('RGB SHA mismatch')
            rgb = cv2.imread(str(capture / image['rgb']))
            left, top, right, bottom = projectors[view].geometry.pad_ltrb
            labels = arrays[view + '_class']
            crop = labels[top:labels.shape[0] - bottom, left:labels.shape[1] - right]
            native = cv2.resize(crop, (rgb.shape[1], rgb.shape[0]), interpolation=cv2.INTER_NEAREST)
            seg = rgb.copy()
            for label, color in ((1, (255, 255, 255)), (2, (0, 255, 255))):
                seg[native == label] = .35 * seg[native == label] + .65 * np.array(color)
            row = []
            for name, canvas in [('segmentation', seg)] + [(n, overlay(rgb, record['variants'][n], corrected[view] if n.startswith('M1') else cams[view]))
                    for n in ('M0', 'M1_source', 'F_width0', 'M1_source_width0')]:
                canvas = cv2.resize(canvas, (420, 260))
                cv2.putText(canvas, view + ' ' + name, (5, 23), cv2.FONT_HERSHEY_SIMPLEX, .52, (0, 100, 255), 2)
                row.append(canvas)
            panels.append(np.hstack(row))
        file = 'source-%03d-%s.jpg' % (index, record['phase'])
        cv2.imwrite(str(args.output / file), np.vstack(panels))
        gallery.append(file)
        details.append(dict(index=index, phase=record['phase'], stage_pixel_distance=stages,
            final_pixel_distance={n: record['variants'][n]['semantic_model_px'] for n in ('M0', 'M1_source_width0')}))

    previous = [r for r in read(args.dropout) if r['case'].startswith('drop:') and not r['result']['valid']]
    if len(previous) != 5: raise ValueError('expected original five boundary-drop failures')
    counters = defaultdict(int); lookup = {}
    for frame in read(args.legacy / 'representatives.jsonl'):
        run = frame['run_id']; index = counters[run]; counters[run] += 1
        lookup[run, index] = frame
    signals = defaultdict(list)
    for row in read(args.legacy / 'inputs/signals.jsonl'):
        if row['topic'] == '/imu': signals[row['run_id']].append(row)
    for values in signals.values(): values.sort(key=lambda r: r['source_ns'])
    dropout = []
    for old in previous:
        frame = lookup[old['run_id'], old['index']]
        path = args.legacy / 'representative-grids' / ('%s-%03d.npz' % (old['run_id'], old['index']))
        if sha(path) != old['grid_sha256']: raise ValueError('historical dropout grid SHA mismatch')
        arrays = dict(np.load(path)); variants = {}
        for name in ('M0', 'M1_receive_width0'):
            pieces = []
            for view in VIEWS:
                evidence = projectors[view].extract(arrays[view + '_class'], arrays[view + '_confidence'], {1: .5, 2: .5})
                if name != 'M0':
                    q = interpolate(signals[old['run_id']], frame['views'][view]['header_ns'], 'imu')
                    if q is None: raise ValueError('legacy IMU bracket unavailable')
                    evidence = correct_evidence(projectors[view], evidence, attitude_camera(cams[view], q['q_xyzw']))
                pieces.append(evidence)
            nodes = reduce_evidence_to_nodes(MetricEvidence.concatenate(pieces), cfg['x_bin_m'], cfg['lateral_cluster_gap_m'], cfg['evidence_x_min_m'])
            tracks, _ = associate_lane_nodes_with_audit(nodes, **{k: cfg[k] for k in
                ('max_track_gap_m', 'max_lateral_residual_m', 'max_heading_change_deg', 'min_track_nodes', 'min_observed_span_m')})
            tracks, _ = select_shadow_lane_tracks_with_audit(tracks, cfg['shadow_boundary'])
            parameters = dict(cfg['shared_lane_family'], width_prior_strength=0. if name != 'M0' else .35)
            assigned, _ = _assign_observed_roles(tracks, _config(parameters))
            remaining = next(t for case, t, _ in cases(assigned) if case == old['case'])
            family, audit = fit_shared_lane_family_with_audit(remaining, parameters)
            variants[name] = describe(family, audit)
        if variants['M0']['valid'] != old['result']['valid']:
            raise ValueError('old dropout failure no longer reproduced')
        dropout.append(dict(run_id=old['run_id'], index=old['index'], phase=old['phase'], case=old['case'],
            original_reason=old['result']['reason'], variants=variants,
            source_timing_available=False, operational_fix_claimed=False))
    report = dict(schema='imu_width_failure_review_v1', tool_sha256=sha(__file__), source_examples=details,
        historical_dropout=dropout, dropout_input_sha256=sha(args.dropout), independent_gt=False,
        note='Legacy M1 is receive-time sensitivity. Role support removal is downstream fitter stress, not image occlusion. No operational fixes claimed.')
    (args.output / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
    page = '<!doctype html><meta charset="utf-8"><title>IMU 폭 완화 실패 장면</title><style>body{background:#14202a;color:white;font:16px sans-serif;margin:24px}img{width:100%}pre{white-space:pre-wrap}a{color:#8df}</style><h1>IMU·폭 완화: 측방 밀착도와 지원 범위</h1><p>열: segmentation / M0 / M1 / M0+prior0 / M1+prior0. 같은 RGB·후보별 투영. 독립 GT가 아닙니다.</p>'
    page += ''.join('<figure><figcaption>' + html.escape(f) + '</figcaption><img src="' + f + '"></figure>' for f in gallery)
    page += '<p><a href="report.json">중간 단계·기존 실패5건</a></p><pre>' + html.escape(json.dumps(report, ensure_ascii=False, indent=2)) + '</pre>'
    (args.output / 'index.html').write_text(page)
    print(json.dumps(dict(source_examples=len(details), historical_dropout=len(dropout),
        candidate_dropout_valid=sum(r['variants']['M1_receive_width0']['valid'] for r in dropout))), flush=True)


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    for name in ('trial', 'review', 'legacy', 'dropout', 'config', 'output'):
        p.add_argument('--' + name, type=Path, required=True)
    main(p.parse_args())
