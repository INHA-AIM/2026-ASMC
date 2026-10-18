#!/usr/bin/env python3
"""Offline multiview density sensitivity; never changes the runtime fitter.

Same semantic grids, source stamps, 36cm plane and family constraints. Camera
pixel density is normalized only inside the existing same-class metric clusters.
Semantic reprojection is internal consistency, not independent accuracy GT.
"""
import argparse
import html
import json
from pathlib import Path
import time

import cv2
import numpy as np
import yaml
from asmc_camera.bev.calibration import calibrations_from_dict
from asmc_camera.bev.model_grid_projection import ModelGridGroundProjector, MetricEvidence
from asmc_camera.bev.lane_evidence import LaneNode
from asmc_camera.bev.lane_tracking import associate_lane_nodes_with_audit
from asmc_camera.bev.boundary_candidates import select_shadow_lane_tracks_with_audit
from asmc_camera.bev.shared_lane_family import fit_shared_lane_family_with_audit
from tools.perception.review_family_motion import Fitter, family_dict, read_rows, stats, sha, reprojection
from tools.perception.summarize_family_motion import sample_boundary
from tools.perception.review_family_motion import render_family
from asmc_camera.bev.grid import BevGridSpec
from asmc_camera.bev.visualization import draw_metric_grid


def balanced_nodes(evidence, x_bin_m=.25, lateral_gap_m=.30, x_origin_m=-5.):
    """Preserve single-view nodes; cap multiview pixel-count leverage.

    Cameras retain confidence*geometric-quality weights, so weak edge evidence
    is not given equal trust. Same-class disagreement beyond the existing gap
    stays separate. Source bits and effective pixel support are preserved.
    """
    if x_bin_m <= 0 or lateral_gap_m <= 0:
        raise ValueError('invalid cluster dimensions')
    if not len(evidence.xy):
        return [], []
    xy = np.asarray(evidence.xy, dtype=float)
    source = np.asarray(evidence.source_mask, dtype=np.uint8)
    if np.any((source == 0) | ((source & (source - 1)) != 0)):
        raise ValueError('input points must retain a single camera bit')
    bins = np.floor((xy[:, 0] - x_origin_m) / x_bin_m).astype(np.int64)
    cls = np.asarray(evidence.class_id)
    order = np.lexsort((xy[:, 1], bins, cls))
    split = ((cls[order][1:] != cls[order][:-1]) |
             (bins[order][1:] != bins[order][:-1]) |
             (np.diff(xy[order, 1]) > lateral_gap_m))
    groups = np.split(order, np.flatnonzero(split) + 1)
    result, overlap = [], []
    for group in groups:
        conf = evidence.confidence[group].astype(float)
        quality = evidence.geometric_quality[group].astype(float)
        weights = np.maximum(conf * quality, 1e-6)
        cameras, inv, counts = np.unique(source[group], return_inverse=True, return_counts=True)
        if len(cameras) > 1:
            centers = [np.average(xy[group][inv == i], axis=0, weights=weights[inv == i])
                       for i in range(len(cameras))]
            overlap.append(dict(class_id=int(cls[group[0]]), x_bin=int(bins[group[0]]),
                                source_mask=int(np.bitwise_or.reduce(cameras)),
                                centers_xy=np.asarray(centers).tolist(),
                                lateral_disagreement_m=float(np.ptp(np.asarray(centers)[:, 1])),
                                dominant_pixel_fraction=float(counts.max() / counts.sum())))
            weights = weights / counts[inv]
        result.append(LaneNode(np.average(xy[group], axis=0, weights=weights),
                               int(cls[group[0]]), float(np.average(conf, weights=weights)),
                               float(np.average(quality, weights=weights)),
                               int(np.bitwise_or.reduce(cameras)), int(counts.max()),
                               int(bins[group[0]])))
    return sorted(result, key=lambda node: (node.class_id, node.x_bin, float(node.xy[1]))), overlap


def fit_nodes(nodes, config):
    tracks, _ = associate_lane_nodes_with_audit(nodes, **{k: config[k] for k in (
        'max_track_gap_m', 'max_lateral_residual_m', 'max_heading_change_deg',
        'min_track_nodes', 'min_observed_span_m')})
    tracks, _ = select_shadow_lane_tracks_with_audit(tracks, config['shadow_boundary'])
    return fit_shared_lane_family_with_audit(tracks, config['shared_lane_family'])


def write_gallery(output, report, frames):
    """Metric plots only: no neighboring RGB frame is substituted for exact RGB."""
    grid = BevGridSpec(-5, 20, -8, 8, .05)
    cards = []
    for index, row in enumerate(frames):
        if row['sequence_index'] != 5:
            continue
        canvas = np.zeros((*grid.shape, 3), dtype=np.uint8)
        draw_metric_grid(canvas, grid)
        for node in row['overlap']:
            sources = [bit for bit in (1, 2, 4) if node['source_mask'] & bit]
            for bit, xy in zip(sources, node['centers_xy']):
                uv = np.rint(grid.metric_to_pixel(np.asarray([xy]))[0]).astype(int)
                color = {1: (255, 220, 30), 2: (80, 255, 80), 4: (20, 140, 255)}[bit]
                cv2.circle(canvas, tuple(uv), 2, color, -1)
        panels = [canvas] + [render_family(row['variants'][key], grid)
                            for key in ('all_views', 'density_balanced', 'front_only')]
        labels = ['overlap nodes: front cyan / left green / right orange',
                  'current 3 views', 'offline density balanced', 'front only (ablation)']
        framed = []
        for panel, label in zip(panels, labels):
            panel = cv2.copyMakeBorder(panel, 36, 0, 0, 0, cv2.BORDER_CONSTANT)
            cv2.putText(panel, label, (5, 19), cv2.FONT_HERSHEY_SIMPLEX, .34, (255, 255, 255), 1)
            framed.append(panel)
        name = 'metric-%03d.png' % index
        cv2.imwrite(str(output / name), np.hstack(framed))
        title = '%s / %s' % (row['run_id'], row['phase'])
        cards.append('<figure><figcaption>%s</figcaption><a href="%s"><img loading="lazy" src="%s"></a></figure>'
                     % (html.escape(title), name, name))
    rows = []
    for name, row in report['summary'].items():
        coverage = ' / '.join('%.1f%%' % (100 * row['usability'][zone]['coverage_ratio']['p50'])
                              for zone in ('0-5', '5-10', '10-15'))
        rows.append('<tr><td>%s</td><td>%d/216</td><td>%s</td><td>%.2f px</td><td>%.1f ms</td></tr>'
                    % (name, row['valid'], coverage, row['reprojection_frame_p95_px']['p50'], row['compute_ms']['p50']))
    page = '''<!doctype html><html lang="ko"><meta charset="utf-8"><title>3뷰 차선 겹침·밀도 보정 대조</title>
<style>body{font:16px system-ui;background:#151b24;color:#e9eef7;margin:30px}a{color:#84c9ff}table{border-collapse:collapse}td,th{border:1px solid #566070;padding:10px}img{width:100%%;max-width:1500px}figure{margin:25px 0}figcaption{padding:8px}</style>
<h1>3뷰 차선 겹침·밀도 보정 대조</h1><p>고정 Large semantics · 0.36m · 6개 날씨/시간 run × 진입/내부/출구 · 216묶음.</p>
<p>현재 피팅은 연속 차량 좌표 근거를 사용합니다. 아래 격자는 결과 표시용입니다. 영상 GT의 정확도 향상을 검증한 실험이 아닙니다.</p>
<table><tr><th>구성</th><th>family 유효</th><th>두 차로 띠 지지 비율 중앙값: 0–5 / 5–10 / 10–15m</th><th>자체 semantic 재투영 p95의 중앙값</th><th>CPU 계산 중앙값</th></tr>%s</table>
<p>겹침 노드의 카메라별 횡방향 차이 p50=%.1fcm / p95=%.1fcm. 밀도 보정 후보는 유효성을 유지했지만 자체 재투영과 계산 비용이 개선되지 않아 운영에 채택하지 않았습니다.</p>
<p>전방 단독도 family 전체는 유효하지만 근거리 지원 범위는 다릅니다. 자체 semantic 재투영 값만으로 전방 단독을 더 정확하다고 판단하지 않습니다.</p>
<p>CPU 계산은 오프라인 Python 후보의 관측값이며 실시간 모델 속도 순위가 아닙니다. 원본 시각 보존 후 움직이는 구간의 시간 정합과 독립 RGB/거리 GT 대조가 다음 단계입니다.</p>
<p><a href="report.json">집계</a> · <a href="frames.jsonl">프레임별 원본 결과</a></p>%s</html>''' % (
        ''.join(rows), 100 * report['overlap_lateral_disagreement_m']['p50'],
        100 * report['overlap_lateral_disagreement_m']['p95'], ''.join(cards))
    (output / 'index.html').write_text(page)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('grids', 'config', 'output'):
        parser.add_argument('--' + name, type=Path, required=True)
    args = parser.parse_args(); args.output.mkdir(parents=True, exist_ok=False)
    cv2.setNumThreads(1)
    config = yaml.safe_load((args.config / 'bev/lane_fitting.yaml').read_text())
    cameras = calibrations_from_dict(yaml.safe_load((args.config / 'cameras_v3.yaml').read_text()))
    projectors = {view: ModelGridGroundProjector(camera, source_mask=1 << i,
                  model_hw=(544, 960) if view == 'front' else (480, 640), max_ground_range_m=22)
                  for i, (view, camera) in enumerate(cameras.items())}
    fitter = Fitter(config); frames = []; disagreements = []; dominant = []; changes = []
    query = np.arange(.5, 14.501, .25)
    for row in read_rows(args.grids / 'manifest.jsonl'):
        path = args.grids / row['grid']
        if sha(path) != row['grid_sha256']:
            raise ValueError('grid SHA mismatch')
        with np.load(path) as arrays:
            by_view = {view: p.extract(arrays[view + '_class'], arrays[view + '_confidence'], {1: .5, 2: .5})
                       for view, p in projectors.items()}
            evidence = MetricEvidence.concatenate(by_view.values()); variants = {}
            for label in ('all_views', 'front_only', 'density_balanced'):
                started = time.monotonic()
                if label == 'density_balanced':
                    nodes, overlap = balanced_nodes(evidence, config['x_bin_m'],
                        config['lateral_cluster_gap_m'], config['evidence_x_min_m'])
                    family, audit = fit_nodes(nodes, config)
                    disagreements.extend(x['lateral_disagreement_m'] for x in overlap)
                    dominant.extend(x['dominant_pixel_fraction'] for x in overlap)
                else:
                    family, audit, _ = fitter.fit(evidence if label == 'all_views' else by_view['front'])
                elapsed = 1000 * (time.monotonic() - started)
                variants[label] = dict(family_dict(family, audit), compute_ms=elapsed)
                variants[label]['semantic_reprojection_px'] = reprojection(variants[label], cameras, arrays)
            a, b = variants['all_views'], variants['density_balanced']
            for boundary in a['boundaries']:
                other = next((x for x in b['boundaries'] if x['role'] == boundary['role']), None)
                if other is not None:
                    delta = np.abs(sample_boundary(boundary, query) - sample_boundary(other, query))
                    changes.extend(delta[np.isfinite(delta)].tolist())
            frames.append(dict(run_id=row['run_id'], phase=row['phase'], sequence_index=row['sequence_index'],
                               views=row['views'], overlap=overlap, variants=variants))
    summary = {label: dict(frames=len(frames), valid=sum(r['variants'][label]['valid'] for r in frames),
        compute_ms=stats([r['variants'][label]['compute_ms'] for r in frames]),
        reprojection_frame_p95_px=stats([r['variants'][label]['semantic_reprojection_px'].get('p95', np.nan)
                                         for r in frames]),
        usability={zone: dict(coverage_ratio=stats([
                    r['variants'][label]['usability'].get(zone, {}).get('coverage_ratio', 0) for r in frames]),
                    fully_continuous=sum(r['variants'][label]['usability'].get(zone, {}).get('fully_continuous', False)
                                         for r in frames))
                   for zone in ('0-5', '5-10', '10-15', '15-20')})
        for label in ('all_views', 'front_only', 'density_balanced')}
    report = dict(schema='lane_overlap_review_v1', summary=summary,
        overlap_nodes=len(disagreements), overlap_lateral_disagreement_m=stats(disagreements),
        dominant_camera_pixel_fraction=stats(dominant), common_supported_change_m=stats(changes),
        newly_invalid=sum(r['variants']['all_views']['valid'] and not r['variants']['density_balanced']['valid'] for r in frames),
        config_sha256=sha(args.config / 'cameras_v3.yaml'),
        fitter_sha256=sha(args.config / 'bev/lane_fitting.yaml'),
        manifest_sha256=sha(args.grids / 'manifest.jsonl'),
        limitations=['Cached Large FP32 semantics; no new TensorRT/model quality claim.',
                     'Reprojection to the same model output is internal consistency, not independent GT.',
                     'Header stamps in historical bags are receive stamps, not recovered sensor source time.',
                     'Related short sequences do not establish independent-scene accuracy.',
                     'Candidate is offline only; current runtime remains all_views.'])
    (args.output / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
    (args.output / 'frames.jsonl').write_text(''.join(json.dumps(row) + '\n' for row in frames))
    write_gallery(args.output, report, frames)
    print(json.dumps(report, indent=2), flush=True)


if __name__ == '__main__':
    main()
