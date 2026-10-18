#!/usr/bin/env python3
"""Apply the agreed IMU/width quality gate before any live acceptance runs.

Cached per-frame geometry is reused; temporal metrics are recomputed on exactly
the four agreed variants and identical supported positions. Not independent GT.
"""
import argparse
from collections import defaultdict
import html
import json
from pathlib import Path

import numpy as np

from review_family_motion import stats, sha
from review_source_tunnel_motion import source_signals
from review_tunnel_imu_motion import read, temporal
from summarize_family_motion import sample_boundary

VARIANTS = ('M0', 'M1_source', 'F_width0', 'M1_source_width0')
CANDIDATE = VARIANTS[-1]


def support_intervals(boundary):
    """Union of usable segment extents in0–15m, preserving internal holes."""
    xy = np.asarray(boundary['xy'], dtype=float)
    state = np.asarray(boundary['support'])
    intervals = []
    for a, b, sa, sb in zip(xy[:-1], xy[1:], state[:-1], state[1:]):
        if not sa or not sb:
            continue
        lo, hi = max(0., min(a[0], b[0])), min(15., max(a[0], b[0]))
        if hi > lo:
            intervals.append((float(lo), float(hi)))
    union = []
    for lo, hi in sorted(intervals):
        if union and lo <= union[-1][1]:
            union[-1] = (union[-1][0], max(hi, union[-1][1]))
        else:
            union.append((lo, hi))
    return union


def lost_support(reference, candidate):
    """Baseline support no longer covered, not merely net length reduction."""
    total = 0.
    for lo, hi in reference:
        covered = sum(max(0., min(hi, b) - max(lo, a)) for a, b in candidate)
        total += max(0., hi - lo - covered)
    return total


def summarize_quality(records, pairs):
    if not records:
        raise ValueError('quality acceptance requires nonempty matched records')
    missing = [r['index'] for r in records if any(n not in r['variants'] for n in VARIANTS)]
    if missing:
        raise ValueError('required comparison variant missing: ' + str(missing))
    temporal_summary = {}
    for phase in ('all', 'accelerate', 'brake', 'accelerate2', 'brake2', 'hold', 'steer'):
        temporal_summary[phase] = {n: stats([p['variants'][n]['p95'] for p in pairs
            if (phase == 'all' or p['phase'] == phase) and p['variants'][n].get('count')])
            for n in VARIANTS}
    motion = {}
    for phase in ('accelerate', 'brake', 'accelerate2', 'brake2', 'hold', 'steer'):
        a, b = (temporal_summary[phase][n] for n in ('M0', CANDIDATE))
        enough = a.get('count', 0) > 0 and b.get('count', 0) > 0
        large = phase not in ('hold', 'steer')
        motion[phase] = dict(evaluable=enough, baseline_p95_m=a.get('p95'),
            candidate_p95_m=b.get('p95'), criterion='reduction>=30%' if large else 'increase<=0.01m',
            passed=bool(enough and (b['p95'] <= .70 * a['p95'] if large else b['p95'] - a['p95'] <= .01)))
    pixels = defaultdict(lambda: {'baseline': [], 'candidate': [], 'missing_pairs': []})
    losses, invalid, role_errors, bounds = [], [], [], []
    for row in records:
        base, candidate = (row['variants'][n] for n in ('M0', CANDIDATE))
        if base['valid'] and not candidate['valid']:
            invalid.append(row['index'])
        amap = {b['role']: b for b in base['boundaries']}
        bmap = {b['role']: b for b in candidate['boundaries']}
        if set(amap) != set(bmap) or any(amap[k]['class_id'] != bmap[k]['class_id'] for k in amap.keys() & bmap.keys()):
            role_errors.append(dict(index=row['index'], reason='role_or_class_mismatch'))
        for role in amap.keys() & bmap.keys():
            lost = lost_support(support_intervals(amap[role]), support_intervals(bmap[role]))
            if lost > 1e-6:
                losses.append(dict(index=row['index'], phase=row['phase'], role=role, lost_m=lost))
        roles = ('left_outer_boundary', 'yellow_centre_boundary', 'right_outer_boundary')
        if all(k in bmap for k in roles):
            samples = [sample_boundary(bmap[k], np.arange(.5, 14.501, .25)) for k in roles]
            good = np.logical_and.reduce([np.isfinite(v) for v in samples])
            if np.any((samples[0][good] <= samples[1][good]) | (samples[1][good] <= samples[2][good])):
                role_errors.append(dict(index=row['index'], reason='lateral_role_order'))
        if any(not 2.4 <= w <= 4.6 for w in candidate.get('widths_m', [])):
            bounds.append(row['index'])
        a, b = base['semantic_model_px'], candidate['semantic_model_px']
        for key in set(a) | set(b):
            if not key.startswith('observed/'):
                continue
            if not a.get(key, {}).get('count') or not b.get(key, {}).get('count'):
                pixels[key]['missing_pairs'].append(row['index'])
                continue
            pixels[key]['baseline'].append(a[key]['p50'])
            pixels[key]['candidate'].append(b[key]['p50'])
    pixel_summary = {}
    for key, values in pixels.items():
        a, b = stats(values['baseline']), stats(values['candidate'])
        enough = a.get('count', 0) > 0
        delta = b['p95'] - a['p95'] if enough else None
        pixel_summary[key] = dict(baseline=a, candidate=b, increase_px=delta,
            missing_pairs=values['missing_pairs'], passed=bool(enough and delta <= 1. and not values['missing_pairs']))
    width_delta = {}
    for name in VARIANTS:
        values = []
        for a, b in zip(records, records[1:]):
            if a['phase'] == b['phase'] and 0 < b['stamp_ns'] - a['stamp_ns'] <= 200e6:
                values.append(np.abs(np.asarray(b['variants'][name]['widths_m']) - a['variants'][name]['widths_m']))
        width_delta[name] = {side: stats([v[i] for v in values]) for i, side in enumerate(('left', 'right'))}
    checks = dict(motion=all(v['passed'] for v in motion.values()),
        observed_pixel_distance=bool(pixel_summary) and all(v['passed'] for v in pixel_summary.values()),
        support_preserved=not losses, no_new_invalid=not invalid,
        roles_preserved=not role_errors, width_bounds=not bounds)
    return dict(passed=all(checks.values()), checks=checks, frames=len(records),
        independent_gt=False, temporal_pairs=len(pairs), temporal=temporal_summary, motion=motion,
        observed_pixel_distance=pixel_summary, lost_support=losses, new_invalid=invalid,
        role_errors=role_errors, width_bound_failures=bounds, width_step_m=width_delta,
        prior_010_condition_met=any(width_delta[CANDIDATE][s].get('p95', 0) > width_delta['M0'][s].get('p95', 0)
                                   for s in ('left', 'right')),
        defaults_changed=False)


def main(args):
    args.output.mkdir(parents=True, exist_ok=False)
    records = read(args.review / 'frames.jsonl')
    _, ego, _ = source_signals(args.trial / 'capture', read(args.trial / 'poses.jsonl'))
    pairs = temporal(records, {'live-motion-r1': {'/Ego_topic': ego}}, variants=VARIANTS)
    report = summarize_quality(records, pairs)
    report.update(schema='camera_imu_width_acceptance_v1', tool_sha256=sha(__file__),
        input_sha256={str(p): sha(p) for p in (args.review / 'frames.jsonl',
            args.trial / 'capture/events.jsonl', args.trial / 'poses.jsonl')},
        live_acceptance='pending' if report['passed'] else 'skipped_quality_gate_failed')
    (args.output / 'report.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    (args.output / 'temporal-pairs.jsonl').write_text(''.join(json.dumps(r) + '\n' for r in pairs))
    page = '<!doctype html><meta charset="utf-8"><title>IMU + 폭 완화 수락 검사</title><style>body{background:#14202a;color:#eee;font:16px sans-serif;margin:30px}pre{white-space:pre-wrap}a{color:#8df}</style>'
    page += '<h1>IMU + 폭 사전값 완화 품질 검사</h1><p>판정: ' + ('통과' if report['passed'] else '미통과 · 운영 M0 유지') + '</p>'
    page += '<p>원본 시각 터널226묶음. 4개 후보의 공통 지점으로 시간 지표를 재계산했습니다. 독립 GT 정확도가 아닙니다.</p><p><a href="report.json">수락 검사</a> · <a href="temporal-pairs.jsonl">시간 비교</a></p>'
    page += '<pre>' + html.escape(json.dumps(report, indent=2, ensure_ascii=False)) + '</pre>'
    (args.output / 'index.html').write_text(page)
    print(json.dumps({k: report[k] for k in ('passed', 'checks', 'lost_support', 'prior_010_condition_met', 'live_acceptance')}, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('review', 'trial', 'output'):
        parser.add_argument('--' + name, type=Path, required=True)
    main(parser.parse_args())
