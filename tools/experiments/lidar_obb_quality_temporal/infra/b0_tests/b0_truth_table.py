#!/usr/bin/env python3
"""Per-scan topic truth table for one B0 integration run (offline; no ROS needed). G4 version of
analysis/tests/skeletons/b0_truth_table.py.

usage: b0_truth_table.py --run-dir RT/NAME --catalogue fault_catalogue_g4.json --scenario ID [--variant TAG]
                         [--profile integration|production|positive_control] > truth_table.json
Association:
  EXACT   typed V1/V2 arrays: header.stamp == scan stamp (ns)            (FrameBoundaryPublisher.cpp BuildV2/Publish)
  MS      legacy + markers + debug clouds + tf: floor(stamp/1 ms) == scan ms (feeder stamps on a 1 ms grid)
  WINDOW  wall-stamped or unstamped topics (/lidar/object(_boxes), diagnostics, ctrl, planning*, UDP datagrams):
          arrival CLOCK_MONOTONIC in [publish(scan k), publish(next accepted scan))
Any EXACT/MS-class message matching no scan => INCONCLUSIVE (never silently dropped).
Final outputs (decision 4, plan D3/D15): /lidar/local_object, /lidar/map_object. Legacy outputs: /last_lidar,
/lidar/tracked_objects_map, /lidar/motion_debug (+ planner UDP). V1 typed topics, /lidar/object(_boxes),
/lidar_tracking_markers and /lidar/debug_* are recorded but non-final (informational D_all only).
Main's own per-scan decision is read from the fusion trace (lidar_process_end v2_mode/v2_reason/legacy_publish)
and cross-checked against the topics (trace_matches_topics) when the trace was flushed.
"""
import argparse
import json
import math
import os
import sys
from bisect import bisect_right
from collections import Counter, defaultdict

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import b0_scenario as bs  # noqa: E402

EXACT = {'local_v2', 'map_v2', 'local_v1', 'map_v1'}
MSK = {'last_lidar', 'tracked_map', 'motion_debug', 'last_markers', 'map_markers', 'local_markers', 'debug_roi',
       'debug_clustered_map', 'tf'}
WINDOW = {'object_boxes', 'object_cloud', 'diag', 'ctrl_discard', 'ctrl_public', 'planning_tracked', 'planning_objects'}
FINAL = ('local_v2', 'map_v2')
# Bag smoke (no ground truth): only structural checks decide; oracle-based ones are informational.
SMOKE = {'map_requires_same_stamp_local', 'local_present_when_output_expected', 'legacy_only_in_map', 'legacy_frame_wide_motion_gate',
         'frames_and_stamps', 'no_duplicate_per_scan', 'motion_contract_v2', 'no_zero_center_objects',
         'warmup_never_in_legacy', 'trace_matches_topics'}
LEGACY = ('last_lidar', 'tracked_map', 'motion_debug')
DATA = (1, 2)            # OBSERVED, OBSERVED_EMPTY (LidarObjectArray(V2).msg)
INPUT_INVALID, POSE_UNAVAILABLE = 3, 4


def load(path):
    rows = []
    if os.path.exists(path):
        for line in open(path, errors='replace'):
            line = line.strip()
            if line:
                try:
                    rows.append(json.loads(line))
                except ValueError:
                    rows.append(dict(parse_error=line[:120]))
    return rows


def adds(m):
    return (m or {}).get('actions', {}).get('add', 0)


def deleteall_only(m):
    a = (m or {}).get('actions', {})
    return a.get('deleteall', 0) > 0 and not a.get('add', 0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-dir', required=True)
    ap.add_argument('--catalogue', required=True)
    ap.add_argument('--scenario', required=True)
    ap.add_argument('--variant', default='normal')
    ap.add_argument('--profile', default='integration',
                    choices=('integration', 'production', 'positive_control', 'bag', 'target'))
    ap.add_argument('--feeder-file', default='feeder.jsonl', help="'inputs.jsonl' for the bag smoke (recorder --record-inputs)")
    a = ap.parse_args()
    cat = json.load(open(a.catalogue))
    cp = cat['contract_parameters']
    sc = next((s for s in cat['scenarios'] if s['id'] == a.scenario), dict(id=a.scenario, phases=[], hand_expect=[]))
    feed = load(os.path.join(a.run_dir, a.feeder_file))
    rec_all = load(os.path.join(a.run_dir, 'recorder.jsonl'))
    rec = [r for r in rec_all if 'k' in r]
    ready = next((r for r in rec_all if r.get('ev') == 'ready'), {})
    events = [e for e in feed if e.get('ev') in ('imu', 'gps', 'can', 'lidar', 'yolo')]
    truth = {e['k']: e['objects'] for e in feed if e.get('ev') == 'truth'}
    oracle = bs.run_oracle(events, cp)
    scans = sorted((e for e in events if e['ev'] == 'lidar'), key=lambda e: e['mono_ns'])
    by_ns = {e['stamp_ns']: e['k'] for e in scans if oracle[e['k']]['mode'] != 'REJECTED'}
    by_ms = {e['stamp_ns'] // bs.MS: e['k'] for e in scans if oracle[e['k']]['mode'] != 'REJECTED'}
    accepted = [e for e in scans if oracle[e['k']]['mode'] != 'REJECTED']
    starts = [e['mono_ns'] for e in accepted]
    obs = defaultdict(lambda: defaultdict(list))
    unmatched = defaultdict(int)
    for r in rec:
        key = r['k']
        if key in EXACT:
            k = by_ns.get(r.get('stamp_ns'))
        elif key == 'tf':
            k = next((by_ms.get(s // bs.MS) for s in r.get('leveled', []) if by_ms.get(s // bs.MS) is not None), None)
        elif key in MSK and r.get('stamp_ns') is not None:
            k = by_ms.get(r['stamp_ns'] // bs.MS)
        else:
            j = bisect_right(starts, r['mono_ns']) - 1
            k = accepted[j]['k'] if j >= 0 else None
        if k is None:
            if key in EXACT or (key in MSK and r.get('stamp_ns') is not None):
                unmatched[key] += 1
            continue
        obs[k][key].append(r)
    # Main's own per-scan decision (fusion trace, flushed at shutdown; B-15 abort happens after the flush)
    trace = {}
    tpath = os.path.join(a.run_dir, 'fusion_min.jsonl')
    if not os.path.exists(tpath):
        tpath = os.path.join(a.run_dir, 'fusion.jsonl')
    for t in load(tpath):
        if t.get('event') == 'lidar_process_end':
            trace[t.get('stamp_ns')] = t
    # Positive control only: UDP datagrams decoded by b0_udp_sink.py inside the same namespace
    udp_rows = [u for u in load(os.path.join(a.run_dir, 'udp_sink.jsonl')) if u.get('ev') == 'datagram']
    udp_by_scan = defaultdict(list)
    for u in udp_rows:
        j = bisect_right(starts, u['mono_ns']) - 1
        if j >= 0:
            udp_by_scan[accepted[j]['k']].append(u)
    rows, viol, info = [], defaultdict(list), defaultdict(list)
    hand = {(h['phase'], h['scan']): h['mode'] for h in sc.get('hand_expect', [])}
    stats = Counter()
    deleteall_events = Counter()
    diag_values = defaultdict(Counter)
    trace_modes = Counter()
    for e in scans:
        k, o = e['k'], oracle[e['k']]
        ob = obs.get(k, {})
        one = {key: (v[-1] if v else None) for key, v in ob.items()}
        counts = {key: len(v) for key, v in ob.items()}
        dup = {key: len(v) for key, v in ob.items() if key in EXACT | {'last_lidar', 'tracked_map'} and len(v) > 1}
        lv2, mv2 = one.get('local_v2'), one.get('map_v2')
        l_data = bool(lv2 and lv2.get('status') in DATA)
        m_data = bool(mv2 and mv2.get('status') in DATA)
        legacy = any(one.get(t) for t in LEGACY)
        local_any = l_data or adds(one.get('local_markers')) > 0 or adds(one.get('last_markers')) > 0 or \
            adds(one.get('object_boxes')) > 0 or bool(one.get('object_cloud', {}) and one['object_cloud'].get('points'))
        map_any = m_data or legacy or adds(one.get('map_markers')) > 0 or bool(one.get('debug_clustered_map'))

        def mode(lo, ma):
            return 'BOTH' if lo and ma else 'LOCAL' if lo else 'MAP' if ma else 'NONE'
        observed = {'D_v2': mode(l_data, m_data), 'D_cross': mode(l_data, legacy), 'D_all': mode(local_any, map_any)}
        exp = o['mode']
        expected_observed = 'BOTH' if exp == 'MAP' else exp
        obs_mode_cmp = ('REJECTED' if not ob else 'OUTPUT_FOR_REJECTED') if exp == 'REJECTED' else observed['D_v2']
        withheld = exp in ('LOCAL', 'NONE') or (exp == 'MAP' and not legacy)
        v2_msgs = [x for x in (lv2, mv2) if x]
        # A duplicate/regressing scan shares its stamp with an earlier accepted scan: the trace line belongs to
        # the accepted one only (fixes a false trace_matches_topics on S21's duplicate scan in the first run).
        tr = trace.get(e['stamp_ns']) if by_ns.get(e['stamp_ns']) == k else None
        for t in ('last_markers', 'map_markers'):
            for m in ob.get(t, []):
                if deleteall_only(m):
                    deleteall_events[t] += 1
        for d in ob.get('diag', []):
            for key in ('v2_output', 'v2_reason', 'gps_state', 'legacy_output', 'map_reason'):
                if key in d.get('values', {}):
                    diag_values[key][d['values'][key]] += 1
            diag_values['message'][d.get('message')] += 1
        if a.profile == 'target':
            # TARGET contrast (informational): no V2 exists, so the B0 contract cannot be met; record what TARGET
            # publishes per scan instead (V1 dual publish, legacy while the oracle says the map pose is invalid).
            checks = _production_checks(one, ob)
            stats['target_legacy_while_oracle_not_map'] += int(legacy and exp != 'MAP')
            stats['target_v1_local_data_while_oracle_none'] += int(exp == 'NONE' and bool(one.get('local_v1')) and
                                                                   one['local_v1'].get('status') in DATA)
            stats['target_v1_map_pose_unavailable_arrays'] += int(bool(one.get('map_v1')) and one['map_v1'].get('status') == POSE_UNAVAILABLE)
            stats['target_v1_input_invalid_arrays'] += sum(int(bool(one.get(t)) and one[t].get('status') == INPUT_INVALID)
                                                           for t in ('local_v1', 'map_v1'))
        elif a.profile == 'production':
            checks = _production_checks(one, ob)
            # KNOWN_GAP/B1 evidence only (decision 5): Unknown-state (warm-up) tracks on /last_lidar in production.
            states = {x.get('id'): x.get('state') for x in (one.get('motion_debug') or {}).get('objects', [])}
            for x in (one.get('last_lidar') or {}).get('objects', []):
                if states.get(x.get('id')) == 0:
                    stats['known_gap_unknown_state_objects_on_last_lidar'] += 1
                    stats['known_gap_unknown_state_objects_speed_kph_zero'] += abs(x.get('kph') or 0.0) < 1e-6
        else:
            checks = {
                'map_requires_same_stamp_local': not m_data or l_data,
                'mode_matches_oracle': obs_mode_cmp == expected_observed,
                'no_map_output_when_map_invalid': not (exp != 'MAP' and (m_data or legacy or adds(one.get('map_markers')) > 0
                                                                          or bool(one.get('debug_clustered_map')))),
                'no_map_v2_message_in_local_scan': not (exp == 'LOCAL' and mv2 is not None),
                'local_present_when_output_expected': exp not in ('MAP', 'LOCAL') or l_data,
                'v2_count_matches_output': len(v2_msgs) == (2 if exp == 'MAP' else 1 if exp == 'LOCAL' else 0),
                'invalid_signal_policy': _invalid_policy(cp['invalid_signal_policy'], exp, lv2, mv2),
                'deleteall_when_legacy_withheld': (exp == 'REJECTED') or not withheld or
                    all(one.get(t, {}) and one[t].get('actions', {}).get('deleteall', 0) > 0 for t in ('last_markers', 'map_markers')),
                'legacy_only_in_map': not legacy or (exp == 'MAP' and m_data),
                'legacy_frame_wide_motion_gate': _legacy_gate(mv2, legacy),
                'frames_and_stamps': _frames(lv2, mv2, cp, e['stamp_ns']),
                'no_duplicate_per_scan': not dup,
                'motion_contract_v2': _motion(lv2, mv2, cp),
                'no_zero_center_objects': _no_zero(one),
                'warmup_never_in_legacy': _warmup(one, mv2),
                'legacy_present_when_map_and_motion_ready': _legacy_present(exp, mv2, legacy),
                'hand_expectation': (lambda h: h is None or (exp == h and obs_mode_cmp == ('BOTH' if h == 'MAP' else h)))(hand.get((e['phase'], e['i']))),
                'trace_matches_topics': _trace_ok(tr, lv2, mv2, legacy, exp),
            }
            if udp_rows or a.profile == 'positive_control':
                checks['udp_only_with_legacy'] = not udp_by_scan.get(k) or legacy
                checks['udp_never_carries_warmup'] = _udp_warmup(udp_by_scan.get(k, []), mv2)
        for name, ok in checks.items():
            if not ok:
                if a.profile == 'bag' and name not in SMOKE:
                    info[name].append(k)       # bag: oracle arrival is the recorder's, informational only
                else:
                    viol[name].append(k)
        stats['scans'] += 1
        stats['oracle_' + exp] += 1
        stats['observed_' + observed['D_v2']] += 1
        stats['simultaneous_local_map_v2'] += observed['D_v2'] == 'BOTH'
        stats['simultaneous_local_v2_and_legacy'] += observed['D_cross'] == 'BOTH'
        stats['map_output_while_map_pose_invalid'] += int(exp != 'MAP' and (m_data or legacy))
        stats['map_v2_messages'] += counts.get('map_v2', 0)
        stats['local_v2_messages'] += counts.get('local_v2', 0)
        stats['legacy_published_scans'] += legacy
        stats['last_lidar_messages'] += counts.get('last_lidar', 0)
        stats['tracked_map_messages'] += counts.get('tracked_map', 0)
        stats['dummy_zero_center_objects'] += _zero_count(one)
        stats['v1_local_messages'] += counts.get('local_v1', 0)
        stats['v1_map_messages'] += counts.get('map_v1', 0)
        stats['v1_dual_publish_scans'] += int(bool(one.get('local_v1')) and bool(one.get('map_v1')))
        stats['udp_datagrams_assoc'] += len(udp_by_scan.get(k, []))
        if tr:
            trace_modes[tr.get('v2_mode', 'absent')] += 1
        rows.append(dict(scan=k, phase=e['phase'], i=e['i'], stamp_ns=e['stamp_ns'], lidar_mode=e['mode'],
                         oracle=o, hand_expect=hand.get((e['phase'], e['i'])), observed_mode=observed,
                         published={key: _brief(key, v) for key, v in one.items()}, counts=counts, duplicates=dup,
                         main_trace=({x: tr.get(x) for x in ('pose_ready', 'processed', 'v2_mode', 'v2_reason', 'legacy_publish',
                                                            'objects', 'duration_ms')} if tr else None),
                         udp_datagrams=[dict(ids=[s['id'] for s in u.get('slots', [])],
                                             speed_kph=[s['speed_kph'] for s in u.get('slots', [])]) for u in udp_by_scan.get(k, [])],
                         checks={n: c for n, c in checks.items() if not c}, geometry=_geometry(sc, truth.get(k), lv2, mv2)))
    processed = sum(1 for r in rows if r['oracle']['mode'] != 'REJECTED')
    no_evidence = [r['scan'] for r in rows if r['oracle']['mode'] in ('MAP', 'LOCAL') and not r['published']]
    verdict = 'PASS'
    if (unmatched and a.profile != 'bag') or ready.get('missing_required'):
        verdict = 'INCONCLUSIVE'
    if any(viol.values()):
        verdict = 'FAIL'
    if no_evidence and verdict == 'PASS' and a.profile != 'bag':
        verdict = 'INCONCLUSIVE'     # skipped scans (latest-frame-wins, main.cpp TemporalThread) are not silently passed
    if udp_rows:
        stats['udp_datagrams_total'] = len(udp_rows)
    geo = [g for r in rows for g in (r['geometry'] or []) if not g['ok']]
    geo_all = [g for r in rows for g in (r['geometry'] or [])]
    stats['geometry_samples'] = len(geo_all)
    stats['geometry_failures'] = len(geo)
    print(json.dumps(dict(schema='b0_topic_truth_table/1+g4', scenario=sc['id'], variant=a.variant, profile=a.profile,
                          run_dir=a.run_dir, scans=len(rows), processed_expected=processed, unmatched_stamped=dict(unmatched),
                          recorder_ready=ready, scans_without_any_output=no_evidence, fusion_trace_scans=len(trace),
                          main_trace_modes=dict(trace_modes), stats=dict(stats), deleteall_only_events=dict(deleteall_events),
                          diagnostics_values={k: dict(v) for k, v in diag_values.items()},
                          violations={k: v for k, v in viol.items() if v},
                          informational_check_failures={k: len(v) for k, v in info.items() if v},
                          geometry_failures=len(geo),
                          verdict=verdict, rows=rows), indent=1, default=float))


def _invalid_policy(policy, exp, lv2, mv2):
    if exp != 'NONE':
        return True
    msgs = [m for m in (lv2, mv2) if m]
    if any(m.get('status') in DATA for m in msgs):
        return False
    if policy.startswith('local_v2_input_invalid'):
        return bool(lv2 and lv2.get('status') == INPUT_INVALID and not mv2)
    if policy.startswith('none'):
        return not msgs
    return len(msgs) <= 1


def _legacy_gate(mv2, legacy):
    """legacy published => map_v2 of the same scan exists and EVERY object in it is motion_valid (frame-wide
    LegacyMotionReady, FrameBoundary.cpp)."""
    if not legacy:
        return True
    if not mv2:
        return False
    objs = mv2.get('objects', [])
    if objs and 'mv' not in objs[0]:
        return True               # V2 motion fields unavailable (pre-B0 type): not decidable here
    return all(o.get('mv') for o in objs)


def _legacy_present(exp, mv2, legacy):
    if exp != 'MAP' or not mv2 or mv2.get('status') not in DATA:
        return True
    objs = mv2.get('objects', [])
    if objs and 'mv' not in objs[0]:
        return True
    return legacy or not all(o.get('mv') for o in objs)


def _frames(lv2, mv2, cp, stamp_ns):
    ok = True
    if lv2:
        ok &= lv2.get('frame') == cp['v2_local_frame'] and lv2.get('stamp_ns') == stamp_ns and bool(lv2.get('prov'))
    if mv2:
        ok &= mv2.get('frame') == cp['v2_map_frame'] and mv2.get('stamp_ns') == stamp_ns and bool(mv2.get('prov'))
    return ok


def _motion(lv2, mv2, cp):
    allowed = set(cp.get('motion_valid_false_allowed_states', [0]))
    for m in (lv2, mv2):
        for o in (m or {}).get('objects', []):
            if 'ms' not in o:
                continue
            if o['ms'] not in (0, 1, 2, 3):
                return False
            if not o['mv'] and o['ms'] not in allowed:
                return False
            if cp['derived_fields'].get('dynamic_valid_equals_motion_valid') and o['dv'] != o['mv']:
                return False
            if cp['derived_fields'].get('is_dynamic_equals_state_dynamic') and o['dyn'] != (o['ms'] == 3):
                return False
    return True


def _zero_count(one):
    n = 0
    for key in ('local_v2', 'map_v2', 'last_lidar', 'tracked_map'):
        for o in (one.get(key) or {}).get('objects', []):
            if all(abs(c) < 1e-9 for c in o.get('c', [1])):
                n += 1
    return n


def _no_zero(one):
    return _zero_count(one) == 0


def _warmup(one, mv2):
    """A track that is not motion_valid in map_v2 of this scan must not be on the legacy wire of this scan."""
    if not mv2:
        return True
    not_ready = {o['id'] for o in mv2.get('objects', []) if 'mv' in o and not o['mv']}
    legacy_ids = {o['id'] for t in ('last_lidar', 'tracked_map') for o in (one.get(t) or {}).get('objects', [])}
    return not (legacy_ids & not_ready)


def _udp_warmup(datagrams, mv2):
    if not datagrams:
        return True
    if not mv2:
        return False
    not_ready = {o['id'] for o in mv2.get('objects', []) if 'mv' in o and not o['mv']}
    return not any(s['id'] in not_ready for u in datagrams for s in u.get('slots', []))


def _trace_ok(tr, lv2, mv2, legacy, exp):
    if tr is None:
        return True          # trace not flushed or scan not processed: informational (fusion_trace_scans)
    m = tr.get('v2_mode')
    if m == 'local_and_map' and not (mv2 and lv2):
        return False
    if m == 'local' and not (lv2 and not mv2):
        return False
    if m == 'none' and (lv2 or mv2):
        return False
    if bool(tr.get('legacy_publish')) != bool(legacy):
        return False
    return True


def _production_checks(one, ob):
    """KNOWN_GAP characterization only (common geometry OFF): no V2 topic must exist; nothing is asserted about
    warm-up exposure (orchestrator decision 5)."""
    return {'no_v2_in_production': not one.get('local_v2') and not one.get('map_v2')}


def _brief(key, r):
    keep = ('status', 'reason', 'n', 'frame', 'actions', 'message', 'points', 'level')
    b = {x: r[x] for x in keep if x in r}
    if key == 'diag':
        b['values'] = {x: r.get('values', {}).get(x) for x in ('v2_output', 'v2_reason', 'gps_state', 'legacy_output',
                                                               'map_reason', 'leveling_store_count', 'leveling_source')}
    if key in ('local_v2', 'map_v2') and r.get('objects') and 'ms' in r['objects'][0]:
        b['motion_states'] = sorted({(o['ms'], o['mv']) for o in r['objects']})
    if key in ('last_lidar', 'tracked_map'):
        b['ids_kph'] = [(o.get('id'), o.get('kph')) for o in r.get('objects', [])]
    if key == 'motion_debug':
        b['ids_state'] = [(o.get('id'), o.get('state')) for o in r.get('objects', [])]
    return b


def _geometry(sc, truth, lv2, mv2):
    g = sc.get('geometry_checks')
    if not g or not truth:
        return None
    res = []
    for frame, msg, key, ykey in (('local_v2', lv2, 'base', 'yaw_base'), ('map_v2', mv2, 'map', 'yaw_map')):
        if frame not in g['frames'] or not msg or msg.get('status') not in DATA:
            continue
        for oid, t in truth.items():
            best = min(msg.get('objects', []), key=lambda o: math.dist(o['c'][:2], t[key][:2]), default=None)
            if best is None:
                res.append(dict(frame=frame, obj=oid, ok=False, why='missing'))
                continue
            d = math.dist(best['c'][:2], t[key][:2])
            mod = math.radians(g.get('yaw_modulo_deg', 90.0))
            dy = abs(math.degrees(math.remainder(best['yaw'] - t[ykey], mod)))
            res.append(dict(frame=frame, obj=oid, track=best['id'], center_err_m=round(d, 3), yaw_err_deg=round(dy, 2),
                            ok=d <= g['center_tol_m'] and dy <= g['yaw_tol_deg']))
    return res


if __name__ == '__main__':
    main()
