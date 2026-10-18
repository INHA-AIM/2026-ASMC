#!/usr/bin/env python3
"""G4: merge every integration-matrix run (truth_table.json + udp_audit.json + ns_meta.log + host_before_after.txt +
main_params.yaml + launch_key_lines.txt) into results/TOPIC_TRUTH_TABLE.json. Offline, stdlib only.
usage: 90_collect.py --rt logs/g4/runtime --out results/TOPIC_TRUTH_TABLE.json [--with-rows]
Labels: B0_MEASUREMENT (planner_udp_enabled:=false; must show IPv4/IPv6 UDP out delta 0 and 0 strace UDP sends),
POSITIVE_CONTROL (decision 7: product UDP default inside its own namespace; must show the detectors firing),
KNOWN_GAP_CHARACTERIZATION (production main.launch, common geometry OFF; informational, decision 5),
BAG_SMOKE (informational replay, not real-data validation)."""
import argparse
import glob
import json
import os
import re
from collections import Counter


def read(path, default=''):
    try:
        return open(path, errors='replace').read()
    except OSError:
        return default


def jload(path):
    try:
        return json.load(open(path))
    except (OSError, ValueError):
        return None


def label_of(run):
    if run.startswith('POSITIVE_CONTROL'):
        return 'POSITIVE_CONTROL'
    if run.startswith('KNOWN_GAP'):
        return 'KNOWN_GAP_CHARACTERIZATION'
    if run.startswith('TARGET_'):
        return 'TARGET_CONTRAST'
    if 'BAG_SMOKE' in run:
        return 'BAG_SMOKE'
    return 'B0_MEASUREMENT'


def params(text):
    out = {}
    for key in ('enable_common_geometry', 'lidar_level_roll', 'planner_udp_enabled'):
        m = re.search(r'^%s:\s*(\S+)' % key, text, re.M)
        out[key] = m.group(1) if m else None
    return out


def isolation(d, udp):
    meta = read(os.path.join(d, 'ns_meta.log'))
    host = read(os.path.join(d, 'host_before_after.txt'))
    after = host.split('--- host after', 1)[-1] if '--- host after' in host else ''
    after_body = after.split('\n', 1)[-1].split('listeners:')[0]
    m = re.search(r'interop_probe_rc=(\d+)', meta)
    port = re.search(r'private_master_port=(\d+)', meta)
    uri = re.search(r'ROS_MASTER_URI=http://127\.0\.0\.1:(\d+)', meta)
    teardown = re.search(r'ns processes after teardown: (\d+)', meta)
    return dict(lo_only=udp.get('ns_lo_only'), routes_zero=udp.get('ns_routes_zero'),
                wslinterop_absent=udp.get('wslinterop_absent_in_ns'),
                interop_probe_rc=int(m.group(1)) if m else None,
                interop_blocked=bool(m and m.group(1) != '0'),
                private_master_port=int(port.group(1)) if port else (int(uri.group(1)) if uri else None),
                port_allowed=bool(port and 20000 <= int(port.group(1)) <= 29999 and port.group(1) not in ('11311', '11312', '11400')),
                ns_processes_after_teardown=int(teardown.group(1)) if teardown else None,
                host_after_clean=not re.search(r'^\s*\d+\s+\d+\s', after_body, re.M),
                host_after_excerpt=after_body.strip()[:300],
                forbidden_port_refs=(udp.get('verdict_inputs') or {}).get('forbidden_port_refs'),
                nonloopback_inet=(udp.get('verdict_inputs') or {}).get('nonloopback_inet'),
                suspicious_exec=(udp.get('verdict_inputs') or {}).get('suspicious_exec'),
                ipv6_recorded=(udp.get('snmp') or {}).get('ipv6_recorded'))


def compact_rows(rows):
    out = []
    for r in rows:
        out.append(dict(scan=r['scan'], phase=r['phase'], i=r['i'], stamp_ns=r['stamp_ns'], lidar_mode=r['lidar_mode'],
                        oracle=r['oracle']['mode'], oracle_map_reason=r['oracle'].get('map_reason'),
                        oracle_local_reason=r['oracle'].get('local_reason'), hand_expect=r.get('hand_expect'),
                        observed=r['observed_mode'], counts=r.get('counts', {}),
                        topics={k: v for k, v in r['published'].items() if k in ('local_v2', 'map_v2', 'last_lidar',
                                                                                     'tracked_map', 'motion_debug',
                                                                                     'last_markers', 'map_markers', 'diag',
                                                                                     'local_v1', 'map_v1')},
                        main=r.get('main_trace'), udp=r.get('udp_datagrams') or None, failed_checks=sorted(r['checks'])))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--rt', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--with-rows', action='store_true')
    ap.add_argument('--notes', help='JSON file merged into the bundle as "findings" (evidence-backed notes)')
    a = ap.parse_args()
    runs, per_scenario, totals = [], {}, Counter()
    for tt in sorted(glob.glob(os.path.join(a.rt, '*', 'truth_table.json'))):
        d = os.path.dirname(tt)
        run = os.path.basename(d)
        label = label_of(run)
        t = jload(tt)
        if not t:
            runs.append(dict(run=run, label=label, verdict='ERROR', error=read(os.path.join(d, 'truth_table.err'))[:400]))
            continue
        udp = jload(os.path.join(d, 'udp_audit.json')) or {}
        vi = udp.get('verdict_inputs') or {}
        delta = (udp.get('snmp') or {}).get('delta') or {}
        st = udp.get('strace') or {}
        main_prog = (st.get('per_program_udp') or {}).get('main', {})
        iso = isolation(d, udp)
        prm = params(read(os.path.join(d, 'main_params.yaml')))
        keys = read(os.path.join(d, 'launch_key_lines.txt'))
        sockets = read(os.path.join(d, 'main_sockets.txt')).strip()
        v2_total = re.search(r'\[LIDAR_V2_TOTAL\][^\n]*', keys)
        frame_total = re.search(r'\[LIDAR_FRAME_TOTAL\][^\n]*', keys)
        udp_ev = dict(ipv4_udp_out_delta=delta.get('Udp.OutDatagrams'), ipv6_udp6_out_delta=delta.get('Udp6OutDatagrams'),
                      ipv4_udp_in_delta=delta.get('Udp.InDatagrams'), ipv4_noports_delta=delta.get('Udp.NoPorts'),
                      strace_udp_send_ok=vi.get('strace_udp_send_ok'), strace_udp_send_err=vi.get('strace_udp_send_err'),
                      strace_udp_connect=vi.get('strace_udp_connect'), strace_planner_port_sends=vi.get('strace_planner_port_sends'),
                      strace_main_udp_socket_calls=vi.get('main_udp_socket_calls'),
                      strace_main_udp_send_ok=main_prog.get('udp_send_ok', 0),
                      strace_main_send_dest=main_prog.get('udp_send_dest'),
                      strace_syscall_set='execve,clone,fork,vfork,socket,bind,connect,sendto,sendmsg,sendmmsg,write,writev (-f)',
                      strace_events_parsed=st.get('events_parsed'), main_bound_udp_sockets_procfs=sockets or None,
                      sink_datagrams=(t.get('stats') or {}).get('udp_datagrams_total', 0))
        stats = t.get('stats') or {}
        if label == 'POSITIVE_CONTROL':
            udp_ok = ((udp_ev['ipv4_udp_out_delta'] or 0) > 0 and (udp_ev['strace_planner_port_sends'] or 0) > 0 and
                      udp_ev['strace_main_udp_socket_calls'] == 2 and
                      udp_ev['sink_datagrams'] == stats.get('legacy_published_scans') and
                      udp_ev['ipv6_udp6_out_delta'] == 0)
        else:
            udp_ok = (udp_ev['ipv4_udp_out_delta'] == 0 and udp_ev['ipv6_udp6_out_delta'] == 0 and
                      udp_ev['strace_udp_send_ok'] == 0 and udp_ev['strace_udp_connect'] == 0 and
                      udp_ev['strace_main_udp_socket_calls'] == 1)
        iso_ok = (iso['lo_only'] is True and iso['routes_zero'] is True and iso['wslinterop_absent'] is True and
                  iso['interop_blocked'] and iso['port_allowed'] and iso['host_after_clean'] and
                  iso['forbidden_port_refs'] == 0 and iso['nonloopback_inet'] == 0 and iso['suspicious_exec'] == 0 and
                  iso['ipv6_recorded'] is True)
        if label in ('B0_MEASUREMENT', 'POSITIVE_CONTROL'):
            params_ok = prm == dict(enable_common_geometry='true', lidar_level_roll='true',
                                    planner_udp_enabled='true' if label == 'POSITIVE_CONTROL' else 'false')
        elif label == 'KNOWN_GAP_CHARACTERIZATION':
            params_ok = prm == dict(enable_common_geometry='false', lidar_level_roll='false', planner_udp_enabled='false')
        elif label == 'TARGET_CONTRAST':
            # TARGET ignores ~planner_udp_enabled (param set like the tools); its UDP is off by the copy's setting.yaml.
            params_ok = prm == dict(enable_common_geometry='true', lidar_level_roll='true', planner_udp_enabled='false')
        else:
            params_ok = prm.get('planner_udp_enabled') == 'false'
        verdict = t['verdict']
        if not (udp_ok and iso_ok and params_ok):
            verdict = 'FAIL'
        if label in ('KNOWN_GAP_CHARACTERIZATION', 'TARGET_CONTRAST') and verdict == 'PASS':
            verdict = 'CHARACTERIZED'
        entry = dict(run=run, label=label, scenario=t['scenario'], profile=t.get('profile'), verdict=verdict,
                     truth_table_verdict=t['verdict'], udp_ok=udp_ok, isolation_ok=iso_ok, params_ok=params_ok,
                     main_params=prm, scans=t['scans'], processed_expected=t['processed_expected'],
                     unmatched_stamped=t['unmatched_stamped'], recorder_missing_required=(t.get('recorder_ready') or {}).get('missing_required'),
                     scans_without_any_output=t['scans_without_any_output'],
                     violations={k: len(v) for k, v in t['violations'].items()}, violation_scans=t['violations'],
                     modes=_modes(t['rows']), stats=stats, deleteall_only_events=t.get('deleteall_only_events'),
                     diagnostics_values=t.get('diagnostics_values'), main_trace_modes=t.get('main_trace_modes'),
                     fusion_trace_scans=t.get('fusion_trace_scans'),
                     main_totals=dict(frame=frame_total.group(0) if frame_total else None,
                                      v2=v2_total.group(0) if v2_total else None,
                                      shutdown_lock_error_B15=('lock_error' in keys)),
                     planner_interface_log=[l.split(']: ', 1)[-1][:120] for l in keys.splitlines()
                                            if 'Planner object interface' in l or 'PlannerObjectSender' in l],
                     udp=udp_ev, isolation=iso,
                     rows=compact_rows(t['rows']) if a.with_rows else None)
        runs.append(entry)
        # Repeated runs of one scenario keep the WORST verdict here; every run is listed in runs[].
        rank = {'FAIL': 4, 'ERROR': 4, 'INCONCLUSIVE': 3, 'CHARACTERIZED': 1, 'PASS': 0}
        prev = per_scenario.setdefault(label, {}).get(t['scenario'])
        if prev is None or rank.get(verdict, 4) > rank.get(prev.split(' ')[0], 0):
            per_scenario[label][t['scenario']] = verdict
        n_runs = sum(1 for x in runs if x.get('label') == label and x.get('scenario') == t['scenario'])
        if n_runs > 1:
            fails = sum(1 for x in runs if x.get('label') == label and x.get('scenario') == t['scenario'] and x['verdict'] == 'FAIL')
            per_scenario[label][t['scenario']] = '%s (%d runs, %d FAIL)' % (
                per_scenario[label][t['scenario']].split(' ')[0], n_runs, fails)
        if label == 'B0_MEASUREMENT':
            totals['b0_runs'] += 1
            totals['b0_scans'] += t['scans']
            for k in ('simultaneous_local_map_v2', 'simultaneous_local_v2_and_legacy', 'map_output_while_map_pose_invalid',
                      'dummy_zero_center_objects', 'map_v2_messages', 'local_v2_messages', 'legacy_published_scans',
                      'last_lidar_messages', 'tracked_map_messages', 'v1_dual_publish_scans'):
                totals['b0_' + k] += stats.get(k, 0)
            for k, v in (t.get('deleteall_only_events') or {}).items():
                totals['b0_deleteall_only_events_' + k] += v
            totals['b0_ipv4_udp_out_delta'] += udp_ev['ipv4_udp_out_delta'] or 0
            totals['b0_ipv6_udp6_out_delta'] += udp_ev['ipv6_udp6_out_delta'] or 0
            totals['b0_strace_udp_send_ok'] += udp_ev['strace_udp_send_ok'] or 0
            totals['b0_runs_pass'] += verdict == 'PASS'
            totals['b0_runs_fail'] += verdict == 'FAIL'
            totals['b0_runs_inconclusive'] += verdict == 'INCONCLUSIVE'
            for k, v in entry['violations'].items():
                totals['b0_violation_scans_' + k] += v
        if label == 'TARGET_CONTRAST':
            totals['target_runs'] += 1
            totals['target_scans'] += t['scans']
            for k in ('v1_dual_publish_scans', 'target_legacy_while_oracle_not_map', 'target_v1_local_data_while_oracle_none',
                      'map_v2_messages', 'local_v2_messages', 'legacy_published_scans'):
                totals['target_' + k.replace('target_', '')] += stats.get(k, 0)
    bundle = dict(schema='b0_topic_truth_table_bundle/1+g4', generated_by='tools/b0_tests/90_collect.py',
                  final_outputs=['/lidar/local_object', '/lidar/map_object'],
                  legacy_outputs=['/last_lidar', '/lidar/tracked_objects_map', '/lidar/motion_debug', 'planner UDP 7510'],
                  non_final_recorded=['/perception/lidar/objects_local', '/perception/lidar/objects_map (V1, decision 4)',
                                      '/lidar/object(_boxes)', '/lidar_tracking_markers', '/lidar/debug_*'],
                  none_policy='decision 1: NONE scans publish nothing on V2; reason in diagnostics/trace',
                  per_scenario=per_scenario, totals=dict(totals),
                  findings=(jload(a.notes) if a.notes else None), runs=runs)
    json.dump(bundle, open(a.out, 'w'), indent=1)
    print(json.dumps(dict(per_scenario=per_scenario, totals=dict(totals)), indent=1))


def _modes(rows):
    res = {}
    for r in rows:
        for d, m in r['observed_mode'].items():
            res.setdefault(d, Counter())[m] += 1
        res.setdefault('oracle', Counter())[r['oracle']['mode']] += 1
        tr = r.get('main_trace') or {}
        res.setdefault('main_trace_v2_mode', Counter())[tr.get('v2_mode', 'absent')] += 1
    return {k: dict(v) for k, v in res.items()}


if __name__ == '__main__':
    main()
