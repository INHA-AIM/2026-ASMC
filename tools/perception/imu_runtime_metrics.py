"""Join IMU/fit evidence to published results, including window boundaries."""
from collections import Counter, defaultdict


def family_probes(message, stations=(5., 10., 15.)):
    """Small audit samples; never interpolate across INVALID support."""
    result = []
    for boundary in message.boundaries:
        for x in stations:
            for i, (a, b) in enumerate(zip(boundary.polyline, boundary.polyline[1:])):
                if not (boundary.support_state[i] and boundary.support_state[i + 1]): continue
                if a.x <= x <= b.x and b.x > a.x:
                    t = (x - a.x) / (b.x - a.x)
                    result.append([boundary.role, x, a.y + t * (b.y - a.y), a.z + t * (b.z - a.z)])
                    break
    return result


def fallback_transitions(trace, families, start, end):
    """Stationary live audit only; neighbouring curves are not geometric GT."""
    policies = {r['source_stamp_ns']: r['mode'] for r in trace if r['event'] == 'family_fit_policy'}
    active = [r for r in families if start <= r['at'] < end]
    transitions = []
    missing = 0
    for a, b in zip(active, active[1:]):
        before, after = policies.get(a['stamp']), policies.get(b['stamp'])
        if before is None or after is None or before == after: continue
        if not a['valid'] or not b['valid'] or not 0 < b['at'] - a['at'] <= .25: continue
        if 'probes' not in a or 'probes' not in b:
            missing += 1
            continue
        pa = {(p[0], p[1]): p[2] for p in a['probes']}
        pb = {(p[0], p[1]): p[2] for p in b['probes']}
        common = sorted(set(pa) & set(pb))
        transitions.append(dict(from_mode=before, to_mode=after, stamp_ns=b['stamp'],
                                samples=[dict(role=k[0], x_m=k[1], lateral_delta_m=abs(pb[k] - pa[k])) for k in common]))
    distances = [r['lateral_delta_m'] for t in transitions for r in t['samples']]
    return dict(transitions=transitions, missing_geometry_pairs=missing,
                max_lateral_delta_m=max(distances) if distances else None,
                note='Adjacent current-result geometry at fixed x; internal consistency, not independent GT or temporal smoothing.')


def imu_application(trace, start, end, source_enabled):
    published = [r for r in trace if r['event'] == 'family_publish_end'
                 and start <= r['monotonic'] < end]
    indexed = defaultdict(list)
    for row in trace:
        if row['event'] in ('imu_projection', 'family_fit_policy'):
            indexed[(row.get('session_id'), row.get('source_stamp_ns'), row['event'])].append(row)
    attitudes, policies, failures = [], [], []
    for result in published:
        key = (result.get('session_id'), result.get('source_stamp_ns'))
        required = ('imu_projection', 'family_fit_policy') if source_enabled else ('family_fit_policy',)
        matched = {}
        for event in required:
            rows = [r for r in indexed[key + (event,)] if r['monotonic'] <= result['monotonic']]
            if len(rows) != 1:
                failures.append(dict(session_id=key[0], stamp_ns=key[1], event=event, matches=len(rows)))
            else:
                matched[event] = rows[0]
        if 'imu_projection' in matched: attitudes.append(matched['imu_projection'])
        if 'family_fit_policy' in matched: policies.append(matched['family_fit_policy'])
    return dict(aggregation='published_session_stamp_join_v1', frames=len(published),
                attitude_matches=len(attitudes), policy_matches=len(policies),
                corrected=sum(bool(r.get('used')) for r in attitudes),
                reasons=dict(Counter(r.get('reason', 'unknown') for r in attitudes)),
                policy_modes=dict(Counter(r.get('mode', 'unknown') for r in policies)),
                joint_frames=sum(bool(r.get('joint_width_fit')) for r in policies),
                unmatched=failures)
