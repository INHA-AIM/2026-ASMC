#!/usr/bin/env python3
"""Time-aligned slowdown evidence, not a certificate that braking is unnecessary.

Only already-received, fresh ROS records are joined to independent simulator
samples. Signal waits and protected maneuvers are kept separate from review
candidates. No driving policy or safety limit is changed by this analyzer.
"""
import argparse
import bisect
import collections
import csv
import json
from pathlib import Path

from analyze_speed_trial import measure


REVIEW_KINDS = {
    'signal_perception_review', 'avoid_blocked', 'planner_hold',
    'keep_profile_review', 'tracking_gap_review', 'jamming_input_review',
    'jamming_geometry_review',
}


class Timeline:
    def __init__(self, rows):
        self.rows = collections.defaultdict(list)
        for row in sorted(rows, key=lambda r: r['monotonic']):
            self.rows[row['topic']].append(row)
        self.times = {k: [r['monotonic'] for r in v] for k, v in self.rows.items()}

    def at(self, topic, now, maximum_age=.35):
        index = bisect.bisect_right(self.times.get(topic, []), now) - 1
        if index < 0:
            return {}
        row = self.rows[topic][index]
        if maximum_age is not None and now - row['monotonic'] > maximum_age:
            return {}
        return row


def classify(actual, command, feedback, feature, mode, jamming, green_colors, acceleration):
    """Return an evidence category and detail, never a permission to accelerate."""
    if mode.get('mode') == 1:
        statuses = jamming.get('statuses', [])
        if not statuses:
            return 'missing_diagnostics', 'jamming diagnostic absent/stale'
        status = statuses[0]
        detail = status['message']
        if any(k in detail for k in ('expired', 'stale', 'timeout', 'missing', 'invalid', 'source_age')):
            return 'jamming_input_review', detail
        if 'outside_road' in detail:
            return 'jamming_geometry_review', detail
        if any(k in detail for k in ('obstacle', 'blocked', 'confirm', 'clear')):
            return 'jamming_obstacle', detail
        return 'jamming_lane_follow', detail
    if not command or not feature:
        return 'missing_diagnostics', 'command/feature absent/stale'
    maneuver = command['maneuver_type']
    detail = feedback.get('result_detail', '')
    if feature.get('pedestrian_risk'):
        return 'pedestrian_safety', feature.get('hard_rule_reason', '')
    if maneuver == 5:
        light = feature.get('traffic_light_id')
        aspect = actual.get('aspects', {}).get(light)
        signal_stop = 'traffic_light' in feature.get('hard_rule_reason', '')
        if signal_stop and aspect is not None and light in green_colors:
            if aspect == green_colors[light]:
                return 'signal_perception_review', 'actual permitted aspect; STOP still requested'
            return 'signal_wait', 'actual non-permitted aspect'
        return 'other_planned_stop', feature.get('hard_rule_reason', '') or detail
    if maneuver == 6:
        return 'emergency_safety', feature.get('hard_rule_reason', '') or detail
    if feedback and not feedback.get('plan_valid') and feedback.get('trajectory_source') == 4:
        return ('avoid_blocked' if maneuver == 4 else 'planner_hold'), detail
    if maneuver == 4:
        return 'obstacle_avoidance', detail
    if maneuver == 2:
        return 'merge_wait' if actual['speed_kmh'] < 1 else 'merge_driving', detail
    if maneuver == 1 or feature.get('has_front_vehicle'):
        return 'following_vehicle', detail
    if feature.get('intersection_approach_active'):
        return 'signal_approach_cap', feature.get('traffic_light_id', '')
    desired = command.get('desired_speed_mps', 0) * 3.6
    if actual['speed_kmh'] < desired - 10 and acceleration > .3:
        return 'accelerating', 'speed recovering toward request'
    applied = feedback.get('applied_speed_mps', -1) * 3.6
    if maneuver == 0 and desired >= 40 and actual['speed_kmh'] < desired - 10:
        if applied >= actual['speed_kmh'] + 8:
            return 'tracking_gap_review', 'actual speed below fresh planner target; cause unproven'
        if applied >= 0:
            return 'keep_profile_review', 'curve/road/longitudinal profile may explain lower target'
    return 'ordinary_driving', detail


def read_rows(path):
    with path.open() as stream:
        return [json.loads(line) for line in stream if line.strip()]


def analyze(trial, green_colors):
    oracle = read_rows(trial / 'oracle.jsonl')
    timeline = Timeline(read_rows(trial / 'ros.jsonl'))
    result = json.loads((trial / 'result.json').read_text()) if (trial / 'result.json').exists() else {}
    if len(oracle) < 2:
        summary = dict(trial=trial.name, samples=len(oracle), error=result.get('error'),
                       no_driving_samples=True, scope='driving_not_started', passed=False)
        (trial / 'slowdown-summary.json').write_text(json.dumps(summary, indent=2))
        return summary
    bins = {}; totals = collections.Counter(); stops = []; active_stop = None; rows = []
    times = [r['monotonic'] for r in oracle]
    for i, (a, b) in enumerate(zip(oracle, oracle[1:])):
        now = a['monotonic']; dt = b['monotonic'] - now
        if dt <= 0 or dt > .3 or not a['observation_complete'] or not b['observation_complete']:
            if active_stop:
                stops.append(active_stop); active_stop = None
            continue
        old_index = max(0, bisect.bisect_right(times, now - .5) - 1)
        old = oracle[old_index]; age = now - old['monotonic']
        acceleration = (a['speed_kmh'] - old['speed_kmh']) / 3.6 / age if age >= .2 else 0.
        command = timeline.at('command', now, .6)
        feedback = timeline.at('feedback', now)
        # A different command's feedback cannot explain this request.
        if feedback.get('command_id') != command.get('command_id'):
            feedback = {}
        feature = timeline.at('features', now)
        mode = timeline.at('mode', now, None)
        kind, detail = classify(a, command, feedback, feature, mode,
                                timeline.at('jamming', now), green_colors, acceleration)
        station = (a['station'] + b['station']) / 2
        key = int(station // 100) * 100
        bucket = bins.setdefault(key, dict(elapsed_s=0., distance_m=0., stopped_s=0.,
                                          causes_s=collections.Counter(), details_s=collections.Counter()))
        bucket['elapsed_s'] += dt
        bucket['distance_m'] += (a['speed_kmh'] + b['speed_kmh']) / 7.2 * dt
        bucket['causes_s'][kind] += dt
        bucket['details_s'][kind + ': ' + detail] += dt
        totals[kind] += dt
        stopped = max(a['speed_kmh'], b['speed_kmh']) < .36
        if stopped:
            bucket['stopped_s'] += dt
            if active_stop is None:
                active_stop = dict(start_s=now-times[0], end_s=now-times[0], station=station,
                                   duration_s=0., causes_s=collections.Counter())
            active_stop['duration_s'] += dt
            active_stop['end_s'] = b['monotonic'] - times[0]
            active_stop['causes_s'][kind] += dt
        elif active_stop:
            stops.append(active_stop); active_stop = None
        rows.append(dict(elapsed_s=now-times[0], ros_time=a['ros_time'], station=station,
                         speed_kmh=a['speed_kmh'], acceleration_mps2=acceleration, cause=kind, detail=detail,
                         desired_kmh=command.get('desired_speed_mps', -1)*3.6,
                         applied_kmh=feedback.get('applied_speed_mps', -1)*3.6,
                         brake=timeline.at('control', now).get('brake'),
                         light_id=feature.get('traffic_light_id'), actual_aspect=a.get('aspects', {}).get(feature.get('traffic_light_id')),
                         detector_label=feature.get('traffic_light_raw_label'), dt=dt))
    if active_stop:
        stops.append(active_stop)
    for bucket in bins.values():
        bucket['mean_kmh'] = bucket['distance_m'] / bucket['elapsed_s'] * 3.6
        bucket['review_s'] = sum(bucket['causes_s'].get(k, 0) for k in REVIEW_KINDS)
        bucket['top_details'] = bucket.pop('details_s').most_common(5)
    summary = measure(oracle, result.get('completed', False))
    observer_stop = trial / 'observer-stop.json'
    summary.update(trial=trial.name, error=result.get('error'), passed=result.get('passed', False),
                   observer_stop=json.loads(observer_stop.read_text()) if observer_stop.exists() else None,
                   route_returned=bool(result.get('completed') or 'returned to finish without CP14' in str(result.get('error', ''))),
                   maximum_station_m=max(r['station'] for r in oracle), causes_s=totals,
                   stops=[s for s in stops if s['duration_s'] >= .5], bins=bins,
                   review_is_not_proven_unnecessary=True, independent_evaluation=result.get('independent_evaluation'))
    (trial / 'slowdown-summary.json').write_text(json.dumps(summary, indent=2))
    with (trial / 'slowdown-timeline.csv').open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]) if rows else ['elapsed_s'])
        writer.writeheader(); writer.writerows(rows)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('survey', type=Path)
    args = parser.parse_args()
    reports = []
    for trial in sorted(args.survey.glob('run-*-seed*')):
        if not trial.is_dir() or not (trial / 'result.json').exists():
            continue
        cfg = json.loads((trial / 'trial-config.json').read_text())
        green = {b['traffic_light_id']: b['green_color'] for b in cfg['reviewed_bindings']}
        reports.append(analyze(trial, green))
    (args.survey / 'slowdown-comparison.json').write_text(json.dumps(reports, indent=2))
    for r in reports:
        print(json.dumps({k:r.get(k) for k in ('trial','route_returned','elapsed_s',
              'mean_including_stops_kmh','maximum_kmh','stopped_below_036_kmh_s','error','causes_s')}))


if __name__ == '__main__':
    main()
