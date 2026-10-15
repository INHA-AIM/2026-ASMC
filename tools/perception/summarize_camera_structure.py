#!/usr/bin/env python3
"""Summarize retained trials and compare outputs by source image, never ROS seq."""
import argparse
from collections import defaultdict
import csv
import hashlib
import json
from pathlib import Path
from lifecycle_measurements import percentile


def rows(path):
    return [json.loads(s) for s in path.read_text().splitlines() if s]


def parity(root, variant_names=('R0','R1','R2')):
    variants = {}
    for variant in variant_names:
        folder = root/('complete-'+variant)
        sources = json.loads((folder/'replay-sources.json').read_text())
        boxes, lanes, traffic = defaultdict(list), defaultdict(list), defaultdict(list)
        for r in json.loads((folder/'boxes.json').read_text()):
            key = str(r['stamp'])+':'+r['frame']
            if r['valid'] and key in sources:
                boxes[(r['frame'], sources[key])].append(r['boxes'])
        for path in folder.glob('*.jsonl'):
            for r in rows(path):
                if r['event'] != 'output':
                    continue
                key = str(r['stamp'])+':'+r['frame']
                if r['event'] != 'output' or key not in sources or r.get('stale'):
                    continue
                identity = (r['frame'], sources[key])
                if r.get('payload_sha256'):
                    lanes[identity].append(r['payload_sha256'])
                if r.get('classification'):
                    traffic[identity].append(r['classification'])
        variants[variant] = dict(boxes=boxes, lanes=lanes, traffic=traffic)
    result = {}
    for variant in variant_names[1:]:
        checks = {}
        for kind in ('boxes','lanes','traffic'):
            ref, candidate = variants[variant_names[0]][kind], variants[variant][kind]
            common = set(ref)&set(candidate)
            mismatched, max_delta = [], 0.
            for key in common:
                a, b = ref[key][0], candidate[key][0]
                if kind != 'traffic':
                    if a != b or any(v != a for v in ref[key]) or any(v != b for v in candidate[key]):
                        mismatched.append(key)
                else:
                    equal = a['label'] == b['label'] and a['state'] == b['state'] and a['candidate_labels'] == b['candidate_labels']
                    delta = max([abs(a['confidence']-b['confidence'])]+[abs(x-y) for x,y in zip(a['candidate_confidences'],b['candidate_confidences'])])
                    max_delta = max(max_delta,delta)
                    if not equal or delta > .02:
                        mismatched.append(key)
            checks[kind] = dict(common=len(common), mismatched=mismatched, max_confidence_delta=max_delta)
        result[variant] = checks
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--results', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--rate-followup', action='store_true', help='15Hz/10Hz minimum follow-up; preserve original report decisions')
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=True)
    if a.rate_followup:
        return rate_followup(a)
    summaries, pooled = [], defaultdict(lambda:defaultdict(list))
    controls = {}
    csv_rows = []
    for path in sorted(a.results.glob('*/*/report.json')):
        r = json.loads(path.read_text())
        label = path.parent.name
        if r.get('mode') == 'sensor_bridge_only':
            controls[label] = r
            continue
        report = dict(matrix=path.parent.parent.name, trial=label, variant=r['variant'],
            active=r['active'], lane_hz=r['lane_hz'], nms_iou=r.get('nms_iou',.7),
            failures=r['failures'], complete=r.get('complete',False), resources=r.get('resources'),
            streams=r.get('streams',{}), final_status=r.get('final_status',{}))
        if report['resources'] and report['resources']['vram_peak_fraction'] > .9:
            report['failures'].append('VRAM over 90%')
        summaries.append(report)
        for key, stream in report['streams'].items():
            csv_rows.append(dict(matrix=report['matrix'], trial=label,variant=r['variant'],stream=key,
                hz=stream['hz'],target_hz=stream['target_hz'],min_5s_hz=min(stream['five_second_hz'],default=0),
                p50_ms=stream['wall_ms']['p50'],p95_ms=stream['wall_ms']['p95'],p99_ms=stream['wall_ms']['p99'],
                max_gap_ms=stream['max_gap_sec']*1000,duplicates=stream['duplicate_stamps'],
                regressions=stream['reversed_stamps'],target_pass=stream['target_95pct_pass'],
                timing_pass=stream['timing_pass']))
        if not (label.startswith(('sunny-r','foggy-r','final15-','accepted-'))):
            continue
        weather = 'foggy' if 'foggy' in label else 'sunny'
        group = r['variant']+('-final' if label.startswith('accepted-') else '')+'-lane'+str(int(r['lane_hz']))+'-'+weather
        for raw_path in path.parent.glob('*.jsonl'):
            for event in rows(raw_path):
                if event['event']=='output' and event.get('received') is not None and not event.get('stale') and r['start'] <= event['at'] < r['end']:
                    if event['topic']=='/perception/camera/objects' and not event['valid']:
                        continue
                    pooled[group][event['topic']+':'+event['frame']].append(event['wall_ms'])
    aggregates = {g:{key:dict(count=len(values),**{f'p{q}_ms':percentile(values,q) for q in (50,95,99)})
                       for key,values in streams.items()} for g,streams in pooled.items()}
    out = dict(version=1, trials=summaries, sensor_only_controls=controls, pooled_frame_latency=aggregates,
        pooling='Raw per-frame samples across complete runs; linear interpolation. Not a mean of run medians.',
        resource_scope='Whole GPU including MORAI/background; Linux guest system metrics; summed process RSS may share pages.',
        parity=parity(a.results/('postfix-parity' if (a.results/'postfix-parity').exists() else 'complete-parity')),
        supply_blocking_trials=[r['trial'] for r in summaries if r['trial'].startswith('accepted-') and any(not s['timing_pass'] or not s['target_95pct_pass'] for s in r['streams'].values())],
        decisions=dict(runtime='R1 separate processes', lane_hz=10, objects_hz=15, traffic_light_hz=10,
            shared='Rejected: no 10% CPU/p95 gain; other worker latency grows >5ms.',
            lane15='Not promoted: SUNNY repeat2 has a complete 5s window at14.2Hz <14.25Hz.',
            default_confidence=.4,default_nms_iou=.7,validated_nms_candidate=.5,exclusive_names=False,
            scope='Camera producer only. No asmc_lidar/planner cutover or autonomous-driving acceptance.',
            full_live_supply_acceptance=False,
            supply_issue='Two final trials contain upstream input/header gaps. Do not waive failed end-to-end supply gates.'))
    (a.output/'summary.json').write_text(json.dumps(out,indent=2)+'\n')
    with (a.output/'trials.csv').open('w') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(csv_rows[0]));writer.writeheader();writer.writerows(csv_rows)
    source = Path(__file__).resolve().parents[2]
    files = sorted(list((source/'src/asmc_camera').rglob('*.py'))+list((source/'src/asmc_camera').rglob('*.launch'))+
                   list((source/'src/asmc_camera/config').rglob('*.yaml'))+list((source/'src/asmc_msgs/msg').glob('*.msg')))
    hashes = {str(f.relative_to(source)):hashlib.sha256(f.read_bytes()).hexdigest() for f in files if '__pycache__' not in f.parts}
    (a.output/'final-source-hashes.json').write_text(json.dumps(hashes,indent=2)+'\n')
    print(json.dumps(dict(trials=len(summaries),rows=len(csv_rows),parity=out['parity'])))


def rate_followup(a):
    trials, csv_rows, pooled = [], [], defaultdict(lambda: defaultdict(list))
    family_pooled = defaultdict(list)
    for path in sorted(a.results.rglob('report.json')):
        report = json.loads(path.read_text())
        trial = dict(report, trial=path.parent.name, matrix=path.parent.parent.name)
        trial.setdefault('streams', {})
        failures = list(report['failures'])
        for key, stream in report.get('streams', {}).items():
            floor = 9.5 if key.startswith('/perception/traffic_light') else 10.
            minimum = bool(stream['five_second_hz']) and min(stream['five_second_hz']) >= floor
            stream['minimum_supply_pass'] = minimum
            if not minimum or not stream['timing_pass'] or stream['duplicate_stamps'] or stream['reversed_stamps']:
                failures.append('supply/timing: ' + key)
            csv_rows.append(dict(trial=trial['trial'],stream=key,hz=stream['hz'],
                min_5s_hz=min(stream['five_second_hz'],default=0),
                **{f'p{q}_ms':stream['wall_ms'][f'p{q}'] for q in (50,95,99)},
                max_gap_ms=1000*stream['max_gap_sec'],minimum_pass=minimum,target_pass=stream['target_95pct_pass']))
        family = report.get('lane_family', {})
        if not family.get('five_second_hz') or min(family['five_second_hz']) < 10:
            failures.append('lane family below10Hz')
        if family.get('duplicate_stamps') or family.get('reversed_stamps'):
            failures.append('lane family repeated/reversed source')
        if any(s['errors'] or not s['prepared'] or (r=='lane' and not s['pipeline_ready'])
               for r,s in report.get('final_status',{}).items()):
            failures.append('model readiness/errors')
        status_path = path.parent/'statuses.json'
        if status_path.exists() and report.get('start') is not None:
            status_rows = json.loads(status_path.read_text())
            trial['active_status_failures'] = {role:sum(not s['prepared'] or (role=='lane' and not s['pipeline_ready']) or bool(s['errors'])
                for s in status_rows if s['role']==role and report['start']<=s['at']<report['end']) for role in report['active']}
            if any(trial['active_status_failures'].values()):
                failures.append('transient readiness/errors during ACTIVE')
        if report.get('resources') and report['resources']['vram_peak_fraction'] > .9:
            failures.append('whole GPU VRAM over90%')
        trial['minimum_acceptance_failures'] = failures
        snapshots = []
        diag = path.parent/'diagnostics.json'
        if diag.exists() and report.get('start') is not None:
            for event in json.loads(diag.read_text()):
                if not report['start'] <= event['at'] < report['end']: continue
                for status in event['statuses']:
                    if status['name']=='camera_semantic_perception/inference':
                        snapshots.append((event['at'],status['values']))
        if len(snapshots)>1:
            first,last = snapshots[0],snapshots[-1]
            keys=('output_batches','model_grid_output_batches','model_grid_throttled_batches')
            delta={k:int(last[1][k])-int(first[1][k]) for k in keys}
            trial['model_grid_counter_delta']=dict(delta,window_sec=last[0]-first[0])
            trial['stage_ms_cumulative']=json.loads(last[1]['average_stage_ms'])
        if trial['trial'].startswith('live-latest-') and report.get('complete'):
            source_inputs = {}
            for raw in path.parent.glob('*.jsonl'):
                for row in rows(raw):
                    if row['event']=='input' and raw.stem=='camera_lane':
                        source_inputs[row['stamp']] = min(row['at'],source_inputs.get(row['stamp'],row['at']))
                    if row['event']=='output' and not row.get('stale') and row.get('received') is not None and report['start'] <= row['at'] < report['end']:
                        if row['topic']=='/perception/camera/objects' and not row['valid']: continue
                        group='foggy' if 'foggy' in trial['trial'] else 'sunny'
                        pooled[group][row['topic']+':'+row['frame']].append(row['wall_ms'])
            group='foggy' if 'foggy' in trial['trial'] else 'sunny'
            for row in json.loads((path.parent/'lane-families.json').read_text()):
                if report['start'] <= row['at'] < report['end'] and row['stamp'] in source_inputs:
                    family_pooled[group].append(1000*(row['at']-source_inputs[row['stamp']]))
        trials.append(trial)
    aggregates={weather:{key:dict(count=len(v),**{f'p{q}_ms':percentile(v,q) for q in (50,95,99)})
                           for key,v in values.items()} for weather,values in pooled.items()}
    out=dict(title='15Hz 공급·주기 경계 수정·confidence 대조 — 2026-10-15',
        introduction='YOLO/TwinLite 목표15Hz·ResNet10Hz, E8, 동일 클래스 NMS0.50. 목표95% 달성과 최소10Hz 공급은 별도 판정한다. Confidence0.50/0.60은 차량 recall 하락으로 기본 승격하지 않는다. 독립 프로세스와 기존 모델·mask·정책을 유지한다.',
        minimum_supply_hz=10,trials=trials,pooled_frame_latency=aggregates,
        pooled_family_latency={k:dict(count=len(v),**{f'p{q}_ms':percentile(v,q) for q in (50,95,99)}) for k,v in family_pooled.items()},
        latency_definition='Model callback -> publish; family callback-source lookup -> subscriber reception. Family latency is separate from model latency.',
        pooling='Raw per-frame samples across repetitions; not a mean of medians.',
        live_failures={r['trial']:r['minimum_acceptance_failures'] for r in trials if r['trial'].startswith('live-latest-') and r['minimum_acceptance_failures']},
        additional_load_failures={r['trial']:r['minimum_acceptance_failures'] for r in trials if r['trial'].startswith('extra-') and r['minimum_acceptance_failures']},
        full_stack_driving_acceptance=False,
        historical_live_failures={r['trial']:r['minimum_acceptance_failures'] for r in trials if r['trial'].startswith('live-') and not r['trial'].startswith('live-latest-') and r['minimum_acceptance_failures']})
    (a.output/'summary.json').write_text(json.dumps(out,indent=2)+'\n')
    with (a.output/'trials.csv').open('w') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(csv_rows[0]));writer.writeheader();writer.writerows(csv_rows)
    print(json.dumps(dict(trials=len(trials),live_failures=out['live_failures'])))


if __name__ == '__main__':
    main()
