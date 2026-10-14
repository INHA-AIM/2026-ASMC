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


def parity(root):
    variants = {}
    for variant in ('R0','R1','R2'):
        folder = root/('complete-'+variant)
        sources = json.loads((folder/'replay-sources.json').read_text())
        boxes, lanes, traffic = defaultdict(list), defaultdict(list), defaultdict(list)
        for r in json.loads((folder/'boxes.json').read_text()):
            key = str(r['stamp'])+':'+r['frame']
            if r['valid'] and key in sources:
                boxes[(r['frame'], sources[key])].append(r['boxes'])
        for path in folder.glob('*.jsonl'):
            for r in rows(path):
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
    for variant in ('R1','R2'):
        checks = {}
        for kind in ('boxes','lanes','traffic'):
            ref, candidate = variants['R0'][kind], variants[variant][kind]
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
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=True)
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


if __name__ == '__main__':
    main()
