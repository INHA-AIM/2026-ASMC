#!/usr/bin/env python3
"""Summarize full batch trials and qualitative, reviewed target-region observations."""
import argparse
from collections import defaultdict
import hashlib
import json
from pathlib import Path
import statistics


def summarize_batches(report):
    grouped=defaultdict(list)
    for row in report['runs']:
        grouped[(row['model'],row['width'],row['batch'])].append(row)
    output=[]
    for (model,width,batch),runs in sorted(grouped.items()):
        complete=[r for r in runs if 'summary' in r and not r.get('error')]
        repeats=[r['repeat'] for r in runs]
        if len(repeats)!=len(set(repeats)):raise ValueError('duplicate batch repeat')
        entry=dict(model=model,width=width,batch=batch,completed_repeats=len(complete),
                   requested_repeats=len(runs),failures=[r.get('error') for r in runs if r.get('error')])
        if complete:
            entry.update(p50_ms_mean=statistics.mean(r['summary']['p50_ms'] for r in complete),
                p95_ms_max=max(r['summary']['p95_ms'] for r in complete),
                p50_ms_repeats=[r['summary']['p50_ms'] for r in complete],
                mean_predict_ms=statistics.mean(r['summary']['mean_predict_ms'] for r in complete),
                mean_mask_transfer_ms=statistics.mean(r['summary']['mean_mask_transfer_ms'] for r in complete),
                peak_whole_gpu_mib=max(r['summary']['observed_peak_whole_gpu_mib'] for r in complete),
                tensor_pixels=complete[0]['tensor_pixels'])
        output.append(entry)
    return output


def target_observation(predictions, expected, region):
    """Region is a reviewed search area, NOT a target bbox or segmentation GT."""
    x1,y1,x2,y2=region
    labels=[];confidence=[]
    for box in predictions:
        a,b,c,d=box['xyxy'];x,y=(a+c)/2,(b+d)/2
        if x1<=x<=x2 and y1<=y<=y2:
            labels.append(box['label'])
            if box['label']==expected:confidence.append(box['confidence'])
    return dict(expected_label_present=bool(confidence),labels=sorted(set(labels)),
                expected_confidence_max=max(confidence) if confidence else None)


def summarize_ranges(inspection, captures):
    cases={}
    for capture in captures:
        manifest=json.loads((capture/'capture.json').read_text())
        if not manifest.get('ego_after'):raise ValueError('unfinished capture')
        for case in manifest['cases']:
            if case.get('error'):raise ValueError('failed placement cannot be scored')
            cases[(capture.name,case['index'])]=case
    if not cases:raise ValueError('no range cases')
    results=[]
    for candidate in inspection['candidates']:
        profile=candidate['profile_path']
        predictions=[json.loads(line) for line in (profile/'predictions.jsonl').read_text().splitlines()]
        grouped=defaultdict(list)
        for row in predictions:grouped[(Path(row['registry']).name,row['case'])].append(row)
        if set(grouped)!=set(cases):raise ValueError('incomplete capture/model pairing')
        observations=[]
        for key,case in cases.items():
            sources={r['source_stamp_ns']:r for r in case['frames'] if r['view']=='front'}
            rows=grouped[key]
            if len(rows)!=len(sources) or len({r['source_stamp_ns'] for r in rows})!=len(rows):
                raise ValueError('missing/duplicate range source stamp')
            metrics=[]
            for row in rows:
                source=sources[row['source_stamp_ns']]
                if source['sha256']!=row['sha256']:raise ValueError('range RGB SHA mismatch')
                # All 42 original RGB contact-sheet crops were reviewed. This
                # broad central search area contains the stationary spawned
                # actor; it intentionally does not pretend to be a pixel GT box.
                metrics.append(target_observation(row['predictions'],case['asset']['expected_label'],[570,230,685,455]))
            count=sum(m['expected_label_present'] for m in metrics)
            observations.append(dict(weather=key[0],case=key[1],asset=case['asset']['api_name'],
                expected_label=case['asset']['expected_label'],placement=case['placement'],
                requested_forward_anchor_m=case['requested_forward_anchor_m'],
                bbox_center_distance_from_ego_anchor_m=case['bbox_center_distance_from_ego_anchor_m'],
                frames=len(metrics),expected_label_region_frames=count,
                all_frames_expected_label=count==len(metrics),at_least_three_of_five=count>=3 and len(metrics)==5,
                region_labels=sorted({label for m in metrics for label in m['labels']}),
                frame_observations=metrics))
        results.append(dict(config=candidate['config'],observations=observations))
    return results


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--benchmark',type=Path,required=True)
    parser.add_argument('--inspection',type=Path,required=True)
    parser.add_argument('--captures',type=Path,nargs='+',required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():raise ValueError('summary already exists')
    batch=json.loads((args.benchmark/'report.json').read_text())
    inspection=json.loads((args.inspection/'report.json').read_text())
    for c in inspection['candidates']:c['profile_path']=args.inspection/f'{c["config"]["model"]}-{c["config"]["width"]}'
    evidence=[args.benchmark/'report.json',args.inspection/'report.json']+[c/'capture.json' for c in args.captures]
    output=dict(batch_scope=batch['scope'],range_scope='reviewed broad target-region label presence; no pixel GT, recall, AP, or moving approach timing',
        target_review_region_xyxy=[570,230,685,455],batches=summarize_batches(batch),
        ranges=summarize_ranges(inspection,args.captures),
        evidence=[dict(path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in evidence])
    args.output.write_text(json.dumps(output,indent=2)+'\n')


if __name__=='__main__':main()
