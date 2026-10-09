#!/usr/bin/env python3
"""Separate camera reuse from new temporal evidence in complete Main traces."""
import argparse
from collections import Counter
import json
from pathlib import Path


def reuse_report(rows, policy):
    uses=[];empties=[];observed=set();vision=set();duplicates=[];violations=[]
    current=None;invalidations=[];finals=[];inputs={}
    for row in rows:
        event=row['event']
        if event=='yolo_input':
            inputs[row['stamp_ns']]=row['boxes']
        elif event=='yolo_admitted' and row['stamp_ns'] in inputs and not inputs[row['stamp_ns']]:
            empties.append(row['stamp_ns'])
        elif event=='camera_epoch_invalidated':
            invalidations.append(row);current=None;empties=[]
        elif event=='camera_observation_use':
            current=row
            key=(row['camera_id'],row['epoch'],row['calibration_id'],row['source_stamp_ns'])
            already=key in vision
            if already!=row['reused']:
                violations.append(dict(reason='fresh/reused flag disagrees with distinct source observations',row=row))
            vision.add(key);uses.append(row)
            barrier=max(empties,default=0)
            if (policy=='bounded_reuse' and row['source_stamp_ns']<barrier and
                    inputs.get(row['source_stamp_ns']) != []):
                violations.append(dict(reason='older nonempty observation resurrected after empty result',row=row,barrier=barrier))
        elif event in ('class_observation','class_miss'):
            if current is None or row['stamp_ns']!=current['source_stamp_ns']:
                violations.append(dict(reason='class update lacks the matching camera observation',row=row))
                continue
            key=(current['camera_id'],current['epoch'],current['calibration_id'],row['stamp_ns'],row['id'])
            if key in observed:
                duplicates.append(dict(event=event,key=key))
            observed.add(key)
        elif event=='fusion_summary':
            finals.append(row)
    counts=Counter('reused' if r['reused'] else 'fresh' for r in uses)
    complete=len(finals)==1
    conserved=complete and finals[0]['matched']==len(uses) and all(
        finals[0].get(k)==counts[k] for k in ('fresh','reused'))
    return dict(complete=complete,accounting_conserved=conserved,
        fresh_matches=counts['fresh'],reused_matches=counts['reused'],
        distinct_used_camera_observations=len(vision),
        duplicate_temporal_class_evidence=duplicates,policy_violations=violations,
        invalidations=len(invalidations),discarded_pending_at_invalidation=sum(r['discarded_pending_lidar'] for r in invalidations),
        passed=complete and conserved and not duplicates and not violations,
        scope='recorded geometry association and classification state only; no independent object recall claim')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--trace',type=Path,required=True)
    p.add_argument('--policy',choices=['unique','bounded_reuse'],required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    # Diagnostic traces also contain point clouds. Retain only the small
    # accounting state rather than every decoded scan in memory at once.
    with a.trace.open() as stream:
        report=reuse_report((json.loads(line) for line in stream),a.policy)
    a.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report));return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
