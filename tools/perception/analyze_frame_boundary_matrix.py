#!/usr/bin/env python3
"""Report all frame-boundary trials without granting operational acceptance."""
import argparse
import json
from pathlib import Path
from frame_boundary_measurements import compare_local, map_gate
from fusion_log_accounting import parse_fusion_observations


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    index=json.loads((args.root/'index.json').read_text())
    observations, reports, errors={}, {}, []
    for trial in index:
        label=trial['label']; path=args.root/label
        try:
            report=json.loads((path/'report.json').read_text())
            observed=json.loads((path/'observations.json').read_text())
            logs=(path/'node.log').read_text(errors='replace')
            rosout=[dict(name=row['name'],message=row['msg']) for row in observed['rosout']]
            intervals,total,complete=parse_fusion_observations(logs,rosout)
            report['fusion']=dict(intervals=intervals,total=total,complete_shutdown_accounting=complete)
            if trial['job']['profile']!='legacy' and trial['job']['subscribers'] in ('map','both'):
                report['map_gate']=map_gate(observed['outputs'].get('typed_map',[]),
                    observed['inputs'].get('gps',[]),observed['inputs'].get('lidar',[]))
            observations[label]=observed;reports[label]=report
        except (OSError,ValueError,KeyError) as error:
            errors.append(dict(label=label,error=str(error)))
    comparisons=[]
    for trial in index:
        job=trial['job']
        if job['suite']!='state' or job['profile']=='legacy' or job['gps_mode']=='normal': continue
        label=trial['label']
        reference=label.replace('-'+job['gps_mode']+'-','-normal-')
        if label not in observations or reference not in observations:
            errors.append(dict(label=label,error='missing GPS contrast trial'));continue
        try:
            result=compare_local(observations[reference]['outputs'].get('typed_local',[]),
                observations[label]['outputs'].get('typed_local',[]))
            comparisons.append(dict(reference=reference,candidate=label,local=result))
        except ValueError as error:
            errors.append(dict(label=label,error=str(error)))
    result=dict(trials=reports,index=index,errors=errors,gps_local_comparisons=comparisons,
        operational_acceptance=False,roll_promotion=False,boundary_promotion=False,
        pending=['deterministic full-pipeline GPS contrast', 'independent object-preservation review',
            'normal map tracking quality comparison','actual GPU inference and subscriber costs',
            'user direct launch review'],
        interpretation='matched counts are frame pairs; ID counts do not prove object continuity; every failure retained')
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(trials=len(reports),errors=len(errors),operational_acceptance=False)))
    return 1 if errors or any(not trial['complete'] for trial in index) else 0


if __name__=='__main__':raise SystemExit(main())
