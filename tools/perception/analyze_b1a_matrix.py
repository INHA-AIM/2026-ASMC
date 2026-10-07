#!/usr/bin/env python3
"""Account for B1a activation and point removal without inventing real-world GT."""
import argparse
from collections import Counter
import json
from pathlib import Path
import math

from b0_measurements import quantiles


def offline(rows):
    active=[r for r in rows if r['B']['active']]
    sums=[sum(r['B']['counts'][i] for r in active) for i in range(7)] if active else None
    return dict(frames=len(rows),active_frames=len(active),
        denied_frames=sum(r['denied'] for r in rows),
        stage_counts_sum=(dict(zip(['input_voxel','ceiling_removed','ground_removed','wall_removed','kept','clusters','protected'],sums)) if sums else None),
        stage_accounting_failures=[r['stamp'] for r in active if r['B']['counts'][0] != sum(r['B']['counts'][1:5])],
        normal_path_raw_roi_voxel_sum={key:sum(r['A'][key] for r in rows) for key in ('raw','roi','voxel','ransac')},
        protection_available_frames=sum(r['B']['protect'][0] for r in active),
        unavailable_protection_frames=sum(not r['B']['protect'][0] for r in active),
        protection_offset_ms=quantiles([r['B']['protect'][1] for r in active if r['B']['protect'][0]]),
        ground_source_counts=dict(Counter(r['B']['ground'][0] for r in active)),
        baseline_objects=sum(len(r['A']['objs']) for r in rows),
        counts_are_not_GT_accuracy=True)


def realtime(observation, report, config, start):
    window=config['bags'][report['job']['bag_id']]['injection_window_sec']
    result={}
    def phase(stamp):
        t=stamp/1e9-start
        return 'before' if t < window[0] else 'window' if t < window[1] else 'after'
    for p in ('before','window','after'):
        result[p]=dict(inputs={n:sum(phase(r['stamp_ns'])==p for r in rows) for n,rows in observation['inputs'].items()},
            publications={n:sum(phase(r['stamp_ns'])==p for r in rows) for n,rows in observation['outputs'].items()},
            nonempty_publications={n:sum(phase(r['stamp_ns'])==p and bool(r.get('objects')) for r in rows)
                                  for n,rows in observation['outputs'].items() if n!='processed_roi'},
            active_diagnostics=sum(phase(r['stamp_ns'])==p and r['values'].get('tunnel_active')=='true' for r in observation['diagnostics']))
    diag=[d for d in observation['diagnostics'] if d['values'].get('tunnel_active')=='true']
    normal_gps=[r for r in observation['inputs'].get('gps',[]) if r['status']!=0 and (r['latitude'] or r['longitude'] or r['altitude'])]
    recovery=[r for r in normal_gps if r['stamp_ns']/1e9 >= start+window[1]]
    return dict(phases=result,first_three_restored_gps_stamps_ns=[r['stamp_ns'] for r in recovery[:3]],
        active_first_source_sec=(min(d['stamp_ns'] for d in diag)/1e9-start if diag else None),
        active_last_source_sec=(max(d['stamp_ns'] for d in diag)/1e9-start if diag else None),
        time_frame_total_ms=quantiles([float(d['values']['time_frame_total_ms']) for d in diag]),
        removed_ceiling_points_sum=sum(int(d['values']['ceiling_removed_count']) for d in diag),
        removed_wall_points_sum=sum(int(d['values']['wall_left_removed'])+int(d['values']['wall_right_removed']) for d in diag),
        wall_validity={side:sum(d['values'].get('wall_'+side+'_removal_applied')=='true' for d in diag) for side in ('left','right')},
        diagnostics_are_periodic_or_active_frame_records_not_GT=True,
        world_pose_validity_not_expressed_by_legacy_message=True)


def summarize(root):
    manifest=json.loads((root/'manifest.json').read_text()); config=manifest['config']
    rows=json.loads((root/'index.json').read_text())
    trials=[]
    for entry in rows:
        path=root/entry['label']; report_path=path/'report.json'
        item=dict(index=entry)
        if report_path.exists():
            report=json.loads(report_path.read_text()); item['report']=report
            if entry['job']['phase']=='offline':
                frames_path=path/'frames.jsonl'
                frames=[json.loads(s) for s in frames_path.read_text().splitlines()] if frames_path.exists() else []
                item['analysis']=offline(frames)
            else:
                observation=json.loads((path/'observations.json').read_text())
                start=manifest['fixtures']['bags'][entry['job']['bag_id']]['source_start_sec']
                item['analysis']=realtime(observation,report,config,start)
        trials.append(item)
    return dict(trials=trials,completed=sum(r.get('complete',False) for r in rows),attempted=len(rows),
        expected_offline=8,expected_realtime=36,real_GT_present=False,operational_adoption='none',
        full_stack_performance_accepted=False)


def main():
    parser=argparse.ArgumentParser(description=__doc__); parser.add_argument('root',type=Path)
    args=parser.parse_args(); data=summarize(args.root)
    (args.root/'analysis.json').write_text(json.dumps(data,indent=2)+'\n')
    print(json.dumps({k:v for k,v in data.items() if k!='trials'}))


if __name__=='__main__': main()
