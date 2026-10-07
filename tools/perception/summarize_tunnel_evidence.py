#!/usr/bin/env python3
"""Summarize every fixed B1a repetition and retain unresolved operating limits."""
import argparse
import json
from pathlib import Path
from collections import defaultdict
from b0_measurements import quantiles
from b1a_protocol import jobs


def protocol_closed(entries, config):
    def key(job):return tuple(job[k] for k in ('phase','bag_id','profile','gps_mode','repeat'))
    expected=[key(j) for phase in ('offline','realtime') for j in jobs(config,phase)]
    actual=[key(e['job']) for e in entries]
    return len(actual)==len(expected) and set(actual)==set(expected) and all(e.get('complete',False) for e in entries)


def summarize(protocol):
    analysis=json.loads((protocol/'analysis.json').read_text())
    manifest=json.loads((protocol/'manifest.json').read_text())
    groups=defaultdict(list);offline={};failures=[]
    for trial in analysis['trials']:
        entry=trial['index'];job=entry['job']
        if not entry.get('complete'):failures.append(entry)
        if 'report' not in trial:continue
        report=trial['report'];detail=trial['analysis']
        if job['phase']=='offline':
            rows=[json.loads(s) for s in (protocol/entry['label']/'frames.jsonl').read_text().splitlines()]
            objects=[o for r in rows if r['B']['active'] for o in r['B']['objs']]
            detail.update(native_replica_mismatch=report['native']['replica_mismatch'],
                object_candidates=len(objects),removed_obb_candidates=sum(o[17] for o in objects),
                protected_wall_attached_candidates=sum(o[13] and o[18] and not o[17] for o in objects),
                removed_protected_candidates=sum(o[13] and o[17] for o in objects),
                flags_are_not_independent_GT=True)
            offline[job['bag_id']+'-'+job['profile']]=detail
            continue
        observation=json.loads((protocol/entry['label']/'observations.json').read_text())
        raw=observation['inputs']['lidar'];roi=observation['outputs'].get('processed_roi',[])
        anchors={r['stamp_ns']//1000000*1000000:r for r in raw}
        latency=[(r['arrival']-anchors[r['stamp_ns']]['arrival'])*1000 for r in roi if r['stamp_ns'] in anchors]
        start=manifest['fixtures']['bags'][job['bag_id']]['source_start_sec']
        window=manifest['config']['bags'][job['bag_id']]['injection_window_sec']
        def injected(row):return window[0]<=row['stamp_ns']/1e9-start<window[1]
        maps=[r for r in observation['outputs'].get('map',[]) if injected(r)]
        centers=[abs(v) for r in maps for o in r['objects'] for v in o['center'][:2]]
        diag=[d for d in observation['diagnostics'] if d['values'].get('tunnel_active')=='true']
        output_metrics={}
        for name in ('map','nogps','last','tunnel'):
            output=observation['outputs'].get(name,[])
            first={}
            for row in output:first.setdefault(row['stamp_ns'],row)
            matched=[r for stamp,r in first.items() if stamp in anchors]
            output_metrics[name]=dict(publications=len(output),unique_source_stamps=len(first),
                duplicate_source_publications=len(output)-len(first),
                backwards_source_stamps=sum(b['stamp_ns']<a['stamp_ns'] for a,b in zip(output,output[1:])),
                unmatched_source_stamps=sorted(set(first)-set(anchors)),
                first_publication_wall_ms=quantiles([(r['arrival']-anchors[r['stamp_ns']]['arrival'])*1000 for r in matched]),
                source_age_ms=quantiles([(r['ros_ns']-r['stamp_ns'])/1e6 for r in matched]),
                nonempty_publications=sum(bool(r['objects']) for r in output))
        groups[job['bag_id']+'-'+job['profile']+'-'+job['gps_mode']].append(dict(
            label=entry['label'],repeat=job['repeat'],input_scans=report['raw_input_frames'],
            processed_scans=len(report['processed_unique_stamps_ns']),
            unprocessed_scans=len(report['unprocessed_input_stamps_ns']),
            duplicate_processed_scans=report['duplicate_processed_stamps'],
            observer_input_complete=report['observer_input_complete'],
            public_control_messages=report['public_control_messages'],
            single_frozen_yolo_publisher=report['single_frozen_yolo_publisher'],
            active_diagnostics=report['active_diagnostics'],phases=detail['phases'],
            active_first_source_sec=detail['active_first_source_sec'],active_last_source_sec=detail['active_last_source_sec'],
            recovery_gps_stamps_ns=detail['first_three_restored_gps_stamps_ns'],
            protected_point_samples=sum(int(d['values']['protected_points']) for d in diag),
            protection_unavailable_samples=sum(d['values']['protection_available']!='true' for d in diag),
            source_to_roi_wall_ms=quantiles(latency),
            source_age_to_roi_ms=quantiles([(r['ros_ns']-r['stamp_ns'])/1e6 for r in roi]),
            output_metrics=output_metrics,
            pipeline_frame_ms=detail['time_frame_total_ms'],
            max_absolute_map_center_during_window_m=max(centers) if centers else None,
            resources=report['resources'],resource_interval_sec=report['resource_interval_sec'],
            all_runtime_errors=[r for r in observation['rosout'] if r['level']>=8]))
    entries=[t['index'] for t in analysis['trials']]
    offline_closed=all(detail['frames']==manifest['fixtures']['bags'][name.split('-')[0]]['variants']['invalid']['topics']['/velodyne_points'] for name,detail in offline.items())
    return dict(expected_trials=44,attempted=analysis['attempted'],completed=analysis['completed'],
        fixed_protocol_complete=protocol_closed(entries,manifest['config']) and len(offline)==8 and offline_closed and sum(len(g) for g in groups.values())==36,
        failures=failures,offline=offline,realtime=dict(groups),
        independent_object_GT=False,operational_adoption='none',
        full_stack_performance_claim=False,roi_wall_latency_includes_observer_cost=True,
        known_limits=['Silent GPS gaps do not activate this branch tunnel mode.',
                     'GPS-invalid map publications do not express pose validity.',
                     'Legacy velodyne includes leveled sensor coordinates, not base_link.',
                     'Point-level ceiling cutoff precedes object protection.',
                     'Retained protected wall-attached candidates require B1b review.',
                     'No independently labeled object recall or collision safety evaluation.'])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('protocol',type=Path);args=parser.parse_args()
    result=summarize(args.protocol)
    (args.protocol/'summary-b1a.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:result[k] for k in ('expected_trials','attempted','completed','fixed_protocol_complete','operational_adoption')}))


if __name__=='__main__':main()
