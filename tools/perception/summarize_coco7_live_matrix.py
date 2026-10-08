#!/usr/bin/env python3
"""Summarize complete repeat groups without discarding failures or mixing revisions."""
import argparse
from collections import defaultdict
import json
from pathlib import Path


def summarize(root):
    groups=defaultdict(list)
    rejected=[]
    for parent in sorted(root.glob('live-*')):
        if not parent.is_dir(): continue
        if (parent/'REJECTED_PROVENANCE.json').exists():
            rejected.append(str(parent.relative_to(root))); continue
        for file in sorted(parent.glob('*/report.json')):
            report=json.loads(file.read_text()); job=report['job']
            key=(parent.name,job['model'],job['width'],job.get('confidence',.7),
                 job['rates'],job['batch'],job['profile'],job.get('full_stack',False),job.get('debug',False))
            groups[key].append((file,report))
    rows=[]
    for key,trials in groups.items():
        repeats=[r['job']['repeat'] for _,r in trials]
        row=dict(zip(('experiment','model','width','confidence','rates','batch','profile','full_stack','debug'),key))
        row.update(repeats=repeats,complete_three_repeats=sorted(repeats)==[1,2,3],
                   all_passed=sorted(repeats)==[1,2,3] and all(r['passed'] for _,r in trials),
                   failures=[dict(path=str(f.relative_to(root)),failures=r['failures']) for f,r in trials if not r['passed']],
                   heads=sorted({r.get('provenance',{}).get('head','unrecorded') for _,r in trials}))
        row['views']={}
        for view in ('front','left','right'):
            values=[r.get('measurements',{}).get('yolo_'+view,{}) for _,r in trials]
            row['views'][view]=dict(hz=[v.get('hz') for v in values],
                p95_wall_ms=[v.get('input_to_output_wall_ms',{}).get('p95') for v in values],
                maximum_gap_seconds=[v.get('max_gap_sec') for v in values],
                target_met=[r.get('target_attainment',{}).get(view,{}).get('met') for _,r in trials])
        row['peak_vram_mib']=[r.get('resources_startup_and_active',r.get('resources',{})).get('gpu_memory_mib',{}).get('maximum') for _,r in trials]
        row['resource_scopes']=[r.get('resource_scope','ACTIVE only; startup unmeasured') for _,r in trials]
        rows.append(row)
    return dict(groups=rows,rejected_provenance=rejected,
                scope='Each experiment remains separate. All three original repeats must pass; partial groups and missing measurements are not accepted.')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();report=summarize(a.root)
    a.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(groups=len(report['groups']),complete_pass_groups=sum(r['all_passed'] for r in report['groups']))))


if __name__=='__main__': main()
