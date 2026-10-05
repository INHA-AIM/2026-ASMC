#!/usr/bin/env python3
"""Correct only the initial generated timing label; retain each original summary."""
import argparse
import json
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('results',type=Path);a=p.parse_args()
for rel in ('sync_metrics/summary.json','raw_metrics/summary.json','manual_summary.json'):
    path=a.results/rel;data=json.loads(path.read_text());backup=path.with_name(path.stem+'_original_timing_label.json')
    if backup.exists():raise FileExistsError(backup)
    if 'timing_ms' in data:
        for times in data['timing_ms'].values():times['accounted_stage_sum_ms']=times.pop('total_core_ms')
    else:data['accounted_stage_sum_ms']=data.pop('total_core_ms')
    data['timing_definition']='accounted_stage_sum_ms excludes untimed bookkeeping; live compute_ms/receipt_to_publish_ms are measured wall times'
    backup.write_bytes(path.read_bytes());path.write_text(json.dumps(data,indent=2,sort_keys=True)+'\n')
    print('retained original and corrected '+rel)
