#!/usr/bin/env python3
import argparse
import json
import gzip
from pathlib import Path
from lidar_continuous_measurements import compare,speed_events


def read(path, event=None):
    opener=gzip.open if str(path).endswith(".gz") else open
    with opener(path,"rt") as f:
        return [json.loads(x) for x in f if x.strip() and (event is None or ('"event":"'+event+'"') in x[:100])]


def main():
    p=argparse.ArgumentParser();p.add_argument('root',type=Path)
    p.add_argument('--output',type=Path);args=p.parse_args()
    index=json.loads((args.root/'index.json').read_text());manifest=json.loads((args.root/'manifest.json').read_text());result=dict(repeats=[],gps_contrasts=[],speed_diagnostics=[])
    for trial in index:
        label=trial['label'];base=dict(trial)
        rows=read(args.root/(label+'.jsonl'))
        if len(rows)!=manifest['expected_scans'][trial['bag']] or trial['return_code']:
            raise ValueError('incomplete preserved replay '+label)
        if trial['repeat']==2:
            ref=read(args.root/(label[:-1]+'1.jsonl'));base.update(compare(ref,rows));result['repeats'].append(base)
        elif trial['gps_mode']!='normal':
            ref=read(args.root/(f"{trial['bag']}-{trial['profile']}-normal-r1.jsonl"));base.update(compare(ref,rows));result['gps_contrasts'].append(base)
        trace=args.root/(label+'.trace.jsonl.gz')
        if trace.exists():result['speed_diagnostics'].append(dict(label=label,events=speed_events(read(trace,"tracking_state"))))
    result['complete_matrix']=len(index)==20
    result['all_comparisons_pass']=result['complete_matrix'] and all(x['exact'] for kind in ('repeats','gps_contrasts') for x in result[kind])
    result['scope']='actual ground/clustering/tracker plus production fusion; frozen logical arrivals, not projection GT or realtime scheduling proof'
    (args.output or args.root/'analysis.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(pass_=result['all_comparisons_pass'],repeats=len(result['repeats']),gps_contrasts=len(result['gps_contrasts']))))
    return 0 if result['all_comparisons_pass'] else 1

if __name__=='__main__':raise SystemExit(main())
