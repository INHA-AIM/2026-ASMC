#!/usr/bin/env python3
"""Preserve failed output gates and identify matching upstream camera outages."""
import argparse
import json
from pathlib import Path


def analyze(directory):
    report = json.loads((directory/'report.json').read_text())
    inputs = json.loads((directory/'inputs.json').read_text())
    outputs = json.loads((directory/'outputs.json').read_text())
    requests = {r['model']:r for r in report['requests'] if r['mode']==2}
    result = dict(trial=directory.name, passed=report['passed'], failures=report['failures'], views={})
    for view in ('front','left','right'):
        req = requests.get('yolo')
        if req is None: continue
        start = req['call_started']; end = start+report['job']['seconds']
        raw = [r for r in inputs.get(view,[]) if start <= r['arrival'] < end]
        produced = [r for r in outputs.get('yolo_'+view,[]) if start <= r['arrival'] < end]
        raw_gaps = [(b['arrival']-a['arrival'],a,b) for a,b in zip(raw,raw[1:])]
        out_gaps = [(b['arrival']-a['arrival'],a,b) for a,b in zip(produced,produced[1:])]
        failures = []
        for gap,a,b in out_gaps:
            if gap <= .25: continue
            # An overlap is a diagnostic association, not proof of causality or
            # permission to erase the output failure. Endpoint/initial gaps are
            # already handled by the original gate and remain in its report.
            upstream = [dict(seconds=g,begin=x['arrival'],end=y['arrival']) for g,x,y in raw_gaps
                        if g>.25 and x['arrival']<b['arrival'] and y['arrival']>a['arrival']]
            failures.append(dict(seconds=gap,begin=a['arrival'],end=b['arrival'],
                                 upstream_camera_gaps=upstream))
        result['views'][view] = dict(input_max_gap=max((g for g,_,_ in raw_gaps),default=None),
                                     output_max_gap=max((g for g,_,_ in out_gaps),default=None),
                                     output_gaps_over_250ms=failures)
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args(); rows=[]
    for path in sorted(a.root.glob('*/report.json')):
        if (path.parent/'inputs.json').is_file(): rows.append(analyze(path.parent))
    a.output.write_text(json.dumps(dict(trials=rows,
        interpretation='source delivery and processing failures are separate; original acceptance gates are unchanged'),indent=2)+'\n')


if __name__=='__main__': main()
