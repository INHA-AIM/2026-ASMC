#!/usr/bin/env python3
"""Join drive provenance with same-input masks; suspect planes are not ceiling GT."""
import argparse
from collections import Counter, defaultdict
import gzip
import itertools
import json
from pathlib import Path

import numpy as np


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ['capture','input','result','score','output']:p.add_argument(name,type=Path)
    a=p.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    meta=json.loads((a.capture/'manifest.json').read_text())
    if not meta['complete'] or not meta['restored']:raise ValueError('capture not accepted')
    samples=meta['samples'];times=np.array([s['wall_s'] for s in samples])
    stage_counts=defaultdict(Counter);classes=defaultdict(lambda:defaultdict(lambda:defaultdict(Counter)))
    planes=[];attitude=[];previous=None;asphalt_causes=Counter();linefit_events=Counter()
    with (a.input/'snapshots.jsonl').open() as source,a.result.open() as results, \
            gzip.open(a.score/'frames.jsonl.gz','rt') as scores:
        for entries in itertools.zip_longest(source,results,scores):
            if None in entries:raise ValueError('result length mismatch')
            src,row,score=map(json.loads,entries)
            if not src['input_id']==row['input_id']==score['input_id']:raise ValueError('identity mismatch')
            t=src['scan_arrival_ns']/1e9
            si=max(0,min(len(samples)-1,int(np.searchsorted(times,t,side='right')-1)))
            link=samples[si]['map_link']
            phase='tunnel' if link.endswith('000126') else 'recovery' if link.endswith('000128') else 'approach' if 'tunnel' in a.capture.name else 'road'
            stage_counts[phase]['scans']+=1
            with np.load(a.input/'labels'/(src['input_id']+'.npz')) as labels:
                asphalt=labels['semantic']==127
            for method,bit,name in [('patchwork',2048,'guard_resurrected'),
                    ('linefit',4,'no_own_line'),('linefit',8,'no_neighbor_line'),('linefit',16,'distance')]:
                method_row=row['methods'][method]
                retained=asphalt & (np.asarray(method_row['mask'])==2)
                asphalt_causes[method+'/'+name]+=int(np.count_nonzero(retained &
                    ((np.asarray(method_row['reasons']) & bit)!=0)))
            linefit_events.update(row['methods']['linefit']['events'])
            for method in ['jaeho_rp','patchwork_rp','linefit_rp']:
                for label,data in score['methods'][method]['classes'].items():classes[phase][method][label].update(data)
            selected=[v for v in row['methods']['jaeho']['plane_attempts'] if v['accepted_ground'] and v['height_valid']]
            height=selected[-1]['height_at_origin_m'] if selected else None
            stage_counts[phase]['no_selected_plane']+=int(height is None)
            stage_counts[phase]['positive_origin_plane']+=int(height is not None and height>0)
            stage_counts[phase]['above_minus1m_plane']+=int(height is not None and height>-1)
            if height is not None:
                if previous is not None and t-previous[0]<.3 and abs(height-previous[1])>.5:
                    stage_counts[phase]['adjacent_height_jump_over_0p5m']+=1
                previous=(t,height)
            else:previous=None
            planes.append({'input_id':src['input_id'],'elapsed_s':t-meta['measurement_start_wall_s'],
                'phase':phase,'height_m':height,'selected_attempt':selected[-1] if selected else None})
            attitude.append([t-meta['measurement_start_wall_s'],np.degrees(src['roll']),np.degrees(src['pitch'])])
    report={'stages':stage_counts,'stage_classes':classes,'planes':planes,
        'asphalt_retained_causes_overlapping':asphalt_causes,'linefit_events':linefit_events,
        'limits':['Plane-height jumps are diagnostics, not labelled roof errors.',
                  'Map link at preceding RPC sample is used only for evaluation stage grouping.',
                  'Offline starts at first admitted scan; live included pre-measurement warmup, so not exact adaptive-state replay.']}
    a.output.write_text(json.dumps(report,indent=2,sort_keys=True)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axs=plt.subplots(3,1,figsize=(11,9),constrained_layout=True)
    x=[s['state']['transform']['location']['x'] for s in samples]
    y=[s['state']['transform']['location']['y'] for s in samples]
    axs[0].plot(x,y);axs[0].set(xlabel='map x [m]',ylabel='map y [m]',title=a.capture.name);axs[0].axis('equal')
    att=np.asarray(attitude)
    axs[1].plot(att[:,0],att[:,1],label='roll');axs[1].plot(att[:,0],att[:,2],label='pitch');axs[1].legend();axs[1].set_ylabel('IMU attitude [deg]')
    axs[2].plot([v['elapsed_s'] for v in planes],[v['height_m'] if v['height_m'] is not None else np.nan for v in planes],'.',ms=2)
    axs[2].set(xlabel='receive wall time [s]',ylabel='selected plane z at sensor origin [m]')
    for ax in axs:ax.grid(alpha=.2)
    fig.savefig(a.output.with_suffix('.png'),dpi=140)
    print(json.dumps(stage_counts,indent=2))


if __name__=='__main__':main()
