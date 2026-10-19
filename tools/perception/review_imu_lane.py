#!/usr/bin/env python3
"""Replay exact captured model grids through M0/M1; never publishes or promotes.

Semantic reprojection measures internal consistency, not independent GT accuracy.
Each native RGB overlay uses its own variant's camera/ground transform.
"""
import argparse
from collections import Counter, defaultdict
import html
import json
from pathlib import Path
import time

import cv2
import numpy as np
import yaml

from asmc_camera.bev.calibration import calibrations_from_dict
from asmc_camera.bev.model_grid_projection import MetricEvidence, ModelGridGroundProjector
from asmc_camera.bev.imu_projection import attitude_camera, correct_evidence
from review_family_motion import Fitter, family_dict, overlay, reprojection, rpy, sha, stats

VIEWS = ('front', 'left', 'right')


def rows(path):
    with path.open() as stream:
        yield from map(json.loads, stream)


def online_report(directory):
    events = list(rows(directory/'events.jsonl'))
    family = [r for r in events if r['kind'] == 'family']
    diagnostics = [r for r in events if r['kind'] == 'diagnostics']
    times = [r['received_monotonic'] for r in family]
    stamp = [r['message']['header']['stamp'] for r in family]
    valid_times = [r['received_monotonic'] for r in family if r['message']['valid']]
    values = [{kv['key']: kv['value'] for kv in r['message']['status'][0]['values']}
              for r in diagnostics]
    imu = [json.loads(v['imu_motion']) for v in values]
    windows = []
    if times:
        begin, end = times[0], times[-1]
        for start in np.arange(begin, end-5, 5):
            windows.append(sum(start <= t < start+5 for t in valid_times)/5.)
    counts = Counter()
    if len(imu) > 1:
        counts.update(imu[-1]['counts'])
        counts.subtract(imu[0]['counts'])
    return dict(capture=json.loads((directory/'report.json').read_text()),
        messages=len(family), valid=sum(r['message']['valid'] for r in family),
        valid_complete_5s_hz=stats(windows),
        receive_gap_ms=stats(np.diff(times)*1000),
        valid_receive_gap_ms=stats(np.diff(valid_times)*1000),
        duplicate_or_reverse_stamp=sum(b <= a for a,b in zip(stamp,stamp[1:])),
        imu_counts_between_diagnostics=dict(counts),
        latest_stage_ms=json.loads(values[-1]['runtime_stage_ms']) if values else {},
        latest_imu=imu[-1] if imu else {},
        note='Read-only observer; GUI off, A748 stationary, not shadow ODD acceptance.' +
             (' First M1 capture overlaps CPU regression tests; rates are diagnostic only.'
              if directory.name.endswith('m1-r1') else ''))


def compare(args):
    args.output.mkdir(parents=True, exist_ok=False)
    cv2.setNumThreads(1)
    cfg=yaml.safe_load((args.config/'bev/lane_fitting.yaml').read_text())
    imu_cfg=yaml.safe_load((args.config/'bev/imu_motion.yaml').read_text())
    cameras=calibrations_from_dict(yaml.safe_load((args.config/'cameras_v3.yaml').read_text()))
    projectors={v:ModelGridGroundProjector(cameras[v],
        model_hw=(544,960) if v=='front' else (480,640),source_mask=1<<i,
        x_bounds_m=(cfg['evidence_x_min_m'],cfg['evidence_x_max_m']),
        y_bounds_m=(cfg['evidence_y_min_m'],cfg['evidence_y_max_m']),
        max_ground_range_m=cfg['max_ground_range_m']) for i,v in enumerate(VIEWS)}
    fitter=Fitter(cfg); records=[]; costs=defaultdict(list); gallery=[]
    for directory in args.inputs:
        captured=list(rows(directory/'frames.jsonl'))
        for row in captured:
            if not all(row['views'][v]['quaternion'] is not None for v in VIEWS):
                continue
            source=[row['views'][v]['lineage']['sensor_timing'] for v in VIEWS]
            source_ns=[r['source_sec']*10**9+r['source_nsec'] for r in source]
            if max(source_ns)-min(source_ns)>35e6:
                continue
            arrays=dict(np.load(directory/row['grid']))
            corrected={v:attitude_camera(cameras[v],row['views'][v]['quaternion'],
                imu_cfg['imu_mount_rotation_deg'],imu_cfg['max_tilt_deg']) for v in VIEWS}
            base={v:projectors[v].extract(arrays[v+'_class'],arrays[v+'_confidence'],{1:.5,2:.5}) for v in VIEWS}
            variants={}
            for name in (('M0','M1') if row['index']%2==0 else ('M1','M0')):
                started=time.perf_counter()
                evidence=(list(base.values()) if name=='M0' else
                          [correct_evidence(projectors[v],base[v],corrected[v]) for v in VIEWS])
                remap_ms=1000*(time.perf_counter()-started)
                family,audit,stages=fitter.fit(MetricEvidence.concatenate(evidence))
                stages.update(remap_ms=remap_ms, total_ms=1000*(time.perf_counter()-started))
                value=family_dict(family,audit)
                value['semantic_reprojection_model_px']=reprojection(value, cameras if name=='M0' else corrected,arrays)
                variants[name]=value
                for key,ms in stages.items():costs[name+'_'+key].append(ms)
            delta=[];a,b=variants['M0'],variants['M1']
            for member in b['boundaries']:
                prior=next((m for m in a['boundaries'] if m['role']==member['role']),None)
                if prior is None:continue
                old=np.asarray(prior['xy']);new=np.asarray(member['xy'])
                if old.shape==new.shape:
                    valid=(np.asarray(prior['support'])==1)&(np.asarray(member['support'])==1)&(new[:,0]>=0)&(new[:,0]<=15)
                    delta.extend(np.linalg.norm(new[valid]-old[valid],axis=1).tolist())
            record=dict(run=directory.name,index=row['index'],grid_sha256=sha(directory/row['grid']),
                source_ns=source_ns,brackets_ms=[row['views'][v]['bracket_ms'] for v in VIEWS],
                roll_pitch_deg=np.rad2deg(rpy(row['views']['front']['quaternion']))[:2].tolist(),
                variants=variants,paired_curve_displacement_m=stats(delta))
            records.append(record)
            if row['index'] in (0,len(captured)//2,len(captured)-1):
                tiles=[]
                for name in ('M0','M1'):
                    images=[]
                    for v in VIEWS:
                        source_image=directory/row['views'][v]['rgb']
                        if sha(source_image)!=row['views'][v]['rgb_sha256']:raise ValueError('RGB SHA mismatch')
                        im=cv2.imread(str(source_image));im=overlay(im,variants[name],cameras[v] if name=='M0' else corrected[v])
                        im=cv2.resize(im,(480,270 if v=='front' else 360))
                        canvas=np.zeros((390,480,3),np.uint8);canvas[:im.shape[0]]=im
                        cv2.putText(canvas,name+' '+v,(8,380),cv2.FONT_HERSHEY_SIMPLEX,.6,(255,255,255),1)
                        images.append(canvas)
                    tiles.append(np.concatenate(images,axis=1))
                file=directory.name+'-%03d.jpg'%row['index']
                cv2.imwrite(str(args.output/file),np.concatenate(tiles,axis=0))
                gallery.append(file)
    if not records:raise ValueError('no source-aligned three-view samples')
    import asmc_camera.bev.imu_projection as projection_code
    import review_family_motion as review_code
    summary=dict(schema='camera_imu_projection_review_v1',tool_sha256=sha(__file__),
        dependency_sha256={module.__name__:sha(module.__file__) for module in (projection_code,review_code)},
        versions=dict(numpy=np.__version__,opencv=cv2.__version__),
        configs={p:sha(args.config/p) for p in ('cameras_v3.yaml','bev/imu_motion.yaml','bev/lane_fitting.yaml')},
        inputs=[dict(path=str(p),frames_sha256=sha(p/'frames.jsonl'),events_sha256=sha(p/'events.jsonl')) for p in args.inputs],
        paired_frames=len(records),
        validity={name:sum(r['variants'][name]['valid'] for r in records) for name in ('M0','M1')},
        semantic_reprojection_model_px={name:stats([r['variants'][name]['semantic_reprojection_model_px']['p50']
            for r in records if r['variants'][name]['semantic_reprojection_model_px'].get('count')]) for name in ('M0','M1')},
        paired_curve_displacement_m=stats([r['paired_curve_displacement_m']['p50'] for r in records if r['paired_curve_displacement_m'].get('count')]),
        roll_pitch_deg={k:stats([r['roll_pitch_deg'][i] for r in records]) for i,k in enumerate(('roll','pitch'))},
        stages_ms={k:stats(v) for k,v in costs.items()},online={p.name:online_report(p) for p in args.inputs},
        decision='M1 source-time trial implemented; keep canonical M0 until shadow driving contrast.',
        limits=['Stationary A748 is outside shadow family ODD.',
          'Internal semantic consistency and paired displacement are not independent metric GT.',
          'No heave, translation/yaw alignment or temporal support; fixed normal distance0.36m.',
          'Existing native RGB/BEV debug warp remains fixed-plane; use these variant-specific native overlays.'])
    (args.output/'frames.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in records))
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    def escape(x):return html.escape(str(x))
    page='<!doctype html><meta charset="utf-8"><title>Source-time IMU M0/M1 projection</title><style>body{font:16px sans-serif;max-width:1450px;margin:24px auto;background:#121820;color:#eee}img{width:100%}pre{white-space:pre-wrap}a{color:#8cf}</style>'
    page+='<h1>원본 시각 IMU: M0/M1 지면 투영 대조 · 2026-10-18</h1><p>A748 정차 연결 시험. 위 M0 / 아래 M1. 각 행의 자체 자세로 같은 원본 RGB에 재투영. 음영구간 주행 수락이나 독립 GT 정확도 시험이 아닙니다.</p>'
    page+='<p>paired frames %d · valid M0 %d / M1 %d</p>'%(len(records),summary['validity']['M0'],summary['validity']['M1'])
    page+='<p>정본 M0 유지. M1은 내부 시험 launch에서 적용. 지연·공백 수락은 별도이며 IMU 적용률과 family validity를 구분합니다.</p>'
    page+=''.join('<figure><figcaption>'+escape(file)+'</figcaption><img loading="lazy" src="'+escape(file)+'"></figure>' for file in gallery)
    page+='<p><a href="summary.json">전체 집계</a> · <a href="frames.jsonl">프레임별 비교</a></p><pre>'+escape(json.dumps(summary,indent=2,ensure_ascii=False))+'</pre>'
    (args.output/'index.html').write_text(page)
    print(json.dumps({k:summary[k] for k in ('paired_frames','validity','paired_curve_displacement_m')},indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--inputs',nargs='+',type=Path,required=True)
    parser.add_argument('--config',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    compare(parser.parse_args())
