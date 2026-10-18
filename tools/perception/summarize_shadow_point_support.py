#!/usr/bin/env python3
"""Summarize measured saved-bag evidence without inventing GT accuracy."""
import argparse
from collections import Counter, defaultdict
import hashlib
import html
import json
from pathlib import Path
import shutil

import cv2
import numpy as np


def read_rows(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def quantiles(values):
    return {f'p{q}':float(np.percentile(values,q)) for q in (50,95,99)} if values else {}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path)
    parser.add_argument('--portable',type=Path,required=True)
    args=parser.parse_args(); root=args.directory
    models={}; observations={}
    for model in ('e8','yolo26s-seg','yoloe26s-seg','pidnet-s'):
        folder=root/'results'/model
        summary=json.loads((folder/'summary.json').read_text())
        rows=read_rows(folder/'observations.jsonl'); observations[model]=rows
        if len(rows)!=624 or Counter(r['view'] for r in rows)!=dict(front=208,left=208,right=208):
            raise ValueError('incomplete model run '+model)
        if len({(r['stamp_ns'],r['view']) for r in rows})!=len(rows):
            raise ValueError('duplicate model observation')
        if model!='pidnet-s':
            supported=[b for r in rows for b in r['boxes'] if b['box']['points']]
            active=[r['geometry_only_ms'] for r in rows if r['boxes']]
            summary.update(front_labels=dict(Counter(b['label'] for r in rows if r['view']=='front' for b in r['boxes'])),
                labels_at_07=dict(Counter(b['label'] for r in rows for b in r['boxes'] if b['confidence']>=.7)),
                detection_frames_by_view={v:sum(bool(r['boxes']) for r in rows if r['view']==v) for v in ('front','left','right')},
                box_supported_but_geometry_unknown=sum(b['geometry']['points']==0 for b in supported),
                detections_without_raw_support=sum(b['box']['points']==0 for r in rows for b in r['boxes']),
                active_frame_geometry_ms=quantiles(active),
                maximum_frame_geometry_ms=max(r['geometry_only_ms'] for r in rows))
        summary.pop('gallery',None); models[model]=summary
    pcl=read_rows(root/'pcl-structure.jsonl')
    if len(pcl)!=208 or len({r['stamp_ns'] for r in pcl})!=208:
        raise ValueError('incomplete PCL evidence')
    pcl_summary=dict(scans=len(pcl),timings_ms={k:quantiles([r[k] for r in pcl]) for k in ('normal_ms','plane_ms')},
        combined_ms=quantiles([r['normal_ms']+r['plane_ms'] for r in pcl]),
        plane_counts=dict(Counter(len(r['planes']) for r in pcl)),
        parameters=dict(normal_k=12,maximum_planes=4,ransac_iterations=150,distance_m=.12),
        input_manifest=json.loads((root/'pcl-input-manifest.json').read_text()),
        scope='raw sensor ROI/voxel, observation only; no gravity/IMU semantic classification, no PCL RegionGrowing/CEC executed',
        deletion_enabled=False)
    rows=read_rows(root/'tracker-common-pitch.jsonl'); tracks=defaultdict(list)
    for i,r in enumerate(rows):
        for obj in r['objects']: tracks[obj['id']].append((i,r,obj))
    speed=[]; events=[]
    for track,values in tracks.items():
        for i,row,obj in values:
            value=float(np.linalg.norm(obj['kf_ca'][2:4]));speed.append(value)
            if value>10:
                events.append(dict(track_id=track,scan_index=i,stamp_ns=row['stamp_ns'],relative_kf_speed_mps=value,
                                   initialization_count=obj['kf_init_count'],lidar_misses=obj['lidar_misses']))
    tracker=dict(scans=len(rows),ready=sum(r['ready'] for r in rows),plane_valid=sum(r.get('plane_valid',False) for r in rows),
        track_ids=len(tracks),single_observation_ids=sum(len(v)==1 for v in tracks.values()),
        observation_records=len(speed),internal_kf_relative_speed_mps=quantiles(speed),
        maximum_internal_relative_speed_mps=max(speed,default=0),over_10mps_observations=len(events),
        largest_internal_speed_events=sorted(events,key=lambda x:x['relative_kf_speed_mps'],reverse=True)[:20],
        profile='common_pitch',camera_boxes_replayed=False,
        binary_sha256='fbe82c1d814a9588735b248dc0ba2e62c34d2f04eb10954a0788d1bebcefed09',
        settings_sha256='a6d84af1ab14015cdf555ae715cdd8288cde9f1c1ee7d8e0a96e3931751ad7a2',
        scope='2026-10-14 saved deployment snapshot, logical record-order replay; not newly built 2026-10-18 Main, not latency test',
        warning='IDs are algorithm outputs, not actors. Relative KF state is not absolute actor speed or independently validated velocity.')
    tracker['state_speed_by_observation_status']={}
    for name,predicted in (('lidar_misses_zero',False),('lidar_misses_positive',True)):
        values=[float(np.linalg.norm(o['kf_ca'][2:4])) for r in rows for o in r['objects']
                if (o['lidar_misses']>0)==predicted]
        tracker['state_speed_by_observation_status'][name]=dict(records=len(values),quantiles_mps=quantiles(values),
                                                              maximum_mps=max(values,default=0))
    result=dict(experiment='lidar-fusion-shadow-20261018-r1',date='2026-10-18',
        gpu='RTX 4060 Ti 8GB',input=json.loads((root/'input-manifest.json').read_text()),
        executed_source_hashes=json.loads((root/'executed-source-hashes.json').read_text()),
        pidnet_weights=json.loads((root/'pidnet-weight-source.json').read_text()),
        models=models,pcl=pcl_summary,tracker=tracker,
        followup_priority=['retain E8 + bounded geometry, explicit unsupported/unknown',
            'PCL finite-support evidence with gravity/ground + object preservation validation',
            'YOLOE small only if adapted to MORAI and bounded point processing',
            'PIDNet only after tunnel semantic adaptation','COCO YOLO26s-seg replacement deferred'],
        not_executed=['PCL RegionGrowing/Conditional Euclidean Clustering','official depth_clustering',
            'HARP-NeXt/FRNet','CenterPoint-Pillar','tracker replacement','4th/rear camera','concurrent full stack / control'],
        limitations=['one development bag; no independent object/semantic/velocity GT',
            'offline nearest image may arrive after scan; not causal runtime policy',
            'raw points; no production ground/voxel before camera support adapter',
            'E8 FP16 TensorRT vs seg FP32 at different input sizes and vocabularies; not isolated architecture comparison',
            'PIDNet latency excludes CPU resize/normalization/H2D; no end-to-end latency ranking',
            'Torch allocator peaks exclude TensorRT allocations; no total GPU peak claim',
            'one pass per candidate, incidental host/CPU load; not repeated performance acceptance'],
        validation=dict(point_lineage_and_timestamp_tests=3,all_model_frames_complete=True,
                        pcl_scans_complete=True,tracker_scans_complete=len(rows)==208),
        operational_defaults_changed=False)
    (root/'summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    args.portable.mkdir(parents=True,exist_ok=True)
    shutil.copyfile(root/'summary.json',args.portable/'summary.json')
    for figure in ('tracking-diagnostic.png','structure-evidence.png'):
        shutil.copyfile(root/figure,args.portable/figure)
    # Uniform matched frames, with all candidates and original source image.
    frames=[0,32,64,96,112,144,176,192]
    for index in frames:
        tiles=[]
        for model in models:
            im=cv2.imread(str(root/'results'/model/'gallery'/f'{index:03d}-front.jpg'))
            if im is None: raise ValueError('missing uniform gallery sample')
            im=cv2.resize(im,(640,360));cv2.rectangle(im,(0,0),(640,28),(20,20,20),-1)
            cv2.putText(im,model+f' / front scan {index}',(10,20),cv2.FONT_HERSHEY_SIMPLEX,.55,(255,255,255),1)
            tiles.append(im)
        montage=np.vstack((np.hstack(tiles[:2]),np.hstack(tiles[2:])))
        name=f'comparison-{index:03d}.jpg';cv2.imwrite(str(root/name),montage)
        if index in (64,112): shutil.copyfile(root/name,args.portable/name)
    table=''.join('<tr><td>'+html.escape(name)+'</td><td>'+str(m['frames'])+'</td><td>'+str(round(m['timings_ms']['inference_ms']['p95'],1))+'</td><td>'+html.escape(json.dumps(m.get('front_labels',{}),ensure_ascii=False))+'</td></tr>' for name,m in models.items())
    notice=('624 images per model = 208 scans × front/left/right. Offline diagnostic on one development bag; '
            'no independent GT and no control/full-stack acceptance. E8 and seg differ in precision, input size and training. '
            'PIDNet timing excludes input preprocessing. Red=box point support, yellow=tentative geometry support, green=instance mask. '
            'PIDNet overlay uses Cityscapes colors and has no ceiling class.')
    page=('<!doctype html><meta charset="utf-8"><title>음영 bag 인지 후보 비교</title>'
          '<style>body{font:16px sans-serif;max-width:1320px;margin:30px auto}table{border-collapse:collapse}td,th{border:1px solid #ccc;padding:8px}img{max-width:100%}p{line-height:1.6}</style>'
          '<h1>음영 bag 인지 후보 비교 · 2026-10-18</h1><p>'+notice+'</p>'
          '<table><tr><th>Model</th><th>Images</th><th>Measured call p95 ms*</th><th>Front output labels @0.4</th></tr>'+table+'</table>'
          '<p><a href="summary.json">전체 수치·조건·SHA·미실행 후보</a></p>'
          '<p><a href="structure-evidence.png">PCL 표면 지지점</a> · <a href="tracking-diagnostic.png">추적 관측·예측 상태</a></p>'
          '<p>PCL normal + 4 plane evidence p95 '+str(round(pcl_summary['combined_ms']['p95'],1))+' ms/scan. No structure suppression.</p>'
          '<p>Compare original objects and background, not just point count reduction. Scan 112: E8 detects the crate but has zero projected raw LiDAR support.</p>')
    (root/'index.html').write_text(page+''.join('<figure><figcaption>front scan '+str(i)+'</figcaption><img loading="lazy" src="comparison-%03d.jpg"></figure>'%i for i in frames))
    (args.portable/'index.html').write_text(page+''.join('<figure><figcaption>front scan '+str(i)+'</figcaption><img src="comparison-%03d.jpg"></figure>'%i for i in (64,112)))
    print(json.dumps(dict(models=list(models),pcl_combined_ms=pcl_summary['combined_ms'],tracker=tracker),ensure_ascii=False,indent=2))


if __name__=='__main__': main()
