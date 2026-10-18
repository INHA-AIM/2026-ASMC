#!/usr/bin/env python3
"""Compare measured-rest geometry without changing runtime calibration.

Road semantic selection is a diagnostic aid, not independent ground truth.
A candidate is not promoted merely because it fits its own source semantics.
"""
import argparse
from dataclasses import replace
import html
import json
from pathlib import Path

import cv2
import numpy as np
import yaml
from asmc_camera.bev.calibration import calibrations_from_dict, GroundPlane
from asmc_camera.bev.model_grid_projection import ModelGridGroundProjector, MetricEvidence
from tools.perception.calibrate_stationary_lane_plane import fit_plane
from tools.perception.review_family_motion import (
    rotation, interpolate, read_rows, stats, Fitter, family_dict, attitude_plane,
    geometry_evidence, reprojection, overlay, sha,
)


def calibrated_plane(camera, q_current, q_reference, normal_reference, height):
    """Transport a measured rest-plane normal, never subtract Euler angles.

    R_WB(t)^T R_WB(0) n_B(0); no extra camera mount rotation. Constant
    normal distance assumes no heave. It is not a height estimator.
    """
    normal = np.asarray(normal_reference, dtype=float)
    if normal.shape != (3,) or not np.isfinite(normal).all() or np.linalg.norm(normal)<1e-8:
        raise ValueError('invalid measured normal')
    if not np.isfinite(height) or height <= 0:
        raise ValueError('invalid plane height')
    normal = rotation(q_current).T @ rotation(q_reference) @ (normal/np.linalg.norm(normal))
    if normal[2] < .8: raise ValueError('normal outside plane domain')
    return replace(camera, ground_plane=GroundPlane('base_link', -height/normal[2],
        -normal[0]/normal[2], -normal[1]/normal[2], 'offline_measured_rest'))


def road_selection(points, frames, cameras, source, stamp):
    keep=np.zeros(len(points), dtype=bool)
    for view,camera in cameras.items():
        candidates=[r for r in frames if r['view']==view]
        if not candidates: continue
        row=min(candidates,key=lambda r:abs(r['source_ns']-stamp))
        if abs(row['source_ns']-stamp)>100_000_000:continue
        mask=np.load(source/row['grid'])['drivable']
        optical=camera.ground_to_optical(points)
        uv=optical@camera.intrinsic.T
        depth=uv[:,2]; safe=depth>.1
        px=np.zeros((len(points),2),dtype=int)
        px[safe]=np.rint(uv[safe,:2]/depth[safe,None]).astype(int)
        valid=safe&(px[:,0]>=0)&(px[:,0]<mask.shape[1])&(px[:,1]>=0)&(px[:,1]<mask.shape[0])
        keep[valid] |= mask[px[valid,1],px[valid,0]]>=204
    return keep


def run(args):
    args.output.mkdir(parents=True,exist_ok=False)
    config=yaml.safe_load((args.config/'bev/lane_fitting.yaml').read_text())
    cameras=calibrations_from_dict(yaml.safe_load((args.config/'cameras_v3.yaml').read_text()))
    source=args.inputs;original=json.loads((source/'report.json').read_text())
    frames=read_rows(source/'frames.jsonl');imu=read_rows(source/'imu.jsonl')
    q_reference=original['reference_quaternion_xyzw']
    cloud_rows=json.loads((source/'cloud-fits.json').read_text())
    plane_rows=[]
    for i,row in enumerate(cloud_rows):
        points=np.load(source/('ground-%03d.npz'%i))['points_base']
        road=road_selection(points,frames,cameras,source,row['source_ns'])
        rois={'all':np.ones(len(points),bool),'semantic_road':road,
              'near_road':road&(points[:,0]<10), 'far_road':road&(points[:,0]>=10)}
        for label,selected in rois.items():
            try:fit=fit_plane(points[selected])
            except ValueError as error:fit={'error':str(error)}
            plane_rows.append(dict(cloud=i,roi=label,**fit))
    fits=[r for r in plane_rows if r['roi']=='semantic_road' and 'error' not in r]
    if not fits: raise ValueError('no road-plane candidate')
    normal=np.median([r['normal'] for r in fits],axis=0);normal/=np.linalg.norm(normal)
    height=float(np.median([r['normal_distance_m'] for r in fits]))
    summary={}
    for label in ('all','semantic_road','near_road','far_road'):
        selected=[r for r in plane_rows if r['roi']==label and 'error' not in r]
        summary[label]={k:stats([r[k] for r in selected]) for k in
                        ('normal_distance_m','inlier_ratio','inlier_residual_p95_m')}
    heights=[summary[label]['normal_distance_m'].get('p50') for label in ('semantic_road','near_road','far_road')]
    spread=max(heights)-min(heights) if all(h is not None for h in heights) else None
    gates={k:v for k,v in original['gates'].items() if k!='supported_ground_frames'}
    gates['road_supported_frames']=sum(r['inlier_ratio']>=.7 for r in fits)>=20
    gates['roi_height_consistency_le_2cm']=spread is not None and spread<=.02
    reference_valid=all(gates.values())
    projectors={v:ModelGridGroundProjector(cam, model_hw=(544,960) if v=='front' else (480,640),
                                         source_mask=1<<i, max_ground_range_m=22)
                for i,(v,cam) in enumerate(cameras.items())}
    fitter=Fitter(config);results=[];gallery=[]
    fronts=[r for r in frames if r['view']=='front']
    for index,front in enumerate(fronts):
        bundle={v:min((r for r in frames if r['view']==v),key=lambda r:abs(r['source_ns']-front['source_ns']))
                for v in cameras}
        if max(r['source_ns'] for r in bundle.values())-min(r['source_ns'] for r in bundle.values())>35_000_000:continue
        qs={v:interpolate(imu,r['source_ns'],'imu') for v,r in bundle.items()}
        if not all(qs.values()):continue
        arrays={};grids={}
        for v,row in bundle.items():
            grids[v]=np.load(source/row['grid'])
            arrays[v+'_class']=grids[v]['classes']
        variants={};variant_cameras={}
        for name in ('M0','M1_rest_old','measured_rest_candidate'):
            cams={};evidence=[]
            for v,cam in cameras.items():
                if name=='M1_rest_old':cam=attitude_plane(cam,qs[v]['q_xyzw'],q_reference)
                elif name=='measured_rest_candidate':
                    cam=calibrated_plane(cam,qs[v]['q_xyzw'],q_reference,normal,height)
                cams[v]=cam
                evidence.append(geometry_evidence(projectors[v],grids[v]['classes'],
                                                  grids[v]['confidence'],cam))
            family,audit,timing=fitter.fit(MetricEvidence.concatenate(evidence))
            result=family_dict(family,audit);result['timings_ms']=timing
            result['semantic_consistency_px']=reprojection(result,cams,arrays)
            variants[name]=result;variant_cameras[name]=cams
        results.append(dict(source_ns=front['source_ns'],variants=variants))
        if index in (len(fronts)//2,):
            panels=[]
            for v in ('left','front','right'):
                image=cv2.imread(str(source/bundle[v]['image']))
                for name in variants:
                    panel=overlay(image,variants[name],variant_cameras[name][v])
                    panel=cv2.resize(panel,(480,270))
                    cv2.putText(panel,v+' '+name,(8,22),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,140,255),1)
                    panels.append(panel)
            target='stationary-overlays.jpg';cv2.imwrite(str(args.output/target),np.vstack([
                np.hstack(panels[i:i+3]) for i in range(0,len(panels),3)]))
            gallery.append(target)
    report=dict(schema='stationary_geometry_review_v1',stationary_input=original,
        plane_roi_summary=summary,roi_height_spread_m=spread,reference_gates=gates,
        reference_valid=reference_valid,normal_candidate=normal.tolist(),height_candidate_m=height,
        tested_frames=len(results),variants={name:dict(valid=sum(r['variants'][name]['valid'] for r in results),
            semantic_frame_p95_px=stats([r['variants'][name]['semantic_consistency_px']['p95'] for r in results
                if r['variants'][name]['semantic_consistency_px'].get('count')]))
            for name in ('M0','M1_rest_old','measured_rest_candidate')},
        adopted=False,operational_geometry_changed=False,
        source_sha256={name:sha(source/name) for name in ('report.json','frames.jsonl','imu.jsonl')},
        limitations=['Road semantics are model output, not surveyed ground GT.',
                     'Measured-rest candidate is sensitivity only when any reference gate fails.',
                     'Same fitting code for all variants; changed projected evidence can change selected tracks and curves.',
                     'Static case does not validate moving time offset or all tunnel phases.'])
    (args.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    (args.output/'planes.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in plane_rows))
    (args.output/'frames.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in results))
    (args.output/'index.html').write_text('<meta charset="utf-8"><h1>Stationary plane and M0/M1 contrast</h1>'
        '<p>Offline candidates only. M0 runtime retained. <a href="report.json">Report</a></p>'
        +'<pre>'+html.escape(json.dumps(report,indent=2))+'</pre>'
        +''.join('<img style="max-width:100%" src="'+p+'">' for p in gallery))
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('inputs','config','output'):parser.add_argument('--'+name,type=Path,required=True)
    run(parser.parse_args())
