#!/usr/bin/env python3
"""Compare two static offsets on existing semantic grids, without inference.

This is a supported-geometry sensitivity check, not an independent metric GT
calibration. Model weights, mounts, fitter and source timestamps stay fixed.
"""
import argparse
from dataclasses import replace
import json
from pathlib import Path
import time

import cv2
import numpy as np
import yaml
from asmc_camera.bev.calibration import calibrations_from_dict, GroundPlane
from asmc_camera.bev.model_grid_projection import ModelGridGroundProjector, MetricEvidence
from tools.perception.review_family_motion import Fitter, family_dict, read_rows, stats, sha
from tools.perception.summarize_family_motion import sample_boundary


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('grids','config','output'):parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    cv2.setNumThreads(1)
    cameras=calibrations_from_dict(yaml.safe_load((args.config/'cameras_v3.yaml').read_text()))
    fitter=Fitter(yaml.safe_load((args.config/'bev/lane_fitting.yaml').read_text()))
    projectors={}
    for label,z in [('old_35cm',-.35),('nominal_36cm',-.36)]:
        projectors[label]={view:ModelGridGroundProjector(replace(camera,
            ground_plane=GroundPlane('base_link',z,0,0,'offset_sensitivity')),source_mask=1<<i,
            model_hw=(544,960) if view=='front' else (480,640),max_ground_range_m=22)
            for i,(view,camera) in enumerate(cameras.items())}
    frames=[];deltas=[];query=np.arange(.5,14.501,.25)
    for row in read_rows(args.grids/'manifest.jsonl'):
        path=args.grids/row['grid']
        if sha(path)!=row['grid_sha256']:raise ValueError('grid SHA mismatch')
        arrays=np.load(path);variants={}
        for label,views in projectors.items():
            started=time.monotonic()
            evidence=MetricEvidence.concatenate([p.extract(arrays[v+'_class'],arrays[v+'_confidence'],{1:.5,2:.5})
                                                  for v,p in views.items()])
            family,audit,timing=fitter.fit(evidence)
            variants[label]=dict(family_dict(family,audit),compute_ms=1000*(time.monotonic()-started))
        a,b=(variants[k] for k in ('old_35cm','nominal_36cm'))
        for boundary in a['boundaries']:
            other=next((x for x in b['boundaries'] if x['role']==boundary['role']),None)
            if other is None:continue
            delta=np.abs(sample_boundary(boundary,query)-sample_boundary(other,query))
            deltas.extend(delta[np.isfinite(delta)].tolist())
        frames.append(dict(run_id=row['run_id'],phase=row['phase'],variants=variants))
    summary={label:dict(valid=sum(r['variants'][label]['valid'] for r in frames),frames=len(frames),
        compute_ms=stats([r['variants'][label]['compute_ms'] for r in frames])) for label in projectors}
    report=dict(schema='static_ground_offset_comparison_v1',summary=summary,
        newly_invalid=sum(r['variants']['old_35cm']['valid'] and not r['variants']['nominal_36cm']['valid'] for r in frames),
        common_supported_lateral_change_m=stats(deltas),config_sha256=sha(args.config/'cameras_v3.yaml'),
        grid_manifest_sha256=sha(args.grids/'manifest.jsonl'),
        limitations=['Same-model semantics and fitted curves, not independent ground accuracy.',
                     'Six weather/time runs and three phases; short related sequences are not independent scenes.',
                     'Offset change is user-approved nominal VehicleSpec wheel radius, not measured suspension height.'])
    (args.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    (args.output/'frames.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in frames))
    print(json.dumps(report,indent=2),flush=True)


if __name__=='__main__':main()
