#!/usr/bin/env python3
"""Compare actual camera/LiDAR projection cores with pinned reference clones.

Optional research tool only. Does not install/import neural networks or mutate
the reference repositories. CLOCs' pure calibration functions are AST-selected
to avoid importing its unrelated detector/runtime dependencies.
"""
import argparse
import ast
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import numpy as np
import yaml

ROOT=Path(__file__).resolve().parents[2]
sys.dont_write_bytecode=True
sys.path.insert(0,str(ROOT/'src/asmc_camera/src'))
from asmc_camera.bev.calibration import load_calibration_snapshot

def module(path,name):
    spec=importlib.util.spec_from_file_location(name,path)
    result=importlib.util.module_from_spec(spec);spec.loader.exec_module(result)
    return result

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--references',type=Path,required=True,help='external/baselines')
    parser.add_argument('--probe',type=Path,required=True,help='compiled frame_projection_probe.cpp')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    files={'frustum-pointnets':'kitti/kitti_util.py',
        'PointPainting-community':'painting/pcdet_utils/calibration_kitti.py',
        'CLOCs':'second/core/box_np_ops.py'}
    provenance={}
    for repo,file in files.items():
        path=args.references/repo
        provenance[repo]=dict(revision=subprocess.check_output(['git','-C',str(path),'rev-parse','HEAD'],text=True).strip(),
            dirty=bool(subprocess.check_output(['git','-C',str(path),'status','--porcelain'],text=True)),
            file=file,sha256=hashlib.sha256((path/file).read_bytes()).hexdigest())
        if provenance[repo]['dirty']: raise RuntimeError('reference checkout is modified: '+repo)
    for repo,paths in {
        'bevfusion':['tools/data_converter/nuscenes_converter.py','mmdet3d/models/vtransforms/base.py'],
        'lift-splat-shoot':['src/data.py','src/models.py'],
        'Lidar_AI_Solution':['CUDA-BEVFusion/src/bevfusion/camera-geometry.cu',
            'CUDA-BEVFusion/src/bevfusion/camera-depth.cu']}.items():
        path=args.references/repo
        provenance[repo]=dict(revision=subprocess.check_output(['git','-C',str(path),'rev-parse','HEAD'],text=True).strip(),
            dirty=bool(subprocess.check_output(['git','-C',str(path),'status','--porcelain'],text=True)),
            source_review_only=True,files={f:hashlib.sha256((path/f).read_bytes()).hexdigest() for f in paths})
        if provenance[repo]['dirty']: raise RuntimeError('reference checkout is modified: '+repo)
    fp=module(args.references/'frustum-pointnets'/files['frustum-pointnets'],'frustum_calibration')
    pp=module(args.references/'PointPainting-community'/files['PointPainting-community'],'painting_calibration')
    tree=ast.parse((args.references/'CLOCs'/files['CLOCs']).read_text())
    selected=ast.Module(body=[node for node in tree.body if isinstance(node,ast.FunctionDef)
        and node.name in ('project_to_image','lidar_to_camera','camera_to_lidar')],type_ignores=[])
    clocs={'np':np};exec(compile(selected,'CLOCs/calibration-selected','exec'),clocs)
    snapshot,cameras=load_calibration_snapshot(ROOT/'src/asmc_camera/config/cameras_v3.yaml')
    mount=yaml.safe_load((ROOT/'src/asmc_lidar/config/autonav_v3.yaml').read_text())['sensor_pose_base_link']
    sensor_t=np.array([mount[k+'_m'] for k in ('x','y','z')])
    rpy=lambda degrees: cameras['front'].__class__(**{
        **vars(cameras['front']), 'rotation_deg':tuple(degrees)}).body_from_mount_rotation
    cases=[];lines=[]
    for name,camera in cameras.items():
        for mount_angles in ([0,0,0],[4,-3,8]):
            sensor_r=rpy(mount_angles)
            for roll,pitch in ((0,0),(9,-6),(-7,5)):
                for x,y,z in ((5,0,-.35),(12,-1,.5),(30,2,1)):
                    # Camera-visible base points, including side-view cameras.
                    base=np.array(camera.translation_m)+camera.body_from_mount_rotation@np.array([x,y,z])
                    raw=sensor_r.T@(base-sensor_t)
                    k=camera.intrinsic
                    expected=k@(camera.optical_from_body_rotation@base+camera.optical_from_body_translation)
                    v2c=np.column_stack([camera.optical_from_body_rotation@sensor_r,
                        camera.optical_from_body_rotation@sensor_t+camera.optical_from_body_translation])
                    cases.append(dict(view=name,roll=roll,pitch=pitch,mount=mount_angles,raw=raw,
                        expected=expected,k=k,v2c=v2c))
                    lines.append(' '.join(map(str,[*camera.translation_m,*camera.rotation_deg,*sensor_t,
                        *mount_angles,roll,pitch,*raw,k[0,0],k[1,1],k[0,2],k[1,2]])))
    result=subprocess.run([str(args.probe.resolve())],input='\n'.join(lines)+'\n',text=True,
        capture_output=True,check=True)
    core=np.array([[float(v) for v in line.split()] for line in result.stdout.splitlines()]).reshape(-1,5,3)
    if len(core)!=len(cases): raise RuntimeError('probe output cardinality mismatch')
    errors={'camera_vs_lidar_raw':[], 'camera_vs_lidar_leveled':[], 'frustum':[], 'painting':[], 'clocs_rigid_roundtrip':[]}
    negative={k:[] for k in ('missing_inverse','extra_inverse','wrong_observation_attitude')}
    stereo={k:[] for k in ('frustum_error_px','painting_error_px','clocs_difference_px')}
    pixel=lambda h:h[..., :2]/h[..., 2:3]
    with tempfile.TemporaryDirectory(prefix='asmc-coordinate-review-') as temporary:
        calib_path=Path(temporary)/'calib.txt'
        for index,c in enumerate(cases):
            raw=c['raw'][None];v2c=c['v2c'];k=c['k'];expected=pixel(c['expected'])
            for label,i in (('camera_vs_lidar_raw',0),('camera_vs_lidar_leveled',1)):
                errors[label].append(float(np.max(np.abs(pixel(core[index,i])-expected))))
            if c['roll'] or c['pitch']:
                for (label,i) in zip(negative,(2,3,4)):
                    negative[label].append(float(np.linalg.norm(pixel(core[index,i])-expected)))
            for baseline in (0.,-.54):
                p=np.column_stack([k,np.array([k[0,0]*baseline,0,0])])
                data={'P2':p,'R0_rect':np.eye(3),'Tr_velo_to_cam':v2c}
                calib_path.write_text(''.join(key+': '+' '.join(map(str,value.ravel()))+'\n' for key,value in data.items()))
                fcal=fp.Calibration(str(calib_path))
                pcal=pp.Calibration(dict(P2=p,R0=np.eye(3),Tr_velo2cam=v2c))
                rect=(np.column_stack([raw,np.ones(1)])@v2c.T)
                rect4=np.eye(4);rigid4=np.vstack([v2c,[0,0,0,1]])
                cref=clocs['lidar_to_camera'](raw,rect4,rigid4)
                back=clocs['camera_to_lidar'](cref,rect4,rigid4)
                exp=pixel(np.column_stack([rect,np.ones(1)])@p.T)
                fuv=fcal.project_velo_to_image(raw);puv,_=pcal.lidar_to_img(raw)
                cuv=clocs['project_to_image'](cref,p)
                if baseline==0:
                    errors['frustum'].append(float(np.max(np.abs(fuv-expected))))
                    errors['painting'].append(float(np.max(np.abs(puv-expected))))
                    errors['clocs_rigid_roundtrip'].append(float(np.max(np.abs(back-raw))))
                else:
                    stereo['frustum_error_px'].append(float(np.max(np.abs(fuv-exp))))
                    stereo['painting_error_px'].append(float(np.max(np.abs(puv-exp))))
                    stereo['clocs_difference_px'].append(float(np.max(np.abs(cuv-exp))))
    maxima={k:max(v) for k,v in errors.items()}
    passed=all(v<1e-8 for v in maxima.values()) and all(min(v)>1 for v in negative.values())
    passed=passed and max(stereo['frustum_error_px'])<1e-8 and max(stereo['painting_error_px'])<1e-8 and min(stereo['clocs_difference_px'])>1
    evidence=dict(schema='asmc.fusion_coordinate_audit.v1',passed=passed,cases=len(cases),
        reference_sources=provenance,camera_snapshot=dict(source=snapshot['source_sensor_set'],sha256=snapshot['source_sha256']),
        maximum_errors=maxima,negative_control_minimum_pixel_error={k:min(v) for k,v in negative.items()},
        stereo_probe={k:dict(minimum=min(v),maximum=max(v)) for k,v in stereo.items()},
        input_sha256={str(path.relative_to(ROOT)):hashlib.sha256(path.read_bytes()).hexdigest() for path in
            [ROOT/'src/asmc_camera/config/cameras_v3.yaml',ROOT/'src/asmc_lidar/config/autonav_v3.yaml',
             ROOT/'src/asmc_lidar/src/geometry/projection.cpp',ROOT/'src/asmc_camera/src/asmc_camera/bev/calibration.py',Path(__file__),
             ROOT/'tools/perception/frame_projection_probe.cpp']},
        limits=['synthetic projection consistency, not physical extrinsic accuracy',
            'no neural inference benchmark or sensor time alignment validation',
            'CLOCs probe is limited to the selected helper at the pinned revision'])
    args.output.write_text(json.dumps(evidence,indent=2)+'\n')
    print(json.dumps(dict(passed=passed,cases=len(cases),maximum_errors=maxima)))
    return 0 if passed else 1

if __name__=='__main__':raise SystemExit(main())
