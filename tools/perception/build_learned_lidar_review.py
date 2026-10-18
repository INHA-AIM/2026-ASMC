#!/usr/bin/env python3
"""Create comparable source-image/point-label/box evidence, without GT claims."""
import argparse
from collections import Counter
import hashlib
import html
import json
from pathlib import Path
import shutil
import sys
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src/asmc_camera/src'))
from asmc_camera.bev.calibration import load_calibration_snapshot
from benchmark_shadow_point_support import project


def read(path):return json.loads(path.read_text())
def rows(path):return [json.loads(l) for l in path.read_text().splitlines()]


def point_overlay(image,xyz,raw,pred_path,summary,lidar,camera):
    result=image.copy();data=np.load(pred_path)
    if not np.array_equal(data['raw_index'],raw):raise ValueError('prediction point lineage differs')
    prediction=data['semantic'];names=summary['classes']
    pixels,visible,depth=project(xyz,lidar,camera)
    angles=np.degrees(np.arcsin(xyz[:,2]/np.linalg.norm(xyz,axis=1)))
    fov=summary['range_projection'];clipped=(angles>fov['fov_up'])|(angles<fov['fov_down'])
    road={'road','driveable_surface','parking'}
    structures={'building','manmade','fence','pole','traffic-sign'}
    other={'vegetation','trunk','terrain','sidewalk','other_flat','other-ground'}
    for index in np.argsort(depth)[::-1]:
        if not visible[index]:continue
        name=names[int(prediction[index])]
        color=(255,150,0) if name in road else ((230,50,230) if name in structures else
              ((80,180,80) if name in other else (0,150,255)))
        if clipped[index]:color=(220,220,220)
        cv2.circle(result,tuple(pixels[index].astype(int)),1,color,-1)
    return result


def box_overlay(image,detections,lidar,camera):
    output=image.copy()
    edges=((0,1),(1,2),(2,3),(3,0),(4,5),(5,6),(6,7),(7,4),(0,4),(1,5),(2,6),(3,7))
    for detection in detections:
        if detection['score']<.4:continue
        x,y,z,dx,dy,dz,yaw=detection['box'][:7]
        template=np.array([[-.5,-.5,0],[-.5,.5,0],[.5,.5,0],[.5,-.5,0],
                           [-.5,-.5,1],[-.5,.5,1],[.5,.5,1],[.5,-.5,1]])*np.array([dx,dy,dz])
        rotation=np.array([[np.cos(yaw),-np.sin(yaw),0],[np.sin(yaw),np.cos(yaw),0],[0,0,1]])
        corners=template@rotation.T+[x,y,z]
        pixels,visible,depth=project(corners,lidar,camera)
        if not visible.any() or not (depth>.1).all():continue
        pixels=np.clip(pixels,-100000,100000).astype(int)
        for i,j in edges:cv2.line(output,tuple(pixels[i]),tuple(pixels[j]),(0,220,255),1)
        text=detection['label']+' %.2f'%detection['score']
        cv2.putText(output,text,tuple(np.maximum(10,np.minimum(pixels[0],np.array([1100,700])))),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,220,255),1)
    return output


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('experiment',type=Path)
    p.add_argument('--baseline',type=Path,required=True);p.add_argument('--portable',type=Path,required=True)
    a=p.parse_args();a.portable.mkdir(parents=True,exist_ok=True)
    model_dirs=('harp-nuscenes','harp-semantickitti-gpu','centerpoint-pillar')
    models={name:read(a.experiment/name/'summary.json') for name in model_dirs}
    source=read(a.experiment/'xyzi/manifest.json')
    for name in model_dirs:
        observed=rows(a.experiment/name/'observations.jsonl')
        if len(observed)!=208 or len({r['stamp_ns'] for r in observed})!=208:raise ValueError('incomplete '+name)
        if not models[name]['strict_checkpoint']:raise ValueError('incomplete checkpoint')
    baseline=rows(a.baseline/'inputs/frames.jsonl')
    e8=rows(a.baseline/'results/e8/observations.jsonl')
    cp=rows(a.experiment/'centerpoint-pillar/observations.jsonl')
    _,cameras=load_calibration_snapshot(ROOT/'src/asmc_camera/config/cameras_v3.yaml');camera=cameras['front']
    lidar=read(ROOT/'config/morai/sensors/ASMC_sensor_set_v3.json')['Lidar3DList'][0]
    cases=[];images=[]
    for number in (0,32,64,96,112,144,176,192):
        row=next(r for r in baseline if r['scan_index']==number and r['view']=='front')
        stamp=row['stamp_ns'];image=cv2.imread(str(a.baseline/'inputs'/row['image']))
        with np.load(a.experiment/'xyzi'/f'{stamp}.npz') as data:xyz=data['xyzi'][:,:3];raw=data['raw_index']
        panels=[cv2.imread(str(a.baseline/'results/e8/gallery'/f'{number:03d}-front.jpg'))]
        titles=['Existing E8 detector (image boxes)']
        for name in model_dirs[:2]:
            panels.append(point_overlay(image,xyz,raw,a.experiment/name/f'{stamp}.npz',models[name],lidar,camera));titles.append(name)
        panels.append(box_overlay(image,cp[number]['detections'],lidar,camera));titles.append('CenterPoint-Pillar: projected boxes @0.4')
        tiles=[]
        for panel,title in zip(panels,titles):
            tile=cv2.resize(panel,(640,360));cv2.rectangle(tile,(0,0),(640,26),(15,15,15),-1)
            cv2.putText(tile,title,(8,18),cv2.FONT_HERSHEY_SIMPLEX,.48,(255,255,255),1);tiles.append(tile)
        name=f'comparison-{number:03d}.jpg';cv2.imwrite(str(a.experiment/name),np.vstack((np.hstack(tiles[:2]),np.hstack(tiles[2:]))))
        if number in (64,112):shutil.copyfile(a.experiment/name,a.portable/name)
        images.append((number,name))
        if number==64:
            e8row=next(r for r in e8 if r['scan_index']==number and r['view']=='front')
            lineage=np.load(a.baseline/'results/e8/indices'/f'{stamp}-front.npz')['0_box_raw_index']
            case=dict(scan_index=number,e8_box=e8row['boxes'][0],hints_are_not_point_gt=True)
            for model in model_dirs[:2]:
                data=np.load(a.experiment/model/f'{stamp}.npz');indices=np.searchsorted(data['raw_index'],lineage)
                if not np.array_equal(data['raw_index'][indices],lineage):raise ValueError('box support lineage lost')
                case[model+'_labels_in_e8_box']=dict(Counter(models[model]['classes'][int(v)] for v in data['semantic'][indices]))
            cases.append(case)
    reuse=read(a.experiment/'segmentation-reuse/summary.json')
    dataset_proof=read(a.experiment/'segmentation-reuse/inspected-masks.json')
    calibration=ROOT/'src/asmc_camera/config/cameras_v3.yaml'
    report=dict(experiment='lidar-learned-candidates-20261018-r1',models=models,
        input_manifest_sha256=hashlib.sha256((a.experiment/'xyzi/manifest.json').read_bytes()).hexdigest(),
        input_bag_sha256=source['bag_sha256'],input_scans=208,point_features=source['features'],
        intensity_percentiles=source['intensity_percentiles'],camera_calibration_sha256=hashlib.sha256(calibration.read_bytes()).hexdigest(),
        segmentation_reuse=reuse,native_mask_sample_classes=sorted({r['class_name'] for r in dataset_proof if r['passed']}),
        cases=cases,review_tool_hashes={name:hashlib.sha256((ROOT/'tools/perception'/name).read_bytes()).hexdigest() for name in
            ('prepare_learned_lidar_inputs.py','run_harp_shadow.py','run_centerpoint_shadow.py',
             'audit_segmentation_reuse.py','research_download.py','build_learned_lidar_review.py')},
        limitations=['one reused development bag, no independent point/box/velocity GT',
            'concurrent GPU work from another chat, no exclusive runtime or end-to-end acceptance',
            'HARP raw sensor FoV differs: clipped points are drawn gray; no point suppression',
            'CenterPoint 10-sweep checkpoint probed with one VLP16 scan; velocity invalid; no box class',
            'native mask metadata is not all-annotation QA; 48 sampled masks only',
            'no training performed and no runtime default changes',
            'early smoke metadata failed due missing git; first nuScenes GPU summary failed after all 208 outputs; raw logs preserved, not a primary comparison'],
        branch='research/lidar-perception-20261018',previous_baseline_commit='4da2a5b0')
    report['models']['centerpoint-pillar'].pop('resolved_config',None)
    for directory in (a.experiment,a.portable):(directory/'summary.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    for name in ('summary.json','inspected-masks.json'):
        shutil.copyfile(a.experiment/'segmentation-reuse'/name,a.portable/('segmentation-reuse-'+name))
    for number in (0,5):
        source=a.experiment/'segmentation-reuse/gallery'/f'{number:03d}.jpg'
        shutil.copyfile(source,a.portable/f'mask-proof-{number:03d}.jpg')
    cards=''.join('<figure><figcaption>front scan '+str(i)+'</figcaption><img loading="lazy" src="'+name+'"></figure>' for i,name in images)
    text=('<!doctype html><meta charset="utf-8"><title>학습 인지 후보 실행</title><style>body{font:16px sans-serif;max-width:1320px;margin:24px auto}p{line-height:1.6}img{max-width:100%}</style>'
        '<h1>HARP-NeXt · CenterPoint-Pillar · mask 데이터 검토</h1>'
        '<p>모델별 208 scan. 공유 GPU에서 실행했으며 독립 GT 정확도·실시간 수락이 아닙니다. 센서 raw frame의 출력입니다.</p>'
        '<p>HARP 점: 노면 파랑, 구조물 계열 분홍, 객체 계열 주황, 기타 초록, 학습 FoV 밖 회색. 구조물 계열에는 충돌 가능한 물체가 포함될 수 있습니다.</p>'
        '<p>CenterPoint는 nuScenes 10-sweep 가중치에 단일 scan을 입력했습니다. 상자 클래스가 없고 속도 출력은 유효하지 않습니다.</p>'
        '<p><a href="summary.json">모든 수치·SHA·제약</a> · <a href="segmentation-reuse-summary.json">학습 mask 재사용 감사</a></p>')
    (a.experiment/'index.html').write_text(text+cards)
    (a.portable/'index.html').write_text(text+''.join('<figure><figcaption>front scan '+str(i)+'</figcaption><img src="comparison-%03d.jpg"></figure>'%i for i in (64,112))+
        '<p>Train native-mask inspection examples; source IDs and checks in segmentation-reuse-inspected-masks.json.</p>'+
        ''.join('<img src="mask-proof-%03d.jpg">'%i for i in (0,5)))
    print(json.dumps(dict(models={n:m.get('total_ms',m.get('latency_ms')) for n,m in models.items()},mask_inspection=reuse['inspection'],cases=cases),ensure_ascii=False,indent=2))


if __name__=='__main__':main()
