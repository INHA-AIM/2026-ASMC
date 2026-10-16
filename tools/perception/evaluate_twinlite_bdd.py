#!/usr/bin/env python3
"""Compare unmodified BDD Large and deployed MORAI Medium on collected RGB."""
import argparse
from collections import Counter, defaultdict
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import html
import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src/asmc_camera/src'))
from asmc_camera.camera.backends.twinlite import TwinLiteBackend
from asmc_camera.camera.geometry import LetterboxGeometry, restore_class_ids
from asmc_camera.camera.inference_engine import InferenceEngine


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def counts(truth, predicted, valid):
    return [int(np.count_nonzero(truth & predicted & valid)),
            int(np.count_nonzero((truth | predicted) & valid)),
            int(np.count_nonzero(truth & valid))]


def condition(run):
    import re
    weather = 'SUNNY' if '_sunny_' in run else 'FOGGY' if '_foggy_' in run else 'unknown'
    match = re.search(r'_sim(11am|1pm|3pm)_', run)
    hour = {'11am':'11', '1pm':'13', '3pm':'15'}.get(match.group(1), 'unknown') if match else 'unknown'
    return weather, hour


def overlay(image, da, marking, binary=False):
    colors = np.zeros_like(image)
    active = da >= .5
    colors[active] = (40, 200, 40)
    if binary:
        selected = marking >= .5
        colors[selected] = (255, 230, 0)  # cyan, not a yellow-lane class
        active |= selected
    else:
        for c, color in [(1, (255,255,255)), (2, (0,230,255)), (3, (0,0,255))]:
            selected = marking == c
            colors[selected] = color
            active |= selected
    result = image.copy()
    result[active] = cv2.addWeighted(image, .52, colors, .48, 0)[active]
    return result


def panel(image, text, width=640):
    height = round(image.shape[0]*width/image.shape[1])
    canvas = cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)
    canvas = cv2.copyMakeBorder(canvas, 30, 0, 0, 0, cv2.BORDER_CONSTANT, value=(25,25,25))
    cv2.putText(canvas, text, (8,21), cv2.FONT_HERSHEY_SIMPLEX,.54,(255,255,255),1,cv2.LINE_AA)
    return canvas


def write_gallery(output, report, cards):
    rows=[]
    labels={'bdd_official640':'BDD Large · 공식384×640','bdd_runtime':'BDD Large · 전방960/측방640',
            'medium_v6':'현재 Medium v6 · 전방960/측방640'}
    for record in report['quality']:
        if record['group'] == 'all':
            rows.append('<tr><td>{}</td><td>{:.2f}%</td><td>{:.2f}%</td></tr>'.format(
                html.escape(labels[record['model']]),100*record['drivable_iou'],100*record['lane_iou']))
    payload=json.dumps(cards,ensure_ascii=False).replace('<','\\u003c')
    template=Path(__file__).with_name('templates')/'twinlite_bdd.html'
    content=template.read_text().replace('__CARDS__',payload).replace('__ROWS__',''.join(rows))
    content=content.replace('__IMAGES__',str(report['images'])).replace('__CARDS_COUNT__',str(len(cards)))
    content=content.replace('__DATE__',report['completed_at']).replace('__SHA__',report['models']['bdd_runtime']['checkpoint_sha256'])
    (output/'index.html').write_text(content)


def main():
    import torch
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--data-root',type=Path,default=Path('/data'))
    p.add_argument('--validation',type=Path)
    p.add_argument('--bdd-checkpoint',type=Path,required=True)
    p.add_argument('--medium-checkpoint',type=Path,required=True)
    p.add_argument('--twinlite-root')
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--gallery-triplets-per-run',type=int,default=12)
    p.add_argument('--limit-triplets',type=int,default=0,help='0=full fixed validation')
    a=p.parse_args()
    torch.set_num_threads(1);cv2.setNumThreads(1)
    validation=a.validation or a.data_root/'dataset_versions/twinlite_morai_v6_road_marking_actor_negative/val.jsonl'
    a.output.mkdir(parents=True,exist_ok=False)
    (a.output/'images').mkdir()
    groups=defaultdict(dict)
    for row in map(json.loads,validation.read_text().splitlines()):
        key=(row['run_id'],int(row['frame_id']))
        if row['view'] in groups[key]:raise ValueError('duplicate view '+row['sample_id'])
        groups[key][row['view']]=row
    cases=sorted(groups.items())
    if a.limit_triplets:cases=cases[:a.limit_triplets]
    by_run=defaultdict(list)
    for key,rows in cases:
        if set(rows)!={'front','left','right'}:raise ValueError('incomplete triplet '+str(key))
        by_run[key[0]].append(key)
    gallery_keys=set()
    for values in by_run.values():
        for i in np.linspace(0,len(values)-1,min(len(values),a.gallery_triplets_per_run),dtype=int):
            gallery_keys.add(values[i])
    common=dict(twinlite_root=a.twinlite_root,device='cuda',amp=True,channels_last=True)
    bdd=TwinLiteBackend(checkpoint=str(a.bdd_checkpoint),model_config='large',
                       input_hw=(384,640),input_hw_by_view={'front':(544,960),'left':(480,640),'right':(480,640)},**common)
    medium=TwinLiteBackend(checkpoint=str(a.medium_checkpoint),model_config='medium',
                          input_hw=(480,640),input_hw_by_view={'front':(544,960)},**common)
    official=TwinLiteBackend(checkpoint=str(a.bdd_checkpoint),model_config='large',
                            input_hw=(384,640),**common)
    runtime=InferenceEngine(bdd,resize_interpolation='linear')
    current=InferenceEngine(medium)
    engines={'bdd_runtime':runtime,'bdd_official640':InferenceEngine(official,resize_interpolation='linear'),'medium_v6':current}
    model_info={'bdd_runtime':asdict(bdd.info),'bdd_official640':asdict(official.info),'medium_v6':asdict(medium.info)}
    model_info['bdd_runtime']['resize_interpolation']='linear'
    model_info['bdd_official640']['resize_interpolation']='linear'
    model_info['medium_v6']['resize_interpolation']='auto'
    accum=defaultdict(lambda:np.zeros((2,3),np.int64));cards=[];timing=[];distribution=Counter()
    started=time.monotonic()
    with (a.output/'samples.jsonl').open('w') as journal:
        for index,((run,frame),rows) in enumerate(cases):
            images={v:cv2.imread(str(a.data_root/r['image'])) for v,r in rows.items()}
            if any(x is None for x in images.values()):raise ValueError('missing RGB '+str((run,frame)))
            predictions={}
            for name,engine in engines.items():
                begin=time.monotonic()
                predictions[name]=engine.infer(images).views
                timing.append(dict(model=name,triplet=index,ms=1000*(time.monotonic()-begin)))
            weather,hour=condition(run)
            for view,row in sorted(rows.items()):
                gtroot=a.data_root/'datasets'/run/'derived/perception_targets_native_v3_actor_negative'
                filename=Path(row['image']).name
                da,mark,ignore=[cv2.imread(str(gtroot/folder/view/filename),0) for folder in
                                ('drivable_masks','road_marking_masks','drivable_ignore_masks')]
                if any(x is None or x.shape!=images[view].shape[:2] for x in (da,mark,ignore)):
                    raise ValueError('native GT missing or mismatched '+row['sample_id'])
                geometry=LetterboxGeometry(**{k:tuple(row['geometry'][k]) for k in
                    ('original_hw','target_hw','resized_hw','pad_ltrb','scale_xy')})
                valid_da=restore_class_ids(cv2.imread(str(a.data_root/row['valid_masks']['drivable']),0),geometry)>0
                valid_lane=restore_class_ids(cv2.imread(str(a.data_root/row['valid_masks']['lane']),0),geometry)>0
                valid_da &= ignore==0
                valid_lane &= (mark<4)&(mark!=3)  # stopline has no corresponding BDD class
                record=dict(sample_id=row['sample_id'],image=row['image'],image_sha256=sha(a.data_root/row['image']),
                            gt_sha256={folder:sha(gtroot/folder/view/filename) for folder in ('drivable_masks','road_marking_masks','drivable_ignore_masks')},
                            run=run,view=view,weather=weather,hour=hour,counts={})
                for name,pred in predictions.items():
                    value=pred[view].probabilities
                    lane=(value['lane']>=.5) if 'lane' in value else np.isin(value['road_marking'],[1,2])
                    delta=np.array([counts(da>0,value['drivable']>=.5,valid_da),
                                    counts(np.isin(mark,[1,2]),lane,valid_lane)])
                    record['counts'][name]=delta.tolist()
                    for group in ('all','view:'+view,'condition:'+weather+hour,'run:'+run):accum[(name,group)]+=delta
                journal.write(json.dumps(record)+'\n')
                distribution[(weather,hour,view)]+=1
                if (run,frame) in gallery_keys:
                    rgb=images[view]
                    p_m=predictions['medium_v6'][view].probabilities
                    p_r=predictions['bdd_runtime'][view].probabilities
                    p_o=predictions['bdd_official640'][view].probabilities
                    note=np.zeros_like(rgb)
                    for j,text in enumerate(['Green: drivable','BDD cyan: any lane','MORAI white / yellow / red','No fine-tuning performed','Native RGB / native GT']):
                        cv2.putText(note,text,(20,45+40*j),cv2.FONT_HERSHEY_SIMPLEX,.7,(255,255,255),1,cv2.LINE_AA)
                    top=np.hstack([panel(rgb,'Collected RGB'),panel(overlay(rgb,da,mark),'MORAI GT'),panel(note,'Legend')])
                    bottom=np.hstack([panel(overlay(rgb,p_m['drivable'],p_m['road_marking']),'Current Medium v6 / 960-640'),
                        panel(overlay(rgb,p_r['drivable'],p_r['lane'],True),'Official BDD Large / 960-640'),
                        panel(overlay(rgb,p_o['drivable'],p_o['lane'],True),'Official BDD Large / 384x640')])
                    dest='images/{:04d}-{}.jpg'.format(index,view)
                    if not cv2.imwrite(str(a.output/dest),np.vstack([top,bottom]),[cv2.IMWRITE_JPEG_QUALITY,85]):raise IOError(dest)
                    cards.append(dict(file=dest,**record))
            if index%12==0:print('completed {}/{} triplets ({:.1f}s)'.format(index+1,len(cases),time.monotonic()-started),flush=True)
    quality=[]
    for (name,group),c in sorted(accum.items()):
        quality.append(dict(model=name,group=group,drivable_iou=float(c[0,0]/max(1,c[0,1])),
                            lane_iou=float(c[1,0]/max(1,c[1,1])),counts=c.tolist()))
    report=dict(schema='twinlite_bdd_morai_comparison_v1',completed_at=datetime.now(timezone.utc).isoformat(),
                images=3*len(cases),triplets=len(cases),gallery_images=len(cards),
                validation=str(validation),validation_sha256=sha(validation),models=model_info,
                distribution=[dict(weather=w,hour=h,view=v,images=n) for (w,h,v),n in sorted(distribution.items())],
                quality=quality,timing=timing,elapsed_seconds=time.monotonic()-started,
                evaluation=dict(threshold=.5,drivable='MORAI native actor-negative GT; this differs from BDD labeling',
                                lane='white or yellow union; GT stopline pixels excluded from binary comparison',
                                preprocessing='BDD: RGB /255, LINEAR letterbox pad114. Current: existing auto interpolation.',
                                limitation='Development comparison; same runs / adjacent frames are not independent scenes. Not a Large-vs-Medium architecture ablation; training domains differ. No family or deployment acceptance.'))
    repo=Path(__file__).resolve().parents[2]
    report['code_sha256']={str(path.relative_to(repo)):sha(path) for path in
        (Path(__file__).resolve(),repo/'src/asmc_camera/src/asmc_camera/camera/geometry.py',
         repo/'src/asmc_camera/src/asmc_camera/camera/inference_engine.py',
         repo/'src/asmc_camera/src/asmc_camera/camera/backends/twinlite.py')}
    (a.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    write_gallery(a.output,report,cards)
    print(json.dumps(dict(images=report['images'],gallery_images=len(cards),quality=[q for q in quality if q['group']=='all']),indent=2))


if __name__=='__main__':
    main()
