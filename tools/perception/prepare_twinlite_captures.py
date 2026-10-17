#!/usr/bin/env python3
"""Audit all collected RGB/Semantic pairs and build an immutable mixed-size dataset.

Source RGB stays in explicitly mounted read-only roots. One compact PNG stores
the two targets and validity, avoiding redundant RGB copies on the system disk.
No teacher labels, bbox size filter, GPU work or source modifications are used.
"""
import argparse
from collections import Counter, defaultdict
from concurrent.futures import ThreadPoolExecutor, wait, FIRST_COMPLETED
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import sqlite3
import sys
import time

import cv2
import numpy as np
from PIL import Image

REPO=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(REPO/'src/data_collection/src'),str(REPO/'tools/perception')]
from data_collection.mask_baker import semantic_policy_masks, semantic_unknown_colors, letterbox_mask
from capture_rgb_quality import validate_rendered_rgb

VIEWS={'CAMERA_1':'front','CAMERA_2':'left','CAMERA_3':'right'}
NATIVE={'front':(720,1280),'left':(480,640),'right':(480,640)}
TARGET={'front':(544,960),'left':(480,640),'right':(480,640)}
CLASSES={'background':0,'white_lane':1,'yellow_lane':2,'stopline':3}


def now():return datetime.now(timezone.utc).isoformat()
def sha(data):return hashlib.sha256(data).hexdigest()
def dump(path,value):
    temporary=path.with_suffix(path.suffix+'.part')
    temporary.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n');temporary.replace(path)
def read_json(path,default=None):
    return json.loads(path.read_text()) if path.is_file() else ({} if default is None else default)


def source_identity(name):
    match=re.fullmatch(r'(.+)_(\d+)',name)
    if match:return match.group(1),int(match.group(2))
    return 'unregistered_'+name,0


def gather(data,raw,aim):
    """Prefer native raw paths; use Silver to recover absent raw pairs/lineage."""
    catalog={};contexts={};exclusions=set();aim_inventory=[]
    old=data/'dataset_versions/twinlite_morai_v6_road_marking_actor_negative'
    oldmeta=read_json(old/'dataset.json')
    fixed={r['run_id']:r['split'] for r in oldmeta.get('runs',[])}
    for run,frames in oldmeta.get('explicit_frame_exclusions',{}).items():
        exclusions.update((run,int(frame)) for frame in frames)
    # Old evaluation membership is preserved; new collection groups stay train.
    for run in sorted((data/'capture_runs').iterdir()):
        if not run.is_dir():continue
        collection=read_json(run/'collection.json')
        contexts[run.name]=dict(split=fixed.get(run.name,collection.get('split','train')),
            split_group=collection.get('split_group') or 'run:'+run.name,
            collection_training_eligible=collection.get('training_eligible'),
            segmentation_admission='independent RGB/semantic audit; actor subtype approval is not required')
    for run in sorted((data/'datasets').iterdir()):
        manifest=run/'manifest.jsonl'
        if not manifest.is_file():continue
        with manifest.open() as stream:
            for line in stream:
                row=json.loads(line);custom=row.get('custom_name')
                if not custom:continue
                canonical,frame=source_identity(custom)
                for view in VIEWS.values():
                    paths=row.get('paths',{});rgb=paths.get('intensity',{}).get(view)
                    sem=paths.get('semantic',{}).get(view)
                    if not rgb or not sem:continue
                    key=(custom,view)
                    record=dict(custom_name=custom,run_id=canonical,frame_id=frame,view=view,
                        rgb_ref=dict(root='silver',path=str((run/rgb).relative_to(data))),
                        semantic_ref=dict(root='silver',path=str((run/sem).relative_to(data))),
                        lineage=dict(manifest=str(manifest),source_run=run.name,
                                     requested_at_utc=row.get('requested_at_utc'),pose=row.get('state_snapshot',{}).get('ego')))
                    catalog.setdefault(key,record)
    for sensor,view in VIEWS.items():
        directory=raw/sensor
        if not directory.is_dir():continue
        names={e.name for e in os.scandir(directory) if e.is_file()}
        stems={n[:-len('_Intensity.png')] for n in names if n.endswith('_Intensity.png')}
        stems|={n[:-len('_Semantic.png')] for n in names if n.endswith('_Semantic.png')}
        for custom in sorted(stems):
            run,frame=source_identity(custom);key=(custom,view)
            previous=catalog.get(key,{})
            # If a modality is absent in raw but present in Silver, keep the
            # paired Silver copy. Otherwise explicitly record missing modalities.
            if previous and not all(custom+'_'+suffix+'.png' in names for suffix in ('Intensity','Semantic')):continue
            catalog[key]=dict(custom_name=custom,run_id=run,frame_id=frame,view=view,
                rgb_ref=dict(root='capture_raw',path=sensor+'/'+custom+'_Intensity.png'),
                semantic_ref=dict(root='capture_raw',path=sensor+'/'+custom+'_Semantic.png'),
                lineage=previous.get('lineage',dict(source_run=run)))
    # Only the agreed AIM folders are inspected. RGB-only Sensor Record is not
    # converted to negative segmentation GT, and unfinished transfers stay out.
    if aim is not None and aim.is_dir():
        for folder in ('static_010101','dynamic_obstacle','tunnel_static_obstacle'):
            base=aim/folder
            if not base.is_dir():continue
            for directory,subdirs,files in os.walk(base):
                if 'CAMERA_1' in subdirs:
                    subdirs[:]=['CAMERA_1']  # No need to walk LiDAR/episode recordings.
                if Path(directory).name!='CAMERA_1':continue
                names=set(files);semantic=[n for n in names if n.endswith('_Semantic.png')]
                parent=Path(directory).parent
                complete=(parent/'.NAS_TRANSFER_COMPLETE.json').is_file()
                aim_inventory.append(dict(directory=str(parent.relative_to(aim)),rgb_files=sum(n.endswith('_Intensity.png') or n.lower().endswith('.jpg') for n in names),
                    semantic_files=len(semantic),transfer_complete=complete,
                    admission='candidate' if semantic and complete else 'missing semantic GT' if not semantic else 'transfer not confirmed'))
                if not semantic or not complete:continue
                group='aim:'+folder+':'+parent.parent.name
                for sensor,view in VIEWS.items():
                    camera=parent/sensor
                    if not camera.is_dir():continue
                    for entry in os.scandir(camera):
                        if not entry.name.endswith('_Semantic.png'):continue
                        custom=entry.name[:-len('_Semantic.png')]
                        run='aim_'+folder+'_'+parent.name
                        _,frame=source_identity(custom)
                        key=(run+':'+custom,view)
                        contexts[run]=dict(split='train',split_group=group)
                        catalog[key]=dict(custom_name=run+':'+custom,run_id=run,frame_id=frame,view=view,
                            rgb_ref=dict(root='aim',path=str((camera/(custom+'_Intensity.png')).relative_to(aim))),
                            semantic_ref=dict(root='aim',path=str(Path(entry.path).relative_to(aim))),lineage=dict(source_run=run))
                subdirs[:]=[]
    for record in catalog.values():
        run=record['run_id'];context=contexts.get(run,dict(split=fixed.get(run,'train'),split_group='run:'+run))
        record.update(context)
        if record['split'] not in ('train','val','test'):raise ValueError('unknown split '+record['split'])
        record['excluded']= (run,record['frame_id']) in exclusions
        name=run.lower();record['weather']='SUNNY' if 'sunny' in name else 'FOGGY' if 'foggy' in name else 'unknown'
        match=re.search(r'_sim(11am|1pm|3pm|5pm)_',name)
        record['hour']={'11am':11,'1pm':13,'3pm':15,'5pm':17}[match.group(1)] if match else 'unknown'
    records=sorted(catalog.values(),key=lambda r:({'val':0,'test':1,'train':2}[r['split']],r['custom_name'],r['view']))
    return records,dict(old_dataset=str(old),old_dataset_sha256=sha((old/'dataset.json').read_bytes()),
                        old_runs=fixed,explicit_frame_exclusions=sorted(exclusions),aim=aim_inventory)


def source_path(roots,ref):
    root=roots[ref['root']].resolve();path=(root/ref['path']).resolve()
    path.relative_to(root)
    return path


def process(record,roots,policy):
    """Decode and audit each pair once. No writes or GPU work in workers."""
    identity=dict(custom_name=record['custom_name'],view=record['view'],run_id=record['run_id'])
    if record['excluded']:return dict(rejected='preserved explicit evaluation exclusion',**identity)
    try:
        rgb_path=source_path(roots,record['rgb_ref']);sem_path=source_path(roots,record['semantic_ref'])
        data=[];images=[]
        for path in (rgb_path,sem_path):
            before=path.stat();blob=path.read_bytes();after=path.stat()
            if before.st_mtime_ns!=after.st_mtime_ns or before.st_size!=len(blob):raise ValueError('source changed during read')
            image=cv2.imdecode(np.frombuffer(blob,np.uint8),cv2.IMREAD_UNCHANGED)
            if image is None or image.ndim!=3 or image.shape[2] not in (3,4):raise ValueError('unreadable color PNG')
            data.append(blob);images.append(image[:,:,:3])
        rgb,semantic=images
        if rgb.shape!=semantic.shape or rgb.shape[:2]!=NATIVE[record['view']]:raise ValueError('RGB/semantic/native size mismatch')
        validate_rendered_rgb(Image.fromarray(cv2.cvtColor(rgb,cv2.COLOR_BGR2RGB)))
        unknown=semantic_unknown_colors(semantic)
        if unknown:raise ValueError('unknown semantic palette: '+str(unknown[:3]))
        # Preserve the previous target policy's unsupported blue-lane guard.
        if np.any(np.all(semantic==np.array([255,178,0],np.uint8),axis=2)):raise ValueError('blue lane needs an explicit target policy')
        masks=semantic_policy_masks(semantic,policy)
        da,geometry=letterbox_mask(masks['drivable'],TARGET[record['view']])
        marking,other=letterbox_mask(masks['road_marking'],TARGET[record['view']])
        valid,last=letterbox_mask(np.ones(rgb.shape[:2],np.uint8),TARGET[record['view']])
        if geometry!=other or geometry!=last:raise ValueError('RGB/target geometry mismatch')
        packed=(da|(marking<<1)|(valid<<3)).astype(np.uint8)
        ok,encoded=cv2.imencode('.png',packed,[cv2.IMWRITE_PNG_COMPRESSION,3])
        if not ok:raise ValueError('target encoding failed')
        pixel_sha=sha(np.asarray(rgb.shape,dtype=np.int64).tobytes()+rgb.tobytes())
        target_sha=sha(packed.tobytes());key=sha((record['custom_name']+'/'+record['view']).encode())[:24]
        path='targets/'+record['run_id']+'/'+record['view']+'/'+key+'.png'
        row=dict(schema_version='morai-perception-sample-1.2.0',sample_id=record['custom_name']+'/'+record['view'],
            run_id=record['run_id'],frame_id=record['frame_id'],view=record['view'],split=record['split'],split_group=record['split_group'],
            image=record['rgb_ref']['root']+':'+record['rgb_ref']['path'],source_ref=record['rgb_ref'],semantic_source_ref=record['semantic_ref'],
            image_sha256=sha(data[0]),semantic_sha256=sha(data[1]),pixel_sha256=pixel_sha,supervision_sha256=target_sha,
            geometry=geometry,packed_targets=path,targets={'drivable':path,'road_marking':path},valid_mask=path,
            valid_masks={'drivable':path,'road_marking':path},task_valid={'drivable':True,'road_marking':True},
            target_encodings={'road_marking':{'type':'categorical_uint8','class_values':CLASSES},
                              'packed_targets':{'type':'uint8_bitfield','drivable_bit':0,'road_marking_bits':[1,2],'valid_bit':3}},
            weather=record['weather'],hour=record['hour'],lineage=record.get('lineage',{}),
            collection_training_eligible=record.get('collection_training_eligible'),
            segmentation_admission='RGB+semantic independently audited; bbox/actor approval is separate',
            class_pixels={name:int(np.count_nonzero((marking==c)&(valid>0))) for name,c in CLASSES.items()},
            drivable_pixels=int(da.sum()),valid_pixels=int(valid.sum()))
        return dict(row=row,encoded=encoded.tobytes(),**identity)
    except (ValueError,OSError) as exc:
        return dict(rejected=str(exc),**identity)


def bounded_results(executor,records,roots,policy,capacity):
    iterator=iter(records);pending=set()
    for _ in range(capacity):
        record=next(iterator,None)
        if record is None:break
        pending.add(executor.submit(process,record,roots,policy))
    while pending:
        done,pending=wait(pending,return_when=FIRST_COMPLETED)
        for future in done:
            yield future.result()
            record=next(iterator,None)
            if record is not None:pending.add(executor.submit(process,record,roots,policy))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--silver-root',type=Path,required=True)
    p.add_argument('--capture-root',type=Path,required=True)
    p.add_argument('--aim-root',type=Path)
    p.add_argument('--output-root',type=Path,required=True,help='prepared data root, not original data')
    p.add_argument('--dataset-version',default='twinlite_morai_v7_large_capture_r1')
    p.add_argument('--policy',type=Path,default=REPO/'config/perception_target_policy_v3_actor_negative.json')
    p.add_argument('--gallery',type=Path,required=True)
    p.add_argument('--workers',type=int,default=2)
    p.add_argument('--limit',type=int,default=0,help='smoke only; default consumes all candidates')
    a=p.parse_args();cv2.setNumThreads(1)
    if a.workers<1:raise ValueError('workers must be positive')
    output=a.output_root/'dataset_versions'/a.dataset_version
    output.mkdir(parents=True,exist_ok=False);a.gallery.mkdir(parents=True,exist_ok=False)
    roots={'capture_raw':a.capture_root,'silver':a.silver_root}
    if a.aim_root:roots['aim']=a.aim_root
    policy=read_json(a.policy)
    records,inventory=gather(a.silver_root,a.capture_root,a.aim_root)
    if a.limit:records=records[:a.limit]
    dump(output/'inventory.json',dict(candidates=len(records),sources={k:str(v) for k,v in roots.items()},**inventory))
    (output/'candidate-manifest.jsonl').write_text(''.join(json.dumps(r,ensure_ascii=False)+'\n' for r in records))
    (a.gallery/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>TwinLite Large 전체 Capture 준비</title><style>body{background:#111827;color:white;font:17px system-ui;padding:28px;max-width:1400px;margin:auto}img{max-width:100%}pre{white-space:pre-wrap}</style><a href="../index.html">ASMC 실험 목록</a><h1>TwinLite Large · 전체 Capture 데이터 준비</h1><p>GPU 학습 대기 · CPU 전수검수 및 라벨 생성 중</p><progress id="bar" max="100" style="width:100%;height:26px"></progress><pre id="status"></pre><div id="preview"></div><script>async function tick(){const r=await fetch("status.json",{cache:"no-store"}).then(x=>x.json());document.getElementById("bar").value=100*r.processed/Math.max(1,r.total);document.getElementById("status").textContent=JSON.stringify(r,null,2);if(r.completed){const q=await fetch("preview.json").then(x=>x.json());document.getElementById("preview").replaceChildren();for(const v of q){let p=document.createElement("p"),im=document.createElement("img");p.textContent=v.sample_id;im.src=v.image;im.loading="lazy";document.getElementById("preview").append(p,im)}}else setTimeout(tick,5000)}tick()</script>')
    db=sqlite3.connect(str(output/'audit.sqlite'));db.execute('create table samples(pixel text primary key, supervision text, rank text, row text)');db.execute('create table conflicts(pixel text primary key)')
    reasons=Counter();processed=0;start=time.monotonic();previews=[];preview_runs=Counter()
    def status(completed=False,**extras):
        dump(a.gallery/'status.json',dict(completed=completed,training_started=False,processed=processed,total=len(records),
            elapsed_seconds=round(time.monotonic()-start,1),accepted_unique=db.execute('select count(*) from samples').fetchone()[0],
            rejected=dict(reasons),updated_at=now(),**extras))
    status()
    with (output/'rejected.jsonl').open('w') as reject,ThreadPoolExecutor(max_workers=a.workers) as executor:
        for result in bounded_results(executor,records,roots,policy,a.workers*4):
            processed+=1
            if 'rejected' in result:
                reason=result['rejected'];reasons[reason.split(':',1)[0]]+=1;reject.write(json.dumps(result)+'\n')
            else:
                row=result['row'];pixel=row['pixel_sha256'];target=row['supervision_sha256']
                rank=str({'val':0,'test':1,'train':2}[row['split']])+':'+row['sample_id']
                previous=db.execute('select supervision,rank,row from samples where pixel=?',(pixel,)).fetchone()
                conflict=db.execute('select 1 from conflicts where pixel=?',(pixel,)).fetchone()
                if conflict or (previous and previous[0]!=target):
                    reason='same RGB pixels with conflicting target'
                    db.execute('delete from samples where pixel=?',(pixel,));db.execute('insert or ignore into conflicts values(?)',(pixel,))
                    reasons[reason]+=1;reject.write(json.dumps(dict(reason=reason,sample_id=row['sample_id']))+'\n')
                elif previous and previous[1]<=rank:
                    reasons['exact RGB/target duplicate']+=1
                    reject.write(json.dumps(dict(reason='exact RGB/target duplicate',sample_id=row['sample_id'],retained=json.loads(previous[2])['sample_id']))+'\n')
                else:
                    if previous:reasons['exact RGB/target duplicate replaced by canonical evaluation-first sample']+=1
                    path=a.output_root/row['packed_targets'];path.parent.mkdir(parents=True,exist_ok=True)
                    path.write_bytes(result['encoded']);row['packed_target_file_sha256']=sha(result['encoded'])
                    db.execute('insert or replace into samples values(?,?,?,?)',(pixel,target,rank,json.dumps(row,ensure_ascii=False)))
                    key=(row['run_id'],row['view'])
                    if preview_runs[key]<2:
                        source=cv2.imread(str(source_path(roots,row['source_ref'])));pack=cv2.imdecode(np.frombuffer(result['encoded'],np.uint8),0)
                        geometry=row['geometry'];rh,rw=geometry['resized_hw'];left,top,_,_=geometry['pad_ltrb']
                        resized=cv2.resize(source,(rw,rh),interpolation=cv2.INTER_AREA)
                        canvas=np.full(tuple(geometry['target_hw'])+(3,),114,np.uint8);canvas[top:top+rh,left:left+rw]=resized
                        mark=(pack>>1)&3;da=pack&1;colors=np.zeros_like(canvas);colors[da>0]=(40,200,40)
                        for c,color in [(1,(255,255,255)),(2,(0,230,255)),(3,(0,0,255))]:colors[mark==c]=color
                        overlay=canvas.copy();active=(da>0)|(mark>0);overlay[active]=cv2.addWeighted(canvas,.5,colors,.5,0)[active]
                        image='{:04d}.jpg'.format(len(previews));cv2.imwrite(str(a.gallery/image),np.hstack([canvas,overlay]))
                        previews.append(dict(sample_id=row['sample_id'],image=image));preview_runs[key]+=1
            if processed%300==0:
                db.commit();status();print('audited {}/{} pairs in {:.1f}s'.format(processed,len(records),time.monotonic()-start),flush=True)
    db.commit();summary=Counter();pixels=defaultdict(Counter);positive=defaultdict(Counter);groups={};source_counts=Counter();view_counts=Counter();conditions=Counter();files={}
    with (output/'train.jsonl').open('w') as train,(output/'val.jsonl').open('w') as val,(output/'test.jsonl').open('w') as test:
        streams={'train':train,'val':val,'test':test}
        for (text,) in db.execute('select row from samples order by rank'):
            row=json.loads(text);split=row['split'];group=row['split_group']
            if group in groups and groups[group]!=split:raise ValueError('split group leakage: '+group)
            groups[group]=split;streams[split].write(text+'\n');summary[split]+=1
            source_counts[row['source_ref']['root']]+=1;view_counts[(split,row['view'])]+=1;conditions[(split,row['weather'],str(row['hour']))]+=1
            for name,count in row['class_pixels'].items():pixels[split][name]+=count;positive[split][name]+=int(count>0)
            pixels[split]['drivable']+=row['drivable_pixels'];positive[split]['drivable']+=int(row['drivable_pixels']>0)
    for split in ('train','val','test'):files[split]=sha((output/(split+'.jsonl')).read_bytes())
    required=[r[0] for r in db.execute("select distinct json_extract(row,'$.source_ref.root') from samples")]
    meta=dict(schema_version='morai-perception-dataset-1.2.0',dataset_version=a.dataset_version,created_at_utc=now(),
        target_hw=[480,640],target_hw_by_view={k:list(v) for k,v in TARGET.items()},training_heads=['drivable','road_marking'],
        source_roots_required=required,source_paths_recorded={k:str(v) for k,v in roots.items()},
        rgb_policy=dict(copied=False,interpolation='INTER_AREA when shrinking, otherwise INTER_LINEAR',normalization='RGB /255',padding=114),
        semantic_policy_sha256=sha(a.policy.read_bytes()),semantic_policy=policy,
        packed_target_encoding=dict(drivable_bit=0,road_marking_bits=[1,2],valid_bit=3,reserved_bits_zero=[4,5,6,7]),
        split_counts=dict(summary),class_pixels=dict(pixels),positive_images=dict(positive),source_images=dict(source_counts),
        view_images=[dict(split=s,view=v,images=n) for (s,v),n in sorted(view_counts.items())],
        conditions=[dict(split=s,weather=w,hour=h,images=n) for (s,w,h),n in sorted(conditions.items())],
        split_groups=groups,rejections=dict(reasons),candidate_images=len(records),split_manifest_sha256=files,
        verification=dict(all_pairs_decoded=True,all_semantic_palettes_checked=True,all_RGB_render_checks=True,
                          original_GT_preserved=True,exact_RGB_duplicate_and_conflict_checked=True,split_group_leakage=False,
                          temporal_near_duplicate_independence='not claimed; lineage/group retained',
                          unknown_conditions='not inferred',evaluation='old development val retained; no fresh independent test fabricated'))
    dump(output/'dataset.json',meta);dump(output/'verification.json',meta['verification']);dump(output/'_SUCCESS',dict(completed_at=now(),dataset_sha256=sha((output/'dataset.json').read_bytes())))
    dump(a.gallery/'preview.json',previews);dump(a.gallery/'dataset-summary.json',meta);status(True,splits=dict(summary),class_pixels=dict(pixels),positive_images=dict(positive),views=meta['view_images'],conditions=meta['conditions'],dataset_version=a.dataset_version)
    db.close();print(json.dumps(dict(dataset=str(output),splits=dict(summary),rejections=dict(reasons)),indent=2),flush=True)


if __name__=='__main__':main()
