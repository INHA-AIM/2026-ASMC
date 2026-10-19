#!/usr/bin/env python3
"""Exhaustive immutable v9 audit and spatial background supervision export.

Every image/label and available synchronized GT is checked. This does not
claim human visual inspection of every frame or exhaustive RGB annotations.
"""
import argparse
from collections import Counter, defaultdict
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np
from PIL import Image
import yaml
from camera_pretrain_policy import (CLASSES, ROOT, sha, validate_box,
    validate_dataset_contract, letterbox_geometry, load_policy, guarded_data_yaml, epoch_order, view_masks)
from prepare_camera_pretrain import read_rows, yolo_lines, verify_export, write_json
from capture_rgb_quality import validate_rendered_rgb
sys.path.insert(0, str(ROOT/'src/data_collection/src'))
from data_collection.mask_baker import OFFICIAL_PALETTE_RGB


def semantic_grid(semantic, safe_colors, visible_known=None, ego=None, target=960):
    """All pixels in an 8px cell must be safe; coarse heads pool again."""
    safe = np.zeros(semantic.shape[:2], dtype=bool)
    packed = semantic[:, :, 0].astype(np.uint32)+(semantic[:, :, 1].astype(np.uint32)<<8)+(semantic[:, :, 2].astype(np.uint32)<<16)
    colors = np.array([r+(g<<8)+(b<<16) for r,g,b in safe_colors], dtype=np.uint32)
    safe |= np.isin(packed, colors)
    if visible_known is not None:
        safe |= visible_known
    if ego is not None:
        safe &= ~ego
    h,w = safe.shape
    _,_,left,top,rw,rh = letterbox_geometry(w,h,target)
    # Conservative area coverage: a native unsafe pixel cannot disappear when
    # downsampling. Native-to-tensor expansion includes every intersected pixel.
    unsafe = cv2.resize((~safe).astype(np.float32), (rw,rh), interpolation=cv2.INTER_AREA)
    # Nearest can introduce false-safe cells on expansion boundaries; area is
    # conservative for shrink, and max dilation protects expansion edges.
    resized = unsafe == 0
    if rw>w or rh>h:
        resized = cv2.erode(resized.astype(np.uint8),np.ones((3,3),np.uint8)).astype(bool)
    tensor = np.zeros((target,target),dtype=bool)
    tensor[top:top+rh,left:left+rw] = resized
    return tensor.reshape(target//8,8,target//8,8).all(axis=(1,3))


def main(a):
    cv2.setNumThreads(1)
    started=time.time();validate_dataset_contract(a.source,ignore_aware=True)
    if a.output.exists():raise ValueError('choose a new immutable revision')
    a.output.mkdir(parents=True);a.evidence.mkdir(parents=True,exist_ok=True)
    rows=list(read_rows(a.source/'manifest.jsonl'));policy=load_policy(a.policy)
    if policy.get('spatial_background_version')!=2:raise ValueError('v2 policy required')
    masks=view_masks(policy);color_values=[OFFICIAL_PALETTE_RGB[c] for c in policy['safe_semantic_classes']]
    forbidden=set();forbidden_ids=set()
    for path in a.reserve:
        for r in read_rows(path):
            forbidden_ids.add(r['id']);forbidden.update(r[k] for k in ('image_sha256','pixel_sha256') if r.get(k))
    # Frame manifests are immutable per run; decode their small metadata once.
    frames={}
    for r in rows:
        if r.get('provenance',{}).get('source_run'):
            rgb=Path(r['image']);run=rgb.parents[3];m=run/'manifest.jsonl'
            if str(run) not in frames:
                expected=r['provenance']['source_manifest_sha256']
                if sha(m)!=expected:
                    candidates=[x/'manifest.jsonl' for x in run.parent.glob(run.name+'-*')]
                    exact=[x for x in candidates if x.is_file() and sha(x)==expected]
                    if len(exact)!=1:raise ValueError('pinned native source manifest unavailable: '+str(run))
                    m=exact[0]  # immutable quality-filtered aborted-run snapshot
                frames[str(run)]={str(f['frame_id']):f for f in read_rows(m)}
    for folder in ('images','labels','supervision','background'):
        for role in ('train','val','test'):(a.output/folder/role).mkdir(parents=True,exist_ok=True)

    def process(old):
        r=dict(old);r['annotations']=[dict(x) for x in old['annotations']];image=Path(r['image'])
        audit=dict(id=r['id'],source=r['source'],split=r['split'],checks=[],status='passed')
        try:
            raw=image.read_bytes()
            if hashlib.sha256(raw).hexdigest()!=r['image_sha256']:raise ValueError('RGB SHA mismatch')
            im=cv2.imdecode(np.frombuffer(raw,np.uint8),cv2.IMREAD_COLOR)
            if im is None or im.shape[:2]!=(r['height'],r['width']):raise ValueError('RGB decode/shape mismatch')
            rgb=cv2.cvtColor(im,cv2.COLOR_BGR2RGB)
            validate_rendered_rgb(Image.fromarray(rgb))
            px=hashlib.sha256(f"{r['width']}x{r['height']}:".encode()+rgb.tobytes()).hexdigest()
            legacy=hashlib.sha256(rgb.tobytes()).hexdigest()
            if r['pixel_sha256'] not in (px,legacy):raise ValueError('decoded pixel SHA mismatch')
            if r['split']=='train' and (r['id'] in forbidden_ids or any(h in forbidden for h in (r['image_sha256'],px,legacy))):
                raise ValueError('reserved evaluation overlap')
            audit['checks']+=['RGB file and decoded pixels','render quality','reserved evaluation isolation']
            for ann in r['annotations']:
                if ann['supervision'] in ('positive','ignore'):
                    validate_box(ann['xyxy'],r['width'],r['height'])
                    if ann.get('training_class') not in CLASSES:raise ValueError('unresolved target class')
                if ann['supervision']=='positive' and (ann.get('visible_pixels')==0 or ann.get('supervision_reasons')):
                    raise ValueError('invalid positive visibility/disposition')
            role={'selection':'val','heldout':'test'}.get(r['split'],r['split'])
            old_label=a.source/'labels'/role/(r['id']+'.txt')
            if sha(old_label)!=r['export_label_sha256'] or old_label.read_text().splitlines()!=yolo_lines(r):
                raise ValueError('fixed16/YOLO source labels disagree')
            audit['checks']+=['all annotations and existing YOLO labels']
            prov=r.get('provenance',{});grid=None
            # Synchronized Capture files have exact per-frame checksum evidence.
            if r['source'].startswith('capture:'):
                sem_path=None;inst_path=None;expected={}
                if prov.get('source_run'):
                    run=image.parents[3];f=frames[str(run)][str(prov['frame_id'])]
                    for kind in ('semantic','instance','bbox','intensity'):
                        rel=f['paths'][kind][r['view']]
                        if sha(run/rel)!=f['sha256'][rel]:raise ValueError('native '+kind+' SHA mismatch')
                    sem_path=run/f['paths']['semantic'][r['view']];inst_path=run/f['paths']['instance'][r['view']]
                elif prov.get('original_annotation',{}).get('source_sha256'):
                    original=prov['original_annotation'];run=image.parents[3]
                    sem_path=run/'frames/semantic'/r['view']/image.name
                    inst_path=run/'frames/instance'/r['view']/image.name
                    for kind,path in [('semantic',sem_path),('instance',inst_path)]:
                        if sha(path)!=original['source_sha256'][kind]:raise ValueError('old Capture '+kind+' SHA mismatch')
                elif prov.get('semantic'):
                    sem_path=Path(prov['semantic']);inst_path=Path(prov['instance'])
                    if sha(sem_path)!=prov['semantic_sha256'] or sha(inst_path)!=prov['instance_sha256']:
                        raise ValueError('negative source GT SHA mismatch')
                if sem_path is not None:
                    sem=np.array(Image.open(sem_path).convert('RGB'));inst=np.array(Image.open(inst_path).convert('RGBA'))
                    if sem.shape!=(r['height'],r['width'],3) or inst.shape!=sem.shape[:2]+(4,):raise ValueError('GT/RGB shape mismatch')
                    packed=np.ascontiguousarray(inst).view(np.uint32).reshape(inst.shape[:2]);known=np.zeros(inst.shape[:2],bool)
                    originals=prov.get('original_annotation',{}).get('annotations',[])
                    by_actor={str(x.get('actor_id')):x for x in originals}
                    for ann in r['annotations']:
                        if ann['supervision']!='positive':continue
                        rgba=ann.get('rgba') or by_actor.get(str(ann.get('actor_id')),{}).get('rgba')
                        if rgba:
                            tag=np.asarray(rgba,dtype=np.uint8).view(np.uint32).item();pixel=packed==tag
                            if not pixel.any():raise ValueError('positive actor absent from instance GT')
                            # Instance pixels are clipped to the preserved visible bbox;
                            # do not expand labels or assume hidden extents.
                            x1,y1,x2,y2=ann['xyxy'];x1,y1=int(np.floor(x1)),int(np.floor(y1));x2,y2=int(np.ceil(x2)),int(np.ceil(y2))
                            cropped=np.zeros_like(pixel);cropped[y1:y2,x1:x2]=pixel[y1:y2,x1:x2];known|=cropped
                    ego=masks[r['view']].geometry(r['height'],r['width'])[0] if r.get('ego_mask_applied') else None
                    grid=semantic_grid(sem,color_values,known,ego,policy['imgsz'])
                    audit['checks']+=['synchronized semantic/instance SHA, dimensions and positive actor presence','conservative safe background grid']
                    audit['safe_cell_fraction']=float(grid.mean())
                    audit['semantic_sha256']=sha(sem_path);audit['instance_sha256']=sha(inst_path)
            r['known_class_background']=bool(grid is None and r['supervision']=='partial')
            if grid is not None:
                rel=Path('background')/role/(r['id']+'.npz');np.savez_compressed(a.output/rel,packed=np.packbits(grid))
                r['background_safe_grid']=str(rel);r['background_safe_grid_sha256']=sha(a.output/rel)
            r['background_policy']='spatial_v2'
            # Preserve every positive, ignore, source label, subtype and split.
            image_rel=Path('images')/role/(r['id']+image.suffix)
            (a.output/image_rel).symlink_to(image)
            label_rel=Path('labels')/role/(r['id']+'.txt');(a.output/label_rel).write_text(old_label.read_text())
            side_rel=Path('supervision')/role/(r['id']+'.json')
            write_json(a.output/side_rel,dict(supervision=r['supervision'],annotations=r['annotations'],
                background_policy=r['background_policy'],known_class_background=r['known_class_background'],
                background_safe_grid=r.get('background_safe_grid'),background_safe_grid_sha256=r.get('background_safe_grid_sha256')))
            r.update(export_image=str(image_rel),export_label_sha256=sha(a.output/label_rel),supervision_sha256=sha(a.output/side_rel))
            audit['known_class_background']=r['known_class_background'];audit['positive_count']=sum(x['supervision']=='positive' for x in r['annotations'])
            return r,audit
        except (OSError,KeyError,ValueError) as exc:
            audit.update(status='quarantined',reason=str(exc));return None,audit

    accepted=[];audits=[];counts=Counter()
    with ThreadPoolExecutor(max_workers=a.workers) as pool, (a.evidence/'audit.jsonl').open('w') as log:
        for i,(row,audit) in enumerate(pool.map(process,rows)):
            log.write(json.dumps(audit)+'\n');audits.append(audit)
            if row is not None:accepted.append(row)
            counts[audit['status']]+=1
            if i%1000==0:
                log.flush();print(json.dumps(dict(processed=i,total=len(rows),counts=dict(counts),seconds=round(time.time()-started))),flush=True)
    (a.output/'manifest.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in accepted))
    write_json(a.output/'policy.json',policy);(a.output/'data.yaml').write_text(yaml.safe_dump(guarded_data_yaml(a.output),sort_keys=False))
    parent_shas={str(a.source):sha(a.source/'manifest.jsonl')}
    verify=verify_export(a.output,accepted,parent_shas)
    # Every exported supervision grid is read back; no silent omission.
    for row in accepted:
        if row.get('background_safe_grid'):
            path=a.output/row['background_safe_grid']
            if sha(path)!=row['background_safe_grid_sha256']:raise ValueError('export grid SHA mismatch')
            with np.load(path) as z:
                if np.unpackbits(z['packed']).size!=(policy['imgsz']//8)**2:raise ValueError('export grid shape mismatch')
    verify.update(full_audit_images=len(rows),quarantined=counts['quarantined'],visual_every_image_claimed=False,
        all_available_synchronized_GT_rechecked=True,positive_boxes_changed=False,split_membership_changed=False)
    write_json(a.output/'verification.json',verify)
    if not verify['passed']:raise ValueError('v9 export verification failed')
    train=[r for r in accepted if r['split']=='train'];freq=Counter();objects=Counter()
    for row in train:
        positives=[x for x in row['annotations'] if x['supervision']=='positive'];freq.update(set(x['training_class'] for x in positives));objects.update(x['training_class'] for x in positives)
    spatial=[x for x in audits if x['split']=='train' and 'safe_cell_fraction' in x and x['status']=='passed']
    summary=dict(images=len(accepted),train_images=len(train),classes=list(CLASSES),class_train_images=dict(freq),class_train_boxes=dict(objects),
        splits=dict(Counter(r['split'] for r in accepted)),quarantined=[x for x in audits if x['status']!='passed'],
        spatial_background_images=len(spatial),known_class_region_images=sum(r['known_class_background'] for r in train),
        mean_safe_cells_fraction=sum(x['safe_cell_fraction'] for x in spatial)/len(spatial) if spatial else None,
        partial_complete_counts=dict(Counter(r['supervision'] for r in train)),audit_sha256=sha(a.evidence/'audit.jsonl'),
        visual_every_image_claimed=False,original_data_unchanged=True)
    write_json(a.output/'summary.json',summary)
    order,order_report=epoch_order(train,policy['seed'],batch_size=4,alignment=16)
    (a.output/'epoch-000-order.jsonl').write_text(''.join(json.dumps(dict(index=i,id=train[i]['id']))+'\n' for i in order))
    write_json(a.output/'epoch-000-summary.json',order_report)
    meta=json.loads((a.source/'pretrain.json').read_text());meta.update(schema_version='camera-pretrain-1',final_version=True,
        original_data_root=str(a.source.parents[1]),parents=parent_shas,policy_sha256=sha(a.policy),
        requires_spatial_background_trainer=2,source_stage=str(a.source),
        artifact_shas={name:sha(a.output/name) for name in ['manifest.jsonl','policy.json','data.yaml','verification.json','epoch-000-order.jsonl']})
    write_json(a.output/'pretrain.json',meta);(a.output/'_SUCCESS').write_text(sha(a.output/'pretrain.json')+'\n')
    validate_dataset_contract(a.output,ignore_aware=True)
    write_json(a.evidence/'summary.json',summary);print(json.dumps(dict(completed=True,train_images=len(train),spatial=len(spatial),quarantined=counts['quarantined']),indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--policy',type=Path,required=True);p.add_argument('--evidence',type=Path,required=True);p.add_argument('--reserve',type=Path,action='append',default=[]);p.add_argument('--workers',type=int,default=4)
    main(p.parse_args())
