#!/usr/bin/env python3
"""Offline map-drum label proposals from synchronized semantic + RGB.

Never converts all standing_object pixels to drums. A geographic gate, RGB
appearance and isolated component geometry propose instances for review.
No active dataset or running training process is modified.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
from functools import lru_cache
import hashlib
import json
from pathlib import Path
import re
import cv2
import numpy as np
from PIL import Image,ImageDraw
from camera_pretrain_policy import sha,view_masks,load_policy


def components(rgb, semantic, ego=None):
    if rgb.shape!=semantic.shape:raise ValueError('RGB/semantic shape mismatch')
    mask=(semantic==[113,178,37]).all(2).astype(np.uint8)
    if ego is not None:mask[ego]=0
    n,labels,stats,_=cv2.connectedComponentsWithStats(mask,8)
    result=[]
    for i,(x,y,w,h,area) in enumerate(stats[1:],1):
        if area<16 or min(w,h)<4:continue
        pixels=rgb[labels==i].astype(np.float32)
        orange=(pixels[:,0]>pixels[:,1]*1.12)&(pixels[:,1]>pixels[:,2]*1.10)&(pixels[:,0]-pixels[:,2]>18)
        fraction=float(orange.mean());fill=float(area/(w*h))
        # Tall isolated cylindrical profile; merged rows, poles and barricades
        # are review/unknown rather than fabricated multiple boxes.
        if not (.38<=w/h<=.90 and fill>=.67 and fraction>=.12):continue
        result.append(dict(xyxy=[int(x),int(y),int(x+w),int(y+h)],visible_pixels=int(area),orange_fraction=fraction,fill_fraction=fill,component_id=int(i)))
    return result


@lru_cache(maxsize=100000)
def position(path):
    try:s=Path(path).read_text()
    except OSError:return None
    vals=[]
    for key in ('Latitude','Longitude'):
        m=re.search(key+r'\s*:\s*([-\d.]+)',s)
        if not m:return None
        vals.append(float(m.group(1)))
    return vals


def bbox_iou(a,b):
    area=max(0,min(a[2],b[2])-max(a[0],b[0]))*max(0,min(a[3],b[3])-max(a[1],b[1]))
    return area/max(1,(a[2]-a[0])*(a[3]-a[1])+(b[2]-b[0])*(b[3]-b[1])-area)


def collect(a):
    if a.output.exists():raise ValueError('immutable output exists')
    cv2.setNumThreads(1);masks=view_masks(load_policy(a.dataset/'policy.json'))
    byid={};active=set()
    for name in ['reserved-selection.jsonl','reserved-heldout.jsonl','manifest.jsonl']:
        p=a.dataset/name
        if not p.exists():continue
        for line in p.open():
            r=json.loads(line);byid[r['id']]=r
            if name=='manifest.jsonl':active.add(r['id'])
    candidates=[]
    for r in byid.values():
        if not r['source'].startswith('capture:') or r.get('view') not in masks:continue
        im=Path(r['image']);run=im.parents[3];pos=position(str(run/'state/gps'/ (im.stem+'.txt')))
        if not pos or not (37.2406<=pos[0]<=37.24165 and 126.7749<=pos[1]<=126.77565):continue
        sem=run/'frames/semantic'/r['view']/im.name
        if not sem.exists():continue
        candidates.append((r,sem,pos))
    a.output.mkdir(parents=True);(a.output/'crops').mkdir();(a.output/'frames').mkdir()
    records=[];summary=dict(geographic_candidate_images=len(candidates),processed_images=0,proposals=0)
    def job(item):
        r,sem,pos=item;image=Path(r['image']);raw=image.read_bytes()
        if hashlib.sha256(raw).hexdigest()!=r['image_sha256']:raise ValueError('RGB SHA mismatch')
        rgb=np.array(Image.open(image).convert('RGB'));sraw=sem.read_bytes();s=np.array(Image.open(sem).convert('RGB'))
        ego=masks[r['view']].geometry(r['height'],r['width'])[0] if r.get('ego_mask_applied') else None
        comps=components(rgb,s,ego);valid=[]
        for c in comps:
            overlap=[x for x in r['annotations'] if x.get('supervision')!='excluded' and bbox_iou(c['xyxy'],x['xyxy'])>=.5]
            if any(x.get('training_class')=='traffic_drum' for x in overlap):continue
            c['conflicting_existing_labels']=[x.get('training_class') for x in overlap]
            token=hashlib.sha256((r['id']+json.dumps(c['xyxy'])).encode()).hexdigest()[:18]
            c.update(proposal_id=token,training_class='traffic_drum',subtype='map_fixed_orange_drum',status='requires_review',method='geographic_region + isolated standing_object semantic component + orange RGB; not teacher GT')
            x1,y1,x2,y2=c['xyxy'];pad=max(x2-x1,y2-y1)*.25+5
            crop=(max(0,int(x1-pad)),max(0,int(y1-pad)),min(rgb.shape[1],int(x2+pad)),min(rgb.shape[0],int(y2+pad)))
            tile=Image.fromarray(rgb).crop(crop);ImageDraw.Draw(tile).rectangle([x1-crop[0],y1-crop[1],x2-crop[0]-1,y2-crop[1]-1],outline='yellow',width=2);tile.save(a.output/'crops'/(token+'.jpg'));valid.append(c)
        if not valid:return None
        pic=Image.fromarray(rgb);d=ImageDraw.Draw(pic)
        for c in valid:d.rectangle(c['xyxy'],outline='cyan',width=2)
        pic.thumbnail((960,540));pic.save(a.output/'frames'/(r['id']+'.jpg'))
        return dict(id=r['id'],image=r['image'],image_sha256=r['image_sha256'],source=r['source'],view=r['view'],width=r['width'],height=r['height'],split=r['split'],split_group=r['split_group'],active_manifest_member=r['id'] in active,position=pos,semantic=str(sem),semantic_sha256=hashlib.sha256(sraw).hexdigest(),proposals=valid)
    with (a.output/'proposals.jsonl').open('w') as stream,ThreadPoolExecutor(max_workers=2) as pool:
        for rec in pool.map(job,candidates):
            summary['processed_images']+=1
            if rec:
                stream.write(json.dumps(rec,ensure_ascii=False)+'\n');stream.flush();records.append(rec);summary['proposals']+=len(rec['proposals'])
            if summary['processed_images']%100==0:print(json.dumps(summary),flush=True)
    summary.update(images_with_proposals=len(records),active_dataset=str(a.dataset.resolve()),parent_manifest_sha256=sha(a.dataset/'manifest.jsonl'),automatic_adoption=False,active_training_changed=False)
    (a.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary),flush=True)



def silhouette_ratio(mask):
    """Reject narrow joins between overlapping barrels; never split a merged mask."""
    widths=mask.sum(axis=1)
    if len(widths)<4 or not widths.max():return 0.
    inner=widths[max(1,int(len(widths)*.15)):max(2,int(len(widths)*.85))]
    return float(inner.min()/widths.max()) if len(inner) else 0.


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--dataset',type=Path,required=True);p.add_argument('--output',type=Path,required=True);collect(p.parse_args())
