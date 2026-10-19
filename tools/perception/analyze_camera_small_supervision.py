#!/usr/bin/env python3
"""Read-only size/ignore sensitivity audit; never changes training or labels."""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path
import math
import cv2
import numpy as np
from PIL import Image, ImageDraw
from camera_pretrain_policy import transform_boxes, sha, view_masks, load_policy

POLICIES = {'short4_area32': (4, 32), 'short4_area64': (4, 64),
            'short6_area64': (6, 64), 'short8_area128': (8, 128)}

def rows(path):
    with open(path) as stream:
        for line in stream:
            yield json.loads(line)

def size_bin(short):
    return next((str(v) for v in (2,4,6,8,12,16,32) if short < v), '32+')

def rejected(box, width, height, policy):
    if policy == 'gallery':
        raise ValueError('gallery requires class context')
    short, area = POLICIES[policy]
    # Proposed thresholds checked at BOTH native and actual 960 tensor scale.
    for b in (box, transform_boxes([box], width, height)[0]):
        w,h=b[2]-b[0],b[3]-b[1]
        if min(w,h)<short or w*h<area:
            return True
    return False

def footprint(boxes, stride, mode='intersection'):
    n=960//stride
    grid=np.zeros((n,n),bool)
    for x1,y1,x2,y2 in boxes:
        if mode=='intersection':
            lo=np.floor(np.array([x1,y1])/stride).astype(int)
            hi=np.ceil(np.array([x2,y2])/stride).astype(int)
        elif mode=='center':
            lo=np.ceil(np.array([x1,y1])/stride-.5).astype(int)
            hi=np.ceil(np.array([x2,y2])/stride-.5).astype(int)
        else: raise ValueError(mode)
        lo=np.clip(lo,0,n);hi=np.clip(hi,0,n)
        grid[lo[1]:hi[1],lo[0]:hi[0]]=True
    return grid

def main(a):
    a.output.mkdir(parents=True,exist_ok=False)
    stats=defaultdict(Counter);views=defaultdict(Counter);reasons=Counter()
    samples=defaultdict(list);crops=defaultdict(list);lineages={};distances=defaultdict(Counter)
    ledger_index={};data_root=a.dataset.resolve().parents[1]
    for folder in ('collection-evidence-20261012','capture_layouts'):
        for candidate in (data_root/folder).rglob('*.jsonl'):
            if candidate.stat().st_size < 8_000_000:
                ledger_index.setdefault(sha(candidate),candidate)
    distance_missing=Counter();n=0
    def lineage(row):
        if not row.get('provenance',{}).get('source_run'):return None
        run=Path(row['image']).parents[3]
        if str(run) not in lineages:
            folder=run/'lineage';ledger=folder/'spawn-ledger.jsonl';captures=folder/'capture_manifest.jsonl'
            expected=row['provenance']['ledger_sha256']
            if not ledger.exists() or sha(ledger)!=expected:
                ledger=ledger_index.get(expected)
            if ledger is None or not captures.exists():lineages[str(run)]=None
            else:
                actors={}
                for actor in rows(ledger):
                    if not actor.get('created') or not actor.get('placement_approved'):continue
                    location=actor.get('settled_location') or actor.get('location')
                    for key in actor.get('native_ids',[]):actors[str(key)]=location
                states={str(c['sequence']):c for c in rows(captures) if c.get('success')}
                lineages[str(run)]=(actors,states)
        return lineages[str(run)]
    for r in rows(a.dataset/'manifest.jsonl'):
        if r['split']!='train':continue
        n+=1;key=r['source']+'|'+r['view'];score=hashlib.sha256(('20261013:'+r['id']).encode()).hexdigest()
        if r.get('background_safe_grid'):
            samples[key].append((score,r));samples[key]=sorted(samples[key],key=lambda x:x[0])[:8]
        state=lineage(r);loc=None;actors={};weather='unknown'
        if state:
            actors,captures=state;c=captures.get(str(r['provenance']['frame_id']),{})
            if c.get('state',{}).get('ego_age_sec',999) is not None and c.get('state',{}).get('ego_age_sec',999)<=.2:
                loc=c.get('state',{}).get('ego',{}).get('position')
        for ann in r['annotations']:
            status=ann.get('supervision');name=ann.get('training_class')
            if status not in ('positive','ignore'):continue
            stat=stats[name];stat[status]+=1;reasons.update(ann.get('supervision_reasons',[]))
            b=ann['xyxy'];tb=transform_boxes([b],r['width'],r['height'])[0];w,h=tb[2]-tb[0],tb[3]-tb[1]
            stat[status+'_short_lt_'+size_bin(min(w,h))]+=1
            if status=='positive':
                for policy in POLICIES:
                    if rejected(b,r['width'],r['height'],policy):stat['would_remove_'+policy]+=1
                l,s=(24,12) if name in ('car','bus','truck') else (16,8)
                if max(b[2]-b[0],b[3]-b[1])<l or min(b[2]-b[0],b[3]-b[1])<s:stat['below_gallery']+=1
                views[r['view']][size_bin(min(w,h))]+=1
                bucket=(name,'small' if min(w,h)<8 else 'medium' if min(w,h)<16 else 'large')
                crops[bucket].append((score+str(ann.get('id')),r,ann));crops[bucket]=sorted(crops[bucket],key=lambda x:x[0])[:2]
            actor=actors.get(str(ann.get('actor_id')))
            if loc and actor:
                d=math.sqrt(sum((float(actor[k])-float(loc[k]))**2 for k in ('x','y','z')))
                band=next((str(x) for x in (20,40,60,80,120,200) if d<x),'200+')
                distances[name+'|'+status+'|'+band]['objects']+=1
                distances[name+'|'+status+'|'+band]['short_lt8']+=int(min(w,h)<8)
                if status=='positive':
                    for policy in POLICIES:
                        distances[name+'|'+status+'|'+band]['would_remove_'+policy]+=int(rejected(b,r['width'],r['height'],policy))
            else:distance_missing[status]+=1
    grid_results=[]
    for group,items in sorted(samples.items()):
        for _,r in items:
            with np.load(a.dataset/r['background_safe_grid']) as z:g=np.unpackbits(z['packed']).reshape(120,120).astype(bool)
            boxes=transform_boxes([x['xyxy'] for x in r['annotations'] if x['supervision']=='ignore'],r['width'],r['height'])
            result=dict(id=r['id'],source=r['source'],view=r['view'],ignore_boxes=len(boxes),strides={})
            for stride in (8,16,32):
                step=stride//8;size=960//stride;safe=g.reshape(size,step,size,step).all(axis=(1,3))
                hit=footprint(boxes,stride);center=footprint(boxes,stride,'center')
                result['strides'][str(stride)]=dict(safe_before=float(safe.mean()),ignore_footprint=float(hit.mean()),
                    safe_lost=float((safe&hit).mean()),safe_remaining=float((safe&~hit).mean()),
                    hypothetical_center_safe_lost=float((safe&center).mean()))
            grid_results.append(result)
    (a.output/'grid-sample.jsonl').write_text(''.join(json.dumps(x)+'\n' for x in grid_results))
    report=dict(train_images=n,classes=stats,view_positive_size_bins=views,ignore_reasons=reasons,
        counterfactual_thresholds=POLICIES,range_approx_counts=distances,range_unknown=distance_missing,
        range_definition='3D static actor pivot to request-time Ego, <=0.2s state age; NOT exposure-time optical depth or visibility. Only exact native IDs from SHA-pinned same-run ledger; no child-ID guessing.',
        grid_sample_count=len(grid_results),grid_sampling='up to 8 deterministic images per source/view, seed20261013; not population-weighted',
        grid_summary={str(s):{key:dict(mean=float(np.mean(vals)),p95=float(np.percentile(vals,95)),maximum=max(vals))
            for key in ['safe_before','ignore_footprint','safe_lost','safe_remaining','hypothetical_center_safe_lost']
            for vals in [[r['strides'][str(s)][key] for r in grid_results]]} for s in (8,16,32)},
        limitation='Assignment positives override ignores and are excluded from this geometric mask estimate. Counts are counterfactual, not a retrained-model comparison.',
        dataset_manifest_sha256=sha(a.dataset/'manifest.jsonl'))
    (a.output/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
    # Crop sheets preserve a 1:1 actual training-size patch plus explicit 3x nearest zoom.
    sheets=[];entries=[];masks=view_masks(load_policy())
    for name in sorted(stats):
        sheet=Image.new('RGB',(900,250*3),'#202530');draw=ImageDraw.Draw(sheet)
        for k,band in enumerate(('small','medium','large')):
            values=crops.get((name,band),[])
            for j,(_,r,ann) in enumerate(values):
                raw=cv2.imread(r['image']);gain=960/max(r['width'],r['height'])
                if r.get('ego_mask_applied'):raw=masks[r['view']].apply(raw)
                raw=cv2.resize(raw,(round(r['width']*gain),round(r['height']*gain)),interpolation=cv2.INTER_LINEAR)
                im=Image.fromarray(cv2.cvtColor(raw,cv2.COLOR_BGR2RGB))
                b=np.array(ann['xyxy'])*gain;cx,cy=(b[:2]+b[2:])/2
                patch=im.crop((round(cx)-40,round(cy)-32,round(cx)+40,round(cy)+32))
                x=j*450;y=k*250;sheet.paste(patch,(x+10,y+45));sheet.paste(patch.resize((240,192),Image.Resampling.NEAREST),(x+110,y+45))
                draw.text((x+8,y+5),f'{name} {band} | tensor {b[2]-b[0]:.1f}x{b[3]-b[1]:.1f}',fill='white')
                draw.text((x+8,y+23),r['view']+' | '+r['source'][:48],fill='white')
                entries.append(dict(class_name=name,band=band,id=r['id'],annotation_id=ann.get('id'),image=r['image'],bbox=ann['xyxy']))
        file=name+'.jpg';sheet.save(a.output/file);sheets.append(file)
    (a.output/'crop-index.json').write_text(json.dumps(entries,indent=2)+'\n')
    (a.output/'index.html').write_text('<meta charset="utf-8"><title>2026-10-13 · Small GT / ignore audit</title><style>body{background:#202530;color:white;font:16px sans-serif}img{max-width:100%}</style><h1>2026-10-13 · Small GT / ignore audit</h1><p>Read-only diagnostic. Actual 960-scale RGB crop + 3x zoom; training Ego mask and OpenCV resize, before stochastic HSV/flip. Target bbox center at crop center; large boxes may extend beyond patch. Crop is not proof of visibility; original context in crop-index.json.</p><a href="summary.json">Full statistics</a>'+''.join('<h2>'+f+'</h2><img src="'+f+'">' for f in sheets))
    print(json.dumps(dict(train_images=n,grid_samples=len(grid_results),output=str(a.output))))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--dataset',type=Path,required=True);p.add_argument('--output',type=Path,required=True);main(p.parse_args())
