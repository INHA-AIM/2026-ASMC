#!/usr/bin/env python3
"""Audit existing detector labels for genuine pixel-mask evidence, without training."""
import argparse
from collections import Counter,defaultdict
import hashlib
import html
import json
from pathlib import Path
import cv2
import numpy as np
from PIL import Image


def mask_evidence(row,ann):
    if ann.get('rgba') is not None and len(ann['rgba'])==4:
        path=row['image'].replace('/frames/intensity/','/frames/instance/')
        return 'native_rgba',path
    if ann.get('native_mask') and ann.get('mask_index') is not None:
        return 'packed_mask',str(Path(ann['native_mask_root'])/ann['native_mask'])
    if ann.get('native_segmentation'):
        return 'polygon_requires_validation',None
    return 'box_only',None


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('manifest',type=Path)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--samples-per-class',type=int,default=3)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False);(a.output/'masks').mkdir();(a.output/'gallery').mkdir()
    sources=Counter();supervision=Counter();evidence=defaultdict(Counter);classes=defaultdict(Counter)
    eligible=Counter();samples=defaultdict(list);class_samples=Counter();image_rows=0
    for line in a.manifest.open():
        row=json.loads(line);image_rows+=1;role=row['split'];kind=row['source'].split(':')[0]
        sources[(role,kind)]+=1;supervision[(role,row.get('supervision'))]+=1
        positive=[x for x in row['annotations'] if x.get('supervision')=='positive']
        methods=[]
        for ann in positive:
            method,path=mask_evidence(row,ann);methods.append(method)
            evidence[role][method]+=1;label=ann.get('training_class',ann.get('canonical','unknown'))
            classes[role][label]+=1
            # Inspection sample only: full inventory above retains tiny/partial labels.
            if role=='train' and method in ('native_rgba','packed_mask') and class_samples[label]<a.samples_per_class:
                samples[row['id']].append((row,ann,method,path));class_samples[label]+=1
        if positive and row.get('supervision')=='complete' and all(m in ('native_rgba','packed_mask') for m in methods):
            eligible[role]+=1
    tested=[];gallery=[];palette=Counter()
    for index,items in enumerate(samples.values()):
        row=items[0][0];image=cv2.imread(row['image'])
        if image is None:
            tested.append(dict(row_id=row['id'],passed=False,reason='missing_rgb'));continue
        render=image.copy()
        # Inspect semantic palette on the same matched samples. Coarse standing != wall/ceiling GT.
        semantic=row.get('provenance',{}).get('semantic') or row['image'].replace('/frames/intensity/','/frames/semantic/')
        if Path(semantic).is_file():
            rgb=np.asarray(Image.open(semantic).convert('RGB'))
            unique,number=np.unique(rgb.reshape(-1,3),axis=0,return_counts=True)
            palette.update({','.join(map(str,k)):int(v) for k,v in zip(unique,number)})
        rgba_counts=Counter(tuple(ann['rgba']) for _,ann,method,_ in items if method=='native_rgba')
        for k,(_,ann,method,path) in enumerate(items):
            result=dict(row_id=row['id'],source=row['source'],class_name=ann.get('training_class',ann.get('canonical')),
                        method=method,source_mask=path,split=row['split'],passed=False)
            try:
                if method=='native_rgba':
                    instance=np.asarray(Image.open(path).convert('RGBA'))
                    mask=np.all(instance==np.asarray(ann['rgba'],np.uint8),axis=2)
                else:
                    with np.load(path,allow_pickle=False) as data:
                        mask=np.unpackbits(data['masks'][int(ann['mask_index'])],axis=1)[:,:int(data['width'])].astype(bool)
                if mask.shape!=image.shape[:2]:raise ValueError('mask/RGB dimensions disagree')
                y,x=np.nonzero(mask)
                if not len(x):raise ValueError('empty native mask')
                bounds=np.asarray([x.min(),y.min(),x.max()+1,y.max()+1],float)
                target=np.asarray(ann['xyxy'],float)
                delta=float(np.max(np.abs(bounds-target)))
                if delta>2:raise ValueError('native mask extent differs from curated box by >2 pixels')
                name=f'{index:03d}-{k:02d}.png';cv2.imwrite(str(a.output/'masks'/name),mask.astype(np.uint8)*255)
                render[mask]=(render[mask]*.6+np.asarray([40,210,50])*.4).astype(np.uint8)
                cv2.rectangle(render,tuple(bounds[:2].astype(int)),tuple(bounds[2:].astype(int)),(0,220,255),1)
                result.update(passed=True,visible_pixels=int(len(x)),bounds=bounds.tolist(),box_max_abs_delta_px=delta,
                              native_mask_sha256=hashlib.sha256(Path(path).read_bytes()).hexdigest(),export_mask='masks/'+name)
            except (OSError,KeyError,ValueError,IndexError) as exc:result['reason']=str(exc)
            tested.append(result)
        name=f'{index:03d}.jpg';cv2.imwrite(str(a.output/'gallery'/name),render)
        gallery.append((name,row['id']))
    summary=dict(images=image_rows,manifest_sha256=hashlib.sha256(a.manifest.read_bytes()).hexdigest(),
        sources={f'{s}/{k}':v for (s,k),v in sources.items()},supervision={f'{s}/{k}':v for (s,k),v in supervision.items()},
        positive_annotations_by_evidence={s:dict(c) for s,c in evidence.items()},positive_annotations_by_class={s:dict(c) for s,c in classes.items()},
        complete_positive_frames_with_mask_metadata=dict(eligible),
        inspection=dict(sampled_train_annotations=len(tested),passed=sum(r['passed'] for r in tested),
                        class_sampling='first up to 3 train annotations/class with native evidence, no heldout training'),
        inspected_semantic_palette_pixels=dict(palette),
        limitations=['metadata evidence is not a full mask-file/annotation audit','partial label images require ignore-aware loss or complete-frame filtering',
            'box/teacher-only records are not pixel GT','coarse standing_object includes fixed objects; not a wall/ceiling-only class',
            'no model training or performance gain is claimed'])
    (a.output/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
    (a.output/'inspected-masks.json').write_text(json.dumps(tested,ensure_ascii=False,indent=2)+'\n')
    (a.output/'gallery/index.html').write_text('<!doctype html><meta charset="utf-8"><p>Training split only; native mask evidence inspection, no training performed.</p>'+''.join(
        '<figure><figcaption>'+html.escape(label)+'</figcaption><img width="960" src="'+name+'"></figure>' for name,label in gallery))
    print(json.dumps({k:v for k,v in summary.items() if k not in ('inspected_semantic_palette_pixels','positive_annotations_by_class')},ensure_ascii=False,indent=2))


if __name__=='__main__':main()
