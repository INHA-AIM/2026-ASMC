#!/usr/bin/env python3
"""Export independently linked Capture GT views; unresolved images stay quarantined."""
import argparse
from collections import Counter
import hashlib
import html
import json
from pathlib import Path
import sys
from datetime import datetime, timezone
import numpy as np
from PIL import Image,ImageDraw

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src/data_collection/src'))
from data_collection.capture_collection import CLASSES,native_annotation_class
from prepare_camera_seg_gt import parse_bbox,sha,SEMANTIC
from capture_rgb_quality import validate_rendered_rgb


def convert(instance,semantic,boxes,ledger,exclude_stroller_infants=True,excluded_annotations=None):
    if instance.ndim!=3 or instance.shape[2]!=4 or semantic.shape!=instance.shape[:2]+(3,):
        raise ValueError('native GT dimensions/channels mismatch')
    annotations=[];linked=np.zeros(instance.shape[:2],bool)
    packed=(np.ascontiguousarray(instance).view(np.uint32).reshape(instance.shape[:2])
            if instance.dtype==np.uint8 else None)
    colors=Counter(tuple(r['rgba']) for r in boxes)
    for box in boxes:
        if colors[tuple(box['rgba'])]!=1:raise ValueError('duplicate instance color')
        mask=(packed==np.asarray(box['rgba'],dtype=np.uint8).view(np.uint32).item()
              if packed is not None else np.all(instance==box['rgba'],axis=2))
        ys,xs=np.nonzero(mask)
        # MORAI also reports fully occluded native boxes. Preserve them as raw evidence,
        # but there is no visible positive rectangle to export.
        if not len(xs):continue
        label,subtype=native_annotation_class(box['actor_id'],ledger,box['gt_source_label'],box['rgba'])
        linked|=mask
        infant=label=='person' and subtype in ('BabyStroller1','BabyStroller2','BabyStroller3') and any(
            r.get('api_name')==subtype and r.get('native_ids') and str(r['native_ids'][0]).isdecimal() and
            str(box['actor_id'])==str(int(r['native_ids'][0])*10000+2) for r in ledger)
        if exclude_stroller_infants and infant:
            if excluded_annotations is not None:
                excluded_annotations.append(dict(box,canonical=label,subtype=subtype,
                    xyxy=[int(xs.min()),int(ys.min()),int(xs.max()+1),int(ys.max()+1)],
                    visible_pixels=len(xs),reason='user_policy_exclude_stroller_infant_only'))
            continue
        annotations.append(dict(box,canonical=label,subtype=subtype,
            xyxy=[int(xs.min()),int(ys.min()),int(xs.max()+1),int(ys.max()+1)],visible_pixels=len(xs)))
        linked|=mask
    actor_pixels=np.zeros_like(linked)
    for color in SEMANTIC:actor_pixels|=np.all(semantic==color,axis=2)
    if np.any(actor_pixels & ~linked):raise ValueError('unlinked target semantic pixels; incomplete GT')
    return annotations


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',type=Path,required=True)
    p.add_argument('--ledger',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--gallery',type=Path,required=True)
    p.add_argument('--layout-id',required=True)
    p.add_argument('--include-stroller-infants-legacy',action='store_true',help='Reproduce historical exports; current default excludes only verified stroller infant component 2')
    a=p.parse_args()
    if not a.layout_id or a.output.exists():raise ValueError('new output version and nonempty layout ID required')
    collection_path=a.run/'lineage/collection.json'
    if not collection_path.exists():collection_path=a.run/'collection.json'
    if collection_path.exists() and json.loads(collection_path.read_text()).get('layout_id')!=a.layout_id:
        raise ValueError('layout ID differs from source collection; preserve the actual split group')
    ledger=[json.loads(l) for l in a.ledger.read_text().splitlines() if l.strip()]
    frames=[json.loads(l) for l in (a.run/'manifest.jsonl').read_text().splitlines() if l.strip()]
    a.output.mkdir(parents=True);a.gallery.mkdir(parents=True,exist_ok=True)
    images=a.output/'images/train';labels=a.output/'labels/train'
    images.mkdir(parents=True);labels.mkdir(parents=True)
    rows=[];quarantine=[];seen=set();counts=Counter();cards=[];excluded_count=0
    for frame in frames:
        for view in ('front','left','right'):
            key='{}_{}_{:06d}'.format(frame['run_id'],view,frame['frame_id'])
            try:
                paths=frame['paths']
                missing=[m for m in ('intensity','semantic','instance','bbox') if view not in paths.get(m,{})]
                if missing:raise ValueError('missing_native_files: '+view+': '+','.join(missing))
                rgb=a.run/paths['intensity'][view]
                selected=[paths[m][view] for m in ('intensity','semantic','instance','bbox')]
                for relative in selected:
                    if sha(a.run/relative)!=frame['sha256'][relative]:raise ValueError('source SHA mismatch')
                image=Image.open(rgb).convert('RGB');image.load()
                validate_rendered_rgb(image)
                pixel_sha=hashlib.sha256(image.tobytes()).hexdigest()
                if pixel_sha in seen:
                    quarantine.append(dict(key=key,reason='exact_pixel_duplicate_within_layout'));continue
                instance=np.array(Image.open(a.run/paths['instance'][view]).convert('RGBA'))
                semantic=np.array(Image.open(a.run/paths['semantic'][view]).convert('RGB'))
                if instance.shape[:2]!=(image.height,image.width):raise ValueError('RGB/GT dimensions mismatch')
                boxes=parse_bbox((a.run/paths['bbox'][view]).read_text(encoding='utf-8-sig'))
                excluded=[]
                anns=convert(instance,semantic,boxes,ledger,not a.include_stroller_infants_legacy,excluded)
                if not anns:raise ValueError('negative requires separate complete-empty review')
                lines=[]
                for ann in anns:
                    x1,y1,x2,y2=ann['xyxy'];w,h=image.size
                    lines.append('{} {:.9f} {:.9f} {:.9f} {:.9f}'.format(CLASSES.index(ann['canonical']),
                        (x1+x2)/2/w,(y1+y2)/2/h,(x2-x1)/w,(y2-y1)/h))
                (images/(key+'.png')).symlink_to(rgb.resolve())
                label_path=labels/(key+'.txt');label_path.write_text('\n'.join(lines)+'\n')
                # Re-read the real training label and verify class/geometry/count.
                parsed=[list(map(float,l.split())) for l in label_path.read_text().splitlines()]
                if len(parsed)!=len(anns) or any(int(v[0])!=CLASSES.index(ann['canonical']) or
                    not all(0<=q<=1 for q in v[1:]) or v[3]<=0 or v[4]<=0 for v,ann in zip(parsed,anns)):
                    raise ValueError('YOLO re-read failed')
                seen.add(pixel_sha)
                counts.update(ann['canonical'] for ann in anns)
                rows.append(dict(key=key,split='train',split_group=a.layout_id,layout_id=a.layout_id,
                    source_run=frame['run_id'],frame_id=frame['frame_id'],view=view,image_sha256=sha(rgb),
                    pixel_sha256=pixel_sha,source_manifest_sha256=sha(a.run/'manifest.jsonl'),
                    ledger_sha256=sha(a.ledger),annotation_sha256=sha(label_path),annotations=anns,excluded_annotations=excluded,
                    source_native_boxes=boxes, invisible_native_actor_ids=[b['actor_id'] for b in boxes
                        if b['actor_id'] not in {ann['actor_id'] for ann in anns}]))
                excluded_count+=len(excluded)
                draw=ImageDraw.Draw(image)
                for ann in excluded:
                    draw.rectangle(ann['xyxy'],outline='#ff5ca8',width=2)
                    draw.text(ann['xyxy'][:2],'excluded stroller infant',fill='#ff5ca8')
                for ann in anns:
                    draw.rectangle(ann['xyxy'],outline='#22ee88',width=2)
                    draw.text(ann['xyxy'][:2],ann['canonical']+' id='+ann['actor_id'],fill='#22ee88')
                image.save(a.gallery/(key+'.jpg'))
                cards.append('<article><h3>'+html.escape(key)+'</h3><img loading="lazy" decoding="async" src="'+key+'.jpg"></article>')
            except (KeyError,ValueError,OSError) as exc:
                # A rejected image cannot remain discoverable as an orphan train pair.
                for rejected in (images/(key+'.png'),labels/(key+'.txt')):
                    if rejected.exists() or rejected.is_symlink():rejected.unlink()
                quarantine.append(dict(key=key,reason=str(exc)))
    (a.output/'manifest.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
    (a.output/'quarantine.json').write_text(json.dumps(quarantine,indent=2))
    (a.output/'data.yaml').write_text('path: '+str(a.output.resolve())+'\ntrain: images/train\nnames:\n'+
        ''.join('  {}: {}\n'.format(i,c) for i,c in enumerate(CLASSES)))
    summary=dict(images=len(rows),objects=sum(counts.values()),classes=dict(counts),quarantined=len(quarantine),
        fixed_class_ids=list(CLASSES),split='train',split_group=a.layout_id,
        original_eval_splits_unchanged=True,training_ready=False,
        reason='new Capture subset only; merge with frozen v7 and resample before training')
    summary['annotation_policy']='legacy_include_stroller_infants' if a.include_stroller_infants_legacy else 'exclude_verified_stroller_infant_component2_v2'
    summary['excluded_stroller_infant_annotations']=excluded_count
    summary['annotation_policy_sha256']=sha(ROOT/'config/datasets/capture_annotation_policy_v2.json')
    runtime_evidence={}
    for name in ('validation.json','restoration.json','collection.json'):
        path=a.run/'lineage'/name
        if not path.exists():path=a.run/name
        if path.exists():
            runtime_evidence[name]=dict(sha256=sha(path),value=json.loads(path.read_text()))
    summary['source_runtime_evidence']=runtime_evidence
    restoration=runtime_evidence.get('restoration.json',{}).get('value',{})
    summary['source_run_restore_passed']=all(restoration.get(k) is True
        for k in ('stopped','gear_matches','mode_matches')) if restoration else None
    (a.output/'verification.json').write_text(json.dumps(summary,indent=2))
    inventory=dict(summary,dataset_role='capture_subset',classes=list(CLASSES),class_objects=dict(counts),
        splits={'train':len(rows)},manifest_sha256=sha(a.output/'manifest.jsonl'),
        labels_sha256=sha(ROOT/'config/datasets/camera_object_labels_v2.json'))
    (a.output/'summary.json').write_text(json.dumps(inventory,indent=2))
    date=datetime.now(timezone.utc).isoformat(timespec='seconds')
    (a.gallery/'index.html').write_text('<meta charset="utf-8"><title>E Capture '+date+' GT export</title>'+ 
        '<style>body{font:16px system-ui;background:#101827;color:white}img{max-width:100%}</style>'+
        '<h1>E Capture '+date+' — native GT and reviewed spawn identities</h1><pre>'+ 
        html.escape(json.dumps(summary,indent=2))+'</pre>'+''.join(cards))
    print(json.dumps(summary,indent=2))


if __name__=='__main__':main()
