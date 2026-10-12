#!/usr/bin/env python3
"""Create an immutable Capture subset revision excluding only verified stroller infants."""
import argparse
from collections import Counter
import copy
import hashlib
import json
from pathlib import Path
import shutil
import sys
from PIL import Image,ImageDraw
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src/data_collection/src'))
from data_collection.capture_collection import CLASSES


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--dataset',type=Path,required=True)
    p.add_argument('--ledger',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--gallery',type=Path,required=True)
    a=p.parse_args()
    if a.output.exists() or a.gallery.exists():raise ValueError('new immutable dataset and gallery required')
    ledger=[json.loads(l) for l in a.ledger.read_text().splitlines() if l.strip()]
    excluded_ids={str(int(r['native_ids'][0])*10000+2):r for r in ledger
        if r.get('api_name') in ('BabyStroller1','BabyStroller2','BabyStroller3') and r.get('native_ids')}
    records=[json.loads(l) for l in (a.dataset/'manifest.jsonl').read_text().splitlines() if l.strip()]
    ledger_sha=sha(a.ledger)
    # Check immutable provenance before creating any output directories.
    for source in records:
        if source['ledger_sha256']!=ledger_sha:raise ValueError('source ledger SHA mismatch')
        label=a.dataset/'labels/train'/(source['key']+'.txt')
        if sha(label)!=source['annotation_sha256']:raise ValueError('parent annotation SHA mismatch')
    a.output.mkdir(parents=True);a.gallery.mkdir(parents=True)
    for folder in ('images/train','labels/train'):(a.output/folder).mkdir(parents=True)
    rows=[];counts=Counter();excluded_count=0;cards=[];empty=[];ledger_sha=sha(a.ledger)
    for source in records:
        if source['ledger_sha256']!=ledger_sha:raise ValueError('source ledger SHA mismatch')
        old_label=a.dataset/'labels/train'/(source['key']+'.txt')
        if sha(old_label)!=source['annotation_sha256']:raise ValueError('parent annotation SHA mismatch')
        row=copy.deepcopy(source);kept=[];excluded=[]
        for ann in row['annotations']:
            actor=str(ann['actor_id']);owner=excluded_ids.get(actor)
            if owner and ann['canonical']=='person' and ann['subtype']==owner['api_name']:
                if owner.get('native_source_classes',{}).get(actor,{}).get(ann['gt_source_label'])!='person':
                    raise ValueError('infant source identity not verified')
                excluded.append(dict(ann,reason='user_policy_exclude_stroller_infant_only'))
            else:kept.append(ann)
        if not kept:
            empty.append(dict(key=row['key'],reason='only excluded stroller infant; not approved as a negative'));continue
        image=a.dataset/'images/train'/(row['key']+'.png')
        with Image.open(image) as im:width,height=im.size
        (a.output/'images/train'/(row['key']+'.png')).symlink_to(image.resolve())
        label=a.output/'labels/train'/(row['key']+'.txt')
        lines=[]
        for ann in kept:
            x1,y1,x2,y2=ann['xyxy'];lines.append('{} {:.9f} {:.9f} {:.9f} {:.9f}'.format(CLASSES.index(ann['canonical']),
                (x1+x2)/2/width,(y1+y2)/2/height,(x2-x1)/width,(y2-y1)/height))
        label.write_text('\n'.join(lines)+'\n')
        row.update(annotations=kept,annotation_sha256=sha(label),parent_annotation_sha256=source['annotation_sha256'],
                   excluded_annotations=row.get('excluded_annotations',[])+excluded,
                   annotation_policy='exclude_verified_stroller_infant_component2_v2')
        rows.append(row);counts.update(x['canonical'] for x in kept);excluded_count+=len(excluded)
        if excluded and len(cards)<36:
            with Image.open(image) as im:
                im=im.convert('RGB');d=ImageDraw.Draw(im)
                for ann in kept:d.rectangle(ann['xyxy'],outline='#22ee88',width=2)
                for ann in excluded:
                    d.rectangle(ann['xyxy'],outline='#ff5ca8',width=2);d.text(ann['xyxy'][:2],'excluded stroller infant',fill='#ff5ca8')
                name=row['key']+'.jpg';im.save(a.gallery/name);cards.append('<p>'+row['key']+'</p><img loading="lazy" src="'+name+'">')
    (a.output/'manifest.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
    old_quarantine=json.loads((a.dataset/'quarantine.json').read_text())
    (a.output/'quarantine.json').write_text(json.dumps(old_quarantine+empty,indent=2))
    (a.output/'data.yaml').write_text('path: '+str(a.output.resolve())+'\ntrain: images/train\nnames:\n'+''.join('  {}: {}\n'.format(i,c) for i,c in enumerate(CLASSES)))
    summary=json.loads((a.dataset/'summary.json').read_text())
    summary.update(images=len(rows),objects=sum(counts.values()),class_objects=dict(counts),classes=list(CLASSES),
        splits={'train':len(rows)},quarantined=len(old_quarantine)+len(empty),manifest_sha256=sha(a.output/'manifest.jsonl'),
        annotation_policy='exclude_verified_stroller_infant_component2_v2',excluded_stroller_infant_annotations=excluded_count,
        annotation_policy_sha256=sha(ROOT/'config/datasets/capture_annotation_policy_v2.json'),
        parent_dataset=str(a.dataset.resolve()),parent_manifest_sha256=sha(a.dataset/'manifest.jsonl'),training_ready=False)
    (a.output/'summary.json').write_text(json.dumps(summary,indent=2))
    (a.output/'verification.json').write_text(json.dumps(dict(status='independent verifier required',images=len(rows),objects=sum(counts.values())),indent=2))
    (a.gallery/'index.html').write_text('<meta charset="utf-8"><title>E-CAP-20261012 infant exclusion</title><style>body{background:#101827;color:white;font:16px system-ui}img{max-width:100%}</style><h1>E-CAP-20261012 · 유모차 아기 제외</h1><p>초록: 유지한 객체 / 분홍: 유모차 안 아기만 제외. 원본 GT는 보존합니다.</p>'+''.join(cards))
    print(json.dumps(dict(images=len(rows),objects=sum(counts.values()),excluded_stroller_infant_annotations=excluded_count,output=str(a.output))))

if __name__=='__main__':main()
