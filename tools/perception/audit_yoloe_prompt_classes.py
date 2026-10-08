#!/usr/bin/env python3
"""Review every frozen prompt using real saved boxes and source RGB, not aliases as GT."""
import argparse
from collections import Counter
import hashlib
import html
import json
from pathlib import Path

import cv2
import yaml
from seg_measurements import match_boxes


def samples_for_review(rows, limit=4):
    if not rows:
        return []
    candidates = [max(rows, key=lambda r:r['prediction']['confidence']),
                  max(rows, key=lambda r:(r['prediction']['xyxy'][2]-r['prediction']['xyxy'][0]) *
                                         (r['prediction']['xyxy'][3]-r['prediction']['xyxy'][1])),
                  rows[len(rows)//2], rows[-1]]
    unique = {}
    for r in candidates:
        unique.setdefault((r['image'],tuple(r['prediction']['xyxy'])),r)
    return list(unique.values())[:limit]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--predictions',type=Path,required=True)
    p.add_argument('--gt',type=Path,required=True)
    p.add_argument('--policy',type=Path,required=True)
    p.add_argument('--datasets',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--confidence',type=float,default=.3)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    policy=yaml.safe_load(a.policy.read_text());names=policy['prompt_profiles']['rich42']
    gt={(r['run'],r['view'],r['frame']):r for r in
        (json.loads(line) for line in a.gt.read_text().splitlines()) if r['split']=='selection' and r['annotation_complete']}
    buckets={n:[] for n in names};counts={n:Counter() for n in names}
    for r in (json.loads(line) for line in a.predictions.read_text().splitlines()):
        truth=gt[(r['run'],r['view'],r['frame'])]
        boxes=[d for d in r['predictions'] if d['confidence']>=a.confidence]
        paired={c:idx for idx,c,_ in match_boxes(truth['annotations'],boxes)}
        for c,d in enumerate(boxes):
            label=d['source_class_name'];ann=truth['annotations'][paired[c]] if c in paired else None
            counts[label]['observations']+=1
            counts[label]['group_GT_matched' if ann else 'group_GT_unmatched']+=1
            if ann:
                counts[label]['verified_subtype' if ann.get('subtype_verified') else 'unverified_subtype']+=1
            buckets[label].append(dict(image=r['image'],run=r['run'],view=r['view'],frame=r['frame'],
                                      prediction=d,matched_GT=ann))
    report={};articles=[]
    for label in names:
        seen=buckets[label];selected=samples_for_review(seen)
        figures=[]
        for i,row in enumerate(selected):
            path=a.datasets/row['image'];image=cv2.imread(str(path))
            if image is None:raise ValueError('RGB missing: '+str(path))
            render=image.copy();d=row['prediction'];ann=row['matched_GT']
            if ann:
                x1,y1,x2,y2=map(int,ann['xyxy']);cv2.rectangle(render,(x1,y1),(x2,y2),(255,120,0),2)
            x1,y1,x2,y2=map(int,d['xyxy']);cv2.rectangle(render,(x1,y1),(x2,y2),(0,255,0),2)
            cv2.putText(render,label+' %.3f'%d['confidence'],(max(0,x1),max(20,y1)),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,255,0),2)
            filename=label.replace(' ','_')+'-'+str(i)+'.jpg';cv2.imwrite(str(a.output/filename),render)
            figures.append('<figure><figcaption>'+html.escape(row['view']+' '+row['frame'])+'</figcaption><img loading="lazy" width="640" src="'+filename+'"></figure>')
        report[label]=dict(counts=dict(counts[label]),samples=selected,
                           no_observations_is_not_proof_of_absent_assets=not bool(seen))
        articles.append('<article id="'+label.replace(' ','_')+'"><h2>'+html.escape(label)+' · '+str(len(seen))+'</h2>'+''.join(figures)+'</article>')
    result=dict(confidence=a.confidence,all_prompts_reviewed=names,classes=report,
                predictions_sha256=hashlib.sha256(a.predictions.read_bytes()).hexdigest(),
                matching_scope='group IoU>=0.5 one-to-one; unverified subtype is not name accuracy',
                annotation_scope='Actor GT does not exhaustively label fixed background assets; unmatched is not a verified false positive',
                user_reported_live_cases='WoodBox and container buildings labelled concrete barrier; no paired live RGB supplied')
    (a.output/'audit.json').write_text(json.dumps(result,indent=2)+'\n')
    (a.output/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>YOLOE prompt audit</title><style>article{display:flex;flex-wrap:wrap}h2{width:100%}figure{margin:5px}</style><h1>42 prompt audit · actual TensorRT outputs</h1><p>Green: selected detection. Blue: matched GT if available. Coarse GT does not verify subtype. Zero observations does not prove an asset absent. Actor GT does not exhaustively annotate background assets: unmatched is not a verified false positive.</p>'+''.join(articles))
    print(json.dumps({n:dict(counts[n]) for n in names}))


if __name__=='__main__':main()
