#!/usr/bin/env python3
"""Review every newly missed verified GT instance in identical mask ON/OFF inputs."""
import argparse
from collections import Counter
import html
import json
from pathlib import Path

import cv2
import numpy as np
from evaluate_coco7_models import evaluate


def hits(row, predictions, confidence):
    _,_,pairs,known,_=evaluate(row['annotations'],predictions,confidence)
    return {known[g]['actor_id'] for g,_,_ in pairs}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--gt',type=Path,required=True)
    p.add_argument('--on',type=Path,required=True)
    p.add_argument('--off',type=Path,required=True)
    p.add_argument('--datasets',type=Path,required=True)
    p.add_argument('--confidence',type=float,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    gt={r['image']:r for r in map(json.loads,a.gt.read_text().splitlines())}
    on={r['image']:r for r in map(json.loads,a.on.read_text().splitlines())}
    off={r['image']:r for r in map(json.loads,a.off.read_text().splitlines())}
    if on.keys()!=off.keys(): raise RuntimeError('different evaluated image sets; refusing partial control')
    rows=[];new=Counter();recovered=Counter();page=['<!doctype html><meta charset="utf-8"><h1>Mask OFF / ON · all newly missed GT</h1>']
    for image,po in on.items():
        row=gt[image]
        if po['source_sha256']!=off[image]['source_sha256'] or po['source_sha256']!=row['source_sha256']:
            raise RuntimeError('source SHA mismatch '+image)
        a_on=hits(row,po['predictions'],a.confidence);a_off=hits(row,off[image]['predictions'],a.confidence)
        lost=a_off-a_on;gain=a_on-a_off
        new[row['view']]+=len(lost);recovered[row['view']]+=len(gain)
        if not lost and not gain: continue
        event=dict(image=image,view=row['view'],new_missed_actor_ids=sorted(lost),recovered_actor_ids=sorted(gain),
                   annotations=row['annotations'],on=po['predictions'],off=off[image]['predictions'])
        rows.append(event)
        if not lost: continue
        source=cv2.imread(str(a.datasets/image))
        if source is None: raise RuntimeError('missing RGB '+image)
        panels=[]
        for name,result in [('OFF',off[image]),('ON',po)]:
            render=source.copy()
            for ann in row['annotations']:
                if ann['actor_id'] not in lost: continue
                xy=list(map(int,ann['xyxy']));cv2.rectangle(render,tuple(xy[:2]),tuple(xy[2:]),(0,0,255),2)
            for box in result['predictions']:
                if box['confidence']<a.confidence: continue
                xy=list(map(int,box['xyxy']));cv2.rectangle(render,tuple(xy[:2]),tuple(xy[2:]),(0,255,0),1)
                cv2.putText(render,box['source_class_name']+' %.2f'%box['confidence'],(max(0,xy[0]),max(16,xy[1])),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,255,0),1)
            cv2.putText(render,'mask '+name,(10,25),cv2.FONT_HERSHEY_SIMPLEX,.7,(255,255,255),2)
            panels.append(render)
        filename=f'{len(rows):05}.jpg';cv2.imwrite(str(a.output/filename),np.concatenate(panels,axis=1))
        page.append('<figure><figcaption>'+html.escape(image)+' lost '+html.escape(str(sorted(lost)))+'</figcaption><img loading="lazy" width="1400" src="'+filename+'"></figure>')
    report=dict(images=len(on),confidence=a.confidence,new_missed=dict(new),recovered=dict(recovered),differences=rows,
                interpretation='verified GT matches only; approved front exception is not automatically applied to another actor/frame or a side view')
    (a.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    (a.output/'index.html').write_text(''.join(page))
    print(json.dumps({k:v for k,v in report.items() if k!='differences'}))


if __name__=='__main__': main()
