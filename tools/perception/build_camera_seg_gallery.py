#!/usr/bin/env python3
"""Compare saved results; no additional inference or heldout inspection."""
import argparse
from collections import defaultdict
import html
import json
from pathlib import Path
import cv2
import numpy as np


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--gt',type=Path,required=True);p.add_argument('--accuracy',type=Path,required=True)
    p.add_argument('--datasets',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    models=('baseline','yolo26','yoloe')
    rows={m:[json.loads(x) for x in (a.accuracy/m/'predictions.jsonl').read_text().splitlines()] for m in models}
    keys=lambda r:(r['run'],r['view'],r['frame'])
    gt={keys(r):r for r in (json.loads(x) for x in (a.gt/'frames.jsonl').read_text().splitlines()) if r['split']=='selection'}
    if any([keys(r) for r in rows[m]]!=[keys(r) for r in rows['baseline']] for m in models):raise ValueError('candidate frame lists differ')
    selected=set(range(0,len(rows['baseline']),max(1,len(rows['baseline'])//20)))
    label_frames=defaultdict(list)
    for i,row in enumerate(rows['yoloe']):
        for d in row['predictions']:
            label_frames[d['source_class_name']].append((i,d['confidence']))
    selections={}
    for label,values in sorted(label_frames.items()):
        ids=sorted({i for i,_ in values})
        samples={ids[0],max(values,key=lambda x:x[1])[0]}
        samples.update(ids[int(k*(len(ids)-1)/2)] for k in (0,1,2))
        selections[label]=sorted(samples);selected.update(samples)
    for group in ('person','vehicle','obstacle'):
        largest=sorted(((sum(ann['visible_pixels'] for ann in gt[keys(row)]['annotations'] if ann['group']==group),i)
                        for i,row in enumerate(rows['baseline'])),reverse=True)[:3]
        selected.update(i for size,i in largest if size)
    thresholds={m:json.loads((a.accuracy/m/'summary.json').read_text())['selected_threshold']['confidence'] for m in models}
    figures=[];manifest=[]
    for index in sorted(selected):
        frame=rows['baseline'][index];truth=gt[keys(frame)]
        image=cv2.imread(str(a.datasets/frame['image']))
        details=[]
        for fixed in (False,True):
            panels=[]
            for model in models:
                result=rows[model][index];threshold=.7 if fixed else thresholds[model]
                mask_values=[]
                if result['masks']:
                    with np.load(a.accuracy/model/result['masks']) as data:
                        mask_values=np.unpackbits(data['masks'],axis=2)[:,:,:image.shape[1]].astype(bool)
                rendered=image.copy()
                for ann in truth['annotations']:
                    x1,y1,x2,y2=map(int,ann['xyxy']);cv2.rectangle(rendered,(x1,y1),(x2,y2),(255,100,20),2)
                for d in result['predictions']:
                    if d['confidence']<threshold:continue
                    if len(mask_values):
                        mask=mask_values[d['mask_index']];rendered[mask]=(rendered[mask]*.6+np.array([0,240,30])*.4).astype(np.uint8)
                    x1,y1,x2,y2=map(int,d['xyxy']);cv2.rectangle(rendered,(x1,y1),(x2,y2),(0,255,30),2)
                    cv2.putText(rendered,d['source_class_name']+' %.2f'%d['confidence'],(max(0,x1),max(14,y1)),cv2.FONT_HERSHEY_SIMPLEX,.45,(0,255,30),1)
                filename='%04d-%s-%s.jpg'%(index,model,'07' if fixed else 'chosen')
                rendered=cv2.resize(rendered,(800,int(rendered.shape[0]*800/rendered.shape[1])))
                cv2.imwrite(str(a.output/filename),rendered,[cv2.IMWRITE_JPEG_QUALITY,78])
                panels.append('<figure><figcaption>'+model+' conf='+str(threshold)+'</figcaption><a href="'+filename+'"><img loading="lazy" src="'+filename+'"></a></figure>')
            details.append('<details '+('open' if not fixed else '')+'><summary>'+('Fixed 0.7' if fixed else 'Selection threshold')+'</summary><div class="row">'+''.join(panels)+'</div></details>')
        title=frame['run']+' '+frame['view']+' '+frame['frame']
        figures.append('<section><h3>'+html.escape(title)+'</h3>'+''.join(details)+'</section>')
        manifest.append(dict(index=index,run=frame['run'],view=frame['view'],frame=frame['frame'],image=frame['image'],
                             gt=truth['annotations'],review='pending_user_review'))
    (a.output/'manifest.json').write_text(json.dumps(dict(frames=manifest,selection_by_label=selections,
        heldout_inspected=False,thresholds=thresholds),indent=2)+'\n')
    (a.output/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>Seg comparison</title><style>body{font:15px sans-serif;background:#eee} .row{display:grid;grid-template-columns:repeat(3,1fr)} figure{margin:4px} img{width:100%} section{background:white;padding:12px;margin:18px} summary{cursor:pointer}</style><h1>Capture selection · blue GT / green predictions and masks</h1><p>Raw labels preserved. No animal GT. Click image for enlargement. Missing tiny/occluded objects remain in GT. Infrastructure observations require review.</p>'+''.join(figures))
    print(json.dumps(dict(frames=len(manifest),labels=len(selections))))


if __name__=='__main__':main()
