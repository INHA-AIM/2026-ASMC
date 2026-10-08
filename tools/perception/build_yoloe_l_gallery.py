#!/usr/bin/env python3
"""Same-image gallery and actor continuity from stored genuine prompt runs."""
import argparse
from collections import Counter
import html
import json
from pathlib import Path
import cv2
import numpy as np
from seg_measurements import match_boxes, iou
from asmc_camera.detection.hood_mask import HoodMask


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--results',type=Path,required=True);p.add_argument('--datasets',type=Path,required=True)
    p.add_argument('--hood-config',type=Path,required=True);p.add_argument('--confidence',type=float,default=.7)
    a=p.parse_args();root=a.results;dest=root/'comparison-gallery';dest.mkdir(exist_ok=False)
    gt=[json.loads(line) for line in (root/'gt-v2/frames.jsonl').read_text().splitlines()]
    gt={(r['run'],r['view'],r['frame']):r for r in gt if r['split']=='selection' and r['annotation_complete']}
    paths={'rich42':root/'quality-rich42-pytorch-v2','base27':root/'quality-base27-pytorch'}
    predictions={name:[json.loads(line) for line in (path/'predictions.jsonl').read_text().splitlines()] for name,path in paths.items()}
    hood=HoodMask.load(True,a.hood_config)
    summaries={};selected=set();lookups={};events={}
    for name,rows in predictions.items():
        state={};counter=Counter();actors={};lookup={};labels=Counter();details=[];budgets=Counter();label_frames={}
        for index,row in enumerate(rows):
            key=(row['run'],row['view'],row['frame']);lookup[key]=(index,row);truth=gt[key]
            boxes=[d for d in row['predictions'] if d['confidence']>=a.confidence]
            pairs=match_boxes(truth['annotations'],boxes);by_gt={r:c for r,c,_ in pairs}
            duplicate=sum(any(iou(boxes[c]['xyxy'],d['xyxy'])>=.5 and boxes[c]['group']==d['group'] for k,d in enumerate(boxes) if k!=c) for _,c,_ in pairs)
            missed=[d for n,d in enumerate(truth['annotations']) if n not in by_gt]
            switches=[]
            for n,ann in enumerate(truth['annotations']):
                actor=(row['run'],row['view'],str(ann['actor_id']));label=boxes[by_gt[n]]['source_class_name'] if n in by_gt else None
                previous=state.get(actor);frame=int(row['frame'])
                record=actors.setdefault(actor,dict(first_visible_frame=frame,first_detected_frame=None,matched_to_missed=0,label_transitions=0))
                if label and record['first_detected_frame'] is None:record['first_detected_frame']=frame
                if previous and previous[0]==frame-1:
                    if previous[1] and label is None:record['matched_to_missed']+=1;counter['matched_to_missed']+=1
                    if previous[1] and label and label!=previous[1]:record['label_transitions']+=1;counter['label_transitions']+=1;switches.append([previous[1],label])
                state[actor]=(frame,label)
            for d in boxes:
                labels[d['source_class_name']]+=1
                label_frames.setdefault(d['source_class_name'],[]).append((key,d['confidence']))
            reasons=[]
            if duplicate:reasons.append('overlapping same-group labels');counter['GT_with_duplicate_prediction']+=duplicate
            if any(d['group']=='obstacle' for d in missed):reasons.append('unmatched obstacle GT; subtype unverified')
            if any(d['group'] in ('person','vehicle') and (d['xyxy'][2]-d['xyxy'][0])*(d['xyxy'][3]-d['xyxy'][1])>truth['width']*truth['height']*.05 for d in truth['annotations']):reasons.append('close person/vehicle')
            if any(d['source_class_name'] in ('guardrail','fence','tree','road sign','streetlight') for d in boxes):reasons.append('structure/infrastructure observation')
            if switches:reasons.append('label transitions')
            if index%max(1,len(rows)//30)==0:reasons.append('uniform sample')
            admitted=[r for r in reasons if r=='uniform sample' or budgets[r]<8]
            if admitted:
                for reason in admitted:budgets[reason]+=1
                selected.add(key);details.append(dict(key=key,reasons=reasons,transitions=switches))
        for label,seen in label_frames.items():
            for key,score in (seen[0],max(seen,key=lambda s:s[1]),seen[len(seen)//2]):
                selected.add(key);details.append(dict(key=key,reasons=['label sample: '+label]))
        summaries[name]=dict(counters=dict(counter),labels=dict(labels),actors=[dict(run=k[0],view=k[1],actor_id=k[2],**v) for k,v in actors.items()],
            timing_unit='Capture frame index; no invented wall/source timestamps',GT_subtype_unverified_not_scored_as_box_type=True)
        lookups[name]=lookup;events[name]={tuple(d['key']):d for d in details}
    gallery=[]
    for no,key in enumerate(sorted(selected)):
        truth=gt[key];image=cv2.imread(str(a.datasets/truth['image']))
        figures=[]
        for name,path in paths.items():
            index,row=lookups[name][key];render=image.copy();masks=[]
            if row.get('masks'):
                with np.load(path/row['masks']) as data:masks=np.unpackbits(data['masks'],axis=2)[:,:,:truth['width']].astype(bool)
            if key[1]=='front':
                bitmap,polygon=hood.geometry(*image.shape[:2]);render[bitmap]=(render[bitmap]*.65+np.array([0,0,255])*.35).astype(np.uint8)
            for ann in truth['annotations']:
                x1,y1,x2,y2=map(int,ann['xyxy']);cv2.rectangle(render,(x1,y1),(x2,y2),(255,120,0),1)
            for k,d in enumerate(row['predictions']):
                if d['confidence']<a.confidence:continue
                if len(masks):render[masks[k]]=(render[masks[k]]*.7+np.array([30,220,30])*.3).astype(np.uint8)
                x1,y1,x2,y2=map(int,d['xyxy']);cv2.rectangle(render,(x1,y1),(x2,y2),(0,255,0),1)
                cv2.putText(render,d['source_class_name']+' %.2f'%d['confidence'],(max(0,x1),max(14,y1)),cv2.FONT_HERSHEY_SIMPLEX,.4,(0,255,0),1)
            filename=f'{no:04d}-{name}.jpg';cv2.imwrite(str(dest/filename),render)
            caption=name+' / '+', '.join(events[name].get(key,{}).get('reasons',[]))
            figures.append('<figure><figcaption>'+html.escape(caption)+'</figcaption><img loading="lazy" width="640" src="'+filename+'"></figure>')
        gallery.append('<article><h3>'+html.escape(' / '.join(key))+'</h3>'+''.join(figures)+'</article>')
    (root/'continuity.json').write_text(json.dumps(summaries,indent=2)+'\n')
    (dest/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>YOLOE-l prompt comparison</title><style>article{display:flex;flex-wrap:wrap}h3{width:100%}figure{margin:5px}</style><h1>Actual rich42 vs base27 · confidence '+str(a.confidence)+'</h1><p>Blue: GT. Green: predictions and instance mask. Red: front hood exclusion. Unverified object subtypes remain unverified.</p>'+''.join(gallery))


if __name__=='__main__':main()
