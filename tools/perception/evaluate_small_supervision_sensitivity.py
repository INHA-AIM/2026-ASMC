#!/usr/bin/env python3
"""Counterfactual size filtering of saved predictions, not a training ablation."""
import argparse
from collections import Counter, defaultdict
import json
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw
from analyze_camera_small_supervision import rows, rejected, POLICIES, size_bin
from camera_pretrain_policy import transform_boxes, sha
from evaluate_camera_bbox_pilot import frame_counts
from capture_size_scope import ignored_prediction_indices

def main(a):
    gt={r['id']:r for r in rows(a.validation/'manifest.jsonl')};out={};cards=[]
    for folder in [a.job/'baseline-1280']+sorted(a.job.glob('epoch-*-1280')):
        file=folder/'predictions.jsonl'
        if not file.exists():continue
        totals=Counter();bins=defaultdict(Counter);policies=defaultdict(Counter);largest=[]
        for item in rows(file):
            r=gt[item['id']];assert r['image_sha256']==item['image_sha256']
            preds=[p for p in item['predictions'] if p['confidence']>=.4]
            counts,pairs=frame_counts(r,preds,.4);matched={p for _,p,_ in pairs}
            ignored=ignored_prediction_indices(preds,[x for x in r['annotations'] if x.get('supervision')=='ignore'],matched,dict(ignore_prediction_ioa=.5))
            for k in counts.values():totals['gt']+=k['gt']
            fps=0
            for i,p in enumerate(preds):
                if i in ignored:continue
                kind='tp' if i in matched else 'fp';totals[kind]+=1;fps+=kind=='fp'
                b=transform_boxes([p['xyxy']],r['width'],r['height'])[0]
                bins[p['class_name']+'|'+size_bin(min(b[2]-b[0],b[3]-b[1]))][kind]+=1
                for policy in POLICIES:
                    if rejected(p['xyxy'],r['width'],r['height'],policy):policies[policy][kind+'_filtered']+=1
            largest.append((fps,r,preds))
        out[folder.name]=dict(totals=totals,size_bins=bins,threshold_filter_sensitivity=policies,prediction_sha256=sha(file))
        if folder.name.startswith('epoch'):
            for rank,(fps,r,preds) in enumerate(sorted(largest,key=lambda x:x[0],reverse=True)[:4]):
                im=Image.open(r['image']).convert('RGB');draw=ImageDraw.Draw(im)
                for ann in r['annotations']:
                    if ann.get('supervision')=='positive':draw.rectangle(ann['xyxy'],outline='cyan',width=2)
                for p in preds:draw.rectangle(p['xyxy'],outline='yellow',width=1)
                path=f'{folder.name}-fp{rank}.jpg';im.save(a.output/path)
                cards.append(dict(file=path,id=r['id'],fp=fps,source=r['source']))
    result=dict(confidence=.4,validation_sha256=sha(a.validation/'manifest.jsonl'),models=out,
        interpretation='Hypothetical prediction size filter only. Original GT stays fixed; removed TP become FN. This does not predict retraining accuracy or endorse filtering at deployment.',gallery=cards)
    (a.output/'evaluation-sensitivity.json').write_text(json.dumps(result,indent=2)+'\n')
    with (a.output/'index.html').open('a') as f:
        f.write('<h2>Fixed GT, saved predictions: FP cases</h2><p>Cyan: positive GT. Yellow: predictions. Confidence 0.40.</p>'+''.join('<p>'+x['file']+' FP='+str(x['fp'])+'</p><img src="'+x['file']+'">' for x in cards))
    print(json.dumps({k:v['totals'] for k,v in out.items()}))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--job',type=Path,required=True);p.add_argument('--validation',type=Path,required=True);p.add_argument('--output',type=Path,required=True);main(p.parse_args())
