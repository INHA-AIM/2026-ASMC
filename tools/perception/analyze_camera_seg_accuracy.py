#!/usr/bin/env python3
"""Size/edge proxies and consecutive visible-actor gaps from saved predictions."""
import argparse
from collections import Counter, defaultdict
import json
from pathlib import Path
from seg_measurements import match_boxes, INFRASTRUCTURE


def strata(annotation,width,height):
    x1,y1,x2,y2=annotation['xyxy'];area=max(0,x2-x1)*max(0,y2-y1)
    names=['all']
    if area/(width*height)<.001:names.append('small_visible_box')
    if area/(width*height)>=.03:names.append('large_visible_box')
    if x1<=1 or y1<=1 or x2>=width-1 or y2>=height-1:names.append('image_edge')
    sx1,sy1,sx2,sy2=annotation['source_xyxy'];source_area=max(0,sx2-sx1)*max(0,sy2-sy1)
    if source_area and annotation['visible_pixels']/source_area<.25:names.append('low_instance_fill')
    return names


def analyze(gt,predictions,threshold):
    bins=defaultdict(Counter);history={};gaps=Counter();visible_intervals=Counter();reappearances=Counter()
    for frame in predictions:
        key=(frame['run'],frame['view'],frame['frame']);truth=gt[key]
        predicted=[d for d in frame['predictions'] if d['confidence']>=threshold and d['source_class_name'] not in INFRASTRUCTURE]
        matched={r for r,_,_ in match_boxes(truth['annotations'],predicted)}
        ordinal=int(frame['frame'])
        for i,ann in enumerate(truth['annotations']):
            yes=i in matched
            for label in strata(ann,truth['width'],truth['height']):
                bins[label,ann['group']]['gt']+=1;bins[label,ann['group']]['matched']+=yes
            actor=(frame['run'],frame['view'],ann['actor_id'])
            previous=history.get(actor)
            if previous and previous[0]==ordinal-1:
                visible_intervals[ann['group']]+=1
                gaps[ann['group']]+=previous[1] and not yes
            elif previous:reappearances[ann['group']]+=1
            history[actor]=(ordinal,yes)
    return dict(confidence=threshold,recall_by_proxy=[dict(proxy=label,group=group,**dict(count),recall=count['matched']/count['gt'])
        for (label,group),count in sorted(bins.items())],visible_consecutive_intervals=dict(visible_intervals),
        matched_to_missed_transitions=dict(gaps),actor_reappearances=dict(reappearances),
        scope='Capture consecutive frame IDs only; no missing GT frame interpolation; not 20Hz bag continuity',
        proximity_and_occlusion='image-size/edge/fill proxies only; not metric distance or verified occlusion labels')


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--gt',type=Path,required=True)
    p.add_argument('--accuracy',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    gt={(r['run'],r['view'],r['frame']):r for r in (json.loads(x) for x in (a.gt/'frames.jsonl').read_text().splitlines()) if r['split']=='selection' and r['annotation_complete']}
    result={}
    for model in ('baseline','yolo26','yoloe'):
        rows=[json.loads(x) for x in (a.accuracy/model/'predictions.jsonl').read_text().splitlines()]
        summary=json.loads((a.accuracy/model/'summary.json').read_text())
        result[model]=dict(fixed_07=analyze(gt,rows,.7),selected=analyze(gt,rows,summary['selected_threshold']['confidence']))
    (a.output/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({m:d['selected']['matched_to_missed_transitions'] for m,d in result.items()}))


if __name__=='__main__':main()
