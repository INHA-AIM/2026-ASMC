#!/usr/bin/env python3
"""Frozen consecutive-input diagnostic, distinct from sparse accuracy samples."""
import argparse
from collections import defaultdict
from pathlib import Path
import json

from camera_data_audit import read_jsonl,save,sha,training_label,write_jsonl
from evaluate_camera_bbox_pilot import frame_counts


def frame(row):return int(row['source_key'].rsplit('/',1)[1])


def eligible_actors(row):
    actors=[]
    for a in row['annotations']:
        label=training_label(a)
        if not label or a.get('issues') or a.get('actor_id') is None:continue
        x,y,x2,y2=a['xyxy'];w=x2-x;h=y2-y
        long,short=(24,12) if label in ('car','truck','bus') else (16,8)
        if max(w,h)>=long and min(w,h)>=short:
            actors.append((str(a['actor_id']),a.get('original_label'),label))
    return actors


def prepare(rows,limit=300):
    by=defaultdict(list)
    for row in rows:
        if (not row['source'].startswith('capture:') or row['split']=='train' or row.get('issues') or
                any('split' in r or 'duplicate' in r or 'missing_capture' in r for r in row['reasons'])):
            continue
        if eligible_actors(row):by[(row['split'],row['source'],row['view'])].append(row)
    segments=[]
    for key,values in sorted(by.items()):
        current=[];common=set()
        for row in sorted(values,key=frame):
            actors=set(eligible_actors(row))
            if not current or frame(row)!=frame(current[-1])+1 or not actors&common:
                if len(current)>=5:segments.append((key,current))
                current=[row];common=actors
            else:current.append(row);common&=actors
        if len(current)>=5:segments.append((key,current))
    # Round-robin run/view segments; never use predictions to pick a scene.
    selected={};count=defaultdict(int)
    for key,segment in sorted(segments,key=lambda item:(-min(30,len(item[1])),item[0])):
        part=segment[:30]
        if count[key[0]]+len(part)>limit:continue
        for row in part:selected[row['id']]=row
        count[key[0]]+=len(part)
    return sorted(selected.values(),key=lambda r:(r['source'],r['view'],frame(r)))


def analyze(rows,predictions,confidence,level='class'):
    by=defaultdict(list);pred={p['id']:p for p in predictions}
    for row in rows:
        if row['id'] not in pred or pred[row['id']]['image_sha256']!=row['image_sha256']:
            raise ValueError('temporal source/prediction mismatch')
        usable=[a for a in row['annotations'] if (str(a.get('actor_id')),a.get('original_label'),training_label(a)) in eligible_actors(row)]
        r=dict(row,annotations=usable)
        counts,pairs=frame_counts(r,pred[row['id']]['predictions'],confidence,level,pixel_scope=True)
        matched={g for g,_,_ in pairs}
        for index,a in enumerate(usable):
            key=(row['split'],row['source'],row['view'],a['original_label'],str(a['actor_id']),training_label(a))
            by[key].append((frame(row),index in matched))
    result=[]
    for key,observations in sorted(by.items()):
        segments=[]
        for item in sorted(observations):
            if not segments or item[0]!=segments[-1][-1][0]+1:segments.append([])
            segments[-1].append(item)
        for segment in segments:
            if len(segment)<5:continue
            stable=next((i for i in range(4,len(segment)) if sum(d for _,d in segment[i-4:i+1])>=3),None)
            gap=0;largest=0
            for _,detected in segment:
                gap=0 if detected else gap+1;largest=max(largest,gap)
            result.append(dict(split=key[0],source=key[1],view=key[2],original_label=key[3],actor_id=key[4],
                training_class=key[5],first_frame=segment[0][0],last_frame=segment[-1][0],inputs=len(segment),
                matched_inputs=sum(d for _,d in segment),largest_miss_streak_inputs=largest,
                first_stable_window_end_frame=segment[stable][0] if stable is not None else None,
                first_stable_offset_inputs=stable if stable is not None else None))
    return dict(confidence=confidence,level=level,segments=result,
        definition='first full window of five consecutive stored Capture inputs with >=3 IoU .5 matches',
        time_basis='frame index only; not milliseconds, source rate or live sensor continuity',
        limitation='first within this verified segment, not first global visibility/approach; known actor recall only, no FP or precision')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--curated',type=Path,required=True)
    p.add_argument('--dataset',type=Path,required=True);a=p.parse_args()
    rows=prepare(read_jsonl(a.curated))
    target=a.dataset/'temporal-probe.jsonl'
    if target.exists():raise ValueError('temporal inputs already frozen')
    write_jsonl(target,rows)
    save(a.dataset/'temporal-probe-summary.json',dict(images=len(rows),manifest_sha256=sha(target),
        original_curated_sha256=sha(a.curated),source_splits_only=['selection','heldout'],training_membership_changed=False,
        evaluation_kind='additional temporal recall diagnostic; not sparse accuracy/threshold selection'))
    print(json.dumps(dict(images=len(rows),manifest_sha256=sha(target))))
