#!/usr/bin/env python3
"""Source/class coverage audit; image and lineage counts are not car-model counts."""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path


def read_rows(path):
    with Path(path).open() as f:
        for line in f:
            if line.strip(): yield json.loads(line)


def digest(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
    return h.hexdigest()


def audit(rows):
    stats={}
    for row in rows:
        for ann in row['annotations']:
            if ann.get('supervision')!='positive':continue
            cls=ann['training_class'];key=(row['split'],cls,row['source'])
            if key not in stats:stats[key]=dict(boxes=0,images=set(),groups=set(),runs=set(),views=Counter(),weather=Counter(),subtypes=Counter(),complete=Counter())
            v=stats[key];v['boxes']+=1;v['images'].add(row['id']);v['groups'].add(row['split_group']);v['runs'].add(row['source'])
            v['views'][row.get('view','unknown')]+=1;v['weather'][row.get('weather','unknown')]+=1;v['complete'][row.get('supervision','unknown')]+=1
            v['subtypes'][ann.get('morai_asset_name') or ann.get('subtype') or 'unknown']+=1
    result=[]
    for (split,cls,source),v in sorted(stats.items()):
        result.append(dict(split=split,training_class=cls,source=source,**{k:len(x) if isinstance(x,set) else x for k,x in v.items()}))
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--dataset',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists():raise ValueError('immutable report output exists')
    active=list(read_rows(a.dataset/'manifest.jsonl'));summary=audit(active);perclass={}
    for cls in sorted({v['training_class'] for v in summary}):
        train=[v for v in summary if v['split']=='train' and v['training_class']==cls];total=sum(v['boxes'] for v in train)
        source_kind=Counter();models=Counter()
        for v in train:source_kind[v['source'].split(':',1)[0]]+=v['boxes'];models.update(v['subtypes'])
        perclass[cls]=dict(train_boxes=total,source_kind_boxes=source_kind,known_subtypes=models,
            largest_source_share=max((v['boxes'] for v in train),default=0)/max(1,total),
            distinct_source_names=len(train),model_coverage_complete=False)
    reserved=[]
    for role in ('selection','heldout'):
        path=a.dataset/f'reserved-{role}.jsonl'
        if path.exists():reserved+=audit(read_rows(path))
    a.output.mkdir(parents=True)
    (a.output/'summary.json').write_text(json.dumps(dict(dataset=str(a.dataset.resolve()),manifest_sha256=digest(a.dataset/'manifest.jsonl'),classes=perclass,source_class_split=summary,reserved=reserved,notes=['Unknown subtypes are not inferred from class names.','Related runs and augmentations are not independent vehicle models.','Existing evaluation reservations are retained.']),ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(perclass,ensure_ascii=False))

if __name__=='__main__':main()
