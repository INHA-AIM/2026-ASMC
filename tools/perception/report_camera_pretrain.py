#!/usr/bin/env python3
"""Reproducible v8 coverage, distinct from loss validity or independent scene claims."""
import argparse
from collections import Counter,defaultdict
import json
from pathlib import Path
from camera_pretrain_policy import CLASSES,sha,validate_dataset_contract
from prepare_camera_pretrain import read_rows


def report(root):
    meta=validate_dataset_contract(root,ignore_aware=True)
    rows=list(read_rows(root/'manifest.jsonl'))
    summary=json.loads((root/'summary.json').read_text())
    reasons=Counter();views=Counter();sizes=Counter();original=Counter()
    valid=Counter();groups=defaultdict(set);correlation=defaultdict(set);sources=defaultdict(set)
    for r in rows:
        views[(r['view'],r['supervision'])]+=1
        labels=set()
        for a in r['annotations']:
            if a['supervision']=='ignore':reasons.update(a['supervision_reasons'])
            if a['supervision']!='positive':continue
            labels.add(a['training_class'])
            b=a['xyxy'];side=min(b[2]-b[0],b[3]-b[1])
            bucket='2-7px' if side<8 else '8-15px' if side<16 else '16-31px' if side<32 else '32px+'
            sizes[(r['split'],bucket)]+=1
        if r['split']=='train':
            valid.update(labels)
            for c in labels:
                groups[c].add(r['split_group']);correlation[c].add(r['correlation_group']);sources[c].add(r['source'])
    for parent in meta['parents']:
        if parent==meta['source_stage']:continue
        # Older metadata can contain a relative path to the same stage.
        if Path(parent).resolve()==Path(meta['source_stage']).resolve():continue
        for r in read_rows(Path(parent)/'manifest.jsonl'):
            if r['split']=='train':
                original.update({a.get('training_class') or a.get('canonical') for a in r['annotations']} & set(CLASSES))
    coverage={c:dict(original_image_count=original[c],valid_positive_images=valid[c],
        split_groups=len(groups[c]),correlation_groups=len(correlation[c]),sources=len(sources[c]),
        exposure_images=summary['balance']['class_exposures'].get(c,0),independent_scene_count_claimed=False) for c in CLASSES}
    return dict(dataset=root.name,manifest_sha256=sha(root/'manifest.jsonl'),class_coverage=coverage,
        ignore_reasons=dict(reasons),view_supervision_images={v+'/'+s:n for (v,s),n in views.items()},
        positive_native_short_side={s+'/'+b:n for (s,b),n in sizes.items()},
        weather_images=dict(Counter(r.get('weather','unknown') for r in rows)),
        tunnel_images=dict(Counter(str(r.get('tunnel','unknown')) for r in rows)),
        unknown_conditions_not_inferred=True,
        occlusion_fraction='not reliably available for all sources; pixel occupancy is not an occlusion exclusion rule',
        training_performed=False)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--dataset',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    with a.output.open('x') as f:json.dump(report(a.dataset),f,indent=2)
