#!/usr/bin/env python3
"""Independent restoration invariants and structural publication; never starts training."""
import argparse,json
from pathlib import Path
from collections import Counter
import numpy as np
from camera_pretrain_policy import sha,tiny_reasons,load_policy,CLASSES
from prepare_camera_pretrain import read_rows,verify_export,write_json
from review_camera_supervision_policy import assert_isolation,indexes


def check(a):
 root=a.dataset
 if (root/'_SUCCESS').exists():raise ValueError('already published; use read-only verify_camera_pretrain.py')
 meta=json.loads((root/'pretrain.json').read_text())
 if (root/'_PREPARED').read_text().strip()!=sha(root/'pretrain.json'):raise ValueError('prepared metadata mismatch')
 for file,digest in meta['artifact_shas'].items():
  if sha(root/file)!=digest:raise ValueError('prepared artifact mismatch: '+file)
 rows=list(read_rows(root/'manifest.jsonl'));old=list(read_rows(a.source/'manifest.jsonl'));oldmap={r['id']:r for r in old};policy=load_policy(root/'policy.json');train=[r for r in rows if r['split']=='train']
 result=verify_export(root,rows,meta['parents'])
 if not result['passed']:raise ValueError(result['failures'])
 if len({r['id'] for r in rows})!=len(rows):raise ValueError('duplicate IDs')
 oldtest={r['id']:r for r in old if r['split']=='heldout'};newtest={r['id']:r for r in rows if r['split']=='heldout'}
 if oldtest.keys()!=newtest.keys():raise ValueError('heldout membership changed')
 for key,r in newtest.items():
  for field in ('annotations','image_sha256','split_group','supervision'):
   if r[field]!=oldtest[key][field]:raise ValueError('heldout changed: '+field)
 for role in ('selection','heldout'):assert_isolation(train,indexes(read_rows(root/f'reserved-{role}.jsonl')))
 background_count=0
 for r in rows:
  role={'selection':'val','heldout':'test'}.get(r['split'],r['split'])
  if sha(root/'labels'/role/(r['id']+'.txt'))!=r['export_label_sha256']:raise ValueError('label hash')
  if sha(root/'supervision'/role/(r['id']+'.json'))!=r['supervision_sha256']:raise ValueError('sidecar hash')
  positives=[x for x in r['annotations'] if x['supervision']=='positive']
  if any(tiny_reasons(x['xyxy'],r['width'],r['height'],policy,x.get('visible_pixels')) for x in positives):raise ValueError('positive violates policy')
  if r.get('background_safe_grid'):
   file=root/r['background_safe_grid']
   if sha(file)!=r['background_safe_grid_sha256']:raise ValueError('background hash')
   with np.load(file) as z:cells=np.unpackbits(z['packed'])
   if cells.size!=14400:raise ValueError('background shape')
  if r.get('spatial_background_only'):
   if positives or r['supervision']!='partial' or r['known_class_background'] or not cells.any():raise ValueError('invalid spatial-only supervision')
   background_count+=1
  if r['split']=='train' and not positives and r['supervision']!='reviewed_negative' and not r.get('spatial_background_only'):raise ValueError('unreviewed empty train')
 exposure=[r['id'] for r in rows if r['split']=='selection' and r['id'] in oldmap and oldmap[r['id']]['split']=='train']
 plan=json.loads((root/'restoration-plan.json').read_text())
 if set(exposure)!=set(plan['new_selection_seen_by_current_model']):raise ValueError('history exposure mismatch')
 car=sum(x['training_class']=='car' and x['supervision']=='positive' for r in train if r['source'].startswith('capture:') for x in r['annotations'])
 if car<1000:raise ValueError('Capture car restoration failed')
 order=list(read_rows(root/'epoch-000-order.jsonl'));count=Counter(r['id'] for r in order)
 if set(count)!={r['id'] for r in train} or max(count.values())>2:raise ValueError('sampler omitted or over-repeated images')
 result.update(heldout_labels_and_images_unchanged=True,full_reserved_isolation=True,positive_policy_consistent=True,spatial_background_only_images=background_count,capture_train_car_boxes=car,selection_seen_by_current_model=len(exposure),all_train_images_used=True,training_started=False)
 a.report.parent.mkdir(exist_ok=True,parents=True);write_json(a.report,result)
 meta['restoration_verification']=dict(path=str(a.report.resolve()),sha256=sha(a.report))
 meta['restoration_input_pins']={str(p.resolve()):sha(p) for p in (a.assignment,a.patch/'manifest.jsonl',a.patch/'quarantine-removals.json',a.reviews,a.negative_audit)}
 write_json(root/'pretrain.json',meta);(root/'_PREPARED').write_text(sha(root/'pretrain.json')+'\n');(root/'_SUCCESS').write_text(sha(root/'pretrain.json')+'\n')
 print(json.dumps(result))
if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__)
 for name in ('dataset','source','assignment','patch','reviews','negative-audit','report'):p.add_argument('--'+name,type=Path,required=True)
 check(p.parse_args())
