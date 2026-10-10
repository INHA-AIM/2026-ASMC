#!/usr/bin/env python3
"""Replay only the pinned CPU sampler, never images, augmentations or GPU work.

The resulting stream is accepted only when its full hash matches a settled
real training run. Prefix hashes can then prove equal complete-epoch order
even if the other run reaches its wall budget in the middle of an epoch.
"""
import argparse
import hashlib
import json
from pathlib import Path
from train_camera_bbox_pilot import save,sha


class FileDataset:
    def __init__(self,files):self.files=files
    def __len__(self):return len(self.files)
    def __getitem__(self,index):return {'im_file':self.files[index]}


def run(args):
    import numpy as np
    import torch
    import ultralytics
    from ultralytics.data.build import build_dataloader
    if ultralytics.__version__!='8.4.150' or not torch.__version__.startswith('2.1.2'):
        raise ValueError('sampler replay is bound to the measured installation')
    if torch.cuda.is_initialized():raise ValueError('CPU-only verifier must not own a CUDA context')
    cfg=json.loads(args.config.read_text());cache=args.dataset/'labels/train.cache'
    # This is our locally generated Ultralytics cache, not downloaded pickle.
    files=[r['im_file'] for r in np.load(cache,allow_pickle=True).item()['labels']]
    rows=[json.loads(l) for l in (args.dataset/'manifest.jsonl').read_text().splitlines()]
    if {Path(f).stem for f in files}!={r['id'] for r in rows if r['split']=='train'}:
        raise ValueError('cached training membership differs from frozen manifest')
    records=[json.loads(path.read_text()) for path in args.runs]
    batch=records[0]['batch'];epochs=max(r['epochs'] for r in records)
    if any(r['batch']!=batch or r['manifest_sha256']!=sha(args.dataset/'manifest.jsonl') for r in records):
        raise ValueError('candidate identity differs')
    torch.manual_seed(cfg['seed'])
    loader=build_dataloader(FileDataset(files),batch=batch,workers=0,rank=-1,device='cpu')
    digest=hashlib.sha256();prefixes={};checks=[];wanted={r.get('completed_batches',0) for r in records}
    count=0
    for epoch in range(epochs):
        for item in loader:
            names='\n'.join(item['im_file'])
            digest.update((names+'\n').encode());digest.update(names.encode());count+=1
            if count in wanted:prefixes[str(count)]=digest.hexdigest()
        prefixes['epoch:'+str(epoch+1)]=digest.hexdigest()
    for path,r in zip(args.runs,records):
        actual=r.get('sample_order_sha256');n=r.get('completed_batches',0)
        checks.append(dict(run=str(path),status=r['status'],batches=n,
            actual_sha256=actual,expected_sha256=prefixes.get(str(n)),
            matched=actual==prefixes.get(str(n)) if actual else None))
    if torch.cuda.is_initialized():raise RuntimeError('unexpected GPU context in sampler-only verification')
    result=dict(status='failed' if any(c['matched'] is False for c in checks) else 'complete',
        checks=checks,prefix_sha256=prefixes,manifest_sha256=sha(args.dataset/'manifest.jsonl'),
        cache_sha256=sha(cache),seed=cfg['seed'],batch=batch,batches_per_epoch=len(loader),
        verifier_source_sha256=sha(__file__),torch=torch.__version__,ultralytics=ultralytics.__version__,
        gpu_context=False,augmentation_reproducibility='not claimed; file order only')
    save(args.output,result)
    print(json.dumps(dict(status=result['status'],checks=checks),ensure_ascii=False))
    if result['status']=='failed':raise ValueError('sampler replay does not match actual training order')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,required=True);p.add_argument('--dataset',type=Path,required=True)
    p.add_argument('--runs',type=Path,nargs='+',required=True);p.add_argument('--output',type=Path,required=True)
    run(p.parse_args())
