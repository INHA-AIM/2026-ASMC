#!/usr/bin/env python3
"""Materialize SHA-pinned experiment configs; never substitute another checkpoint's prompts."""
import argparse
import hashlib
from pathlib import Path
import numpy as np
import yaml
from asmc_camera.detection.segmentation import load_prompt_embeddings


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def configuration(directory, scale, width, names, runtime_root):
    checkpoint=directory/scale/f'yoloe-26{scale}-seg.pt'
    prompts=directory/scale/'obstacle17.npz';engine=directory/scale/f'obstacle17-{width}.engine'
    actual_names,_=load_prompt_embeddings(prompts,sha(checkpoint))
    if actual_names!=names:raise ValueError('prompt names/order differs from frozen experiment')
    # Read metadata before publishing a launch config; an arbitrary .engine is insufficient.
    import json,struct
    with engine.open('rb') as stream:
        length=struct.unpack('<i',stream.read(4))[0]
        if not 0<length<1024*1024:raise ValueError('invalid engine header')
        metadata=json.loads(stream.read(length))
    exported=[v for _,v in sorted((int(k),v) for k,v in metadata['names'].items())]
    if exported!=names or metadata.get('end2end') is not False or metadata.get('task')!='segment':
        raise ValueError('exported names/task/head differs from experiment')
    base=str(Path(runtime_root)/scale)
    return dict(model_id=f'yoloe-26{scale}-seg',weights=base+'/'+engine.name,expected_sha256=sha(engine),source_checkpoint_sha256=sha(checkpoint),
        expected_prompt_sha256=sha(prompts),prompt_embeddings=base+'/'+prompts.name,
        model_kind='yoloe',runtime_backend='tensorrt',seg_experiment=True,batched_experiment=True,
        task='segment',front_width=width,batch_mode='2plus1',batch_wait_ms=10.,label_mode='source',
        allowed_names=names,conf_thres=.4,device='cuda:0',require_cuda=True,views=['front','left','right'],
        target_hz_by_view=dict(front=15.,left=15.,right=15.))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--directory',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--policy',type=Path,required=True)
    p.add_argument('--runtime-root',default='/data/models/camera_yoloe_selection_v1')
    a=p.parse_args();policy=yaml.safe_load(a.policy.read_text());a.output.mkdir(parents=True,exist_ok=True)
    for scale in ('m','l'):
        for width in (640,960,1280):
            cfg=configuration(a.directory,scale,width,policy['prompt_profiles']['obstacle17'],a.runtime_root)
            (a.output/f'{scale}_{width}.yaml').write_text(yaml.safe_dump(cfg,sort_keys=False))


if __name__=='__main__':main()
