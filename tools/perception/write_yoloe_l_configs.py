#!/usr/bin/env python3
"""Pin prepared prompt/engine hashes without importing CUDA on the host."""
import argparse
import hashlib
from pathlib import Path
import yaml
from prepare_yoloe_l import profile_names


def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--prepared',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--policy',type=Path,required=True);p.add_argument('--runtime-directory',default='/data/models/camera_yoloe_l_v1')
    a=p.parse_args();policy=yaml.safe_load(a.policy.read_text());a.output.mkdir(parents=True,exist_ok=True)
    checkpoint_sha=sha(a.prepared/'yoloe-26l-seg.pt')
    for profile,names in policy['prompt_profiles'].items():
        names = profile_names(policy, profile)
        for backend in ['pytorch','tensorrt']:
            filename='yoloe-26l-seg.pt' if backend=='pytorch' else profile+'.engine'
            if not (a.prepared/filename).is_file():continue
            for mode in policy['first_run']['batch_modes']:
                config=dict(views=['front','left','right'],image_topic='/image_jpeg/compressed',
                    weights=a.runtime_directory+'/'+filename,expected_sha256=sha(a.prepared/filename),
                    source_checkpoint_sha256=checkpoint_sha,expected_prompt_sha256=sha(a.prepared/(profile+'.npz')),
                    device='cuda:0',require_cuda=True,conf_thres=.7,allowed_names=names,
                    seg_experiment=True,batched_experiment=True,model_kind='yoloe',runtime_backend=backend,
                    batch_mode=mode,batch_wait_ms=10,prompt_embeddings=a.runtime_directory+'/'+profile+'.npz')
                (a.output/(profile+'_'+backend+'_'+mode+'.yaml')).write_text(yaml.safe_dump(config,sort_keys=False))


if __name__=='__main__':main()
