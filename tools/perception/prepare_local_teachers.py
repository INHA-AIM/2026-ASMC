#!/usr/bin/env python3
"""Download official local teacher assets, pin revisions and verify file hashes.

Preparation is networked; inference uses local_files_only and an isolated master.
No API key, image upload, shared ROS package install or TensorRT generation.
"""
import argparse
import fcntl
import json
from pathlib import Path
import subprocess
import time
import urllib.request

from camera_data_audit import save, sha


def download(url, target, chunk_size=4*1024*1024):
    """Range requests survive interrupted large public downloads; never skip bytes."""
    target.parent.mkdir(parents=True,exist_ok=True)
    with target.with_suffix(target.suffix+'.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        _download_locked(url,target,chunk_size)


def _download_locked(url,target,chunk_size):
    temporary=target.with_suffix(target.suffix+'.partial')
    if target.exists():return
    failures=0
    total=None
    while total is None or (temporary.stat().st_size if temporary.exists() else 0)<total:
        start=temporary.stat().st_size if temporary.exists() else 0
        request=urllib.request.Request(url,headers={'Range':'bytes=%d-%d'%(start,start+chunk_size-1)})
        try:
            with urllib.request.urlopen(request,timeout=120) as response:
                content_range=response.headers.get('Content-Range')
                if content_range:
                    range_part,size=content_range.split('/')
                    if int(range_part.split()[1].split('-')[0])!=start:raise ValueError('range start mismatch')
                    total=int(size)
                elif start:
                    raise ValueError('server ignored resume range')
                else:
                    total=int(response.headers['Content-Length'])
                with temporary.open('ab') as output:
                    while True:
                        block=response.read(256*1024)
                        if not block:break
                        output.write(block)
            failures=0
        except Exception as error:
            failures+=1
            print('download retry',failures,type(error).__name__,flush=True)
            if failures>=10:raise
            time.sleep(min(failures,5))
        if total is not None and temporary.exists() and temporary.stat().st_size>total:raise ValueError('download size overflow')
    temporary.rename(target)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--models',type=Path,required=True);p.add_argument('--source',type=Path,required=True)
    p.add_argument('--config',type=Path,default=Path(__file__).resolve().parents[2]/'config/experiments/camera_local_teacher_v6.json')
    args=p.parse_args();root=args.models/'teacher_v6';root.mkdir(parents=True,exist_ok=True)
    from huggingface_hub import HfApi
    api=HfApi();pins={}
    for repository,folder in [('IDEA-Research/grounding-dino-base','grounding-dino-base'),('google-bert/bert-base-uncased','bert-base-uncased')]:
        pin_path=root/(folder+'.pin.json')
        if pin_path.exists():pin=json.loads(pin_path.read_text())
        else:
            info=api.model_info(repository,files_metadata=True)
            pin=dict(repository=repository,revision=info.sha,
                files=[dict(name=s.rfilename,sha256=s.lfs.sha256 if s.lfs else None) for s in info.siblings
                    if s.rfilename in ('config.json','preprocessor_config.json','tokenizer.json','tokenizer_config.json','special_tokens_map.json','vocab.txt','model.safetensors')])
            save(pin_path,pin)
        directory=root/folder;directory.mkdir(exist_ok=True)
        for item in pin['files']:
            path=directory/item['name']
            if not path.exists():
                print('prepare',repository,item['name'],flush=True)
                download('https://huggingface.co/%s/resolve/%s/%s'%(repository,pin['revision'],item['name']),path)
            if item['sha256'] and sha(path)!=item['sha256']:raise ValueError('official LFS SHA mismatch: '+str(path))
        pins[folder]=dict(pin,local_files_sha256={x.name:sha(x) for x in directory.iterdir() if x.is_file()})
        save(root/'preparation-progress.json',pins)
    config=json.loads(args.config.read_text())
    spec=config['models']['mm_gdino_l']
    download(spec['checkpoint_url'],root/spec['checkpoint'])
    expected_prefix=Path(spec['checkpoint']).stem.rsplit('-',1)[-1]
    if not sha(root/spec['checkpoint']).startswith(expected_prefix):
        raise ValueError('official MM checkpoint filename SHA prefix mismatch')
    sources={}
    for name in ('mmdetection','GroundingDINO'):
        path=args.source/name
        sources[name]=dict(commit=subprocess.check_output(['git','-C',str(path),'rev-parse','HEAD'],text=True).strip(),
                          license_sha256=sha(path/'LICENSE'))
    save(root/'provenance.json',dict(huggingface=pins,source=sources,
        mm_gdino_l=dict(checkpoint=spec['checkpoint'],url=spec['checkpoint_url'],sha256=sha(root/spec['checkpoint'])),
        config_sha256=sha(args.config),images_uploaded=False,training_performed=False))


if __name__=='__main__':main()
