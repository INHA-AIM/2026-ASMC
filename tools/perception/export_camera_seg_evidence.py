#!/usr/bin/env python3
"""Package complete reports and checksum large local raw artifacts."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import tarfile


def sha(path):
    h=hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda:stream.read(1<<20),b''):h.update(chunk)
    return h.hexdigest()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--raw',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    summary=json.loads((a.raw/'matrix-v1/summary.json').read_text())
    if not summary['protocol_complete']:raise ValueError('fixed protocol incomplete; no completed baseline export')
    for name,source in [('matrix-summary',a.raw/'matrix-v1/summary.json'),('gt-summary',a.raw/'gt-v2/summary.json'),
                        ('gt-lineage-summary',a.raw/'gt-lineage-v1/summary.json'),('accuracy-strata',a.raw/'accuracy-strata-v1/summary.json'),
                        ('mask-support',a.raw/'mask-support-v1/summary.json'),
                        ('parity',a.raw/'parity-v1.json'),
                        ('gt-selection-identity',a.raw/'gt-selection-identity.json')]:
        shutil.copyfile(source,a.output/(name+'.json'))
    for model in ('baseline','yolo26','yoloe'):
        shutil.copyfile(a.raw/'accuracy'/model/'summary.json',a.output/('accuracy-'+model+'.json'))
    shutil.copytree(a.raw/'comparison-gallery-v1',a.output/'gallery')
    shutil.copytree(a.raw/'mask-support-v1/gallery',a.output/'mask-support-gallery')
    # Keep all successful/failed execution logs, metrics, observations and metadata.
    folders=['runtime','matrix-v1','smoke-v1','accuracy-strata-v1','mask-support-v1']
    with tarfile.open(a.output/'runtime-raw.tar.gz','w:gz',compresslevel=6) as archive:
        for folder in folders:
            archive.add(a.raw/folder,arcname=folder)
        for file in ('parity-v1.json','parity-v1.frames.json','gt-selection-identity.json'):
            archive.add(a.raw/file,arcname=file)
    # Offline results and packed masks allow rescoring without another GPU run.
    with tarfile.open(a.output/'selection-raw.tar.gz','w:gz',compresslevel=6) as archive:
        archive.add(a.raw/'gt-v2',arcname='gt-v2')
        for file in ('summary.json','frames.jsonl'):
            archive.add(a.raw/'gt-v1'/file,arcname='gt-v1-initial/'+file)
        archive.add(a.raw/'gt-lineage-v1',arcname='gt-lineage-v1')
        for model in ('baseline','yolo26','yoloe'):
            folder=a.raw/'accuracy'/model
            for file in ('summary.json','predictions.jsonl'):
                archive.add(folder/file,arcname='accuracy/'+model+'/'+file)
            archive.add(folder/'masks',arcname='accuracy/'+model+'/masks')
    manifest=dict(raw_root=str(a.raw.resolve()),files={str(f.relative_to(a.raw)):dict(bytes=f.stat().st_size,sha256=sha(f))
                   for f in sorted(a.raw.rglob('*')) if f.is_file()},
                  archives={f.name:dict(bytes=f.stat().st_size,sha256=sha(f)) for f in a.output.glob('*.tar.gz')},
                  rgb_sources='existing Capture data/datasets and original archive; per-frame SHA in GT manifest',
                  model_sources='data/models/camera_seg_v1/sources.json; weight/encoder SHA in matrix manifest',
                  training_performed=False,heldout_inferred=False)
    (a.output/'raw-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps(dict(files=len(manifest['files']),archives=manifest['archives'])))


if __name__=='__main__':main()
