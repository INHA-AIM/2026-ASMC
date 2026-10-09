#!/usr/bin/env python3
"""Convert existing Capture instance GT; missing BBox is never assumed negative."""
import argparse
from collections import Counter
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
import random
import re
import tarfile

import numpy as np
from PIL import Image

PATTERN = re.compile(r'^(\S+)\s+([-+.\deE]+)\s+([-+.\deE]+)\s+([-+.\deE]+)\s+([-+.\deE]+)\s+(\S+)\s+\((\d+),\s*(\d+),\s*(\d+),\s*(\d+)\)$')
SEMANTIC = {
    (98,2,255): ('person','person'), (236,255,2): ('Obstacle','obstacle'),
    (255,2,2): ('Vehicle','vehicle'), (255,60,60): ('car','vehicle'),
    (255,75,75): ('car','vehicle'), (255,90,90): ('truck','vehicle'),
    (255,105,105): ('bus','vehicle'), (255,135,135): ('car','vehicle'),
    (255,150,150): ('car','vehicle'),
}
LABELS = {'Pedestrian': ('person','person'), 'Obstacle': ('Obstacle','obstacle'),
          'Sedan': ('car','vehicle'), 'SUV': ('car','vehicle'), 'MPV': ('car','vehicle'),
          'Wagon': ('car','vehicle'), 'Truck': ('truck','vehicle'), 'Bus': ('bus','vehicle'),
          'Vehicle': ('Vehicle','vehicle')}


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def parse_bbox(text):
    rows = []
    for line in text.splitlines():
        if not line.strip():
            continue
        match = PATTERN.fullmatch(line.strip())
        if not match:
            raise ValueError('malformed BBox2D: '+line)
        v = match.groups()
        xyxy = list(map(float,v[1:5]))
        if not np.isfinite(xyxy).all() or xyxy[2] < xyxy[0] or xyxy[3] < xyxy[1]:
            raise ValueError('invalid BBox2D coordinates')
        rows.append(dict(gt_source_label=v[0], source_xyxy=xyxy,
                         actor_id=v[5], rgba=list(map(int,v[6:10]))))
    return rows


def split_runs(names, seed=260108):
    names = sorted(names)
    if len(names) != 18:
        raise ValueError('frozen Capture protocol requires exactly 18 runs')
    random.Random(seed).shuffle(names)
    return {name: role for role, subset in [('train',names[:12]), ('selection',names[12:15]),
                                           ('heldout',names[15:])] for name in subset}


def convert_arrays(instance, semantic, rows):
    """Only directly linked actor masks become GT; unmatched actor pixels quarantine."""
    if instance.shape[:2] != semantic.shape[:2] or instance.shape[2] != 4 or semantic.shape[2] != 3:
        raise ValueError('GT channel dimensions disagree')
    known = np.zeros(semantic.shape[:2], bool)
    for color in SEMANTIC:
        known |= np.all(semantic == color, axis=2)
    linked = np.zeros_like(known)
    counts = Counter(tuple(r['rgba']) for r in rows)
    annotations, masks, issues = [], [], []
    for row in rows:
        if row['gt_source_label'] not in LABELS:
            issues.append(dict(reason='unknown_gt_label', **row))
            continue
        mask = np.all(instance == row['rgba'], axis=2)
        if counts[tuple(row['rgba'])] > 1:
            issues.append(dict(reason='ambiguous_instance_color', **row))
            continue
        ys,xs = np.nonzero(mask)
        if not len(xs):
            issues.append(dict(reason='no_visible_instance_pixels', **row))
            continue
        colors, freq = np.unique(semantic[mask],axis=0,return_counts=True)
        color = tuple(colors[int(freq.argmax())].tolist())
        raw,group = LABELS[row['gt_source_label']]
        resolved = SEMANTIC.get(color)
        if resolved is None or resolved[1] != group or freq.max()/len(xs) < .95:
            issues.append(dict(reason='semantic_instance_disagreement', semantic_rgb=color, **row))
            continue
        if raw == 'Vehicle':
            raw = resolved[0]
        ann = dict(row, source_class_name=raw, group=group, semantic_rgb=list(color),
                   visible_pixels=len(xs), xyxy=[int(xs.min()),int(ys.min()),int(xs.max()+1),int(ys.max()+1)],
                   subtype_verified=raw != 'Obstacle' and raw != 'Vehicle')
        annotations.append(ann)
        masks.append(mask)
        linked |= mask
    unmatched = int(np.count_nonzero(known & ~linked))
    if unmatched:
        issues.append(dict(reason='unlinked_actor_semantic_pixels', pixels=unmatched))
    return annotations, masks, issues


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--datasets',type=Path,required=True)
    p.add_argument('--meta-archive',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--workers',type=int,default=4)
    args=p.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    run_dirs=sorted(r for r in args.datasets.iterdir() if (r/'dataset.json').is_file())
    roles=split_runs([r.name for r in run_dirs])
    bbox, missing=[], []
    lookup={}
    with tarfile.open(args.meta_archive,'r:xz') as archive:
        for entry in archive:
            if not entry.name.endswith('_BBox2D.txt'):
                continue
            _,run,view,name=entry.name.split('/')
            key=(run,view,name[:-len('_BBox2D.txt')])
            rows=parse_bbox(archive.extractfile(entry).read().decode('utf-8-sig'))
            lookup[key]=(entry.name,rows)
            if not (args.datasets/run/'frames/intensity'/view/(key[2]+'.png')).is_file():
                missing.append(dict(run=run,view=view,frame=key[2],reason='rgb_not_in_local_capture',bbox_source=entry.name))
    jobs=[]
    for run in run_dirs:
        for view in ('front','left','right'):
            jobs.extend((run,view,path.stem) for path in sorted((run/'frames/intensity'/view).glob('*.png')))
    def convert(job):
        run,view,frame=job
        relative=Path(run.name)/'frames'
        paths={channel:args.datasets/relative/channel/view/(frame+'.png')
               for channel in ('intensity','instance','semantic')}
        row=dict(run=run.name,view=view,frame=frame,split=roles[run.name],
                 image=str(paths['intensity'].relative_to(args.datasets)),annotations=[],issues=[])
        if not all(x.is_file() for x in paths.values()):
            row['issues']=[dict(reason='missing_gt_channel')]
            row['annotation_complete']=False
            return row
        with Image.open(paths['intensity']) as image:
            w,h=image.size
        row.update(width=w,height=h,source_sha256={k:sha(v) for k,v in paths.items()})
        inst=np.array(Image.open(paths['instance']).convert('RGBA'))
        sem=np.array(Image.open(paths['semantic']).convert('RGB'))
        if inst.shape[:2] != (h,w):
            row['issues']=[dict(reason='rgb_gt_shape_mismatch')]
            row['annotation_complete']=False
            return row
        source,rows=lookup.get((run.name,view,frame),('',[]))
        annotations,masks,issues=convert_arrays(inst,sem,rows)
        row.update(annotations=annotations,issues=issues,bbox_source=source,
                   annotation_complete=not issues, bbox_file_present=bool(source))
        if masks:
            dest=args.output/'masks'/run.name/view/(frame+'.npz')
            dest.parent.mkdir(parents=True,exist_ok=True)
            np.savez_compressed(dest, masks=np.packbits(np.stack(masks),axis=2), width=w,height=h)
            row['masks']=str(dest.relative_to(args.output))
        return row
    counts=Counter(); issues=Counter(); groups=Counter(); examples=[]
    with (args.output/'frames.jsonl').open('w') as out, ThreadPoolExecutor(max_workers=args.workers) as executor:
        for index,row in enumerate(executor.map(convert,jobs)):
            out.write(json.dumps(row)+'\n')
            counts['frames']+=1
            counts[row['split']+'_frames']+=1
            counts['complete_frames' if row['annotation_complete'] else 'quarantined_frames']+=1
            counts['annotations']+=len(row['annotations'])
            if row['annotation_complete']:
                counts[row['split']+'_complete']+=1
                groups.update(a['group'] for a in row['annotations'])
            for issue in row['issues']:
                issues[issue['reason']]+=1
            if row['issues'] and len(examples)<100:
                examples.append(row)
            if (index+1)%500==0:
                print('GT',index+1,'/',len(jobs),dict(counts),flush=True)
    summary=dict(schema_version=1,split_seed=260108,splits=roles,counts=dict(counts),issues=dict(issues),
                 complete_group_counts=dict(groups),meta_sha256=sha(args.meta_archive),
                 manifest_sha256=sha(args.output/'frames.jsonl'),missing_rgb=missing,
                 dataset_metadata_sha256={r.name:sha(r/'dataset.json') for r in run_dirs},
                 unresolved_examples=examples,subtype_labels_manually_verified=False,
                 scope='direct RGBA actor/instance links; unknown/missing GT quarantined, no inferred instances',
                 animal_gt_present=bool(groups['animal']),training_performed=False)
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print('COMPLETE',json.dumps(dict(counts)),flush=True)


if __name__=='__main__':
    main()
