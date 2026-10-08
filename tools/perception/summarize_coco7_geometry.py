#!/usr/bin/env python3
"""Explain verified-GT recall by view and apparent height without inventing distance."""
import argparse
from collections import defaultdict
import json
from pathlib import Path
from evaluate_coco7_models import evaluate


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--gt',type=Path,required=True)
    p.add_argument('--predictions',type=Path,required=True)
    p.add_argument('--confidence',type=float,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    gt={r['image']:r for r in map(json.loads,a.gt.read_text().splitlines())}
    counts=defaultdict(lambda:dict(gt=0,tp=0))
    for row in map(json.loads,a.predictions.read_text().splitlines()):
        truth=gt[row['image']]
        if row['source_sha256']!=truth['source_sha256']: raise RuntimeError('source SHA mismatch')
        _,_,pairs,known,_=evaluate(truth['annotations'],row['predictions'],a.confidence)
        matched={g for g,_,_ in pairs}
        for g,ann in enumerate(known):
            height=ann['xyxy'][3]-ann['xyxy'][1]
            scale='under10px' if height<10 else '10to29px' if height<30 else 'atleast30px'
            for view in ('all',truth['view']):
                counts[view,ann['source_class_name'],scale]['gt']+=1
                counts[view,ann['source_class_name'],scale]['tp']+=g in matched
    rows=[dict(view=view,label=label,height_bin=scale,**c,recall=c['tp']/c['gt'])
          for (view,label,scale),c in sorted(counts.items())]
    a.output.write_text(json.dumps(dict(confidence=a.confidence,rows=rows,
        interpretation='apparent visible GT height in original pixels; not physical distance or a replacement for the fixed macro-F1 selection metric'),indent=2)+'\n')


if __name__=='__main__': main()
