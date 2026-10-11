#!/usr/bin/env python3
"""Serial checkpoint-specific preparation; preserve every failed export attempt."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--directory',type=Path,required=True)
    p.add_argument('--policy',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--scales',nargs='+',choices=['m','l'],default=['m','l'])
    p.add_argument('--widths',nargs='+',type=int,choices=[640,960,1280],default=[640,960,1280])
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    script=Path(__file__).with_name('prepare_yoloe_l.py');rows=[]
    for scale in a.scales:
        for width,stage in [(1280,'prompts')]+[(w,s) for w in a.widths for s in ('onnx','engine')]:
            label=f'{scale}-{width}-{stage}'
            command=[sys.executable,str(script),'--directory',str(a.directory/scale),'--config',str(a.policy),
                     '--profile','obstacle17','--scale',scale,'--front-width',str(width),'--variant','--stage',stage]
            row=dict(label=label,command=command,started_wall=time.time());rows.append(row)
            print('START '+label,flush=True)
            with (a.output/(label+'.log')).open('w') as log:
                child=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT)
            row.update(return_code=child.returncode,ended_wall=time.time())
            (a.output/'index.json').write_text(json.dumps(rows,indent=2)+'\n')
            print('END '+label+' '+str(child.returncode),flush=True)
            if child.returncode:return child.returncode
    return 0


if __name__=='__main__':raise SystemExit(main())
