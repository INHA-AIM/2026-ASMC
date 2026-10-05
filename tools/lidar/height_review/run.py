#!/usr/bin/env python3
"""Run independent variants and exact deterministic repeats; never overwrite results."""
import argparse
import json
from pathlib import Path
import subprocess
from prepare import rows, sha, dump

def comparable(r):
    return {k:v for k,v in r.items() if k not in ('segment_ms','timing_ms')}

def main():
    p=argparse.ArgumentParser();p.add_argument('--binary',type=Path,required=True)
    p.add_argument('--inputs',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    records=[]
    for dataset in ('manual','stationary','capture'):
        for representation,attitude in [('voxel','pitch'),('raw','pitch'),('voxel','full')]:
            policies=('jaeho','original','singleton','fixed') if attitude=='pitch' else ('original','singleton','fixed')
            for policy in policies:
                name=f'{dataset}_{representation}_{attitude}_{policy}'
                common=[str(a.binary),str(a.inputs/dataset/'snapshots.jsonl'),representation,attitude,policy,'trace']
                files=[]
                for suffix in ('','_repeat'):
                    output=a.output/(name+suffix+'.jsonl');files.append(output)
                    with (a.output/(name+suffix+'.log')).open('w') as log:
                        subprocess.run(common+[str(output)],stdout=log,stderr=subprocess.STDOUT,check=True)
                first,second=map(rows,files)
                assert len(first)==len(second) and all(comparable(x)==comparable(y) for x,y in zip(first,second)),name
                if policy=='fixed':assert all(r['height_before']==1.8 and r['height_after']==1.8 for r in first)
                records.append({'name':name,'rows':len(first),'deterministic':True,
                                'sha256':sha(files[0]),'command':common+[str(files[0])]})
                print(name,len(first),'deterministic',flush=True)
    # Timing without patch logging, on the primary continuous input only.
    for policy in ('original','singleton','fixed'):
        output=a.output/f'manual_voxel_pitch_{policy}_quiet.jsonl'
        with output.with_suffix('.log').open('w') as log:
            subprocess.run([str(a.binary),str(a.inputs/'manual/snapshots.jsonl'),
                            'voxel','pitch',policy,'quiet',str(output)],stdout=log,stderr=subprocess.STDOUT,check=True)
        quiet=rows(output);traced=rows(a.output/f'manual_voxel_pitch_{policy}.jsonl')
        assert all(x['mask']==y['mask'] and x['height_after']==y['height_after'] for x,y in zip(quiet,traced))
    dump(a.output/'runs.json',{'runs':records,'binary_sha256':sha(a.binary),
         'inputs_manifest_sha256':sha(a.inputs/'manifest.json'),
         'timing_caveat':'jaeho segment includes existing clustering/OBB; not comparable to PP segmentation-only',
         'no_operating_change':True})

if __name__=='__main__':main()
