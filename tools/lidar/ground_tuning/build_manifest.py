#!/usr/bin/env python3
"""Source, configuration, data lineage and result inventory for the V2 review."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

def sha(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda:f.read(1024*1024),b''):h.update(block)
    return h.hexdigest()

def rec(path,root):
    return {'path':str(path.relative_to(root)),'bytes':path.stat().st_size,'sha256':sha(path)}

p=argparse.ArgumentParser();p.add_argument('--repo',type=Path,required=True);p.add_argument('--data',type=Path,required=True)
p.add_argument('--output',type=Path,required=True);a=p.parse_args()
if a.output.exists():raise FileExistsError(a.output)
repo=a.repo.resolve();data=a.data.resolve();out=data/'lidar_assessment/20261004_ground_v2'
old_inventory=data/'lidar_assessment/20261004_ground_live_v1/input_inventory_v1.json'
inputs=json.loads(old_inventory.read_text())
for f in inputs:
    path=data/f['path']
    if path.stat().st_size!=f['bytes'] or sha(path)!=f['sha256']:raise ValueError('input changed: '+str(path))
manual=data/'lidar_assessment/20261003_l2a2_h1/regenerated_inputs/manual'
manual_inputs=[rec(manual/'snapshots.jsonl',data)]
with (manual/'snapshots.jsonl').open() as f:
    for text in f:
        row=json.loads(text);manual_inputs.append(rec(manual/row['cloud_path'],data))
sources=[]
for directory in ('src/asmc_lidar','tools/lidar/ground_tuning','tools/lidar/ground_live_review','tools/lidar/height_review','tools/lidar/frame_time'):
    sources.extend(rec(f,repo) for f in sorted((repo/directory).rglob('*')) if f.is_file() and '__pycache__' not in f.parts)
results=[rec(f,data) for f in sorted(out.rglob('*')) if f.is_file() and f!=a.output]
gates={}
for name in ('sync','raw'):
    s=json.loads((out/(name+'_metrics')/'summary.json').read_text())
    gates[name]={n:{k:g[k] for k in ('object_pass','structural_pass','eligible_on_this_recording','violations')} for n,g in s['gates'].items()}
eligible=sorted(n for n in gates['sync'] if all(gates[d][n]['eligible_on_this_recording'] for d in gates))
git=lambda *args:subprocess.check_output(['git',*args],cwd=repo,text=True).strip()
algorithm_paths=['src/asmc_lidar/src/'+n for n in ('core.cpp','general.cpp','ground_live.cpp','ground_config.cpp')]+[
    'src/asmc_lidar/include/asmc_lidar/ground_live.hpp','tools/lidar/ground_tuning/generate_diagnostics.py',
    'tools/lidar/ground_live_review/generate_linefit.py','tools/lidar/height_review/generate.py']
for path in algorithm_paths:
    original=subprocess.check_output(['git','show','848587d:'+path],cwd=repo)
    if hashlib.sha256(original).hexdigest()!=sha(repo/path):raise ValueError('frozen algorithm changed: '+path)
manifest={'schema':'asmc-lidar-l2a2-v2-review-1','base_revision':'3d43ca7','revision':git('rev-parse','HEAD'),
    'algorithm_revision':'848587d','algorithm_files_verified_unchanged':algorithm_paths,
    'branch':git('branch','--show-current'),'data_root_variable':'ASMC_DATA',
    'identity':'file hashes plus per-profile executable hashes; uncommitted review additions are explicit',
    'source_files':sources,'verified_input_inventory':rec(old_inventory,data),'verified_input_files':len(inputs),
    'manual_inputs':manual_inputs,
    'manual_source_manifest':rec(manual.parent/'manifest.json',data),
    'results':results,'gates':gates,'eligible_on_both_recordings':eligible,
    'decision':'no operating replacement; user review required','defaults_changed':False,
    'fixed_conditions':{'voxel_m':.1,'leveling':'pitch_only','patchwork_height_m':1.8,'linefit_height_m':1.78,
        'patchwork_guard_m':.25,'ground_truth_input_to_backend':False,'topology_radii_m':[.2,.4,.8],'minimum_actor_points':10},
    'limitations':['Previously analysed data, not held out.','No verified temporal surface correspondence.',
       'Live movement is a bounded short gRPC smoke, not full-route acceptance.','No 15Hz end-to-end acceptance.',
       'Ground segmentation is not detector or tracking performance.']}
a.output.write_text(json.dumps(manifest,indent=2,sort_keys=True)+'\n')
print(json.dumps({'verified_input_files':len(inputs),'source_files':len(sources),'result_files':len(results),'eligible':eligible}))
