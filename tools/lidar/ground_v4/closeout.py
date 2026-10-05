#!/usr/bin/env python3
"""Recompute frozen gates and fingerprint the actual inputs/results; no tuning."""
import argparse
import hashlib
import json
from pathlib import Path

def sha(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for part in iter(lambda:f.read(1024*1024),b''):h.update(part)
    return h.hexdigest()

def actor_gate(summary,candidate,baseline):
    failures=[]
    for cls,count in summary['additional_complete_loss'].get(candidate,{}).items():
        if count:failures.append(f'{cls}: {count} additional complete losses')
    for key,b in summary['objects'][baseline].items():
        if not key.endswith('/observed10plus'):continue
        c=summary['objects'][candidate][key]
        for count,total in [('retained','input'),('lower_retained','lower_input')]:
            if b[total] and c[count]*b[total]<b[count]*c[total]:
                failures.append(f'{key}: decreased {count}')
    for cls,b in summary['classes'][baseline].items():
        c=summary['classes'][candidate][cls]
        if c['uncertain']*b['input']>b['uncertain']*c['input']:
            failures.append(f'{cls}: increased uncertain fraction')
    return {'pass':not failures,'failures':failures}

def main():
    p=argparse.ArgumentParser();p.add_argument('data_root',type=Path)
    p.add_argument('output',type=Path);a=p.parse_args()
    root=Path(__file__).resolve().parents[3]
    prefix=a.data_root/'lidar_assessment';v4=prefix/'20261005_ground_v4'
    inputs={n:prefix/'20261004_gt_dual_ground'/n for n in ['sync','raw']}
    inputs.update({n:prefix/'20261005_ground_v3_drive'/(n+'_gt') for n in ['general','tunnel','banked']})
    record={'schema':'lidar_ground_v4_closeout_v1','operating_output_changed':False,
            'datasets':{},'actor_gates':{},'input_index':'input-files.jsonl',
            'hash_note':'Final audit fingerprints. Earlier run manifests omitted shared-library SHA; repeated candidate geometry is checked separately.',
            'selection':'jaeho_support_as_followup_development_baseline_not_operational_acceptance'}
    a.output.mkdir(parents=True,exist_ok=False)
    with (a.output/'input-files.jsonl').open('x') as index:
        for name,folder in inputs.items():
            with (folder/'snapshots.jsonl').open() as f:
                rows=[json.loads(line) for line in f]
            for row in rows:
                for file in [folder/row['cloud_path'],folder/'labels'/(row['input_id']+'.npz')]:
                    index.write(json.dumps({'path':str(file.relative_to(a.data_root)),
                        'bytes':file.stat().st_size,'sha256':sha(file)},sort_keys=True)+'\n')
            summary_path=v4/(name+'_support_score')/'summary.json'
            s=json.loads(summary_path.read_text())
            files=[folder/'snapshots.jsonl',folder/'source.json',summary_path,
                   v4/(name+'_score')/'summary.json']
            files.extend(v4/name/(c+'.jsonl') for c in ['jaeho_height','jaeho_support','patchwork_support'])
            record['datasets'][name]={'frames':len(rows),'files':{
                str(f.relative_to(a.data_root)):sha(f) for f in files}}
            if name in ['raw','sync']:
                record['actor_gates'][name]={n:actor_gate(s,n,ref) for n,ref in [
                    ('jaeho_height','jaeho_baseline'),('patchwork_support','patchwork_baseline')]}
    record['input_index_sha256']=sha(a.output/'input-files.jsonl')
    tunnel=json.loads((v4/'tunnel_support_score/summary.json').read_text())
    before=tunnel['plane_diagnostics']['jaeho_baseline'];after=tunnel['plane_diagnostics']['jaeho_height']
    record['tunnel_plane_gate']={'before':before,'after':after,
        'pass':after['positive']==0 and after['missing']==0 and
        after.get('height_jump_over_0p5m',0)<=.25*before['height_jump_over_0p5m']}
    bank=json.loads((v4/'banked_support_score/summary.json').read_text())
    b=bank['classes']['patchwork_baseline']['asphalt'];c=bank['classes']['patchwork_support']['asphalt']
    reduction=1-(c['retained']/c['input'])/(b['retained']/b['input'])
    record['patchwork_banked_gate']={'relative_residual_reduction':reduction,'pass':reduction>=.25}
    patterns=['include/asmc_lidar/*','src/*.cpp','config/ground_v4/*.yaml','config/autonav_v3.yaml','CMakeLists.txt']
    files=set()
    for pattern in patterns:files.update((root/'src/asmc_lidar').glob(pattern))
    files.update(Path(__file__).parent.glob('*.py'))
    files.add(Path(__file__).parent/'EXPERIMENT.md')
    files.update(root/'devel'/f for f in ['lib/libasmc_lidar_core.so','lib/libasmc_ground_live_backend.so',
        'lib/asmc_lidar/asmc_lidar_ground_live_offline','lib/asmc_lidar/asmc_lidar_ground_live_node'])
    record['source_and_binary_sha256']={str(f.relative_to(root)):sha(f) for f in sorted(files) if f.is_file()}
    record['limits']=['New drives have no actor GT.',
        'Jaeho lower-object loss and observed-mask fluctuations remain.',
        'Score key jaeho_height denotes jaeho_support when summary jaeho_profile says so.',
        'No wall/clustering/tracking/planner acceptance; Patchwork environment guard candidate rejected, not upstream algorithm in general.']
    (a.output/'manifest.json').write_text(json.dumps(record,indent=2,sort_keys=True)+'\n')
    print(json.dumps({k:record[k] for k in ['actor_gates','tunnel_plane_gate','patchwork_banked_gate']},indent=2))

if __name__=='__main__':main()
