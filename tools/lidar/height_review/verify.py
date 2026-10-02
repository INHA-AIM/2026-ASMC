#!/usr/bin/env python3
"""Read-only verification of the H1 release of local research artifacts."""
import argparse
import json
from pathlib import Path
import yaml
from prepare import sha, rows
from run import comparable

def main():
    p=argparse.ArgumentParser();p.add_argument('--repo-root',type=Path,required=True)
    p.add_argument('--data-root',type=Path,required=True);a=p.parse_args()
    m=yaml.safe_load((a.repo_root/'config/baselines/lidar_l2a2_h1_20261003.yaml').read_text())
    assert m['operational_change'] is False and m['patchwork_adopted'] is False
    for path,expected in m['tool_sources'].items():
        assert sha(a.repo_root/path)==expected,path
    root=a.data_root.resolve();artifacts={}
    for key in ('inputs','audit','results','report','synthetic'):
        path=(root/m[key]['path']).resolve()
        assert root in path.parents
        assert sha(path)==m[key]['sha256'],key
        artifacts[key]=json.loads(path.read_text())
    input_root=(root/m['inputs']['path']).parent
    for name,item in artifacts['inputs']['files'].items():
        path=(input_root/name).resolve();assert input_root in path.parents
        assert path.stat().st_size==item['bytes'] and sha(path)==item['sha256'],name
    result_root=(root/m['results']['path']).parent
    assert len(artifacts['results']['runs'])==33
    repeated=0
    for run in artifacts['results']['runs']:
        path=result_root/(run['name']+'.jsonl')
        assert sha(path)==run['sha256']
        first=rows(path);second=rows(result_root/(run['name']+'_repeat.jsonl'))
        assert len(first)==len(second)==run['rows']
        assert all(comparable(x)==comparable(y) for x,y in zip(first,second)),run['name']
        repeated+=len(first)
    assert artifacts['report']['adoption']=='HOLD'
    gate=artifacts['report']['candidate_gate']
    assert gate['fixed_height_stable'] and gate['additional_complete_loss_vs_original']==0
    assert gate['additional_complete_loss_vs_jaeho']==2
    assert len(artifacts['synthetic']['invalid_rejected'])==13
    assert artifacts['audit']['frames_without_ring0_asphalt']==61
    print(json.dumps({'artifact_manifests':5,'input_files':len(artifacts['inputs']['files']),
                      'deterministic_combinations':33,'paired_rows':repeated,'decision':'HOLD'}))

if __name__=='__main__':main()
