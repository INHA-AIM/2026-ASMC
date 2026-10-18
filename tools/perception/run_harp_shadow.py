#!/usr/bin/env python3
"""Run original HARP-NeXt checkpoint/preprocessor; record point lineage and cost."""
import argparse
from collections import Counter
import hashlib
import json
import os
from pathlib import Path
import sys
import time
import numpy as np
import torch
import yaml


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--repo',type=Path,required=True);p.add_argument('--inputs',type=Path,required=True)
    p.add_argument('--weights',type=Path,required=True);p.add_argument('--dataset',choices=('nuscenes','semantic_kitti'),required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--limit',type=int)
    p.add_argument('--preprocessing',choices=('cpu','gpu'),default='cpu')
    p.add_argument('--intensity-scale',type=float,default=1.)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    sys.path.insert(0,str(a.repo));os.chdir(a.repo)
    from core.network import Network
    from datasets.pc_processors.harpnext_pcprocessor import HARPNeXtPCProcessor
    config=a.repo/'configs/net'/('harpnext-nuscenes.yaml' if a.dataset=='nuscenes' else 'harpnext-semantickitti.yaml')
    cfg=yaml.safe_load(config.read_text());meta=json.loads((a.inputs/'manifest.json').read_text())
    scans=meta['rows'][:a.limit] if a.limit else meta['rows']
    class Adapter(HARPNeXtPCProcessor):
        def load_pc(self,index):
            with np.load(a.inputs/f'{scans[index]["stamp_ns"]}.npz') as item: points=item['xyzi'].copy()
            points[:,3]*=a.intensity_scale
            # Labels are unused by the network. Zero arrays only satisfy official preprocessor API.
            return points,np.zeros(len(points),np.int64),np.zeros(len(points),np.int64),'','',''
    processor=Adapter(dataset=a.dataset,input_feat=['intensity'],phase='val',batch_size=1,
        preproc_gpu=a.preprocessing=='gpu',rank=0,**cfg['range_proj'])
    torch.set_num_threads(2);torch.manual_seed(0);np.random.seed(0)
    model=Network('harpnext',cfg).build_network()
    checkpoint=torch.load(a.weights,map_location='cpu',weights_only=False)
    state=checkpoint['net'];state={k.removeprefix('module.'):v for k,v in state.items()}
    model.load_state_dict(state,strict=True);model.eval().cuda()
    torch.cuda.reset_peak_memory_stats();times=[];counts=Counter();fov_clipped=0;point_count=0
    with (a.output/'observations.jsonl').open('w') as stream:
        for i,scan in enumerate(scans):
            start=time.perf_counter()
            if a.preprocessing=='gpu':
                pc,labels=processor.load_batch_to_gpu(i)
                batch,interpolated=processor.process_batch_gpu(pc,labels)
                net_inputs=dict(points=batch['points'],voxels=dict(voxels=batch['voxels'],coors=batch['coors']))
            else:
                batch,interpolated=processor.process_batch_cpu(i)
                net_inputs=dict(points=[v.cuda() for v in batch['points']],
                    voxels=dict(voxels=batch['voxels'].cuda(),coors=batch['coors'].cuda()))
            torch.cuda.synchronize();pre_ms=(time.perf_counter()-start)*1000
            if i==0:
                with torch.inference_mode(),torch.autocast('cuda',dtype=torch.float16):
                    for _ in range(3):model(net_inputs,training=False)
                torch.cuda.synchronize()
            start=time.perf_counter()
            with torch.inference_mode(),torch.autocast('cuda',dtype=torch.float16):
                logits=model(net_inputs,training=False)['seg_logits']
                probability=torch.softmax(logits.float(),dim=1)
                confidence,prediction=probability.max(1)
                result=prediction[:scan['points']].byte().cpu().numpy()
                conf=confidence[:scan['points']].cpu().numpy()
            torch.cuda.synchronize();forward_ms=(time.perf_counter()-start)*1000
            with np.load(a.inputs/f'{scan["stamp_ns"]}.npz') as item:
                xyz=item['xyzi'][:,:3];raw=item['raw_index'].copy()
            pitch=np.degrees(np.arcsin(xyz[:,2]/np.linalg.norm(xyz,axis=1)))
            clipped=int(((pitch>cfg['range_proj']['fov_up'])|(pitch<cfg['range_proj']['fov_down'])).sum())
            fov_clipped+=clipped;point_count+=len(xyz)
            np.savez_compressed(a.output/f'{scan["stamp_ns"]}.npz',raw_index=raw,semantic=result,confidence=conf)
            count=Counter(map(int,result));counts.update(count)
            row=dict(scan,preprocess_h2d_ms=pre_ms,forward_output_ms=forward_ms,
                     interpolated_points=len(interpolated)-len(xyz),fov_clipped_points=clipped,
                     classes=dict(count),mean_confidence=float(conf.mean()))
            stream.write(json.dumps(row)+'\n');times.append(row)
            if (i+1)%40==0:print(a.dataset,i+1,'/',len(scans),flush=True)
    q=lambda v:{f'p{x}':float(np.percentile(v,x)) for x in (50,95,99)}
    class_names=(processor.scan_gpu if a.preprocessing=='gpu' else processor.scan).classes_names
    summary=dict(dataset=a.dataset,scans=len(scans),classes=class_names,
        class_points={class_names[k]:v for k,v in counts.items()},
        preprocess_h2d_ms=q([r['preprocess_h2d_ms'] for r in times]),forward_output_ms=q([r['forward_output_ms'] for r in times]),
        total_ms=q([r['preprocess_h2d_ms']+r['forward_output_ms'] for r in times]),
        torch_peak_allocated_mib=torch.cuda.max_memory_allocated()/2**20,torch_peak_reserved_mib=torch.cuda.max_memory_reserved()/2**20,
        weights_sha256=hashlib.sha256(a.weights.read_bytes()).hexdigest(),config_sha256=hashlib.sha256(config.read_bytes()).hexdigest(),
        repo_revision=(a.repo/'.git'/((a.repo/'.git/HEAD').read_text().strip()[5:])).read_text().strip()
                      if (a.repo/'.git/HEAD').read_text().startswith('ref: ') else (a.repo/'.git/HEAD').read_text().strip(),
        input_manifest_sha256=hashlib.sha256((a.inputs/'manifest.json').read_bytes()).hexdigest(),strict_checkpoint=True,
        intensity_scale=a.intensity_scale,
        intensity_policy='raw unchanged for nuScenes; SemanticKITTI /255 is a declared byte-to-unit input adaptation, not a calibrated reflectance conversion',
        range_projection=cfg['range_proj'],clipped_fraction=fov_clipped/point_count,
        preprocessing=a.preprocessing+' official RangeInterpolation + SemLaserScan + Collate; appended points excluded from raw prediction output',
        script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        torch=torch.__version__,precision='autocast float16',semantic_gt=False,
        scope='single original scan, no ground removal; shared GPU with other chat workload; no realtime or velocity acceptance')
    (a.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary),flush=True)


if __name__=='__main__':main()
