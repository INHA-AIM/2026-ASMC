#!/usr/bin/env python3
"""Render same-scan filter stages; no independent object GT or accuracy claim."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
import yaml
from tunnel_case_geometry import clouds,projection,project_leveled


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--protocol',type=Path,required=True)
    parser.add_argument('--source',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    args.output.mkdir(parents=True,exist_ok=False)
    root=Path(__file__).resolve().parents[2]
    b0=root/'config/baselines/evidence/camera_lidar_b0_v1'
    cases=json.loads((b0/'qualitative-review.json').read_text())['cases']
    setting=args.source/'src/Main/setting.yaml'
    vehicle=yaml.safe_load(setting.read_text())['Vehicle']; p=projection(vehicle)
    palette={0:'#34cf72',1:'#cf60df',2:'#dc9f36',3:'#fa5959',4:'#aaaaaa',255:'#aaaaaa'}
    results=[]
    for bag in ('primary','shadow'):
        selected=[c for c in cases if c['case'].startswith('cases-'+bag+'/')]
        records={}; data={}; indices={}
        for profile in ('T1','T3','T4'):
            folder=args.protocol/('offline-'+bag+'-'+profile+'-invalid-r1')
            records[profile]=[json.loads(s) for s in (folder/'frames.jsonl').read_text().splitlines()]
            indices[profile]=[min(range(len(records[profile])),key=lambda i:abs(records[profile][i]['stamp']-c['source_stamp_ns']/1e9)) for c in selected]
            data[profile]=clouds(folder/'clouds.bin',set(indices[profile]))
        for ci,case in enumerate(selected):
            old=b0/case['case']; meta=json.loads((old/'metadata.json').read_text())
            frame={s:records[s][indices[s][ci]] for s in records}
            cloud={s:data[s][indices[s][ci]] for s in records}
            target=meta['raw_lidar_stamp_ns']/1e9
            if any(abs(c['stamp']-target)>.0001 for c in cloud.values()):raise ValueError('same raw scan not found')
            rgb=plt.imread(old/'source.jpg'); b=cloud['T3']['B']; tags=cloud['T3']['tags']
            colors=[palette[int(v)] for v in tags['label']]
            uv,valid=project_leveled(b,frame['T3']['roll'],frame['T3']['pitch'],p)
            visible=valid&(uv[:,0]>=0)&(uv[:,0]<rgb.shape[1])&(uv[:,1]>=0)&(uv[:,1]<rgb.shape[0])
            fig,ax=plt.subplots(2,3,figsize=(18,10))
            ax[0,0].imshow(rgb);ax[0,0].set_title('Original RGB; fixed B0 boxes; not GT')
            for box in (meta.get('yolo_observation') or {}).get('boxes',[]):
                x1,y1,x2,y2=box['xyxy'];ax[0,0].add_patch(Rectangle((x1,y1),x2-x1,y2-y1,fill=False,edgecolor='lime',linewidth=2))
            ax[0,1].imshow(rgb)
            ax[0,1].scatter(uv[visible,0],uv[visible,1],c=np.asarray(colors)[visible],s=2,alpha=.8)
            ax[0,1].set_title('T3 stage labels, inverse leveling + mount projection')
            for a in ax[0,:2]:a.set_axis_off()
            def xy(a,xyz,title,c):
                a.scatter(xyz[:,0],xyz[:,1],c=c,s=1);a.set_xlim(-30,40);a.set_ylim(-12,12)
                a.set_aspect('equal');a.set_xlabel('leveled sensor x (m)');a.set_ylabel('y (m)');a.set_title(title);a.grid(alpha=.2)
            xy(ax[0,2],cloud['T1']['A'],'T1 actual RANSAC (replica verified)', '#64afff')
            xy(ax[1,0],b,'T3: green kept / purple ceiling / orange ground / red wall',colors)
            xy(ax[1,1],cloud['T4']['B'],'T4: dynamic ROI OFF',[palette[int(v)] for v in cloud['T4']['tags']['label']])
            ax[1,2].scatter(b[:,0],b[:,2],c=colors,s=1);ax[1,2].set_xlim(-30,40);ax[1,2].set_ylim(-2.5,6)
            ax[1,2].set_xlabel('leveled sensor x (m)');ax[1,2].set_ylabel('z (m)');ax[1,2].set_title('T3 XZ: labels do not prove object identity');ax[1,2].grid(alpha=.2)
            fig.suptitle('%s | raw LiDAR ns %d | camera offset %.1f ms | diagnostic only'%(case['case'],meta['raw_lidar_stamp_ns'],meta['camera_lidar_offset_ms']))
            fig.text(.01,.01,'Stage points precede final OBB rejection. Evaluation roll/pitch rounded to 5 decimals. No independent GT; removed points are not free space.',fontsize=10)
            fig.tight_layout(rect=[0,.035,1,.96]);name=bag+'-case-%02d.png'%ci
            fig.savefig(args.output/name,dpi=110);plt.close(fig)
            results.append(dict(case=case['case'],image=name,raw_lidar_stamp_ns=meta['raw_lidar_stamp_ns'],
                raw_camera_stamp_ns=meta['raw_camera_stamp_ns'],camera_lidar_offset_ms=meta['camera_lidar_offset_ms'],
                frame_indices={k:indices[k][ci] for k in indices},stages={s:frame[s]['B'] for s in frame},
                projected_visible_points=int(visible.sum()),source_rgb_sha256=hashlib.sha256((old/'source.jpg').read_bytes()).hexdigest(),
                projection_is_diagnostic=True,independent_GT=False,source_pose_rounding_rad=1e-5,
                physical_bag_calibration_independently_verified=False))
    manifest=dict(cases=results,setting_sha256=hashlib.sha256(setting.read_bytes()).hexdigest(),
                  projection_matrix=p.tolist(),convention='K * optical_from_mount * inverse(vehicle_from_camera) * vehicle_from_lidar',
                  leveling_inverse='(Ry(pitch) Rx(roll)).T',real_GT_accuracy_claim=False)
    (args.output/'cases.json').write_text(json.dumps(manifest,indent=2)+'\n')
    (args.output/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>B1a same-scan diagnostics</title><h1>B1a same-scan diagnostics</h1><p>No independent object GT. Removed structures are not free space. Projection uses pinned calibration and rounded IMU angles.</p>'+''.join('<h2>'+r['case']+'</h2><img style="width:100%" src="'+r['image']+'">' for r in results))


if __name__=='__main__':main()
