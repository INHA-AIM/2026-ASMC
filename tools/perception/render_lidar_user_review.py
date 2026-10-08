#!/usr/bin/env python3
"""Render actual recorded class differences and the unknown-point fixture.

Offline review only: no inference, ROS subscriptions, or vehicle commands.
Uses each observation's recorded leveling matrix, original JPEG headers and
actual filter-exported points. Images and correspondence are not object GT.
"""
import argparse
import bisect
import gzip
import hashlib
import json
from pathlib import Path

import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
import rosbag
import yaml

from tunnel_case_geometry import projection

PROFILES=('common_pitch','common_roll_pitch')
COLORS=('#1376bd','#e2761b')
EDGES=((0,1),(1,2),(2,3),(3,0),(4,5),(5,6),(6,7),(7,4),(0,4),(1,5),(2,6),(3,7))


def rows(path):
    with path.open() as f:return [json.loads(line) for line in f]


def quantiles(values):
    if not len(values):return None
    return dict(zip(('mean','p50','p95','p99','max'),
                    map(float,[np.mean(values),*np.percentile(values,[50,95,99]),max(values)])))


def ground_figure(root,out):
    old=json.loads((root/'legacy.json').read_text());new=json.loads((root/'common.json').read_text())
    assert old['input']==new['input'] and len(old['output'])==0 and new['output']==new['input']
    p=np.array(new['input']);g=np.array(new['initial_ground'])
    fig=plt.figure(figsize=(14,9),layout='constrained')
    for k,(name,data,color) in enumerate((('Previous-plane fallback',old,'#ca3030'),
                                          ('Current-fit failure: preserve unknown',new,'#25834a'))):
        ax=fig.add_subplot(2,2,k+1,projection='3d')
        ax.scatter(*g.T,s=12,c='#aaaaaa',label='Previous scan: fitted ground (64)')
        ax.scatter(*p.T,s=35,c=color,marker='x' if k==0 else 'o',
                   label='24 discarded inputs' if k==0 else '24 retained inputs')
        x,y=np.meshgrid(np.linspace(4.7,8.7,9),np.linspace(-1.2,1.4,7))
        ax.plot_surface(x,y,np.full_like(x,-1.5),alpha=.12,color='#55799b')
        ax.set(xlabel='Filter-frame X (m)',ylabel='Filter-frame Y (m)',zlabel='Z (m)',
               zlim=(-1.9,-1.1),title=f'{name}\nOutput: {len(data["output"])} / 24 points')
        ax.view_init(24,-55);ax.legend(loc='upper left',fontsize=8)
        ax=fig.add_subplot(2,2,k+3)
        ax.axhspan(-1.8,-1.2,alpha=.12,color='#55799b',label='Old plane deletion band: +/-0.30 m')
        ax.axhline(-1.5,color='#55799b',linestyle='--',label='Old fitted plane Z=-1.50 m')
        ax.scatter(p[:,0],p[:,2],s=45,c=color,marker='x' if k==0 else 'o')
        ax.set(xlim=(7.95,8.51),ylim=(-1.85,-1.15),xlabel='Filter-frame X (m)',ylabel='Z (m)',
               title='Same 24 input points; output count shown above')
        ax.legend(loc='upper right',fontsize=8);ax.grid(alpha=.3)
    fig.suptitle('Synthetic unknown points, NOT a recorded obstacle\n'
                 'Current fit fails: 24 collinear seeds < minimum 30; prior plane exists',fontsize=15)
    fig.supxlabel('Roll context changes 0 -> 10 degrees; points already in filter coordinates.\n'
                  'No raw-scan rotation here. These may be ground: no independent object truth.',fontsize=11)
    fig.savefig(out/'ground-24.png',dpi=150);plt.close(fig)
    (out/'ground-data.json').write_text(json.dumps(dict(legacy=old,common=new),indent=2)+'\n')


def transforms_and_clouds(root,selected,exact):
    result={}
    for profile in PROFILES:
        contexts={};clouds={}
        with gzip.open(root/f'primary-{profile}-normal-r1.trace.jsonl.gz','rt') as f:
            for line in f:
                if '"event":"lidar_coordinate_context"' in line[:100]:
                    r=json.loads(line)
                    if r['stamp_ns'] in selected:
                        contexts[r['stamp_ns']]=r
                elif '"event":"ground_points"' in line[:100]:
                    # Avoid deserializing unselected megabyte point records.
                    if any(str(ns) in line[:160] for ns in exact):
                        r=json.loads(line);clouds[r['stamp_ns']]=r
        if set(contexts)!=set(selected) or set(clouds)!=set(exact):
            raise ValueError('missing observation context/point data')
        for ns,c in contexts.items():
            level=np.array(c['leveling']).reshape(3,3)
            if not np.allclose(level.T@level,np.eye(3),atol=1e-6):raise ValueError('invalid recorded rotation')
            if ns in clouds:
                pts=np.array(clouds[ns]['points'])
                if not np.allclose(pts[:,1:4]@level.T,pts[:,4:7],atol=1e-4):
                    raise ValueError('recorded leveling does not match actual point correspondence')
        result[profile]=dict(contexts=contexts,clouds=clouds)
        print('Loaded actual point/context records:',profile,flush=True)
    return result


def raw_geometry(obj,context):
    level=np.array(context['leveling']).reshape(3,3)
    # Row vectors: inverse of level is multiplication by level, exactly once.
    return np.array(obj['corners'])@level,np.array(obj['center'])@level


def pixels(raw,p):
    q=np.column_stack([raw,np.ones(len(raw))])@p.T
    if np.any(q[:,2]<=1e-3):return None
    return q[:,:2]/q[:,2,None]


def draw_box(ax,obj,context,p,color):
    corners,center=raw_geometry(obj,context);uv=pixels(corners,p)
    if uv is None:return None
    for a,b in EDGES:ax.plot(uv[[a,b],0],uv[[a,b],1],c=color,lw=2)
    return uv


def case_figure(case,index,states,data,camera,p,out,bag_start):
    ns=case['stamp_ns'];camera_ns,jpeg=camera
    bgr=cv2.imdecode(np.frombuffer(jpeg,dtype=np.uint8),cv2.IMREAD_COLOR)
    if bgr is None or bgr.shape[:2]!=(720,1280):raise ValueError('unexpected original JPEG shape')
    rgb=cv2.cvtColor(bgr,cv2.COLOR_BGR2RGB)
    objects=[next(o for o in states[profile][ns]['objects'] if o['id']==case[key])
             for profile,key in zip(PROFILES,('a_id','b_id'))]
    geometry=[raw_geometry(obj,data[profile]['contexts'][ns]) for profile,obj in zip(PROFILES,objects)]
    uv=[pixels(g[0],p) for g in geometry]
    valid=[v for v in uv if v is not None]
    if not valid:raise ValueError('target is not projectable')
    alluv=np.vstack(valid)
    visible_in_front=bool(np.all(alluv.max(0)>[0,0]) and np.all(alluv.min(0)<[1280,720]))
    lo=alluv.min(0)-[90,70];hi=alluv.max(0)+[90,70]
    if visible_in_front:
        lo=np.maximum(lo,[0,0]);hi=np.minimum(hi,[1279,719])
    fig=plt.figure(figsize=(16,13),layout='constrained');gs=fig.add_gridspec(3,2,height_ratios=(1,.8,1))
    for k,(profile,obj,color) in enumerate(zip(PROFILES,objects,COLORS)):
        ctx=data[profile]['contexts'][ns]
        for row in (0,1):
            ax=fig.add_subplot(gs[row,k]);ax.imshow(rgb)
            draw_box(ax,obj,ctx,p,color)
            ax.set_xlim((0,1280) if row==0 else (lo[0],hi[0]));ax.set_ylim((720,0) if row==0 else (hi[1],lo[1]))
            if not visible_in_front:
                ax.set_facecolor('#eeeeee')
                ax.text(.5,.1,'Target is outside the recorded FRONT image',transform=ax.transAxes,
                        ha='center',color='#a00000',bbox=dict(facecolor='white',alpha=.9))
            ax.set_title((f'{profile}: ID {obj["id"]}, class={obj["class"]}\n'
                          f'candidate={obj["candidate"]}, confirmations={obj["confirmations"]}, '
                          f'class misses={obj["class_misses"]}') if row==0 else
                         ('Same original JPEG / same crop' if visible_in_front else 'Projected OBB outside image; no visible object GT'))
            ax.axis('off')
        ax=fig.add_subplot(gs[2,k]);pts=np.array(data[profile]['clouds'][ns]['points'])
        raw_center=geometry[k][1];visible=(np.linalg.norm(pts[:,1:3]-raw_center[:2],axis=1)<5)
        cloud=pts[visible];base=cloud[:,1:4]+[.8,0,1.45];kept=cloud[:,7]>0
        ax.scatter(base[~kept,1],base[~kept,0],s=2,c='#bbbbbb',label='Ground-filter removed')
        ax.scatter(base[kept,1],base[kept,0],s=3,c='#404040',label='Retained voxel centroids')
        corners=geometry[k][0]+[.8,0,1.45]
        for a,b in EDGES:ax.plot(corners[[a,b],1],corners[[a,b],0],c=color,lw=2)
        c=raw_center+[.8,0,1.45]
        ax.set(xlim=(c[1]+4,c[1]-4),ylim=(c[0]-4,c[0]+4),aspect='equal',
               xlabel='base_link Y (left+, m)',ylabel='base_link X (forward, m)',title='Same LiDAR observation, actual 3D OBB')
        ax.legend(fontsize=8,loc='upper right');ax.grid(alpha=.25)
    seconds=(ns-bag_start)/1e9
    fig.suptitle(f'Class difference {index:02d} / 11 | primary bag t={seconds:.3f}s\n'
                 f'LiDAR source ns: {ns} | RGB source offset: {(camera_ns-ns)/1e6:+.2f} ms',fontsize=15)
    fig.supxlabel('Recorded leveling inverse -> raw sensor -> calibrated camera; no YOLO recomputation.\n'
                  'Class shown is post-fusion state handed to publishing. Spatial pairing is diagnostic, NOT GT.',fontsize=11)
    name=f'class-{index:02d}.png';fig.savefig(out/name,dpi=125);plt.close(fig)
    cv2.imwrite(str(out/f'rgb-{index:02d}.jpg'),bgr)
    return dict(case,index=index,image=name,original_image=f'rgb-{index:02d}.jpg',
                time_sec=seconds,camera_stamp_ns=camera_ns,camera_offset_ms=(camera_ns-ns)/1e6,
                target_visible_in_front_image=visible_in_front,
                profiles={profile:dict(object=obj,context=data[profile]['contexts'][ns])
                          for profile,obj in zip(PROFILES,objects)})


def context_gif(episode,indices,states,data,cameras,p,out,start):
    frames=[]
    for ns in indices:
        _,jpeg=cameras[ns];image=cv2.imdecode(np.frombuffer(jpeg,np.uint8),cv2.IMREAD_COLOR)
        panels=[]
        for profile,color in zip(PROFILES,((210,105,20),(20,120,230))):
            panel=image.copy()
            for obj in states[profile][ns]['objects']:
                if obj['lidar_misses']!=0:continue
                corners,_=raw_geometry(obj,data[profile]['contexts'][ns]);uv=pixels(corners,p)
                if uv is None:continue
                # Offscreen points never become huge/clipped integer coordinates.
                if np.max(np.abs(uv))>100000:continue
                uv=np.rint(uv).astype(int)
                for a,b in EDGES:
                    cv2.line(panel,tuple(uv[a]),tuple(uv[b]),color,2)
                x,y=uv.min(axis=0)
                if 0<=x<1280 and 25<=y<720:
                    cv2.putText(panel,f'{obj["id"]} {obj["class"]}',(x,y-5),cv2.FONT_HERSHEY_SIMPLEX,.42,color,1)
            panel=cv2.resize(panel,(800,450));header=np.full((40,800,3),245,np.uint8)
            cv2.putText(header,f'{profile}   t={(ns-start)/1e9:.3f}s',(10,26),cv2.FONT_HERSHEY_SIMPLEX,.65,(0,0,0),1)
            panels.append(np.vstack([header,panel]))
        frames.append(Image.fromarray(cv2.cvtColor(np.hstack(panels),cv2.COLOR_BGR2RGB)))
    name=f'context-{episode:02d}.gif'
    durations=[max(20,int((b-a)/1e6)) for a,b in zip(indices,indices[1:])]+[800]
    frames[0].save(out/name,save_all=True,append_images=frames[1:],duration=durations,loop=0)
    return dict(image=name,stamps_ns=indices,scope='all currently measured OBBs; numeric IDs are local to each run')


def side_case(case,side_camera,vehicle,camera_config,out):
    camera_ns,jpeg=side_camera;bgr=cv2.imdecode(np.frombuffer(jpeg,np.uint8),cv2.IMREAD_COLOR)
    camera=camera_config['cameras']['right']
    width,height=camera['width'],camera['height']
    if bgr is None or bgr.shape[:2]!=(height,width):raise ValueError('side snapshot / image resolution mismatch')
    camera_vehicle=dict(vehicle)
    sensor=dict(zip(('x','y','z'),camera['translation_m']))
    sensor.update(zip(('roll','pitch','yaw'),camera['rotation_deg']))
    f=width/(2*np.tan(np.deg2rad(camera['horizontal_fov_deg'])/2))
    sensor.update(fx=f,fy=f,cx=width/2,cy=height/2);camera_vehicle['Camera']=sensor
    p=projection(camera_vehicle);rgb=cv2.cvtColor(bgr,cv2.COLOR_BGR2RGB)
    fig,axes=plt.subplots(1,2,figsize=(14,6),layout='constrained')
    for ax,profile,color in zip(axes,PROFILES,COLORS):
        sample=case['profiles'][profile];ax.imshow(rgb)
        draw_box(ax,sample['object'],sample['context'],p,color)
        ax.set(xlim=(0,width),ylim=(height,0),title=f'{profile}: {sample["object"]["class"]}');ax.axis('off')
    fig.suptitle(f'Class difference {case["index"]:02d}: original RIGHT camera\n'
                 f'RGB source offset: {(camera_ns-case["stamp_ns"])/1e6:+.2f} ms',fontsize=14)
    fig.supxlabel('Same 3D observation projected with pinned right-camera pose/FOV.\n'
                  'Front-only YOLO classes can persist after an object moves outside the front view.',fontsize=11)
    name=f'class-{case["index"]:02d}-right.png';fig.savefig(out/name,dpi=125);plt.close(fig)
    case.update(side_image=name,side_stamp_ns=camera_ns,side_topic='/cam_right/image_jpeg/compressed')
    cv2.imwrite(str(out/f'rgb-{case["index"]:02d}-right.jpg'),bgr)


def speed_report(analysis,out):
    report=dict(metric='same-track successive CA relative velocity vector delta >2 m/s; NOT physical speed/error',
                threshold_mps=2,independent_speed_GT=False,bags={})
    fig,axes=plt.subplots(2,2,figsize=(14,10),layout='constrained')
    for row,bag in enumerate(('primary','shadow')):
        profiles=analysis['bags'][bag]['profiles'];report['bags'][bag]={}
        allstamps=[e['stamp_ns'] for x in profiles.values() for e in x['speed_jumps']]
        anchor=min(allstamps)
        for name,color in zip(('legacy',)+PROFILES,('#808080',)+COLORS):
            events=profiles[name]['speed_jumps'];v=np.array([e['jump_mps'] for e in events])
            report['bags'][bag][name]=dict(count=len(v),jump_mps=quantiles(v),
                distinct_ids=len({e['id'] for e in events}),
                initialization_transition_candidates=sum(e['cause']=='initialization transition candidate' for e in events),
                observed_age_at_least_5_candidates=sum(e['age']>=5 and e['misses']==0 for e in events),
                largest=sorted(events,key=lambda e:e['jump_mps'],reverse=True)[:5])
            axes[row,0].plot(np.sort(v),np.arange(1,len(v)+1)/len(v),label=f'{name} ({len(v)})',c=color)
            if name!='legacy':
                axes[row,1].scatter([(e['stamp_ns']-anchor)/1e9 for e in events],v,s=5,alpha=.5,c=color,label=name)
        axes[row,0].set(title=bag+' | distribution of counted jumps',xlabel='CA velocity vector change (m/s)',ylabel='Cumulative fraction')
        axes[row,1].set(title=bag+' | every counted event',xlabel='Seconds after first counted event',ylabel='CA velocity vector change (m/s)')
        for ax in axes[row]:ax.grid(alpha=.3);ax.legend(fontsize=8)
    fig.suptitle('Internal speed-estimate instability: counts and magnitude\n'
                 'Different clusters, initialization, geometry and real motion can all contribute; no bag speed GT',fontsize=14)
    fig.savefig(out/'speed-jumps.png',dpi=150);plt.close(fig)
    (out/'speed-summary.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


def gallery(cases,contexts,speed,out):
    trs=[]
    for bag in ('primary','shadow'):
        for name in ('legacy',)+PROFILES:
            r=speed['bags'][bag][name];q=r['jump_mps']
            trs.append(f'<tr><td>{bag}</td><td>{name}</td><td>{r["count"]}</td><td>{q["p50"]:.2f}</td><td>{q["p95"]:.2f}</td><td>{q["max"]:.2f}</td></tr>')
    cards=[]
    for case in cases:
        i=case['index'];p=case['profiles'];a=p[PROFILES[0]]['object'];b=p[PROFILES[1]]['object']
        cards.append(f'<option value="{i-1}">{i:02d}: {case["time_sec"]:.3f}s · {a["class"]} → {b["class"]}</option>')
    context_html=''.join(f'<details><summary>구간 {i+1} 전후 연속 영상 — 펼치면 재생</summary><img loading="lazy" src="{c["image"]}"><p>같은 영상에 각 실행의 현재 실측 OBB 표시. 숫자 ID는 실행 간 비교 기준이 아닙니다.</p></details>' for i,c in enumerate(contexts))
    content='''<!doctype html><html lang="ko"><meta charset="utf-8"><title>PR72 LiDAR 비교 검토</title>
<style>body{font-family:sans-serif;margin:24px auto;max-width:1600px;background:#f6f7f9;color:#17212c}section{background:white;padding:24px;margin:18px 0;border-radius:10px}img{width:100%;height:auto}button,select{font-size:18px;padding:10px;margin:6px}table{border-collapse:collapse}th,td{padding:9px;border:1px solid #bbb}.note{background:#fff3cd;padding:16px}a{color:#0066a6}summary{cursor:pointer;font-size:20px;padding:15px}</style>
<h1>PR #72 — 직접 검토할 비교 자료</h1><p>운영 설정 변경·새 추론·차량 제어 없이 기존 증거를 렌더링했습니다. 공통 경계·roll 기본값은 OFF입니다.</p>
<section><h2>1. 지면 추정 실패 시 24점: 이전 평면으로 삭제 / 불명확해서 보존</h2>
<p class="note">실제 bag 장애물 점이 아닌 <b>합성 선형 점 24개</b>입니다. 이전 지면 높이 Z=-1.5m 위에 놓여 있습니다. 현재 입력은 30점 미만이고 일직선이어서 새 평면을 추정할 수 없습니다. 24점은 지면일 수도 있으며 장애물 정답은 없습니다. 보존을 객체 검출 개선으로 판단하면 안 됩니다.</p>
<p>회전 0→10°는 context 값입니다. 이 시험은 이미 필터 좌표에 있는 점을 넣으며, 실제 raw scan을 회전한 시험은 아닙니다. 실제 PCL 지면 함수의 결과는 이전 fallback 0점 / common 보존 24점입니다.</p>
<img src="ground-24.png"><p><a href="ground-data.json">정확한 입력·출력 점과 평면 데이터</a></p></section>
<section><h2>2. 신규 분류 차이 11개 관측</h2><p>주 bag의 동일 LiDAR source stamp를 비교합니다. 10개는 car→unknown, 1개는 unknown→car이며 동일 객체의 연속 관측이 포함됩니다. 대상 박스는 두 실행에서 같은 원본 영상에 투영했습니다. 아래 확대 및 base_link 평면도도 함께 비교하세요.</p>
<p class="note">분류 차이는 객체 삭제와 다릅니다. 같은 관측에 실측 3D 박스가 있어도 확인 카운터·이력 때문에 unknown일 수 있습니다. 실제 차량 여부는 원본 영상으로 검토해야 합니다. 상호 유일 최근접 0.25m 대응은 독립 GT가 아닙니다.</p>
<button onclick="change(-1)">이전</button><select id="pick" onchange="show(this.selectedIndex)">OPTIONS</select><button onclick="change(1)">다음</button>
<p id="meta"></p><a id="full" target="_blank"><img id="case"></a><img id="side" hidden><p><a id="rgb" target="_blank">원본 JPEG만 보기</a> · <a href="class-review.json">11건 source stamp·행렬·OBB·카운터</a></p>
CONTEXTS</section><section><h2>3. 속도 튐: 횟수와 변화량</h2>
<p>같은 추적 ID의 내부 CA 상대속도 벡터가 직전 상태보다 <b>2m/s 초과</b> 변한 이벤트입니다. 실제 속도·가속도 또는 속도 오차가 아닙니다. 초기화·군집 중심 변화·오연결·자세 변화·실제 운동이 섞일 수 있습니다.</p>
<table><tr><th>bag</th><th>구성</th><th>횟수</th><th>튀는 변화량 p50(m/s)</th><th>p95(m/s)</th><th>최대(m/s)</th></tr>ROWS</table>
<p>roll 추가만 분리하면 주 bag은 657→599(-8.8%), 음영은 527→555(+5.3%)입니다. 사용자가 음영의 증가를 이 지표에 한해 허용했습니다. 전체 객체 보존·신규 분류 차이까지 자동 승인한 것은 아닙니다.</p>
<img src="speed-jumps.png"><p><a href="speed-summary.json">전체 통계 및 최대 변화 5건의 stamp·ID·중심 이동</a></p></section>
<script>const cases=DATA;let at=0;function show(i){at=i;document.getElementById('pick').selectedIndex=i;let c=cases[i];document.getElementById('case').src=c.image;document.getElementById('full').href=c.image;document.getElementById('rgb').href=c.original_image;document.getElementById('meta').textContent=`${i+1}/11 | LiDAR ${c.stamp_ns} | RGB ${c.camera_stamp_ns} | RGB offset ${c.camera_offset_ms.toFixed(2)}ms`;const side=document.getElementById('side');side.hidden=!c.side_image;if(c.side_image)side.src=c.side_image; }function change(d){show((at+d+cases.length)%cases.length)}show(0);</script></html>'''
    # JavaScript Number cannot preserve 19-digit ROS nanoseconds. Display strings.
    browser_cases=[]
    for case in cases:
        item={key:(str(case[key]) if key.endswith('_ns') else case[key])
              for key in ('image','original_image','stamp_ns','camera_stamp_ns','camera_offset_ms')}
        if 'side_image' in case:item['side_image']=case['side_image']
        browser_cases.append(item)
    content=content.replace('OPTIONS',''.join(cards)).replace('CONTEXTS',context_html).replace('ROWS',''.join(trs)).replace('DATA',json.dumps(browser_cases))
    (out/'index.html').write_text(content)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,required=True);parser.add_argument('--analysis',type=Path,required=True)
    parser.add_argument('--ground',type=Path,required=True);parser.add_argument('--bag',type=Path,required=True)
    parser.add_argument('--calibration',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--original-bag',type=Path,required=True)
    parser.add_argument('--camera-config',type=Path,required=True)
    args=parser.parse_args();out=args.output;out.mkdir(parents=True,exist_ok=False)
    analysis=json.loads(args.analysis.read_text())
    comparison=next(x for x in analysis['bags']['primary']['comparisons'] if x['reference']=='common_pitch')
    cases=comparison['class_differences'];assert len(cases)==11
    states={profile:{r['stamp_ns']:r for r in rows(args.root/f'primary-{profile}-normal-r1.jsonl')} for profile in PROFILES}
    exact={c['stamp_ns'] for c in cases};all_ns=sorted(set(states[PROFILES[0]])&set(states[PROFILES[1]]))
    groups=[]
    for ns in sorted(exact):
        if not groups or ns-groups[-1][-1]>1_000_000_000:groups.append([ns])
        else:groups[-1].append(ns)
    context_indices=[[ns for ns in all_ns if group[0]-1_000_000_000<=ns<=group[-1]+1_000_000_000][::3] for group in groups]
    selected=exact|{ns for group in context_indices for ns in group}
    data=transforms_and_clouds(args.root,selected,exact)
    p=projection(yaml.safe_load(args.calibration.read_text())['Vehicle'])
    images=[]
    with rosbag.Bag(str(args.bag)) as bag:
        start=int(round(bag.get_start_time()*1e9))
        for _,message,_ in bag.read_messages(topics=['/camera/image/compressed']):
            ns=message.header.stamp.to_nsec()
            if min(selected)-100_000_000<=ns<=max(selected)+100_000_000:images.append((ns,bytes(message.data)))
    images.sort(key=lambda x:x[0]);camera_stamps=[x[0] for x in images];cameras={}
    for ns in selected:
        at=bisect.bisect_left(camera_stamps,ns)
        candidates=[images[i] for i in (at-1,at) if 0<=i<len(images)]
        cameras[ns]=min(candidates,key=lambda x:abs(x[0]-ns))
        if abs(cameras[ns][0]-ns)>50_000_000:raise ValueError('original JPEG outside 50ms review bound')
    ground_figure(args.ground,out);speed=speed_report(analysis,out)
    rendered=[]
    for index,case in enumerate(cases,1):
        rendered.append(case_figure(case,index,states,data,cameras[case['stamp_ns']],p,out,start))
        print('Rendered exact class difference',index,flush=True)
    outside=[case for case in rendered if not case['target_visible_in_front_image']]
    side_images=[]
    with rosbag.Bag(str(args.original_bag)) as bag:
        for _,message,_ in bag.read_messages(topics=['/cam_right/image_jpeg/compressed']):
            ns=message.header.stamp.to_nsec()
            if any(abs(ns-case['stamp_ns'])<=100_000_000 for case in outside):
                side_images.append((ns,bytes(message.data)))
    vehicle=yaml.safe_load(args.calibration.read_text())['Vehicle']
    camera_config=yaml.safe_load(args.camera_config.read_text())
    front=camera_config['cameras']['front']
    if (front['translation_m']!=[vehicle['Camera'][key] for key in ('x','y','z')] or
        front['rotation_deg']!=[vehicle['Camera'][key] for key in ('roll','pitch','yaw')]):
        raise ValueError('camera snapshot is not consistent with Main front calibration')
    for case in outside:
        nearest=min(side_images,key=lambda x:abs(x[0]-case['stamp_ns']))
        if abs(nearest[0]-case['stamp_ns'])>50_000_000:raise ValueError('side JPEG outside review time bound')
        side_case(case,nearest,vehicle,camera_config,out)
    contexts=[context_gif(i,indices,states,data,cameras,p,out,start) for i,indices in enumerate(context_indices,1)]
    (out/'class-review.json').write_text(json.dumps(dict(cases=rendered,contexts=contexts,
        independent_object_GT=False,projection='recorded inverse leveling -> raw sensor -> calibrated camera',
        original_image_topic='/camera/image/compressed',bag=str(args.bag)),indent=2)+'\n')
    gallery(rendered,contexts,speed,out)
    manifest={str(file.relative_to(out)):hashlib.sha256(file.read_bytes()).hexdigest() for file in out.iterdir() if file.is_file()}
    manifest['source:'+str(args.calibration)]=hashlib.sha256(args.calibration.read_bytes()).hexdigest()
    manifest['source:'+str(args.analysis)]=hashlib.sha256(args.analysis.read_bytes()).hexdigest()
    manifest['source:'+str(args.camera_config)]=hashlib.sha256(args.camera_config.read_bytes()).hexdigest()
    for tool in ('render_lidar_user_review.py','export_ground_failure_review.py','export_ground_failure_review.cpp','tunnel_case_geometry.py'):
        path=Path(__file__).parent/tool
        manifest['source:'+str(path)]=hashlib.sha256(path.read_bytes()).hexdigest()
    (out/'sha256.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print('Review gallery complete:',out,flush=True)


if __name__=='__main__':main()
