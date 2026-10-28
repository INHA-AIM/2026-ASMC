"""Source-exact LOCAL-1 semantic/RGB/path gallery, read-only bag analysis."""
import argparse
from collections import Counter
import html
import json
import math
from pathlib import Path


def gallery(root, name='local-gallery'):
    import cv2
    import numpy as np
    import rosbag
    from .analyze_semantic import raster
    root=Path(root); out=root/name; out.mkdir(exist_ok=False)
    selected={}; seen=Counter()
    for n,line in enumerate((root/'decisions.jsonl').open()):
        row=json.loads(line); stamp=row['scene_stamp_ns']; reason=row['reason']
        if stamp and (seen[reason]==0 or n%40==0) and len(selected)<24:
            selected[stamp]=row
        seen[reason]+=1
    prefix='/experimental/shadow/semantic_space_trial/'
    topics={prefix+'context':'context',prefix+'coverage':'coverage',prefix+'drivable_probability':'probability'}
    frames={s:{} for s in selected};rgb={};needed=set()
    bags=sorted(root.glob('sensors*.bag'))
    for path in bags:
        with rosbag.Bag(str(path)) as bag:
            for topic,m,_ in bag.read_messages(topics=list(topics)):
                stamp=m.header.stamp.to_nsec()
                if stamp not in frames:continue
                frames[stamp][topics[topic]]=m
                if topics[topic]=='context':
                    i=list(m.views).index('front');needed.add(m.image_headers[i].stamp.to_nsec())
    for path in bags:
        with rosbag.Bag(str(path)) as bag:
            for _,m,_ in bag.read_messages(topics=['/image_jpeg/compressed']):
                stamp=m.header.stamp.to_nsec()
                if stamp in needed:rgb[stamp]=m.data
    cases=[];missing=Counter()
    for stamp,row in selected.items():
        f=frames[stamp]
        if set(f)!=set(topics.values()):missing['incomplete_exact_bundle']+=1;continue
        context=f['context'];j=list(context.views).index('front')
        source=context.image_headers[j].stamp.to_nsec()
        if source not in rgb:missing['missing_exact_rgb']+=1;continue
        p,c=[np.frombuffer(f[k].data,np.uint8).reshape(f[k].height,f[k].step)[:,:f[k].width] for k in ('probability','coverage')]
        image=cv2.resize(raster(p,c),None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST)
        debug=row.get('scene',{})
        if 'rotation' in debug:
            R=np.asarray(debug['rotation']).reshape(3,3);z=debug['ground_z']
        else:
            # Historical trace lacked native R. This display-only inverse is
            # reconstructed from that exact context, never used by planning.
            a,b,offset=context.plane_coefficients[3*j:3*j+3]
            roll=math.atan2(-b,1);pitch=math.atan2(a,math.sqrt(1+b*b))
            cr,sr,cp,sp=math.cos(roll),math.sin(roll),math.cos(pitch),math.sin(pitch)
            R=np.array([[cp,sp*sr,sp*cr],[0,cr,-sr],[-sp,cp*sr,cp*cr]])
            z=offset/math.sqrt(1+a*a+b*b)
        def pt(x,y):
            dx,dy,angle=debug.get('source_in_plan',[0,0,0])
            c,s=math.cos(angle),math.sin(angle);x,y=x-dx,y-dy
            bx,by,_=R.T@np.array([c*x+s*y,-s*x+c*y,z]);return int(round((8-by)*30)),int(round((20-bx)*30))
        v=row.get('vehicle',dict(front=3.845,rear=.79,width=2.160835))
        for x,y,yaw in row['path'][::5]:
            cs,sn=math.cos(yaw),math.sin(yaw)
            poly=[pt(x+cs*a-sn*b,y+sn*a+cs*b) for a,b in [(-v['rear'],-v['width']/2),(v['front'],-v['width']/2),(v['front'],v['width']/2),(-v['rear'],v['width']/2)]]
            cv2.polylines(image,[np.asarray(poly,np.int32)],True,(220,190,60),1)
        if row['path']:cv2.polylines(image,[np.asarray([pt(q[0],q[1]) for q in row['path']],np.int32)],False,(255,255,0),2)
        for ob in debug.get('obstacles',[]):
            cv2.polylines(image,[np.asarray([pt(*q) for q in ob['polygon']],np.int32)],True,(0,100,255),2)
        stem='case-%02d'%len(cases);cv2.imwrite(str(out/(stem+'.png')),image)
        (out/(stem+'.jpg')).write_bytes(rgb[source])
        (out/(stem+'.json')).write_text(json.dumps(row,indent=2))
        cases.append(dict(stem=stem,scene_stamp_ns=stamp,front_stamp_ns=source,reason=row['reason'],recovery=row.get('recovery',False),plan_sequence=row['plan_sequence']))
    summary=dict(cases=cases,missing=dict(missing),interpretation='Exact RGB/semantic stamp; projected path is proposed, not proof of executed path or collision clearance.')
    (out/'index.json').write_text(json.dumps(summary,indent=2))
    cards=''.join('<article><h2>'+html.escape(c['reason'])+' · plan '+str(c['plan_sequence'])+'</h2><img width="640" src="'+c['stem']+'.jpg"><img width="360" src="'+c['stem']+'.png"><p><a href="'+c['stem']+'.json">판단 원문</a></p></article>' for c in cases)
    (out/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>LOCAL-1</title><style>body{background:#141a22;color:#eee;font:16px sans-serif;margin:24px}img{vertical-align:top;max-width:100%}article{border-top:1px solid #555;margin-top:24px}</style><h1>'+html.escape(root.name)+'</h1><p>청록 경로=제안 경로, 노란 외곽=차체 sweep. 초록=노면 확률, 빨강=비노면/가림 그림자, 보라=차체 가림, 갈색=시야 밖. 회복 경로는 충돌 무결성이 검증된 경로가 아닙니다. 실제 진행·접촉은 events/trial-result와 대조합니다.</p><a href="index.json">시각 대응·누락</a>'+cards,encoding='utf-8')
    print(json.dumps(dict(cases=len(cases),missing=dict(missing))))

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('root');p.add_argument('--name',default='local-gallery');a=p.parse_args();gallery(a.root,a.name)
