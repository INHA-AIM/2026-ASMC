"""Exact family/IMU semantic trial gallery; no planner or motion replay.

Run after recording closes. This intentionally distinguishes known ego-mask
blindness from FOV absence and observed non-road/shadows. No cell is labeled
collision-free based on an empty LiDAR result.
"""
import argparse
from collections import Counter, OrderedDict
import html
import json
from pathlib import Path
from .analyze_shadow import completed_bags, TopicStats


def raster(probability, coverage):
    import numpy as np
    image=np.zeros((*probability.shape,3),np.uint8)
    image[coverage==0]=(80,60,40)       # BGR: unknown FOV
    image[coverage==128]=(160,70,170)  # known ego mask
    image[(coverage==255)&(probability>=166)]=(50,180,30)
    image[(coverage==255)&(probability<166)]=(20,30,220)
    return image


def analyze(root, output_name='semantic-analysis'):
    import cv2
    import numpy as np
    import rosbag
    root=Path(root)
    if Path(output_name).name!=output_name or output_name in ('.','..'):
        raise ValueError('new child output name required')
    bags=completed_bags(root,True)
    out=root/output_name;out.mkdir(exist_ok=False)
    prefix='/experimental/shadow/semantic_space_trial/'
    topics={prefix+'drivable_probability':'probability',prefix+'coverage':'coverage',
            prefix+'context':'context','/perception/lane/boundary_family':'family',
            '/image_jpeg/compressed':'rgb','/planning/camera_bbox_lane_change/diagnostics':'decision'}
    frames=OrderedDict();rgb=OrderedDict();pending=OrderedDict();seen=OrderedDict()
    counts=Counter();case_counts=Counter();cases=[];stats={}
    unmatched=Counter()

    def gallery(stamp):
        if stamp not in pending or stamp not in frames:return
        row=frames[stamp];v,reason=pending[stamp]
        if not all(k in row for k in ('probability','coverage','context','family')):return
        context=row['context'];headers=dict(zip(context.views,context.image_headers))
        if 'front' not in headers:return
        source=headers['front'].stamp.to_nsec()
        if source not in rgb:return
        key=(reason,v.get('semantic_target_lane'))
        pending.pop(stamp)
        if case_counts[key]>=2 or len(cases)>=16:return
        p,c=[np.frombuffer(row[k].data,np.uint8).reshape(row[k].height,row[k].step)[:,:row[k].width] for k in ('probability','coverage')]
        if p.shape!=(250,160) or c.shape!=p.shape:
            unmatched['grid_shape']+=1;return
        image=cv2.resize(raster(p,c),None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST)
        def point(x,y):return (int(round((8-y)*30)),int(round((20-x)*30)))
        for boundary in row['family'].boundaries:
            for i in range(1,len(boundary.polyline)):
                if i>=len(boundary.support_state) or not boundary.support_state[i-1] or not boundary.support_state[i]:continue
                a,b=boundary.polyline[i-1:i+1]
                cv2.line(image,point(a.x,a.y),point(b.x,b.y),(0,240,255) if boundary.role==2 else (255,255,255),1)
        cv2.rectangle(image,point(3.845,1.05),point(-1.03,-1.05),(255,255,255),2)
        if reason!='following' and v.get('semantic_first_blocked_x_m'):
            cv2.circle(image,point(float(v['semantic_first_blocked_x_m']),float(v['semantic_first_blocked_y_m'])),6,(0,240,255),2)
        stem='case-%02d'%len(cases);cv2.imwrite(str(out/(stem+'.png')),image)
        (out/(stem+'.jpg')).write_bytes(rgb[source].data)
        cases.append(dict(stamp_ns=stamp,front_source_stamp_ns=source,reason=reason,values=v,
                          grid=stem+'.png',rgb=stem+'.jpg'))
        case_counts[key]+=1

    for bag in bags:
        with rosbag.Bag(str(bag)) as stream:
            for topic,message,receipt in stream.read_messages(topics=list(topics)):
                kind=topics[topic];t=receipt.to_sec()
                stats.setdefault(kind,TopicStats()).observe(t,message.header.stamp.to_sec())
                if kind=='decision':
                    for status in message.status:
                        v={x.key:x.value for x in status.values}
                        if v.get('bev_mode')!='semantic_trial':continue
                        counts[status.message]+=1
                        stamp=int(v.get('semantic_input_stamp_ns','0'))
                        if not stamp:unmatched['no_decision_source']+=1;continue
                        key=(status.message,v.get('semantic_target_lane'))
                        if case_counts[key]>=2 or len(cases)>=16:continue
                        if stamp not in seen:pending[stamp]=(v,status.message);seen[stamp]=True
                        while len(seen)>128:seen.popitem(last=False)
                elif kind=='rgb':rgb[message.header.stamp.to_nsec()]=message
                else:frames.setdefault(message.header.stamp.to_nsec(),{})[kind]=message
                for stamp in list(pending):gallery(stamp)
                while len(frames)>32:frames.popitem(last=False)
                while len(rgb)>64:rgb.popitem(last=False)
                while len(pending)>32:pending.popitem(last=False);unmatched['expired_exact_pair']+=1
    unmatched['remaining_exact_pair']=len(pending)
    summary=dict(run_id=root.name,decision_samples=dict(counts),gallery_cases=cases,
                 unmatched=dict(unmatched),topics={k:v.summary() for k,v in stats.items()},
                 interpretation='Exact source/IMU context, not synchronized independent GT. Green is semantic road probability, not guaranteed free space. Purple is ego mask; brown is FOV unknown; red includes obstacle silhouettes/occlusion. Family lines are support-valid source geometry, not executed trajectories. Counts include preparation/cleanup.')
    (out/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
    cards=''.join('<article><h2>'+html.escape(c['reason'])+'</h2><p>source '+str(c['stamp_ns'])+'</p><img width="640" src="'+c['rgb']+'"><img width="320" src="'+c['grid']+'"><pre>'+html.escape(json.dumps(c['values'],ensure_ascii=False,indent=2))+'</pre></article>' for c in cases)
    (out/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>Semantic local path trial</title><style>body{background:#161b22;color:#eee;font:16px sans-serif;margin:24px}img{vertical-align:top;max-width:100%}pre{white-space:pre-wrap}article{border-top:1px solid #555}</style><h1>'+html.escape(root.name)+'</h1><p>초록=영상 노면 확률 · 빨강=비노면/가림 그림자 · 보라=차체 가림 · 갈색=시야 밖. 차선은 같은 시각 family, 실행 경로가 아닙니다. 빈 LiDAR는 공간 무장애 증명이 아닙니다.</p><a href="summary.json">수량·누락·출처</a>'+cards,encoding='utf-8')
    return summary


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('run_directory',type=Path)
    parser.add_argument('--output-name',default='semantic-analysis');args=parser.parse_args()
    result=analyze(args.run_directory,args.output_name)
    print(json.dumps({k:result[k] for k in ('run_id','decision_samples','unmatched')},ensure_ascii=False))

if __name__=='__main__':main()
