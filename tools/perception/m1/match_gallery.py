#!/usr/bin/env python3
"""Render exact native matching rectangles on recorded RGB; no projection math."""
import argparse,collections,html,json
from pathlib import Path
import cv2
import numpy as np
import rosbag


def main(a):
    if json.loads((a.run/'registration.json').read_text())['run_id']!=a.run_id:raise ValueError('run ID mismatch')
    out=a.run/a.analysis_id;out.mkdir(exist_ok=False)
    topics={'camera_front':'/image_jpeg/compressed','camera_left':'/cam_left/image_jpeg/compressed','camera_right':'/cam_right/image_jpeg/compressed'}
    cards=[];wanted=collections.defaultdict(list);last={};counts=collections.Counter()
    for line in a.trace.open():
        row=json.loads(line)
        for d in row.get('fusion',{}).get('decisions',[]):
            view=d['camera_id']
            if d['action']!='match' or not d['associations'] or counts[view]>=12:continue
            if d['scan_stamp_ns']-last.get(view,0)<4_000_000_000:continue
            key=(topics[view],d['image_stamp_ns'],d['wire_sequence'])
            wanted[key].append(len(cards));cards.append(dict(emitting_seq=row['seq'],decision=d))
            last[view]=d['scan_stamp_ns'];counts[view]+=1
    found=collections.Counter()
    with rosbag.Bag(str(a.bag)) as bag:
        for topic,m,_ in bag.read_messages(topics=list(topics.values())):
            key=(topic,m.header.stamp.to_nsec(),m.header.seq)
            if key not in wanted:continue
            found[key]+=1
            for i in wanted[key]:
                cards[i]['rgb_matches']=found[key]
                if found[key]!=1:cards[i].pop('image',None);continue
                im=cv2.imdecode(np.frombuffer(bytes(m.data),np.uint8),cv2.IMREAD_COLOR)
                d=cards[i]['decision'];expected=(1280,720) if d['camera_id']=='camera_front' else (640,480)
                if (im.shape[1],im.shape[0])!=expected:raise ValueError('original RGB size mismatch')
                for match in d['associations']:
                    x1,y1,x2,y2=map(round,match['camera_box']);cv2.rectangle(im,(x1,y1),(x2,y2),(0,190,255),2)
                    x1,y1,x2,y2=map(round,match['projected_box']);cv2.rectangle(im,(x1,y1),(x2,y2),(0,255,0),1)
                    cv2.putText(im,'#%s %s %.2f'%(match['track_id'],match['raw_class'],match['iou']),
                        (max(0,x1),max(14,y1)),cv2.FONT_HERSHEY_SIMPLEX,.45,(0,255,0),1)
                name='match-%02d.jpg'%i;cv2.imwrite(str(out/name),im);cards[i]['image']=name
    (out/'matches.json').write_text(json.dumps(cards,indent=2))
    parts=['<!doctype html><meta charset="utf-8"><style>body{background:#14202b;color:white;font:16px sans-serif}img{max-width:95vw}section{margin:2em}</style><h1>M1 실제 3뷰 matching</h1><p>주황=YOLO 원본 bbox, 초록=동일 scan context의 native 투영. 정확한 header·wire sequence만 연결. 독립 GT 평가 아님.</p>']
    for c in cards:
        d=c['decision'];parts.append('<section><h2>'+html.escape('%s / matched scan %d / emitting scan %d / Δ %.2fms'%(d['camera_id'],d['scan_sequence'],c['emitting_seq'],d['image_minus_scan_ns']/1e6))+'</h2>')
        parts.append('<img src="'+c['image']+'">' if 'image'in c else '<p>RGB missing/ambiguous</p>');parts.append('</section>')
    (out/'index.html').write_text(''.join(parts));print(json.dumps(dict(cards=len(cards),exact=sum(c.get('rgb_matches')==1 for c in cards),views=counts)))

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for n in ('run','bag','trace'):p.add_argument('--'+n,required=True,type=Path)
    p.add_argument('--run-id',required=True);p.add_argument('--analysis-id',required=True);main(p.parse_args())
