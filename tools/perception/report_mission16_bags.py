#!/usr/bin/env python3
"""Summarize original trial failures and render identical-stamp candidate pairs."""
import argparse,json,collections,html
from pathlib import Path
import numpy as np

def main():
 p=argparse.ArgumentParser();p.add_argument('--workspace',type=Path,required=True);a=p.parse_args();w=a.workspace;out=w/'bag-review';out.mkdir(exist_ok=True);summary=[]
 for phase in ('compare','engine'):
  for f in sorted((w/('bags-'+phase)).glob('*/report.json')):
   r=json.loads(f.read_text());o=json.loads((f.parent/'observations.json').read_text())
   resource=r.get('resources',{});views={}
   raw_resources=json.loads((f.parent/'resources.json').read_text())
   ratios=[s['gpu']['memory_used_mib']/s['gpu']['memory_total_mib'] for s in raw_resources
           if s.get('gpu',{}).get('memory_total_mib',0)>0 and s['gpu'].get('memory_used_mib') is not None]
   vram=dict(maximum_percent=100*max(ratios) if ratios else None,
             within_90_percent=bool(ratios) and max(ratios)<=.9,
             scope='whole GPU, all recorded phases; not process-only VRAM')
   for name,m in r.get('measurements',{}).items():
    if not name.startswith('yolo_'):continue
    q=m.get('input_to_output_wall_ms',{});good=bool(m.get('five_second_hz')) and min(m['five_second_hz'])>=9.5 and (q.get('p95') or 1e9)<=150 and (q.get('p99') or 1e9)<=200 and m.get('max_gap_sec',1e9)<=.25 and not m.get('duplicate_stamps') and not m.get('reversed_stamps')
    views[name]=dict(hz=m.get('hz'),five_second_hz=m.get('five_second_hz'),p50=q.get('p50'),p95=q.get('p95'),p99=q.get('p99'),gap_ms=m.get('max_gap_sec',0)*1000,gate=good,duplicates=m.get('duplicate_stamps'),reverse=m.get('reversed_stamps'))
   names=collections.Counter((v[1],v[2]) for row in o.get('tracks',[]) for v in row['objects'])
   fusion=r.get('fusion',{});fusion={k:fusion.get(k) for k in ('totals','offset_p95_max_ms','accounting_complete','pending_end','duplicate_processed_stamps')}
   summary.append(dict(phase=phase,trial=f.parent.name,views=views,vram=vram,original_pass=r.get('passed'),failed_checks=[k for k,v in r.get('checks',{}).items() if v is False],fusion=fusion,resources=resource,
     fused_labels=[dict(group=g,source=s,observations=n) for (g,s),n in names.items()],control_messages=len(o.get('control_messages',[]))))
 (out/'summary.json').write_text(json.dumps(summary,indent=2))
 import rosbag,cv2,yaml
 from asmc_camera.detection.hood_mask import HoodMask
 cfg=yaml.safe_load((w/'config/experiments/camera_lidar_b0_v1.yaml').read_text());cards=[]
 topics=dict(front='/image_jpeg/compressed',left='/cam_left/image_jpeg/compressed',right='/cam_right/image_jpeg/compressed')
 masks={v:HoodMask.load(True,w/'src/asmc_camera/config/camera'/f) for v,f in [('front','yolo_hood_mask_v1.yaml'),('left','yolo_ego_mask_left_v1.yaml'),('right','yolo_ego_mask_right_v1.yaml')]}
 for bag_id in ('primary','shadow'):
  paths=[w/f'bags-compare/e{e}-{bag_id}-r1/observations.json' for e in (8,22)]
  if not all(p.exists() for p in paths):continue
  obs=[json.loads(p.read_text()) for p in paths];by={e:{v:{r['stamp_ns']:r for r in o['outputs'].get('yolo_'+v,[])} for v in topics} for e,o in zip((8,22),obs)}
  selected={}
  for v in topics:
   stamps=sorted(set(by[8][v])&set(by[22][v]));selected[v]={stamps[i] for i in np.linspace(0,len(stamps)-1,min(12,len(stamps)),dtype=int)} if stamps else set()
  with rosbag.Bag(cfg['fusion']['bags'][bag_id]) as bag:
   for topic,msg,_ in bag.read_messages(topics=list(topics.values())):
    view=next(v for v,t in topics.items() if t==topic);stamp=msg.header.stamp.to_nsec()
    if stamp not in selected[view]:continue
    image=cv2.imdecode(np.frombuffer(msg.data,np.uint8),cv2.IMREAD_COLOR);image=masks[view].apply(image);tiles=[]
    for e in (8,22):
     tile=image.copy()
     for b in by[e][view][stamp].get('boxes',[]):
      x1,y1,x2,y2=map(int,b['xyxy']);cv2.rectangle(tile,(x1,y1),(x2,y2),(0,220,255),2)
      cv2.putText(tile,b['label']+' %.2f'%b['confidence'],(x1,max(y1-3,16)),0,.45,(0,220,255),1)
     tile=cv2.resize(tile,(640,360));cv2.putText(tile,f'Epoch {e} | {view}',(12,25),0,.65,(255,255,255),2);tiles.append(tile)
    name=f'{bag_id}-{view}-{stamp}.jpg';cv2.imwrite(str(out/name),np.hstack(tiles));cards.append(f'<p>{bag_id} / {view} / {stamp}</p><img loading="lazy" width="1280" src="{name}">')
 table=''
 for row in summary:
  for view,m in row['views'].items():
   table+='<tr>'+''.join('<td>'+html.escape(str(x))+'</td>' for x in [row['phase'],row['trial'],view,round(m['hz'] or 0,2),*[round(m[k] or 0,1) for k in ['p50','p95','p99','gap_ms']],m['gate']])+'</tr>'
 (out/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>E345 · 2026-10-14 · 배포 후보 bag 비교</title><style>body{background:#142031;color:#eef;font:15px system-ui;margin:24px}a{color:#9df}td,th{padding:8px}img{max-width:100%}</style><h1>E345 · 2026-10-14 · 배포 후보 bag 비교</h1><p>동일 원본 stamp의 E8/E22 비교. GT 없는 bag 사례이며 박스 수는 recall이 아닙니다. PyTorch 후보 비교와 TensorRT 배포 부하는 분리합니다. 첫 안정 검출·객체별 누락률은 독립 연속 GT 부족으로 정량 수락하지 않습니다.</p><a href="summary.json">원본 실패·자원·fusion JSON</a><table><tr><th>backend</th><th>trial</th><th>view</th><th>Hz</th><th>p50</th><th>p95</th><th>p99</th><th>max gap</th><th>gate</th></tr>'+table+'</table>'+''.join(cards))
 print(json.dumps(dict(trials=len(summary),images=len(cards))))
if __name__=='__main__':main()
