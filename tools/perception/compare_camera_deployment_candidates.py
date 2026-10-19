#!/usr/bin/env python3
"""Reproduce frozen E8/E22 metrics, then evaluate agreed mobility groups."""
import argparse, json, sys, hashlib, html
from pathlib import Path

def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def rows(p): return [json.loads(s) for s in Path(p).read_text().splitlines() if s]
def vehicle_diagnosis(ev, gt, predictions):
    from collections import defaultdict, Counter
    counts=defaultdict(Counter); by={p['id']:p for p in predictions}
    for row in gt:
        boxes=by[row['id']]['predictions'];_,pairs=ev.frame_counts(row,boxes,.4,'accepted',True)
        matches={g:p for g,p,_ in pairs}
        positives=[a for a in row['annotations'] if a.get('supervision') not in ('ignore','excluded') and ev.training_label(a)]
        for i,ann in enumerate(positives):
            label=ev.training_label(ann)
            if label not in ('car','bus','truck'):continue
            counts[label]['annotated_gt']+=1
            if i in matches:
                counts[label]['vehicle_detected']+=1
                counts[label]['as_'+boxes[matches[i]]['class_name']]+=1
            else:counts[label]['vehicle_missed']+=1
    return dict(counts)

def main():
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    repo=Path(__file__).resolve().parents[2]
    sys.path.insert(0,str(repo/'src/asmc_camera/src'))
    from asmc_camera.detection.mission import GROUPS, group_of
    from asmc_camera.detection.output_nms import suppress, POLICY
    sys.path.insert(0,str(a.run/'code/tools/perception'))
    import evaluate_camera_bbox_pilot as ev
    ev.accepted_name=lambda name:group_of(name) or 'unknown'
    gt=[r for r in rows(a.run/'evaluation/manifest.jsonl') if r['split']=='selection']
    results={}; predictions={}
    a.output.mkdir(parents=True,exist_ok=True)
    for epoch in (8,22):
        folder=a.run/f'epoch-{epoch:03d}-1280';raw=rows(folder/'predictions.jsonl')
        original=json.loads((folder/'evaluation.json').read_text())['control']['selection']['class']
        control=ev.summarize(gt,raw,.4,pixel_scope=True)
        for name,c in original['classes'].items():
            for key in ('gt','tp','complete_gt','complete_tp','complete_predictions'):
                assert c.get(key,0)==control['classes'][name].get(key,0),(epoch,name,key)
        reduced=[dict(r,predictions=suppress(r['predictions'],.4,.7)[0]) for r in raw]
        full=ev.summarize(gt,reduced,.4,pixel_scope=True)
        grouped=ev.summarize(gt,reduced,.4,level='accepted',pixel_scope=True)
        scores=[grouped['classes'].get(g,{}).get('f1') for g in GROUPS]
        assert all(s is not None for s in scores)
        grouped['macro_f1_fixed3']=sum(scores)/3
        info=json.loads((folder/'inference.json').read_text())
        results[epoch]=dict(raw16=control,post_nms16=full,groups=grouped,
            vehicle_diagnosis=vehicle_diagnosis(ev,gt,reduced),
            checkpoint_sha256=info['weights_sha256'],predictions_sha256=sha(folder/'predictions.jsonl'))
        predictions[epoch]={r['id']:r['predictions'] for r in reduced}
    delta=results[22]['groups']['macro_f1_fixed3']-results[8]['groups']['macro_f1_fixed3']
    preferred=22 if delta>.01 else 8
    out=dict(version=1,policy=POLICY,groups=GROUPS,manifest_sha256=sha(a.run/'evaluation/manifest.jsonl'),
        images=len(gt),results=results,provisional_candidate=preferred,
        decision_scope='GT-only; bag failures can block, no runtime promotion',runtime_approved=False)
    (a.output/'comparison.json').write_text(json.dumps(out,indent=2))
    import cv2
    from asmc_camera.detection.hood_mask import HoodMask
    masks={v:HoodMask.load(True,repo/'src/asmc_camera/config/camera'/f) for v,f in
        [('front','yolo_hood_mask_v1.yaml'),('left','yolo_ego_mask_left_v1.yaml'),('right','yolo_ego_mask_right_v1.yaml')]}
    cards=[];(a.output/'images').mkdir(exist_ok=True)
    for i,r in enumerate(gt):
        image=cv2.imread(r['image']); assert image is not None,r['image']
        if r.get('view') in masks: image=masks[r['view']].apply(image)
        tiles=[]
        for epoch in (8,22):
            tile=image.copy()
            for ann in r['annotations']:
                if ann.get('supervision') not in ('positive',None):continue
                if not ev.training_label(ann):continue
                x1,y1,x2,y2=map(int,ann['xyxy']);cv2.rectangle(tile,(x1,y1),(x2,y2),(255,180,0),1)
            for box in predictions[epoch][r['id']]:
                x1,y1,x2,y2=map(int,box['xyxy']);cv2.rectangle(tile,(x1,y1),(x2,y2),(0,230,255),2)
                cv2.putText(tile,box['class_name']+' %.2f'%box['confidence'],(x1,max(15,y1-3)),0,.45,(0,230,255),1)
            tile=cv2.resize(tile,(640,360));cv2.putText(tile,f'Epoch {epoch}',(12,25),0,.7,(255,255,255),2);tiles.append(tile)
        import numpy as np
        cv2.imwrite(str(a.output/'images'/f'{i:03d}.jpg'),np.hstack(tiles))
        cards.append(f'<article><p>{i+1:03d} · {html.escape(r["id"])} · {html.escape(r.get("view","unknown"))}</p><img loading="lazy" src="images/{i:03d}.jpg" width="1280"></article>')
    lines=''.join(f'<tr><td>{e}</td><td>{v["post_nms16"]["macro_f1_fixed16"]:.2%}</td><td>{v["groups"]["macro_f1_fixed3"]:.2%}</td>'+''.join(f'<td>{v["groups"]["classes"][g]["f1"]:.2%}</td>' for g in GROUPS)+'</tr>' for e,v in results.items())
    (a.output/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>E346 · 2026-10-14 · E8/E22 배포 후보</title><style>body{background:#142031;color:#eef;font:16px system-ui;margin:24px}a{color:#9df}td,th{padding:12px}img{max-width:100%}article{margin:30px 0}</style><a href="../index.html">ASMC 실험 목록</a><h1>E346 · 2026-10-14 · E8/E22 배포 후보</h1><p>같은 클래스 NMS IoU 0.7 · confidence 0.40 · 정상 GT 먼저 일대일 대응 · partial FP 제외. 청록 GT / 노랑 예측. 그룹 후 NMS 없음. 개발 val205이며 비공개 시험이 아닙니다.</p><a href="comparison.json">전체 평가 근거</a><table><tr><th>Epoch</th><th>16종 F1</th><th>3그룹 F1</th><th>vehicle</th><th>dynamic</th><th>static</th></tr>'+lines+'</table>'+''.join(cards))
    print(json.dumps(dict(provisional_candidate=preferred,metrics={e:dict(f1_16=v['post_nms16']['macro_f1_fixed16'],f1_3=v['groups']['macro_f1_fixed3']) for e,v in results.items()})))
if __name__=='__main__': main()
