#!/usr/bin/env python3
"""Compare saved engines on identical RGB; gallery is evidence, not acceptance."""
import argparse
import html
import json
from pathlib import Path
import time
import cv2
from camera_data_audit import read_jsonl,save,training_label,sha
from evaluate_rfdetr_candidate import seven_rows,validate_saved_evaluation
from diagnose_camera_bbox_regression import diagnose_frame


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--dataset',type=Path,required=True);p.add_argument('--baseline',type=Path,required=True)
    p.add_argument('--candidate',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--parity-report',type=Path,help='Exact candidate engine numerical qualification evidence')
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    rows=seven_rows([r for r in read_jsonl(a.dataset/'manifest.jsonl') if r['split'] in ('selection','heldout')])
    bp={r['id']:r for r in read_jsonl(a.baseline/'predictions.jsonl')}
    cp={r['id']:r for r in read_jsonl(a.candidate/'predictions.jsonl')}
    validate_saved_evaluation(a.dataset,a.baseline,rows,list(bp.values()))
    candidate_info=validate_saved_evaluation(a.dataset,a.candidate,rows,list(cp.values()))
    parity=json.loads(a.parity_report.read_text()) if a.parity_report else None
    if parity and parity['model_sha256']!=candidate_info['weights_sha256']:
        raise ValueError('gallery parity engine SHA mismatch')
    be=json.loads((a.baseline/'evaluation.json').read_text());ce=json.loads((a.candidate/'evaluation.json').read_text())
    kind=json.loads((a.candidate/'inference.json').read_text())['kind']
    candidate_label='RF-DETR TensorRT diagnostic' if kind=='rfdetr-trt' else 'RF-DETR PyTorch diagnostic'
    failures=[];picked=[]
    for r in rows:
        b=diagnose_frame(r,bp[r['id']]['predictions'],.4);c=diagnose_frame(r,cp[r['id']]['predictions'],.4)
        changes=[dict(baseline=x,candidate=y) for x,y in zip(b,c) if x['category']!=y['category']]
        if changes:failures.append(dict(id=r['id'],split=r['split'],view=r.get('view'),changes=changes))
        if changes:picked.append((r,changes))
    # Keep all changes in machine-readable evidence; bound only preview images.
    selected=picked[:36]
    present={r['id'] for r,_ in selected}
    selected += [(r,[]) for r in rows[::max(1,len(rows)//12)] if r['id'] not in present][:12]
    blocks=[]
    for index,(row,changes) in enumerate(selected):
        raw=cv2.imread(row['image']);panels=[]
        for label,items,color in [
            ('Original GT',[dict(class_name=training_label(ann),confidence=1,xyxy=ann['xyxy'])
                for ann in row['annotations'] if training_label(ann)],(255,230,0)),
            ('YOLO26l operating engine',bp[row['id']]['predictions'],(0,255,0)),
            (candidate_label,cp[row['id']]['predictions'],(0,150,255))]:
            image=raw.copy()
            for box in items:
                if box['confidence']<.4:continue
                x1,y1,x2,y2=[int(v) for v in box['xyxy']]
                cv2.rectangle(image,(x1,y1),(x2,y2),color,2)
                cv2.putText(image,'%s %.2f'%(box['class_name'],box['confidence']),
                    (x1,max(14,y1-3)),cv2.FONT_HERSHEY_SIMPLEX,.45,color,1,cv2.LINE_AA)
            target=(640,round(image.shape[0]*640/image.shape[1]))
            image=cv2.resize(image,target)
            cv2.putText(image,label,(8,24),cv2.FONT_HERSHEY_SIMPLEX,.6,(255,255,255),2,cv2.LINE_AA)
            panels.append(image)
        import numpy as np
        name='frame-%03d.jpg'%index;cv2.imwrite(str(a.output/name),np.concatenate(panels,axis=1))
        blocks.append('<article><h2>%s · %s · %s</h2><img loading="lazy" src="%s"><pre>%s</pre></article>'%
            (html.escape(row['id']),row['split'],html.escape(str(row.get('view'))),name,
             html.escape(json.dumps(changes,ensure_ascii=False,indent=2))))
    report=dict(created_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),baseline=be,candidate=ce,
        changed_frames=failures,preview_images=len(selected),candidate_kind=kind,
        parity=parity,parity_report_sha256=sha(a.parity_report) if a.parity_report else None,
        runtime_accepted=False,
        scope='same RGB at confidence .40; GT original, only known COCO7; no new labels/training')
    save(a.output/'comparison.json',report)
    parity_blocks=[]
    if parity:
        all_rows={r['id']:r for r in read_jsonl(a.dataset/'manifest.jsonl')}
        all_rows.update({r['id']:r for r in read_jsonl(a.dataset/'temporal-probe.jsonl')})
        ordered=list(cp);seen=set()
        for record in parity['records']:
            start=int(record['key']) if record['key'].isdigit() else 0
            for result in record['thresholds']:
                image_index=start+result['image_in_batch']
                for match in result['matches']:
                    if match['iou']>=.98 and match['confidence_delta']<=.02:continue
                    key=(image_index,tuple(match['reference_xyxy']),tuple(match['candidate_xyxy']))
                    if key in seen:continue
                    seen.add(key);row=all_rows[ordered[image_index]];image=cv2.imread(row['image'])
                    for label,box,color in [('PT',match['reference_xyxy'],(0,255,255)),
                                            ('TRT',match['candidate_xyxy'],(0,0,255))]:
                        x1,y1,x2,y2=map(int,box);cv2.rectangle(image,(x1,y1),(x2,y2),color,2)
                    aa,bb=match['reference_xyxy'],match['candidate_xyxy']
                    x1=max(0,int(min(aa[0],bb[0]))-40);y1=max(0,int(min(aa[1],bb[1]))-40)
                    x2=min(image.shape[1],int(max(aa[2],bb[2]))+40);y2=min(image.shape[0],int(max(aa[3],bb[3]))+40)
                    stem='parity-%02d'%len(seen);cv2.imwrite(str(a.output/(stem+'.jpg')),image)
                    crop=image[y1:y2,x1:x2];cv2.imwrite(str(a.output/(stem+'-zoom.jpg')),
                        cv2.resize(crop,(640,round(crop.shape[0]*640/crop.shape[1]))))
                    parity_blocks.append('<article><h2>수치 일치 미달 · %s</h2><p>노랑 PT / 빨강 TRT · IoU %.4f · confidence 차이 %.4f</p><img src="%s.jpg"><img style="max-width:640px" src="%s-zoom.jpg"><pre>%s</pre></article>'%
                        (html.escape(row['id']),match['iou'],match['confidence_delta'],stem,stem,
                         html.escape(json.dumps(match,ensure_ascii=False,indent=2))))
    table=[]
    for role in ('selection','heldout'):
        for name in sorted(set(be['control'][role]['classes'])|set(ce['control'][role]['classes'])):
            b=be['control'][role]['classes'].get(name,{});c=ce['control'][role]['classes'].get(name,{})
            table.append('<tr><td>%s</td><td>%s</td><td>%s</td><td>%s</td><td>%s</td><td>%s</td></tr>'%
                (role,name,b.get('gt',0),b.get('tp',0),c.get('tp',0),html.escape(str(c.get('f1')))))
    document='''<!doctype html><html lang="ko"><meta charset="utf-8"><title>RF-DETR L704 / YOLO26l 동일 RGB 비교</title>
<style>body{font:16px system-ui;margin:24px;background:#edf2f7;color:#142535}img{width:100%%}article{background:white;padding:16px;margin:24px 0}pre{white-space:pre-wrap}td,th{padding:8px;border:1px solid #aaa}</style>
<a href="../../index.html">전체 실험 목차</a><h1>RF-DETR L704 / YOLO26l 동일 RGB 비교</h1>
<p style="background:#ffe0a3;padding:16px"><strong>진단용 · 운영 미수락</strong> — QUALIFICATION.
MORAI 포함 통합 지연은 이 페이지에서 측정하지 않았습니다. 품질·속도 순위나 운영 교체 승인을 뜻하지 않습니다.</p>
<p>좌: 원본 GT · 중앙: 현재 YOLO26l TensorRT · 우: CANDIDATE_KIND. confidence 0.40.
작은 GT는 원본대로 표시하되 지표의 크기 범위는 기존 기준을 유지합니다. 부분 GT에서 미대응 검출을 FP로 세지 않습니다.
오토바이 heldout은 단일 소스에 편중돼 있으며 일부 클래스는 평가 자료가 없습니다. 오프라인 추론 시간은 통합 지연이 아닙니다.</p>
<table><tr><th>분할</th><th>클래스</th><th>GT</th><th>YOLO TP</th><th>RF TP</th><th>RF F1 (완전 GT만)</th></tr>%s</table>
<p><a href="comparison.json">전체 지표·confidence 탐색·변경 목록</a></p>%s%s</html>'''%(''.join(table),''.join(parity_blocks),''.join(blocks))
    qualification=('FP16 수치 일치 기준 미달' if parity and not parity['passed'] else
                   '수치 일치 통과, 통합·품질 수락은 별도' if parity else '수치 일치 검증 근거 미첨부')
    (a.output/'index.html').write_text(document.replace('CANDIDATE_KIND',html.escape(candidate_label)).replace(
        'QUALIFICATION',qualification),encoding='utf-8')
    print(json.dumps(dict(changed_frames=len(failures),preview_images=len(selected),gallery=str(a.output))))


if __name__=='__main__':main()
