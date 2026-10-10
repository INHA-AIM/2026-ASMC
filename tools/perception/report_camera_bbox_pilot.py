#!/usr/bin/env python3
"""Publish pilot progress, limitations and GT/FP comparisons in the existing hub."""
import argparse
from collections import Counter
import html
import json
from pathlib import Path
import shutil

from camera_data_audit import read_jsonl,save,sha
from evaluate_camera_bbox_pilot import frame_counts


def build(args):
    args.output.mkdir(parents=True,exist_ok=True)
    sampling=json.loads((args.dataset/'sampling.json').read_text())
    rows=[r for r in read_jsonl(args.dataset/'manifest.jsonl') if r['split']!='train']
    summary=dict(dataset=sampling,comparison=None,evaluations={},selection=None,
        blockers=['selection 신규 장애물 GT 부족', 'animal heldout GT 1개: 2%p 회귀 판정 불가',
                  'stroller 학습/평가 자료 없음', 'sparse eval: 실제 시간적 공백·최초 안정 검출 미측정',
                  'TensorRT·전체 스택 지연·fusion 검증은 이번 파일럿 이후'])
    state=args.output/'paired-r1/comparison.json'
    if state.exists():summary['comparison']=json.loads(state.read_text())
    galleries=[]
    for folder in sorted((args.output/'paired-r1').glob('*-after-*')):
        path=folder/'evaluation.json'
        if not path.exists():continue
        summary['evaluations'][folder.name]=json.loads(path.read_text())
        # Record before/after without rewriting the evaluated GT.
        before=args.output/'paired-r1'/folder.name.replace('-after-','-before-')
        if (before/'evaluation.json').exists():summary['evaluations'][before.name]=json.loads((before/'evaluation.json').read_text())
        predictions={p['id']:p for p in read_jsonl(folder/'predictions.jsonl')}
        examples=[];counts=Counter()
        for row in rows:
            pred=predictions[row['id']]['predictions']
            c,pairs=frame_counts(row,pred,.4,pixel_scope=True)
            missed=any(v['gt']>v['tp'] for v in c.values())
            fp=any(v['complete_predictions']>v['complete_tp'] for v in c.values())
            kind='FP' if fp else 'miss' if missed else 'matched'
            if counts[kind]>=12:continue
            counts[kind]+=1;examples.append((row,pred,kind))
        if examples:
            from PIL import Image,ImageDraw,ImageOps
            destination=args.output/'gallery'/folder.name;destination.mkdir(parents=True,exist_ok=True)
            cards=[]
            for row,pred,kind in examples:
                with Image.open(row['image']) as original:
                    image=original.convert('RGB');draw=ImageDraw.Draw(image)
                    for ann in row['annotations']:
                        if not ann.get('training_class'):continue
                        box=ann['xyxy'];draw.rectangle(box,outline='#00e8ff',width=2)
                        draw.text((box[0],max(0,box[1]-12)),ann['training_class'],fill='#00e8ff')
                    for p in pred:
                        if p['confidence']<.4:continue
                        box=p['xyxy'];draw.rectangle(box,outline='#ff4aaa',width=2)
                        draw.text((box[0],box[1]+2),f"{p['class_name']} {p['confidence']:.2f}",fill='#ff4aaa')
                    ImageOps.contain(image,(960,720)).save(destination/(row['id']+'.jpg'),quality=90)
                cards.append(f'<article><p>{kind} · {html.escape(row["source"])} · {row["split"]} · {row["evaluation_scope"]}</p><img src="{row["id"]}.jpg"></article>')
            (destination/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>bbox pilot '+folder.name+'</title><a href="../../index.html">실험 목차</a><p>cyan 원본 GT / pink confidence .40 검출. partial GT의 미대응 검출은 FP가 아닙니다.</p>'+''.join(cards))
            galleries.append(f'<li><a href="gallery/{folder.name}/index.html">{folder.name} GT·오검출·누락</a></li>')
    save(args.output/'report.json',summary)
    frequencies=sampling['train']
    table=''.join(f'<tr><td>{name}</td><td>{count}</td><td>{frequencies["image_frequency_after"].get(name,0)}</td></tr>'
        for name,count in frequencies['image_frequency_before'].items())
    (args.output/'index.html').write_text('''<!doctype html><meta charset="utf-8">
<title>2026-10-11 · bbox 파일럿: 샘플링·YOLO26l/YOLOE 비교</title>
<style>body{font:16px sans-serif;max-width:1200px;margin:32px auto;background:#111827;color:#e5e7eb}a{color:#67e8f9}td,th{border:1px solid #4b5563;padding:8px}table{border-collapse:collapse}pre{white-space:pre-wrap}img{max-width:100%}</style>
<a href="../index.html">전체 실험 목차</a><h1>2026-10-11 · bbox 비교학습</h1>
<p>원본 22,498장 보존. 파일럿 train 8,000장·selection 300장·heldout 300장. 16종 ID와 모든 원본 대상 박스 유지.</p>
<p>YOLO26l detection / YOLOE-26l detection. 960·AMP·batch4·유효 batch16·17epoch, 직렬 실행. 운영 모델 변경 없음.</p>
<h2>현재 진행</h2><pre id="progress">읽는 중…</pre>
<p>학습 중에는 MORAI/실시간 모델을 함께 실행하지 않습니다. 아래 시간은 학습 시간이며 통합 자율주행 지연이 아닙니다.</p>
<h2>학습 이미지 등장 수</h2><table><tr><th>클래스</th><th>원본 train</th><th>파일럿</th></tr>'''+table+'''</table>
<h2>평가 범위</h2><p>selection 신규 객체는 box 6개뿐입니다. animal heldout은 deer 1개입니다. 소표본과 미지원 종류를 지원 완료로 해석하지 않습니다. 부분 주석 영상은 recall-only이며 precision/FP/F1을 만들지 않습니다. 같은 run·증강 계보는 독립 장면 수가 아닙니다.</p>
<p>희소 클래스 자료 전부·상자 1,007장·콘크리트 방벽 844장·철제 방벽 145장·PE 방벽149장 포함. Capture 1,268장. 나머지 원본 자료는 본학습 후보용으로 보존.</p>
<h2>결과·실패 사례</h2><ul>'''+''.join(galleries)+'''</ul><a href="report.json">전체 집계 JSON</a>
<script>
async function update(){let text='';try{const c=await(await fetch('paired-r1/comparison.json?t='+Date.now())).json();text+=JSON.stringify({status:c.status,stage:c.stage,batch:c.batch,epochs:c.epochs,wall_seconds:c.wall_seconds},null,2)+'\\n';
for(const model of ['yolo26l','yoloe26l']){try{const r=await(await fetch('paired-r1/'+model+'/run.json?t='+Date.now())).json();text+=model+' '+JSON.stringify({status:r.status,batches:r.completed_batches,last_epoch:r.last_epoch,loss:r.last_loss,wall_seconds:r.wall_seconds})+'\\n';}catch{ text+=model+' 대기\\n';}}
}catch{text='학습 준비 중';}document.getElementById('progress').textContent=text;}update();setInterval(update,30000);
</script>''')
    return summary


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--dataset',required=True,type=Path);p.add_argument('--output',required=True,type=Path)
    result=build(p.parse_args());print(json.dumps(dict(evaluations=list(result['evaluations']),blockers=result['blockers']),ensure_ascii=False))
