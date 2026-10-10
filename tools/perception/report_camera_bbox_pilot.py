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


NEW_CLASSES={'deer','bollard','box','concrete_barrier','steel_barricade','traffic_drum',
             'plastic_barrier','traffic_cone','stroller'}


def compare(evaluations):
    """Descriptive pilot choice, never independent final-test/production approval."""
    comparisons=[]
    for key,after in evaluations.items():
        if '-after-' not in key:continue
        before=evaluations.get(key.replace('-after-','-before-'))
        if not before:continue
        # Regression is against today's deployed model/resolution, not a
        # weaker candidate's own pretrained result. Retain each before/after
        # evaluation separately for describing the fine-tuning gain.
        reference_key='yolo26l-before-1280'
        reference=evaluations.get(reference_key,before)
        if reference is before and reference_key not in evaluations:reference_key=key.replace('-after-','-before-')
        classes=after['control']['heldout']['class']['classes']
        scores=[c['f1'] for n,c in classes.items() if n in NEW_CLASSES and c.get('complete_gt',0)>=20 and c['f1'] is not None]
        old={};bad=[];uncertain=[]
        aa=after['control']['heldout']['accepted']['classes']
        bb=reference['control']['heldout']['accepted']['classes']
        for group,labels in [('person',['person']),('animal',['animal']),('vehicle',['motor_vehicle','bicycle','motorcycle'])]:
            gt=sum(aa.get(n,{}).get('gt',0) for n in labels)
            tp=sum(aa.get(n,{}).get('tp',0) for n in labels)
            bgt=sum(bb.get(n,{}).get('gt',0) for n in labels)
            btp=sum(bb.get(n,{}).get('tp',0) for n in labels)
            delta=tp/gt-btp/bgt if gt and bgt else None
            old[group]=dict(gt=gt,before=btp/bgt if bgt else None,after=tp/gt if gt else None,delta=delta)
            if gt<20:uncertain.append(group)
            elif delta is not None and delta<-.02-1e-12:bad.append(group)
        selected_classes=after.get('selected_heldout',{}).get('classes',classes)
        selected_scores=[c['f1'] for n,c in selected_classes.items() if n in NEW_CLASSES
            and c.get('complete_gt',0)>=20 and c['f1'] is not None]
        selected_old={};selected_bad=[]
        selected_accepted=after.get('selected_heldout_accepted',{}).get('classes',aa)
        for group,labels in [('person',['person']),('animal',['animal']),('vehicle',['motor_vehicle','bicycle','motorcycle'])]:
            gt=sum(selected_accepted.get(n,{}).get('gt',0) for n in labels)
            tp=sum(selected_accepted.get(n,{}).get('tp',0) for n in labels)
            baseline=old[group]['before'];delta=tp/gt-baseline if gt and baseline is not None else None
            selected_old[group]=dict(gt=gt,after=tp/gt if gt else None,delta_from_pretrained_control=delta)
            if gt>=20 and delta is not None and delta<-.02-1e-12:selected_bad.append(group)
        comparisons.append(dict(configuration=key,new_macro_f1=sum(scores)/len(scores) if scores else None,
            selected_new_macro_f1=sum(selected_scores)/len(selected_scores) if selected_scores else None,
            selected_old_recall=selected_old,selected_old_regression_failures=selected_bad,
            supported_new_classes=[n for n,c in classes.items() if n in NEW_CLASSES and c.get('complete_gt',0)>=20],
            old_recall=old,old_regression_failures=bad,old_regression_unresolved=uncertain,
            old_recall_reference=reference_key,
            confidence_selection_supported_new_classes=[n for n,c in after.get('selected_selection',{}).get('classes',{}).items()
                if n in NEW_CLASSES and c.get('complete_gt',0)>=20],
            confidence_control=.4,selection_confidence=after['selection_confidence']))
        comparisons[-1]['complete_fp_after']=sum(c.get('complete_fp',0) for c in classes.values())
        comparisons[-1]['complete_fp_selected']=sum(c.get('complete_fp',0) for c in selected_classes.values())
        comparisons[-1]['complete_fp_before']=sum(c.get('complete_fp',0) for c in before['control']['heldout']['class']['classes'].values())
    # .40 is a calibration-controlled comparison. Eligibility uses the
    # selection-frozen confidence, compared with today's .40 reference.
    candidates=[c for c in comparisons if c['selected_new_macro_f1'] is not None
        and not c['selected_old_regression_failures']]
    selected=max(candidates,key=lambda c:c['selected_new_macro_f1']) if candidates else None
    return dict(comparisons=comparisons,provisional_candidate=selected,
        within_one_percentage_point=[c['configuration'] for c in candidates
            if selected and selected['selected_new_macro_f1']-c['selected_new_macro_f1']<=.01+1e-12],
        status='provisional_requires_missing_GT_and_deployment_checks' if selected else 'selection_deferred',
        heldout_usage='fixed comparative pilot diagnostic, already inspected; not a fresh final deployment test',
        tie_policy='<=1pp needs TensorRT deployment latency/VRAM; offline PT latency is not a deployment tie-break')


def build(args):
    args.output.mkdir(parents=True,exist_ok=True)
    sampling=json.loads((args.dataset/'sampling.json').read_text())
    rows=[r for r in read_jsonl(args.dataset/'manifest.jsonl') if r['split']!='train']
    summary=dict(dataset=sampling,comparison=None,evaluations={},inference={},temporal={},selection=None,
        blockers=['selection 신규 장애물 GT 부족', 'animal heldout GT 1개: 2%p 회귀 판정 불가',
                  'stroller 학습/평가 자료 없음', 'sparse eval: 실제 시간적 공백·최초 안정 검출 미측정',
                  'TensorRT·전체 스택 지연·fusion 검증은 이번 파일럿 이후'])
    state=args.output/'paired-r1/comparison.json'
    if state.exists():summary['comparison']=json.loads(state.read_text())
    recovery=args.output/'paired-r1/comparison-equal-epoch.json'
    if recovery.exists():
        summary['original_comparison']=summary['comparison']
        summary['comparison']=json.loads(recovery.read_text())
    galleries=[]
    for folder in sorted((args.output/'paired-r1').glob('*-after-*')):
        path=folder/'evaluation.json'
        if not path.exists():continue
        summary['evaluations'][folder.name]=json.loads(path.read_text())
        # Record before/after without rewriting the evaluated GT.
        before=args.output/'paired-r1'/folder.name.replace('-after-','-before-')
        if (before/'evaluation.json').exists():summary['evaluations'][before.name]=json.loads((before/'evaluation.json').read_text())
        for evaluated in (folder,before):
            for filename,key in [('inference.json','inference'),('temporal.json','temporal')]:
                if (evaluated/filename).exists():summary[key][evaluated.name]=json.loads((evaluated/filename).read_text())
        predictions={p['id']:p for p in read_jsonl(folder/'predictions.jsonl')}
        original_predictions={p['id']:p for p in read_jsonl(before/'predictions.jsonl')} if (before/'predictions.jsonl').exists() else {}
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
                    panels=[]
                    for title,boxes in [('before',original_predictions.get(row['id'],{}).get('predictions',[])),('after',pred)]:
                        image=original.convert('RGB');draw=ImageDraw.Draw(image)
                        for ann in row['annotations']:
                            if not ann.get('training_class'):continue
                            box=ann['xyxy'];w=box[2]-box[0];h=box[3]-box[1]
                            long,short=(24,12) if ann['training_class'] in ('car','truck','bus') else (16,8)
                            scoped=max(w,h)>=long and min(w,h)>=short
                            color='#00e8ff' if scoped else '#ffb000'
                            draw.rectangle(box,outline=color,width=2)
                            if scoped:draw.text((box[0],max(0,box[1]-12)),ann['training_class'],fill=color)
                        for p in boxes:
                            if p['confidence']<.4:continue
                            box=p['xyxy'];draw.rectangle(box,outline='#ff4aaa',width=2)
                            draw.text((box[0],box[1]+2),f"{p['class_name']} {p['confidence']:.2f}",fill='#ff4aaa')
                        panel=Image.new('RGB',(image.width,image.height+24),'#111827')
                        panel.paste(image,(0,24));ImageDraw.Draw(panel).text((8,5),title,fill='white');panels.append(panel)
                    joined=Image.new('RGB',(panels[0].width*2,panels[0].height))
                    joined.paste(panels[0],(0,0));joined.paste(panels[1],(panels[0].width,0))
                    ImageOps.contain(joined,(1920,900)).save(destination/(row['id']+'.jpg'),quality=90)
                cards.append(f'<article><p>{kind} · {html.escape(row["source"])} · {row["split"]} · {row["evaluation_scope"]}</p><img src="{row["id"]}.jpg"></article>')
            (destination/'index.html').write_text('<!doctype html><meta charset="utf-8"><title>2026-10-11 · bbox pilot '+folder.name+'</title><style>body{font:16px sans-serif;background:#111827;color:white}img{max-width:100%}a{color:cyan}</style><a href="../../index.html">실험 목차</a><p>왼쪽 before / 오른쪽 after. cyan 평가 크기 GT / amber 작은 원본 GT / pink confidence .40 검출. 원본 RGB 위에 표시하며 추론에는 알려진 뷰의 Ego mask를 적용했습니다. partial GT의 미대응 검출은 FP가 아닙니다.</p>'+''.join(cards))
            galleries.append(f'<li><a href="gallery/{folder.name}/index.html">{folder.name} GT·오검출·누락</a></li>')
    summary['selection']=compare(summary['evaluations'])
    save(args.output/'report.json',summary)
    frequencies=sampling['train']
    table=''.join(f'<tr><td>{name}</td><td>{count}</td><td>{frequencies["image_frequency_after"].get(name,0)}</td></tr>'
        for name,count in frequencies['image_frequency_before'].items())
    comparison_table=''.join('<tr><td>'+html.escape(c['configuration'])+'</td><td>'+
        (f"{c['new_macro_f1']:.3f}" if c['new_macro_f1'] is not None else '평가 불가')+
        '</td><td>'+(f"{c['selected_new_macro_f1']:.3f}" if c['selected_new_macro_f1'] is not None else '평가 불가')+
        '</td><td>'+str(c['selection_confidence'])+'</td><td>'+html.escape(str(c['selected_old_recall']))+
        '</td><td>'+html.escape(str(c['selected_old_regression_failures']))+'</td></tr>'
        for c in summary['selection']['comparisons'])
    latency_table=''.join(f'<tr><td>{html.escape(name)}</td><td>{v["p50_ms"]:.1f}</td><td>{v["p95_ms"]:.1f}</td><td>{v["p99_ms"]:.1f}</td></tr>'
        for name,v in summary['inference'].items())
    (args.output/'index.html').write_text('''<!doctype html><meta charset="utf-8">
<title>2026-10-11 · bbox 파일럿: 샘플링·YOLO26l/YOLOE 비교</title>
<style>body{font:16px sans-serif;max-width:1200px;margin:32px auto;background:#111827;color:#e5e7eb}a{color:#67e8f9}td,th{border:1px solid #4b5563;padding:8px}table{border-collapse:collapse}pre{white-space:pre-wrap}img{max-width:100%}</style>
<a href="../index.html">전체 실험 목차</a><h1>2026-10-11 · bbox 비교학습</h1>
<p>원본 22,498장 보존. 파일럿 train 8,000장·selection 300장·heldout 300장. 16종 ID와 모든 원본 대상 박스 유지.</p>
<p>YOLO26l detection / YOLOE-26l detection. 960·AMP·batch4·유효 batch16·17epoch 배정, 직렬 실행. 예산 중단 시 동일 완료 epoch checkpoint만 비교하고 원래 실패는 보존합니다. 운영 모델 변경 없음.</p>
<h2>현재 진행</h2><pre id="progress">읽는 중…</pre>
<p>학습 중에는 MORAI/실시간 모델을 함께 실행하지 않습니다. 아래 시간은 학습 시간이며 통합 자율주행 지연이 아닙니다.</p>
<h2>학습 이미지 등장 수</h2><table><tr><th>클래스</th><th>원본 train</th><th>파일럿</th></tr>'''+table+'''</table>
<h2>평가 범위</h2><p>selection 신규 객체는 box 6개뿐입니다. animal heldout은 deer 1개입니다. 소표본과 미지원 종류를 지원 완료로 해석하지 않습니다. 부분 주석 영상은 recall-only이며 precision/FP/F1을 만들지 않습니다. 같은 run·증강 계보는 독립 장면 수가 아닙니다.</p>
<p>희소 클래스 자료 전부·상자 1,007장·콘크리트 방벽 844장·철제 방벽 145장·PE 방벽149장 포함. Capture 1,268장. 나머지 원본 자료는 본학습 후보용으로 보존.</p>
<h2>비교 결과</h2><table><tr><th>설정</th><th>신규 F1 (.40)</th><th>신규 F1 (선정 confidence)</th><th>confidence</th><th>선정 confidence의 기존 recall</th><th>2%p 회귀 실패</th></tr>'''+comparison_table+'''</table>
<p>고정 heldout의 파일럿 비교이며 새로운 독립 최종 시험이 아닙니다. GT가 충분한 종류만 macro에 포함합니다. 1%p 내 동률은 TensorRT 통합 지연·VRAM 확인 전 확정하지 않습니다.</p>
<h2>오프라인 PT batch1 지연 (ms)</h2><p>평가 이미지의 여러 뷰·해상도를 합친 분포입니다. 실제 3뷰 TensorRT 통합 지연이나 배포 동률 판정값이 아닙니다.</p><table><tr><th>설정</th><th>p50</th><th>p95</th><th>p99</th></tr>'''+latency_table+'''</table>
<h2>결과·실패 사례</h2><ul>'''+''.join(galleries)+'''</ul><a href="report.json">전체 집계 JSON</a>
<script>
async function update(){let text='';try{let c;for(const file of ['comparison-equal-epoch.json','comparison.json']){const response=await fetch('paired-r1/'+file+'?t='+Date.now());if(response.ok){c=await response.json();break;}}text+=JSON.stringify({status:c.status,stage:c.stage,batch:c.batch,epochs:c.epochs,wall_seconds:c.wall_seconds},null,2)+'\\n';
for(const model of ['yolo26l','yoloe26l']){try{const r=await(await fetch('paired-r1/'+model+'/run.json?t='+Date.now())).json();text+=model+' '+JSON.stringify({status:r.status,batches:r.completed_batches,last_epoch:r.last_epoch,loss:r.last_loss,wall_seconds:r.wall_seconds})+'\\n';}catch{ text+=model+' 대기\\n';}}
}catch{text='학습 준비 중';}document.getElementById('progress').textContent=text;}update();setInterval(update,30000);
</script>''')
    return summary


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--dataset',required=True,type=Path);p.add_argument('--output',required=True,type=Path)
    result=build(p.parse_args());print(json.dumps(dict(evaluations=list(result['evaluations']),blockers=result['blockers']),ensure_ascii=False))
