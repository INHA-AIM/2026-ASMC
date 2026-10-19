#!/usr/bin/env python3
"""Render every frozen selection image from SHA-verified saved inference results."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import shutil
import sys


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def read_rows(path):
    return [json.loads(line) for line in Path(path).read_text().splitlines() if line]


def build(run, output, latest_epoch=None, latest_results=None):
    import cv2
    from PIL import Image, ImageDraw, ImageFont
    cv2.setNumThreads(1)
    frozen = run/'code'
    sys.path.insert(0, str(frozen/'tools/perception'))
    sys.path.insert(0, str(frozen/'src/asmc_camera/src'))
    from evaluate_camera_bbox_pilot import summarize, frame_counts, scoped_complete
    from camera_pretrain_policy import CLASSES
    from asmc_camera.detection.hood_mask import HoodMask

    supervisor = json.loads((run/'supervisor.json').read_text())
    evaluations = supervisor['evaluations']
    latest = latest_epoch or max(e['epoch'] for e in evaluations)
    # "Best" explicitly means evaluated v11 fixed16 macro F1 at confidence .40.
    best = max(evaluations, key=lambda e: (e['metrics']['macro_f1_fixed16'], -e['epoch']))['epoch']
    completed = [int(p.stem.replace('epoch', '')) + 1 for p in (run/'main/fit/weights').glob('epoch*.pt')]
    if latest_epoch is None and max(completed) != latest:
        raise ValueError('Newest completed checkpoint has not been evaluated; infer it before building this report')
    if latest not in completed or (latest_results is not None and latest_epoch is None):
        raise ValueError('Explicit results require a completed, explicit checkpoint epoch')
    manifest = run/'evaluation/manifest.jsonl'
    rows = read_rows(manifest)
    if len({r['id'] for r in rows}) != len(rows):
        raise ValueError('Duplicate validation IDs')
    confidence = .4
    masks = {v: HoodMask.load(True, frozen/'src/asmc_camera/config/camera'/f) for v, f in (
        ('front', 'yolo_hood_mask_v1.yaml'), ('left', 'yolo_ego_mask_left_v1.yaml'),
        ('right', 'yolo_ego_mask_right_v1.yaml'))}
    predictions, metrics, proofs = {}, {}, {}
    for epoch in {best, latest}:
        folder = latest_results if epoch == latest and latest_results is not None else run/f'epoch-{epoch:03d}-1280'
        info = json.loads((folder/'inference.json').read_text())
        if info['manifest_sha256'] != sha(manifest):
            raise ValueError('Frozen evaluation manifest mismatch')
        checkpoint = run/f'main/fit/weights/epoch{epoch-1}.pt'
        if sha(checkpoint) != info['weights_sha256']:
            raise ValueError('Checkpoint mismatch')
        if info['mask_shas'] != {v: m.config_sha256 for v, m in masks.items()}:
            raise ValueError('Frozen mask mismatch')
        pred = read_rows(folder/'predictions.jsonl')
        report = summarize(rows, pred, confidence)
        recorded = (json.loads((folder/'evaluation.json').read_text())['control']['selection']['class']['macro_f1_fixed16']
                    if epoch == latest and latest_results is not None else
                    next(e['metrics']['macro_f1_fixed16'] for e in evaluations if e['epoch'] == epoch))
        if not math.isclose(report['macro_f1_fixed16'], recorded, abs_tol=1e-12):
            raise ValueError('Recomputed metric mismatch')
        predictions[epoch] = {p['id']: p for p in pred}
        metrics[epoch] = report
        proofs[epoch] = dict(inference=info, predictions_sha256=sha(folder/'predictions.jsonl'),
                            checkpoint=str(checkpoint), predictions=len(pred))
    for key in ('front_width', 'side_shape', 'config_sha256', 'head'):
        if proofs[best]['inference'][key] != proofs[latest]['inference'][key]:
            raise ValueError('Unequal inference settings: '+key)
    def metric_sha(info):
        return info.get('metric_evaluator_sha256', info['evaluator_source_sha256'])
    if metric_sha(proofs[best]['inference']) != metric_sha(proofs[latest]['inference']):
        raise ValueError('Unequal metric evaluator')
    precision_note = ('최고 기록은 GPU FP16, 최신 결과는 CPU FP32 진단입니다. 수치 경계 차이가 있을 수 있으며 CPU 시간은 배포 지연이 아닙니다.'
                      if proofs[best]['inference']['actual_fp16'] != proofs[latest]['inference']['actual_fp16']
                      else '두 모델 모두 GPU FP16 저장 결과입니다.')
    output.mkdir(parents=True, exist_ok=False)
    (output/'images').mkdir()
    font_path = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
    records = []
    for i, row in enumerate(rows):
        if sha(row['image']) != row['image_sha256']:
            raise ValueError('Source image SHA mismatch: '+row['id'])
        array = cv2.imread(row['image'])
        if array is None or array.shape[:2] != (row['height'], row['width']):
            raise ValueError('Image decoding/shape mismatch')
        view = row.get('view', 'unknown')
        if view in masks:
            array = masks[view].apply(array)
        original = Image.fromarray(cv2.cvtColor(array, cv2.COLOR_BGR2RGB))
        positives = [a for a in row['annotations'] if a['supervision'] == 'positive']
        record = dict(index=i+1, id=row['id'], source=row['source'], family=row['source'].split(':')[0],
                      view=view, complete=scoped_complete(row, False),
                      classes=sorted({a['training_class'] for a in positives}),
                      ignore=sum(a['supervision']=='ignore' for a in row['annotations']),
                      positive=len(positives), files={}, counts={})
        for panel, epoch in [('gt', None), ('best', best), ('latest', latest)]:
            image = original.copy()
            draw = ImageDraw.Draw(image)
            size = max(12, round(image.width/80))
            font = ImageFont.truetype(font_path, size)
            if epoch is None:
                boxes = [(a['xyxy'], a['training_class']) for a in positives]
                color = '#40e5ff'
            else:
                item = predictions[epoch][row['id']]
                if item['ego_mask_applied'] != (view in masks):
                    raise ValueError('Mask application mismatch')
                boxes = [(a['xyxy'], '%s %.2f' % (a['class_name'], a['confidence']))
                         for a in item['predictions'] if a['confidence'] >= confidence]
                counts, _ = frame_counts(row, item['predictions'], confidence)
                tp = sum(c['tp'] for c in counts.values())
                fp = sum(c['complete_predictions']-c['complete_tp'] for c in counts.values()) if record['complete'] else None
                record['counts'][panel] = dict(tp=tp, fp=fp, fn=len(positives)-tp, predictions=len(boxes))
                color = '#ffde59'
            for box, label in boxes:
                draw.rectangle(box, outline=color, width=max(2, image.width//500))
                left = min(max(0, box[0]), max(0, image.width-draw.textlength(label,font=font)-4))
                top = max(0, box[1]-size-5)
                bounds = draw.textbbox((left+2, top), label, font=font)
                draw.rectangle((left, top, bounds[2]+2, bounds[3]+2), fill='#101824')
                draw.text((left+2, top), label, fill=color, font=font)
            target = f'images/{i:03d}-{panel}.jpg'
            image.save(output/target, quality=92)
            record['files'][panel] = target
        record['changed_counts'] = record['counts']['best'] != record['counts']['latest']
        records.append(record)
    payload = dict(created_utc=datetime.now(timezone.utc).isoformat(), best=best, latest=latest, precision_note=precision_note,
                   confidence=confidence, classes=CLASSES, metrics=metrics, frames=records)
    proof = dict(created_utc=payload['created_utc'], manifest_sha256=sha(manifest), images=len(rows),
                 best_epoch=best, latest_epoch=latest, criterion='highest scheduled GPU-evaluated v11 fixed16 macro F1 @ .40',
                 candidates=[dict(epoch=e['epoch'], f1=e['metrics']['macro_f1_fixed16']) for e in evaluations],
                 checkpoints=proofs, gpu_inference_started=False, cpu_inference_started=False,
                 saved_inference_reused=True, training_modified=False, heldout_used=False, precision_note=precision_note,
                 source_images_verified=True, summaries_recomputed=True, rendered_images=len(records)*3)
    (output/'comparison.json').write_text(json.dumps(payload,ensure_ascii=False,indent=2)+'\n')
    (output/'provenance.json').write_text(json.dumps(proof,ensure_ascii=False,indent=2)+'\n')
    data = json.dumps(payload,ensure_ascii=False).replace('<','\\u003c')
    (output/'index.html').write_text(PAGE.replace('__DATA__', data),encoding='utf-8')
    shutil.copy2(__file__, output/Path(__file__).name)
    print(json.dumps(dict(output=str(output), best=best, latest=latest, images=len(rows),
                         f1={e:m['macro_f1_fixed16'] for e,m in metrics.items()})))


PAGE = '''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>v11 · val 205장 전체 · 최고/최신 체크포인트 비교</title>
<style>
body{margin:0;background:#101722;color:#e7effa;font:15px/1.6 system-ui}main{max-width:1800px;margin:auto;padding:22px}a{color:#8ed4ff}h1{font-size:26px;margin:8px 0}p{max-width:1100px}.toolbar{position:sticky;top:0;background:#172435;padding:12px;z-index:2;display:flex;gap:10px;flex-wrap:wrap}select,button{background:#23364b;color:white;border:1px solid #567089;border-radius:6px;padding:8px}article{background:#192536;border-radius:10px;padding:14px;margin:20px 0}h2{font-size:17px;margin:0 0 6px}.panels{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:10px}.panels.single{grid-template-columns:1fr;max-width:1280px;margin:auto}figure{margin:0}img{width:100%;height:auto;cursor:zoom-in;display:block}figcaption{font-weight:600;padding:8px;background:#24364a}.meta{color:#b8cadd;overflow-wrap:anywhere;font-size:13px}table{border-collapse:collapse;margin:12px 0}td,th{padding:5px 16px;text-align:right;border-bottom:1px solid #374a60}td:first-child,th:first-child{text-align:left}dialog{background:#101722;color:white;border:1px solid #567089;max-width:95vw;max-height:95vh;padding:12px}dialog img{max-height:83vh;object-fit:contain;width:auto;max-width:90vw}dialog::backdrop{background:#000c}@media(max-width:850px){.panels{grid-template-columns:1fr}}
</style><main><a href="../index.html">ASMC 실험 목록</a><h1>v11 · val 전체 체크포인트 비교</h1><p id="intro"></p>
<p id="precision"></p><p>전방1280·측방640, Ego mask ON(뷰 미확인 자료는 미적용), confidence 0.40. GT는 4px·64px² 범위의 양성만 청록색으로 표시하고 ignore는 숨겼습니다. 예측은 크기와 관계없이 노란색으로 모두 표시합니다.</p>
<p>평가본: complete194장·partial11장. F1은 complete에서 계산하며, partial의 미대응 검출을 FP로 세지 않습니다. 출처·actor 반복과 소표본 한계가 있는 개발 평가이며 배포 승인 결과가 아닙니다. 최고는 정식 GPU 평가된 v11 체크포인트의 16종 macro F1 기준입니다. 최신은 요청 시점의 완성본이며 자동 갱신하지 않습니다.</p>
<details><summary>16종 클래스별 F1 보기</summary><div id="metrics"></div></details>
<p><a href="?mode=compare">GT·두 모델 나란히</a> · <a href="?mode=best">최고 체크포인트만</a> · <a href="?mode=latest">최신 체크포인트만</a> · <a href="comparison.json">집계·프레임 기록</a> · <a href="provenance.json">SHA·재사용 근거</a></p>
<div class="toolbar"><select id="mode"><option value="compare">GT / 최고 / 최신</option><option value="best">최고만</option><option value="latest">최신만</option></select><select id="cls"><option value="">모든 GT 클래스</option></select><select id="source"><option value="">모든 출처</option><option>capture</option><option>roboflow</option><option>aim</option></select><select id="view"><option value="">모든 뷰</option><option>front</option><option>left</option><option>right</option><option>unknown</option></select><label><input id="changed" type="checkbox">TP/FP/FN·검출 수가 다른 장면</label><span id="count"></span></div><div id="frames"></div>
<dialog id="zoom"><button id="close">닫기 · Esc</button><p id="zoomtitle"></p><img id="zoomimg" alt="확대 결과"></dialog>
<script>
const D=__DATA__;
const $=id=>document.getElementById(id), pct=x=>(100*x).toFixed(2)+'%';
$('precision').textContent=D.precision_note;
$('intro').textContent=`최고 E${D.best}: ${pct(D.metrics[D.best].macro_f1_fixed16)} / 최신 완성 E${D.latest}: ${pct(D.metrics[D.latest].macro_f1_fixed16)} · 총 ${D.frames.length}장. 동일한 순서로 전체 장면을 제공합니다. 이미지를 누르면 크게 볼 수 있습니다.`;
for(const c of D.classes){let o=document.createElement('option');o.value=o.textContent=c;$('cls').append(o)}
let table=document.createElement('table');table.innerHTML=`<thead><tr><th>클래스</th><th>Complete GT</th><th>E${D.best} F1</th><th>E${D.latest} F1</th></tr></thead>`;
for(const c of D.classes){let row=table.insertRow();for(const v of [c,D.metrics[D.best].classes[c].complete_gt,pct(D.metrics[D.best].classes[c].f1),pct(D.metrics[D.latest].classes[c].f1)]){row.insertCell().textContent=v}}
$('metrics').append(table);
let initial=new URLSearchParams(location.search).get('mode');if(['best','latest','compare'].includes(initial))$('mode').value=initial;
function render(){const mode=$('mode').value;history.replaceState(null,'','?mode='+mode);const frames=D.frames.filter(r=>(!$('cls').value||r.classes.includes($('cls').value))&&(!$('source').value||r.family===$('source').value)&&(!$('view').value||r.view===$('view').value)&&(!$('changed').checked||r.changed_counts));$('count').textContent=`${frames.length} / ${D.frames.length}장`;$('frames').replaceChildren();
for(const r of frames){const card=document.createElement('article');let h=document.createElement('h2');h.textContent=`#${String(r.index).padStart(3,'0')} · ${r.view} · ${r.classes.join(', ')||'양성 없음'}`;card.append(h);let meta=document.createElement('p');meta.className='meta';meta.textContent=`${r.source} | ${r.complete?'complete':'partial · FP 미평가'} | GT ${r.positive}, ignore ${r.ignore} | ${r.id}`;card.append(meta);let panels=document.createElement('div');panels.className='panels'+(mode==='compare'?'':' single');
for(const kind of mode==='compare'?['gt','best','latest']:[mode]){let fig=document.createElement('figure'),cap=document.createElement('figcaption'),img=document.createElement('img');let label=kind==='gt'?'GT · 양성':`${kind==='best'?'최고':'최신'} · E${D[kind]}`;if(kind!=='gt'){let c=r.counts[kind];label+=` | TP ${c.tp} · FP ${c.fp===null?'미평가':c.fp} · FN ${c.fn} | 표시 ${c.predictions}`}cap.textContent=label;img.src=r.files[kind];img.loading='lazy';img.decoding='async';img.alt=`#${r.index} ${label}`;img.onclick=()=>{$('zoomimg').src=img.src;$('zoomtitle').textContent=`#${r.index} ${label}`;$('zoom').showModal()};fig.append(cap,img);panels.append(fig)}card.append(panels);$('frames').append(card)}}
for(const id of ['mode','cls','source','view','changed'])$(id).onchange=render;$('close').onclick=()=>$('zoom').close();render();
</script></main></html>'''


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--latest-epoch', type=int)
    parser.add_argument('--latest-results', type=Path)
    args = parser.parse_args()
    build(args.run.resolve(), args.output.resolve(), args.latest_epoch,
          args.latest_results.resolve() if args.latest_results else None)
