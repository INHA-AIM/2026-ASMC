#!/usr/bin/env python3
"""Compare deployed Medium and a frozen Large checkpoint on the same full val.

Reuse the existing CPU Medium report and the training evaluator, without using
the training GPU or test split. Training-history AMP scores stay separately named.
"""
import argparse
from collections import defaultdict
from datetime import datetime, timezone
import hashlib
import html
import json
from pathlib import Path
import sys

import cv2
import numpy as np
import torch
from torch.utils.data import DataLoader

REPO = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(REPO/'scripts'), str(REPO/'src/asmc_camera/src')]
import train_twinlite as base
from asmc_camera.training.twinlite import adapt_twinlite_outputs
from asmc_camera.training.twinlite.batching import ShapeBatchSampler


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as source:
        for block in iter(lambda: source.read(1024*1024), b''):
            digest.update(block)
    return digest.hexdigest()


def write_json(path, value):
    temporary = path.with_suffix('.json.part')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    temporary.replace(path)


def model_from(path, scale, task, root):
    checkpoint = torch.load(path, map_location='cpu')
    model, commit = base.load_external_twinlite(config=scale, root=root)
    if checkpoint['upstream_commit'] != commit or base.task_from_checkpoint(checkpoint) != task:
        raise RuntimeError('checkpoint source/task mismatch')
    base.configure_twinlite_task(model, task)
    model.load_state_dict(checkpoint['ema_state_dict'])
    return model.eval(), checkpoint


def percentages(values):
    return ''.join('<td>{:.2f}%</td>'.format(100*value) for value in values)


def render(report, output):
    rows = []
    for label, result in report['models'].items():
        metrics = result['validation']
        marks = metrics['metrics']['road_marking']['all']['class_iou']
        rows.append('<tr><th>'+html.escape(label)+'</th>'+percentages([
            metrics['six_condition_macro_mean_head_iou'],
            metrics['six_condition_macro_head_scores']['drivable'],
            metrics['six_condition_macro_head_scores']['road_marking'],
            marks['white_lane'], marks['yellow_lane'], marks['stopline']])+'</tr>')
    history = report['training_history_amp']
    epoch_rows = ''.join('<tr><td>{}</td>'.format(row['epoch'])+percentages([
        row['validation']['six_condition_macro_mean_head_iou'],
        row['validation']['six_condition_macro_head_scores']['drivable'],
        row['validation']['six_condition_macro_head_scores']['road_marking']])+
        '<td>{:.4f}</td></tr>'.format(row['train_losses']['loss']) for row in history)
    conditions = []
    for condition in sorted(report['models']['Medium v6']['validation']['condition_head_scores']['drivable']):
        row = '<tr><th>'+html.escape(condition)+'</th>'
        for label in report['models']:
            values = report['models'][label]['validation']['condition_head_scores']
            row += percentages([values['drivable'][condition], values['road_marking'][condition]])
        conditions.append(row+'</tr>')
    views = []
    view_classes = []
    for view in ('front','left','right'):
        row = '<tr><th>'+view+'</th>'
        for label in report['models']:
            metrics = report['models'][label]['validation']['metrics']
            row += percentages([metrics['drivable'][view]['iou'],
                                metrics['road_marking'][view]['mean_foreground_iou']])
        views.append(row+'</tr>')
        values = [model['validation']['metrics'] for model in report['models'].values()]
        for head,classes in [('drivable',('drivable',)),('road_marking',('white_lane','yellow_lane','stopline'))]:
            for cls in classes:
                before,after = (metrics[head][view]['class_iou'][cls] for metrics in values)
                change = 100*(after-before)
                view_classes.append('<tr><th>'+view+' / '+cls+'</th>'+percentages([before,after])+
                    '<td style="color:{}">{:+.2f}%p</td></tr>'.format('#fb7185' if change<0 else '#4ade80',change))
    points = [(90+580*i/max(1,len(history)-1),
               30+850*(1-row['validation']['six_condition_macro_mean_head_iou'])) for i,row in enumerate(history)]
    medium_score = report['models']['Medium v6']['validation']['six_condition_macro_mean_head_iou']
    graph = '<svg viewBox="0 0 720 340" role="img" aria-label="Large epoch별 조건 균형 IoU와 Medium 기준선">'
    for score in (.7,.8,.9,1.):
        y = 30+850*(1-score)
        graph += '<line x1="60" x2="690" y1="{0}" y2="{0}" stroke="#334155"/><text x="8" y="{0}" fill="#cbd5e1">{1:.0%}</text>'.format(y,score)
    graph += '<line x1="60" x2="690" y1="{0}" y2="{0}" stroke="#fbbf24" stroke-dasharray="8 5"/>'.format(30+850*(1-medium_score))
    graph += '<polyline points="'+ ' '.join('{},{}'.format(x,y) for x,y in points)+'" fill="none" stroke="#38bdf8" stroke-width="3"/>'
    for row,(x,y) in zip(history,points):
        graph += '<circle cx="{}" cy="{}" r="6" fill="#38bdf8"><title>epoch {}: {:.3%}</title></circle><text x="{}" y="335" fill="#cbd5e1">E{}</text>'.format(x,y,row['epoch'],row['validation']['six_condition_macro_mean_head_iou'],x,row['epoch'])
    graph += '</svg><p>파랑: Large 학습 중 AMP 평가 · 노랑: Medium CPU FP32 기준선. 정밀도를 맞춘 최종 수치는 위 표.</p>'
    cards = ''.join('<figure><figcaption>{} · {}시 · {}<br>RGB / GT / Medium v6 / Large epoch {}</figcaption><img loading="lazy" src="{}"></figure>'.format(
        html.escape(c['weather']), html.escape(str(c['hour'])), html.escape(c['view']),
        report['large_epoch'], html.escape(c['image'])) for c in report['previews'])
    page = '''<!doctype html><html lang="ko"><meta charset="utf-8"><title>TwinLite Medium·Large 동일 val 비교</title>
<style>body{font:16px system-ui;background:#111827;color:#e5e7eb;max-width:1500px;margin:24px auto;padding:20px}a{color:#7dd3fc}table{border-collapse:collapse;width:100%;margin:20px 0}td,th{padding:10px;border:1px solid #475569;text-align:right}th:first-child,td:first-child{text-align:left}img,svg{max-width:100%;height:auto}figure{margin:24px 0}p{line-height:1.7}.scroll{overflow-x:auto}</style>
<a href="../index.html">ASMC 실험 목록</a> · <a href="../camera-seg-large-train-20261017-r1/index.html">실시간 학습 현황</a>
<h1>TwinLite Medium·Large 동일 val 비교</h1>
<p>val 1,404장 전체 · 전방 544×960, 좌우 480×640(H×W) · 동일 RGB/GT/유효 영역 · EMA · CPU FP32 · batch4 · 같은 평가기. Test 미사용, optimizer update 없음. 아래 비교는 모델 품질이며 CPU 실행 속도로 배포 지연을 판단하지 않습니다.</p>
<p>조건 균형은 SUNNY/FOGGY×11/13/15시를 동일 비중으로 평균합니다. 노면 표시 macro는 흰차선·황색차선·정지선의 평균이며 background를 제외합니다. 이 val은 과거 Medium 개발에도 사용한 자료이며 비공개 독립 시험이 아닙니다. 모델 크기·초기화·데이터·학습량이 함께 달라졌으므로 차이를 모델 크기 하나의 효과로 해석하지 않습니다.</p>
<p>FOGGY15의 정지선 GT는 전체 432장 중 총444픽셀로 매우 작습니다. 이 조건의 정지선 IoU와 오른쪽 뷰의 희소 황색차선은 pooled 지표·원본 사례와 함께 해석합니다. 정답이나 선정 기준은 변경하지 않습니다. Semantic 비교이며 family·TensorRT 지연·배포 수락은 별도 검증입니다.</p>
<h2>같은 정밀도에서 전체 val 평가</h2><div class="scroll"><table><tr><th>모델</th><th>조건 균형 두 head</th><th>조건 균형 drivable</th><th>조건 균형 노면 표시</th><th>흰차선 pooled</th><th>황색차선 pooled</th><th>정지선 pooled</th></tr>'''+''.join(rows)+'''</table></div>
<h2>Large 수렴 추이</h2>'''+graph+'''<table><tr><th>Epoch</th><th>조건 균형 두 head</th><th>drivable</th><th>노면 표시</th><th>Train loss</th></tr>'''+epoch_rows+'''</table>
<h2>조건별 IoU</h2><table><tr><th>조건</th><th>Medium drivable</th><th>Medium 표시</th><th>Large drivable</th><th>Large 표시</th></tr>'''+''.join(conditions)+'''</table>
<h2>뷰별 IoU</h2><table><tr><th>뷰</th><th>Medium drivable</th><th>Medium 표시</th><th>Large drivable</th><th>Large 표시</th></tr>'''+''.join(views)+'''</table>
<details open><summary>뷰별 클래스 증감</summary><table><tr><th>뷰 / 클래스</th><th>Medium</th><th>Large</th><th>변화</th></tr>'''+''.join(view_classes)+'''</table></details>
<h2>같은 18개 사례</h2><p>초록: drivable · 흰색/노랑/빨강: 흰차선/황색차선/정지선. Padding 제외. 이 선별 사례는 전체 val 정량 평가를 대신하지 않습니다.</p>'''+cards+'''<p><a href="comparison.json">SHA·정밀도·전체 수치</a></p></html>'''
    (output/'index.html').write_text(page)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('data-root','dataset-version','medium-checkpoint','medium-report','large-checkpoint','training-history','twinlite-root','output'):
        parser.add_argument('--'+name, required=True)
    parser.add_argument('--large-report', help='Reuse a completed full CPU FP32 evaluation')
    args = parser.parse_args()
    torch.set_num_threads(2); cv2.setNumThreads(1)
    output = Path(args.output); output.mkdir(parents=True,exist_ok=True)
    root = Path(args.data_root)/'dataset_versions'/args.dataset_version
    medium = json.loads(Path(args.medium_report).read_text())
    meta_sha = sha(root/'dataset.json'); val_sha = sha(root/'val.jsonl')
    if medium['dataset_metadata_sha256'] != meta_sha or medium['dataset_split_manifest_sha256']['val'] != val_sha:
        raise RuntimeError('Medium evaluation dataset SHA mismatch')
    if (medium['checkpoint_sha256'] != sha(args.medium_checkpoint) or medium['device'] != 'cpu' or medium['amp']
            or medium['weights'] != 'ema_state_dict' or medium['batch_size'] != 4):
        raise RuntimeError('Medium weights/precision mismatch')
    dataset = base.MoraiTwinLiteDataset(args.data_root,args.dataset_version,'val',verify_files=False)
    task = dataset.task_spec
    large, checkpoint = model_from(args.large_checkpoint,'large',task,args.twinlite_root)
    if not checkpoint['epoch_complete']:
        raise RuntimeError('partial epoch cannot be used for final comparison')
    if checkpoint['dataset_provenance']['metadata_sha256'] != meta_sha:
        raise RuntimeError('Large training dataset SHA mismatch')
    loader = DataLoader(dataset,batch_sampler=ShapeBatchSampler(dataset.rows,4,shuffle=False),
                        num_workers=1,persistent_workers=True)
    if args.large_report:
        saved = json.loads(Path(args.large_report).read_text())
        if (saved['dataset_metadata_sha256'] != meta_sha or saved['dataset_split_manifest_sha256']['val'] != val_sha
                or saved['checkpoint_sha256'] != sha(args.large_checkpoint) or saved['device'] != 'cpu' or saved['amp']
                or saved['weights'] != 'ema_state_dict' or saved['batch_size'] != 4):
            raise RuntimeError('Large evaluation provenance mismatch')
        result = saved['results']['val']
    else:
        result = base.validate(large,loader,base.MaskedTwinLiteLoss(task_spec=task),task,
                               torch.device('cpu'),False,log_interval=25)
    if medium['results']['val']['sample_count'] != len(dataset) or result['sample_count'] != len(dataset):
        raise RuntimeError('incomplete validation')
    cells = defaultdict(list)
    for index,row in enumerate(dataset.rows):
        cells[(row['weather'],str(row['hour']),row['view'])].append(index)
    selected = [indices[len(indices)//2] for _,indices in sorted(cells.items())]
    medium_model,medium_checkpoint = model_from(args.medium_checkpoint,'medium',task,args.twinlite_root)
    previews = []; (output/'previews').mkdir(exist_ok=True)
    with torch.no_grad():
        for index in selected:
            sample = dataset[index]; rgb = (sample['image'].permute(1,2,0).numpy()*255).round().astype(np.uint8)
            original = cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR); valid = sample['valid_mask'].numpy()
            labels = [(sample['targets']['drivable'].numpy(),sample['targets']['road_marking'].numpy())]
            for model in (medium_model,large):
                outputs = adapt_twinlite_outputs(model(sample['image'].unsqueeze(0)),task.secondary_head)
                labels.append(tuple(outputs[head][0].argmax(0).numpy() for head in task.heads))
            panels = [original]
            for da,marks in labels:
                color = np.zeros_like(original); color[da>0] = (40,190,40)
                for cls,value in [(1,(255,255,255)),(2,(0,230,255)),(3,(0,0,255))]:color[marks==cls] = value
                active = ((da>0)|(marks>0))&valid; canvas = original.copy()
                canvas[active] = cv2.addWeighted(original,.5,color,.5,0)[active]; panels.append(canvas)
            name = 'previews/{:02d}.jpg'.format(len(previews))
            if not cv2.imwrite(str(output/name),np.hstack(panels)):raise RuntimeError('preview encoding failed')
            row = dataset.rows[index]
            previews.append(dict(image=name,sample_id=row['sample_id'],weather=row['weather'],hour=row['hour'],view=row['view']))
    history = json.loads(Path(args.training_history).read_text())
    report = dict(completed_at_utc=datetime.now(timezone.utc).isoformat(),dataset_version=args.dataset_version,
        dataset_metadata_sha256=meta_sha,val_manifest_sha256=val_sha,device='cpu',amp=False,
        batch_size=4,training_performed=False,test_used=False,large_epoch=checkpoint['epoch'],
        models={'Medium v6':dict(checkpoint_sha256=sha(args.medium_checkpoint),validation=medium['results']['val']),
                'Large epoch '+str(checkpoint['epoch']):dict(checkpoint_sha256=sha(args.large_checkpoint),validation=result)},
        medium_training_initialization=medium_checkpoint.get('initialization'),
        medium_checkpoint_epoch=medium_checkpoint.get('epoch'),
        evaluator_code_sha256={str(path.relative_to(REPO)):sha(path) for path in (
            REPO/'scripts/train_twinlite.py',REPO/'scripts/evaluate_twinlite.py',
            REPO/'src/asmc_camera/src/asmc_camera/training/twinlite/dataset.py')},
        training_history_amp=history,previews=previews,comparison_code_sha256=sha(__file__))
    write_json(output/'comparison.json',report); render(report,output)
    print(json.dumps({label:row['validation']['six_condition_macro_head_scores'] for label,row in report['models'].items()}),flush=True)


if __name__=='__main__':main()
