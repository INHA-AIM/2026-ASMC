#!/usr/bin/env python3
"""Consolidate six prompted candidates without scoring unlabelled obstacles as GT."""
import argparse
from collections import Counter,defaultdict
import hashlib
import html
import json
from pathlib import Path
from analyze_coco7_broad_selection import score,selected_frames
from analyze_coco7_weather import load_jsonl

COMBINATIONS=[(f'yoloe-26{s}-seg',w) for s in ('m','l') for w in (640,960,1280)]
EVALUATION_GROUPS={
    'person':'person','dog':'animal','deer':'animal',
    **{n:'vehicle' for n in ('bicycle','car','truck','bus','motorcycle','scooter','electric scooter')},
    **{n:'obstacle' for n in ('bollard','traffic bollard','cargo box','wooden box','steel barricade','traffic cone','traffic barrel','baby stroller')}}


def best_threshold(thresholds):
    available=[r for r in thresholds if r.get('macro_f1') is not None]
    return max(available,key=lambda r:(r['macro_f1'],r['confidence'])) if available else None


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root',type=Path,required=True)
    p.add_argument('--reference',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--quality-only',action='store_true',help='Freeze quality screening before timed trials; do not read partial live/spawn files')
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    manifest=a.reference/'gt-v2/frames.jsonl';digest=hashlib.sha256(manifest.read_bytes()).hexdigest()
    rows=[r for r in load_jsonl(manifest) if r['split']=='selection' and r['annotation_complete']]
    report=dict(gt_manifest_sha256=digest,frames=len(rows),quality=[],live=[],spawn_observations=[],
        evaluation_groups=EVALUATION_GROUPS,
        scope='Four verified COCO7 subtypes in Capture; new obstacles are qualitative observations, not recall. No operational promotion.')
    conditions=json.loads((a.reference/'broad-model-conditions-v1.json').read_text())
    exclusions=conditions['reviewed_exclusions']
    report['previously_user_reviewed_exclusions']=exclusions
    sources=[(m,w,a.root/'quality-v1'/f'{m}-{w}-selection') for m,w in COMBINATIONS]
    sources += [('yolo26l',1280,a.reference/'confidence-followup-actual40-v1/yolo26l-1280-selection')]
    for model,width,directory in sources:
        s=json.loads((directory/'summary.json').read_text())
        if s['gt_manifest_sha256']!=digest:raise ValueError('different GT manifest')
        gt,predictions=selected_frames(rows,load_jsonl(directory/'predictions.jsonl'),[])
        reviewed_gt,reviewed_predictions=selected_frames(rows,predictions,exclusions)
        best=best_threshold(s['thresholds'])
        fixed=score(gt,predictions,.4)
        reviewed_scores=[score(reviewed_gt,reviewed_predictions,t['confidence']) for t in s['thresholds']]
        reviewed_best=max(reviewed_scores,key=lambda t:(t['pooled']['macro_f1'],t['confidence']))
        report['quality'].append(dict(model=model,width=width,frames=s['frames'],
            predictor_cutoff=s['model']['conf_thres'],fixed=fixed,selection_best=best,
            selected=score(gt,predictions,best['confidence']) if best else None,
            reviewed_fixed=score(reviewed_gt,reviewed_predictions,.4),
            reviewed_selection_best=reviewed_best,reviewed_thresholds=reviewed_scores,
            thresholds=s['thresholds'],latency_ms=s['latency_ms'],
            summary_sha256=hashlib.sha256((directory/'summary.json').read_bytes()).hexdigest(),
            predictions_sha256=hashlib.sha256((directory/'predictions.jsonl').read_bytes()).hexdigest()))
    for path in ([] if a.quality_only else sorted(a.root.glob('live-*/**/report.json'))):
        r=json.loads(path.read_text())
        if 'job' not in r:continue
        report['live'].append(dict(path=str(path),job=r['job'],complete=r.get('observation_window_completed',False),
            passed=r['passed'],failures=r['failures'],measurements=r.get('measurements'),
            target_attainment=r.get('target_attainment'),resources=r.get('resources_startup_and_active'),planning=r.get('planning')))
    visual=json.loads((a.root/'spawn-visual-review-v1.json').read_text()) if (a.root/'spawn-visual-review-v1.json').exists() else dict(invalid=[])
    report['spawn_visual_review']=visual
    invalid={(r['capture'],r['asset']):r['reason'] for r in visual['invalid']}
    spawn_paths=sorted(p for directory in ('spawn-inspection-v1','spawn-scooter-native-v1')
                       for p in (a.root/directory).glob('*/predictions.jsonl'))
    for path in ([] if a.quality_only else spawn_paths):
        groups=defaultdict(list)
        for row in load_jsonl(path):groups[row['capture_id'],row['case']].append(row)
        for (capture,case),observations in groups.items():
            labels=Counter(n for r in observations for n in {d['label'] for d in r['predictions']})
            report['spawn_observations'].append(dict(candidate=path.parent.name,capture=capture,case=case,
                asset=observations[0]['asset'],placement=observations[0]['placement'],view=observations[0]['view'],
                visual_rejection_reason=invalid.get((capture,observations[0]['asset']['api_name'])),
                frames=len(observations),frames_with_label=dict(labels),
                interpretation='Label anywhere in selected view; NOT target association or recall. Review paired gallery.'))
    (a.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    page=['<!doctype html><meta charset="utf-8"><title>YOLOE 6조합 비교</title><style>body{font:16px system-ui;margin:24px}table{border-collapse:collapse}td,th{border:1px solid #ccc;padding:7px}code{overflow-wrap:anywhere}</style>',
        '<h1>YOLOE 6조합 · obstacle17 · 동일 영상 비교</h1>',
        '<p>현재 YOLO26l 기본값은 유지합니다. Capture 정량 GT에는 person/car/bus/truck만 있습니다. 신규 장애물은 동일 RGB 검토 대상이며 표의 F1에 포함되지 않습니다. 실패 실행과 planner deadline 실패도 보존합니다.</p>',
        '<p><a href="../spawn-inspection-v1/comparison.html">거리·날씨·모델을 고르는 스폰 비교 갤러리</a> · <a href="../spawn-scooter-native-v1/comparison.html">Scooter1 정상 지면 재수집</a> · <a href="../prompt-probe-v1/comparison.html">프롬프트 실제 재추론</a> · <a href="../parity-review-v1/index.html">PT/TRT 불일치 원본</a> · <a href="../spawn-source-review/index.html">검출 전 원본 장면</a> · <a href="report.json">전체 결과 JSON</a></p><p>기존 첫 수집의 Scooter1은 수치상 BBox 높이 gate를 통과했으나 영상에서 떠 있어 평가 부적합으로 보존합니다. 별도 native 지면 재수집 결과와 구분합니다. 나머지 배치도 pixel GT를 보장하지 않습니다.</p>',
        '<table><tr><th>모델</th><th>전방</th><th>F1 @0.40</th><th>TP / FP @0.40</th><th>선정 자료 best confidence / F1</th><th>정량 GT 원본</th></tr>']
    for row in report['quality']:
        fixed=row['fixed']['pooled'];best=row['selection_best']
        link=f'../quality-v1/{row["model"]}-{row["width"]}-selection/gallery/index.html' if row['model'].startswith('yoloe-') else '../../camera-coco7-live-v1/confidence-followup-actual40-v1/yolo26l-1280-selection/gallery/index.html'
        page.append(f'<tr><td>{row["model"]}</td><td>{row["width"]}</td><td>{fixed["macro_f1"]:.4f}</td><td>{fixed["total_tp"]} / {fixed["false_positives"]}</td><td>{best["confidence"]:.2f} / {best["macro_f1"]:.4f}</td><td><a href="{link}">영상</a></td></tr>')
    page+=['</table><p>Confidence sweep는 선정 자료에서만 수행했습니다. 실제 선정 cutoff 재실행·별도 heldout 검증은 원본 증거와 함께 확인합니다.</p>',
        '<h2>실시간 반복 전체</h2><table><tr><th>실행</th><th>전/좌/우 Hz</th><th>p95 최대 ms</th><th>전체 VRAM MiB</th><th>실패 이유</th></tr>']
    for row in report['live']:
        m=row['measurements'] or {};views=[m.get('yolo_'+v,{}) for v in ('front','left','right')]
        rates=' / '.join(f'{v.get("hz",0):.2f}' for v in views)
        p95=[v.get('input_to_output_wall_ms',{}).get('p95') for v in views];p95=[v for v in p95 if v is not None]
        memory=(row['resources'] or {}).get('gpu_memory_mib',{}).get('maximum')
        job=row['job'];label=f'{job["model"]}/{job["width"]}/{job.get("weather_requested","")}/r{job["repeat"]}'
        page.append(f'<tr><td>{html.escape(label)}</td><td>{rates}</td><td>{max(p95) if p95 else "unavailable"}</td><td>{memory}</td><td>{html.escape(str(row["failures"]))}</td></tr>')
    page+=['</table><p>추가학습 후 우열과 운영 수락을 사전학습 비교로 보장하지 않습니다. 관측 그룹은 평가용이며 원본 프롬프트 라벨을 유지합니다.</p>']
    (a.output/'index.html').write_text(''.join(page))
    print(json.dumps(dict(quality=len(report['quality']),live=len(report['live']),spawn_cases=len(report['spawn_observations']))))


if __name__=='__main__':main()
