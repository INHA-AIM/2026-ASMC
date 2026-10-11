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
from analyze_fusion_identity import compare_stable_observations

COMBINATIONS=[(f'yoloe-26{s}-seg',w) for s in ('m','l') for w in (640,960,1280)]
EVALUATION_GROUPS={
    'person':'person','dog':'animal','deer':'animal',
    **{n:'vehicle' for n in ('bicycle','car','truck','bus','motorcycle','scooter','electric scooter')},
    **{n:'obstacle' for n in ('bollard','traffic bollard','cargo box','wooden box','steel barricade','traffic cone','traffic barrel','baby stroller')}}


def best_threshold(thresholds):
    available=[r for r in thresholds if r.get('macro_f1') is not None]
    return max(available,key=lambda r:(r['macro_f1'],r['confidence'])) if available else None


def input_pauses(rows, begin, end):
    arrivals=[r for r in rows if begin<=r['arrival']<end]
    pairs=list(zip(arrivals,arrivals[1:]))
    return dict(scope='Consecutive input arrivals inside ACTIVE window; diagnostic only, no gate exemption',
        maximum_gap_ms=max([(b['arrival']-a['arrival'])*1000 for a,b in pairs],default=None),
        pauses=[dict(begin=a['arrival'],end=b['arrival'],wall_gap_ms=(b['arrival']-a['arrival'])*1000,
                     source_gap_ms=(b['stamp_ns']-a['stamp_ns'])/1e6)
                for a,b in pairs if b['arrival']-a['arrival']>.25])



def resolution_choice(quality, live):
    """All six full-stack trials must pass; never inherit another run's waiver."""
    eligible=[]
    for q in quality:
        if not q['model'].startswith('yoloe-'):continue
        rows=[r for r in live if r['candidate']['model']==q['model'] and r['candidate']['width']==q['width']]
        if len(rows)==2 and {r['weather'] for r in rows}=={'sunny','foggy'} and all(r['whole_stack_passed'] for r in rows):
            eligible.append(q)
    if not eligible:return dict(selected=None,reason='No YOLOE resolution passed all six integrated trials; hold selection and deployment.')
    def rank(q):
        observations=q['reviewed_fixed']['temporal_objects']
        stable=[o['stable_input_delay'] for o in observations if o.get('first_stable_frame') is not None]
        windows=[r for r in live if r['candidate']['model']==q['model'] and r['candidate']['width']==q['width']]
        return (-q['reviewed_fixed']['pooled']['macro_f1'],
                q['reviewed_fixed']['temporal_summary']['not_stably_detected'],
                sum(stable)/len(stable) if stable else float('inf'),
                max(r['wall_p95_ms_max'] for r in windows),max(r['peak_whole_vram_mib'] for r in windows))
    q=min(eligible,key=rank)
    return dict(selected=dict(model=q['model'],width=q['width'],confidence=.4,prompt='bollard17'),
                reason='Eligible full-stack candidate ranked by verified macro F1, stable acquisition, latency and memory; fusion acceptance remains separate.')


def lifecycle_stages(rows, begin, end, generation):
    from b0_measurements import quantiles
    received={};started={};durations=defaultdict(list)
    for row in rows:
        if row.get('generation')!=generation:continue
        now=row['monotonic']
        if row['event']=='input':received[row['view'],row['source_stamp_ns']]=now
        elif row['event'] in ('begin','complete'):
            for view,stamp in row['stamps'].items():
                key=view,stamp
                if row['event']=='begin':
                    started[key]=now
                    if begin<=now<end and key in received:
                        durations[view,'input_to_begin'].append((now-received[key])*1000)
                elif begin<=now<end and key in started:
                    durations[view,'begin_to_complete'].append((now-started[key])*1000)
    return {view:{stage:quantiles(durations[view,stage]) for stage in ('input_to_begin','begin_to_complete')}
            for view in ('front','left','right')}


def resolution_followup(root, reference, output):
    from analyze_coco7_broad_selection import confirmation_summary
    from b0_measurements import quantiles
    output.mkdir(parents=True,exist_ok=False)
    manifest=reference/'gt-v2/frames.jsonl'
    gt_rows=[r for r in load_jsonl(manifest) if r['split']=='selection' and r['annotation_complete']]
    exclusions=json.loads((reference/'broad-model-conditions-v1.json').read_text())['reviewed_exclusions']
    candidates=json.loads((root/'candidates.json').read_text())
    report=dict(scope='Same bollard17/checkpoint/confidence; 60-second C6 integrated trials. Verified GT has only person/car/bus/truck.',
        operational_defaults_changed=False,training=False,roboflow=False,quality=[],trials=[],live=[],fusion=[],fusion_class_comparison=[],
        percentile_definition='Per-run/view quantiles and explicitly pooled raw latency observations. Never average per-run medians.',
        source_time_warning='Receive-stamped ROS headers identify bridge publication times; do not prove simulator capture outages.',
        gt_manifest_sha256=hashlib.sha256(manifest.read_bytes()).hexdigest())
    for width in (960,1280):
        path=root/f'quality{width}-selection'/f'yoloe-26l-seg-{width}-selection'
        summary=json.loads((path/'summary.json').read_text())
        if summary['gt_manifest_sha256']!=report['gt_manifest_sha256']:raise ValueError('GT manifest mismatch')
        predictions=load_jsonl(path/'predictions.jsonl')
        gt,p=selected_frames(gt_rows,predictions,[])
        reviewed,rp=selected_frames(gt_rows,predictions,exclusions)
        report['quality'].append(dict(model='yoloe-26l-seg',width=width,frames=len(gt_rows),fixed=score(gt,p,.4),
            reviewed_fixed=score(reviewed,rp,.4),summary=summary,summary_sha256=hashlib.sha256((path/'summary.json').read_bytes()).hexdigest()))
    trials=[];pooled=defaultdict(list)
    for weather in ('sunny','foggy'):
        index=root/'live60'/weather/'index.json'
        if not index.exists():continue
        for entry in json.loads(index.read_text()):
            directory=index.parent/entry['name']
            r=json.loads((directory/'report.json').read_text());trials.append(r)
            row=dict(path=str(directory),job=r['job'],complete=r.get('observation_window_completed',False),
                passed=r.get('passed',False),failures=r['failures'],timestamp_policy=r.get('timestamp_policy'),
                measurements=r.get('measurements'),planning=r.get('planning'),target_attainment=r.get('target_attainment'),
                resources=r.get('resources_startup_and_active'),inputs={},
                sha256=hashlib.sha256((directory/'report.json').read_bytes()).hexdigest())
            requests=[v for v in r['requests'] if v['mode']==2 and v['model']=='yolo']
            if requests:
                req=requests[-1];begin=req['call_started'];end=begin+r['job']['seconds']
                if (directory/'inputs.json').exists():
                    row['inputs']={v:input_pauses(data,begin,end) for v,data in json.loads((directory/'inputs.json').read_text()).items()}
                status=json.loads((directory/'status.json').read_text())
                cutoffs={s['source_cutoff_ns'] for s in status if s['model']=='yolo' and s['generation']==req['generation']}
                if row['complete'] and len(cutoffs)==1:
                    cutoff=next(iter(cutoffs))
                    for view,outputs in json.loads((directory/'outputs.json').read_text()).items():
                        if not view.startswith('yolo_'):continue
                        values=[o['wall_latency_ms'] for o in outputs if begin<=o['arrival']<end and o['stamp_ns']>cutoff and o.get('wall_latency_ms') is not None]
                        pooled[r['job']['model'],r['job']['width'],weather,view].extend(values)
            report['trials'].append(row)
    report['live']=confirmation_summary(trials,candidates)
    report['pooled_latency']=[dict(model=m,width=w,weather=weather,view=v,quantiles_ms=quantiles(values))
        for (m,w,weather,v),values in sorted(pooled.items())]
    report['decision']=resolution_choice(report['quality'],report['live'])
    report['diagnostics']=[]
    index=root/'diagnostic-c1'/'index.json'
    if index.exists():
        for entry in json.loads(index.read_text()):
            directory=index.parent/entry['name'];r=json.loads((directory/'report.json').read_text())
            row=dict(job=r['job'],passed=r['passed'],failures=r['failures'],measurements=r.get('measurements'),
                timestamp_policy=r.get('timestamp_policy'),scope='C1 trace ON; not a C6 acceptance replacement')
            req=next((v for v in r['requests'] if v['mode']==2 and v['model']=='yolo'),None)
            if req:
                begin=req['call_started'];end=begin+r['job']['seconds']
                raw=json.loads((directory/'inputs.json').read_text())
                row['input_pauses']={v:input_pauses(rows,begin,end) for v,rows in raw.items()}
                trace=directory/'yolo.lifecycle.jsonl'
                if trace.exists():row['worker_stages']=lifecycle_stages(load_jsonl(trace),begin,end,req['generation'])
                row['unanchored_outputs']=[]
                for view,rows in json.loads((directory/'outputs.json').read_text()).items():
                    if not view.startswith('yolo_'):continue
                    for o in rows:
                        if not begin<=o['arrival']<end or o['input_anchor']!='missing':continue
                        anchors=[i for i in raw.get(view[5:],[]) if i['stamp_ns']==o['stamp_ns']]
                        row['unanchored_outputs'].append(dict(view=view,stamp_ns=o['stamp_ns'],
                            later_input_callback_ms=[(i['arrival']-o['arrival'])*1000 for i in anchors],
                            interpretation='Observer missing/delayed input callback; preserve invalid wall anchor and original failed gate.'))
            report['diagnostics'].append(row)
    sensor=root/'sensor-only'/'report.json'
    if sensor.exists():report['sensor_only']=json.loads(sensor.read_text())
    for family in ('fusion-reference','fusion-candidate'):
        index=root/family/'index.json'
        if not index.exists():continue
        for entry in json.loads(index.read_text()):
            path=index.parent/entry['path']/'report.json'
            if not path.exists():
                report['fusion'].append(dict(family=family,run=entry['path'],complete=False,entry=entry));continue
            r=json.loads(path.read_text())
            report['fusion'].append(dict(family=family,run=entry['path'],complete=True,passed=r['passed'],
                failed_checks=[k for k,v in r['checks'].items() if not v],fusion=r['fusion'],measurements=r['measurements']))
    for bag in ('primary','shadow'):
        sources=[[root/family/f'{bag}-unique-r{i}'/'observations.json' for i in (1,2)]
                 for family in ('fusion-reference','fusion-candidate')]
        if all(p.exists() for group in sources for p in group):
            tracks=[[json.loads(p.read_text())['tracks'] for p in group] for group in sources]
            report['fusion_class_comparison'].append(dict(bag=bag,comparison=compare_stable_observations(*tracks),
                scope='Same stamp and mutually unique centers <=1mm; stable baseline is not GT; no inherited exceptions.'))
    (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    page=['<!doctype html><meta charset="utf-8"><title>YOLOE L 960/1280 최종 통합 비교</title>',
        '<style>body{font:16px system-ui;margin:24px}table{border-collapse:collapse}td,th{border:1px solid #bbb;padding:7px}pre{white-space:pre-wrap}</style>',
        '<h1>YOLOE L · 동일 bollard17 · 960/1280 · 60초 통합 반복</h1>',
        '<p>운영 기본 YOLO26l은 유지합니다. 검증 GT는 person/car/bus/truck만 있으며 신규 장애물은 관찰용입니다. p50은 실행별 평균의 중앙값으로 계산하지 않습니다.</p>',
        '<p><a href="../spawn-comparison/comparison.html">동일 스폰 RGB 비교</a> · <a href="report.json">전체 결과</a> · <a href="../../../index.html">갤러리 목차</a></p>',
        '<h2>동일 Capture 품질</h2><table><tr><th>전방</th><th>원본 F1</th><th>승인2장 제외 F1</th><th>TP / FP</th><th>mask IoU</th></tr>']
    for q in report['quality']:
        fixed=q['reviewed_fixed']['pooled']
        page.append(f'<tr><td>{q["width"]}</td><td>{q["fixed"]["pooled"]["macro_f1"]:.4f}</td><td>{fixed["macro_f1"]:.4f}</td><td>{fixed["total_tp"]} / {fixed["false_positives"]}</td><td>{q["summary"]["thresholds"][0]["mask_iou"]}</td></tr>')
    page+=['</table><h2>실행별 · 뷰별 지연</h2><table><tr><th>구성 / 날씨 / 반복</th><th>뷰</th><th>Hz</th><th>p50 / p95 / p99 ms</th><th>최대 공백 ms</th><th>실패</th></tr>']
    for r in report['trials']:
        job=r['job'];label=f'{job["model"]}/{job["width"]}/{job["weather_requested"]}/r{job["repeat"]}'
        for view in ('front','left','right'):
            m=(r['measurements'] or {}).get('yolo_'+view)
            if m:
                q=m['input_to_output_wall_ms'];latency=' / '.join(f'{q[k]:.1f}' if q[k] is not None else 'NA' for k in ('p50','p95','p99'))
                page.append(f'<tr><td>{label}</td><td>{view}</td><td>{m["hz"]:.2f}</td><td>{latency}</td><td>{m["max_gap_sec"]*1000:.1f}</td><td>{html.escape(str(r["failures"]))}</td></tr>')
            else:page.append(f'<tr><td>{label}</td><td>{view}</td><td colspan="4">Incomplete: {html.escape(str(r["failures"]))}</td></tr>')
    page+=['</table><h2>후보 결정 · 운영 승격 별도</h2><pre>'+html.escape(json.dumps(report['decision'],ensure_ascii=False,indent=2))+'</pre>',
           '<h2>센서 대조 · YOLO 단독 진단</h2><p>별도 관측이며 통합 반복의 실패를 면제하지 않습니다. worker begin→complete에는 전처리/후처리가 포함되고 GPU 연산만의 시간이 아닙니다.</p><pre>'+html.escape(json.dumps(report.get('sensor_only'),ensure_ascii=False,indent=2))+'</pre>',
           '<h2>Fusion 원본 판정</h2><pre>'+html.escape(json.dumps(report['fusion_class_comparison'],ensure_ascii=False,indent=2))+'</pre>']
    (output/'index.html').write_text(''.join(page))
    print(json.dumps(dict(quality=len(report['quality']),trials=len(trials),decision=report['decision'])))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root',type=Path,required=True)
    p.add_argument('--reference',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--resolution-followup',action='store_true',help='Analyze matched bollard17 960/1280 and 60-second trials')
    p.add_argument('--quality-only',action='store_true',help='Freeze quality screening before timed trials; do not read partial live/spawn files')
    a=p.parse_args()
    if a.resolution_followup:
        resolution_followup(a.root,a.reference,a.output)
        return
    a.output.mkdir(parents=True,exist_ok=False)
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
        active=[x['call_started'] for x in r['requests'] if x['mode']==2]
        inputs={}
        if active and (path.parent/'inputs.json').exists():
            begin=min(active);end=begin+r['job']['seconds']
            inputs={v:input_pauses(rows,begin,end) for v,rows in json.loads((path.parent/'inputs.json').read_text()).items()}
        report['live'].append(dict(path=str(path),job=r['job'],complete=r.get('observation_window_completed',False),
            passed=bool(r.get('passed',False)),
            failures=r['failures']+([] if r.get('observation_window_completed',False) else ['incomplete observation window; not accepted']),measurements=r.get('measurements'),
            target_attainment=r.get('target_attainment'),resources=r.get('resources_startup_and_active'),planning=r.get('planning'),input_arrival_diagnostics=inputs))
    report['selected_actual_cutoff']=[]
    for name in ('selected-quality40-v1','selected-heldout40-v1'):
        for path in (a.root/name).glob('*/summary.json'):
            s=json.loads(path.read_text())
            if s['gt_manifest_sha256']!=digest:raise ValueError('selected result uses different GT')
            report['selected_actual_cutoff'].append(dict(source=str(path),summary=s))
    report['fusion']=[]
    report['fusion_class_comparison']=[]
    for family in ('fusion-reference-v1','fusion-v1'):
        index=a.root/family/'index.json'
        if not a.quality_only and index.exists():
            for entry in json.loads(index.read_text()):
                f=a.root/family/entry['path']/'report.json'
                r=json.loads(f.read_text())
                report['fusion'].append(dict(family=family,run=entry['path'],passed=r['passed'],
                    failed_checks=[k for k,v in r['checks'].items() if not v],
                    fusion=r['fusion'],measurements=r['measurements'],
                    source_sha256=hashlib.sha256(f.read_bytes()).hexdigest()))
    for bag in ('primary','shadow'):
        sources=[[a.root/family/f'{bag}-unique-r{i}'/'observations.json' for i in (1,2)]
                 for family in ('fusion-reference-v1','fusion-v1')]
        if not a.quality_only and all(p.exists() for group in sources for p in group):
            tracks=[[json.loads(p.read_text())['tracks'] for p in group] for group in sources]
            report['fusion_class_comparison'].append(dict(bag=bag,
                scope='Same LiDAR stamp and mutually unique 3D centers <=1mm; stable reference labels are not independent GT. No inherited acceptance exception.',
                sources={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for group in sources for p in group},
                comparison=compare_stable_observations(*tracks)))
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
        '<table><tr><th>모델</th><th>전방</th><th>원본2252장 F1 @0.40</th><th>기존 승인2장 제외2250장 F1 @0.40</th><th>TP / FP (2250장)</th><th>2250장 best confidence / F1</th><th>정량 GT 원본</th></tr>']
    for row in report['quality']:
        original=row['fixed']['pooled'];fixed=row['reviewed_fixed']['pooled'];best=row['reviewed_selection_best']
        link=f'../quality-v1/{row["model"]}-{row["width"]}-selection/gallery/index.html' if row['model'].startswith('yoloe-') else '../../camera-coco7-live-v1/confidence-followup-actual40-v1/yolo26l-1280-selection/gallery/index.html'
        page.append(f'<tr><td>{row["model"]}</td><td>{row["width"]}</td><td>{original["macro_f1"]:.4f}</td><td>{fixed["macro_f1"]:.4f}</td><td>{fixed["total_tp"]} / {fixed["false_positives"]}</td><td>{best["confidence"]:.2f} / {best["pooled"]["macro_f1"]:.4f}</td><td><a href="{link}">영상</a></td></tr>')
    page+=['</table><p>Confidence sweep는 선정 자료에서만 수행했습니다. 실제 선정 cutoff 재실행·별도 heldout 검증은 원본 증거와 함께 확인합니다.</p>',
        '<h2>실시간 반복 전체</h2><table><tr><th>실행</th><th>전/좌/우 Hz</th><th>p95 최대 ms</th><th>전체 VRAM MiB</th><th>실패 이유</th></tr>']
    for row in report['live']:
        m=row['measurements'] or {};views=[m.get('yolo_'+v,{}) for v in ('front','left','right')]
        rates=' / '.join(f'{v.get("hz",0):.2f}' for v in views)
        p95=[v.get('input_to_output_wall_ms',{}).get('p95') for v in views];p95=[v for v in p95 if v is not None]
        memory=(row['resources'] or {}).get('gpu_memory_mib',{}).get('maximum')
        job=row['job'];label=f'{job["model"]}/{job["width"]}/{job.get("weather_requested","")}/r{job["repeat"]}'
        page.append(f'<tr><td>{html.escape(label)}</td><td>{rates}</td><td>{max(p95) if p95 else "unavailable"}</td><td>{memory}</td><td>{html.escape(str(row["failures"]))}</td></tr>')
    page+=['</table><h2>최종 후보 · l1280 / bollard17 / confidence 0.40</h2>',
        '<p>운영 기본 YOLO26l은 유지합니다. 최종 반복의 실패를 면제하지 않습니다. 이름만 바꾼 실제 재추론 후 선정한 프롬프트이며 wooden box 미검출과 deer 라벨 혼동은 남아 있습니다.</p>',
        '<p><a href="../selected-spawn-v1/comparison.html">선정 프롬프트와 YOLO26 동일 RGB 대조</a> · <a href="../prompt-bollard-v1/comparison.html">bollard / traffic bollard 대조</a> · <a href="../confidence-review-v1/index.html">0.20 / 0.40 오검출과 회복 목록</a></p>']
    for row in report['selected_actual_cutoff']:
        s=row['summary'];t=s['thresholds'][0]
        page.append(f'<p>Actual cutoff {s["frozen_confidence"]} · {s["split"]} {s["frames"]} frames · macro F1 {t["macro_f1"]:.4f} · FP {t["false_positives"]}</p>')
    page+=['<h2>전방 bbox fusion · 현재 YOLO26l과 새로 재생한 대조</h2>',
        '<p>unique / ±50ms / 200ms를 유지했습니다. 매칭 쌍 수는 객체 recall이 아닙니다. 종료 pending과 실패 검사를 그대로 표시합니다. 3뷰 mask fusion이나 분류 정확도 검증으로 확대하지 않습니다.</p>',
        '<table><tr><th>경로</th><th>실행</th><th>matched / dropped</th><th>종료 pending LiDAR / YOLO</th><th>offset p95 최대 ms</th><th>실패 검사</th></tr>']
    for r in report['fusion']:
        f=r['fusion'];t=f['totals']
        page.append(f'<tr><td>{r["family"]}</td><td>{r["run"]}</td><td>{t["matched"]} / {t["dropped"]}</td><td>{t["pending_lidar"]} / {t["pending_yolo"]}</td><td>{f["offset_p95_max_ms"]}</td><td>{html.escape(str(r["failed_checks"]))}</td></tr>')
    page+=['</table>']
    for r in report['fusion_class_comparison']:
        c=r['comparison']
        page.append(f'<p>{r["bag"]}: reference-stable matched observations {c["old_stable"]}, label differences {len(c["regressions"])}, unmatched common observations {c["unmatched_common_observations"]}. 안정된 기준선도 GT가 아니며 차이 원본은 report.json에 보존합니다.</p>')
    page+=['<p>추가학습 후 우열과 운영 수락을 사전학습 비교로 보장하지 않습니다. 관측 그룹은 평가용이며 원본 프롬프트 라벨을 유지합니다.</p>']
    (a.output/'index.html').write_text(''.join(page))
    print(json.dumps(dict(quality=len(report['quality']),live=len(report['live']),spawn_cases=len(report['spawn_observations']))))


if __name__=='__main__':main()
